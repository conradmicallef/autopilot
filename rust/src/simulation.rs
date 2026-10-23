//! The world: aircraft, wind, airfield, autopilot and the flight record, stepped at a fixed 50 Hz.

use crate::aircraft::{c172_like, Aircraft, Contact, ModelKind};
use crate::airfield::{
    normal_procedure, short_field_procedure, standard_circuit, Circuit, CircuitSpec, Runway, OBSTACLE_HEIGHT, OBSTACLE_OFFSET,
};
use crate::atmosphere::{Rng, Wind, WindSettings};
use crate::autopilot::{Autopilot, EngineOutPlan, Phase};
use crate::tuning::{Perturb, Tuning};
use crate::units::{f1, f2, r0, D2R, FT, KT};
use serde::Serialize;

#[derive(Clone, Debug, Serialize)]
pub struct TrackPoint {
    pub t: f64,
    pub x: f64,
    pub y: f64,
    pub h: f64,
    #[serde(rename = "V")]
    pub v: f64,
    pub phase: &'static str,
    pub reach: Option<bool>,
}

#[derive(Clone, Debug, Serialize)]
pub struct Touchdown {
    pub t: f64,
    pub x: f64,
    pub y: f64,
    pub sink: f64,
    #[serde(rename = "V")]
    pub v: f64,
    pub crab: f64,
    #[serde(rename = "onRunway")]
    pub on_runway: bool,
    /// Pitch attitude at touchdown, rad.
    pub pitch: f64,
    /// Flap at touchdown, deg.
    pub flap: f64,
    /// Airspeed over the stall speed for that flap.
    #[serde(rename = "vsRatio")]
    pub vs_ratio: f64,
    /// Steady crosswind component across the landing direction, m/s.
    pub xwind: f64,
    /// Stall speed for the flap set, m/s.
    pub vs: f64,
}

#[derive(Clone, Debug, Serialize)]
pub struct Criterion {
    pub label: String,
    pub target: String,
    pub value: Option<String>,
    pub pass: Option<bool>,
    pub info: bool,
}

fn crit(label: &str, target: &str) -> Criterion {
    Criterion { label: label.into(), target: target.into(), value: None, pass: None, info: false }
}

pub struct SimOptions {
    pub wind: WindSettings,
    pub seed: f64,
    pub circuit: CircuitSpec,
    pub model: ModelKind,
    /// Runway / strip length, m. Shorter than 1,200 m gets 50 ft trees beyond each end.
    pub runway_length: f64,
    pub short_field: bool,
    /// Gains and landing parameters; `None` takes the model's defaults (`Tuning::for_model`).
    pub tuning: Option<Tuning>,
    /// Model error, for robustness tests (default: none).
    pub perturb: Perturb,
}

/// Take-off and landing distances, measured as they happen.
#[derive(Clone, Debug, Default, Serialize)]
pub struct Performance {
    /// Along-runway position at lift-off (brake release at 0).
    pub liftoff_x: Option<f64>,
    /// Position when first through 50 ft after take-off.
    pub takeoff_50ft_x: Option<f64>,
    /// Height passing the departure-end trees.
    pub departure_obstacle_h: Option<f64>,
    /// Position when last descending through 50 ft on the approach.
    pub landing_50ft_x: Option<f64>,
    /// Height passing the approach-end trees.
    pub approach_obstacle_h: Option<f64>,
}

impl Default for SimOptions {
    fn default() -> Self {
        SimOptions {
            wind: WindSettings { from_deg: 270.0, speed_kt: 0.0, gusts: false },
            seed: 1.0,
            circuit: standard_circuit(),
            model: ModelKind::PointMass,
            runway_length: 1200.0,
            short_field: false,
            tuning: None,
            perturb: Perturb::default(),
        }
    }
}

pub struct Simulation {
    pub t: f64,
    pub aircraft: Aircraft,
    pub wind: Wind,
    pub autopilot: Autopilot,
    pub track: Vec<TrackPoint>,
    pub touchdown: Option<Touchdown>,
    pub max_bank: f64,
    pub max_alt: f64,
    next_record: f64,
    /// Is the runway within power-off gliding reach right now? None on the ground or once the engine failed.
    pub runway_reachable: Option<bool>,
    /// Which runway direction the glide planner would use right now.
    pub reach_end: Option<&'static str>,
    /// Seconds airborne with the engine running, and how many of those had the runway within glide reach.
    pub air_time: f64,
    pub reach_time: f64,
    next_reach_check: f64,
    pub perf: Performance,
    /// At an engine failure: could an ideal plan have reached the runway (`ceiling_reachable`)?
    pub eo_ceiling: Option<bool>,
    /// 50 ft trees beyond each end of the strip.
    pub obstacles: bool,
    prev_x: f64,
    prev_h: f64,
}

/// Touchdown speed limit, as a multiple of the stall speed for the flap set (about 1.05–1.1 is the
/// target; 1.15 allows a gust margin).
pub const TD_SPEED_MAX: f64 = 1.15;
/// Lowest pitch at touchdown, deg: main wheels first, nose wheel held off.
pub const TD_PITCH_MIN: f64 = 5.0;

fn touchdown_rows(td: &Touchdown) -> (Criterion, Criterion) {
    // in a crosswind the technique touches down faster by half the crosswind component
    let limit = TD_SPEED_MAX * td.vs + crate::autopilot::TD_XWIND_K * td.xwind;
    let mut sp = crit("Touchdown speed", &format!("≤ {TD_SPEED_MAX} × stall speed + ¾ crosswind"));
    sp.value = Some(format!("{} kt, {} × Vs (flap {}; limit {} kt)", r0(td.v / KT), f2(td.vs_ratio), r0(td.flap), r0(limit / KT)));
    sp.pass = Some(td.v <= limit);
    let mut pt = crit("Nose-up at touchdown", &format!("≥ {TD_PITCH_MIN}° (main wheels first)"));
    pt.value = Some(format!("{}°", f1(td.pitch / D2R)));
    pt.pass = Some(td.pitch / D2R >= TD_PITCH_MIN);
    (sp, pt)
}

impl Simulation {
    pub const DT: f64 = 0.02;

    pub fn new(opts: SimOptions) -> Self {
        let len = opts.runway_length;
        let proc_ = if opts.short_field { short_field_procedure(&opts.circuit, len) } else { normal_procedure(&opts.circuit, len) };
        let mut spec = opts.circuit;
        spec.approach_angle = proc_.approach_angle;
        spec.vapp = proc_.vapp;
        let runway = Runway::new(len, if len < 1199.0 { 20.0 } else { 30.0 }, proc_.aim_point, proc_.approach_angle);
        let circuit = Circuit::new(spec, runway);
        let tu = opts.tuning.clone().unwrap_or_else(|| Tuning::for_model(opts.model));
        let pt = opts.perturb;
        let mut airframe = c172_like();
        airframe.mass *= pt.mass;
        let mut aircraft = Aircraft::new(airframe, opts.model);
        if let Some(six) = aircraft.six.as_mut() {
            six.fcs.gains = crate::sixdof::fcs::FcsGains::from_tuning(&tu);
            six.prm.control_power = pt.control_power;
            six.prm.pitch_stability = pt.pitch_stability;
            six.prm.actuator_tau *= pt.actuator_lag;
        }
        let autopilot = Autopilot::with_procedure(circuit, proc_, tu);
        Simulation {
            t: 0.0,
            aircraft,
            wind: Wind::new(opts.wind, Rng::new(opts.seed)),
            autopilot,
            track: Vec::new(),
            touchdown: None,
            max_bank: 0.0,
            max_alt: 0.0,
            next_record: 0.0,
            runway_reachable: None,
            reach_end: None,
            air_time: 0.0,
            reach_time: 0.0,
            next_reach_check: 0.0,
            perf: Performance::default(),
            eo_ceiling: None,
            obstacles: len < 1199.0,
            prev_x: 0.0,
            prev_h: 0.0,
        }
    }

    pub fn phase(&self) -> Phase {
        self.autopilot.phase
    }
    pub fn finished(&self) -> bool {
        self.phase() == Phase::Stopped
    }
    pub fn runway(&self) -> &Runway {
        &self.autopilot.circuit.runway
    }

    pub fn request_go_around(&mut self) -> bool {
        let ok = matches!(self.phase(), Phase::Base | Phase::Final | Phase::Flare) && self.autopilot.engine_out.is_none();
        if ok {
            self.autopilot.go_around_requested = true;
        }
        ok
    }

    /// Could the runway be reached at all from here? An ideal plan: the glide planner with no margins,
    /// the true mean wind, any aim point down to 60 m in. An estimate of the physical ceiling (the
    /// planner's model, not the flight), recorded at the failure so results can be read against it.
    pub fn ceiling_reachable(&self) -> bool {
        let ac = &self.aircraft;
        if ac.on_ground || self.phase() == Phase::Flare {
            return true;
        }
        let mut spec = self.autopilot.planner.spec.clone();
        spec.margin_factor = 1.0;
        spec.margin_height = 0.0;
        let p = crate::emergency::GlidePlanner::new(self.runway().clone(), spec);
        let st = self.wind.settings;
        let to = (90.0 - (st.from_deg + 180.0)) * D2R;
        let w = crate::units::Vec2 { x: st.speed_kt * KT * to.cos(), y: st.speed_kt * KT * to.sin() };
        p.plan(ac, w, None).reachable
    }

    /// Stop the engine now; the autopilot decides how to handle it.
    pub fn fail_engine(&mut self) -> bool {
        if self.autopilot.engine_out.is_none() && self.phase() != Phase::Stopped {
            self.eo_ceiling = Some(self.ceiling_reachable());
        }
        let ok = self.autopilot.fail_engine(&self.aircraft, &self.wind);
        if ok {
            self.aircraft.engine_failed = true;
        }
        ok
    }

    pub fn step(&mut self) {
        let dt = Self::DT;
        if self.finished() {
            self.aircraft.vg = 0.0;
            self.aircraft.v = 0.0;
            self.t += dt;
            return;
        }
        self.wind.update(dt);
        let cmd = self.autopilot.update(&self.aircraft, &self.wind, self.t, dt);
        let contact = self.aircraft.step(dt, &cmd, &self.wind);
        if let Some(Contact::Liftoff { .. }) = contact {
            if self.perf.liftoff_x.is_none() {
                self.perf.liftoff_x = Some(self.aircraft.x);
            }
        }
        self.measure();
        if let Some(Contact::Touchdown { sink, crab, v, .. }) = contact {
            let ac = &self.aircraft;
            self.touchdown = Some(Touchdown {
                t: self.t,
                x: ac.x,
                y: ac.y,
                sink,
                v,
                crab,
                on_runway: self.autopilot.circuit.runway.contains(ac.x, ac.y),
                pitch: ac.theta,
                flap: ac.flap_pos,
                vs_ratio: v / ac.stall_speed(ac.flap_pos),
                xwind: {
                    let s = self.wind.settings;
                    let to = (90.0 - (s.from_deg + 180.0)) * D2R;
                    let w = crate::units::Vec2 { x: s.speed_kt * KT * to.cos(), y: s.speed_kt * KT * to.sin() };
                    crate::emergency::across(w, ac.runway_heading).abs()
                },
                vs: ac.stall_speed(ac.flap_pos),
            });
            self.autopilot.on_unexpected_touchdown(&self.aircraft);
        }
        if !self.aircraft.on_ground {
            self.max_bank = self.max_bank.max(self.aircraft.phi.abs());
            self.max_alt = self.max_alt.max(self.aircraft.h);
        }
        self.update_reach(dt);
        self.t += dt;
        if self.t >= self.next_record {
            let ac = &self.aircraft;
            self.track.push(TrackPoint {
                t: self.t,
                x: ac.x,
                y: ac.y,
                h: ac.h,
                v: ac.v,
                phase: self.phase().code(),
                reach: self.runway_reachable,
            });
            self.next_record = self.t + 0.25;
        }
    }

    /// Take-off and landing distance bookkeeping (runway 09 operations; power-off returns are scored separately).
    fn measure(&mut self) {
        let (x, h, p) = (self.aircraft.x, self.aircraft.h, self.phase());
        let len = self.runway().length;
        let fifty = 50.0 * FT;
        let pf = &mut self.perf;
        if matches!(p, Phase::Rotate | Phase::Climb) && !self.aircraft.on_ground && pf.takeoff_50ft_x.is_none() && h >= fifty {
            pf.takeoff_50ft_x = Some(x);
        }
        let dep = len + OBSTACLE_OFFSET;
        if matches!(p, Phase::Climb | Phase::Crosswind | Phase::Rotate)
            && pf.departure_obstacle_h.is_none()
            && self.prev_x < dep
            && x >= dep
        {
            pf.departure_obstacle_h = Some(h);
        }
        if p == Phase::GoAround {
            pf.landing_50ft_x = None;
            pf.approach_obstacle_h = None;
        }
        if matches!(p, Phase::Final | Phase::Flare) && self.prev_h >= fifty && h < fifty {
            pf.landing_50ft_x = Some(x);
        }
        let app = -OBSTACLE_OFFSET;
        if matches!(p, Phase::Base | Phase::Final | Phase::Flare) && self.prev_x < app && x >= app && self.aircraft.y.abs() < 300.0 {
            pf.approach_obstacle_h = Some(h);
        }
        self.prev_x = x;
        self.prev_h = h;
    }

    /// Run until stopped or until `t_max` seconds.
    pub fn run(&mut self, t_max: f64) {
        while !self.finished() && self.t < t_max {
            self.step();
        }
    }

    /// Glide-reach bookkeeping: twice a second, would a power-off return reach the runway from here?
    /// Climbing below the turn-back height counts as no (the policy is to land ahead there).
    fn update_reach(&mut self, dt: f64) {
        let ac = &self.aircraft;
        let ap = &self.autopilot;
        let tracking = !ac.on_ground && !ac.engine_failed && self.phase() != Phase::Flare;
        if !tracking {
            self.runway_reachable = None;
            return;
        }
        if self.t >= self.next_reach_check {
            self.next_reach_check = self.t + 0.5;
            let climbing_low = matches!(self.phase(), Phase::Climb | Phase::GoAround)
                && ac.h < ap.planner.spec.min_turnback_height;
            // what the autopilot would plan, on its own wind estimate
            let g = ap.planner.plan(ac, ap.gust.mean_wind(), None);
            let r = !climbing_low && g.reachable;
            self.runway_reachable = Some(r);
            self.reach_end = if r { Some(g.end.name) } else { None };
        }
        self.air_time += dt;
        if self.runway_reachable == Some(true) {
            self.reach_time += dt;
        }
    }

    pub fn reach_fraction(&self) -> f64 {
        if self.air_time > 0.0 {
            self.reach_time / self.air_time
        } else {
            0.0
        }
    }

    /// Landing pass/fail check against the proof-of-concept criteria.
    pub fn evaluate(&self) -> Vec<Criterion> {
        if self.autopilot.engine_out.is_some() {
            return self.evaluate_engine_out();
        }
        let rwy = self.runway();
        let third = rwy.length / 3.0;
        let stopped = self.finished();
        let short = self.autopilot.procedure.short_field;
        let aim = rwy.aim_point;
        let td_target =
            if short { format!("within 100 m of the {} m aim point", r0(aim)) } else { format!("first third (0–{} m)", r0(third)) };
        let mut rows = vec![
            crit("Touchdown sink rate", "< 3 ft/s"),
            crit("Touchdown point", &td_target),
            crit("Centerline error", "< 5 m"),
            crit("Nose aligned at touchdown", "< 5° crab"),
            crit("Touchdown speed", &format!("≤ {TD_SPEED_MAX} × stall speed + ¾ crosswind")),
            crit("Nose-up at touchdown", &format!("≥ {TD_PITCH_MIN}° (main wheels first)")),
            crit("Stopped on runway", &format!("before {} m", r0(rwy.length))),
        ];
        let td = match &self.touchdown {
            Some(td) => td,
            None => return rows,
        };
        let sink_fps = td.sink / FT;
        rows[0].value = Some(format!("{} ft/s", f1(sink_fps)));
        rows[0].pass = Some(sink_fps < 3.0);
        rows[1].value = Some(format!("{} m past threshold", r0(td.x)));
        rows[1].pass = Some(td.x >= 0.0 && if short { td.x <= aim + 100.0 } else { td.x <= third });
        rows[2].value = Some(format!("{} m", f1(td.y.abs())));
        rows[2].pass = Some(td.y.abs() < 5.0);
        let crab_deg = td.crab.abs() / D2R;
        rows[3].value = Some(format!("{}° crab", f1(crab_deg)));
        rows[3].pass = Some(crab_deg < 5.0);
        let (sp, pt) = touchdown_rows(td);
        rows[4] = sp;
        rows[5] = pt;
        rows[6].value =
            Some(if stopped { format!("{} m mark", r0(self.aircraft.x)) } else { "rolling…".into() });
        rows[6].pass = if stopped { Some(rwy.contains(self.aircraft.x, self.aircraft.y)) } else { None };
        rows.extend(self.distance_rows(stopped));
        rows.push(Criterion {
            label: "Runway within glide reach".into(),
            target: format!("{} circuit, share of time airborne", self.autopilot.circuit.spec.name),
            value: Some(format!("{}%", r0(self.reach_fraction() * 100.0))),
            pass: None,
            info: true,
        });
        rows
    }

    /// Take-off and landing distances, plus obstacle clearance on a short strip.
    fn distance_rows(&self, stopped: bool) -> Vec<Criterion> {
        let p = &self.perf;
        let m = |v: f64| format!("{} m", r0(v));
        let mut rows = Vec::new();
        let mut to = crit("Take-off distance", "ground roll · to 50 ft");
        to.info = true;
        if let Some(lo) = p.liftoff_x {
            to.value = Some(match p.takeoff_50ft_x {
                Some(x50) => format!("{} · {}", m(lo), m(x50)),
                None => format!("{} · climbing", m(lo)),
            });
        }
        rows.push(to);
        if self.obstacles {
            let mut c = crit("Cleared departure trees", "50 ft, 150 m past the end");
            if let Some(h) = p.departure_obstacle_h {
                let clr = (h - OBSTACLE_HEIGHT) / FT;
                c.value = Some(format!("{} ft {}", r0(clr.abs()), if clr >= 0.0 { "clear" } else { "below the tops" }));
                c.pass = Some(clr >= 0.0);
            }
            rows.push(c);
        }
        let mut ld = crit("Landing distance", "from 50 ft · ground roll");
        ld.info = true;
        if let (Some(td), true) = (&self.touchdown, stopped) {
            let stop = self.aircraft.x;
            ld.value = Some(match p.landing_50ft_x {
                Some(x50) => format!("{} · {}", m(stop - x50), m(stop - td.x)),
                None => format!("— · {}", m(stop - td.x)),
            });
        }
        rows.push(ld);
        if self.obstacles {
            let mut c = crit("Cleared approach trees", "50 ft, 150 m before the threshold");
            if let Some(h) = p.approach_obstacle_h {
                let clr = (h - OBSTACLE_HEIGHT) / FT;
                c.value = Some(format!("{} ft {}", r0(clr.abs()), if clr >= 0.0 { "clear" } else { "below the tops" }));
                c.pass = Some(clr >= 0.0);
            }
            rows.push(c);
        }
        rows
    }

    /// Engine-out outcome: was the decision carried out, and was the touchdown survivable and controlled?
    fn evaluate_engine_out(&self) -> Vec<Criterion> {
        let eo = self.autopilot.engine_out.as_ref().unwrap();
        let ac = &self.aircraft;
        let stopped = self.finished();
        let rwy = self.runway();
        let end = self.autopilot.eo_end();
        let decided = match eo.plan {
            EngineOutPlan::Reject => "Reject take-off".to_string(),
            EngineOutPlan::Return => format!("Glide back to runway {}", end.name),
            EngineOutPlan::LandAhead => "Land ahead".to_string(),
        };
        let mut rows = vec![Criterion {
            label: "Decision".into(),
            target: eo.why.clone(),
            value: Some(format!("{} · failed at {} ft", decided, r0(eo.h / FT))),
            pass: None,
            info: true,
        }];
        if eo.plan == EngineOutPlan::Reject {
            let mut c = crit("Stopped on runway", &format!("before {} m", r0(rwy.length)));
            c.value = Some(if stopped { format!("{} m mark", r0(ac.x)) } else { "braking…".into() });
            c.pass = if stopped { Some(ac.x <= rwy.length) } else { None };
            rows.push(c);
            return rows;
        }
        let td = self.touchdown.as_ref();
        let mut c = crit("Touchdown sink rate", "< 3 ft/s");
        if let Some(td) = td {
            c.value = Some(format!("{} ft/s", f1(td.sink / FT)));
            c.pass = Some(td.sink / FT < 3.0);
        }
        rows.push(c);
        if eo.plan == EngineOutPlan::Return {
            let mut c = crit("Touched down on runway", "within the paved surface");
            if let Some(td) = td {
                let past = end.dir * (td.x - end.threshold);
                c.value = Some(if td.on_runway {
                    format!("{} m past the {} threshold", r0(past), end.name)
                } else if past < 0.0 {
                    format!("{} m short", r0(-past))
                } else {
                    "off the runway".into()
                });
                c.pass = Some(td.on_runway);
            }
            rows.push(c);
            // landed like any landing: on the centreline, nose straight, slow and nose-up
            let mut cl = crit("Centerline error", "< 5 m");
            let mut cr = crit("Nose aligned at touchdown", "< 5° crab");
            if let Some(td) = td {
                cl.value = Some(format!("{} m", f1(td.y.abs())));
                cl.pass = Some(td.y.abs() < 5.0);
                let crab_deg = td.crab.abs() / D2R;
                cr.value = Some(format!("{}° crab", f1(crab_deg)));
                cr.pass = Some(crab_deg < 5.0);
                let (sp, pt) = touchdown_rows(td);
                rows.extend([cl, cr, sp, pt]);
            } else {
                rows.extend([cl, cr]);
            }
            let mut c = crit("Stopped on runway", &format!("within the {} m runway", r0(rwy.length)));
            if stopped {
                c.value =
                    Some(format!("{} m from the {} threshold", r0(end.dir * (ac.x - end.threshold)), end.name));
                c.pass = Some(rwy.contains(ac.x, ac.y));
            } else if td.is_some() {
                c.value = Some("rolling…".into());
            }
            rows.push(c);
        } else {
            let mut c = crit("Wings level, nose straight", "< 5° crab");
            if let Some(td) = td {
                let crab_deg = td.crab.abs() / D2R;
                c.value = Some(format!("{}° crab", f1(crab_deg)));
                c.pass = Some(crab_deg < 5.0);
            }
            rows.push(c);
            // only a runway counts: a field landing, however gentle, is a fail
            let mut c = crit("Touched down on runway", "any part of the runway (a field is a fail)");
            if let Some(td) = td {
                let dist = (td.x - eo.x).hypot(td.y - eo.y);
                c.value = Some(if td.on_runway {
                    format!("on the runway, {} kt", r0(td.v / KT))
                } else {
                    format!("field, {} km from failure, {} kt", f1(dist / 1000.0), r0(td.v / KT))
                });
                c.pass = Some(td.on_runway);
            }
            rows.push(c);
        }
        rows
    }

    /// True when every pass/fail row passed (info rows ignored) and the aircraft has stopped.
    pub fn passed(&self) -> bool {
        self.finished() && self.evaluate().iter().all(|r| r.info || r.pass == Some(true))
    }
}
