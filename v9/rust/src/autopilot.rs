//! Mode manager (phase state machine) plus the outer loops it switches between, go-around and
//! engine-out logic. Each phase sets targets; the controllers turn them into attitude/throttle
//! commands; the aircraft's `CommandShaper` rate-limits whatever comes out.

use crate::aircraft::{Aircraft, Commands};
use crate::airfield::{normal_procedure, Circuit, Leg, Procedure, Runway};
use crate::atmosphere::Wind;
use crate::control::{AutoThrottle, FlightPathController, SpeedOnPitch, TrackGuidance};
use crate::emergency::{default_eo, GlideAssessment, GlidePlanner, RunwayEnd};
use crate::units::{clamp, wrap_pi, Point, D2R, FT, G, KT};
use serde::Serialize;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize)]
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
pub enum Phase {
    TakeoffRoll,
    Rotate,
    Climb,
    Crosswind,
    Downwind,
    Base,
    Final,
    Flare,
    Rollout,
    Stopped,
    GoAround,
    Rejected,
    EoGlide,
    EoFinal,
    ForcedLanding,
}

impl Phase {
    pub fn code(self) -> &'static str {
        match self {
            Phase::TakeoffRoll => "TAKEOFF_ROLL",
            Phase::Rotate => "ROTATE",
            Phase::Climb => "CLIMB",
            Phase::Crosswind => "CROSSWIND",
            Phase::Downwind => "DOWNWIND",
            Phase::Base => "BASE",
            Phase::Final => "FINAL",
            Phase::Flare => "FLARE",
            Phase::Rollout => "ROLLOUT",
            Phase::Stopped => "STOPPED",
            Phase::GoAround => "GO_AROUND",
            Phase::Rejected => "REJECTED",
            Phase::EoGlide => "EO_GLIDE",
            Phase::EoFinal => "EO_FINAL",
            Phase::ForcedLanding => "FORCED_LANDING",
        }
    }
    pub fn is_engine_out(self) -> bool {
        matches!(self, Phase::Rejected | Phase::EoGlide | Phase::EoFinal | Phase::ForcedLanding)
    }
}

/// Flight mode annunciator: what each channel is doing, as a pilot would read it.
#[derive(Clone, Debug, Serialize, PartialEq)]
pub struct Fma {
    pub thr: String,
    pub vert: String,
    pub lat: String,
}

impl Fma {
    fn new(thr: &str, vert: &str, lat: &str) -> Self {
        Fma { thr: thr.into(), vert: vert.into(), lat: lat.into() }
    }
}

#[derive(Clone, Debug, Serialize)]
pub struct ModeEvent {
    pub t: f64,
    /// A phase code, or "DESCENT" / "ENGINE_FAILURE".
    pub phase: &'static str,
    pub h: f64,
    #[serde(rename = "V")]
    pub v: f64,
    pub x: f64,
    pub y: f64,
    pub note: Option<String>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize)]
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
pub enum EngineOutPlan {
    Reject,
    LandAhead,
    Return,
}

#[derive(Clone, Debug, Serialize)]
pub struct EngineOut {
    pub plan: EngineOutPlan,
    pub why: String,
    pub t: f64,
    pub x: f64,
    pub y: f64,
    pub h: f64,
    pub track: f64,
}

/// Stabilised-approach criteria, checked below the gate height on final.
#[derive(Clone, Debug, Serialize)]
pub struct StabilityGates {
    pub gate_height: f64,
    pub speed_low: f64,
    pub speed_high: f64,
    pub glidepath_deg: f64,
    pub lateral: f64,
    pub sink_rate: f64,
    pub bank: f64,
    pub persistence: f64,
}

pub fn default_gates() -> StabilityGates {
    StabilityGates {
        gate_height: 300.0 * FT,
        speed_low: 5.0 * KT,
        speed_high: 10.0 * KT,
        glidepath_deg: 0.7,
        lateral: 30.0,
        sink_rate: (1000.0 * FT) / 60.0,
        bank: 15.0 * D2R,
        persistence: 2.0,
    }
}

#[derive(Clone, Debug)]
pub struct StabilityMonitor {
    unstable_for: f64,
    pub gates: StabilityGates,
    pub runway: Runway,
    pub vapp: f64,
}

impl StabilityMonitor {
    pub fn reset(&mut self) {
        self.unstable_for = 0.0;
    }
    /// Returns a reason once a criterion has been violated for `persistence` seconds.
    pub fn check(&mut self, ac: &Aircraft, dt: f64) -> Option<&'static str> {
        let g = &self.gates;
        if ac.h > g.gate_height {
            self.unstable_for = 0.0;
            return None;
        }
        let dev = self.runway.glidepath_deviation_deg(ac.x, ac.h);
        let why = if ac.v < self.vapp - g.speed_low {
            Some("Slow on approach")
        } else if ac.v > self.vapp + g.speed_high {
            Some("Fast on approach")
        } else if ac.h > 60.0 * FT && dev.abs() > g.glidepath_deg {
            Some(if dev > 0.0 { "High on glidepath" } else { "Low on glidepath" })
        } else if ac.y.abs() > g.lateral {
            Some("Off centreline")
        } else if ac.vertical_speed() < -g.sink_rate {
            Some("Sink rate")
        } else if ac.phi.abs() > g.bank {
            Some("Excessive bank")
        } else {
            None
        };
        self.unstable_for = if why.is_some() { self.unstable_for + dt } else { 0.0 };
        if self.unstable_for > g.persistence {
            why
        } else {
            None
        }
    }
}

pub struct Autopilot {
    pub phase: Phase,
    pub fma: Fma,
    pub descending: bool,
    pub go_arounds: u32,
    pub go_around_reason: String,
    pub go_around_requested: bool,
    pub touchdown_time: f64,
    /// Set when the engine fails: what was decided, why, and where.
    pub engine_out: Option<EngineOut>,
    pub glide: Option<GlideAssessment>,
    pub planner: GlidePlanner,
    pub glide_speed: SpeedOnPitch,
    eo_flaps: f64,
    passed_base_key: bool,
    slip_side: f64,
    last_assess: f64,
    flare_k: f64,

    pub guidance: TrackGuidance,
    pub path: FlightPathController,
    pub auto_throttle: AutoThrottle,
    pub climb: SpeedOnPitch,
    pub stability: StabilityMonitor,
    pub circuit: Circuit,
    pub procedure: Procedure,
    pub events: Vec<ModeEvent>,

    cmd: Commands,
    ga_start: f64,
    ga_gamma: f64,
    t: f64,
}

impl Autopilot {
    pub fn new(circuit: Circuit) -> Self {
        let p = normal_procedure(&circuit.spec, circuit.runway.length);
        Self::with_procedure(circuit, p)
    }

    pub fn with_procedure(circuit: Circuit, procedure: Procedure) -> Self {
        let stability = StabilityMonitor {
            unstable_for: 0.0,
            gates: default_gates(),
            runway: circuit.runway.clone(),
            vapp: procedure.vapp,
        };
        // power-off aim point: well into the runway, but not past the first quarter of a short strip
        let mut eo = default_eo();
        eo.aim_point = eo.aim_point.min(0.25 * circuit.runway.length);
        let planner = GlidePlanner::new(circuit.runway.clone(), eo);
        Autopilot {
            phase: Phase::TakeoffRoll,
            fma: Fma::new("TOGA", "TO", "RWY"),
            descending: false,
            go_arounds: 0,
            go_around_reason: String::new(),
            go_around_requested: false,
            touchdown_time: 0.0,
            engine_out: None,
            glide: None,
            planner,
            glide_speed: SpeedOnPitch::new(0.02, 0.01, -0.35, 0.1),
            eo_flaps: 0.0,
            passed_base_key: false,
            slip_side: 0.0,
            last_assess: 0.0,
            flare_k: 0.28,
            guidance: TrackGuidance::default(),
            path: FlightPathController::default(),
            auto_throttle: AutoThrottle::default(),
            climb: SpeedOnPitch::default(),
            stability,
            circuit,
            procedure,
            events: vec![ModeEvent { t: 0.0, phase: "TAKEOFF_ROLL", h: 0.0, v: 0.0, x: 0.0, y: 0.0, note: None }],
            cmd: Commands { thr: 1.0, flaps: 10.0, ..Default::default() },
            ga_start: 0.0,
            ga_gamma: 0.0,
            t: 0.0,
        }
    }

    /// Landing direction chosen for a power-off return.
    pub fn eo_end(&self) -> RunwayEnd {
        self.glide.as_ref().map(|g| g.end).unwrap_or(self.planner.ends[0])
    }

    fn log(&mut self, ac: &Aircraft, phase: &'static str, note: Option<String>) {
        self.events.push(ModeEvent { t: self.t, phase, h: ac.h, v: ac.v, x: ac.x, y: ac.y, note });
    }

    pub fn set_phase(&mut self, ac: &Aircraft, p: Phase) {
        if self.phase == p {
            return;
        }
        // flare law: sink rate shrinks linearly with height from whatever it was at entry
        if p == Phase::Flare && !ac.on_ground {
            self.flare_k = clamp((-ac.vertical_speed() - 0.4) / ac.h.max(0.5), 0.28, 2.0);
        }
        self.phase = p;
        self.log(ac, p.code(), None);
    }

    /// Engine failure decision: on the ground → reject; low after take-off → land ahead (no turn-back);
    /// otherwise return to whichever runway direction is reachable with margin, else land ahead.
    pub fn fail_engine(&mut self, ac: &mut Aircraft, wind: &Wind) -> bool {
        if self.engine_out.is_some() || self.phase == Phase::Stopped || self.phase == Phase::Rollout {
            return false;
        }
        ac.engine_failed = true;
        ac.shaper.rates.theta = self.planner.spec.pitch_rate;
        self.go_around_requested = false;
        let track = ac.track(wind);
        self.glide = if ac.on_ground { None } else { Some(self.planner.plan(ac, wind, None)) };
        let min_tb = self.planner.spec.min_turnback_height;
        let (plan, why) = if ac.on_ground {
            (EngineOutPlan::Reject, "On the runway: reject take-off".to_string())
        } else if self.phase == Phase::Flare {
            (EngineOutPlan::Return, "Already in the flare".to_string())
        } else if (self.phase == Phase::Climb || self.phase == Phase::GoAround) && ac.h < min_tb {
            (EngineOutPlan::LandAhead, "Below 700 ft: no turn-back".to_string())
        } else if self.glide.as_ref().unwrap().reachable {
            (EngineOutPlan::Return, format!("Runway {} within glide range", self.glide.as_ref().unwrap().end.name))
        } else {
            (EngineOutPlan::LandAhead, "Both runway directions beyond glide range".to_string())
        };
        self.engine_out =
            Some(EngineOut { plan, why: why.clone(), t: self.t, x: ac.x, y: ac.y, h: ac.h, track });
        self.log(ac, "ENGINE_FAILURE", Some(why));
        // start the glide loop at the expected best-glide path angle; the pitch slew does the rest
        self.glide_speed.engage(-1.0 / ac.glide_ratio(self.planner.spec.vbg, 0.0));
        self.eo_flaps = 0.0;
        self.last_assess = self.t;
        self.passed_base_key = false;
        let end = self.eo_end();
        match plan {
            EngineOutPlan::Reject => self.set_phase(ac, Phase::Rejected),
            EngineOutPlan::LandAhead => self.set_phase(ac, Phase::ForcedLanding),
            EngineOutPlan::Return => {
                if self.phase == Phase::Flare {
                    // keep flaring
                } else if ac.y.abs() < 100.0
                    && wrap_pi(track - end.heading).abs() < 30.0 * D2R
                    && self.planner.to_aim(ac.x, &end) > 0.0
                {
                    self.eo_flaps = (ac.flap_pos / 10.0).round() * 10.0;
                    self.set_phase(ac, Phase::EoFinal);
                } else {
                    self.set_phase(ac, Phase::EoGlide);
                }
            }
        }
        true
    }

    fn turn_radius(&self, ac: &Aircraft, wind: &Wind) -> f64 {
        let gv = ac.ground_velocity(wind);
        let vg = gv.x.hypot(gv.y);
        (vg * vg) / (G * self.circuit.spec.bank_max.tan())
    }
    /// Vertical speed to follow a path whose height is `target` here: feed-forward plus correction.
    fn path_vs(&self, ac: &Aircraft, wind: &Wind, target: f64) -> f64 {
        let gv = ac.ground_velocity(wind);
        let vg = gv.x.hypot(gv.y);
        clamp(-vg * self.circuit.spec.approach_angle.tan() + 0.2 * (target - ac.h), -5.5, 0.5)
    }
    fn glide_pitch(&mut self, ac: &Aircraft, v: f64, dt: f64) -> f64 {
        let g = self.glide_speed.update(ac, v, dt, -0.35);
        self.path.gamma(ac, g, dt)
    }
    fn flare_if_low(&mut self, ac: &Aircraft) {
        // a power-off glide is steep: start the round-out higher, about 3 s of sink above the ground
        if ac.h < self.circuit.spec.flare_height.max(3.0 * -ac.vertical_speed()) {
            self.set_phase(ac, Phase::Flare);
        }
    }

    fn start_go_around(&mut self, ac: &Aircraft, why: &str) {
        self.go_arounds += 1;
        self.go_around_reason = why.to_string();
        self.go_around_requested = false;
        self.stability.reset();
        self.circuit.crosswind_x = None;
        self.descending = false;
        self.ga_start = self.t;
        self.ga_gamma = ac.gamma; // flight-path ramp starts where we are: bumpless
        self.phase = Phase::GoAround;
        self.log(ac, "GO_AROUND", Some(why.to_string()));
    }

    fn leg_bank(&self, ac: &Aircraft, wind: &Wind, leg: Leg, limit: f64) -> (f64, f64) {
        let (a, b) = self.circuit.leg_ends(leg);
        let g = self.guidance.follow(ac, a, b);
        (self.guidance.bank_for(ac, wind, g.desired_track, limit), g.remaining)
    }
    fn turn_lead(&self, ac: &Aircraft, wind: &Wind) -> f64 {
        self.guidance.turn_lead(ac, wind, self.circuit.spec.bank_max, ac.shaper.rates.phi)
    }
    fn alt_hold_with_speed(&mut self, ac: &Aircraft, v: f64, dt: f64, c: &mut Commands) {
        if self.fma.vert != "ALT" {
            self.fma.vert = "ALT".into();
            self.auto_throttle.engage(ac.thr);
        }
        c.theta = self.path.altitude(ac, self.circuit.spec.pattern_alt, dt, 3.0);
        c.thr = self.auto_throttle.update(ac, v, dt);
        self.fma.thr = "SPD".into();
    }

    /// Crab → wing-low: rudder swings the nose onto the runway heading; the side force that creates is
    /// cancelled by banking into wind (feed-forward), and the track loop trims what is left.
    fn align(&mut self, ac: &Aircraft, wind: &Wind, c: &mut Commands, heading: f64, line: Option<(Point, Point)>) {
        // bank allowance tapers toward the ground: 10° at 40 ft, 4° at touchdown
        let lim = clamp((2.5 + 0.2 * ac.h / FT) * D2R, 4.0 * D2R, self.circuit.spec.align_bank_max);
        // sideslip only as much as the crosswind needs (plus a margin): if the aircraft is still turning onto
        // the line, the lateral law fixes the track and the nose is not forced round against it
        let xwind = wind.v.y * heading.cos() - wind.v.x * heading.sin();
        let crab = clamp(xwind / ac.v.max(20.0), -0.5, 0.5).asin().abs();
        let b_max = (crab + 3.0 * D2R).min(ac.spec.max_sideslip);
        c.beta = clamp(wrap_pi(heading - ac.psi), -b_max, b_max);
        // Tight lateral tracking for the last few seconds: commanded sideways acceleration from cross-track
        // error and lateral velocity (relative to the runway line, or to the held track in a field),
        // turned into bank on top of the wing-low feed-forward.
        let gv = ac.ground_velocity(wind);
        let (e, vlat) = match line {
            Some((a, b)) => {
                let f = self.guidance.follow(ac, a, b);
                let (ux, uy) = {
                    let (dx, dy) = (b[0] - a[0], b[1] - a[1]);
                    let l = dx.hypot(dy);
                    (dx / l, dy / l)
                };
                (f.cross_track, ux * gv.y - uy * gv.x)
            }
            None => (0.0, heading.cos() * gv.y - heading.sin() * gv.x),
        };
        let a_lat = clamp(-0.12 * e - 0.7 * vlat, -2.5, 2.5);
        c.phi = clamp(ac.wing_low_bank() + (a_lat / G).atan(), -lim, lim);
        self.fma.lat = "ALIGN".into();
    }
    fn landing_ahead(&self) -> bool {
        matches!(&self.engine_out, Some(e) if e.plan == EngineOutPlan::LandAhead)
    }
    /// Centreline for a landing direction, from far out on the approach to the far end.
    fn end_line(&self, end: &RunwayEnd) -> (Point, Point) {
        (
            [end.threshold - end.dir * 5000.0, 0.0],
            [end.threshold + end.dir * self.circuit.runway.length, 0.0],
        )
    }
    fn line_bank(&self, ac: &Aircraft, wind: &Wind, end: &RunwayEnd, limit: f64) -> f64 {
        let (a, b) = self.end_line(end);
        // a power-off final is short: capture the centreline more tightly than a circuit leg
        let d = self.guidance.follow_with(ac, a, b, 90.0).desired_track;
        self.guidance.bank_for(ac, wind, d, limit)
    }

    pub fn update(&mut self, ac: &Aircraft, wind: &Wind, t: f64, dt: f64) -> Commands {
        self.t = t;
        let mut c = self.cmd;
        let s = self.circuit.spec.clone();
        c.allow_liftoff = false;
        c.no_touchdown = false;
        c.flare = false;
        c.beta = 0.0; // coordinated unless aligning

        match self.phase {
            Phase::TakeoffRoll => {
                c.thr = 1.0;
                c.flaps = 10.0;
                c.theta = 0.0;
                c.phi = 0.0;
                // short-field: stand on the brakes until the engine is at full power, then release
                let holding = self.procedure.hold_brakes && ac.thr < 0.97 && ac.vg < 1.0;
                c.brake = if holding { 1.0 } else { 0.0 };
                self.fma = Fma::new("TOGA", if holding { "BRAKES" } else { "TO" }, "RWY");
                if ac.v >= self.procedure.vr {
                    self.set_phase(ac, Phase::Rotate);
                }
            }
            Phase::Rotate => {
                c.thr = 1.0;
                c.theta = self.procedure.rotate_pitch;
                c.phi = 0.0;
                c.allow_liftoff = true;
                c.no_touchdown = true;
                self.fma.vert = "ROT".into();
                if !ac.on_ground {
                    self.climb.engage(0.06);
                    self.path.gamma_pi.i = 0.0;
                    self.set_phase(ac, Phase::Climb);
                }
            }
            Phase::Climb | Phase::Crosswind => {
                c.no_touchdown = self.phase == Phase::Climb;
                if ac.h > s.flaps_up_alt {
                    c.flaps = 0.0;
                }
                let capture = ac.h > s.pattern_alt - 15.0f64.max(ac.vertical_speed() * 6.0);
                if self.fma.vert == "ALT" || capture {
                    self.alt_hold_with_speed(ac, s.vdownwind, dt, &mut c);
                } else {
                    // short-field: best-angle (Vx) climb until clear of obstacles, then the normal Vy climb
                    let vx = self.procedure.vx.filter(|_| ac.h < self.procedure.vx_until);
                    self.fma.vert = if vx.is_some() { "PITCH-VX" } else { "PITCH-SPD" }.into();
                    self.fma.thr = "TOGA".into();
                    c.thr = 1.0;
                    let g_min = if ac.h < 30.0 { 0.05 } else { -0.03 }; // never pitch down near the ground
                    let g = self.climb.update(ac, vx.unwrap_or(s.vy), dt, g_min);
                    c.theta = self.path.gamma(ac, g, dt);
                }
                if self.phase == Phase::Climb {
                    self.fma.lat = "TRK 090".into();
                    c.phi = self.leg_bank(ac, wind, Leg::Upwind, if ac.h < 60.0 { 10.0 * D2R } else { s.bank_max }).0;
                    if ac.h >= s.crosswind_turn_alt {
                        let lead = self.turn_lead(ac, wind);
                        if self.circuit.crosswind_x.is_none() {
                            self.circuit.crosswind_x =
                                Some((ac.x + lead).max(self.circuit.runway.length + s.crosswind_clearance));
                        }
                        if ac.x >= self.circuit.crosswind_x.unwrap() - lead {
                            self.set_phase(ac, Phase::Crosswind);
                        }
                    }
                } else {
                    self.fma.lat = "LEG XWND".into();
                    let (bank, remaining) = self.leg_bank(ac, wind, Leg::Crosswind, s.bank_max);
                    c.phi = bank;
                    if remaining < self.turn_lead(ac, wind) {
                        self.set_phase(ac, Phase::Downwind);
                    }
                }
            }
            Phase::Downwind => {
                self.fma.lat = "LEG DNWD".into();
                let (bank, remaining) = self.leg_bank(ac, wind, Leg::Downwind, s.bank_max);
                c.phi = bank;
                let prof_h = self.circuit.profile_height(true, ac.x, ac.y, self.turn_radius(ac, wind));
                let start_descent = if s.profile_descent { prof_h < s.pattern_alt - 3.0 } else { ac.x < 0.0 };
                if start_descent && !self.descending {
                    self.descending = true;
                    self.log(ac, "DESCENT", None);
                }
                if self.descending {
                    c.flaps = 10.0;
                    self.fma.vert = if s.profile_descent { "PATH" } else { "V/S" }.into();
                    self.fma.thr = "SPD".into();
                    c.theta = if s.profile_descent {
                        let vs = self.path_vs(ac, wind, prof_h);
                        self.path.vertical_speed(ac, vs, dt)
                    } else {
                        self.path.vertical_speed(ac, clamp(0.15 * (s.base_floor - ac.h), s.descent_vs, 0.5), dt)
                    };
                    c.thr = self.auto_throttle.update(ac, s.vdescent, dt);
                } else {
                    self.alt_hold_with_speed(ac, s.vdownwind, dt, &mut c);
                }
                if remaining < self.turn_lead(ac, wind) {
                    self.set_phase(ac, Phase::Base);
                }
            }
            Phase::Base => {
                self.fma.lat = "LEG BASE".into();
                self.fma.vert = "V/S".into();
                c.flaps = 20.0;
                let (bank, remaining) = self.leg_bank(ac, wind, Leg::Base, s.bank_max);
                c.phi = bank;
                if s.profile_descent {
                    self.fma.vert = "PATH".into();
                    let ph = self.circuit.profile_height(false, ac.x, ac.y, self.turn_radius(ac, wind));
                    let vs = self.path_vs(ac, wind, ph);
                    c.theta = self.path.vertical_speed(ac, vs, dt);
                } else {
                    let floor = self.circuit.runway.glidepath_height(ac.x).min(s.base_floor);
                    c.theta = self.path.vertical_speed(ac, clamp(0.15 * (floor - ac.h), s.descent_vs, 0.5), dt);
                }
                c.thr = self.auto_throttle.update(ac, s.vbase, dt);
                if self.go_around_requested {
                    self.start_go_around(ac, "Pilot request");
                } else if remaining < self.turn_lead(ac, wind) {
                    self.set_phase(ac, Phase::Final);
                }
            }
            Phase::Final => {
                self.fma.lat = "LOC".into();
                self.fma.vert = "G/S".into();
                c.flaps = 30.0;
                if ac.h < s.align_height {
                    let line = self.circuit.leg_ends(Leg::Final);
                    self.align(ac, wind, &mut c, 0.0, Some(line));
                } else {
                    c.phi =
                        self.leg_bank(ac, wind, Leg::Final, if ac.h < 30.0 { 10.0 * D2R } else { s.bank_max }).0;
                }
                let gvx = ac.ground_velocity(wind).x;
                let rwy = &self.circuit.runway;
                let vs = clamp(-gvx * rwy.glideslope.tan() + 0.25 * (rwy.glidepath_height(ac.x) - ac.h), -5.5, 1.0);
                c.theta = self.path.vertical_speed(ac, vs, dt);
                // gusts reported: add half the gust factor (≈4 kt) to the approach speed for stall margin
                let gust_add = match (wind.settings.gusts, self.procedure.short_field) {
                    (false, _) => 0.0,
                    (true, false) => 4.0 * KT,
                    (true, true) => 2.0 * KT, // short field: less margin, less float
                };
                c.thr = self.auto_throttle.update(ac, self.procedure.vapp + gust_add, dt);
                let why = if self.go_around_requested { Some("Pilot request") } else { self.stability.check(ac, dt) };
                if let Some(w) = why {
                    self.start_go_around(ac, w);
                } else if ac.h < s.flare_height.max(self.procedure.flare_lead * -ac.vertical_speed()) {
                    self.set_phase(ac, Phase::Flare);
                }
            }
            Phase::Flare => {
                self.fma.vert = "FLARE".into();
                self.fma.thr = if self.engine_out.is_some() { "ENG OUT" } else { "IDLE" }.into();
                c.thr = 0.0;
                c.flare = true;
                if self.landing_ahead() {
                    let tr = self.engine_out.as_ref().unwrap().track;
                    self.align(ac, wind, &mut c, tr, None);
                } else if self.engine_out.is_some() {
                    let end = self.eo_end();
                    let line = self.end_line(&end);
                    self.align(ac, wind, &mut c, end.heading, Some(line));
                } else {
                    let line = self.circuit.leg_ends(Leg::Final);
                    self.align(ac, wind, &mut c, 0.0, Some(line));
                }
                let floor = if self.engine_out.is_some() { 0.4 } else { self.procedure.flare_floor };
                c.theta = self.path.vertical_speed(ac, -(floor + self.flare_k * ac.h), dt);
                if ac.on_ground {
                    self.touchdown_time = t;
                    self.set_phase(ac, Phase::Rollout);
                } else if self.engine_out.is_some() {
                    // no power: committed to land
                } else if self.go_around_requested {
                    self.start_go_around(ac, "Pilot request");
                } else if ac.x > self.procedure.long_landing_limit && self.go_arounds < 2 {
                    // after two long-landing go-arounds, accept the long landing rather than circle forever
                    self.start_go_around(ac, "Long landing");
                }
            }
            Phase::Rollout => {
                let field = self.landing_ahead();
                self.fma = Fma::new(
                    if self.engine_out.is_some() { "ENG OUT" } else { "IDLE" },
                    "ROLLOUT",
                    if field { "FIELD" } else { "RWY" },
                );
                c.thr = 0.0;
                c.theta = 0.0;
                c.phi = 0.0;
                c.flaps = 0.0;
                let delay = if self.engine_out.is_some() && !self.procedure.short_field { 2.0 } else { self.procedure.brake_delay };
                c.brake = if t - self.touchdown_time > delay { 1.0 } else { 0.0 };
                if ac.vg < 0.5 {
                    self.set_phase(ac, Phase::Stopped);
                    self.fma = Fma::new("—", "—", "—");
                }
            }
            Phase::GoAround => {
                self.fma = Fma::new("TOGA", "GA", "TRK 090");
                c.thr = 1.0;
                c.no_touchdown = true;
                self.ga_gamma = (self.ga_gamma + 1.5 * D2R * dt).min(4.0 * D2R);
                let slow = self.procedure.vapp - 3.0 * KT - ac.v; // speed protection: trade climb for airspeed
                let g = clamp(self.ga_gamma - if slow > 0.0 { 0.02 * slow } else { 0.0 }, -0.06, 0.12);
                c.theta = self.path.gamma(ac, g, dt);
                let vs = ac.vertical_speed();
                if c.flaps > 20.0 {
                    c.flaps = 20.0;
                }
                if c.flaps > 10.0 && vs > 1.0 && ac.v > 63.0 * KT && t - self.ga_start > 4.0 {
                    c.flaps = 10.0;
                }
                c.phi = self.leg_bank(ac, wind, Leg::Upwind, 10.0 * D2R).0;
                if vs > 1.5 && ac.h > 150.0 * FT && ac.flap_pos <= 10.5 {
                    self.climb.engage(ac.gamma);
                    self.fma.vert = "PITCH-SPD".into();
                    self.set_phase(ac, Phase::Climb);
                }
            }
            Phase::Rejected => {
                self.fma = Fma::new("ENG OUT", "REJECT", "RWY");
                c.thr = 0.0;
                c.theta = 0.0;
                c.phi = 0.0;
                c.brake = 1.0;
                if ac.vg < 0.5 {
                    self.set_phase(ac, Phase::Stopped);
                    self.fma = Fma::new("—", "—", "—");
                }
            }
            Phase::EoGlide => self.eo_glide(ac, wind, t, dt, &mut c, &s),
            Phase::EoFinal => self.eo_final(ac, wind, dt, &mut c, &s),
            Phase::ForcedLanding => {
                let vbg = self.planner.spec.vbg;
                let vapp_eo = self.planner.spec.vapp_eo;
                let tr = self.engine_out.as_ref().unwrap().track;
                self.fma = Fma::new("ENG OUT", "GLIDE", "TRK HOLD");
                c.thr = 0.0;
                c.flaps = if ac.h < 300.0 * FT { 20.0 } else { 0.0 };
                let gust_add = if wind.settings.gusts { 4.0 * KT } else { 0.0 };
                c.theta = self.glide_pitch(ac, if c.flaps >= 20.0 { vapp_eo + gust_add } else { vbg }, dt);
                if ac.h < s.align_height {
                    self.align(ac, wind, &mut c, tr, None);
                } else {
                    c.phi = self.guidance.bank_for(ac, wind, tr, 15.0 * D2R);
                }
                self.flare_if_low(ac);
            }
            Phase::Stopped => {
                c.thr = 0.0;
                c.brake = 1.0;
            }
        }
        self.cmd = c;
        c
    }

    /// Best glide, clean: direct to a base key point, down a short base leg, then turn final at the intercept.
    fn eo_glide(&mut self, ac: &Aircraft, wind: &Wind, t: f64, dt: f64, c: &mut Commands, s: &crate::airfield::CircuitSpec) {
        let end = self.eo_end();
        let ip_d = self.glide.as_ref().unwrap().ip_distance;
        let wp = self.planner.waypoints(ac, ip_d, &end);
        self.fma = Fma::new("ENG OUT", "GLIDE", &format!("DIR IP {}", end.name));
        c.thr = 0.0;
        c.flaps = 0.0;
        c.theta = self.glide_pitch(ac, self.planner.spec.vbg, dt);
        let gv = ac.ground_velocity(wind);
        let vg = gv.x.hypot(gv.y);
        let r = (vg * vg) / (G * s.bank_max.tan());
        let roll_in = vg * (s.bank_max / ac.shaper.rates.phi) * 0.5;
        match wp.base_key {
            Some([bx, by]) if !self.passed_base_key => {
                let brg = (by - ac.y).atan2(bx - ac.x);
                let dist = (bx - ac.x).hypot(by - ac.y);
                let base_brg = (wp.ip[1] - by).atan2(wp.ip[0] - bx);
                let lead = r * (wrap_pi(base_brg - brg).abs().min(2.5) / 2.0).tan() + roll_in;
                self.fma.lat = "DIR KEY".into();
                c.phi = self.guidance.bank_for(ac, wind, brg, s.bank_max);
                if dist < lead || end.dir * (ac.x - bx) < 0.0 {
                    self.passed_base_key = true;
                }
            }
            Some(key) => {
                let g = self.guidance.follow(ac, key, wp.ip);
                self.fma.lat = "BASE".into();
                c.phi = self.guidance.bank_for(ac, wind, g.desired_track, s.bank_max);
                if g.remaining < r + roll_in {
                    self.set_phase(ac, Phase::EoFinal);
                    return;
                }
            }
            None => {
                let [ipx, ipy] = wp.ip;
                let brg = (ipy - ac.y).atan2(ipx - ac.x);
                let dist = (ipx - ac.x).hypot(ipy - ac.y);
                let lead = r * (wrap_pi(brg - end.heading).abs().min(2.5) / 2.0).tan() + roll_in;
                c.phi = self.guidance.bank_for(ac, wind, brg, s.bank_max);
                if dist < lead || (end.dir * (ac.x - ipx) > 0.0 && ac.y.abs() < 150.0) {
                    self.set_phase(ac, Phase::EoFinal);
                    return;
                }
            }
        }
        if t - self.last_assess > 1.0 {
            // re-plan each second: longest reachable final, keep the current one if only that works,
            // give up on the runway only when nothing reaches
            self.last_assess = t;
            let fresh = self.planner.plan(ac, wind, Some(&end));
            let keep = self.planner.assess(ac, wind, ip_d, &end);
            let g = if fresh.reachable { fresh } else if keep.available >= keep.required { keep } else { fresh };
            let lost = g.available < g.required;
            self.glide = Some(g);
            if lost {
                let tr = ac.track(wind);
                let eo = self.engine_out.as_mut().unwrap();
                eo.plan = EngineOutPlan::LandAhead;
                eo.track = tr;
                self.log(ac, "FORCED_LANDING", Some("Runway no longer reachable".into()));
                self.phase = Phase::ForcedLanding;
            }
        }
    }

    /// Power-off final: add flap in stages when still high (never retract), forward-slip if even full flap
    /// will not get it down, straighten for touchdown below 60 ft.
    fn eo_final(&mut self, ac: &Aircraft, wind: &Wind, dt: f64, c: &mut Commands, s: &crate::airfield::CircuitSpec) {
        let end = self.eo_end();
        let dist = self.planner.to_aim(ac.x, &end);
        let greq = if dist > 30.0 { ac.h / dist } else { f64::INFINITY };
        // flap beyond 10° and slipping wait until lined up, so the turn onto final uses the flatter glide
        let aligned = wrap_pi(ac.track(wind) - end.heading).abs() < 15.0 * D2R && ac.y.abs() < 80.0;
        let flap_cap = if aligned || ac.h < 300.0 * FT { 30.0 } else { 10.0 };
        while self.eo_flaps < flap_cap && greq > self.planner.gradient(ac, wind, self.eo_flaps + 10.0, &end) * 1.05 {
            self.eo_flaps += 10.0;
        }
        c.thr = 0.0;
        c.flaps = self.eo_flaps;
        let gust_add = if wind.settings.gusts { 4.0 * KT } else { 0.0 };
        // short-field technique applies to a power-off landing too: slower final, so less float
        let vapp_eo = if self.procedure.short_field { self.procedure.vapp } else { self.planner.spec.vapp_eo };
        let v = if self.eo_flaps >= 20.0 { vapp_eo + gust_add } else { self.planner.spec.vbg };
        c.theta = self.glide_pitch(ac, v, dt);
        self.fma = Fma::new("ENG OUT", &format!("GLIDE F{}", self.eo_flaps as i32), &format!("LOC {}", end.name));
        let slip = aligned
            && self.eo_flaps == 30.0
            && greq > self.planner.gradient(ac, wind, 30.0, &end) * 1.1
            && ac.h > s.align_height;
        if ac.h < s.align_height {
            let line = self.end_line(&end);
            self.align(ac, wind, c, end.heading, Some(line));
        } else if slip {
            // forward slip: into-wind wing down, opposite rudder, track held on the centreline;
            // the direction is picked once so it does not flip back and forth
            let rel = wrap_pi(ac.psi - end.heading);
            if self.slip_side == 0.0 {
                self.slip_side = if rel.abs() < 1.0 * D2R { 1.0 } else { rel.signum() };
            }
            c.beta = -self.slip_side * 12.0 * D2R;
            c.phi = clamp(ac.wing_low_bank() + self.line_bank(ac, wind, &end, 6.0 * D2R), -12.0 * D2R, 12.0 * D2R);
            self.fma.lat = "SLIP".into();
        } else {
            c.phi = self.line_bank(ac, wind, &end, if ac.h < 30.0 { 10.0 * D2R } else { s.bank_max });
        }
        self.flare_if_low(ac);
    }

    /// Called by the simulation when the wheels touch outside the flare (e.g. a short approach).
    pub fn on_unexpected_touchdown(&mut self, ac: &Aircraft) {
        if self.phase != Phase::Flare {
            self.set_phase(ac, Phase::Flare);
        }
    }
}
