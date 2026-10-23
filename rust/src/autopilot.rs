//! Mode manager (phase state machine) plus the outer loops it switches between, go-around and
//! engine-out logic. Each phase sets targets; the controllers turn them into attitude/throttle
//! commands; the aircraft's `CommandShaper` rate-limits whatever comes out.

use crate::aircraft::{Aircraft, Commands};
use crate::airfield::{normal_procedure, Circuit, Leg, Procedure, Runway};
use crate::atmosphere::Wind;
use crate::control::{AutoThrottle, FlightPathController, GustEstimator, SpeedOnPitch, TrackGuidance};
use crate::dubins::{self, Follower, Motion};
use crate::emergency::{self, EngineOutSpec, GlideAssessment, GlidePlanner, RunwayEnd};
use crate::tuning::Tuning;
use crate::units::{clamp, wrap_pi, Point, Vec2, D2R, FT, G, KT, PI};
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


/// C172P flap limit speeds, KIAS: 10° flap, and beyond 10°.
const VFE_10: f64 = 110.0;
const VFE_FULL: f64 = 85.0;
/// Bank limit: airspeed over the stall speed at the turn's load factor.
const BANK_STALL_MARGIN: f64 = 1.2;
/// Bank always allowed, deg.
const BANK_FLOOR: f64 = 15.0;
/// Heights, ft: bank limited to 10° below LOW_BANK in the initial climb; near the ground below
/// NEAR_GROUND (no pitching down in the climb, 10° bank on final).
const LOW_BANK: f64 = 200.0;
const NEAR_GROUND: f64 = 100.0;
/// Flare hold-off: touchdown speed (× the stall speed for the flap set), the speed band above it over
/// which the sink floor fades in (kt), and the wheel height held while the speed bleeds (m).
const TD_SPEED: f64 = 1.05;
/// Touchdown speed added per unit of crosswind component (crosswind technique: more rudder authority).
pub const TD_XWIND_K: f64 = 0.75;
const HOLD_OFF_BAND: f64 = 5.0;
const HOLD_OFF_HEIGHT: f64 = 0.3;
/// Power-off: no flap is raised below this height, ft (the sink while it comes up costs more than it saves).
const RETRACT_FLOOR: f64 = 100.0;
/// Lowest height for a forward slip, ft (below it the crosswind alignment law takes over).
const SLIP_FLOOR: f64 = 100.0;
/// Sink-rate gain of a full-rudder forward slip in this model (`examples/slip_test.rs`: ×1.25–1.3).
const SLIP_SINK_GAIN: f64 = 1.25;
/// Largest centreline offset and track error for a slip, m and deg.
const SLIP_ALIGN_Y: f64 = 60.0;
const SLIP_ALIGN_TRK: f64 = 15.0;
/// Bank allowed for holding the centreline while slipping, deg.
const SLIP_LINE_BANK: f64 = 6.0;
/// Largest turn toward the wind for a field landing, deg.
const FIELD_TURN_MAX: f64 = 90.0;
/// Power-off centreline capture distance, as a fraction of the ground-speed turn radius.
const LINE_CAPTURE_K: f64 = 0.8;
/// A fresh power-off plan replaces the current one when it has this much more height in hand (counted
/// up to `PLAN_KEEP`), m: re-planning mid-turn costs height, so only for a clear gain.
const SWITCH_GAIN: f64 = 20.0;
const PLAN_KEEP: f64 = 60.0;
/// The power-off path is made afresh when the aircraft is this far off it, m.
const PATH_OFF: f64 = 80.0;
/// On a power-off final, go back to planning a longer path when even full drag lands this far beyond
/// the farthest usable aim point (as a fraction of the distance), above `REPLAN_FLOOR` ft.
const REPLAN_LONG: f64 = 0.25;
const REPLAN_FLOOR: f64 = 400.0;

/// The power-off drag ladder: flap settings, then a forward slip at flap 20, in order of glide gradient.
/// The final picks the level whose glide lands nearest the aim point (never short of the nearest one).
const LADDER: [(f64, bool); 5] = [(0.0, false), (10.0, false), (20.0, false), (20.0, true), (30.0, false)];

/// A power-off path being followed: the Dubins path in the air mass, when and with which wind estimate it
/// was made (the air-mass frame drifts with that wind from then on), and the plan it serves.
#[derive(Clone, Debug)]
struct EoPath {
    follow: Follower,
    t0: f64,
    wind: Vec2,
    end: &'static str,
    fix: f64,
}

/// Flare law: sink rate commanded at wheel height `h` and airspeed `v`. It shrinks in proportion to the
/// height above the hold-off height (gain `k`, set at flare entry), down to a floor that fades in over
/// the last `HOLD_OFF_BAND` kt above the touchdown speed `v_td`: fast, the aircraft holds off just above
/// the runway while the speed bleeds; at the touchdown speed it settles at `floor` m/s.
pub fn flare_sink(h: f64, v: f64, v_td: f64, floor: f64, k: f64) -> f64 {
    let settle = clamp((v_td + HOLD_OFF_BAND * KT - v) / (HOLD_OFF_BAND * KT), 0.0, 1.0);
    floor * settle + k * (h - HOLD_OFF_HEIGHT).max(0.0)
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
    /// Power-off drag ladder level (index into `LADDER`).
    drag: usize,
    /// The power-off path being followed (air-mass frame), when gliding to a final fix.
    eo_path: Option<EoPath>,
    slip_side: f64,
    last_assess: f64,
    flare_k: f64,
    /// Turbulence seen in the airspeed: sizes the gust margin and the flare.
    pub gust: GustEstimator,
    /// Landing parameters searched by `circuitsim autotune`.
    pub tune: Tuning,

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
        Self::with_procedure(circuit, p, Tuning::default())
    }

    /// The one constructor: every tuned value (landing parameters, power-off planning) comes from `tune`.
    pub fn with_procedure(circuit: Circuit, mut procedure: Procedure, tune: Tuning) -> Self {
        procedure.flare_lead = if procedure.short_field { tune.flare_lead_short } else { tune.flare_lead };
        let stability = StabilityMonitor {
            unstable_for: 0.0,
            gates: default_gates(),
            runway: circuit.runway.clone(),
            vapp: procedure.vapp,
        };
        let planner = GlidePlanner::new(circuit.runway.clone(), EngineOutSpec::new(&tune, circuit.runway.length));
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
            drag: 0,
            eo_path: None,
            slip_side: 0.0,
            last_assess: 0.0,
            flare_k: 0.28,
            gust: GustEstimator::default(),
            tune,
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

    /// Approach-speed margin for turbulence: about half the gust factor (1.6 σ), up to 5 kt; short-field
    /// technique takes less, to limit float.
    pub fn gust_add(&self) -> f64 {
        let cap = if self.procedure.short_field { 3.0 * KT } else { 5.0 * KT };
        (self.tune.gust_k * self.gust.sigma()).min(cap)
    }

    /// Power-off: no throttle to catch a lull, so carry more margin into the flare.
    pub fn gust_add_eo(&self) -> f64 {
        (self.tune.gust_k_eo * self.gust.sigma()).min(5.0 * KT)
    }

    /// Landing direction chosen for a power-off return.
    /// The power-off flap setting the autopilot is commanding, deg.
    pub fn eo_flaps_cmd(&self) -> f64 {
        self.eo_flaps
    }
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
    /// otherwise glide to whichever runway direction the planner reaches with margin; lined up on final,
    /// go for the runway whatever the margin (landing ahead points the same way); else land ahead.
    /// `wind` is only passed on to the estimator's frame: decisions use the estimated mean wind.
    pub fn fail_engine(&mut self, ac: &Aircraft, _wind: &Wind) -> bool {
        if self.engine_out.is_some() || self.phase == Phase::Stopped || self.phase == Phase::Rollout {
            return false;
        }
        let est = Wind::steady(self.gust.mean_wind());
        let wind = &est;
        let w = wind.v;
        self.go_around_requested = false;
        let track = ac.track(wind);
        let on_final = self.on_final_end(ac, wind, false);
        self.glide = if ac.on_ground { None } else { Some(self.planner.plan(ac, w, None)) };
        let min_tb = self.planner.spec.min_turnback_height;
        let (plan, why) = if ac.on_ground {
            (EngineOutPlan::Reject, "On the runway: reject take-off".to_string())
        } else if self.phase == Phase::Flare {
            (EngineOutPlan::Return, "Already in the flare".to_string())
        } else if (self.phase == Phase::Climb || self.phase == Phase::GoAround) && ac.h < min_tb {
            (EngineOutPlan::LandAhead, "Below 700 ft: no turn-back".to_string())
        } else if self.glide.as_ref().unwrap().reachable {
            let g = self.glide.as_ref().unwrap();
            let how = if g.aim < self.planner.spec.aim_point - 1.0 { format!(", aiming {} m in", g.aim.round()) } else { String::new() };
            (EngineOutPlan::Return, format!("Runway {} within glide range{how}", g.end.name))
        } else if let Some(e) = on_final {
            self.glide = Some(self.planner.plan_for(ac, w, &e));
            (EngineOutPlan::Return, format!("On final: straight ahead for runway {}, flap up to stretch", e.name))
        } else {
            (EngineOutPlan::LandAhead, "Both runway directions beyond glide range".to_string())
        };
        self.engine_out =
            Some(EngineOut { plan, why: why.clone(), t: self.t, x: ac.x, y: ac.y, h: ac.h, track: if plan == EngineOutPlan::LandAhead { Self::field_track(ac, wind, track) } else { track } });
        self.log(ac, "ENGINE_FAILURE", Some(why));
        // start the glide loop at the expected best-glide path angle; the pitch slew does the rest
        self.glide_speed.engage(-1.0 / ac.glide_ratio(self.planner.spec.vbg, 0.0));
        // flap: raised for the glide (the plan assumes it), kept as it is when already on final
        self.eo_flaps = 0.0;
        self.drag = 0;
        self.eo_path = None;
        self.last_assess = self.t;
        match plan {
            EngineOutPlan::Reject => self.set_phase(ac, Phase::Rejected),
            EngineOutPlan::LandAhead => self.set_phase(ac, Phase::ForcedLanding),
            EngineOutPlan::Return => {
                if self.phase == Phase::Flare {
                    // keep flaring
                } else if on_final.is_some_and(|e| e.name == self.eo_end().name) {
                    self.eo_flaps = (ac.flap_pos / 10.0).round() * 10.0;
                    self.drag = LADDER.iter().rposition(|l| l.0 <= self.eo_flaps && !l.1).unwrap_or(0);
                    self.set_phase(ac, Phase::EoFinal);
                } else {
                    self.set_phase(ac, Phase::EoGlide);
                }
            }
        }
        true
    }

    /// The runway direction the aircraft is lined up on, if any: near the centreline, tracking down it,
    /// before the threshold (or, with `over_runway`, anywhere before the far end).
    fn on_final_end(&self, ac: &Aircraft, wind: &Wind, over_runway: bool) -> Option<RunwayEnd> {
        let track = ac.track(wind);
        let len = self.planner.runway.length;
        self.planner.ends.iter().copied().find(|e| {
            let to_thr = e.dir * (e.threshold - ac.x);
            !ac.on_ground
                && ac.y.abs() < 100.0
                && wrap_pi(track - e.heading).abs() < 30.0 * D2R
                && (to_thr > 0.0 || (over_runway && to_thr > -(len - emergency::MIN_FINAL)))
        })
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
        // a power-off glide is steep: start the round-out higher (about 2.75 s of sink above the ground)
        if ac.h < self.circuit.spec.flare_height.max(self.tune.flare_lead_eo * -ac.vertical_speed()) {
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
        // a power-off final is short: capture the centreline tightly, but never tighter than the
        // aircraft can turn at its ground speed. A fixed 90 m capture with a strong tailwind on base
        // (25 kt) left the turn too late and swung 300 m through the centreline.
        let gv = ac.ground_velocity(wind);
        let vg = gv.x.hypot(gv.y);
        // the radius at the circuit bank, whatever this call's bank limit (a slip allows only a few degrees)
        let radius = vg * vg / (G * self.circuit.spec.bank_max.tan());
        let d = self.guidance.follow_with(ac, a, b, (LINE_CAPTURE_K * radius).max(90.0)).desired_track;
        self.guidance.bank_for(ac, wind, d, limit)
    }

    pub fn update(&mut self, ac: &Aircraft, wind: &Wind, t: f64, dt: f64) -> Commands {
        self.t = t;
        if !ac.on_ground {
            self.gust.update(ac, wind, dt, self.cmd.beta.abs() > 1.0 * D2R);
        }
        // the autopilot flies on what it can measure: from here on, `wind` is the estimated mean wind
        let est = Wind::steady(self.gust.mean_wind());
        let wind = &est;
        // a fresh command each step; only the flap lever stays where it was until a phase moves it
        let mut c = Commands { flaps: self.cmd.flaps, ..Default::default() };
        let s = self.circuit.spec.clone();

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
                    self.path.reset();
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
                    let g_min = if ac.h < NEAR_GROUND * FT { 0.05 } else { -0.03 }; // never pitch down near the ground
                    let g = self.climb.update(ac, vx.unwrap_or(s.v_climb), dt, g_min);
                    c.theta = self.path.gamma(ac, g, dt);
                }
                if self.phase == Phase::Climb {
                    self.fma.lat = "TRK 090".into();
                    c.phi = self.leg_bank(ac, wind, Leg::Upwind, if ac.h < LOW_BANK * FT { 10.0 * D2R } else { s.bank_max }).0;
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
                        self.leg_bank(ac, wind, Leg::Final, if ac.h < NEAR_GROUND * FT { 10.0 * D2R } else { s.bank_max }).0;
                }
                let gvx = ac.ground_velocity(wind).x;
                let rwy = &self.circuit.runway;
                let vs = clamp(-gvx * rwy.glideslope.tan() + 0.25 * (rwy.glidepath_height(ac.x) - ac.h), -5.5, 1.0);
                c.theta = self.path.vertical_speed(ac, vs, dt);
                // gusts reported: add half the gust factor (≈4 kt) to the approach speed for stall margin
                let gust_add = self.gust_add();
                c.thr = self.auto_throttle.update(ac, self.procedure.vapp + gust_add, dt);
                let why = if self.go_around_requested { Some("Pilot request") } else { self.stability.check(ac, dt) };
                if let Some(w) = why {
                    self.start_go_around(ac, w);
                } else if ac.h < s.flare_height.max((self.procedure.flare_lead + 0.3 * self.gust.level()) * -ac.vertical_speed()) {
                    self.set_phase(ac, Phase::Flare);
                }
            }
            Phase::Flare => {
                self.fma.vert = "FLARE".into();
                c.flare = true;
                // lull protection: if a lull drops the airspeed below `lull_vs` × Vs (under the touchdown
                // speed the hold-off bleeds to), add power to hold it, as a pilot would, rather than run out
                // of elevator into a hard arrival
                let vs = ac.stall_speed(ac.flap_pos);
                let deficit = self.tune.lull_vs * vs - ac.v;
                c.thr = if self.engine_out.is_none() && deficit > 0.0 {
                    clamp(0.3 * deficit, 0.0, 0.6) * (0.3 + 0.7 * self.gust.level())
                } else {
                    0.0
                };
                self.fma.thr = if self.engine_out.is_some() {
                    "ENG OUT"
                } else if c.thr > 0.0 {
                    "SPD"
                } else {
                    "IDLE"
                }
                .into();
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
                // in turbulence a firm short-field arrival turns hard when a lull comes: aim softer
                let floor = if self.engine_out.is_some() {
                    0.4
                } else {
                    let f = self.procedure.flare_floor;
                    f - (f - 0.4).max(0.0) * self.gust.level()
                };
                // hold-off: round out to just above the runway and let the speed bleed; the sink floor
                // comes in only as the speed nears the touchdown speed, so the wheels touch slow and
                // nose-up instead of at the approach speed
                // in a crosswind, touch down faster by ¾ of the crosswind for rudder authority
                let rwy_hdg = if self.engine_out.is_some() && !self.landing_ahead() { self.eo_end().heading } else { 0.0 };
                let xwind = emergency::across(wind.v, rwy_hdg).abs();
                let v_td = TD_SPEED * vs + TD_XWIND_K * xwind;
                let theta = self.path.vertical_speed(ac, -flare_sink(ac.h, ac.v, v_td, floor, self.flare_k), dt);
                // never past the tail-strike attitude
                c.theta = theta.min(ac.spec.tail_strike_pitch - 0.5 * D2R);
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
            Phase::EoGlide => self.eo_glide(ac, wind, t, dt, &mut c),
            Phase::EoFinal => self.eo_final(ac, wind, dt, &mut c, &s),
            Phase::ForcedLanding => {
                let tr = self.engine_out.as_ref().unwrap().track;
                self.fma = Fma::new("ENG OUT", "GLIDE", "TRK HOLD");
                c.thr = 0.0;
                // flap that is out stays out; flap 20 for the landing
                if ac.h < 300.0 * FT {
                    self.eo_flaps = self.eo_flaps.max(20.0);
                }
                c.flaps = self.eo_flaps;
                let v = if c.flaps >= 20.0 { self.planner.spec.vapp_eo + self.gust_add_eo() } else { self.planner.spec.vbg };
                c.theta = self.glide_pitch(ac, v.max(self.planner.turn_speed(ac, ac.shaper.phi, c.flaps)), dt);
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
        // stall-margin bank limit: the load factor of a level-ish turn (1/cos φ) must leave the stall speed
        // at that load factor at least BANK_STALL_MARGIN below the airspeed flown, so a power-off turn
        // flown slow cannot become an accelerated stall (floor BANK_FLOOR so the aircraft can always turn)
        if !ac.on_ground {
            let vs = ac.stall_speed(ac.flap_pos);
            let cos_min = (BANK_STALL_MARGIN * vs / ac.v.max(1.0)).powi(2);
            let lim = if cos_min >= 1.0 { 0.0 } else { cos_min.acos() }.max(BANK_FLOOR * D2R);
            c.phi = clamp(c.phi, -lim, lim);
        }
        // flap limits (C172P): beyond 10° only at or below 85 KIAS, any flap only at or below 110 KIAS;
        // the command waits (a 2 kt margin) rather than let a phase put flap out too fast
        if ac.v > (VFE_FULL - 2.0) * KT {
            c.flaps = c.flaps.min(10.0);
        }
        if ac.v > (VFE_10 - 2.0) * KT {
            c.flaps = 0.0;
        }
        if self.engine_out.is_some() {
            c.pitch_rate = Some(self.planner.spec.pitch_rate);
        }
        self.cmd = c;
        c
    }

    /// Power-off glide along the planned Dubins path to the final fix. The path is made in the air mass
    /// and followed there (the aircraft's air-mass position is its ground position less the wind's drift
    /// since the path was made) with one vector-field law; it is made afresh when the plan changes
    /// (direction, fix) or the aircraft is more than `PATH_OFF` m off it. The energy is re-checked every
    /// second; the runway is given up only when no plan reaches it.
    fn eo_glide(&mut self, ac: &Aircraft, wind: &Wind, t: f64, dt: f64, c: &mut Commands) {
        let g = self.glide.clone().unwrap();
        let end = g.end;
        let w = wind.v;
        c.thr = 0.0;
        c.flaps = self.eo_flaps;
        let bank = self.planner.spec.turn_bank;
        let stale = match &self.eo_path {
            None => true,
            Some(p) => p.end != end.name || (p.fix - g.ip_distance).abs() > 1.0,
        };
        if stale {
            let (path, _, _) = self.planner.path_to_fix(ac, w, &end, g.ip_distance);
            self.eo_path = Some(EoPath { follow: Follower::new(path), t0: t, wind: w, end: end.name, fix: g.ip_distance });
        }
        let m = Motion { v: ac.v, phi: ac.phi, roll: ac.shaper.rates.phi, bank };
        let ep = self.eo_path.as_mut().unwrap();
        let (qx, qy) = (ac.x - ep.wind.x * (t - ep.t0), ac.y - ep.wind.y * (t - ep.t0));
        let st = ep.follow.steer(qx, qy, ac.psi, m);
        let seg = ep.follow.seg;
        let (arc_left, straight_left) = (st.arc_left, st.straight_left);
        if st.offset > PATH_OFF {
            self.eo_path = None; // made afresh next step from where the aircraft is
        }
        // bank: the arc's feed-forward, plus a correction from the heading error (3 s to take it out)
        c.phi = dubins::bank_command(&st, m, ac.psi, 3.0);
        c.theta = self.glide_pitch(ac, self.planner.turn_speed(ac, ac.shaper.phi, self.eo_flaps), dt);
        let what = if st.turn > 0.0 { "L" } else if st.turn < 0.0 { "R" } else { "TO" };
        self.fma = Fma::new("ENG OUT", "GLIDE", &format!("{what} FIX {}", end.name));
        // joined the final: lined up, the path's last turn done (or nearly), at or near the join point
        let to_join = self.planner.to_point(ac.x, -g.ip_distance, &end);
        let turned = seg == 3 || (seg == 2 && st.to_go < 15.0 * D2R);
        if turned && self.on_final_end(ac, wind, true).is_some_and(|e| e.name == end.name) && to_join <= 150.0 {
            self.set_phase(ac, Phase::EoFinal);
            return;
        }
        if t - self.last_assess > 1.0 {
            self.last_assess = t;
            // keep the plan while it works, unless a fresh one is clearly better: it reaches when this one
            // no longer does, or has `SWITCH_GAIN` more height in hand
            let mut keep = self.planner.assess_along(ac, w, &end, g.ip_distance, g.aim, arc_left, straight_left);
            keep.route = g.route.clone();
            let fresh = self.planner.plan(ac, w, Some(&end));
            let switch = fresh.reachable && (!keep.reachable || fresh.surplus.min(PLAN_KEEP) > keep.surplus.min(PLAN_KEEP) + SWITCH_GAIN);
            if keep.reachable && !switch {
                self.glide = Some(keep);
            } else if switch {
                if fresh.end.name != end.name || (fresh.ip_distance - g.ip_distance).abs() > 1.0 {
                    self.log(ac, "REPLAN", Some(format!("Runway {}, joining {} m {}, aiming {} m in", fresh.end.name, fresh.ip_distance.abs().round(), if fresh.ip_distance >= 0.0 { "out" } else { "in" }, fresh.aim.round())));
                }
                self.glide = Some(fresh);
            } else if keep.available >= keep.required {
                // the margins are eaten but the path still reaches: carry on
                self.glide = Some(keep);
            } else {
                let tr = ac.track(wind);
                let eo = self.engine_out.as_mut().unwrap();
                eo.plan = EngineOutPlan::LandAhead;
                eo.track = Self::field_track(ac, wind, tr);
                self.log(ac, "FORCED_LANDING", Some("Runway no longer reachable".into()));
                self.set_phase(ac, Phase::ForcedLanding);
            }
        }
    }

    /// Track for a field landing: ahead, but turned toward the wind when there is height to spare (any
    /// amount above 700 ft, up to 90° from 400 ft, 30° lower down), so the touchdown is into wind rather
    /// than in a crosswind the rudder cannot straighten. Uses the estimated wind.
    fn field_track(ac: &Aircraft, wind: &Wind, track: f64) -> f64 {
        if wind.v.x.hypot(wind.v.y) < 8.0 * KT || ac.on_ground {
            return track;
        }
        // math angle the wind comes from
        let into = (-wind.v.y).atan2(-wind.v.x);
        let lim = if ac.h > 700.0 * FT { PI } else if ac.h > 400.0 * FT { FIELD_TURN_MAX * D2R } else { 30.0 * D2R };
        wrap_pi(track + clamp(wrap_pi(into - track), -lim, lim))
    }

    /// Power-off final. One energy decision, the drag ladder: of the levels allowed here (flap beyond 10°
    /// only once lined up or low; the slip only when aligned and above the slip floor; no flap raised
    /// near the ground), take the one whose straight glide touches down nearest the planned aim point
    /// without falling short of the runway. When short, that is the lowest drag (flap comes
    /// up); when even the most drag lands well beyond the farthest usable aim point, plan a longer path.
    fn eo_final(&mut self, ac: &Aircraft, wind: &Wind, dt: f64, c: &mut Commands, s: &crate::airfield::CircuitSpec) {
        let g = self.glide.clone().unwrap();
        let end = g.end;
        let w = wind.v;
        let trk_err = wrap_pi(ac.track(wind) - end.heading).abs();
        let lined_up = trk_err < 30.0 * D2R && ac.y.abs() < 150.0;
        let low = ac.h < 300.0 * FT;
        let aligned = trk_err < SLIP_ALIGN_TRK * D2R && ac.y.abs() < SLIP_ALIGN_Y;
        // touch down nearest the planned aim point, never short of the runway's first `aim_min`
        let far = self.planner.far_aim(w, &end).max(g.aim);
        let (d_near, d_nom, d_far) = (
            self.planner.to_point(ac.x, self.planner.spec.aim_min, &end),
            self.planner.to_point(ac.x, g.aim, &end),
            self.planner.to_point(ac.x, far, &end),
        );
        let current = LADDER[self.drag];
        let allowed = |l: &(f64, bool)| {
            (l.0 <= 10.0 || lined_up || low)
                && (!l.1 || (aligned && ac.h > SLIP_FLOOR * FT))
                && (l.0 >= current.0 || ac.h > RETRACT_FLOOR * FT)
        };
        // touchdown distance ahead for a level: the glide's reach plus the hold-off float from its speed
        let reach = |l: &(f64, bool)| {
            let v = if l.0 >= 20.0 { self.planner.spec.vapp_eo } else { self.planner.spec.vbg };
            ac.h / self.planner.gradient(ac, w, l.0, if l.1 { SLIP_SINK_GAIN } else { 1.0 }, &end) + self.planner.float_distance(ac, v, l.0, w, &end)
        };
        let mut best: Option<(usize, f64)> = None;
        for (i, l) in LADDER.iter().enumerate() {
            if !allowed(l) || reach(l) < d_near {
                continue;
            }
            // a small preference for staying put, so the level does not flicker between two
            let err = (reach(l) - d_nom).abs() * if i == self.drag { 0.9 } else { 1.0 };
            if best.map_or(true, |b| err < b.1) {
                best = Some((i, err));
            }
        }
        let lowest = LADDER.iter().position(|l| allowed(l)).unwrap_or(self.drag);
        self.drag = best.map(|b| b.0).unwrap_or(lowest);
        let (flap, slip) = LADDER[self.drag];
        self.eo_flaps = flap;
        // far too high even with the most drag: back to planning, which finds a longer path to a fix
        let most = LADDER.iter().filter(|l| allowed(l)).map(|l| reach(l)).fold(f64::INFINITY, f64::min);
        if ac.h > REPLAN_FLOOR * FT && d_far > 300.0 && most > d_far * (1.0 + REPLAN_LONG) {
            let fresh = self.planner.plan(ac, w, Some(&end));
            if fresh.reachable && fresh.ip_distance > self.planner.to_point(ac.x, 0.0, &end) {
                self.log(ac, "EXTEND", Some("High on final: a longer path".into()));
                self.glide = Some(fresh);
                self.eo_path = None;
                self.set_phase(ac, Phase::EoGlide);
                return;
            }
        }
        c.thr = 0.0;
        c.flaps = flap;
        let v = if flap >= 20.0 { self.planner.spec.vapp_eo + self.gust_add_eo() } else { self.planner.spec.vbg };
        c.theta = self.glide_pitch(ac, v, dt);
        self.fma = Fma::new("ENG OUT", &format!("GLIDE F{}{}", flap as i32, if slip { " SLIP" } else { "" }), &format!("LOC {}", end.name));
        let _ = d_far;
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
            c.phi = clamp(ac.wing_low_bank() + self.line_bank(ac, wind, &end, SLIP_LINE_BANK * D2R), -12.0 * D2R, 12.0 * D2R);
            self.fma.lat = "SLIP".into();
        } else {
            let bank = if ac.h < NEAR_GROUND * FT {
                10.0 * D2R
            } else if trk_err > 60.0 * D2R {
                s.bank_max.max(self.planner.spec.turn_bank)
            } else {
                s.bank_max
            };
            c.phi = self.line_bank(ac, wind, &end, bank);
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn flare_sink_falls_with_height_and_rises_as_speed_bleeds() {
        let (v_td, floor, k) = (25.0, 0.4, 0.5);
        // lower is gentler
        for v in [24.0, 26.0, 30.0] {
            let mut last = f64::INFINITY;
            for i in 0..=20 {
                let h = 10.0 - 0.5 * i as f64;
                let s = flare_sink(h, v, v_td, floor, k);
                assert!(s <= last + 1e-12 && s >= 0.0);
                last = s;
            }
        }
        // at the hold-off height: no sink while fast, the floor once at the touchdown speed
        assert!(flare_sink(HOLD_OFF_HEIGHT, v_td + HOLD_OFF_BAND * KT + 1.0, v_td, floor, k).abs() < 1e-12);
        assert!((flare_sink(HOLD_OFF_HEIGHT, v_td, v_td, floor, k) - floor).abs() < 1e-12);
        // monotone in speed: slower settles more
        assert!(flare_sink(1.0, 26.0, v_td, floor, k) >= flare_sink(1.0, 28.0, v_td, floor, k));
    }
}
