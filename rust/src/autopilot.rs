//! Mode manager (phase state machine) plus the outer loops it switches between, go-around and
//! engine-out logic. Each phase sets targets; the controllers turn them into attitude/throttle
//! commands; the aircraft's `CommandShaper` rate-limits whatever comes out.

use crate::aircraft::{Aircraft, Commands};
use crate::airfield::{normal_procedure, Circuit, Leg, Procedure, Runway};
use crate::atmosphere::Wind;
use crate::control::{AutoThrottle, FlightPathController, GustEstimator, SpeedOnPitch, TrackGuidance};
use crate::emergency::{default_eo, GlideAssessment, GlidePlanner, RunwayEnd};
use crate::tuning::Tuning;
use crate::units::{clamp, wrap_pi, Point, D2R, FT, G, KT, PI};
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


/// Largest turn at the base key for which the turn cuts the corner (fly-by), rad.
const KEY_CUT_MAX: f64 = 2.1;
/// Largest centreline offset for starting or holding a slip, m.
const SLIP_ALIGN_Y: f64 = 60.0;
/// C172P flap limit speeds, KIAS: 10° flap, and beyond 10°.
const VFE_10: f64 = 110.0;
const VFE_FULL: f64 = 85.0;
/// Bank limit: airspeed over the stall speed at the turn's load factor.
const BANK_STALL_MARGIN: f64 = 1.2;
/// No flap retraction below this height, ft.
const RETRACT_FLOOR: f64 = 100.0;
/// Bank always allowed, deg.
const BANK_FLOOR: f64 = 15.0;
/// Best-glide wind correction: fraction of the headwind above the dead band (kt) added to the speed.
const VBG_HEAD_K: f64 = 0.0;
const VBG_HEAD_DEADBAND: f64 = 5.0;
/// Try a margin-free short final to any part of the runway before giving up for a field.
const STRETCH: bool = true;
/// Planned height surplus (m, after margins) above which flap 10 goes out on the way in (×2: flap 20).
const EARLY_FLAP_SURPLUS: f64 = 100.0;
/// Largest turn toward the wind for a field landing, deg.
const FIELD_TURN_MAX: f64 = 90.0;
/// Lowest height for a forward slip, ft (below it the crosswind alignment law takes over).
const SLIP_FLOOR: f64 = 100.0;
/// Sink-rate gain of a full-rudder forward slip in this model (`examples/slip_test.rs`: ×1.25–1.3).
const SLIP_SINK_GAIN: f64 = 1.25;
/// Bank allowed for holding the centreline while slipping, deg.
const SLIP_LINE_BANK: f64 = 6.0;
const S_TURN_MIN_DIST: f64 = 500.0;
const S_TURN_WIDTH: f64 = 150.0;
/// S-turns start when the required gradient exceeds the full-flap glide by this factor.
const S_TURN_RATIO: f64 = 1.4;
/// Power-off centreline capture distance, as a fraction of the ground-speed turn radius.
const LINE_CAPTURE_K: f64 = 0.8;

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
    /// S-turn direction on a high power-off final (+1 / −1 about the centreline, 0 = not weaving).
    s_turn: f64,
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
            s_turn: 0.0,
            last_assess: 0.0,
            flare_k: 0.28,
            gust: GustEstimator::default(),
            tune: Tuning::default(),
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
        } else if let Some(g) = STRETCH.then(|| self.planner.stretch(ac, wind, None)).flatten() {
            let name = g.end.name;
            self.glide = Some(g);
            (EngineOutPlan::Return, format!("Stretching to runway {}: any part of it beats a field", name))
        } else if let Some(e) = self.planner.ends.iter().copied().find(|e| {
            ac.y.abs() < 100.0 && wrap_pi(track - e.heading).abs() < 30.0 * D2R && e.dir * (e.threshold - ac.x) > 0.0
        }) {
            // lined up on final: landing ahead and stretching for the runway are the same direction, so go
            // for the runway (centreline, flap up for the flatter glide); short of it is the field ahead anyway
            self.glide = Some(self.planner.plan_for(ac, wind, &e));
            (EngineOutPlan::Return, format!("On final: straight ahead for runway {}, flap up to stretch", e.name))
        } else {
            (EngineOutPlan::LandAhead, "Both runway directions beyond glide range".to_string())
        };
        self.engine_out =
            Some(EngineOut { plan, why: why.clone(), t: self.t, x: ac.x, y: ac.y, h: ac.h, track: if plan == EngineOutPlan::LandAhead { Self::field_track(ac, wind, track) } else { track } });
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
            self.gust.update(ac, wind, dt);
        }
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
                // lull protection: in turbulence, if airspeed sags below 1.2 Vs in the flare, add power
                // to hold it (as a pilot would) instead of running out of elevator into a hard arrival
                let deficit = self.tune.lull_vs * ac.stall_speed(ac.flap_pos) - ac.v;
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
                let theta = self.path.vertical_speed(ac, -(floor + self.flare_k * ac.h), dt);
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
            Phase::EoGlide => self.eo_glide(ac, wind, t, dt, &mut c, &s),
            Phase::EoFinal => self.eo_final(ac, wind, dt, &mut c, &s),
            Phase::ForcedLanding => {
                let vbg = self.vbg_wind(ac, wind);
                let vapp_eo = self.planner.spec.vapp_eo;
                let tr = self.engine_out.as_ref().unwrap().track;
                self.fma = Fma::new("ENG OUT", "GLIDE", "TRK HOLD");
                c.thr = 0.0;
                c.flaps = if ac.h < 300.0 * FT { 20.0 } else { 0.0 };
                let gust_add = self.gust_add_eo();
                c.theta = self.glide_pitch(ac, Self::turn_speed(if c.flaps >= 20.0 { vapp_eo + gust_add } else { vbg }, ac.shaper.phi), dt);
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
        self.cmd = c;
        c
    }

    /// Best glide, clean: direct to a base key point, down a short base leg, then turn final at the intercept.
    fn eo_glide(&mut self, ac: &Aircraft, wind: &Wind, t: f64, dt: f64, c: &mut Commands, s: &crate::airfield::CircuitSpec) {
        let end = self.eo_end();
        let ip_d = self.glide.as_ref().unwrap().ip_distance;
        let wp = self.planner.waypoints(ac, wind, ip_d, &end);
        self.fma = Fma::new("ENG OUT", "GLIDE", &format!("DIR IP {}", end.name));
        c.thr = 0.0;
        // flap early when the plan shows height to burn, once heading for the intercept (never on the way
        // to a key behind, where the energy may still be needed); it is never retracted
        let surplus = self.glide.as_ref().map(|g| g.surplus).unwrap_or(0.0);
        let committed = wp.base_key.is_none() || self.passed_base_key;
        if committed {
            if surplus > EARLY_FLAP_SURPLUS * 2.0 {
                self.eo_flaps = self.eo_flaps.max(20.0);
            } else if surplus > EARLY_FLAP_SURPLUS {
                self.eo_flaps = self.eo_flaps.max(10.0);
            }
        }
        c.flaps = self.eo_flaps;
        let v_glide = if self.eo_flaps >= 20.0 { self.planner.spec.vapp_eo + self.gust_add_eo() } else { self.vbg_wind(ac, wind) };
        // pitch for the glide speed now, from the previous bank; corrected for this step's bank below
        c.theta = self.glide_pitch(ac, Self::turn_speed(v_glide, ac.shaper.phi), dt);
        let gv = ac.ground_velocity(wind);
        let vg = gv.x.hypot(gv.y);
        let r = (vg * vg) / (G * s.bank_max.tan());
        let roll_in = vg * (s.bank_max / ac.shaper.rates.phi) * 0.5;
        // once past the base key (or going direct to the intercept): if a turn onto final started now
        // would only just reach the centreline (wind drift and any roll reversal included), start it now
        let heading_for_key = wp.base_key.is_some() && !self.passed_base_key;
        if !heading_for_key && self.final_turn_due(ac, wind, &end) {
            self.set_phase(ac, Phase::EoFinal);
            return;
        }
        match wp.base_key {
            Some([bx, by]) if !self.passed_base_key => {
                let brg = (by - ac.y).atan2(bx - ac.x);
                let dist = (bx - ac.x).hypot(by - ac.y);
                let base_brg = (wp.ip[1] - by).atan2(wp.ip[0] - bx);
                // no corner-cutting beyond a 120° turn at the key
                let lead = r * (wrap_pi(base_brg - brg).abs().min(KEY_CUT_MAX) / 2.0).tan() + roll_in;
                self.fma.lat = "DIR KEY".into();
                c.phi = self.guidance.bank_for(ac, wind, brg, Self::eo_turn_bank(ac, wind, brg, s.bank_max, self.tune.eo_turn_bank));
                if dist < lead || end.dir * (ac.x - bx) < 0.0 {
                    self.passed_base_key = true;
                }
            }
            Some(key) => {
                let g = self.guidance.follow(ac, key, wp.ip);
                self.fma.lat = "BASE".into();
                c.phi = self.guidance.bank_for(ac, wind, g.desired_track, s.bank_max);
                // turn onto final when the turn, drift included, will just reach the centreline
                let base_dir = (wp.ip[1] - key[1]).atan2(wp.ip[0] - key[0]);
                if g.remaining < self.planner.turn_reach(ac.v, wind.along(base_dir)) {
                    self.set_phase(ac, Phase::EoFinal);
                    return;
                }
            }
            None => {
                let [ipx, ipy] = wp.ip;
                let brg = (ipy - ac.y).atan2(ipx - ac.x);
                let dist = (ipx - ac.x).hypot(ipy - ac.y);
                let lead = r * (wrap_pi(brg - end.heading).abs().min(2.5) / 2.0).tan() + roll_in;
                c.phi = self.guidance.bank_for(ac, wind, brg, Self::eo_turn_bank(ac, wind, brg, s.bank_max, self.tune.eo_turn_bank));
                // inside the intercept point near the line counts as on final only when roughly heading down it;
                // heading away (failure just after the crosswind turn began), go round to the intercept point
                let aligned = wrap_pi(ac.track(wind) - end.heading).abs() < 75.0 * D2R;
                // the turn lead only counts once the aircraft is actually flying toward the intercept point
                let toward_ip = wrap_pi(brg - ac.track(wind)).abs() < 60.0 * D2R;
                if (dist < lead && toward_ip) || (end.dir * (ac.x - ipx) > 0.0 && ac.y.abs() < 150.0 && aligned) {
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
            let mut lost = g.available < g.required;
            self.glide = Some(g);
            if lost && STRETCH {
                if let Some(st) = self.planner.stretch(ac, wind, Some(&end)) {
                    if st.end.name != end.name || st.ip_distance != ip_d {
                        self.log(ac, "STRETCH", Some(format!("Short final to runway {}: any part will do", st.end.name)));
                    }
                    self.glide = Some(st);
                    lost = false;
                }
            }
            if lost {
                let tr = ac.track(wind);
                let eo = self.engine_out.as_mut().unwrap();
                eo.plan = EngineOutPlan::LandAhead;
                eo.track = Self::field_track(ac, wind, tr);
                self.log(ac, "FORCED_LANDING", Some("Runway no longer reachable".into()));
                self.phase = Phase::ForcedLanding;
            }
        }
    }

    /// Best-glide speed for the wind: in a headwind fly faster (about half the headwind component, up to
    /// +12 kt) to cover more ground per foot of height; in a tailwind best glide is already close to
    /// minimum sink in this model, so the speed stays. From `examples/glide_speed.rs` (6DOF trim).
    fn vbg_wind(&self, ac: &Aircraft, wind: &Wind) -> f64 {
        let head = -wind.along(ac.track(wind));
        self.planner.spec.vbg + (VBG_HEAD_K * (head - VBG_HEAD_DEADBAND * KT).max(0.0)).min(12.0 * KT)
    }

    /// Bank for power-off turns: a big heading change (a turn back) is flown steeper, which loses less
    /// height per degree of turn; small corrections keep the circuit bank.
    /// Glide speed in a turn: the best-glide α is held, so the speed rises with √(load factor); the nose
    /// goes down as the bank goes on, and the stall-margin bank limit then allows the bank the turn needs.
    fn turn_speed(v: f64, phi: f64) -> f64 {
        v / phi.abs().min(50.0 * D2R).cos().sqrt()
    }
    fn eo_turn_bank(ac: &Aircraft, wind: &Wind, brg: f64, circuit_bank: f64, turn_bank_deg: f64) -> f64 {
        let gv = ac.ground_velocity(wind);
        if wrap_pi(brg - gv.y.atan2(gv.x)).abs() > 60.0 * D2R { circuit_bank.max(turn_bank_deg * D2R) } else { circuit_bank }
    }

    /// Track for a field landing: ahead, but turned toward the wind when there is height to spare (any
    /// amount above 700 ft, up to 90° from 400 ft, 30° lower down), so the touchdown is into wind rather
    /// than in a crosswind the rudder cannot straighten.
    fn field_track(ac: &Aircraft, wind: &Wind, track: f64) -> f64 {
        let ws = wind.settings.speed_kt;
        if ws < 8.0 || ac.on_ground {
            return track;
        }
        // math angle the wind comes from (heading_deg is compass; wind from N blows toward −y)
        let into = wrap_pi(PI / 2.0 - wind.settings.from_deg * D2R);
        let lim = if ac.h > 700.0 * FT { PI } else if ac.h > 400.0 * FT { FIELD_TURN_MAX * D2R } else { 30.0 * D2R };
        wrap_pi(track + clamp(wrap_pi(into - track), -lim, lim))
    }

    /// Would a turn onto the final line, begun now at the circuit bank limit, carry the aircraft through
    /// the centreline? Air-mass turn geometry plus the wind's drift over the turn and the roll-in. Only
    /// counts when the aircraft is outside the intercept region and closing on (or turning across) the line.
    fn final_turn_due(&self, ac: &Aircraft, wind: &Wind, end: &RunwayEnd) -> bool {
        let bank = self.circuit.spec.bank_max;
        let rate = ac.shaper.rates.phi;
        let gv = ac.ground_velocity(wind);
        let chi = gv.y.atan2(gv.x);
        // the line's normal, pointing from the centreline to the aircraft
        let side = if ac.y >= 0.0 { 1.0 } else { -1.0 };
        let dist = ac.y.abs();
        if dist < 30.0 || self.planner.to_aim(ac.x, end) < 300.0 {
            return false;
        }
        // heading relative to the final direction, and the velocity component toward the line
        let theta = wrap_pi(chi - end.heading);
        let closing = -side * gv.y;
        if closing <= 0.0 || theta.abs() < 20.0 * D2R {
            return false;
        }
        let v = ac.v.max(20.0);
        let r = v * v / (G * bank.tan());
        // turning from θ to 0: air-mass displacement toward the line
        let d_air = r * (1.0 - theta.cos());
        let t_turn = theta.abs() * r / v;
        // roll from the current bank to the bank of the turn toward the line
        let turn_sign = -theta.signum(); // + = left (CCW)
        let t_roll = (turn_sign * bank - ac.phi).abs() / rate;
        let w_toward = -side * wind.v.y;
        let reach = d_air + w_toward * t_turn + closing * t_roll * 0.8 + 50.0;
        reach >= dist
    }

    /// Power-off final: add flap in stages when still high (never retract), forward-slip if even full flap
    /// will not get it down, straighten for touchdown below 60 ft.
    fn eo_final(&mut self, ac: &Aircraft, wind: &Wind, dt: f64, c: &mut Commands, s: &crate::airfield::CircuitSpec) {
        let end = self.eo_end();
        // any part of the runway will do: when high, move the aim point down the runway (up to where a
        // flare and stop still fit) before reaching for slips and S-turns
        let nominal = self.planner.to_aim(ac.x, &end);
        let g20 = self.planner.gradient(ac, wind, 20.0, &end);
        let along_more = (ac.h / (g20 * 1.05) - nominal).clamp(0.0, (self.planner.far_aim_distance() - self.planner.spec.aim_point).max(0.0));
        let dist = nominal + along_more;
        let greq = if dist > 30.0 { ac.h / dist } else { f64::INFINITY };
        // Energy, in the order a pilot would use it when high:
        //  1. flap in stages to 20° (beyond 10° only once roughly lined up, so the turn uses the flatter glide)
        //  2. a forward slip (cross-controls) at flap 20 — the C172 POH advises against steep slips with
        //     more than 20°; in this model a full-rudder slip adds about 25% to the sink rate
        //  3. S-turns if even that will not get it down
        //  4. flap 30 only when not slipping (no longer high, or below the slip floor)
        let lined_up = wrap_pi(ac.track(wind) - end.heading).abs() < 30.0 * D2R && ac.y.abs() < 150.0;
        // still turning round onto final (failure just after the crosswind turn began): the straight-line
        // energy sums do not apply yet, so no more flap goes out until the turn is done
        let reversing = wrap_pi(ac.track(wind) - end.heading).abs() > 90.0 * D2R && ac.h > 300.0 * FT;
        let low = ac.h < 300.0 * FT;
        // clearly high: flap 20 even before lined up (the energy is not needed for the turn)
        let clearly_high = greq > self.planner.gradient(ac, wind, 20.0, &end) * 1.2;
        let flap_cap = if lined_up || low || clearly_high { 20.0 } else { 10.0 };
        let flap_cap = if reversing { 0.0 } else { flap_cap };
        // short of the runway with flap out: raise it in stages for the flatter clean glide (not close to
        // the ground, where the sink while it comes up would cost more than it gains)
        // "short" means short of the runway itself (60 m past the threshold), not of the normal aim point
        let to_thr = end.dir * (end.threshold - ac.x) + 60.0;
        let greq_thr = if to_thr > 30.0 { ac.h / to_thr } else { f64::INFINITY };
        while self.eo_flaps > 0.0 && lined_up && ac.h > RETRACT_FLOOR * FT && greq_thr < self.planner.gradient(ac, wind, self.eo_flaps, &end) * 0.95 {
            self.eo_flaps -= 10.0;
        }
        while self.eo_flaps < flap_cap && greq > self.planner.gradient(ac, wind, self.eo_flaps + 10.0, &end) * 1.05 {
            self.eo_flaps += 10.0;
        }
        // the slip itself waits until properly lined up: it has little bank left to correct the track
        let aligned = wrap_pi(ac.track(wind) - end.heading).abs() < 15.0 * D2R && ac.y.abs() < SLIP_ALIGN_Y;
        let slip = aligned && self.eo_flaps >= 20.0 && ac.h > SLIP_FLOOR * FT && greq > g20 * 1.05;
        if !slip && (lined_up || low) && self.eo_flaps < 30.0 && greq > self.planner.gradient(ac, wind, 30.0, &end) * 0.95 {
            self.eo_flaps = 30.0;
        }
        c.thr = 0.0;
        c.flaps = self.eo_flaps;
        let gust_add = self.gust_add_eo();
        // with the C172P drag a power-off flare needs the full 66 kt, short field or not (short-field
        // technique still brakes at once)
        let vapp_eo = self.planner.spec.vapp_eo;
        let v = if self.eo_flaps >= 20.0 { vapp_eo + gust_add } else { self.vbg_wind(ac, wind) };
        c.theta = self.glide_pitch(ac, v, dt);
        self.fma = Fma::new("ENG OUT", &format!("GLIDE F{}", self.eo_flaps as i32), &format!("LOC {}", end.name));
        // far too high even for a slip (an early turn in, or a strong tailwind): S-turns across the
        // centreline lengthen the path; they stop once flap and a slip can manage the rest
        let g_slip = g20 * SLIP_SINK_GAIN;
        let weave = ac.h > 350.0 * FT && dist > S_TURN_MIN_DIST && greq > g_slip * S_TURN_RATIO;
        if !weave {
            self.s_turn = 0.0;
        }
        if ac.h < s.align_height {
            let line = self.end_line(&end);
            self.align(ac, wind, c, end.heading, Some(line));
        } else if weave {
            if self.s_turn == 0.0 {
                self.s_turn = if ac.y >= 0.0 { -1.0 } else { 1.0 };
            }
            // swing to the other side once 150 m across
            if ac.y * self.s_turn > S_TURN_WIDTH {
                self.s_turn = -self.s_turn;
            }
            let track = end.heading + end.dir * self.s_turn * 45.0 * D2R;
            c.phi = self.guidance.bank_for(ac, wind, track, s.bank_max);
            self.fma.lat = "S-TURN".into();
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
            let bank = if ac.h < 30.0 { 10.0 * D2R } else if reversing { s.bank_max.max(self.tune.eo_turn_bank * D2R) } else { s.bank_max };
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
