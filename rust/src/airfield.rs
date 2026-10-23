//! Runway 09/27 geometry and the traffic-pattern (circuit) definitions.

use crate::units::{Point, D2R, FT, KT, PI};
use serde::Serialize;

/// Runway 09/27. Threshold of 09 at x = 0, heading 090 along +x, centreline y = 0.
#[derive(Clone, Debug, Serialize)]
pub struct Runway {
    pub length: f64,
    pub width: f64,
    /// Glidepath aim point, metres past the threshold.
    pub aim_point: f64,
    pub glideslope: f64,
}

impl Runway {
    pub fn new(length: f64, width: f64, aim_point: f64, glideslope: f64) -> Self {
        Runway { length, width, aim_point, glideslope }
    }
    /// Height of the glidepath above the runway at along-track position x.
    pub fn glidepath_height(&self, x: f64) -> f64 {
        ((self.aim_point - x) * self.glideslope.tan()).max(0.0)
    }
    /// Angular deviation from the glidepath, degrees (positive = high).
    pub fn glidepath_deviation_deg(&self, x: f64, h: f64) -> f64 {
        let d = (self.aim_point - x).max(1.0);
        (h.atan2(d) - self.glideslope) / D2R
    }
    pub fn contains(&self, x: f64, y: f64) -> bool {
        x >= 0.0 && x <= self.length && y.abs() <= self.width / 2.0
    }
}

/// Left-hand traffic pattern flown to the north of runway 09.
#[derive(Clone, Debug, Serialize)]
pub struct CircuitSpec {
    /// m north of the centreline.
    pub downwind_offset: f64,
    /// m, along-track position of the base leg (negative = before threshold).
    pub base_x: f64,
    pub pattern_alt: f64,
    pub crosswind_turn_alt: f64,
    pub flaps_up_alt: f64,
    pub vr: f64,
    pub vy: f64,
    pub vdownwind: f64,
    pub vdescent: f64,
    pub vbase: f64,
    pub vapp: f64,
    pub bank_max: f64,
    pub descent_vs: f64,
    pub base_floor: f64,
    pub flare_height: f64,
    /// Height below which the crab is taken out (wing-low), and the bank allowed while doing it.
    pub align_height: f64,
    pub align_bank_max: f64,
    /// Minimum distance beyond the departure end before turning crosswind.
    pub crosswind_clearance: f64,
    /// Final approach path angle.
    pub approach_angle: f64,
    /// true: hold pattern altitude on downwind and descend only when the approach path meets it.
    /// false: start a fixed descent abeam the threshold.
    pub profile_descent: bool,
    pub name: &'static str,
}

pub fn standard_circuit() -> CircuitSpec {
    CircuitSpec {
        downwind_offset: 1200.0,
        base_x: -2600.0,
        pattern_alt: 1000.0 * FT,
        crosswind_turn_alt: 500.0 * FT,
        flaps_up_alt: 300.0 * FT,
        vr: 55.0 * KT,
        vy: 65.0 * KT,
        vdownwind: 90.0 * KT,
        vdescent: 75.0 * KT,
        vbase: 70.0 * KT,
        vapp: 65.0 * KT,
        bank_max: 25.0 * D2R,
        descent_vs: -2.5,
        base_floor: 150.0,
        flare_height: 4.8,
        align_height: 60.0 * FT,
        align_bank_max: 10.0 * D2R,
        crosswind_clearance: 400.0,
        approach_angle: 3.0 * D2R,
        profile_descent: false,
        name: "Standard",
    }
}

/// High circuit: tighter pattern, pattern altitude held until the descent point, then a 5.5° path flown
/// along base and final. 5.5° is close to the clean power-off glide angle, so the runway stays within
/// gliding reach for much more of the circuit.
pub fn high_circuit() -> CircuitSpec {
    CircuitSpec {
        downwind_offset: 1000.0,
        base_x: -1500.0,
        approach_angle: 5.5 * D2R,
        profile_descent: true,
        name: "High",
        ..standard_circuit()
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Leg {
    Upwind,
    Crosswind,
    Downwind,
    Base,
    Final,
}

#[derive(Clone, Debug)]
pub struct Circuit {
    pub spec: CircuitSpec,
    pub runway: Runway,
    /// Set once the aircraft decides where to turn crosswind; None until then.
    pub crosswind_x: Option<f64>,
}

impl Circuit {
    pub fn new(spec: CircuitSpec, runway: Runway) -> Self {
        Circuit { spec, runway, crosswind_x: None }
    }

    pub fn leg_ends(&self, leg: Leg) -> (Point, Point) {
        let cx = self.crosswind_x.unwrap_or(2300.0);
        let s = &self.spec;
        let dy = s.downwind_offset;
        match leg {
            Leg::Upwind => ([0.0, 0.0], [20000.0, 0.0]),
            Leg::Crosswind => ([cx, 0.0], [cx, dy]),
            Leg::Downwind => ([cx, dy], [s.base_x, dy]),
            Leg::Base => ([s.base_x, dy], [s.base_x, 0.0]),
            Leg::Final => ([s.base_x, 0.0], [self.runway.length, 0.0]),
        }
    }

    /// Distance still to fly along the circuit to the aim point, from the downwind or base leg.
    pub fn remaining_to_aim(&self, on_downwind: bool, x: f64, y: f64, turn_radius: f64) -> f64 {
        let s = &self.spec;
        let fin = self.runway.aim_point - s.base_x;
        // each 90° turn still ahead cuts its corner by (2 − π/2)·R
        let cut = (2.0 - PI / 2.0) * turn_radius;
        if on_downwind {
            (x - s.base_x).max(0.0) + s.downwind_offset + fin - 2.0 * cut
        } else {
            y.max(0.0) + fin - if y > turn_radius { cut } else { 0.0 }
        }
    }

    pub fn profile_height(&self, on_downwind: bool, x: f64, y: f64, turn_radius: f64) -> f64 {
        self.remaining_to_aim(on_downwind, x, y, turn_radius).max(0.0) * self.spec.approach_angle.tan()
    }

    /// Planned route for display, from brake release to the aim point.
    pub fn planned_route(&self) -> Vec<Point> {
        let cx = self.crosswind_x.unwrap_or(2300.0);
        let s = &self.spec;
        vec![
            [0.0, 0.0],
            [cx, 0.0],
            [cx, s.downwind_offset],
            [s.base_x, s.downwind_offset],
            [s.base_x, 0.0],
            [self.runway.aim_point, 0.0],
        ]
    }
}

/// Take-off and landing technique, independent of the circuit shape.
#[derive(Clone, Debug, Serialize)]
pub struct Procedure {
    pub name: &'static str,
    pub short_field: bool,
    /// Rotation speed and pitch.
    pub vr: f64,
    pub rotate_pitch: f64,
    /// Best-angle climb speed, flown until `vx_until` height (then the normal Vy climb). None = Vy from lift-off.
    pub vx: Option<f64>,
    pub vx_until: f64,
    /// Hold the brakes until full power before releasing.
    pub hold_brakes: bool,
    pub vapp: f64,
    pub approach_angle: f64,
    /// Glidepath aim point, metres past the threshold.
    pub aim_point: f64,
    /// Sink rate the flare settles to just above the runway, m/s (higher = firmer, shorter float).
    pub flare_floor: f64,
    /// Start the flare this many seconds of sink above the runway.
    pub flare_lead: f64,
    /// Delay from touchdown to full braking, s.
    pub brake_delay: f64,
    /// Go around if the wheels are not down by this point, metres past the threshold.
    pub long_landing_limit: f64,
}

/// Normal technique: rotate at 55 kt, Vy climb, 65 kt approach on the circuit's path, aim a quarter of the way in.
pub fn normal_procedure(circuit: &CircuitSpec, runway_length: f64) -> Procedure {
    Procedure {
        name: "Normal",
        short_field: false,
        vr: circuit.vr,
        rotate_pitch: 8.0 * D2R,
        vx: None,
        vx_until: 0.0,
        hold_brakes: false,
        vapp: circuit.vapp,
        approach_angle: circuit.approach_angle,
        aim_point: (runway_length / 4.0).min(250.0),
        flare_floor: 0.4,
        flare_lead: 2.7,
        brake_delay: 2.0,
        long_landing_limit: runway_length / 3.0 + 50.0,
    }
}

/// Short-field technique (C172 POH style): flap 10, brakes held to full power, rotate at 48 kt with more pitch,
/// best-angle climb at 57 kt to 100 ft; steep 5.5° power-on approach at 60 kt aimed 40 m past the threshold,
/// firm touchdown without float, flaps up and maximum braking straight away.
pub fn short_field_procedure(circuit: &CircuitSpec, _runway_length: f64) -> Procedure {
    Procedure {
        name: "Short-field",
        short_field: true,
        vr: 48.0 * KT,
        rotate_pitch: 10.0 * D2R,
        vx: Some(57.0 * KT),
        vx_until: 100.0 * FT,
        hold_brakes: true,
        vapp: 60.0 * KT,
        approach_angle: circuit.approach_angle.max(5.5 * D2R),
        aim_point: 40.0,
        flare_floor: FLARE_FLOOR_SHORT,
        flare_lead: FLARE_LEAD_SHORT,
        brake_delay: 0.5,
        long_landing_limit: 40.0 + 130.0,
    }
}

/// 50 ft obstacles (trees) this far beyond each end of a short strip.
pub const OBSTACLE_OFFSET: f64 = 150.0;
pub const OBSTACLE_HEIGHT: f64 = 50.0 * FT;

pub const FLARE_FLOOR_SHORT: f64 = 0.7;
pub const FLARE_LEAD_SHORT: f64 = 2.3;
