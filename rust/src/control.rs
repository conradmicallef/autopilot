//! Outer loops shared by both flight models: PI controllers, lateral track guidance,
//! flight-path/vertical-speed/altitude holds, auto-throttle and speed-on-pitch.

use crate::aircraft::Aircraft;
use crate::atmosphere::Wind;
use crate::units::{clamp, wrap_pi, Point, D2R, G, PI};

/// Proportional-integral controller: out = i + kp·e, with a clamped integrator.
#[derive(Clone, Debug)]
pub struct Pi {
    pub i: f64,
    pub kp: f64,
    pub ki: f64,
    pub i_min: f64,
    pub i_max: f64,
    pub out: (f64, f64),
}

impl Pi {
    pub fn new(kp: f64, ki: f64, i_min: f64, i_max: f64, out: (f64, f64)) -> Self {
        Pi { i: 0.0, kp, ki, i_min, i_max, out }
    }
    pub fn reset(&mut self, i: f64) {
        self.i = clamp(i, self.i_min, self.i_max);
    }
    /// Output from the current integrator, then integrate only if that does not deepen saturation.
    pub fn update_conditional(&mut self, e: f64, dt: f64) -> f64 {
        let u = self.i + self.kp * e;
        let (lo, hi) = self.out;
        if (u < hi || e < 0.0) && (u > lo || e > 0.0) {
            self.i = clamp(self.i + self.ki * e * dt, self.i_min, self.i_max);
        }
        clamp(u, lo, hi)
    }
}

pub struct FollowResult {
    /// Cross-track error, m (+ = left of track).
    pub cross_track: f64,
    /// Distance still to go along the leg, m.
    pub remaining: f64,
    pub desired_track: f64,
}

/// Lateral guidance: vector-field line following plus a bank command to fly the desired ground track.
#[derive(Clone, Debug)]
pub struct TrackGuidance {
    pub approach_angle: f64,
    pub gain_distance: f64,
    pub turn_time_const: f64,
}

impl Default for TrackGuidance {
    fn default() -> Self {
        TrackGuidance { approach_angle: 60.0 * D2R, gain_distance: 180.0, turn_time_const: 3.0 }
    }
}

impl TrackGuidance {
    pub fn follow(&self, ac: &Aircraft, a: Point, b: Point) -> FollowResult {
        self.follow_with(ac, a, b, self.gain_distance)
    }

    /// Line following with a custom capture distance (smaller = converges onto the line more tightly).
    pub fn follow_with(&self, ac: &Aircraft, a: Point, b: Point, gain_distance: f64) -> FollowResult {
        let dx = b[0] - a[0];
        let dy = b[1] - a[1];
        let l = dx.hypot(dy);
        let ux = dx / l;
        let uy = dy / l;
        let e = ux * (ac.y - a[1]) - uy * (ac.x - a[0]);
        let remaining = (b[0] - ac.x) * ux + (b[1] - ac.y) * uy;
        let desired_track = uy.atan2(ux) - self.approach_angle * (2.0 / PI) * (e / gain_distance).atan();
        FollowResult { cross_track: e, remaining, desired_track }
    }

    pub fn bank_for(&self, ac: &Aircraft, wind: &Wind, desired_track: f64, limit: f64) -> f64 {
        let gv = ac.ground_velocity(wind);
        let chi = gv.y.atan2(gv.x);
        let vg = gv.x.hypot(gv.y);
        clamp(((vg * wrap_pi(desired_track - chi)) / (G * self.turn_time_const)).atan(), -limit, limit)
    }

    /// Distance before a leg's end to start a 90° turn: radius plus roll-in distance.
    pub fn turn_lead(&self, ac: &Aircraft, wind: &Wind, bank_max: f64, roll_rate: f64) -> f64 {
        let gv = ac.ground_velocity(wind);
        let vg = gv.x.hypot(gv.y);
        (vg * vg) / (G * bank_max.tan()) + vg * (bank_max / roll_rate) * 0.5
    }
}

/// Vertical path control: flight-path angle → pitch, with V/S and altitude holds layered on top.
#[derive(Clone, Debug)]
pub struct FlightPathController {
    pub gamma_pi: Pi,
}

impl Default for FlightPathController {
    fn default() -> Self {
        FlightPathController { gamma_pi: Pi::new(0.8, 0.4, -0.08, 0.08, (f64::NEG_INFINITY, f64::INFINITY)) }
    }
}

impl FlightPathController {
    pub fn gamma(&mut self, ac: &Aircraft, g_cmd: f64, dt: f64) -> f64 {
        let err = g_cmd - ac.gamma;
        let p = &mut self.gamma_pi;
        p.i = clamp(p.i + p.ki * err * dt, p.i_min, p.i_max);
        clamp(ac.alpha_f + g_cmd + p.kp * err + p.i, ac.spec.min_pitch, ac.spec.max_pitch)
    }
    pub fn vertical_speed(&mut self, ac: &Aircraft, vs: f64, dt: f64) -> f64 {
        self.gamma(ac, clamp(vs / ac.v.max(10.0), -0.25, 0.25).asin(), dt)
    }
    pub fn altitude(&mut self, ac: &Aircraft, h_cmd: f64, dt: f64, vs_max: f64) -> f64 {
        self.vertical_speed(ac, clamp(0.15 * (h_cmd - ac.h), -vs_max, vs_max), dt)
    }
}

/// Auto-throttle: airspeed error → throttle, with conditional integration (anti-windup).
#[derive(Clone, Debug)]
pub struct AutoThrottle {
    pub pi: Pi,
}

impl Default for AutoThrottle {
    fn default() -> Self {
        AutoThrottle { pi: Pi::new(0.10, 0.03, 0.0, 1.0, (0.0, 1.0)) }
    }
}

impl AutoThrottle {
    /// Bumpless engage from the current throttle.
    pub fn engage(&mut self, current_thr: f64) {
        self.pi.reset(current_thr);
    }
    pub fn update(&mut self, ac: &Aircraft, v_target: f64, dt: f64) -> f64 {
        self.pi.update_conditional(v_target - ac.v, dt)
    }
}

/// Climb (or glide) holding airspeed with pitch: returns a flight-path angle command.
#[derive(Clone, Debug)]
pub struct SpeedOnPitch {
    pub pi: Pi,
}

impl SpeedOnPitch {
    pub fn new(kp: f64, ki: f64, i_min: f64, i_max: f64) -> Self {
        SpeedOnPitch { pi: Pi::new(kp, ki, i_min, i_max, (f64::NEG_INFINITY, f64::INFINITY)) }
    }
    pub fn engage(&mut self, gamma: f64) {
        self.pi.reset(gamma);
    }
    pub fn update(&mut self, ac: &Aircraft, v_target: f64, dt: f64, g_min: f64) -> f64 {
        let e = ac.v - v_target;
        let p = &mut self.pi;
        p.i = clamp(p.i + p.ki * e * dt, p.i_min, p.i_max);
        clamp(p.i + p.kp * e, g_min, 0.2)
    }
}

impl Default for SpeedOnPitch {
    fn default() -> Self {
        SpeedOnPitch::new(0.015, 0.004, -0.05, 0.2)
    }
}

/// Turbulence estimate from the measured wind, as a real autopilot gets it: ground velocity (GPS)
/// minus air velocity (airspeed along the flight path). Each wind component is high-passed (4 s)
/// and the spread of what remains is tracked (6 s), so a steady wind reads zero and gusts don't.
#[derive(Clone, Debug, Default)]
pub struct GustEstimator {
    w_lp: [f64; 2],
    var: f64,
    init: bool,
}

impl GustEstimator {
    pub fn update(&mut self, ac: &Aircraft, wind: &Wind, dt: f64) {
        let gv = ac.ground_velocity(wind);
        let ha = ac.v * ac.gamma.cos();
        let w = [gv.x - ha * ac.psi.cos(), gv.y - ha * ac.psi.sin()];
        if !self.init {
            self.w_lp = w;
            self.init = true;
        }
        let mut e2 = 0.0;
        for i in 0..2 {
            self.w_lp[i] += (w[i] - self.w_lp[i]) * dt / 4.0;
            e2 += (w[i] - self.w_lp[i]).powi(2);
        }
        self.var += (0.5 * e2 - self.var) * dt / 6.0;
    }
    /// Standard deviation of the gusts per horizontal axis, m/s (after a 0.2 m/s dead band).
    pub fn sigma(&self) -> f64 {
        (self.var.sqrt() - 0.2).max(0.0)
    }
    /// 0 in calm air, 1 in moderate turbulence (σ ≥ 1.2 m/s).
    pub fn level(&self) -> f64 {
        clamp(self.sigma() / 1.2, 0.0, 1.0)
    }
}
