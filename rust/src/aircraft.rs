//! The aircraft as the autopilot sees it, plus the point-mass flight model.
//!
//! `Aircraft` holds the state every part of the autopilot reads (position, airspeed, flight path,
//! attitude, configuration). Its dynamics come from one of two models:
//! * `ModelKind::PointMass` — the original model: point mass with first-order attitude response.
//! * `ModelKind::SixDof` — a rigid body with aerodynamic moments, control surfaces and landing gear
//!   (see `sixdof`). The autopilot commands are the same; the 6DOF model turns them into surface
//!   deflections with its own control laws.

use crate::atmosphere::Wind;
use crate::c172p;
use crate::sixdof::SixDof;
use crate::units::{clamp, slew, wrap_pi, Vec2, D2R, G, PI, RHO};
use serde::Serialize;

/// Static description of an airframe (shared by both models).
#[derive(Clone, Debug, Serialize)]
pub struct AircraftSpec {
    pub name: &'static str,
    pub mass: f64,
    pub wing_area: f64,
    /// W at the propeller (already × efficiency).
    pub power: f64,
    pub static_thrust: f64,
    pub rolling_mu: f64,
    pub brake_mu: f64,
    /// Side-force slope with sideslip, per rad (point-mass model).
    pub cy_beta: f64,
    /// Extra drag coefficient from a windmilling propeller after an engine failure.
    pub windmill_cd: f64,
    pub max_sideslip: f64,
    pub max_pitch: f64,
    pub min_pitch: f64,
    pub tail_strike_pitch: f64,
}

pub fn c172_like() -> AircraftSpec {
    AircraftSpec {
        name: "Cessna 172P (JSBSim aerodynamic tables)",
        mass: 1000.0,
        wing_area: c172p::WING_AREA,
        power: 70000.0,
        static_thrust: 2800.0,
        rolling_mu: 0.03,
        brake_mu: 0.35,
        cy_beta: 0.137 / 0.349,
        windmill_cd: 0.012,
        max_sideslip: 15.0 * D2R,
        max_pitch: 15.0 * D2R,
        min_pitch: -10.0 * D2R,
        tail_strike_pitch: c172p::tail_strike_pitch(),
    }
}

impl AircraftSpec {
    pub fn max_thrust(&self, v: f64) -> f64 {
        self.static_thrust.min(self.power / v.max(1.0))
    }
}

/// What the autopilot asks for each step. Neither model applies these directly: see `CommandShaper`.
#[derive(Clone, Copy, Debug, Default, Serialize)]
pub struct Commands {
    /// Pitch attitude, rad.
    pub theta: f64,
    /// Bank, rad (+ = left wing down, i.e. a left/CCW turn).
    pub phi: f64,
    /// Sideslip, rad: nose heading minus air-path heading, + = nose left (CCW).
    pub beta: f64,
    pub thr: f64,
    /// Flap, degrees.
    pub flaps: f64,
    pub brake: f64,
    /// Rotation phase: the wheels may leave the ground.
    pub allow_liftoff: bool,
    /// Just after lift-off or during a go-around a brief settle must not count as a landing.
    pub no_touchdown: bool,
    /// Flare: pitch may change at `FLARE_PITCH_RATE` instead of the normal rate, to arrest the sink in time.
    pub flare: bool,
}

/// Pitch slew allowed in the flare (the normal limit stays everywhere else, including go-arounds).
pub const FLARE_PITCH_RATE: f64 = 6.0 * D2R;

/// Rate limits applied to every command so no mode change can step a control.
#[derive(Clone, Copy, Debug, Serialize)]
pub struct SlewRates {
    pub theta: f64,
    pub phi: f64,
    pub thr: f64,
    pub flaps: f64,
    pub beta: f64,
}

pub const DEFAULT_SLEW: SlewRates =
    SlewRates { theta: 3.0 * D2R, phi: 8.0 * D2R, thr: 0.3, flaps: 3.0, beta: 4.0 * D2R };

#[derive(Clone, Debug)]
pub struct CommandShaper {
    pub theta: f64,
    /// Rate of the shaped pitch command, rad/s (feed-forward for the 6DOF pitch loop).
    pub theta_rate: f64,
    pub phi: f64,
    pub thr: f64,
    pub flaps: f64,
    pub beta: f64,
    pub rates: SlewRates,
}

impl Default for CommandShaper {
    fn default() -> Self {
        CommandShaper { theta: 0.0, theta_rate: 0.0, phi: 0.0, thr: 0.0, flaps: 10.0, beta: 0.0, rates: DEFAULT_SLEW }
    }
}

impl CommandShaper {
    pub fn shape(&mut self, c: &Commands, dt: f64) {
        let pitch_rate = if c.flare { self.rates.theta.max(FLARE_PITCH_RATE) } else { self.rates.theta };
        self.theta_rate = (slew(self.theta, c.theta, pitch_rate, dt) - self.theta) / dt;
        self.theta += self.theta_rate * dt;
        self.phi = slew(self.phi, c.phi, self.rates.phi, dt);
        self.thr = slew(self.thr, c.thr, self.rates.thr, dt);
        self.flaps = slew(self.flaps, c.flaps, self.rates.flaps, dt);
        self.beta = slew(self.beta, c.beta, self.rates.beta, dt);
    }
}

#[derive(Clone, Copy, Debug)]
pub enum Contact {
    Liftoff { v: f64 },
    /// `crab`: nose heading minus ground track, rad.
    Touchdown { sink: f64, crab: f64, v: f64, ground_speed: f64 },
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize)]
pub enum ModelKind {
    PointMass,
    SixDof,
}

impl ModelKind {
    pub fn name(self) -> &'static str {
        match self {
            ModelKind::PointMass => "Point-mass",
            ModelKind::SixDof => "6DOF",
        }
    }
}

pub struct Aero {
    pub l: f64,
    pub d: f64,
    pub stalled: bool,
}

/// The aircraft state used by the autopilot. Angles are in the airfield's math convention:
/// heading/track 0 = east (runway 09), counter-clockwise positive; bank positive = left wing down.
#[derive(Clone, Debug)]
pub struct Aircraft {
    pub x: f64,
    pub y: f64,
    /// Height of the wheels above the runway, m.
    pub h: f64,
    /// True airspeed (sea level, so = IAS), m/s.
    pub v: f64,
    /// Ground speed while on the ground.
    pub vg: f64,
    /// Air-path heading, rad; equals the nose heading in coordinated flight.
    pub psi: f64,
    /// Sideslip, rad: nose heading = psi + beta.
    pub beta: f64,
    /// Air-relative flight-path angle.
    pub gamma: f64,
    pub theta: f64,
    pub q: f64,
    pub phi: f64,
    pub p: f64,
    /// Filtered angle of attack, used as autopilot feed-forward.
    pub alpha_f: f64,
    pub thr: f64,
    pub flap_pos: f64,
    pub brake: f64,
    pub on_ground: bool,
    pub stalled: bool,
    pub engine_failed: bool,
    /// Heading held on the ground after a landing away from the runway; None = steer to the centreline.
    pub rollout_heading: Option<f64>,
    /// Runway direction steered to on the ground: 0 for 09, π for 27.
    pub runway_heading: f64,
    pub shaper: CommandShaper,
    pub spec: AircraftSpec,
    pub model: ModelKind,
    pub six: Option<Box<SixDof>>,
}

impl Aircraft {
    pub fn new(spec: AircraftSpec, model: ModelKind) -> Self {
        let mut ac = Aircraft {
            x: 0.0,
            y: 0.0,
            h: 0.0,
            v: 0.0,
            vg: 0.0,
            psi: 0.0,
            beta: 0.0,
            gamma: 0.0,
            theta: 0.0,
            q: 0.0,
            phi: 0.0,
            p: 0.0,
            alpha_f: 0.0,
            thr: 0.0,
            flap_pos: 10.0,
            brake: 0.0,
            on_ground: true,
            stalled: false,
            engine_failed: false,
            rollout_heading: None,
            runway_heading: 0.0,
            shaper: CommandShaper::default(),
            spec,
            model,
            six: None,
        };
        if model == ModelKind::SixDof {
            let six = SixDof::on_runway(&ac.spec);
            ac.six = Some(Box::new(six));
            ac.sync_from_six();
        }
        ac
    }

    pub fn flaps(&self) -> i32 {
        self.flap_pos.round() as i32
    }
    /// Where the nose points (what a compass reads).
    pub fn nose_heading(&self) -> f64 {
        wrap_pi(self.psi + self.beta)
    }
    /// Ground track angle, rad.
    pub fn track(&self, wind: &Wind) -> f64 {
        let gv = self.ground_velocity(wind);
        gv.y.atan2(gv.x)
    }
    pub fn alpha(&self) -> f64 {
        match &self.six {
            Some(s) => s.diag.alpha,
            None => {
                if self.on_ground {
                    self.theta
                } else {
                    self.theta - self.gamma
                }
            }
        }
    }
    pub fn vertical_speed(&self) -> f64 {
        if self.on_ground {
            return 0.0;
        }
        match &self.six {
            Some(s) => s.vertical_speed(),
            None => self.v * self.gamma.sin(),
        }
    }

    pub fn ground_velocity(&self, wind: &Wind) -> Vec2 {
        if let Some(s) = &self.six {
            return s.ground_velocity();
        }
        if self.on_ground {
            return Vec2 { x: self.vg * self.psi.cos(), y: self.vg * self.psi.sin() };
        }
        let hv = self.v * self.gamma.cos();
        Vec2 { x: hv * self.psi.cos() + wind.v.x, y: hv * self.psi.sin() + wind.v.y }
    }

    /// Side force from sideslip (point-mass), N, positive toward the nose (CCW).
    fn side_force(&self) -> f64 {
        0.5 * RHO * self.v * self.v * self.spec.wing_area * self.spec.cy_beta * self.beta
    }

    /// Bank that cancels the sideslip side force: the wing-low angle for a straight path.
    pub fn wing_low_bank(&self) -> f64 {
        if self.on_ground {
            return 0.0;
        }
        if let Some(s) = &self.six {
            return s.wing_low_bank();
        }
        let l = self.aero(self.v, self.theta - self.gamma).l;
        clamp(-self.side_force() / l.max(1.0), -0.3, 0.3).asin()
    }

    /// Point-mass aerodynamics: the same C172P lift and drag tables as the 6DOF model, without
    /// ground effect, rates or control-surface terms.
    pub fn aero(&self, v: f64, alpha: f64) -> Aero {
        let s = &self.spec;
        let (cl, cd0) = c172p::polar(alpha, self.flap_pos, 9.0, 0.0);
        let cd = cd0
            + c172p::CD_BETA * self.beta.abs()
            + if self.engine_failed { s.windmill_cd } else { 0.0 };
        let qs = 0.5 * RHO * v * v * s.wing_area;
        Aero { l: qs * cl, d: qs * cd, stalled: alpha > c172p::ALPHA_CLMAX }
    }

    pub fn thrust(&self, v: f64) -> f64 {
        if self.engine_failed {
            0.0
        } else {
            self.thr * self.spec.max_thrust(v)
        }
    }

    /// Still-air glide ratio (L/D) in trimmed, power-off, coordinated flight at airspeed V.
    /// 1-g stall speed at the current weight for a flap setting (from the lift table maximum).
    pub fn stall_speed(&self, flap_pos: f64) -> f64 {
        let cl_max = c172p::polar(c172p::ALPHA_CLMAX, flap_pos, 9.0, 0.0).0;
        (2.0 * self.spec.mass * G / (RHO * self.spec.wing_area * cl_max)).sqrt()
    }

    pub fn glide_ratio(&self, v: f64, flap_pos: f64) -> f64 {
        let s = &self.spec;
        let cl = (s.mass * G) / (0.5 * RHO * v * v * s.wing_area);
        let a = c172p::alpha_for_cl(cl, flap_pos);
        let (cl_a, cd) = c172p::polar(a, flap_pos, 9.0, 0.0);
        cl.min(cl_a) / (cd + s.windmill_cd)
    }

    /// Steering target on the ground: the runway centreline (either direction) or a held field heading.
    pub fn ground_heading_cmd(&self) -> f64 {
        match self.rollout_heading {
            Some(h) => h,
            None => self.runway_heading + clamp(-0.03 * self.y * self.runway_heading.cos(), -0.08, 0.08),
        }
    }

    /// After touchdown: decide whether we are on the runway line (steer to it) or in a field (hold heading).
    pub fn set_rollout_steering(&mut self, heading: f64) {
        let near09 = wrap_pi(heading).abs() < 30.0 * D2R;
        let near27 = wrap_pi(heading - PI).abs() < 30.0 * D2R;
        let on_runway_line = (near09 || near27) && self.y.abs() < 60.0;
        self.runway_heading = if near27 { PI } else { 0.0 };
        self.rollout_heading = if on_runway_line { None } else { Some(heading) };
    }

    pub fn step(&mut self, dt: f64, cmd: &Commands, wind: &Wind) -> Option<Contact> {
        self.shaper.shape(cmd, dt);
        if self.six.is_some() {
            self.thr = slew(self.thr, self.shaper.thr, 0.8, dt);
            self.flap_pos = self.shaper.flaps;
            self.brake = cmd.brake;
            let mut six = self.six.take().unwrap();
            let contact = six.step(self, dt, cmd, wind);
            self.six = Some(six);
            self.sync_from_six();
            return contact;
        }
        // point-mass inner loops: attitude error → rate demand → rate with actuator lag
        let sh = &self.shaper;
        let q_cmd = clamp(1.5 * (sh.theta - self.theta), -5.0 * D2R, 5.0 * D2R);
        self.q += ((q_cmd - self.q) * dt) / 0.3;
        self.theta += self.q * dt;
        let p_cmd = clamp(2.0 * (sh.phi - self.phi), -15.0 * D2R, 15.0 * D2R);
        self.p += ((p_cmd - self.p) * dt) / 0.2;
        self.phi += self.p * dt;
        self.thr = slew(self.thr, sh.thr, 0.8, dt);
        self.flap_pos = sh.flaps;
        self.brake = cmd.brake;
        self.beta = if self.on_ground {
            0.0
        } else {
            clamp(sh.beta, -self.spec.max_sideslip, self.spec.max_sideslip)
        };
        if self.on_ground {
            self.step_ground(dt, cmd, wind)
        } else {
            self.step_air(dt, cmd, wind)
        }
    }

    fn step_ground(&mut self, dt: f64, cmd: &Commands, wind: &Wind) -> Option<Contact> {
        self.phi = 0.0;
        self.p = 0.0;
        self.shaper.phi = 0.0;
        self.shaper.beta = 0.0;
        self.gamma = 0.0;
        self.theta = clamp(self.theta, 0.0, self.spec.tail_strike_pitch);
        if self.theta == 0.0 && self.q < 0.0 {
            self.q = 0.0;
        }
        self.v = (self.vg - wind.along(self.psi)).max(0.0);
        let a = self.aero(self.v, self.theta);
        let t = self.thrust(self.v);
        let s = &self.spec;
        let w = s.mass * G;
        let n = (w - a.l).max(0.0);
        let mu = s.rolling_mu + self.brake * s.brake_mu;
        let acc = (t - a.d - mu * n) / s.mass;
        self.vg = (self.vg + acc * dt).max(0.0);
        self.alpha_f = self.theta;
        let psi_cmd = self.ground_heading_cmd();
        self.psi = wrap_pi(self.psi + (wrap_pi(psi_cmd - self.psi) * dt) / 0.6);
        self.x += self.vg * self.psi.cos() * dt;
        self.y += self.vg * self.psi.sin() * dt;
        if cmd.allow_liftoff && a.l > w * 1.01 {
            self.on_ground = false;
            self.h = 0.01;
            return Some(Contact::Liftoff { v: self.v });
        }
        None
    }

    fn step_air(&mut self, dt: f64, cmd: &Commands, wind: &Wind) -> Option<Contact> {
        let alpha = self.theta - self.gamma;
        let a = self.aero(self.v, alpha);
        let t = self.thrust(self.v);
        self.stalled = a.stalled;
        let s = &self.spec;
        let vs = self.v.max(5.0);
        let mv = s.mass * vs;
        let vdot = (t * alpha.cos() - a.d) / s.mass - G * self.gamma.sin();
        let lt = a.l + t * alpha.sin();
        let gdot = (lt * self.phi.cos()) / mv - (G * self.gamma.cos()) / vs;
        // horizontal turn: lift tilted by bank, plus the sideslip side force pulling the path toward the nose
        let psidot = (lt * self.phi.sin() + self.side_force()) / (mv * self.gamma.cos());
        self.v += vdot * dt;
        self.gamma += gdot * dt;
        self.psi = wrap_pi(self.psi + psidot * dt);
        self.alpha_f += ((alpha - self.alpha_f) * dt) / 0.5;
        let gv = self.ground_velocity(wind);
        self.x += gv.x * dt;
        self.y += gv.y * dt;
        self.h += self.v * self.gamma.sin() * dt;

        if self.h > 0.0 {
            return None;
        }
        if cmd.no_touchdown {
            self.h = 0.0;
            self.gamma = self.gamma.max(0.0);
            return None;
        }
        let sink = -self.v * self.gamma.sin();
        let crab = wrap_pi(self.nose_heading() - gv.y.atan2(gv.x));
        self.psi = self.nose_heading();
        self.beta = 0.0; // wheels track where the nose points
        self.set_rollout_steering(self.psi);
        self.h = 0.0;
        self.on_ground = true;
        self.vg = self.v * self.gamma.cos() + wind.along(self.psi);
        self.gamma = 0.0;
        Some(Contact::Touchdown { sink, crab, v: self.v, ground_speed: self.vg })
    }

    /// Copy the 6DOF body state into the fields the autopilot reads.
    fn sync_from_six(&mut self) {
        let six = self.six.as_ref().unwrap();
        let o = six.observe();
        self.x = o.x;
        self.y = o.y;
        self.h = o.h;
        self.v = o.v;
        self.vg = o.vg;
        self.psi = o.psi;
        self.beta = o.beta;
        self.gamma = o.gamma;
        self.theta = o.theta;
        self.phi = o.phi;
        self.q = o.q;
        self.p = o.p;
        self.alpha_f = six.alpha_f;
        self.stalled = six.diag.stalled;
    }
}
