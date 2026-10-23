//! Six-degree-of-freedom rigid-body flight model.
//!
//! * State: position (NED), body velocity (u, v, w), attitude quaternion, body rates (p, q, r).
//! * Forces and moments: aerodynamic coefficients for a C172-class aircraft (stability derivatives,
//!   control derivatives, flap increments, stall, ground effect), propeller thrust with torque and
//!   slipstream yaw, three spring–damper landing-gear legs with tyre friction, plus a tail skid.
//! * Integration: RK4 at 200 Hz (four substeps per 50 Hz autopilot step).
//! * `Fcs`: the inner-loop control laws. They turn the autopilot's attitude commands (pitch, bank,
//!   sideslip) into elevator, aileron and rudder, and steer with nose wheel and rudder on the ground.
//!
//! Body axes: x forward, y right, z down. World: NED with the runway along east (heading 090).

pub mod fcs;
#[cfg(not(target_arch = "wasm32"))]
pub mod analysis;

use crate::aircraft::{Aircraft, AircraftSpec, Commands, Contact};
use crate::atmosphere::Wind;
use crate::units::{clamp, wrap_pi, Vec2, D2R, G, PI, RHO};
use fcs::Fcs;
use serde::Serialize;

pub type V3 = [f64; 3];

#[inline]
fn add(a: V3, b: V3) -> V3 {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}
#[inline]
fn sub(a: V3, b: V3) -> V3 {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}
#[inline]
fn scale(a: V3, k: f64) -> V3 {
    [a[0] * k, a[1] * k, a[2] * k]
}
#[inline]
fn cross(a: V3, b: V3) -> V3 {
    [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]
}
#[inline]
fn norm(a: V3) -> f64 {
    (a[0] * a[0] + a[1] * a[1] + a[2] * a[2]).sqrt()
}

/// Rotation matrix body → NED from quaternion [w, x, y, z].
pub fn rot(q: &[f64]) -> [[f64; 3]; 3] {
    let (w, x, y, z) = (q[0], q[1], q[2], q[3]);
    [
        [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - w * z), 2.0 * (x * z + w * y)],
        [2.0 * (x * y + w * z), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - w * x)],
        [2.0 * (x * z - w * y), 2.0 * (y * z + w * x), 1.0 - 2.0 * (x * x + y * y)],
    ]
}
#[inline]
pub fn mul(r: &[[f64; 3]; 3], v: V3) -> V3 {
    [
        r[0][0] * v[0] + r[0][1] * v[1] + r[0][2] * v[2],
        r[1][0] * v[0] + r[1][1] * v[1] + r[1][2] * v[2],
        r[2][0] * v[0] + r[2][1] * v[1] + r[2][2] * v[2],
    ]
}
#[inline]
pub fn mul_t(r: &[[f64; 3]; 3], v: V3) -> V3 {
    [
        r[0][0] * v[0] + r[1][0] * v[1] + r[2][0] * v[2],
        r[0][1] * v[0] + r[1][1] * v[1] + r[2][1] * v[2],
        r[0][2] * v[0] + r[1][2] * v[1] + r[2][2] * v[2],
    ]
}

/// Quaternion from 3-2-1 Euler angles (roll φ, pitch θ, yaw ψ), NED.
pub fn quat_from_euler(phi: f64, theta: f64, psi: f64) -> [f64; 4] {
    let (cr, sr) = ((phi / 2.0).cos(), (phi / 2.0).sin());
    let (cp, sp) = ((theta / 2.0).cos(), (theta / 2.0).sin());
    let (cy, sy) = ((psi / 2.0).cos(), (psi / 2.0).sin());
    [
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    ]
}

/// 3-2-1 Euler angles (φ, θ, ψ) from quaternion, NED.
pub fn euler(q: &[f64]) -> (f64, f64, f64) {
    let (w, x, y, z) = (q[0], q[1], q[2], q[3]);
    let phi = (2.0 * (w * x + y * z)).atan2(1.0 - 2.0 * (x * x + y * y));
    let theta = clamp(2.0 * (w * y - z * x), -1.0, 1.0).asin();
    let psi = (2.0 * (w * z + x * y)).atan2(1.0 - 2.0 * (y * y + z * z));
    (phi, theta, psi)
}

/// Aerodynamic and mass properties. Stability derivatives are per radian; rate derivatives use
/// p̂ = p·b/2V, q̂ = q·c/2V, r̂ = r·b/2V. Values are C172-class (Roskam / open references), with the
/// lift and drag polar matched to the point-mass model so both models glide the same.
#[derive(Clone, Debug, Serialize)]
pub struct SixDofParams {
    pub span: f64,
    pub chord: f64,
    pub ixx: f64,
    pub iyy: f64,
    pub izz: f64,
    pub cl_q: f64,
    pub cl_de: f64,
    pub cm0: f64,
    pub cm_alpha: f64,
    /// Includes the α̇ contribution.
    pub cm_q: f64,
    pub cm_de: f64,
    /// Pitching moment per degree of flap.
    pub cm_flap: f64,
    pub cy_beta: f64,
    pub cy_dr: f64,
    pub cl_beta: f64,
    pub cl_p: f64,
    pub cl_r: f64,
    pub cl_da: f64,
    pub cl_dr: f64,
    pub cn_beta: f64,
    pub cn_p: f64,
    pub cn_r: f64,
    pub cn_da: f64,
    pub cn_dr: f64,
    /// Propeller torque roll moment per newton of thrust (negative = rolls left), m.
    pub prop_roll_arm: f64,
    /// Slipstream / P-factor yaw moment per newton of thrust (negative = yaws left), m.
    pub prop_yaw_arm: f64,
    /// Surface limits, rad. Elevator: positive trailing edge down (nose down).
    pub de_min: f64,
    pub de_max: f64,
    pub da_max: f64,
    pub dr_max: f64,
    pub steer_max: f64,
    pub actuator_tau: f64,
    pub actuator_rate: f64,
}

pub fn c172_six() -> SixDofParams {
    SixDofParams {
        span: 11.0,
        chord: 1.49,
        ixx: 1285.0,
        iyy: 1825.0,
        izz: 2667.0,
        cl_q: 3.9,
        cl_de: 0.43,
        cm0: 0.04,
        cm_alpha: -0.89,
        cm_q: -12.4 - 5.2,
        cm_de: -1.28,
        cm_flap: -0.0025,
        cy_beta: -0.31,
        cy_dr: -0.187,
        cl_beta: -0.089,
        cl_p: -0.47,
        cl_r: 0.096,
        cl_da: 0.178,
        cl_dr: -0.0147,
        cn_beta: 0.065,
        cn_p: -0.03,
        cn_r: -0.099,
        cn_da: -0.02,
        cn_dr: 0.0657,
        prop_roll_arm: -0.10,
        prop_yaw_arm: -0.04,
        de_min: -25.0 * D2R,
        de_max: 15.0 * D2R,
        da_max: 20.0 * D2R,
        dr_max: 25.0 * D2R,
        steer_max: 25.0 * D2R,
        actuator_tau: 0.04,
        actuator_rate: 70.0 * D2R,
    }
}

/// One landing-gear leg (or the tail skid): a vertical spring–damper with tyre friction.
#[derive(Clone, Debug, Serialize)]
pub struct GearLeg {
    pub name: &'static str,
    /// Body position of the contact point relative to the centre of gravity, m.
    pub pos: V3,
    pub k: f64,
    pub c: f64,
    pub mu_roll: f64,
    pub mu_side: f64,
    pub braked: bool,
    pub steered: bool,
}

pub fn c172_gear() -> [GearLeg; 4] {
    [
        GearLeg { name: "nose", pos: [1.35, 0.0, 1.15], k: 25000.0, c: 2600.0, mu_roll: 0.02, mu_side: 0.8, braked: false, steered: true },
        GearLeg { name: "left", pos: [-0.35, -1.25, 1.10], k: 45000.0, c: 4500.0, mu_roll: 0.02, mu_side: 0.8, braked: true, steered: false },
        GearLeg { name: "right", pos: [-0.35, 1.25, 1.10], k: 45000.0, c: 4500.0, mu_roll: 0.02, mu_side: 0.8, braked: true, steered: false },
        GearLeg { name: "tail", pos: [-4.6, 0.0, 0.15], k: 60000.0, c: 5000.0, mu_roll: 0.35, mu_side: 0.35, braked: false, steered: false },
    ]
}

/// Actual control positions fed into the dynamics.
#[derive(Clone, Copy, Debug, Default, Serialize)]
pub struct Controls {
    pub de: f64,
    pub da: f64,
    pub dr: f64,
    /// Nose-wheel steering angle, rad (+ = right).
    pub steer: f64,
    pub thr: f64,
    pub flap: f64,
    pub brake: f64,
    pub engine_failed: bool,
}

/// Values computed alongside the derivatives, for the control laws, the autopilot and display.
#[derive(Clone, Copy, Debug, Default, Serialize)]
pub struct Diag {
    pub alpha: f64,
    pub beta: f64,
    pub v: f64,
    pub lift: f64,
    /// Aerodynamic side force along body y (+ = right), N.
    pub side: f64,
    pub thrust: f64,
    /// Normal loads on nose, left, right, tail, N.
    pub loads: [f64; 4],
    pub stalled: bool,
}

/// What the autopilot reads, in its own (math, CCW-positive) convention.
pub struct Observed {
    pub x: f64,
    pub y: f64,
    pub h: f64,
    pub v: f64,
    pub vg: f64,
    pub psi: f64,
    pub beta: f64,
    pub gamma: f64,
    pub theta: f64,
    pub phi: f64,
    pub q: f64,
    pub p: f64,
}

pub const N_STATE: usize = 13;
pub type State = [f64; N_STATE];

#[derive(Clone, Debug)]
pub struct SixDof {
    /// [n, e, d, u, v, w, q0, q1, q2, q3, p, q, r]
    pub x: State,
    pub prm: SixDofParams,
    pub gear: [GearLeg; 4],
    pub mass: f64,
    pub spec: AircraftSpec,
    pub ctl: Controls,
    pub fcs: Fcs,
    pub diag: Diag,
    pub alpha_f: f64,
    /// Centre-of-gravity height when parked: the zero of the reported wheel height.
    pub h0: f64,
    wind_ned: V3,
    unloaded_for: f64,
    last_sink: f64,
    pub substeps: usize,
}

impl SixDof {
    /// Parked on the runway threshold, heading 090, settled on its gear.
    pub fn on_runway(spec: &AircraftSpec) -> Self {
        let prm = c172_six();
        let gear = c172_gear();
        let mut x = [0.0; N_STATE];
        x[2] = -gear[1].pos[2];
        let q = quat_from_euler(0.0, 0.0, PI / 2.0);
        x[6..10].copy_from_slice(&q);
        let mut s = SixDof {
            x,
            prm,
            gear,
            mass: spec.mass,
            spec: spec.clone(),
            ctl: Controls { flap: 10.0, brake: 1.0, ..Default::default() },
            fcs: Fcs::default(),
            diag: Diag::default(),
            alpha_f: 0.0,
            h0: 0.0,
            wind_ned: [0.0; 3],
            unloaded_for: 0.0,
            last_sink: 0.0,
            substeps: 4,
        };
        // settle onto the gear with brakes on and the engine idle
        for _ in 0..1600 {
            s.rk4(0.005);
        }
        for v in &mut s.x[3..6] {
            *v = 0.0;
        }
        for v in &mut s.x[10..13] {
            *v = 0.0;
        }
        s.x[0] = 0.0;
        s.x[1] = 0.0;
        s.h0 = -s.x[2];
        s.ctl.brake = 0.0;
        s
    }

    /// Time derivative of the state, plus diagnostics.
    pub fn derivs(&self, s: &State, u: &Controls, wind_ned: V3) -> (State, Diag) {
        let prm = &self.prm;
        let spec = &self.spec;
        let r = rot(&s[6..10]);
        let vb: V3 = [s[3], s[4], s[5]];
        let w: V3 = [s[10], s[11], s[12]];
        let mut diag = Diag::default();

        // --- aerodynamics
        let va = sub(vb, mul_t(&r, wind_ned));
        let v = norm(va);
        let alpha = va[2].atan2(va[0]);
        let beta = if v > 0.1 { clamp(va[1] / v, -1.0, 1.0).asin() } else { 0.0 };
        // keep the linear coefficient model inside its range (taxiing in a tailwind gives α ≈ ±180°)
        let a_eff = clamp(alpha, -0.6, 0.6);
        let b_eff = clamp(beta, -0.6, 0.6);
        let vn = v.max(3.0);
        let qbar = 0.5 * RHO * v * v;
        let f = spec.flap_increment(u.flap);
        let (ph, qh, rh) = (w[0] * prm.span / (2.0 * vn), w[1] * prm.chord / (2.0 * vn), w[2] * prm.span / (2.0 * vn));
        let mut cl_s = spec.cl0 + f.d_cl + spec.cl_alpha * a_eff;
        let cl_max = spec.cl_max + f.d_clmax;
        if cl_s > cl_max {
            diag.stalled = v > 15.0;
            cl_s = cl_max - (cl_s - cl_max) * 1.5;
        }
        cl_s = cl_s.max(-0.8);
        let cl = cl_s + prm.cl_q * qh + prm.cl_de * u.de;
        // ground effect on induced drag (high wing about 1.4 m above the wheels)
        let hw = (-s[2] - self.h0).max(0.0) + 1.4;
        let ge = (16.0 * (hw / prm.span).powi(2)) / (1.0 + 16.0 * (hw / prm.span).powi(2));
        let cd = spec.cd0
            + f.d_cd
            + spec.induced_k * ge * cl * cl
            + 0.3 * b_eff * b_eff
            + if u.engine_failed { spec.windmill_cd } else { 0.0 };
        let cy = prm.cy_beta * b_eff + prm.cy_dr * u.dr;
        let c_l = prm.cl_beta * b_eff + prm.cl_p * ph + prm.cl_r * rh + prm.cl_da * u.da + prm.cl_dr * u.dr;
        let c_m = prm.cm0 + prm.cm_alpha * a_eff + prm.cm_q * qh + prm.cm_de * u.de + prm.cm_flap * u.flap;
        let c_n = prm.cn_beta * b_eff + prm.cn_p * ph + prm.cn_r * rh + prm.cn_da * u.da + prm.cn_dr * u.dr;
        let qs = qbar * spec.wing_area;
        let mut force: V3 = [0.0; 3];
        if v > 0.1 {
            let dhat = scale(va, -1.0 / v);
            let lhat: V3 = [alpha.sin(), 0.0, -alpha.cos()];
            force = add(scale(dhat, qs * cd), scale(lhat, qs * cl));
            force[1] += qs * cy;
        }
        let mut moment: V3 = [qs * prm.span * c_l, qs * prm.chord * c_m, qs * prm.span * c_n];

        // --- propeller
        let thrust = if u.engine_failed { 0.0 } else { u.thr * spec.max_thrust(v) };
        force[0] += thrust;
        moment[0] += prm.prop_roll_arm * thrust;
        moment[2] += prm.prop_yaw_arm * thrust;

        // --- gravity
        force = add(force, mul_t(&r, [0.0, 0.0, self.mass * G]));

        // --- landing gear
        let pos: V3 = [s[0], s[1], s[2]];
        for (i, g) in self.gear.iter().enumerate() {
            let pw = add(pos, mul(&r, g.pos));
            let pen = pw[2];
            if pen <= 0.0 {
                continue;
            }
            let vw = mul(&r, add(vb, cross(w, g.pos)));
            let fn_ = (g.k * pen + g.c * vw[2]).max(0.0);
            if fn_ <= 0.0 {
                continue;
            }
            let st = if g.steered { u.steer } else { 0.0 };
            let wheel = mul(&r, [st.cos(), st.sin(), 0.0]);
            let hl = wheel[0].hypot(wheel[1]).max(1e-6);
            let el = [wheel[0] / hl, wheel[1] / hl];
            let et = [-el[1], el[0]];
            let v_l = vw[0] * el[0] + vw[1] * el[1];
            let v_t = vw[0] * et[0] + vw[1] * et[1];
            let mu_l = g.mu_roll + if g.braked { spec.brake_mu * 1.3 * u.brake } else { 0.0 };
            let f_l = -mu_l * fn_ * (v_l / 0.3).tanh();
            let f_t = -g.mu_side * fn_ * (v_t / 0.1).tanh();
            let fw: V3 = [f_l * el[0] + f_t * et[0], f_l * el[1] + f_t * et[1], -fn_];
            let fb = mul_t(&r, fw);
            force = add(force, fb);
            moment = add(moment, cross(g.pos, fb));
            diag.loads[i] = fn_;
        }

        // --- equations of motion
        let m = self.mass;
        let vdot = sub(scale(force, 1.0 / m), cross(w, vb));
        let iw: V3 = [prm.ixx * w[0], prm.iyy * w[1], prm.izz * w[2]];
        let gyro = cross(w, iw);
        let wdot: V3 = [
            (moment[0] - gyro[0]) / prm.ixx,
            (moment[1] - gyro[1]) / prm.iyy,
            (moment[2] - gyro[2]) / prm.izz,
        ];
        let (q0, q1, q2, q3) = (s[6], s[7], s[8], s[9]);
        let (p, q, rr) = (w[0], w[1], w[2]);
        let qdot = [
            0.5 * (-q1 * p - q2 * q - q3 * rr),
            0.5 * (q0 * p + q2 * rr - q3 * q),
            0.5 * (q0 * q - q1 * rr + q3 * p),
            0.5 * (q0 * rr + q1 * q - q2 * p),
        ];
        let pdot = mul(&r, vb);
        let mut d = [0.0; N_STATE];
        d[0..3].copy_from_slice(&pdot);
        d[3..6].copy_from_slice(&vdot);
        d[6..10].copy_from_slice(&qdot);
        d[10..13].copy_from_slice(&wdot);

        diag.alpha = alpha;
        diag.beta = beta;
        diag.v = v;
        diag.lift = qs * cl;
        diag.side = qs * cy;
        diag.thrust = thrust;
        (d, diag)
    }

    /// One RK4 step with the current controls and wind.
    pub fn rk4(&mut self, h: f64) {
        let u = self.ctl;
        let w = self.wind_ned;
        let s0 = self.x;
        let (k1, _) = self.derivs(&s0, &u, w);
        let s1 = axpy(&s0, &k1, h / 2.0);
        let (k2, _) = self.derivs(&s1, &u, w);
        let s2 = axpy(&s0, &k2, h / 2.0);
        let (k3, _) = self.derivs(&s2, &u, w);
        let s3 = axpy(&s0, &k3, h);
        let (k4, _) = self.derivs(&s3, &u, w);
        for i in 0..N_STATE {
            self.x[i] = s0[i] + h / 6.0 * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]);
        }
        let n = (self.x[6] * self.x[6] + self.x[7] * self.x[7] + self.x[8] * self.x[8] + self.x[9] * self.x[9]).sqrt();
        for v in &mut self.x[6..10] {
            *v /= n;
        }
        let (_, d) = self.derivs(&self.x, &u, w);
        self.diag = d;
    }

    pub fn wheels_loaded(&self) -> bool {
        self.diag.loads.iter().sum::<f64>() > 100.0
    }
    pub fn mains_loaded(&self) -> bool {
        self.diag.loads[1] + self.diag.loads[2] + self.diag.loads[3] > 150.0
    }

    /// Advance by one autopilot step (`dt`), in `substeps` RK4 steps, with the control laws running
    /// at the substep rate. Updates `ac.on_ground` and reports lift-off / touchdown.
    pub fn step(&mut self, ac: &mut Aircraft, dt: f64, cmd: &Commands, wind: &Wind) -> Option<Contact> {
        self.wind_ned = [wind.v.y, wind.v.x, 0.0];
        let h = dt / self.substeps as f64;
        let mut contact = None;
        for _ in 0..self.substeps {
            let mut fcs = std::mem::take(&mut self.fcs);
            let targets = fcs.update(self, ac, cmd, h);
            self.fcs = fcs;
            let p = &self.prm;
            let act = |cur: f64, tgt: f64| -> f64 {
                let want = cur + (tgt - cur) * (h / p.actuator_tau).min(1.0);
                cur + clamp(want - cur, -p.actuator_rate * h, p.actuator_rate * h)
            };
            self.ctl.de = clamp(act(self.ctl.de, targets.de), p.de_min, p.de_max);
            self.ctl.da = clamp(act(self.ctl.da, targets.da), -p.da_max, p.da_max);
            self.ctl.dr = clamp(act(self.ctl.dr, targets.dr), -p.dr_max, p.dr_max);
            self.ctl.steer = clamp(act(self.ctl.steer, targets.steer), -p.steer_max, p.steer_max);
            self.ctl.thr = ac.thr;
            self.ctl.flap = ac.flap_pos;
            self.ctl.brake = ac.brake;
            self.ctl.engine_failed = ac.engine_failed;
            let vs_before = self.vertical_speed();
            self.rk4(h);
            let airborne_alpha = self.diag.alpha;
            self.alpha_f += ((if ac.on_ground { self.observe().theta } else { airborne_alpha }) - self.alpha_f) * h / 0.5;

            // weight on wheels → lift-off and touchdown
            if ac.on_ground {
                self.unloaded_for = if self.wheels_loaded() { 0.0 } else { self.unloaded_for + h };
                if cmd.allow_liftoff && self.unloaded_for > 0.2 && contact.is_none() {
                    ac.on_ground = false;
                    contact = Some(Contact::Liftoff { v: self.diag.v });
                }
            } else if self.mains_loaded() || self.diag.loads[0] > 150.0 {
                if !cmd.no_touchdown && contact.is_none() {
                    ac.on_ground = true;
                    let o = self.observe();
                    let gv = self.ground_velocity();
                    let nose = wrap_pi(o.psi + o.beta);
                    let crab = wrap_pi(nose - gv.y.atan2(gv.x));
                    ac.set_rollout_steering(nose);
                    self.unloaded_for = 0.0;
                    self.fcs.on_touchdown();
                    contact = Some(Contact::Touchdown {
                        sink: self.last_sink.max(-vs_before),
                        crab,
                        v: self.diag.v,
                        ground_speed: gv.x.hypot(gv.y),
                    });
                }
            } else {
                self.last_sink = -self.vertical_speed();
            }
        }
        contact
    }

    pub fn ground_velocity(&self) -> Vec2 {
        let r = rot(&self.x[6..10]);
        let v = mul(&r, [self.x[3], self.x[4], self.x[5]]);
        Vec2 { x: v[1], y: v[0] }
    }
    pub fn vertical_speed(&self) -> f64 {
        let r = rot(&self.x[6..10]);
        -mul(&r, [self.x[3], self.x[4], self.x[5]])[2]
    }
    /// Bank (autopilot convention, + = left wing down) that cancels the current aerodynamic side force.
    pub fn wing_low_bank(&self) -> f64 {
        clamp(self.diag.side / self.diag.lift.max(1.0), -0.3, 0.3).asin()
    }

    pub fn observe(&self) -> Observed {
        let r = rot(&self.x[6..10]);
        let vn = mul(&r, [self.x[3], self.x[4], self.x[5]]);
        let va = sub(vn, self.wind_ned);
        let v = norm(va);
        let (phi, theta, psi_ned) = euler(&self.x[6..10]);
        let nose = wrap_pi(PI / 2.0 - psi_ned);
        let hv = va[0].hypot(va[1]);
        let loaded = self.wheels_loaded();
        let (psi, beta) = if hv > 3.0 && !loaded {
            let path = wrap_pi(PI / 2.0 - va[1].atan2(va[0]));
            (path, wrap_pi(nose - path))
        } else {
            (nose, 0.0)
        };
        let gamma = if v > 3.0 { clamp(-va[2] / v, -1.0, 1.0).asin() } else { 0.0 };
        Observed {
            x: self.x[1],
            y: self.x[0],
            h: (-self.x[2] - self.h0).max(0.0),
            v,
            vg: vn[0].hypot(vn[1]),
            psi,
            beta,
            gamma,
            theta,
            phi: -phi,
            q: self.x[11],
            p: -self.x[10],
        }
    }
}

fn axpy(a: &State, b: &State, k: f64) -> State {
    let mut o = *a;
    for i in 0..N_STATE {
        o[i] += b[i] * k;
    }
    o
}
