//! Six-degree-of-freedom rigid-body flight model.
//!
//! * State: position (NED), body velocity (u, v, w), attitude quaternion, body rates (p, q, r).
//! * Forces and moments: the JSBSim Cessna 172P aerodynamic tables (`crate::c172p`): lift with stall
//!   hysteresis, drag by α and flap, ground effect, stability and control derivatives, slipstream
//!   over elevator and rudder. Propeller thrust with torque and P-factor; the JSBSim gear (three
//!   spring–damper legs with tyre friction) plus a tail skid.
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
use crate::c172p;
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

/// Geometry, inertia, linear derivatives and surface limits. The derivatives here are the C172P
/// (JSBSim) values at small angles: the dynamics use the full tables in `crate::c172p`, and these
/// copies serve the control-law feed-forward and documentation. Rates are normalised:
/// p̂ = p·b/2V, q̂ = q·c/2V, r̂ = r·b/2V.
#[derive(Clone, Debug, Serialize)]
pub struct SixDofParams {
    pub span: f64,
    pub chord: f64,
    pub ixx: f64,
    pub iyy: f64,
    pub izz: f64,
    /// CLq + CLα̇ (α̇ ≈ q in the short-period motion).
    pub cl_q: f64,
    /// Cmq + Cmα̇.
    pub cm_q: f64,
    pub cl_beta: f64,
    pub cl_da: f64,
    pub cl_dr: f64,
    /// Aerodynamic reference point, body axes from the CG, m.
    pub aero_rp: V3,
    /// Propeller hub, body axes from the CG, m (thrust line along body x).
    pub prop_pos: V3,
    /// Propeller torque roll moment per newton of thrust (negative = rolls left), m.
    pub prop_roll_arm: f64,
    /// P-factor / slipstream-swirl yaw moment per newton of thrust (negative = yaws left), m.
    pub prop_yaw_arm: f64,
    /// Surface limits, rad. Elevator: positive trailing edge down (nose down).
    pub de_min: f64,
    pub de_max: f64,
    pub da_max: f64,
    pub dr_max: f64,
    pub steer_max: f64,
    pub actuator_tau: f64,
    pub actuator_rate: f64,
    /// Robustness-test multipliers (1 = nominal): all control-surface moments, and Cmα.
    pub control_power: f64,
    pub pitch_stability: f64,
}

pub fn c172_six() -> SixDofParams {
    SixDofParams {
        span: c172p::SPAN,
        chord: c172p::CHORD,
        ixx: c172p::IXX,
        iyy: c172p::IYY,
        izz: c172p::IZZ,
        cl_q: c172p::CL_Q + c172p::CL_ADOT,
        cm_q: c172p::CM_Q + c172p::CM_ADOT,
        cl_beta: c172p::CL_BETA,
        cl_da: c172p::CL_DA,
        cl_dr: c172p::CL_DR,
        aero_rp: c172p::AERO_RP,
        prop_pos: c172p::PROP,
        prop_roll_arm: -0.10,
        prop_yaw_arm: -0.03,
        de_min: c172p::DE_MIN,
        de_max: c172p::DE_MAX,
        da_max: c172p::DA_MAX,
        dr_max: c172p::DR_MAX,
        steer_max: c172p::STEER_MAX,
        actuator_tau: 0.04,
        actuator_rate: 70.0 * D2R,
        control_power: 1.0,
        pitch_stability: 1.0,
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

/// Gear contact points, springs and dampers from the JSBSim C172P (the tail skid is this model's own).
pub fn c172_gear() -> [GearLeg; 4] {
    use c172p::*;
    [
        GearLeg { name: "nose", pos: NOSE, k: NOSE_K, c: NOSE_C, mu_roll: 0.02, mu_side: 0.8, braked: false, steered: true },
        GearLeg { name: "left", pos: MAIN_L, k: MAIN_K, c: MAIN_C, mu_roll: 0.02, mu_side: 0.8, braked: true, steered: false },
        GearLeg { name: "right", pos: MAIN_R, k: MAIN_K, c: MAIN_C, mu_roll: 0.02, mu_side: 0.8, braked: true, steered: false },
        GearLeg { name: "tail", pos: TAIL, k: 60000.0, c: 5000.0, mu_roll: 0.35, mu_side: 0.35, braked: false, steered: false },
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
    /// Slipstream over the tail: dynamic pressure there over free-stream (≥ 1 with power).
    pub qbar_ratio: f64,
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
    /// Stall hysteresis: 1 once α passes 0.36 rad, back to 0 below 0.09 rad.
    pub stall_hyst: f64,
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
            stall_hyst: 0.0,
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
    /// Steady wings-level power-off glide (engine failed, propeller windmilling) at airspeed `v` and
    /// `flap`: solves α, elevator and flight-path angle for zero u̇, ẇ, q̇, so the elevator's trim drag
    /// is included. Returns the glide ratio, or None where no trim exists (below the stall).
    pub fn glide_trim(&self, v: f64, flap: f64) -> Option<f64> {
        let res = |z: [f64; 3]| -> [f64; 3] {
            let mut x = [0.0; N_STATE];
            x[2] = -2000.0;
            x[3] = v * z[0].cos();
            x[5] = v * z[0].sin();
            x[6..10].copy_from_slice(&quat_from_euler(0.0, z[0] + z[2], 0.0));
            let u = Controls { de: z[1], thr: 0.0, flap, engine_failed: true, ..Default::default() };
            let (d, _) = self.derivs(&x, &u, [0.0; 3]);
            [d[3], d[5], d[11]]
        };
        let det = |a: [[f64; 3]; 3]| {
            a[0][0] * (a[1][1] * a[2][2] - a[1][2] * a[2][1]) - a[0][1] * (a[1][0] * a[2][2] - a[1][2] * a[2][0])
                + a[0][2] * (a[1][0] * a[2][1] - a[1][1] * a[2][0])
        };
        let mut z = [0.08, -0.05, -0.1];
        for _ in 0..40 {
            let r = res(z);
            if r.iter().map(|x| x.abs()).sum::<f64>() < 1e-9 {
                break;
            }
            let mut j = [[0.0; 3]; 3];
            for k in 0..3 {
                let mut zp = z;
                zp[k] += 1e-6;
                let rp = res(zp);
                for i in 0..3 {
                    j[i][k] = (rp[i] - r[i]) / 1e-6;
                }
            }
            let d0 = det(j);
            if d0.abs() < 1e-12 {
                return None;
            }
            for k in 0..3 {
                let mut a = j;
                for i in 0..3 {
                    a[i][k] = -r[i];
                }
                z[k] += det(a) / d0;
            }
        }
        let ok = res(z).iter().map(|x| x.abs()).sum::<f64>() < 1e-6 && z[2] < 0.0 && z[0] < 0.25;
        ok.then(|| 1.0 / (-z[2]).tan())
    }

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
        let vn = v.max(3.0);
        let qbar = 0.5 * RHO * v * v;
        let flap = u.flap;
        let (ph, qh, rh) = (w[0] * prm.span / (2.0 * vn), w[1] * prm.chord / (2.0 * vn), w[2] * prm.span / (2.0 * vn));
        // tables are clamped at their ends (taxiing in a tailwind gives α ≈ ±180°)
        let a_t = if va[0] > 0.0 { alpha } else { 0.0 };
        let b_t = if va[0] > 0.0 { beta } else { 0.0 };
        // ground effect: aero reference point height over span
        let hb = ((-s[2] - self.h0).max(0.0) + c172p::WING_H) / prm.span;
        let (cl_s, cd_s) = c172p::polar(a_t, flap, hb, self.stall_hyst);
        diag.stalled = v > 15.0 && (a_t > c172p::ALPHA_CLMAX || self.stall_hyst > 0.5);
        let cl = cl_s + prm.cl_q * qh + c172p::CL_DE * u.de;
        let cd = cd_s
            + c172p::CD_DE * u.de.abs()
            + c172p::CD_BETA * b_t.abs()
            + if u.engine_failed { spec.windmill_cd } else { 0.0 };

        // --- propeller: power-limited thrust, and the slipstream it blows over the tail
        let thrust = if u.engine_failed { 0.0 } else { u.thr * spec.max_thrust(v) };
        let v_ind = 0.5 * (-va[0].max(0.0) + (va[0].max(0.0).powi(2) + 2.0 * thrust.max(0.0) / (RHO * c172p::PROP_DISC)).sqrt());
        let u_wash = va[0].max(0.0) + 2.0 * v_ind;
        let qbar_ind = 0.5 * RHO * u_wash * u_wash;

        let cy = c172p::cy_b(b_t, flap)
            + c172p::cy_p(a_t, flap) * ph
            + c172p::cy_r(a_t, flap) * rh
            + c172p::CY_DR * u.dr;
        let kc = prm.control_power;
        let c_l = c172p::cl_b(b_t) + c172p::CL_P * ph + c172p::cl_r(a_t, flap) * rh + kc * (c172p::CL_DA * u.da + c172p::CL_DR * u.dr);
        let c_m = c172p::CM0 + prm.pitch_stability * c172p::CM_ALPHA * a_t + prm.cm_q * qh + c172p::cm_df(flap);
        let c_n = c172p::cn_b(b_t) + c172p::CN_P * ph + c172p::CN_R * rh + c172p::CN_DA * u.da;
        let qs = qbar * spec.wing_area;
        let qs_ind = qbar_ind * spec.wing_area;
        let mut force: V3 = [0.0; 3];
        if v > 0.1 {
            let dhat = scale(va, -1.0 / v);
            let lhat: V3 = [alpha.sin(), 0.0, -alpha.cos()];
            force = add(scale(dhat, qs * cd), scale(lhat, qs * cl));
            force[1] += qs * cy;
        }
        let mut moment: V3 = [
            qs * prm.span * c_l,
            qs * prm.chord * c_m + kc * qs_ind * prm.chord * c172p::CM_DE * u.de,
            qs * prm.span * c_n + kc * qs_ind * prm.span * c172p::CN_DR * u.dr,
        ];
        // forces act at the aerodynamic reference point, thrust along the hub line
        moment = add(moment, cross(prm.aero_rp, force));
        force[0] += thrust;
        moment = add(moment, cross(prm.prop_pos, [thrust, 0.0, 0.0]));
        moment[0] += prm.prop_roll_arm * thrust;
        moment[2] += prm.prop_yaw_arm * thrust;
        diag.qbar_ratio = if qbar > 1.0 { qbar_ind / qbar } else { 1.0 };

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
            if self.diag.v > 15.0 && airborne_alpha > c172p::HYST_ON {
                self.stall_hyst = 1.0;
            } else if airborne_alpha < c172p::HYST_OFF {
                self.stall_hyst = 0.0;
            }
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
