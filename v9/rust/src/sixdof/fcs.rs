//! Inner-loop control laws for the 6DOF model.
//!
//! The autopilot's outer loops command pitch attitude, bank and sideslip (rate-limited by the
//! `CommandShaper`). These laws close the loop on the rigid body:
//! * Pitch: attitude error → pitch-rate demand (+ turn compensation) → elevator, with pitch-rate damping
//!   and an integrator that finds the trim elevator.
//! * Roll: bank error → roll-rate demand → aileron, with an integrator for propeller torque.
//! * Yaw: rudder holds the commanded sideslip (zero = coordinated) with a yaw damper.
//! * Ground: nose-wheel steering and rudder track the runway heading; elevator neutral until rotation.
//!
//! Gains are scheduled with dynamic pressure, so control power stays roughly constant across speeds.

use super::SixDof;
use crate::aircraft::{Aircraft, Commands};
use crate::units::{clamp, wrap_pi, D2R, G, PI};
use serde::Serialize;

#[derive(Clone, Debug, Serialize)]
pub struct FcsGains {
    pub k_theta: f64,
    pub k_q: f64,
    pub ki_theta: f64,
    pub k_phi: f64,
    pub k_p: f64,
    pub ki_phi: f64,
    pub k_beta: f64,
    pub ki_beta: f64,
    pub k_r: f64,
    /// Reference airspeed for gain scheduling, m/s.
    pub v_ref: f64,
}

impl Default for FcsGains {
    fn default() -> Self {
        FcsGains {
            k_theta: 2.0,
            k_q: 0.9,
            ki_theta: 1.2,
            k_phi: 3.0,
            k_p: 0.4,
            ki_phi: 0.3,
            k_beta: 1.5,
            ki_beta: 1.0,
            k_r: 0.8,
            v_ref: 30.0,
        }
    }
}

#[derive(Clone, Copy, Debug, Default, Serialize)]
pub struct Targets {
    pub de: f64,
    pub da: f64,
    pub dr: f64,
    pub steer: f64,
}

#[derive(Clone, Debug, Default, Serialize)]
pub struct Fcs {
    pub gains: FcsGains,
    pub i_theta: f64,
    pub i_phi: f64,
    pub i_beta: f64,
    pub last: Targets,
}

impl Fcs {
    pub fn on_touchdown(&mut self) {
        self.i_beta = 0.0;
    }

    pub fn update(&mut self, six: &SixDof, ac: &Aircraft, cmd: &Commands, h: f64) -> Targets {
        let g = self.gains.clone();
        let (phi, theta, psi_ned) = super::euler(&six.x[6..10]);
        let (p, q, r) = (six.x[10], six.x[11], six.x[12]);
        let v = six.diag.v;
        let vs = v.max(12.0);
        let sched = clamp((g.v_ref / vs).powi(2), 0.3, 4.0);
        let sh = &ac.shaper;
        let mut t = Targets::default();

        // --- pitch
        let rotating = cmd.allow_liftoff || !ac.on_ground;
        if !rotating {
            self.i_theta = 0.0;
            t.de = 0.0;
        } else {
            let e = sh.theta - theta;
            let q_turn = if ac.on_ground { 0.0 } else { G * phi.sin().powi(2) / (vs * phi.cos().max(0.3)) };
            // feed-forward the shaped command's own rate so slews (rotation, flare) are tracked without lag
            let q_cmd = clamp(g.k_theta * e + sh.theta_rate, -12.0 * D2R, 12.0 * D2R) + q_turn;
            let i_lim = 0.4 / (g.ki_theta * sched);
            self.i_theta = clamp(self.i_theta + e * h, -i_lim, i_lim);
            // nose-up needs trailing edge up: negative elevator
            t.de = -sched * (g.k_q * (q_cmd - q) + g.ki_theta * self.i_theta);
        }

        // --- roll (autopilot bank is + left wing down; body roll is + right wing down)
        let phi_ref = if ac.on_ground { 0.0 } else { -sh.phi };
        let e_phi = phi_ref - phi;
        let p_cmd = clamp(g.k_phi * e_phi, -20.0 * D2R, 20.0 * D2R);
        let i_lim = 0.15 / (g.ki_phi * sched);
        self.i_phi = clamp(self.i_phi + e_phi * h, -i_lim, i_lim);
        // feed-forward: cancel the rolling moment from sideslip (dihedral) and rudder, known from the model
        let prm = &six.prm;
        let da_ff = if ac.on_ground {
            0.0
        } else {
            -(prm.cl_beta * clamp(six.diag.beta, -0.6, 0.6) + prm.cl_dr * six.ctl.dr) / prm.cl_da
        };
        t.da = da_ff + sched * (g.k_p * (p_cmd - p) + g.ki_phi * self.i_phi);

        if ac.on_ground {
            // --- ground steering: target heading from the aircraft's runway / field logic
            let psi_cmd_ned = PI / 2.0 - ac.ground_heading_cmd();
            let e = wrap_pi(psi_cmd_ned - psi_ned);
            t.steer = clamp(1.2 * e - 0.4 * r, -0.44, 0.44);
            t.dr = clamp(2.0 * e - 0.6 * r, -0.44, 0.44);
            self.i_beta = 0.0;
        } else {
            // --- yaw: sideslip hold with yaw damper; + rudder yaws the nose right
            let r_turn = G * phi.sin() / vs;
            let e_beta = sh.beta - six.diag.beta;
            let i_lim = 0.25 / (g.ki_beta * sched);
            self.i_beta = clamp(self.i_beta + e_beta * h, -i_lim, i_lim);
            t.dr = sched * (-g.k_beta * e_beta - g.ki_beta * self.i_beta + g.k_r * (r_turn - r));
            t.steer = 0.0;
        }
        self.last = t;
        t
    }
}
