//! Trim solver and small-perturbation (natural mode) analysis for the 6DOF model. Native only.

use super::*;
use nalgebra::DMatrix;

#[derive(Clone, Debug, Serialize)]
pub struct Trim {
    pub v: f64,
    pub gamma: f64,
    pub flap: f64,
    pub alpha: f64,
    pub de: f64,
    pub thr: f64,
    pub residual: f64,
}

/// Airborne model (well above the ground) for analysis.
pub fn free_air_model(spec: &AircraftSpec) -> SixDof {
    let mut s = SixDof::on_runway(spec);
    s.x[2] = -2000.0;
    s
}

fn trimmed_state(v: f64, gamma: f64, alpha: f64) -> State {
    let mut x = [0.0; N_STATE];
    x[2] = -2000.0;
    x[3] = v * alpha.cos();
    x[5] = v * alpha.sin();
    let q = quat_from_euler(0.0, alpha + gamma, 0.0);
    x[6..10].copy_from_slice(&q);
    x
}

/// Longitudinal trim: find α, elevator and throttle for steady wings-level flight at (V, γ, flap).
/// Throttle may go negative if the condition needs less than idle (reported, not clamped).
pub fn trim(model: &SixDof, v: f64, gamma: f64, flap: f64) -> Trim {
    let mut z = [0.05, -0.02, 0.4]; // alpha, de, thr
    let resid = |z: &[f64; 3]| -> [f64; 3] {
        let x = trimmed_state(v, gamma, z[0]);
        let u = Controls { de: z[1], thr: z[2], flap, ..Default::default() };
        // thrust is thr · max_thrust; allow negative for the solve
        let (d, _) = model.derivs(&x, &u, [0.0; 3]);
        [d[3], d[5], d[11]]
    };
    let mut res = resid(&z);
    for _ in 0..50 {
        let mut j = [[0.0; 3]; 3];
        for k in 0..3 {
            let mut zp = z;
            let dz = 1e-6;
            zp[k] += dz;
            let rp = resid(&zp);
            for i in 0..3 {
                j[i][k] = (rp[i] - res[i]) / dz;
            }
        }
        let jm = DMatrix::from_fn(3, 3, |i, k| j[i][k]);
        let rv = DMatrix::from_fn(3, 1, |i, _| -res[i]);
        let step = match jm.lu().solve(&rv) {
            Some(s) => s,
            None => break,
        };
        for k in 0..3 {
            z[k] += step[(k, 0)];
        }
        res = resid(&z);
        if res.iter().map(|r| r.abs()).sum::<f64>() < 1e-9 {
            break;
        }
    }
    Trim { v, gamma, flap, alpha: z[0], de: z[1], thr: z[2], residual: res.iter().map(|r| r.abs()).sum() }
}

#[derive(Clone, Debug, Serialize)]
pub struct Mode {
    pub name: String,
    pub re: f64,
    pub im: f64,
    /// Natural frequency, rad/s (oscillatory modes).
    pub wn: Option<f64>,
    pub zeta: Option<f64>,
    pub period: Option<f64>,
    /// Time constant (real modes): −1/λ; negative means divergent (time to e-fold).
    pub tau: Option<f64>,
}

/// Linearise about a trim using body velocities, rates and Euler angles, then split longitudinal
/// (u, w, q, θ) and lateral (v, p, r, φ) dynamics and return their eigenvalues as named modes.
pub fn modes(model: &SixDof, tr: &Trim) -> Vec<Mode> {
    let theta0 = tr.alpha + tr.gamma;
    let u = Controls { de: tr.de, thr: tr.thr, flap: tr.flap, ..Default::default() };
    // z = [u, v, w, p, q, r, φ, θ, ψ]
    let z0 = [tr.v * tr.alpha.cos(), 0.0, tr.v * tr.alpha.sin(), 0.0, 0.0, 0.0, 0.0, theta0, 0.0];
    let f = |z: &[f64; 9]| -> [f64; 9] {
        let mut x = [0.0; N_STATE];
        x[2] = -2000.0;
        x[3] = z[0];
        x[4] = z[1];
        x[5] = z[2];
        x[10] = z[3];
        x[11] = z[4];
        x[12] = z[5];
        let q = quat_from_euler(z[6], z[7], z[8]);
        x[6..10].copy_from_slice(&q);
        let (d, _) = model.derivs(&x, &u, [0.0; 3]);
        let (phi, th) = (z[6], z[7]);
        let (p, q_, r) = (z[3], z[4], z[5]);
        [
            d[3],
            d[4],
            d[5],
            d[10],
            d[11],
            d[12],
            p + (q_ * phi.sin() + r * phi.cos()) * th.tan(),
            q_ * phi.cos() - r * phi.sin(),
            (q_ * phi.sin() + r * phi.cos()) / th.cos(),
        ]
    };
    let mut a = [[0.0; 9]; 9];
    for k in 0..9 {
        let mut zp = z0;
        let mut zm = z0;
        let dz = 1e-5 * if k < 3 { tr.v } else { 1.0 };
        zp[k] += dz;
        zm[k] -= dz;
        let fp = f(&zp);
        let fm = f(&zm);
        for i in 0..9 {
            a[i][k] = (fp[i] - fm[i]) / (2.0 * dz);
        }
    }
    let lon = [0usize, 2, 4, 7];
    let lat = [1usize, 3, 5, 6];
    let mut out = Vec::new();
    for (idx, is_lon) in [(lon, true), (lat, false)] {
        let m = DMatrix::from_fn(4, 4, |i, k| a[idx[i]][idx[k]]);
        let mut ev: Vec<(f64, f64)> = m.complex_eigenvalues().iter().map(|c| (c.re, c.im)).collect();
        ev.sort_by(|x, y| y.0.abs().partial_cmp(&x.0.abs()).unwrap());
        let mut osc: Vec<(f64, f64)> = ev.iter().filter(|e| e.1 > 1e-6).cloned().collect();
        let real: Vec<(f64, f64)> = ev.iter().filter(|e| e.1.abs() <= 1e-6).cloned().collect();
        osc.sort_by(|x, y| (y.0 * y.0 + y.1 * y.1).partial_cmp(&(x.0 * x.0 + x.1 * x.1)).unwrap());
        let named = |name: &str, (re, im): (f64, f64)| -> Mode {
            if im.abs() > 1e-6 {
                let wn = (re * re + im * im).sqrt();
                Mode { name: name.into(), re, im, wn: Some(wn), zeta: Some(-re / wn), period: Some(2.0 * PI / im), tau: None }
            } else {
                Mode { name: name.into(), re, im, wn: None, zeta: None, period: None, tau: Some(-1.0 / re) }
            }
        };
        if is_lon {
            if osc.len() == 2 {
                out.push(named("Short period", osc[0]));
                out.push(named("Phugoid", osc[1]));
            } else {
                for (i, e) in ev.iter().enumerate() {
                    out.push(named(&format!("Longitudinal {}", i + 1), *e));
                }
            }
        } else {
            if let Some(d) = osc.first() {
                out.push(named("Dutch roll", *d));
            }
            let mut real = real.clone();
            real.sort_by(|x, y| x.0.partial_cmp(&y.0).unwrap());
            if real.len() == 2 {
                out.push(named("Roll subsidence", real[0]));
                out.push(named("Spiral", real[1]));
            } else {
                for (i, e) in real.iter().enumerate() {
                    out.push(named(&format!("Lateral real {}", i + 1), *e));
                }
            }
        }
    }
    out
}
