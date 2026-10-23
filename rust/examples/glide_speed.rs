// Steady power-off glide (engine failed: windmilling prop) against airspeed, flap 0 and 20, from an exact
// trim of the 6DOF model, compared with the planner's estimate; and the best speed in head/tailwind.
use circuitsim::aircraft::{Aircraft, ModelKind};
use circuitsim::sixdof::{quat_from_euler, Controls, N_STATE};
use circuitsim::units::KT;
fn glide_trim(m: &circuitsim::sixdof::SixDof, v: f64, flap: f64) -> (f64, f64) {
    // unknowns: alpha, de, gamma; residuals: u_dot, w_dot, q_dot
    let res = |z: [f64; 3]| -> [f64; 3] {
        let mut x = [0.0; N_STATE];
        x[2] = -2000.0;
        x[3] = v * z[0].cos();
        x[5] = v * z[0].sin();
        x[6..10].copy_from_slice(&quat_from_euler(0.0, z[0] + z[2], 0.0));
        let u = Controls { de: z[1], thr: 0.0, flap, engine_failed: true, ..Default::default() };
        let (d, _) = m.derivs(&x, &u, [0.0; 3]);
        [d[3], d[5], d[11]]
    };
    let mut z = [0.08, -0.05, -0.1];
    for _ in 0..60 {
        let r = res(z);
        let mut j = [[0.0; 3]; 3];
        for k in 0..3 {
            let mut zp = z; zp[k] += 1e-6;
            let rp = res(zp);
            for i in 0..3 { j[i][k] = (rp[i] - r[i]) / 1e-6; }
        }
        // solve 3x3 by Cramer
        let det = |a: [[f64; 3]; 3]| a[0][0] * (a[1][1] * a[2][2] - a[1][2] * a[2][1]) - a[0][1] * (a[1][0] * a[2][2] - a[1][2] * a[2][0]) + a[0][2] * (a[1][0] * a[2][1] - a[1][1] * a[2][0]);
        let d0 = det(j);
        for k in 0..3 {
            let mut a = j;
            for i in 0..3 { a[i][k] = -r[i]; }
            z[k] += det(a) / d0;
        }
    }
    let gamma = z[2];
    (-v * gamma.sin(), 1.0 / (-gamma).tan()) // sink m/s, glide ratio
}
fn main() {
    let ac = Aircraft::new(circuitsim::aircraft::c172_like(), ModelKind::SixDof);
    let mut m = ac.six.as_ref().unwrap().clone();
    m.x[2] = -2000.0;
    let mut pac = Aircraft::new(circuitsim::aircraft::c172_like(), ModelKind::PointMass);
    pac.engine_failed = true;
    for flap in [0.0, 20.0] {
        println!("flap {flap}:  IAS kt  sink fpm  L/D (6DOF trim)  L/D (planner)");
        let mut rows = vec![];
        for kt in (52..=86).step_by(2) {
            let v = kt as f64 * KT;
            let (s, ld) = glide_trim(&m, v, flap);
            println!("          {:>4}   {:>6.0}      {:>5.2}           {:>5.2}", kt, s / 0.3048 * 60.0, ld, pac.glide_ratio(v, flap));
            rows.push((v, s));
        }
        for w_kt in [-15.0, 0.0, 10.0, 15.0, 25.0] {
            let w = w_kt * KT;
            let best = rows.iter().max_by(|a, b| ((a.0 - w) / a.1).partial_cmp(&((b.0 - w) / b.1)).unwrap()).unwrap();
            println!("   {} {:>2.0} kt: best {:.0} kt, ground glide ratio {:.1}", if w_kt >= 0.0 { "head" } else { "tail" }, w_kt.abs(), best.0 / KT, (best.0 - w) / best.1);
        }
        let minsink = rows.iter().min_by(|a, b| a.1.partial_cmp(&b.1).unwrap()).unwrap();
        println!("   minimum sink {:.0} fpm at {:.0} kt", minsink.1 / 0.3048 * 60.0, minsink.0 / KT);
    }
}
