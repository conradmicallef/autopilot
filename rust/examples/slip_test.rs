// Power-off glide at 66 kt, flap 30, with a commanded forward slip (cross-controls): how much steeper?
use circuitsim::aircraft::{Aircraft, Commands, ModelKind};
use circuitsim::atmosphere::{Rng, Wind, WindSettings};
use circuitsim::sixdof::{analysis::trim, quat_from_euler, N_STATE};
use circuitsim::units::{D2R, KT, PI};
fn main() {
    let v = 66.0 * KT;
    let flap: f64 = std::env::var("FLAP").ok().and_then(|s| s.parse().ok()).unwrap_or(30.0);
    for beta_deg in [0.0, 6.0, 9.0, 12.0, 15.0, 18.0, 22.0] {
        let mut ac = Aircraft::new(circuitsim::aircraft::c172_like(), ModelKind::SixDof);
        let tr = { let mut m = ac.six.as_ref().unwrap().clone(); m.x[2] = -2000.0; trim(&m, v, -0.1, flap) };
        {
            let six = ac.six.as_mut().unwrap();
            six.x = [0.0; N_STATE];
            six.x[2] = -(six.h0 + 1500.0);
            six.x[3] = v * tr.alpha.cos();
            six.x[5] = v * tr.alpha.sin();
            six.x[6..10].copy_from_slice(&quat_from_euler(0.0, tr.alpha - 0.1, PI / 2.0));
            six.ctl.de = tr.de;
            six.ctl.flap = flap;
        }
        ac.on_ground = false;
        ac.engine_failed = true;
        ac.flap_pos = flap;
        ac.shaper.flaps = flap;
        ac.shaper.theta = tr.alpha - 0.1;
        let wind = Wind::new(WindSettings { from_deg: 0.0, speed_kt: 0.0, gusts: false }, Rng::new(1.0));
        let mut cmd = Commands { theta: tr.alpha - 0.1, flaps: flap, ..Default::default() };
        let (mut vs, mut vv, mut b, mut dr, mut phi, mut n) = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        for k in 0..2500 {
            // speed on pitch, wing-low bank to keep the path straight, sideslip as commanded
            cmd.theta = (cmd.theta + 0.0015 * (ac.v - v)).clamp(-0.4, 0.3);
            cmd.beta = -beta_deg * D2R;
            cmd.phi = ac.wing_low_bank();
            ac.step(0.02, &cmd, &wind);
            if k > 1500 {
                let six = ac.six.as_ref().unwrap();
                vs += ac.vertical_speed(); vv += ac.v; b += six.diag.beta; dr += six.ctl.dr; phi += ac.phi; n += 1.0;
            }
        }
        let (vs, vv) = (vs / n, vv / n);
        println!("β cmd {:>4.0}°: achieved {:>5.1}°, rudder {:>5.1}°, bank {:>5.1}°, V {:>4.1} kt, sink {:>4.0} fpm, glide angle {:>4.1}°, ratio {:>4.1}",
            beta_deg, (b / n).abs() / D2R, (dr / n) / D2R, (phi / n) / D2R, vv / KT, -vs / 0.3048 * 60.0, (-vs / vv).asin() / D2R, vv / -vs);
    }
}
