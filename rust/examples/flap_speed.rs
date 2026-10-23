// Flap-limit check: highest airspeed seen with flap above 10° (C172P Vfe 85 KIAS) and with any flap
// (110 KIAS), over the engine and scenario suites.
use circuitsim::scenarios::*;
use circuitsim::units::KT;
use circuitsim::ModelKind;
fn main() {
    for model in [ModelKind::SixDof] {
        let (mut over85, mut over110, mut n, mut worst85, mut worst110) = (0, 0, 0, 0.0_f64, 0.0_f64);
        let mut check = |s: &mut circuitsim::Simulation, trig: Option<&EoTrigger>| {
            let mut done = false; let (mut e85, mut e110) = (false, false);
            while !s.finished() && s.t < 1200.0 {
                if let Some(t) = trig { if !done && (t.when)(s) { done = s.fail_engine(); } }
                s.step();
                let ac = &s.aircraft;
                if ac.flap_pos > 10.5 && ac.v > 85.0 * KT { e85 = true; worst85 = worst85.max(ac.v / KT); }
                if ac.flap_pos > 0.5 && ac.v > 110.0 * KT { e110 = true; worst110 = worst110.max(ac.v / KT); }
            }
            n += 1; over85 += e85 as i32; over110 += e110 as i32;
        };
        for c in circuits() {
            for (w, seed, ga) in normal_cases() { let mut s = new_sim(model, w, seed, c.clone()); let _ = ga; check(&mut s, None); }
            for w in eo_winds() { for t in eo_triggers() { let mut s = new_sim(model, w, 4.0, c.clone()); check(&mut s, Some(&t)); } }
        }
        println!("{}: {n} flights; flap >10° above 85 kt in {over85} (worst {worst85:.0} kt); any flap above 110 kt in {over110} (worst {worst110:.0} kt)", model.name());
    }
}
