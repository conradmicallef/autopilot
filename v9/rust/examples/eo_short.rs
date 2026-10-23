use circuitsim::airfield::high_circuit;
use circuitsim::scenarios::*;
use circuitsim::{ModelKind, SimOptions, Simulation};
fn main() {
    for model in [ModelKind::SixDof, ModelKind::PointMass] {
        let (mut p, mut n) = (0, 0);
        for w in [wind(270.0, 0.0, false), wind(90.0, 10.0, false)] {
            for tr in eo_triggers() {
                let mut s = Simulation::new(SimOptions { wind: w, seed: 4.0, circuit: high_circuit(), model, runway_length: 450.0, short_field: true });
                let mut done = false;
                while !s.finished() && s.t < 1200.0 { if !done && (tr.when)(&s) { done = s.fail_engine(); } s.step(); }
                n += 1; if s.passed() { p += 1; } else {
                    println!("  {} {:03}/{} {:<14} FAIL | {}", model.name(), w.from_deg, w.speed_kt, tr.name,
                      s.evaluate().iter().map(|r| r.value.clone().unwrap_or("—".into())).collect::<Vec<_>>().join(" | "));
                }
            }
        }
        println!("{}: {p}/{n} engine failures passed on the 450 m strip", model.name());
    }
}
