// Power-off returns in strong winds: pass rate and worst centreline overshoot on final.
use circuitsim::airfield::{high_circuit, standard_circuit};
use circuitsim::scenarios::*;
use circuitsim::ModelKind;
fn main() {
    for model in [ModelKind::SixDof] {
        let (mut pass, mut n, mut worst, mut sum) = (0, 0, 0.0_f64, 0.0);
        let mut fails = vec![];
        for circuit in [standard_circuit(), high_circuit()] {
            for from in (0..360).step_by(45) {
                for kt in [15.0, 20.0, 25.0] {
                    for fx in [1800.0, 1000.0, 0.0] {
                        let mut s = new_sim(model, wind(from as f64, kt, false), 1.0, circuit.clone());
                        let mut done = false;
                        let mut over = 0.0_f64;
                        let mut prev_side = 0.0;
                        while !s.finished() && s.t < 900.0 {
                            if !done && s.phase().code() == "DOWNWIND" && s.aircraft.x < fx { done = s.fail_engine(); }
                            s.step();
                            let ph = s.phase().code();
                            if ph == "EO_FINAL" || ph == "FLARE" {
                                let y = s.aircraft.y;
                                if prev_side == 0.0 { prev_side = y.signum(); }
                                if y.signum() != prev_side { over = over.max(y.abs()); }
                            }
                        }
                        n += 1;
                        if s.passed() { pass += 1; } else { {
                            let why: Vec<String> = s.evaluate().iter().filter(|r| !r.info && r.pass != Some(true)).map(|r| r.label.clone()).collect();
                            fails.push(format!("{} {:03}/{} fx{}: {}", circuit.name, from, kt, fx, why.join("+")));
                        } }
                        worst = worst.max(over);
                        sum += over;
                    }
                }
            }
        }
        println!("{}: {pass}/{n} passed, overshoot through the centreline worst {:.0} m, mean {:.0} m", model.name(), worst, sum / n as f64);
        if std::env::var("SHOW").is_ok() { for f in &fails { println!("  {f}"); } }
    }
}
