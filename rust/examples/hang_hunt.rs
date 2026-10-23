// Look for hangs/panics: engine failure at many points of the climb-out and crosswind leg.
use circuitsim::airfield::{high_circuit, standard_circuit};
use circuitsim::scenarios::*;
use circuitsim::units::FT;
use circuitsim::ModelKind;
use std::time::Instant;
fn main() {
    for (cn, c) in [("high", high_circuit()), ("std", standard_circuit())] {
        for (from, kt) in [(90.0, 8.0), (90.0, 0.0), (270.0, 8.0), (360.0, 10.0)] {
            for step in 0..40 {
                let mut s = new_sim(ModelKind::SixDof, wind(from, kt, false), 1.0, c.clone());
                let mut done = false; let mut fail_at = String::new();
                let t0 = Instant::now();
                let mut k = 0u64;
                while !s.finished() && s.t < 1200.0 {
                    let ph = s.phase().code();
                    if !done && (ph == "CLIMB" || ph == "CROSSWIND") && s.t > 30.0 + step as f64 * 2.0 {
                        fail_at = format!("{ph} t {:.0} x {:.0} y {:.0} h {:.0} ft", s.t, s.aircraft.x, s.aircraft.y, s.aircraft.h / FT);
                        done = s.fail_engine();
                    }
                    s.step(); k += 1;
                    if t0.elapsed().as_secs_f64() > 5.0 { println!("SLOW {cn} {from}/{kt} {fail_at} at t {:.0} {}", s.t, s.phase().code()); break; }
                    if !s.aircraft.x.is_finite() || !s.aircraft.h.is_finite() { println!("NaN {cn} {from}/{kt} {fail_at}"); break; }
                }
                if !s.finished() { println!("NOT FINISHED {cn} {from}/{kt} {fail_at}: phase {} t {:.0} x {:.0} y {:.0} h {:.0} V {:.1} on_ground {}", s.phase().code(), s.t, s.aircraft.x, s.aircraft.y, s.aircraft.h, s.aircraft.v, s.aircraft.on_ground); }
                let _ = k;
            }
        }
    }
    println!("done");
}
