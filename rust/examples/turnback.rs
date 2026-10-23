// The "impossible turn": engine failure in the climb-out, turn back to the runway allowed at any height.
// Sweeps failure height, wind and turn bank; reports whether the aircraft makes the runway.
use circuitsim::airfield::standard_circuit;
use circuitsim::scenarios::*;
use circuitsim::units::{D2R, FT};
use circuitsim::ModelKind;
fn main() {
    let model = ModelKind::SixDof;
    let winds = [("calm", wind(0.0, 0.0, false)), ("10 kt head", wind(90.0, 10.0, false)), ("10 kt cross", wind(360.0, 10.0, false))];
    for bank in [25.0, 35.0, 45.0] {
        println!("{} model, turn-back bank {bank}°", model.name());
        for (wn, w) in winds {
            let mut line = format!("  {wn:<12}");
            for hft in [300.0, 500.0, 600.0, 700.0, 800.0, 900.0, 1000.0] {
                let mut c = standard_circuit();
                c.crosswind_turn_alt = 1500.0 * FT; // keep climbing straight out
                let rwy: f64 = std::env::var("RWY").ok().and_then(|v| v.parse().ok()).unwrap_or(1200.0);
                let mut s = circuitsim::Simulation::new(circuitsim::SimOptions { wind: w, seed: 1.0, circuit: c, model, runway_length: rwy, short_field: false, ..Default::default() });
                s.autopilot.planner.spec.min_turnback_height = 0.0;
                let mut done = false;
                while !s.finished() && s.t < 900.0 {
                    if !done && s.phase().code() == "CLIMB" && s.aircraft.h > hft * FT {
                        s.autopilot.circuit.spec.bank_max = bank * D2R;
                        done = s.fail_engine();
                    }
                    s.step();
                }
                let eo = s.autopilot.engine_out.as_ref().expect("failure not triggered");
                let back = matches!(eo.plan, circuitsim::autopilot::EngineOutPlan::Return);
                let mark = match (back, s.passed()) {
                    (true, true) => "BACK ",
                    (true, false) => "back✗",
                    (false, true) => "ahead",
                    (false, false) => "ahd✗ ",
                };
                line += &format!(" {:>4.0}:{mark}", hft);
                if std::env::var("WHY").is_ok() {
                    let why: Vec<String> = s.evaluate().iter().map(|r| r.value.clone().unwrap_or_default()).collect();
                    eprintln!("    {wn} {hft} ft bank {bank}: x at failure {:.0} m | {}", eo.x, why.join(" | "));
                }
            }
            println!("{line}");
        }
    }
}
