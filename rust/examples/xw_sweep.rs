// Engine failure at points along the crosswind leg: outcome per circuit, wind and delay.
use circuitsim::airfield::{high_circuit, standard_circuit};
use circuitsim::scenarios::*;
use circuitsim::units::FT;
use circuitsim::autopilot::Phase;
use circuitsim::ModelKind;
fn main() {
    let a: Vec<String> = std::env::args().skip(1).collect();
    let verbose = a.iter().any(|x| x == "-v");
    let mut pass = 0; let mut n = 0;
    let env = |k: &str| std::env::var(k).ok().and_then(|v| v.parse::<f64>().ok());
    let tweak = |mut c: circuitsim::airfield::CircuitSpec| { if let Some(a) = env("XW_ALT") { c.crosswind_turn_alt = a * FT; } if let Some(v) = env("VY") { c.v_climb = v * circuitsim::units::KT; } if let Some(v) = env("CLR") { c.crosswind_clearance = v; } c };
    let only_high = env("ONLY_HIGH").is_some();
    for (cn, c) in [("high", tweak(high_circuit())), ("std", tweak(standard_circuit()))] {
        if only_high && cn == "std" { continue; }
        for (from, kt) in [(90.0, 8.0), (90.0, 0.0), (90.0, 15.0), (270.0, 8.0), (360.0, 10.0), (180.0, 10.0)] {
            let mut line = format!("{cn:4} {from:3}/{kt:2}: ");
            for delay in [0.0, 5.0, 10.0, 15.0, 20.0, 25.0] {
                let mut s = new_sim(ModelKind::SixDof, wind(from, kt, false), 1.0, c.clone());
                let mut t_xw = None; let mut done = false; let mut info = String::new();
                while !s.finished() && s.t < 1200.0 {
                    if s.phase() == Phase::Crosswind && t_xw.is_none() { t_xw = Some(s.t); }
                    if !done { if let Some(t0) = t_xw { if s.t >= t0 + delay && (s.phase() == Phase::Crosswind) {
                        done = s.fail_engine();
                        info = format!("x {:.0} y {:.0} h {:.0}", s.aircraft.x, s.aircraft.y, s.aircraft.h / FT);
                    }}}
                    s.step();
                }
                if !done { line += "  -  "; continue; }
                n += 1;
                let ok = s.evaluate().iter().all(|r| r.pass != Some(false));
                if ok { pass += 1; }
                let stop = format!("({:.0},{:.0})", s.aircraft.x, s.aircraft.y);
                line += &format!(" {}{:>12}", if ok { "OK" } else { "XX" }, stop);
                if verbose { if let Some(td) = &s.touchdown { print!("  TD ({:.0},{:.0}) V {:.0} crab {:.0} ", td.x, td.y, td.v / circuitsim::units::KT, td.crab.to_degrees()); } println!("  {cn} {from}/{kt} d{delay}: fail at {info}; {}", s.evaluate().iter().filter(|r| r.pass == Some(false)).map(|r| format!("{}={}", r.label, r.value.clone().unwrap_or_default())).collect::<Vec<_>>().join(", ")); }
            }
            println!("{line}");
        }
    }
    println!("{pass}/{n}");
}
