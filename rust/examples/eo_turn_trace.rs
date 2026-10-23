// Trace a power-off return's turns: bank, heading, speed against the planned turn speed, once a second.
// cargo run --release --example eo_turn_trace -- [High|Standard] FROM KT "TRIGGER"
use circuitsim::airfield::{high_circuit, standard_circuit};
use circuitsim::atmosphere::WindSettings;
use circuitsim::scenarios::*;
use circuitsim::ModelKind;
fn main() {
    let a: Vec<String> = std::env::args().collect();
    let c = if a.get(1).map(|s| s.as_str()) == Some("High") { high_circuit() } else { standard_circuit() };
    let w = WindSettings { from_deg: a.get(2).and_then(|s| s.parse().ok()).unwrap_or(270.0), speed_kt: a.get(3).and_then(|s| s.parse().ok()).unwrap_or(0.0), gusts: false };
    let name = a.get(4).cloned().unwrap_or("downwind mid".into());
    let trig = eo_triggers().into_iter().find(|t| t.name == name).unwrap();
    let mut s = new_sim(ModelKind::SixDof, w, 4.0, c);
    let (mut done, mut last) = (false, -1.0);
    while !s.finished() && s.t < 1200.0 {
        if !done && (trig.when)(&s) { done = s.fail_engine(); }
        s.step();
        if done && s.t - last >= 1.0 && s.aircraft.h > 15.0 {
            last = s.t;
            let ac = &s.aircraft;
            let vs = ac.stall_speed(ac.flap_pos);
            println!("{:6.1} {:<10} x {:6.0} y {:5.0} h {:4.0} phi {:5.1} psi {:6.1} v {:4.1} 1.2Vs√n {:4.1} fl {:2.0}", s.t, format!("{:?}", s.phase()), ac.x, ac.y, ac.h / 0.3048, ac.phi.to_degrees(), ac.psi.to_degrees(), ac.v, 1.2 * vs / ac.phi.cos().sqrt(), ac.flap_pos);
        }
    }
    for e in &s.autopilot.events { if e.t > 0.0 { println!("  {:6.1} {:<12} x {:6.0} y {:6.0} h {:5.0} {}", e.t, e.phase, e.x, e.y, e.h / 0.3048, e.note.clone().unwrap_or_default()); } }
}
