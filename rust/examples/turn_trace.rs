// Trace the base-to-final turn: position, bank, heading, ground track and leg distances once a second.
// cargo run --release --example turn_trace -- [High|Standard] FROM KT
use circuitsim::airfield::{high_circuit, standard_circuit};
use circuitsim::atmosphere::WindSettings;
use circuitsim::scenarios::*;
use circuitsim::ModelKind;
fn main() {
    let a: Vec<String> = std::env::args().collect();
    let c = if a.get(1).map(|s| s.as_str()) == Some("Standard") { standard_circuit() } else { high_circuit() };
    let w = WindSettings { from_deg: a.get(2).and_then(|s| s.parse().ok()).unwrap_or(360.0), speed_kt: a.get(3).and_then(|s| s.parse().ok()).unwrap_or(20.0), gusts: false };
    let mut s = new_sim(ModelKind::SixDof, w, 1.0, c);
    let mut last = -1.0;
    let mut t_base = None;
    while !s.finished() && s.t < 1500.0 {
        s.step();
        let ph = format!("{:?}", s.phase());
        if ph == "Base" && t_base.is_none() { t_base = Some(s.t); }
        if let Some(tb) = t_base {
            if s.t - last >= 1.0 && s.t < tb + 70.0 {
                last = s.t;
                let ac = &s.aircraft;
                println!("{:6.1} {:<6} x {:6.0} y {:5.0} phi {:5.1} psi {:6.1} v {:4.1} vg {:4.1} h {:4.0} fl {:3.0} thr {:3.2}", s.t, ph, ac.x, ac.y, ac.phi.to_degrees(), ac.psi.to_degrees(), ac.v, ac.vg, ac.h / 0.3048, ac.flap_pos, ac.thr);
            }
        }
    }
}
