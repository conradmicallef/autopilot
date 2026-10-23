// Debug: the glide plan at a downwind failure (High circuit).
use circuitsim::airfield::high_circuit;
use circuitsim::scenarios::*;
use circuitsim::ModelKind;
fn main() {
    let a: Vec<f64> = std::env::args().skip(1).map(|s| s.parse().unwrap()).collect();
    let circ = if std::env::var("STD").is_ok() { circuitsim::airfield::standard_circuit() } else { high_circuit() };
    let mut s = new_sim(ModelKind::SixDof, wind(a[0], a[1], false), 1.0, circ);
    while s.t < 600.0 && !(s.phase().code() == "DOWNWIND" && s.aircraft.x < a[2]) { s.step(); }
    let ac = &s.aircraft;
    println!("aircraft x {:.0} y {:.0} h {:.0} m V {:.1}", ac.x, ac.y, ac.h, ac.v);
    for e in s.autopilot.planner.ends {
        let mut d = s.autopilot.planner.spec.ip_max;
        while d >= s.autopilot.planner.spec.ip_min {
            let g = s.autopilot.planner.assess(ac, &s.wind, d, &e);
            if (d as i64) % 500 == 0 { println!("  {} ip {:>4.0}: required {:>4.0} m available {:>4.0} surplus {:>5.0} route {:?}", e.name, d, g.required, g.available, g.surplus, g.route.iter().map(|p| (p[0].round(), p[1].round())).collect::<Vec<_>>()); }
            d -= 50.0;
        }
    }
}
