// Debug: the glide plan at a climb-out failure.
use circuitsim::airfield::standard_circuit;
use circuitsim::scenarios::*;
use circuitsim::units::FT;
use circuitsim::ModelKind;
fn main() {
    let hft: f64 = std::env::args().nth(1).map(|s| s.parse().unwrap()).unwrap_or(600.0);
    let mut c = standard_circuit();
    c.crosswind_turn_alt = 1500.0 * FT;
    let mut s = new_sim(ModelKind::SixDof, wind(0.0, 0.0, false), 1.0, c);
    s.autopilot.planner.spec.min_turnback_height = 0.0;
    while s.t < 600.0 {
        if s.phase().code() == "CLIMB" && s.aircraft.h > hft * FT { break; }
        s.step();
    }
    let ac = &s.aircraft;
    for e in s.autopilot.planner.ends {
        let g = s.autopilot.planner.plan_for(ac, &s.wind, &e);
        println!("end {} reachable {} ip {:.0} required {:.0} m available {:.0} m surplus {:.0} route {:?} | flap {:.0} L/D(vbg,0) {:.1}", e.name, g.reachable, g.ip_distance, g.required, g.available, g.surplus, g.route.iter().map(|p| (p[0].round(), p[1].round())).collect::<Vec<_>>(), ac.flap_pos, ac.glide_ratio(s.autopilot.planner.spec.vbg, 0.0));
    }
}
