// One engine-failure case: circuit (std|high), wind from, wind kt, trigger name. Prints the decision and outcome.
use circuitsim::airfield::{high_circuit, standard_circuit};
use circuitsim::scenarios::*;
use circuitsim::ModelKind;
fn main() {
    let a: Vec<String> = std::env::args().skip(1).collect();
    let circ = if a[0] == "std" { standard_circuit() } else { high_circuit() };
    let w = wind(a[1].parse().unwrap(), a[2].parse().unwrap(), false);
    let model = ModelKind::SixDof;
    let tr = eo_triggers().into_iter().find(|t| t.name == a[3]).expect("trigger");
    let mut s = new_sim(model, w, 1.0, circ);
    s.wind.settings = w;
    let mut done = false;
    let mut k = 0;
    while !s.finished() && s.t < 1200.0 {
        if !done && (tr.when)(&s) {
            done = s.fail_engine();
            let ac = &s.aircraft;
            println!("failure at x {:.0} y {:.0} h {:.0} ft track {:.0}°", ac.x, ac.y, ac.h / 0.3048, circuitsim::units::heading_deg(ac.track(&s.wind)));
            for e in s.autopilot.planner.ends {
                let g = s.autopilot.planner.plan_for(ac, s.autopilot.gust.mean_wind(), &e);
                println!("  end {}: reachable {} ip {:.0} required {:.0} m available {:.0} m surplus {:.0}", e.name, g.reachable, g.ip_distance, g.required, g.available, g.surplus);
            }
        }
        s.step(); k += 1;
        if done && k % 50 == 0 && !s.aircraft.on_ground && (s.t < 115.0 || std::env::var("FULL").is_ok()) {
            println!("  t {:.0} {} {} x {:.0} y {:.0} h {:.0} ft trk {:.0} bank {:.0} V {:.0} glide {:?}", s.t, s.phase().code(), s.autopilot.fma.lat, s.aircraft.x, s.aircraft.y, s.aircraft.h / 0.3048, circuitsim::units::heading_deg(s.aircraft.track(&s.wind)), s.aircraft.phi.to_degrees(), s.aircraft.v/0.5144, s.autopilot.glide.as_ref().map(|g| (g.ip_distance, g.end.name, g.surplus.round())));
        }
    }
    for e in &s.autopilot.events { if let Some(n) = &e.note { println!("  event {} {} at h {:.0} ft: {}", e.t.round(), e.phase, e.h / 0.3048, n); } }
    println!("{}", s.evaluate().iter().map(|r| r.value.clone().unwrap_or_default()).collect::<Vec<_>>().join(" | "));
}
