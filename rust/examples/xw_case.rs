// One crosswind-leg failure: wind from, kt, delay after entering crosswind (s). Env VY overrides climb speed (kt).
use circuitsim::airfield::high_circuit;
use circuitsim::autopilot::Phase;
use circuitsim::scenarios::*;
use circuitsim::units::{FT, KT};
use circuitsim::ModelKind;
fn main() {
    let a: Vec<String> = std::env::args().skip(1).collect();
    let mut c = high_circuit();
    if let Ok(v) = std::env::var("VY") { c.vy = v.parse::<f64>().unwrap() * KT; }
    let delay: f64 = a[2].parse().unwrap();
    let mut s = new_sim(ModelKind::SixDof, wind(a[0].parse().unwrap(), a[1].parse().unwrap(), false), 1.0, c);
    let (mut t_xw, mut done, mut k) = (None, false, 0);
    while !s.finished() && s.t < 1200.0 {
        if s.phase() == Phase::Crosswind && t_xw.is_none() { t_xw = Some(s.t); }
        if !done { if let Some(t0) = t_xw { if s.t >= t0 + delay && s.phase() == Phase::Crosswind {
            done = s.fail_engine();
            let ac = &s.aircraft;
            println!("failure at x {:.0} y {:.0} h {:.0} ft", ac.x, ac.y, ac.h / FT);
            for e in s.autopilot.planner.ends { let g = s.autopilot.planner.plan_for(ac, &s.wind, &e); println!("  end {}: reachable {} ip {:.0} required {:.0} available {:.0} surplus {:.0}", e.name, g.reachable, g.ip_distance, g.required, g.available, g.surplus); }
        }}}
        s.step(); k += 1;
        if done && k % 100 == 0 && !s.aircraft.on_ground {
            let ac = &s.aircraft;
            println!("  t {:.0} {:10} {:9} x {:5.0} y {:5.0} h {:4.0} trk {:3.0} bank {:4.0} V {:3.0} flap {:2.0} glide {:?}", s.t, s.phase().code(), s.autopilot.fma.lat, ac.x, ac.y, ac.h / FT, circuitsim::units::heading_deg(ac.track(&s.wind)), ac.phi.to_degrees(), ac.v / KT, ac.flap_pos, s.autopilot.glide.as_ref().map(|g| (g.ip_distance.round(), g.end.name, g.surplus.round())));
        }
    }
    for e in &s.autopilot.events { println!("  event {:.0} {} h {:.0} ft {}", e.t, e.phase, e.h / FT, e.note.clone().unwrap_or_default()); }
    if let Some(td) = &s.touchdown { println!("TD ({:.0},{:.0}) V {:.0}", td.x, td.y, td.v / KT); }
}
