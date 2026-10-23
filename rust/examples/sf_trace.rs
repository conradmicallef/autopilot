use circuitsim::airfield::high_circuit;
use circuitsim::scenarios::*;
use circuitsim::{ModelKind, SimOptions, Simulation};
fn main() {
    let seed: f64 = std::env::args().nth(1).map(|s| s.parse().unwrap()).unwrap_or(3.0);
    let mut s = Simulation::new(SimOptions { wind: wind(240.0, 10.0, true), seed, circuit: high_circuit(), model: ModelKind::SixDof, runway_length: 450.0, short_field: true, ..Default::default() });
    let mut k = 0;
    while !s.finished() && s.t < 1200.0 {
        s.step(); k += 1;
        let a = &s.aircraft;
        let ph = s.phase().code();
        if (ph == "FLARE" || (ph == "FINAL" && a.h < 15.0)) && k % 5 == 0 {
            let six = a.six.as_ref().unwrap();
            println!("t {:.1} {} x {:.0} h {:.2} V {:.1}kt vs {:.2} θ {:.1} θcmd {:.1} α {:.1} de {:.1} σ {:.2} add {:.1}", s.t, ph, a.x, a.h, a.v / 0.5144, a.vertical_speed(), a.theta.to_degrees(), a.shaper.theta.to_degrees(), six.diag.alpha.to_degrees(), six.ctl.de.to_degrees(), s.autopilot.gust.sigma(), s.autopilot.gust_add()/0.5144);
        }
    }
    println!("{:?}", s.evaluate().iter().map(|r| r.value.clone().unwrap_or_default()).collect::<Vec<_>>());
}
