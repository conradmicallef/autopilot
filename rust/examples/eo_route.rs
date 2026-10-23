// Prints the power-off plan (route, intercept) through a return, for a wind and failure x on downwind.
use circuitsim::airfield::high_circuit;
use circuitsim::scenarios::*;
use circuitsim::ModelKind;
fn main() {
    let a: Vec<f64> = std::env::args().skip(1).map(|s| s.parse().unwrap()).collect();
    let (from, kt, fx) = (a.first().copied().unwrap_or(350.0), a.get(1).copied().unwrap_or(25.0), a.get(2).copied().unwrap_or(1800.0));
    let mut s = new_sim(ModelKind::SixDof, wind(from, kt, false), 1.0, high_circuit());
    let mut done = false;
    let mut k = 0;
    while !s.finished() && s.t < 900.0 {
        if !done && s.phase().code() == "DOWNWIND" && s.aircraft.x < fx { done = s.fail_engine(); }
        s.step();
        k += 1;
        if done && k % 150 == 0 && !s.aircraft.on_ground {
            if let Some(g) = &s.autopilot.glide {
                let r: Vec<String> = g.route.iter().map(|p| format!("({:.0},{:.0})", p[0], p[1])).collect();
                println!("{} t {:.0} {} x {:.0} y {:.0} h {:.0}ft ip {:.0} end {} route {}", s.autopilot.fma.lat, s.t, s.phase().code(), s.aircraft.x, s.aircraft.y, s.aircraft.h / 0.3048, g.ip_distance, g.end.name, r.join(" "));
            }
        }
    }
}
