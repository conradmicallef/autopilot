// Check the glide planner's energy model against the flight: through a power-off glide, accumulate the
// energy height (h + V²/2g) actually lost and what the planner's model predicts for the same flown path
// (straight: ds / L/D; banked: ds · n / L/D at the turn's equivalent 1 g speed), split by bank band.
use circuitsim::airfield::high_circuit;
use circuitsim::scenarios::*;
use circuitsim::units::{FT, G, KT, D2R};
use circuitsim::ModelKind;
fn main() {
    let a: Vec<String> = std::env::args().skip(1).collect();
    let tr = eo_triggers().into_iter().find(|t| t.name == a.get(0).map(|s| s.as_str()).unwrap_or("downwind mid")).unwrap();
    let mut s = new_sim(ModelKind::SixDof, wind(270.0, 0.0, false), 1.0, high_circuit());
    let mut done = false;
    let mut prev: Option<(f64, f64, f64)> = None;
    let mut bands = [(0.0f64, 0.0f64, 0.0f64); 4]; // (actual, predicted, distance) for |bank| <10, 10–30, 30–40, >40
    while !s.finished() && s.t < 1200.0 {
        if !done && (tr.when)(&s) { done = s.fail_engine(); }
        s.step();
        let ac = &s.aircraft;
        if !done || ac.on_ground || ac.h < 60.0 * FT || s.phase().code() == "FLARE" { prev = None; continue; }
        if ac.flap_pos > 1.0 { prev = None; continue; }
        let e = ac.h + ac.v * ac.v / (2.0 * G);
        if let Some((pe, px, py)) = prev {
            let ds = (ac.x - px).hypot(ac.y - py); // calm air: ground = air distance
            let bank = ac.phi.abs();
            let n = 1.0 / bank.cos();
            let pred = ds * n / ac.glide_ratio(ac.v / n.sqrt(), 0.0);
            let b = if bank < 10.0 * D2R { 0 } else if bank < 30.0 * D2R { 1 } else if bank < 40.0 * D2R { 2 } else { 3 };
            bands[b].0 += pe - e;
            bands[b].1 += pred;
            bands[b].2 += ds;
        }
        prev = Some((e, ac.x, ac.y));
    }
    println!("bank band   distance m  actual m  predicted m  actual/predicted");
    for (i, n) in ["<10", "10-30", "30-40", ">40"].iter().enumerate() {
        let (a, p, d) = bands[i];
        if d > 0.0 { println!("{n:>8}  {d:9.0}  {a:8.1}  {p:10.1}  {:6.2}", a / p); }
    }
    let _ = KT;
}
