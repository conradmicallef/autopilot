// How long the power-off returns are lined up before touchdown: the distance flown, back from the
// touchdown, with the ground track within 5° of the runway heading and within 15 m of the centreline.
use circuitsim::airfield::{high_circuit, standard_circuit};
use circuitsim::atmosphere::{Rng, WindSettings};
use circuitsim::autopilot::EngineOutPlan;
use circuitsim::scenarios::*;
use circuitsim::simulation::{AirStart, Simulation};
use circuitsim::ModelKind;

fn lined(s: &Simulation) -> Option<f64> {
    let td = s.touchdown.as_ref()?;
    if !td.on_runway { return None; }
    let tr = &s.track;
    let i = tr.iter().position(|p| p.t >= td.t)?;
    // landing direction from the last metres of the track
    let dir = if tr[i].x < tr[i.saturating_sub(8)].x { -1.0 } else { 1.0 };
    let mut dist = 0.0;
    let mut k = i;
    while k >= 2 {
        let (a, b) = (&tr[k - 2], &tr[k]);
        let (dx, dy) = (b.x - a.x, b.y - a.y);
        let ang = (dy).atan2(dx * dir).abs().to_degrees();
        if ang > 5.0 || b.y.abs() > 15.0 { break; }
        dist += dx.hypot(dy);
        k -= 2;
    }
    Some(dist)
}
fn stats(name: &str, v: &mut Vec<f64>) {
    if v.is_empty() { return; }
    v.sort_by(|a, b| a.partial_cmp(b).unwrap());
    let n = v.len();
    println!("{name:<26} n {n:3}  10% {:5.0} m  median {:5.0} m  90% {:5.0} m  under 150 m: {}  under 300 m: {}", v[n / 10], v[n / 2], v[(n * 9) / 10], v.iter().filter(|x| **x < 150.0).count(), v.iter().filter(|x| **x < 300.0).count());
}
fn main() {
    let m = ModelKind::SixDof;
    let mut eo = Vec::new();
    for c in [standard_circuit(), high_circuit()] {
        for w in eo_winds() {
            for tr in eo_triggers() {
                let s = fly_eo(m, w, 4.0, &tr, c.clone());
                if s.autopilot.engine_out.as_ref().map(|e| e.plan == EngineOutPlan::Return).unwrap_or(false) {
                    if let Some(l) = lined(&s) { eo.push(l); if std::env::var("SHOW").is_ok() { println!("  {} {:03}/{} {:<15} {:4.0} m", c.name, w.from_deg, w.speed_kt, tr.name, l); } }
                }
            }
        }
    }
    let mut rnd = Vec::new();
    let mut rng = Rng::new(11.0);
    for _ in 0..60 {
        let w = WindSettings { from_deg: (rng.next() * 36.0).floor() * 10.0, speed_kt: (rng.next() * 16.0).floor(), gusts: false };
        let mut s = new_sim(m, w, 1.0, high_circuit());
        let a = AirStart::random(&s, &mut rng, true);
        s.start_airborne(&a);
        s.run(1500.0);
        if let Some(l) = lined(&s) { rnd.push(l); }
    }
    let mut nor = Vec::new();
    for c in [standard_circuit(), high_circuit()] {
        for (w, seed, ga) in normal_cases() {
            if ga.is_some() { continue; }
            if let Some(l) = lined(&fly_normal(m, w, seed, None, c.clone())) { nor.push(l); }
        }
    }
    stats("normal circuits", &mut nor);
    stats("power-off returns", &mut eo);
    stats("random engine-out starts", &mut rnd);
}
