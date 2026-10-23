// How often power-off approaches use the forward slip (flights that slipped, seconds slipping), and how
// far from the target point they touch down (+ = long).
use circuitsim::airfield::{high_circuit, standard_circuit};
use circuitsim::atmosphere::{Rng, WindSettings};
use circuitsim::scenarios::*;
use circuitsim::simulation::{AirStart, Simulation};
use circuitsim::ModelKind;
fn run(mut s: Simulation, fail: Option<&EoTrigger>) -> (f64, bool, bool, f64) {
    let mut done = fail.is_none();
    let mut slip = 0.0;
    while !s.finished() && s.t < 1500.0 {
        if !done && (fail.unwrap().when)(&s) { done = s.fail_engine(); }
        s.step();
        if s.autopilot.fma.lat == "SLIP" { slip += Simulation::DT; }
    }
    let on = s.touchdown.as_ref().map(|t| t.on_runway).unwrap_or(false);
    (slip, on, s.passed(), s.accuracy().map(|a| a.0).unwrap_or(f64::NAN))
}
fn main() {
    let m = ModelKind::SixDof;
    let mut res = Vec::new();
    for c in [standard_circuit(), high_circuit()] {
        for w in eo_winds() {
            for tr in eo_triggers() {
                res.push(run(new_sim(m, w, 4.0, c.clone()), Some(&tr)));
            }
        }
    }
    let mut rng = Rng::new(11.0);
    for _ in 0..60 {
        let w = WindSettings { from_deg: (rng.next() * 36.0).floor() * 10.0, speed_kt: (rng.next() * 16.0).floor(), gusts: false };
        let mut s = new_sim(m, w, 1.0, high_circuit());
        let a = AirStart::random(&s, &mut rng, true);
        s.start_airborne(&a);
        res.push(run(s, None));
    }
    let landed: Vec<_> = res.iter().filter(|r| r.1).collect();
    let slipped = landed.iter().filter(|r| r.0 > 0.5).count();
    let secs: f64 = landed.iter().map(|r| r.0).sum();
    let mut miss: Vec<f64> = landed.iter().map(|r| r.3).filter(|m| m.is_finite()).collect();
    miss.sort_by(|a, b| a.partial_cmp(b).unwrap());
    let n = miss.len();
    println!("touchdown vs target: 10% {:+.0} m, median {:+.0} m, 90% {:+.0} m; {} over 100 m long, {} over 100 m short", miss[n / 10], miss[n / 2], miss[n * 9 / 10], miss.iter().filter(|m| **m > 100.0).count(), miss.iter().filter(|m| **m < -100.0).count());
    println!("{} runway landings, {} slipped ({:.0}%), {:.1} s of slip per landing on average, {} passed", landed.len(), slipped, 100.0 * slipped as f64 / landed.len() as f64, secs / landed.len() as f64, res.iter().filter(|r| r.2).count());
}
