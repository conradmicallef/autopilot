// Flap limit check over every suite flight: the largest airspeed seen with flap beyond 10° (limit 85 KIAS)
// and with any flap out (limit 110 KIAS), and how long each limit was exceeded.
use circuitsim::airfield::{high_circuit, standard_circuit};
use circuitsim::atmosphere::{Rng, WindSettings};
use circuitsim::scenarios::*;
use circuitsim::simulation::{AirStart, Simulation};
use circuitsim::units::KT;
use circuitsim::ModelKind;
#[derive(Default)]
struct Acc { max_full: f64, max_any: f64, over_full: f64, over_any: f64, flights: usize, bad: usize }
fn run(acc: &mut Acc, mut s: Simulation, fail: Option<&EoTrigger>) {
    let mut done = fail.is_none();
    let (mut of, mut oa) = (0.0, 0.0);
    while !s.finished() && s.t < 1500.0 {
        if !done && (fail.unwrap().when)(&s) { done = s.fail_engine(); }
        s.step();
        let ac = &s.aircraft;
        if ac.on_ground { continue; }
        let kt = ac.v / KT;
        if ac.flap_pos > 10.5 { acc.max_full = acc.max_full.max(kt); if kt > 85.0 { of += Simulation::DT; if std::env::var("SHOW").is_ok() { println!("  {:?} t {:.1} {:.1} kt flap {:.0} h {:.0} ft theta {:.1}", s.phase(), s.t, kt, ac.flap_pos, ac.h / 0.3048, ac.theta.to_degrees()); } } }
        if ac.flap_pos > 0.5 { acc.max_any = acc.max_any.max(kt); if kt > 110.0 { oa += Simulation::DT; } }
    }
    acc.flights += 1;
    if of > 0.0 || oa > 0.0 { acc.bad += 1; }
    acc.over_full += of; acc.over_any += oa;
}
fn main() {
    let m = ModelKind::SixDof;
    let mut acc = Acc::default();
    for c in [standard_circuit(), high_circuit()] {
        for (w, seed, _) in normal_cases() { run(&mut acc, new_sim(m, w, seed, c.clone()), None); }
        for w in eo_winds() { for tr in eo_triggers() { run(&mut acc, new_sim(m, w, 4.0, c.clone()), Some(&tr)); } }
    }
    let mut rng = Rng::new(7.0);
    for k in 0..60 {
        let w = WindSettings { from_deg: (rng.next() * 36.0).floor() * 10.0, speed_kt: (rng.next() * 16.0).floor(), gusts: rng.next() < 0.4 };
        let mut s = new_sim(m, w, 1.0, high_circuit());
        let a = AirStart::random(&s, &mut rng, k % 2 == 1);
        s.start_airborne(&a);
        run(&mut acc, s, None);
    }
    println!("{} flights: flap > 10° up to {:.1} KIAS (limit 85), any flap up to {:.1} KIAS (limit 110)", acc.flights, acc.max_full, acc.max_any);
    println!("over a limit in {} flights; {:.1} s in total over 85 kt with flap > 10°, {:.1} s over 110 kt with flap", acc.bad, acc.over_full, acc.over_any);
}
