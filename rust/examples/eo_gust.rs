// Gusty engine-failure case with flap and sink trace: circuit trigger [seed].
use circuitsim::airfield::{high_circuit, standard_circuit};
use circuitsim::scenarios::*;
use circuitsim::units::{FT, KT};
use circuitsim::ModelKind;
fn main() {
    let a: Vec<String> = std::env::args().skip(1).collect();
    let c = if a[0] == "std" { standard_circuit() } else { high_circuit() };
    let tr = eo_triggers().into_iter().find(|t| t.name == a[1]).unwrap();
    let seed: f64 = a.get(2).map(|x| x.parse().unwrap()).unwrap_or(1.0);
    let mut s = new_sim(ModelKind::SixDof, wind(360.0, 12.0, true), seed, c);
    let (mut done, mut k) = (false, 0);
    while !s.finished() && s.t < 1200.0 {
        if !done && (tr.when)(&s) { done = s.fail_engine(); }
        s.step(); k += 1;
        if done && k % 25 == 0 && !s.aircraft.on_ground && (s.aircraft.h < 120.0 || std::env::var("ALL").is_ok()) {
            let ac = &s.aircraft;
            println!("t {:.1} {:10} x {:5.0} h {:4.0} V {:3.0} flapcmd {:2.0} flap {:4.1} vs {:5.1} ft/s", s.t, s.phase().code(), ac.x, ac.h / FT, ac.v / KT, s.autopilot.eo_flaps_cmd(), ac.flap_pos, ac.vertical_speed() / FT);
        }
    }
    println!("{}", s.evaluate().iter().map(|r| r.value.clone().unwrap_or_default()).collect::<Vec<_>>().join(" | "));
}
