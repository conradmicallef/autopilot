use circuitsim::airfield::standard_circuit;
use circuitsim::scenarios::*;
use circuitsim::ModelKind;
fn main() {
    let args: Vec<String> = std::env::args().collect();
    let name = args.get(1).cloned().unwrap_or("climb 200ft".into());
    let w = wind(args.get(3).map(|s| s.parse().unwrap()).unwrap_or(270.0), args.get(2).map(|s| s.parse().unwrap()).unwrap_or(0.0), args.get(4).is_some());
    let tr = eo_triggers().into_iter().find(|t| t.name == name).unwrap();
    let mut s = new_sim(ModelKind::SixDof, w, 4.0, standard_circuit());
    let mut done = false;
    let mut k = 0;
    while !s.finished() && s.t < 1200.0 {
        if !done && (tr.when)(&s) { done = s.fail_engine(); }
        s.step();
        k += 1;
        let a = &s.aircraft;
        if done && a.h < 12.0 && k % 5 == 0 && !a.on_ground {
            let six = a.six.as_ref().unwrap();
            println!("t {:.1} {} h {:.1} V {:.1}kt vs {:.2} θ {:.1} θcmd {:.1} α {:.1} de {:.1} flap {:.0} hyst {} loads {:.0?}", s.t, s.phase().code(), a.h, a.v / 0.5144, a.vertical_speed(), a.theta.to_degrees(), a.shaper.theta.to_degrees(), six.diag.alpha.to_degrees(), six.ctl.de.to_degrees(), a.flap_pos, six.stall_hyst, six.diag.loads);
        }
    }
    println!("{:?}", s.evaluate().iter().map(|r| r.value.clone().unwrap_or_default()).collect::<Vec<_>>());
}
