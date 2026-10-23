// Second-by-second trace of a crosswind-leg engine failure: speed, flap, sideslip, sink, glide ratio.
use circuitsim::airfield::high_circuit;
use circuitsim::scenarios::*;
use circuitsim::units::FT;
use circuitsim::ModelKind;
fn main() {
    let a: Vec<String> = std::env::args().skip(1).collect();
    let w = wind(a[0].parse().unwrap(), a[1].parse().unwrap(), false);
    let tr = eo_triggers().into_iter().find(|t| t.name == "crosswind").unwrap();
    let mut s = new_sim(ModelKind::SixDof, w, 1.0, high_circuit());
    let mut done = false; let mut k = 0; let mut last = (0.0, 0.0, 0.0); let mut le = (0.0f64, 0.0f64);
    while !s.finished() && s.t < 1200.0 {
        if !done && (tr.when)(&s) { done = s.fail_engine(); }
        s.step(); k += 1;
        if done && k % 50 == 0 && !s.aircraft.on_ground {
            let ac = &s.aircraft;
            let d = ((ac.x - last.0).powi(2) + (ac.y - last.1).powi(2)).sqrt(); let dh = last.2 - ac.h;
            let six = ac.six.as_ref().unwrap();
            print!("shb {:5.1} dgb {:5.1} ib {:6.3} r {:6.3} rt {:6.3} | ", ac.shaper.beta.to_degrees(), six.diag.beta.to_degrees(), six.fcs.i_beta, six.x[12], 9.81 * circuitsim::sixdof::euler(&six.x[6..10]).0.sin() / six.diag.v);
            print!("dr {:5.1} da {:5.1} de {:5.1} alpha {:5.1} | ", six.ctl.dr.to_degrees(), six.ctl.da.to_degrees(), six.ctl.de.to_degrees(), six.diag.alpha.to_degrees());
            println!("t {:.0} {:14} h {:4.0} V {:4.1} flap {:4.1} beta {:5.1} bank {:4.0} gamma {:5.1} theta {:5.1} sink {:4.1} ft/s  L/D_gnd {:5.1}", s.t, s.phase().code(), ac.h/FT, ac.v/0.5144, ac.flap_pos, ac.beta.to_degrees(), ac.phi.to_degrees(), ac.gamma.to_degrees(), ac.theta.to_degrees(), dh/FT, if dh > 0.1 { d/dh } else { 0.0 });
            let e = ac.h + ac.v * ac.v / (2.0 * 9.81);
            println!("      energy L/D (air) {:5.1}  lift/W {:4.2}", if le.0 - e > 0.1 { (s.t * 0.0 + ac.v) * 1.0 / ((le.0 - e) / 1.0) } else { 0.0 }, six.diag.lift / (ac.spec.mass * 9.81));
            le = (e, 0.0);
            last = (ac.x, ac.y, ac.h);
            if s.t > 130.0 { break; }
        }
    }
}
