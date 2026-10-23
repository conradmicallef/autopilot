use circuitsim::scenarios::*;
use circuitsim::airfield::standard_circuit;
use circuitsim::ModelKind;
fn main() {
    let mut s = new_sim(ModelKind::SixDof, wind(270.0, 0.0, false), 3.0, standard_circuit());
    for i in 0..60 {
        s.step();
        let six = s.aircraft.six.as_ref().unwrap();
        println!("{i:3} {} x{:?} ctl de{:.3} da{:.3} dr{:.3} st{:.3} thr{:.2} loads{:?}", s.phase().code(),
          six.x.iter().map(|v| format!("{v:.3}")).collect::<Vec<_>>(), six.ctl.de, six.ctl.da, six.ctl.dr, six.ctl.steer, six.ctl.thr,
          six.diag.loads.iter().map(|v| v.round()).collect::<Vec<_>>());
    }
}
