use circuitsim::scenarios::*;
use circuitsim::airfield::{standard_circuit, high_circuit};
use circuitsim::ModelKind;
fn main() {
    for (m, c, wd, wk, g, seed) in [(0,0,270.0,0.0,false,1.0),(0,0,300.0,15.0,false,1.0),(1,0,270.0,0.0,false,1.0),(1,1,360.0,15.0,false,1.0),(1,0,220.0,12.0,true,7.0)] {
        let model = if m == 1 { ModelKind::SixDof } else { ModelKind::PointMass };
        let s = fly_normal(model, wind(wd, wk, g), seed, None, if c == 1 { high_circuit() } else { standard_circuit() });
        let td = s.touchdown.as_ref().unwrap();
        println!("{} {}: t={:.2} td x={:.2} y={:.3} sink={:.3}ft/s stop={:.2} passed={} events={}", model.name(), s.autopilot.circuit.spec.name, s.t, td.x, td.y, td.sink/0.3048, s.aircraft.x, s.passed(), s.autopilot.events.len());
    }
}
