// Calibrate the glide planner against the flight model. The planner is made optimistic (it attempts the
// runway whenever the margin-free plan is within reach of a big allowance), and for each case we compare
// the margin-free surplus it predicted at the failure with where the aircraft actually touched down.
// Output: one line per case and a summary of predicted surplus against outcome.
use circuitsim::airfield::{high_circuit, standard_circuit};
use circuitsim::autopilot::{EngineOutPlan, Phase};
use circuitsim::emergency::GlidePlanner;
use circuitsim::scenarios::*;
use circuitsim::tuning::Tuning;
use circuitsim::units::FT;
use circuitsim::{ModelKind, SimOptions, Simulation};
fn main() {
    let mut rows = Vec::new();
    let triggers = eo_triggers();
    for (cn, c) in [("high", high_circuit()), ("std", standard_circuit())] {
        for (from, kt) in [(90.0, 8.0), (90.0, 0.0), (90.0, 15.0), (270.0, 8.0), (360.0, 10.0), (180.0, 10.0)] {
            let mut cases: Vec<(String, Box<dyn Fn(&Simulation, f64) -> bool>)> = Vec::new();
            for d in [0.0, 10.0, 20.0] {
                cases.push((format!("xw+{d}"), Box::new(move |s: &Simulation, t0: f64| s.phase() == Phase::Crosswind && s.t >= t0 + d)));
            }
            for name in ["downwind early", "abeam thr", "base", "final 400ft"] {
                let tr = triggers.iter().find(|t| t.name == name).unwrap().when;
                cases.push((name.to_string(), Box::new(move |s: &Simulation, _| tr(s))));
            }
            for (label, when) in &cases {
                let mut tu = Tuning::default();
                tu.eo_margin_factor = 0.8;
                tu.eo_margin_height = -40.0;
                let mut s = Simulation::new(SimOptions { wind: wind(from, kt, false), seed: 1.0, circuit: c.clone(), model: ModelKind::SixDof, tuning: Some(tu), ..Default::default() });
                let (mut t_xw, mut done, mut pred) = (None, false, None);
                while !s.finished() && s.t < 1200.0 {
                    if s.phase() == Phase::Crosswind && t_xw.is_none() { t_xw = Some(s.t); }
                    if !done && when(&s, t_xw.unwrap_or(f64::INFINITY)) {
                        // margin-free prediction for each end
                        let mut spec = s.autopilot.planner.spec.clone();
                        spec.margin_factor = 1.0;
                        spec.margin_height = 0.0;
                        let p = GlidePlanner::new(s.runway().clone(), spec);
                        let g = p.plan(&s.aircraft, s.autopilot.gust.mean_wind(), None);
                        pred = Some((g.end.name, g.surplus, g.aim, s.aircraft.h / FT));
                        done = s.fail_engine();
                    }
                    s.step();
                }
                let Some((end_name, surplus, aim, h)) = pred else { continue };
                let eo = s.autopilot.engine_out.as_ref().unwrap();
                let end = s.autopilot.eo_end();
                let (onrwy, past) = match &s.touchdown { Some(td) => (td.on_runway, end.dir * (td.x - end.threshold)), None => (false, f64::NAN) };
                let tried = eo.plan == EngineOutPlan::Return;
                println!("{cn:4} {from:3}/{kt:2} {label:15} h {h:4.0} pred {end_name} surplus {surplus:6.0} aim {aim:4.0} | {} td {:6.0} m past {} {}", if tried { "RETURN" } else { "field " }, past, end.name, if onrwy { "ON RWY" } else { "" });
                rows.push((surplus, tried, onrwy, past));
            }
        }
    }
    println!("\npredicted margin-free surplus band: cases, attempted, landed on the runway");
    for (lo, hi) in [(-1e9, -60.0), (-60.0, -30.0), (-30.0, -15.0), (-15.0, 0.0), (0.0, 15.0), (15.0, 30.0), (30.0, 60.0), (60.0, 1e9)] {
        let b: Vec<_> = rows.iter().filter(|r| r.0 >= lo && r.0 < hi).collect();
        println!("  {:>6} to {:>6}: {:3} {:3} {:3}", lo.max(-999.0), hi.min(999.0), b.len(), b.iter().filter(|r| r.1).count(), b.iter().filter(|r| r.2).count());
    }
}
