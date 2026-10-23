// How far turns onto final carry the aircraft through the centreline: normal circuits (both circuits,
// the scenario winds), power-off returns (engine suite) and powered rejoins from random states.
// Overshoot = the largest distance on the far side of the centreline, from the turn onto final until
// 50 ft, measured from the side the aircraft turned in from.
use circuitsim::airfield::{high_circuit, standard_circuit};
use circuitsim::atmosphere::{Rng, WindSettings};
use circuitsim::scenarios::*;
use circuitsim::simulation::{AirStart, Simulation};
use circuitsim::units::FT;
use circuitsim::ModelKind;

fn overshoot(s: &Simulation, final_phases: &[&str]) -> Option<f64> {
    let tr = &s.track;
    let i0 = tr.iter().position(|p| final_phases.contains(&p.phase))?;
    // side the aircraft came from: where it was 20 s before joining final
    let from = tr[i0.saturating_sub(80)].y;
    if from.abs() < 30.0 { return Some(0.0); }
    let side = from.signum();
    // the first final only (a go-around and a second circuit are not this turn)
    let os = tr[i0..].iter().take_while(|p| final_phases.contains(&p.phase)).filter(|p| p.h > 50.0 * FT).map(|p| -side * p.y).fold(0.0, f64::max);
    Some(os)
}
fn stats(name: &str, v: &[f64]) {
    if v.is_empty() { return; }
    let mut w = v.to_vec(); w.sort_by(|a, b| a.partial_cmp(b).unwrap());
    let mean = w.iter().sum::<f64>() / w.len() as f64;
    println!("{name:<28} n {:3}  mean {:5.0} m  median {:5.0}  90% {:5.0}  worst {:5.0}  over 50 m: {}", w.len(), mean, w[w.len() / 2], w[(w.len() * 9) / 10], w[w.len() - 1], w.iter().filter(|x| **x > 50.0).count());
}
fn main() {
    let m = ModelKind::SixDof;
    let mut normal = Vec::new();
    for c in [standard_circuit(), high_circuit()] {
        for (w, seed, ga) in normal_cases() {
            if ga.is_some() { continue; }
            let s = fly_normal(m, w, seed, None, c.clone());
            if let Some(o) = overshoot(&s, &["FINAL", "FLARE"]) { normal.push(o); if std::env::var("SHOW").is_ok() { println!("  normal {} {:03}/{} {:.0} m", c.name, w.from_deg, w.speed_kt, o); } }
        }
    }
    let mut eo = Vec::new();
    for c in [standard_circuit(), high_circuit()] {
        for w in eo_winds() {
            for tr in eo_triggers() {
                let s = fly_eo(m, w, 4.0, &tr, c.clone());
                if s.autopilot.engine_out.as_ref().map(|e| e.plan == circuitsim::autopilot::EngineOutPlan::Return).unwrap_or(false) {
                    if let Some(o) = overshoot(&s, &["EO_FINAL", "FLARE"]) { eo.push(o); if std::env::var("SHOW").is_ok() { println!("  eo {} {:03}/{} {} {:.0} m", c.name, w.from_deg, w.speed_kt, tr.name, o); } }
                }
            }
        }
    }
    let mut rj = Vec::new();
    let mut rng = Rng::new(7.0);
    for _ in 0..30 {
        let w = WindSettings { from_deg: (rng.next() * 36.0).floor() * 10.0, speed_kt: (rng.next() * 16.0).floor(), gusts: false };
        let mut s = new_sim(m, w, 1.0, high_circuit());
        let a = AirStart::random(&s, &mut rng, false);
        s.start_airborne(&a);
        s.run(1500.0);
        if let Some(o) = overshoot(&s, &["FINAL", "FLARE"]) { rj.push(o); if std::env::var("SHOW").is_ok() && o > 50.0 { println!("  rejoin {:03}/{} start ({:.0},{:.0}) {:.0} ft hdg {:.0}: {:.0} m", w.from_deg, w.speed_kt, a.x, a.y, a.h / FT, a.heading.to_degrees(), o); if o > 300.0 { for e in &s.autopilot.events { println!("     {:5.0} {:<12} x {:6.0} y {:6.0} h {:5.0} {}", e.t, e.phase, e.x, e.y, e.h / FT, e.note.clone().unwrap_or_default()); } let i0 = s.track.iter().position(|p| p.phase == "FINAL").unwrap(); for p in s.track[i0..].iter().step_by(12).take(12) { println!("       {:<8} x {:6.0} y {:6.0} h {:5.0}", p.phase, p.x, p.y, p.h / FT); } } } }
    }
    stats("normal circuits", &normal);
    stats("power-off returns", &eo);
    stats("powered rejoins", &rj);
}
