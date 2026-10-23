// Random engine-out starts: how far out the power-off plans join the centreline (a stretched path when
// too high), how far from the field the aircraft goes after the failure, and how the flights end.
use circuitsim::airfield::high_circuit;
use circuitsim::atmosphere::{Rng, WindSettings};
use circuitsim::scenarios::*;
use circuitsim::simulation::AirStart;
use circuitsim::ModelKind;
fn main() {
    let mut rng = Rng::new(11.0);
    let (mut n, mut far, mut ok) = (0, 0, 0);
    for i in 0..100 {
        let w = WindSettings { from_deg: (rng.next() * 36.0).floor() * 10.0, speed_kt: (rng.next() * 16.0).floor(), gusts: false };
        let mut s = new_sim(ModelKind::SixDof, w, 1.0, high_circuit());
        let a = AirStart::random(&s, &mut rng, true);
        s.start_airborne(&a);
        let mut worst: f64 = 0.0;
        let mut orbiting = false;
        let mut join: f64 = 0.0;
        while !s.finished() && s.t < 1500.0 {
            s.step();
            if let Some(g) = &s.autopilot.glide { if g.ip_distance > 1250.0 { orbiting = true; } join = join.max(g.ip_distance); }
            if s.autopilot.engine_out.is_some() { worst = worst.max((s.aircraft.x - 600.0).hypot(s.aircraft.y)); }
        }
        if orbiting {
            n += 1;
            let pass = s.passed();
            if pass { ok += 1; }
            if worst > 4000.0 { far += 1; }
            println!("#{i:3} start ({:6.0},{:6.0}) {:5.0} ft: joins up to {:5.0} m out, up to {:5.0} m from the field's middle, {}", a.x, a.y, a.h / 0.3048, join, worst, if pass { "landed" } else { "failed" });
        }
    }
    println!("{n} stretched (joined beyond 1.25 km), {ok} of them landed, {far} went more than 4 km from the field");
}
