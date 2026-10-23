// Physical reachability of the 27 end from crosswind failure points: glide ratio from the polar, Dubins-style
// path (turn at 45°, straight to a short lined-up final at the threshold), calm air.
use circuitsim::aircraft::{Aircraft};
use circuitsim::scenarios::*;
use circuitsim::airfield::high_circuit;
use circuitsim::ModelKind;
use circuitsim::units::{KT, FT};
fn main() {
    let s = new_sim(ModelKind::SixDof, wind(90.0, 0.0, false), 1.0, high_circuit());
    let ac: &Aircraft = &s.aircraft;
    for v in [60.0, 65.0, 68.0, 70.0, 75.0] { println!("clean L/D at {v} kt: {:.2}", ac.glide_ratio(v * KT, 0.0)); }
    println!("Vs clean {:.1} kt, flap 20 {:.1} kt", ac.stall_speed(0.0) / KT, ac.stall_speed(20.0) / KT);
    println!("runway length {:.0}", s.runway().length);
}
