use circuitsim::aircraft::c172_like;
use circuitsim::sixdof::{euler, SixDof};
fn main() {
    let s = SixDof::on_runway(&c172_like());
    let (p, t, y) = euler(&s.x[6..10]);
    println!("state {:?}", s.x);
    println!("euler deg {:.2} {:.2} {:.2}  h0 {:.3}  loads {:?}", p.to_degrees(), t.to_degrees(), y.to_degrees(), s.h0, s.diag.loads);
}
