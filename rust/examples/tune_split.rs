// Hold-out comparison, split into nominal and perturbed airframes: hand-tuned vs autotuned.
use circuitsim::autotune::{cases, evaluate};
use circuitsim::tuning::Tuning;
use circuitsim::ModelKind;
fn main() {
    let test = cases(240, 777.0);
    let nominal: Vec<_> = test.iter().cloned().enumerate().filter(|(i, _)| i % 2 == 0).map(|(_, c)| c).collect();
    let perturbed: Vec<_> = test.iter().cloned().enumerate().filter(|(i, _)| i % 2 == 1).map(|(_, c)| c).collect();
    for (name, tu) in [("hand-tuned", Tuning::hand_tuned()), ("autotuned", Tuning::default())] {
        let a = evaluate(&nominal, ModelKind::SixDof, &tu, 2);
        let b = evaluate(&perturbed, ModelKind::SixDof, &tu, 2);
        println!("{name:<11} nominal {}/{} (cost {:.3})  perturbed {}/{} (cost {:.3})", a.passed, a.n, a.cost, b.passed, b.n, b.cost);
    }
}
