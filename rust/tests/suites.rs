//! Regression tests on whole flights (run with `cargo test --release`: debug builds are ~10× slower).
//!
//! * Three flights that must always pass: a calm normal circuit, an engine failure abeam the threshold
//!   on the High circuit, a rejected take-off.
//! * The suite totals must not fall below `tests/golden.txt` (normal scenarios, engine suite, engine
//!   failures within the physical ceiling). Raise the numbers there when a change improves them.

use circuitsim::airfield::{high_circuit, standard_circuit};
use circuitsim::scenarios::*;
use circuitsim::ModelKind;

const M: ModelKind = ModelKind::SixDof;

fn trigger(name: &str) -> EoTrigger {
    eo_triggers().into_iter().find(|t| t.name == name).unwrap()
}

#[test]
fn calm_normal_circuit_passes() {
    let s = fly_normal(M, wind(270.0, 0.0, false), 1.0, None, standard_circuit());
    assert!(s.passed(), "{:?}", s.evaluate().iter().filter(|r| r.pass == Some(false)).map(|r| (&r.label, &r.value)).collect::<Vec<_>>());
}

#[test]
fn engine_failure_abeam_the_threshold_returns_to_the_runway() {
    let s = fly_eo(M, wind(270.0, 0.0, false), 4.0, &trigger("abeam thr"), high_circuit());
    assert!(s.passed(), "{:?}", s.evaluate().iter().filter(|r| r.pass == Some(false)).map(|r| (&r.label, &r.value)).collect::<Vec<_>>());
}

#[test]
fn rejected_take_off_stops_on_the_runway() {
    let s = fly_eo(M, wind(270.0, 12.0, false), 4.0, &trigger("roll 40kt"), standard_circuit());
    assert!(s.passed());
}

fn golden() -> std::collections::HashMap<String, usize> {
    include_str!("golden.txt")
        .lines()
        .filter(|l| !l.trim().is_empty() && !l.starts_with('#'))
        .map(|l| {
            let mut it = l.split_whitespace();
            (it.next().unwrap().to_string(), it.next().unwrap().parse().unwrap())
        })
        .collect()
}

#[test]
fn suite_totals_do_not_regress() {
    let g = golden();
    let mut normal = 0;
    for circuit in circuits() {
        for (w, seed, ga) in normal_cases() {
            normal += fly_normal(M, w, seed, ga, circuit.clone()).passed() as usize;
        }
    }
    let (mut engine, mut within) = (0, 0);
    for circuit in circuits() {
        for w in eo_winds() {
            for tr in eo_triggers() {
                let s = fly_eo(M, w, 4.0, &tr, circuit.clone());
                let ok = s.passed();
                engine += ok as usize;
                within += (ok && s.eo_ceiling == Some(true)) as usize;
            }
        }
    }
    eprintln!("normal {normal}, engine {engine}, engine within ceiling {within}");
    assert!(normal >= g["normal"], "normal scenarios {normal} < {}", g["normal"]);
    assert!(engine >= g["engine"], "engine suite {engine} < {}", g["engine"]);
    assert!(within >= g["engine_within_ceiling"], "within ceiling {within} < {}", g["engine_within_ceiling"]);
}
