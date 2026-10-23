//! Native command-line runner: test suites, parity output, mode analysis, Monte Carlo batches.
//!
//! Usage: circuitsim <command> [--model pm|6dof] [args]
//!   scenarios   normal circuits and manual go-arounds (both circuit profiles)
//!   engine      60 engine-failure cases (2 circuits × 3 winds × 10 failure points)
//!   profiles    Standard vs High circuit, glide reach
//!   parity      CSV of key results, for comparing against the TypeScript implementation
//!   modes       trim + natural modes of the 6DOF model
//!   montecarlo  N randomised flights (winds, gusts, seeds, failure points), pass rates
//!   trace       per-second trace of one flight: trace <fromDeg> <kt> <gusts 0|1> <circuit std|high> [fail-at-x]

use circuitsim::airfield::{high_circuit, standard_circuit};
use circuitsim::atmosphere::{Rng, WindSettings};
use circuitsim::scenarios::*;
use circuitsim::units::{heading_deg, D2R, FT, KT};
use circuitsim::{ModelKind, Simulation};
use std::time::Instant;

fn summary(s: &Simulation) -> String {
    s.evaluate().iter().map(|r| r.value.clone().unwrap_or_else(|| "—".into())).collect::<Vec<_>>().join(" | ")
}
fn notes(s: &Simulation) -> String {
    s.autopilot.events.iter().filter_map(|e| e.note.clone()).collect::<Vec<_>>().join(",")
}

fn main() {
    let all: Vec<String> = std::env::args().collect();
    let model = if all.iter().any(|a| a == "6dof") { ModelKind::SixDof } else { ModelKind::PointMass };
    let args: Vec<String> = all.into_iter().filter(|a| a != "6dof" && a != "pm").collect();
    let cmd = args.get(1).map(|s| s.as_str()).unwrap_or("scenarios");
    let t0 = Instant::now();
    let code = match cmd {
        "scenarios" => scenarios(model),
        "engine" => engine(model),
        "profiles" => profiles(model),
        "parity" => parity(model),
        "modes" => modes(),
        "montecarlo" => {
            let n = args.iter().filter_map(|a| a.parse::<usize>().ok()).next().unwrap_or(200);
            montecarlo(model, n)
        }
        "trace" => trace(model, &args[2..]),
        "stol" => stol(model),
        _ => {
            eprintln!("unknown command {cmd}");
            2
        }
    };
    eprintln!("[{} model, {:.2} s wall]", model.name(), t0.elapsed().as_secs_f64());
    std::process::exit(code);
}

fn scenarios(model: ModelKind) -> i32 {
    let mut fails = 0;
    let mut total = 0;
    for circuit in circuits() {
        for (w, seed, ga) in normal_cases() {
            let s = fly_normal(model, w, seed, ga, circuit.clone());
            let ok = s.passed();
            total += 1;
            if !ok {
                fails += 1;
            }
            println!(
                "{:<8} {:03}/{}{} {:<5} {} t={:.0} GA={} | {} {}",
                circuit.name,
                w.from_deg,
                w.speed_kt,
                if w.gusts { "G" } else { " " },
                ga.unwrap_or(""),
                if ok { "PASS" } else { "FAIL" },
                s.t,
                s.autopilot.go_arounds,
                summary(&s),
                notes(&s)
            );
        }
    }
    println!("{}/{} passed", total - fails, total);
    0
}

fn engine(model: ModelKind) -> i32 {
    let mut pass = 0;
    let mut total = 0;
    for circuit in circuits() {
        for w in eo_winds() {
            for tr in eo_triggers() {
                let s = fly_eo(model, w, 4.0, &tr, circuit.clone());
                let ok = s.passed();
                total += 1;
                if ok {
                    pass += 1;
                }
                println!(
                    "{:<8} {:03}/{}{} {:<14} {} t={:.0} | {}",
                    circuit.name,
                    w.from_deg,
                    w.speed_kt,
                    if w.gusts { "G" } else { "" },
                    tr.name,
                    if ok { "PASS" } else { "FAIL" },
                    s.t,
                    summary(&s)
                );
            }
        }
    }
    println!("{pass}/{total} passed");
    0
}

fn profiles(model: ModelKind) -> i32 {
    let winds = [
        wind(270.0, 0.0, false),
        wind(270.0, 10.0, false),
        wind(90.0, 8.0, false),
        wind(360.0, 15.0, false),
        wind(220.0, 12.0, true),
        wind(300.0, 20.0, true),
    ];
    for circuit in circuits() {
        for w in winds {
            let s = fly_normal(model, w, 3.0, None, circuit.clone());
            println!(
                "{:<8} {:03}/{}{} {} t={:.0} reach={:.0}% | {}",
                circuit.name,
                w.from_deg,
                w.speed_kt,
                if w.gusts { "G" } else { " " },
                if s.passed() { "PASS" } else { "FAIL" },
                s.t,
                s.reach_fraction() * 100.0,
                summary(&s)
            );
        }
    }
    0
}

/// One CSV line per case: enough numbers to spot any divergence between implementations.
fn parity(model: ModelKind) -> i32 {
    println!("case,t_end,phase,go_arounds,events,td_x,td_y,td_sink_fps,td_crab_deg,stop_x,stop_y,reach_pct");
    let line = |name: String, s: &Simulation| {
        let td = s.touchdown.as_ref();
        println!(
            "{},{:.2},{},{},{},{:.2},{:.3},{:.3},{:.3},{:.2},{:.3},{:.1}",
            name,
            s.t,
            s.phase().code(),
            s.autopilot.go_arounds,
            s.autopilot.events.len(),
            td.map(|t| t.x).unwrap_or(f64::NAN),
            td.map(|t| t.y).unwrap_or(f64::NAN),
            td.map(|t| t.sink / FT).unwrap_or(f64::NAN),
            td.map(|t| t.crab / D2R).unwrap_or(f64::NAN),
            s.aircraft.x,
            s.aircraft.y,
            s.reach_fraction() * 100.0
        );
    };
    for circuit in circuits() {
        for (w, seed, ga) in normal_cases() {
            let s = fly_normal(model, w, seed, ga, circuit.clone());
            line(format!("{}-{}-{}-{}-{}", circuit.name, w.from_deg, w.speed_kt, w.gusts as u8, ga.unwrap_or("none")), &s);
        }
        for w in eo_winds() {
            for tr in eo_triggers() {
                let s = fly_eo(model, w, 4.0, &tr, circuit.clone());
                line(format!("{}-{}-{}-{}-EO {}", circuit.name, w.from_deg, w.speed_kt, w.gusts as u8, tr.name), &s);
            }
        }
    }
    0
}

#[cfg(not(target_arch = "wasm32"))]
fn modes() -> i32 {
    use circuitsim::aircraft::c172_like;
    use circuitsim::sixdof::analysis::{free_air_model, modes, trim};
    let m = free_air_model(&c172_like());
    println!("6DOF model: trim and natural modes (sea level, 1000 kg)\n");
    for (label, v, gamma, flap) in [
        ("Cruise 100 kt level, clean", 100.0 * KT, 0.0, 0.0),
        ("Downwind 90 kt level, clean", 90.0 * KT, 0.0, 0.0),
        ("Climb 75 kt, +5°, flap 0", 75.0 * KT, 5.0 * D2R, 0.0),
        ("Approach 65 kt, −3°, flap 30", 65.0 * KT, -3.0 * D2R, 30.0),
        ("Best glide 68 kt, power off", 68.0 * KT, f64::NAN, 0.0),
    ] {
        let t = if gamma.is_nan() {
            // find the glide angle where trim throttle is zero
            let mut lo = -12.0 * D2R;
            let mut hi = 0.0;
            for _ in 0..60 {
                let mid = 0.5 * (lo + hi);
                if trim(&m, v, mid, flap).thr > 0.0 {
                    hi = mid
                } else {
                    lo = mid
                }
            }
            trim(&m, v, 0.5 * (lo + hi), flap)
        } else {
            trim(&m, v, gamma, flap)
        };
        println!(
            "{label}: α {:.2}°, elevator {:.2}°, throttle {:.0}%, γ {:.2}°{}, residual {:.1e}",
            t.alpha / D2R,
            t.de / D2R,
            t.thr * 100.0,
            t.gamma / D2R,
            if t.thr.abs() < 1e-3 { format!(" (glide ratio {:.1})", -1.0 / t.gamma.tan()) } else { String::new() },
            t.residual
        );
        for md in modes(&m, &t) {
            match (md.wn, md.tau) {
                (Some(wn), _) => println!(
                    "    {:<16} ωn {:.2} rad/s  ζ {:.3}  period {:.1} s",
                    md.name,
                    wn,
                    md.zeta.unwrap(),
                    md.period.unwrap()
                ),
                (_, Some(tau)) => println!(
                    "    {:<16} τ {:.2} s{}",
                    md.name,
                    tau,
                    if tau < 0.0 { "  (divergent: e-folds in |τ|)" } else { "" }
                ),
                _ => {}
            }
        }
    }
    0
}

fn montecarlo(model: ModelKind, n: usize) -> i32 {
    let mut rng = Rng::new(20261022.0);
    let mut by_kind: std::collections::BTreeMap<&str, (usize, usize)> = Default::default();
    let mut worst: Vec<(String, String)> = Vec::new();
    let mut reasons: std::collections::BTreeMap<String, usize> = Default::default();
    for i in 0..n {
        let w = WindSettings {
            from_deg: (rng.next() * 36.0).floor() * 10.0,
            speed_kt: (rng.next() * 16.0).floor(),
            gusts: rng.next() < 0.4,
        };
        let circuit = if rng.next() < 0.5 { standard_circuit() } else { high_circuit() };
        let seed = (rng.next() * 1e6).floor() + 1.0;
        let failure = rng.next() < 0.5;
        let (kind, s) = if failure {
            let trigs = eo_triggers();
            let tr = &trigs[(rng.next() * trigs.len() as f64) as usize % trigs.len()];
            ("engine failure", fly_eo(model, w, seed, tr, circuit.clone()))
        } else {
            ("normal circuit", fly_normal(model, w, seed, None, circuit.clone()))
        };
        let ok = s.passed();
        if !ok {
            let rows = s.evaluate();
            let failed: Vec<String> = if !s.finished() {
                vec!["did not stop (timeout)".into()]
            } else {
                rows.iter().filter(|r| !r.info && r.pass != Some(true)).map(|r| r.label.clone()).collect()
            };
            for f in failed {
                let key = format!("{kind}: {f}{}", if w.gusts { " (gusty)" } else { "" });
                *reasons.entry(key).or_default() += 1;
            }
        }
        let e = by_kind.entry(kind).or_default();
        e.1 += 1;
        if ok {
            e.0 += 1;
        } else if worst.len() < 12 {
            worst.push((
                format!("#{i} {} {:03}/{:.0}{} {}", circuit.name, w.from_deg, w.speed_kt, if w.gusts { "G" } else { "" }, kind),
                summary(&s),
            ));
        }
    }
    println!("Monte Carlo: {n} flights, winds 0–15 kt any direction, 40% gusty, both circuits");
    for (k, (p, t)) in &by_kind {
        println!("  {k:<15} {p}/{t} passed ({:.0}%)", 100.0 * *p as f64 / *t as f64);
    }
    println!("  failed criteria:");
    for (k, n) in &reasons {
        println!("    {n:3}  {k}");
    }
    if std::env::var("SHOW").is_ok() && !worst.is_empty() {
        println!("  first failures:");
        for (a, b) in worst {
            println!("    {a}: {b}");
        }
    }
    0
}

fn trace(model: ModelKind, a: &[String]) -> i32 {
    let num = |i: usize, d: f64| a.get(i).and_then(|s| s.parse::<f64>().ok()).unwrap_or(d);
    let w = wind(num(0, 270.0), num(1, 0.0), num(2, 0.0) > 0.0);
    let circuit = if a.get(3).map(|s| s == "high").unwrap_or(false) { high_circuit() } else { standard_circuit() };
    let fail_x = a.get(4).and_then(|s| s.parse::<f64>().ok()).filter(|v| *v < 1e8);
    let seed = std::env::var("SEED").ok().and_then(|v| v.parse().ok()).unwrap_or(3.0);
    let mut s = new_sim(model, w, seed, circuit);
    let mut n = 0;
    let mut failed = false;
    while !s.finished() && s.t < 1500.0 {
        if let Some(fx) = fail_x {
            if !failed && s.phase() == circuitsim::autopilot::Phase::Downwind && s.aircraft.x < fx {
                failed = s.fail_engine();
            }
        }
        s.step();
        n += 1;
        if n % (if s.aircraft.h < 40.0 && matches!(s.phase().code(), "FINAL" | "FLARE" | "ROLLOUT") { 5 } else { 50 }) == 0 {
            let ac = &s.aircraft;
            let six = ac.six.as_ref();
            println!(
                "{:6.1} {:<13} {:<9} {:<9} x{:6.0} y{:6.0} h{:5.0}ft V{:3.0} γ{:5.1} θ{:5.1} φ{:5.1} β{:5.1} hdg{:4.0} trk{:4.0} thr{:4.2} fl{:2}{}",
                s.t,
                s.phase().code(),
                s.autopilot.fma.vert,
                s.autopilot.fma.lat,
                ac.x,
                ac.y,
                ac.h / FT,
                ac.v / KT,
                ac.gamma / D2R,
                ac.theta / D2R,
                ac.phi / D2R,
                ac.beta / D2R,
                heading_deg(ac.nose_heading()),
                heading_deg(ac.track(&s.wind)),
                ac.thr,
                ac.flaps(),
                six.map(|x| format!(
                    " de{:5.1} da{:5.1} dr{:5.1} st{:5.1} α{:5.1} loads{:5.0}/{:5.0}/{:5.0}/{:4.0}",
                    x.ctl.de / D2R,
                    x.ctl.da / D2R,
                    x.ctl.dr / D2R,
                    x.ctl.steer / D2R,
                    x.diag.alpha / D2R,
                    x.diag.loads[0],
                    x.diag.loads[1],
                    x.diag.loads[2],
                    x.diag.loads[3]
                ))
                .unwrap_or_default()
            );
        }
    }
    println!("{}", summary(&s));
    for e in &s.autopilot.events {
        println!("  {:6.1} {:<15} h{:5.0}ft V{:3.0} {}", e.t, e.phase, e.h / FT, e.v / KT, e.note.clone().unwrap_or_default());
    }
    0
}

/// Normal vs short-field technique on three strip lengths, in four wind conditions.
fn stol(model: ModelKind) -> i32 {
    let winds = [wind(270.0, 0.0, false), wind(90.0, 10.0, false), wind(360.0, 10.0, false), wind(240.0, 10.0, true)];
    let mut tally: std::collections::BTreeMap<String, (usize, usize)> = Default::default();
    for len in [1200.0, 600.0, 450.0] {
        for short in [false, true] {
            for w in winds {
                for seed in [3.0, 9.0] {
                    let s = fly_strip(model, w, seed, high_circuit(), len, short);
                    let ok = s.passed();
                    let e = tally.entry(format!("{:>4} m {}", len, if short { "short-field" } else { "normal     " })).or_default();
                    e.1 += 1;
                    if ok {
                        e.0 += 1;
                    }
                    if seed == 3.0 {
                        let rows = s.evaluate();
                        let pick = |label: &str| rows.iter().find(|r| r.label == label).and_then(|r| r.value.clone()).unwrap_or_else(|| "—".into());
                        let failed: Vec<String> = rows.iter().filter(|r| !r.info && r.pass == Some(false)).map(|r| r.label.clone()).collect();
                        println!(
                            "{:>4} m {:<11} {:03}/{:>2}{} {} | TO {} | LDG {} | td {} sink {} | obst {}/{} GA={} {}",
                            len, if short { "short-field" } else { "normal" }, w.from_deg, w.speed_kt, if w.gusts { "G" } else { " " },
                            if ok { "PASS" } else { "FAIL" }, pick("Take-off distance"), pick("Landing distance"),
                            pick("Touchdown point"), pick("Touchdown sink rate"), pick("Cleared departure trees"), pick("Cleared approach trees"),
                            s.autopilot.go_arounds, if failed.is_empty() { String::new() } else { format!("fails: {}", failed.join(", ")) }
                        );
                    }
                }
            }
        }
    }
    println!("pass rates (4 winds × 2 seeds):");
    for (k, (p, t)) in tally {
        println!("  {k}: {p}/{t}");
    }
    0
}
