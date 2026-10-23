//! Automatic tuning (native only): score a `Tuning` on a fixed set of flights, check the inner loops'
//! step responses with margin, and search with a separable CMA-ES.
//!
//! * **Fixed cases.** Every candidate flies the same flights (winds, seeds, failure points, strips and
//!   model errors), so two candidates differ only by their parameters, not by luck.
//! * **Soft cost.** Each failed flight costs 1, and every flight adds a little for how close it came to a
//!   limit (sink, crab, centreline, touchdown point). This rewards margin, and gives the search a slope
//!   to follow between pass/fail steps.
//! * **Model error.** Half the flights use a perturbed airframe (mass, control power, actuator lag,
//!   pitch stability), so the result has to be robust, not tuned to one exact model.
//! * **Margin check.** Before flying, a candidate's pitch and roll step responses are simulated on the
//!   nominal model and on a "hot" one (control power × 1.6 ≈ 4 dB of gain margin, actuator lag × 2.5 for
//!   phase margin). Any candidate that overshoots too much or rings is rejected outright.
//! * **Hold-out.** Search on training seeds, judge on separate test seeds.

use crate::aircraft::{Aircraft, Commands, ModelKind};
use crate::airfield::{high_circuit, standard_circuit};
use crate::atmosphere::{Rng, Wind, WindSettings};
use crate::scenarios::eo_triggers;
use crate::simulation::{SimOptions, Simulation};
use crate::sixdof::analysis::trim;
use crate::sixdof::quat_from_euler;
use crate::tuning::{Perturb, Tuning, DIMS};
use crate::units::{clamp, D2R, FT, KT, PI};

#[derive(Clone, Copy, Debug)]
pub enum Kind {
    Normal,
    EngineFailure(usize),
    ShortStrip(f64),
}

#[derive(Clone, Copy, Debug)]
pub struct Case {
    pub wind: WindSettings,
    pub high: bool,
    pub seed: f64,
    pub kind: Kind,
    pub perturb: Perturb,
}

/// A reproducible mix: 40% normal circuits, 40% engine failures, 20% short-field on 600/450 m strips;
/// winds 0–15 kt from anywhere, 40% gusty; half the flights on a perturbed airframe.
pub fn cases(n: usize, seed: f64) -> Vec<Case> {
    let mut r = Rng::new(seed);
    let n_trig = eo_triggers().len();
    (0..n)
        .map(|i| {
            let wind = WindSettings {
                from_deg: (r.next() * 36.0).floor() * 10.0,
                speed_kt: (r.next() * 16.0).floor(),
                gusts: r.next() < 0.4,
            };
            let high = r.next() < 0.5;
            let seed = (r.next() * 1e6).floor() + 1.0;
            let u = r.next();
            let kind = if u < 0.4 {
                Kind::Normal
            } else if u < 0.8 {
                Kind::EngineFailure(1 + (r.next() * (n_trig - 1) as f64) as usize % (n_trig - 1))
            } else {
                Kind::ShortStrip(if r.next() < 0.5 { 600.0 } else { 450.0 })
            };
            let perturb = if i % 2 == 1 {
                Perturb {
                    mass: 0.9 + 0.18 * r.next(),
                    control_power: 0.8 + 0.4 * r.next(),
                    actuator_lag: 1.0 + 1.0 * r.next(),
                    pitch_stability: 0.75 + 0.5 * r.next(),
                }
            } else {
                Perturb::default()
            };
            Case { wind, high, seed, kind, perturb }
        })
        .collect()
}

pub fn fly(c: &Case, model: ModelKind, tu: &Tuning) -> Simulation {
    let (len, short) = match c.kind {
        Kind::ShortStrip(l) => (l, true),
        _ => (1200.0, false),
    };
    let mut s = Simulation::new(SimOptions {
        wind: c.wind,
        seed: c.seed,
        circuit: if c.high { high_circuit() } else { standard_circuit() },
        model,
        runway_length: len,
        short_field: short,
        tuning: Some(tu.clone()),
        perturb: c.perturb,
    });
    let trig = match c.kind {
        Kind::EngineFailure(i) => Some(eo_triggers().swap_remove(i)),
        _ => None,
    };
    let mut done = false;
    while !s.finished() && s.t < 1200.0 {
        if let Some(t) = &trig {
            if !done && (t.when)(&s) {
                done = s.fail_engine();
            }
        }
        s.step();
    }
    s
}

/// How close a flight came to the limits, 0 (comfortable) to 4 (at or past every limit).
pub fn closeness(s: &Simulation) -> f64 {
    let td = match &s.touchdown {
        Some(td) if s.finished() => td,
        _ => return 4.0,
    };
    let sink = td.sink / FT;
    let crab = td.crab.abs() / D2R;
    let mut c = clamp((sink - 1.8) / 1.5, 0.0, 1.0) + clamp((crab - 2.5) / 3.0, 0.0, 1.0);
    if s.autopilot.engine_out.is_none() {
        c += clamp((td.y.abs() - 2.0) / 3.5, 0.0, 1.0);
        let rwy = s.runway();
        let limit = if s.autopilot.procedure.short_field { rwy.aim_point + 100.0 } else { rwy.length / 3.0 };
        c += clamp((td.x - (limit - 60.0)) / 80.0, 0.0, 1.0);
    }
    c
}

/// Extra cost for a hard arrival (over 4 ft/s, more over 7 ft/s), wherever it happens — so the search
/// never trades safety for runway landings. In closeness units
/// (×0.15 in the cost): 4 = 0.6 of a failure, 8 for a crash-grade sink.
pub fn safety(s: &Simulation) -> f64 {
    // any hard arrival, on the runway or off it, costs extra: a firmer flare or a thinner margin must
    // not buy runway landings with hard ones
    match &s.touchdown {
        Some(td) => {
            let sink = td.sink / FT;
            (if sink > 4.0 { 4.0 } else { 0.0 }) + (if sink > 7.0 { 4.0 } else { 0.0 })
        }
        None => 0.0,
    }
}

#[derive(Clone, Debug, Default)]
pub struct Score {
    pub passed: usize,
    pub n: usize,
    /// Failures + 0.15 × closeness, averaged per flight.
    pub cost: f64,
}

pub fn evaluate(cs: &[Case], model: ModelKind, tu: &Tuning, threads: usize) -> Score {
    let results: Vec<(bool, f64)> = std::thread::scope(|sc| {
        let handles: Vec<_> = (0..threads)
            .map(|k| {
                sc.spawn(move || {
                    cs.iter()
                        .enumerate()
                        .filter(|(i, _)| i % threads == k)
                        .map(|(_, c)| {
                            let s = fly(c, model, tu);
                            (s.passed(), closeness(&s) + safety(&s))
                        })
                        .collect::<Vec<_>>()
                })
            })
            .collect();
        handles.into_iter().flat_map(|h| h.join().unwrap()).collect()
    });
    let passed = results.iter().filter(|r| r.0).count();
    let cost = results.iter().map(|(p, c)| (if *p { 0.0 } else { 1.0 }) + 0.15 * c).sum::<f64>() / cs.len() as f64;
    Score { passed, n: cs.len(), cost }
}

/// Step response of one inner loop.
#[derive(Clone, Debug)]
pub struct Step {
    pub axis: &'static str,
    pub overshoot: f64,
    /// Time to stay within 10% of the step, s (∞ if never).
    pub settle: f64,
    /// Peak-to-peak of the error over the last 2 s, as a fraction of the step (ringing).
    pub ring: f64,
}

/// Pitch (+4°) and bank (20°) commands, slewed at 3× the operational rates, in free air at `v_kt` with `flap`, on the 6DOF model.
pub fn step_responses(tu: &Tuning, v_kt: f64, flap: f64, pt: Perturb) -> Vec<Step> {
    let mut out = Vec::new();
    for axis in ["pitch", "roll"] {
        let mut spec = crate::aircraft::c172_like();
        spec.mass *= pt.mass;
        let mut ac = Aircraft::new(spec.clone(), ModelKind::SixDof);
        let v = v_kt * KT;
        let tr = {
            let six = ac.six.as_ref().unwrap();
            let mut m = six.clone();
            m.x[2] = -2000.0;
            m.prm.control_power = pt.control_power;
            m.prm.pitch_stability = pt.pitch_stability;
            trim(&m, v, 0.0, flap)
        };
        {
            let six = ac.six.as_mut().unwrap();
            let g = &mut six.fcs.gains;
            g.k_theta = tu.k_theta;
            g.k_q = tu.k_q;
            g.ki_theta = tu.ki_theta;
            g.k_phi = tu.k_phi;
            g.k_p = tu.k_p;
            g.k_beta = tu.k_beta;
            g.k_r = tu.k_r;
            six.prm.control_power = pt.control_power;
            six.prm.pitch_stability = pt.pitch_stability;
            six.prm.actuator_tau *= pt.actuator_lag;
            six.x = [0.0; crate::sixdof::N_STATE];
            six.x[2] = -(six.h0 + 600.0);
            six.x[3] = v * tr.alpha.cos();
            six.x[5] = v * tr.alpha.sin();
            six.x[6..10].copy_from_slice(&quat_from_euler(0.0, tr.alpha, PI / 2.0));
            six.ctl.de = tr.de;
            six.ctl.flap = flap;
            // pre-load the pitch integrator with the trim elevator
            let sched = clamp((g.v_ref / v.max(12.0)).powi(2), 0.3, 4.0);
            six.fcs.i_theta = -tr.de / (g.ki_theta * sched);
        }
        ac.on_ground = false;
        ac.flap_pos = flap;
        ac.thr = tr.thr.max(0.0);
        ac.shaper.flaps = flap;
        ac.shaper.thr = ac.thr;
        ac.shaper.theta = tr.alpha;
        // three times the operational slew rates: harsher than anything the autopilot asks for
        ac.shaper.rates.theta = 9.0 * D2R;
        ac.shaper.rates.phi = 24.0 * D2R;
        let wind = Wind::new(WindSettings { from_deg: 0.0, speed_kt: 0.0, gusts: false }, Rng::new(1.0));
        let mut cmd = Commands { theta: tr.alpha, phi: 0.0, thr: ac.thr, flaps: flap, ..Default::default() };
        let dt = 0.02;
        // settle 4 s, then step and watch 8 s
        for _ in 0..200 {
            ac.step(dt, &cmd, &wind);
        }
        let (amp, base) = if axis == "pitch" { (4.0 * D2R, ac.theta) } else { (20.0 * D2R, ac.phi) };
        if axis == "pitch" {
            cmd.theta = base + amp;
        } else {
            cmd.phi = amp;
            cmd.theta = ac.theta;
        }
        let mut errs = Vec::new();
        for _ in 0..400 {
            ac.step(dt, &cmd, &wind);
            let y = if axis == "pitch" { ac.theta - base } else { ac.phi };
            errs.push((amp - y) / amp);
        }
        let overshoot = errs.iter().cloned().fold(0.0_f64, |m, e| m.max(-e));
        let last_out = errs.iter().rposition(|e| e.abs() > 0.1);
        let settle = match last_out {
            Some(i) if i + 1 >= errs.len() => f64::INFINITY,
            Some(i) => (i + 1) as f64 * dt,
            None => 0.0,
        };
        // ringing: peak-to-peak of the last 2 s about its straight-line trend (slow drift is not ringing)
        let tail = &errs[errs.len() - 100..];
        let n = tail.len() as f64;
        let mx = (n - 1.0) / 2.0;
        let my = tail.iter().sum::<f64>() / n;
        let sxy: f64 = tail.iter().enumerate().map(|(i, y)| (i as f64 - mx) * (y - my)).sum();
        let sxx: f64 = (0..tail.len()).map(|i| (i as f64 - mx).powi(2)).sum();
        let slope = sxy / sxx;
        let det: Vec<f64> = tail.iter().enumerate().map(|(i, y)| y - my - slope * (i as f64 - mx)).collect();
        let ring = det.iter().cloned().fold(f64::MIN, f64::max) - det.iter().cloned().fold(f64::MAX, f64::min);
        out.push(Step { axis, overshoot, settle, ring });
    }
    out
}

/// The margin check: nominal and "hot" (gain and lag margin) step responses at approach and cruise.
/// Returns a reason when the candidate is rejected.
pub fn margin_check(tu: &Tuning) -> Result<(), String> {
    let hot = Perturb { control_power: 1.6, actuator_lag: 2.5, ..Default::default() };
    for (v, flap) in [(65.0, 30.0), (100.0, 0.0)] {
        for (label, pt, os_max, settle_max) in [("nominal", Perturb::default(), 0.25, 5.0), ("hot", hot, 0.5, 7.0)] {
            for s in step_responses(tu, v, flap, pt) {
                if s.overshoot > os_max || s.settle > settle_max || s.ring > 0.05 {
                    return Err(format!(
                        "{} {} at {} kt: overshoot {:.0}%, settle {:.1} s, ringing {:.0}%",
                        label,
                        s.axis,
                        v,
                        100.0 * s.overshoot,
                        s.settle,
                        100.0 * s.ring
                    ));
                }
            }
        }
    }
    Ok(())
}

/// Standard normal deviate (Box–Muller) from the crate's PRNG.
fn gauss(r: &mut Rng) -> f64 {
    let u1 = r.next().max(1e-12);
    let u2 = r.next();
    (-2.0 * u1.ln()).sqrt() * (2.0 * PI * u2).cos()
}

pub struct SearchLog {
    pub best: Tuning,
    pub best_score: Score,
    pub baseline: Score,
    pub evaluations: usize,
    pub rejected: usize,
}

/// Separable CMA-ES over the normalised parameters (`tuning::DIMS`), starting at `base`.
pub fn search(
    model: ModelKind,
    base: &Tuning,
    train: &[Case],
    generations: usize,
    lambda: usize,
    threads: usize,
    seed: f64,
    mut report: impl FnMut(&str),
) -> SearchLog {
    const N: usize = crate::tuning::NT;
    let nf = N as f64;
    let mu = lambda / 2;
    let w_raw: Vec<f64> = (0..mu).map(|i| (mu as f64 + 0.5).ln() - ((i + 1) as f64).ln()).collect();
    let ws: f64 = w_raw.iter().sum();
    let w: Vec<f64> = w_raw.iter().map(|x| x / ws).collect();
    let mueff = 1.0 / w.iter().map(|x| x * x).sum::<f64>();
    let cs = (mueff + 2.0) / (nf + mueff + 5.0);
    let ds = 1.0 + 2.0 * (((mueff - 1.0) / (nf + 1.0)).sqrt() - 1.0).max(0.0) + cs;
    let cc = 4.0 / (nf + 4.0);
    let c1 = 2.0 / ((nf + 1.3).powi(2) + mueff) * (nf + 2.0) / 3.0;
    let cmu = (2.0 * (mueff - 2.0 + 1.0 / mueff) / ((nf + 2.0).powi(2) + mueff) * (nf + 2.0) / 3.0).min(1.0 - c1);
    let chi_n = nf.sqrt() * (1.0 - 1.0 / (4.0 * nf) + 1.0 / (21.0 * nf * nf));

    let mut rng = Rng::new(seed);
    let mut m = [0.0; N];
    let mut sigma: f64 = 0.6;
    let mut c = [1.0_f64; N];
    let mut ps = [0.0; N];
    let mut pc = [0.0; N];

    let baseline = evaluate(train, model, base, threads);
    report(&format!(
        "baseline: {}/{} passed, cost {:.4}",
        baseline.passed, baseline.n, baseline.cost
    ));
    let mut best = base.clone();
    let mut best_score = baseline.clone();
    let mut evaluations = 1;
    let mut rejected = 0;

    for g in 0..generations {
        let mut pop: Vec<([f64; N], f64)> = Vec::new();
        for _ in 0..lambda {
            let mut z = [0.0; N];
            for i in 0..N {
                z[i] = m[i] + sigma * c[i].sqrt() * gauss(&mut rng);
            }
            let tu = Tuning::at(base, &z);
            let cost = match margin_check(&tu) {
                Err(_) => {
                    rejected += 1;
                    10.0
                }
                Ok(()) => {
                    evaluations += 1;
                    let sc = evaluate(train, model, &tu, threads);
                    if sc.cost < best_score.cost {
                        best = tu.clone();
                        best_score = sc.clone();
                    }
                    sc.cost
                }
            };
            pop.push((z, cost));
        }
        pop.sort_by(|a, b| a.1.partial_cmp(&b.1).unwrap());
        let old = m;
        for i in 0..N {
            m[i] = (0..mu).map(|k| w[k] * pop[k].0[i]).sum();
        }
        let mut norm_ps = 0.0;
        for i in 0..N {
            let y = (m[i] - old[i]) / sigma;
            ps[i] = (1.0 - cs) * ps[i] + (cs * (2.0 - cs) * mueff).sqrt() * y / c[i].sqrt();
            norm_ps += ps[i] * ps[i];
        }
        let norm_ps = norm_ps.sqrt();
        let hsig = if norm_ps / (1.0 - (1.0 - cs).powi(2 * (g as i32 + 1))).sqrt() / chi_n < 1.4 + 2.0 / (nf + 1.0) {
            1.0
        } else {
            0.0
        };
        for i in 0..N {
            let y = (m[i] - old[i]) / sigma;
            pc[i] = (1.0 - cc) * pc[i] + hsig * (cc * (2.0 - cc) * mueff).sqrt() * y;
            let rank_mu: f64 = (0..mu).map(|k| w[k] * ((pop[k].0[i] - old[i]) / sigma).powi(2)).sum();
            c[i] = (1.0 - c1 - cmu) * c[i] + c1 * pc[i] * pc[i] + cmu * rank_mu;
        }
        sigma *= ((cs / ds) * (norm_ps / chi_n - 1.0)).exp();
        sigma = sigma.clamp(0.05, 1.5);
        report(&format!(
            "gen {:>2}: best of generation cost {:.4}, best so far {:.4} ({}/{}), σ {:.2}, rejected so far {}",
            g + 1,
            pop[0].1,
            best_score.cost,
            best_score.passed,
            best_score.n,
            sigma,
            rejected
        ));
    }
    SearchLog { best, best_score, baseline, evaluations, rejected }
}

/// Rust source for a `Tuning` literal, to paste into `Tuning::default()`.
pub fn as_rust(t: &Tuning) -> String {
    let v = t.to_vec();
    let mut s = String::from("Tuning {\n");
    for (i, d) in DIMS.iter().enumerate() {
        let x = if d.log { (v[i] * 100.0).round() / 100.0 } else { (v[i] * 100.0).round() / 100.0 };
        s.push_str(&format!("    {}: {:?},\n", d.name, x));
    }
    s.push('}');
    s
}
