//! Shared test scenarios: the same cases the TypeScript suites flew, usable from the CLI and tests.

use crate::airfield::{high_circuit, standard_circuit, CircuitSpec};
use crate::atmosphere::WindSettings;
use crate::autopilot::Phase;
use crate::simulation::{SimOptions, Simulation};
use crate::units::{FT, KT};
use crate::ModelKind;

pub fn wind(from_deg: f64, speed_kt: f64, gusts: bool) -> WindSettings {
    WindSettings { from_deg, speed_kt, gusts }
}

/// Normal circuits (and manual go-arounds): (wind, seed, go-around trigger).
pub fn normal_cases() -> Vec<(WindSettings, f64, Option<&'static str>)> {
    vec![
        (wind(270.0, 0.0, false), 1.0, None),
        (wind(270.0, 8.0, false), 1.0, None),
        (wind(300.0, 15.0, false), 1.0, None),
        (wind(360.0, 15.0, false), 1.0, None),
        (wind(180.0, 15.0, false), 1.0, None),
        (wind(360.0, 20.0, false), 1.0, None),
        (wind(200.0, 12.0, false), 1.0, None),
        (wind(220.0, 12.0, true), 7.0, None),
        (wind(90.0, 6.0, false), 1.0, None),
        (wind(300.0, 20.0, true), 3.0, None),
        (wind(360.0, 25.0, true), 11.0, None),
        (wind(270.0, 8.0, false), 5.0, Some("final")),
        (wind(270.0, 8.0, false), 5.0, Some("flare")),
        (wind(270.0, 8.0, false), 5.0, Some("base")),
    ]
}

pub fn ga_trigger(sim: &Simulation, which: &str) -> bool {
    let p = sim.phase();
    let h = sim.aircraft.h;
    match which {
        "final" => p == Phase::Final && h < 200.0 * FT,
        "flare" => p == Phase::Flare && h < 2.0,
        "base" => p == Phase::Base,
        _ => false,
    }
}

pub struct EoTrigger {
    pub name: &'static str,
    pub when: fn(&Simulation) -> bool,
}

pub fn eo_triggers() -> Vec<EoTrigger> {
    vec![
        EoTrigger { name: "roll 40kt", when: |s| s.phase() == Phase::TakeoffRoll && s.aircraft.v > 40.0 * KT },
        EoTrigger { name: "climb 200ft", when: |s| s.phase() == Phase::Climb && s.aircraft.h > 200.0 * FT },
        EoTrigger { name: "climb 450ft", when: |s| s.phase() == Phase::Climb && s.aircraft.h > 450.0 * FT },
        EoTrigger { name: "crosswind", when: |s| s.phase() == Phase::Crosswind && s.aircraft.h > 700.0 * FT },
        EoTrigger { name: "downwind early", when: |s| s.phase() == Phase::Downwind && s.aircraft.x < 1800.0 },
        EoTrigger { name: "downwind mid", when: |s| s.phase() == Phase::Downwind && s.aircraft.x < 1000.0 },
        EoTrigger { name: "abeam thr", when: |s| s.phase() == Phase::Downwind && s.aircraft.x < 0.0 },
        EoTrigger { name: "base", when: |s| s.phase() == Phase::Base && s.aircraft.y < 700.0 },
        EoTrigger { name: "final 400ft", when: |s| s.phase() == Phase::Final && s.aircraft.h < 400.0 * FT },
        EoTrigger { name: "final 150ft", when: |s| s.phase() == Phase::Final && s.aircraft.h < 150.0 * FT },
    ]
}

pub fn eo_winds() -> Vec<WindSettings> {
    vec![wind(270.0, 0.0, false), wind(270.0, 12.0, false), wind(360.0, 12.0, true)]
}

pub fn circuits() -> Vec<CircuitSpec> {
    vec![standard_circuit(), high_circuit()]
}

pub fn new_sim(model: ModelKind, w: WindSettings, seed: f64, circuit: CircuitSpec) -> Simulation {
    Simulation::new(SimOptions { wind: w, seed, circuit, model, runway_length: 1200.0, short_field: false, ..Default::default() })
}

/// Fly a full circuit on a given strip with normal or short-field technique.
pub fn fly_strip(model: ModelKind, w: WindSettings, seed: f64, circuit: CircuitSpec, length: f64, short: bool) -> Simulation {
    let mut s = Simulation::new(SimOptions { wind: w, seed, circuit, model, runway_length: length, short_field: short, ..Default::default() });
    s.run(1500.0);
    s
}

/// Fly one normal-circuit case to completion.
pub fn fly_normal(model: ModelKind, w: WindSettings, seed: f64, ga: Option<&str>, circuit: CircuitSpec) -> Simulation {
    let mut s = new_sim(model, w, seed, circuit);
    let mut asked = false;
    while !s.finished() && s.t < 1500.0 {
        if let Some(g) = ga {
            if !asked && ga_trigger(&s, g) {
                asked = s.request_go_around();
            }
        }
        s.step();
    }
    s
}

/// Fly one engine-failure case to completion.
pub fn fly_eo(model: ModelKind, w: WindSettings, seed: f64, trig: &EoTrigger, circuit: CircuitSpec) -> Simulation {
    let mut s = new_sim(model, w, seed, circuit);
    let mut done = false;
    while !s.finished() && s.t < 1200.0 {
        if !done && (trig.when)(&s) {
            done = s.fail_engine();
        }
        s.step();
    }
    s
}
