//! C-ABI exports for the browser build. One simulation instance lives in module memory; the page
//! steps it and reads a JSON frame (state, annunciators, plans, events, evaluation) each animation
//! frame. Only plain numbers cross the boundary, plus one byte buffer for the JSON text.

use crate::airfield::{high_circuit, standard_circuit};
use crate::atmosphere::WindSettings;
use crate::simulation::{SimOptions, Simulation};
use crate::ModelKind;
use serde_json::{json, Value};
use std::cell::RefCell;

struct Host {
    sim: Simulation,
    /// Index of the first track point not yet sent to the page.
    track_cursor: usize,
    out: Vec<u8>,
}

thread_local! {
    static HOST: RefCell<Option<Host>> = const { RefCell::new(None) };
}

fn with<R>(f: impl FnOnce(&mut Host) -> R) -> Option<R> {
    HOST.with(|h| h.borrow_mut().as_mut().map(f))
}

/// Create (or replace) the simulation. model: 0 point-mass, 1 6DOF. circuit: 0 standard, 1 high.
#[no_mangle]
pub extern "C" fn sim_new(
    model: u32,
    circuit: u32,
    from_deg: f64,
    speed_kt: f64,
    gusts: u32,
    seed: f64,
    runway_length: f64,
    short_field: u32,
) {
    let opts = SimOptions {
        wind: WindSettings { from_deg, speed_kt, gusts: gusts != 0 },
        seed,
        circuit: if circuit == 1 { high_circuit() } else { standard_circuit() },
        model: if model == 1 { ModelKind::SixDof } else { ModelKind::PointMass },
        runway_length: if runway_length > 100.0 { runway_length } else { 1200.0 },
        short_field: short_field != 0,
    };
    HOST.with(|h| {
        *h.borrow_mut() = Some(Host { sim: Simulation::new(opts), track_cursor: 0, out: Vec::new() });
    });
}

#[no_mangle]
pub extern "C" fn sim_set_wind(from_deg: f64, speed_kt: f64, gusts: u32) {
    with(|h| h.sim.wind.settings = WindSettings { from_deg, speed_kt, gusts: gusts != 0 });
}

/// Advance `n` fixed 20 ms steps. Returns 1 once the aircraft has stopped.
#[no_mangle]
pub extern "C" fn sim_step(n: u32) -> u32 {
    with(|h| {
        for _ in 0..n {
            if h.sim.finished() {
                break;
            }
            h.sim.step();
        }
        h.sim.finished() as u32
    })
    .unwrap_or(1)
}

#[no_mangle]
pub extern "C" fn sim_fail_engine() -> u32 {
    with(|h| h.sim.fail_engine() as u32).unwrap_or(0)
}

#[no_mangle]
pub extern "C" fn sim_request_go_around() -> u32 {
    with(|h| h.sim.request_go_around() as u32).unwrap_or(0)
}

/// Build the JSON frame into the output buffer; returns its length. Read it at `sim_out_ptr()`.
#[no_mangle]
pub extern "C" fn sim_frame() -> usize {
    with(|h| {
        let v = frame(&h.sim, h.track_cursor);
        h.track_cursor = h.sim.track.len();
        h.out = serde_json::to_vec(&v).unwrap_or_default();
        h.out.len()
    })
    .unwrap_or(0)
}

/// Rewind the track cursor so the next frame resends the whole track (e.g. after the page redraws).
#[no_mangle]
pub extern "C" fn sim_resend_track() {
    with(|h| h.track_cursor = 0);
}

#[no_mangle]
pub extern "C" fn sim_out_ptr() -> *const u8 {
    with(|h| h.out.as_ptr()).unwrap_or(std::ptr::null())
}

fn frame(s: &Simulation, cursor: usize) -> Value {
    let ac = &s.aircraft;
    let ap = &s.autopilot;
    let gv = ac.ground_velocity(&s.wind);
    let six = ac.six.as_ref().map(|x| {
        json!({
            "de": x.ctl.de, "da": x.ctl.da, "dr": x.ctl.dr, "steer": x.ctl.steer,
            "loads": x.diag.loads, "alpha": x.diag.alpha, "beta": x.diag.beta,
        })
    });
    let end = ap.eo_end();
    let aim = ap.planner.aim_for(&end);
    json!({
        "t": s.t,
        "phase": s.phase().code(),
        "finished": s.finished(),
        "model": ac.model.name(),
        "aircraft": {
            "x": ac.x, "y": ac.y, "h": ac.h, "V": ac.v, "Vg": ac.vg,
            "psi": ac.psi, "beta": ac.beta, "gamma": ac.gamma, "theta": ac.theta, "phi": ac.phi,
            "thr": ac.thr, "flapPos": ac.flap_pos, "flaps": ac.flaps(),
            "alpha": if ac.v < 10.0 { 0.0 } else { ac.alpha() }, "verticalSpeed": ac.vertical_speed(),
            "noseHeading": ac.nose_heading(), "track": gv.y.atan2(gv.x), "groundSpeed": gv.x.hypot(gv.y),
            "stalled": ac.stalled, "onGround": ac.on_ground, "engineFailed": ac.engine_failed,
            "stillAirReach": ap.planner.still_air_reach(ac),
            "six": six,
        },
        "autopilot": {
            "fma": ap.fma,
            "descending": ap.descending,
            "goArounds": ap.go_arounds,
            "goAroundReason": ap.go_around_reason,
            "goAroundRequested": ap.go_around_requested,
            "engineOut": ap.engine_out,
            "glide": ap.glide.as_ref().map(|g| json!({ "route": g.route, "ipDistance": g.ip_distance, "end": g.end.name, "reachable": g.reachable })),
            "eoEnd": { "name": end.name, "heading": end.heading, "threshold": end.threshold, "dir": end.dir, "aim": aim },
        },
        "wind": { "fromDeg": s.wind.settings.from_deg, "speedKt": s.wind.settings.speed_kt, "gusts": s.wind.settings.gusts, "vx": s.wind.v.x, "vy": s.wind.v.y },
        "circuit": {
            "name": ap.circuit.spec.name,
            "plannedRoute": ap.circuit.planned_route(),
            "baseX": ap.circuit.spec.base_x,
            "downwindOffset": ap.circuit.spec.downwind_offset,
        },
        "runway": {
            "length": s.runway().length, "width": s.runway().width, "aimPoint": s.runway().aim_point,
            "obstacles": s.obstacles, "obstacleOffset": crate::airfield::OBSTACLE_OFFSET, "obstacleHeight": crate::airfield::OBSTACLE_HEIGHT,
        },
        "procedure": ap.procedure.name,
        "perf": s.perf,
        "touchdown": s.touchdown,
        "reach": { "reachable": s.runway_reachable, "end": s.reach_end, "fraction": s.reach_fraction(), "airTime": s.air_time },
        "events": ap.events,
        "newTrack": &s.track[cursor.min(s.track.len())..],
        "evaluation": s.evaluate(),
        "passed": s.passed(),
    })
}
