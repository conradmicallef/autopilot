//! C-ABI exports for the browser build. One simulation instance lives in module memory; the page
//! steps it and reads a JSON frame (state, annunciators, plans, events, evaluation) each animation
//! frame. Only plain numbers cross the boundary, plus one byte buffer for the JSON text (written by
//! `jsonw`, so neither serde_json nor the float formatter is linked in).

use crate::airfield::{high_circuit, standard_circuit};
use crate::atmosphere::WindSettings;
use crate::simulation::{SimOptions, Simulation};
use crate::ModelKind;
use crate::airfield::{OBSTACLE_HEIGHT, OBSTACLE_OFFSET};
use crate::autopilot::EngineOutPlan;
use crate::jsonw::Json;
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
        // 6DOF only; the argument is kept so the page's call signature stays the same
        model: { let _ = model; ModelKind::SixDof },
        runway_length: if runway_length > 100.0 { runway_length } else { 1200.0 },
        short_field: short_field != 0,
        ..Default::default()
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

/// Put the current simulation into a random airborne state (an upset possibly), to recover and land;
/// `engine_out` != 0 also stops the engine. Returns 1 when started.
#[no_mangle]
pub extern "C" fn sim_random_start(seed: f64, engine_out: u32) -> u32 {
    with(|h| {
        let mut rng = crate::atmosphere::Rng::new(seed);
        let a = crate::simulation::AirStart::random(&h.sim, &mut rng, engine_out != 0);
        h.sim.start_airborne(&a);
        h.track_cursor = 0;
        1
    })
    .unwrap_or(0)
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
        h.out = v.into_bytes();
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

fn frame(s: &Simulation, cursor: usize) -> String {
    let ac = &s.aircraft;
    let ap = &s.autopilot;
    let gv = ac.ground_velocity(&s.wind);
    let end = ap.eo_end();
    let aim = ap.planner.aim_at(ap.glide.as_ref().map_or(ap.planner.spec.aim_point, |g| g.aim), &end);
    let rwy = s.runway();
    let mut j = Json::default();
    j.obj();
    j.k("t").num(s.t).k("phase").str(s.phase().code()).k("finished").bool(s.finished()).k("model").str(ac.model.name());

    j.k("aircraft").obj();
    j.k("x").num(ac.x).k("y").num(ac.y).k("h").num(ac.h).k("V").num(ac.v).k("Vg").num(ac.vg);
    j.k("psi").num(ac.psi).k("beta").num(ac.beta).k("gamma").num(ac.gamma).k("theta").num(ac.theta).k("phi").num(ac.phi);
    j.k("thr").num(ac.thr).k("flapPos").num(ac.flap_pos).k("flaps").int(ac.flaps() as i64);
    j.k("alpha").num(if ac.v < 10.0 { 0.0 } else { ac.alpha() }).k("verticalSpeed").num(ac.vertical_speed());
    j.k("noseHeading").num(ac.nose_heading()).k("track").num(gv.y.atan2(gv.x)).k("groundSpeed").num(gv.x.hypot(gv.y));
    j.k("stalled").bool(ac.stalled).k("onGround").bool(ac.on_ground).k("engineFailed").bool(ac.engine_failed);
    j.k("stillAirReach").num(ap.planner.still_air_reach(ac));
    let (fc, fr) = ap.planner.footprint(ac, ap.gust.mean_wind());
    j.k("glideFootprint").obj().k("c").point(fc).k("r").num(fr).end_obj();
    j.k("six");
    match &ac.six {
        Some(x) => {
            j.obj().k("de").num(x.ctl.de).k("da").num(x.ctl.da).k("dr").num(x.ctl.dr).k("steer").num(x.ctl.steer);
            j.k("loads").arr();
            for l in x.diag.loads {
                j.num(l);
            }
            j.end_arr().k("alpha").num(x.diag.alpha).k("beta").num(x.diag.beta).end_obj();
        }
        None => {
            j.null();
        }
    }
    j.end_obj();

    j.k("autopilot").obj();
    j.k("fma").obj().k("thr").str(&ap.fma.thr).k("vert").str(&ap.fma.vert).k("lat").str(&ap.fma.lat).end_obj();
    j.k("descending").bool(ap.descending).k("goArounds").int(ap.go_arounds as i64);
    j.k("goAroundReason").str(&ap.go_around_reason).k("goAroundRequested").bool(ap.go_around_requested);
    j.k("engineOut");
    match &ap.engine_out {
        Some(e) => {
            let plan = match e.plan {
                EngineOutPlan::Reject => "REJECT",
                EngineOutPlan::LandAhead => "LAND_AHEAD",
                EngineOutPlan::Return => "RETURN",
            };
            j.obj().k("plan").str(plan).k("why").str(&e.why).k("t").num(e.t).k("x").num(e.x).k("y").num(e.y);
            j.k("h").num(e.h).k("track").num(e.track).end_obj();
        }
        None => {
            j.null();
        }
    }
    j.k("glide");
    match &ap.glide {
        Some(g) => {
            j.obj().k("route").arr();
            for p in &g.route {
                j.point(*p);
            }
            j.end_arr().k("ipDistance").num(g.ip_distance).k("end").str(g.end.name).k("reachable").bool(g.reachable).end_obj();
        }
        None => {
            j.null();
        }
    }
    j.k("eoEnd").obj().k("name").str(end.name).k("heading").num(end.heading).k("threshold").num(end.threshold);
    j.k("dir").num(end.dir).k("aim").point(aim).end_obj();
    j.end_obj();

    let w = &s.wind;
    j.k("wind").obj().k("fromDeg").num(w.settings.from_deg).k("speedKt").num(w.settings.speed_kt);
    j.k("gusts").bool(w.settings.gusts).k("vx").num(w.v.x).k("vy").num(w.v.y).end_obj();

    j.k("circuit").obj().k("name").str(ap.circuit.spec.name).k("plannedRoute").arr();
    for p in ap.circuit.planned_route() {
        j.point(p);
    }
    j.end_arr().k("baseX").num(ap.circuit.spec.base_x).k("downwindOffset").num(ap.circuit.spec.downwind_offset).end_obj();

    j.k("runway").obj().k("length").num(rwy.length).k("width").num(rwy.width).k("aimPoint").num(rwy.aim_point);
    j.k("obstacles").bool(s.obstacles).k("obstacleOffset").num(OBSTACLE_OFFSET).k("obstacleHeight").num(OBSTACLE_HEIGHT).end_obj();
    j.k("procedure").str(ap.procedure.name);
    let p = &s.perf;
    j.k("perf").obj().k("liftoff_x").opt_num(p.liftoff_x).k("takeoff_50ft_x").opt_num(p.takeoff_50ft_x);
    j.k("departure_obstacle_h").opt_num(p.departure_obstacle_h).k("landing_50ft_x").opt_num(p.landing_50ft_x);
    j.k("approach_obstacle_h").opt_num(p.approach_obstacle_h).end_obj();

    j.k("touchdown");
    match &s.touchdown {
        Some(td) => {
            j.obj().k("t").num(td.t).k("x").num(td.x).k("y").num(td.y).k("sink").num(td.sink).k("V").num(td.v);
            j.k("crab").num(td.crab).k("onRunway").bool(td.on_runway).end_obj();
        }
        None => {
            j.null();
        }
    }
    j.k("reach").obj().k("reachable").opt_bool(s.runway_reachable).k("end").opt_str(s.reach_end);
    j.k("fraction").num(s.reach_fraction()).k("airTime").num(s.air_time).end_obj();

    j.k("events").arr();
    for e in &ap.events {
        j.obj().k("t").num(e.t).k("phase").str(e.phase).k("h").num(e.h).k("V").num(e.v).k("x").num(e.x).k("y").num(e.y);
        j.k("note").opt_str(e.note.as_deref()).end_obj();
    }
    j.end_arr();

    j.k("newTrack").arr();
    for q in &s.track[cursor.min(s.track.len())..] {
        j.obj().k("t").num(q.t).k("x").num(q.x).k("y").num(q.y).k("h").num(q.h).k("V").num(q.v).k("phase").str(q.phase);
        j.k("reach").opt_bool(q.reach).end_obj();
    }
    j.end_arr();

    j.k("evaluation").arr();
    for r in s.evaluate() {
        j.obj().k("label").str(&r.label).k("target").str(&r.target).k("value").opt_str(r.value.as_deref());
        j.k("pass").opt_bool(r.pass).k("info").bool(r.info).end_obj();
    }
    j.end_arr();
    j.k("passed").bool(s.passed());
    j.end_obj();
    j.s
}
