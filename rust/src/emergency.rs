//! Power-off energy planning: can the runway (either direction) be reached in a glide, and by which path?
//!
//! The path is a Dubins path (turn–straight–turn at the bank actually flown) from the aircraft to a final
//! fix on the extended centreline, then straight in to an aim point on the runway. It is solved in the
//! air mass, where a glide is the same in every direction: there the fix drifts upwind by the wind times
//! the time to get there, so the wind's effect on the turns is included. One height model covers it all:
//!
//! * straight glide: distance / L/D at best-glide speed (the trimmed 6DOF glide, `Aircraft::glide_ratio`);
//! * a turn at bank φ flown at `turn_speed`: load factor n = 1/cos φ and the same lift coefficient as a
//!   1 g glide at turn_speed/√n, so it costs n × arc length / L/D(turn_speed/√n).
//!
//! The plan picks the longest final (up to `fix_max`) that is reachable with the planning margins at the
//! nominal aim point; if none is, nearer aim points down to `aim_min` past the threshold (any part of the
//! runway beats a field). Both runway directions are planned and the one with more height in hand wins.

use crate::aircraft::Aircraft;
use crate::airfield::Runway;
use crate::dubins::{self, DubinsPath, Pose};
use crate::tuning::Tuning;
use crate::units::{clamp, Point, Vec2, D2R, FT, G, KT, PI};
use serde::Serialize;

/// Runway kept beyond the touchdown point for the ground roll in calm air, with a margin, m; it grows
/// with the square of the touchdown ground speed in a tailwind.
const STOP_ALLOWANCE: f64 = 300.0;
/// Touchdown speed of the hold-off, × the full-flap stall speed (as `autopilot::TD_SPEED`).
const TD_SPEED: f64 = 1.05;
/// Height lost raising flap from 20–30° to up, m.
const RETRACT_LOSS: f64 = 8.0;
/// Height lost to rolling into and out of each turn (the arc is flown below the planned bank for a
/// couple of seconds each side), m.
const ROLL_LOSS: f64 = 4.0;
/// Roll rate of the command shaper, rad/s (`aircraft::DEFAULT_SLEW`).
const ROLL_RATE: f64 = 8.0 * D2R;
/// Turn speed: at least this multiple of the stall speed at the turn's load factor.
pub const TURN_STALL_MARGIN: f64 = 1.2;
/// Final-fix distances tried, m apart.
const FIX_STEP: f64 = 50.0;
const AIM_STEP: f64 = 60.0;
/// Shortest straight final before the touchdown, m: time to settle on the centreline and flare.
pub const MIN_FINAL: f64 = 150.0;

#[derive(Clone, Debug, Serialize)]
pub struct EngineOutSpec {
    /// Best-glide speed, clean.
    pub vbg: f64,
    /// Power-off approach speed once flap 20 or more is out.
    pub vapp_eo: f64,
    /// Below this height after take-off, never turn back: land ahead (policy, not physics).
    pub min_turnback_height: f64,
    /// Final fix range, metres before the threshold.
    pub fix_min: f64,
    pub fix_max: f64,
    /// Nominal aim point, and the nearest acceptable one, metres past the threshold.
    pub aim_point: f64,
    pub aim_min: f64,
    /// Required height is multiplied by this, plus `margin_height`, before calling the runway reachable.
    pub margin_factor: f64,
    pub margin_height: f64,
    /// Bank for power-off turns, rad: the plan's turn radius and the bank flown are the same number.
    pub turn_bank: f64,
    /// Height in hand (beyond the margins), as a fraction of the height the path needs, that a plan
    /// keeps before it optimises the aim point: errors grow with the length of a path and its turning.
    pub reserve: f64,
    /// Pitch slew while gliding, rad/s.
    pub pitch_rate: f64,
}

impl EngineOutSpec {
    /// The one constructor: tuned values from `Tuning`, the aim point limited to the first quarter of a
    /// short strip.
    pub fn new(t: &Tuning, runway_length: f64) -> Self {
        EngineOutSpec {
            vbg: t.vbg_kt * KT,
            vapp_eo: t.vapp_eo_kt * KT,
            min_turnback_height: 700.0 * FT,
            fix_min: 150.0,
            fix_max: 1200.0,
            aim_point: 300.0f64.min(0.25 * runway_length),
            aim_min: 60.0,
            margin_factor: t.eo_margin_factor,
            margin_height: t.eo_margin_height,
            turn_bank: t.eo_turn_bank * D2R,
            reserve: t.eo_reserve,
            pitch_rate: 5.0 * D2R,
        }
    }
}

/// One landing direction. 09 lands toward +x from x = 0; 27 lands toward −x from x = length.
#[derive(Clone, Copy, Debug, Serialize, PartialEq)]
pub struct RunwayEnd {
    pub name: &'static str,
    /// Track angle, rad.
    pub heading: f64,
    /// x of the landing threshold.
    pub threshold: f64,
    /// +1 when landing toward +x.
    pub dir: f64,
}

pub fn runway_ends(runway: &Runway) -> [RunwayEnd; 2] {
    [
        RunwayEnd { name: "09", heading: 0.0, threshold: 0.0, dir: 1.0 },
        RunwayEnd { name: "27", heading: PI, threshold: runway.length, dir: -1.0 },
    ]
}

#[derive(Clone, Debug, Serialize)]
pub struct GlideAssessment {
    pub reachable: bool,
    /// Height the path needs, before margins, m.
    pub required: f64,
    /// Height available above the runway, with speed above best glide counted as height, m.
    pub available: f64,
    /// The planned path over the ground (sampled), final fix and aim point included.
    pub route: Vec<Point>,
    /// Where the path joins the centreline, metres before the threshold (negative: over the runway).
    pub ip_distance: f64,
    /// Aim point, metres past the threshold.
    pub aim: f64,
    pub end: RunwayEnd,
    /// Height left over after margins (negative = short).
    pub surplus: f64,
    /// Surplus at the shortest final and nominal aim: how much height this direction has in hand.
    pub headroom: f64,
}

#[derive(Clone, Debug)]
pub struct GlidePlanner {
    pub runway: Runway,
    pub spec: EngineOutSpec,
    pub ends: [RunwayEnd; 2],
}

fn tau_total(p: &DubinsPath, vt: f64, v: f64) -> f64 {
    p.r * p.turn() / vt + p.straight / v
}

/// Wind component along a track angle (positive = tailwind).
pub fn along(w: Vec2, track: f64) -> f64 {
    w.x * track.cos() + w.y * track.sin()
}
/// Wind component to the left of a track angle.
pub fn across(w: Vec2, track: f64) -> f64 {
    -w.x * track.sin() + w.y * track.cos()
}

impl GlidePlanner {
    pub fn new(runway: Runway, spec: EngineOutSpec) -> Self {
        let ends = runway_ends(&runway);
        GlidePlanner { runway, spec, ends }
    }

    pub fn fix_at(&self, distance: f64, end: &RunwayEnd) -> Point {
        [end.threshold - end.dir * distance, 0.0]
    }
    pub fn aim_at(&self, aim: f64, end: &RunwayEnd) -> Point {
        [end.threshold + end.dir * aim, 0.0]
    }
    /// Along-approach distance still to go to a point `aim` past the threshold (positive before it).
    pub fn to_point(&self, x: f64, aim: f64, end: &RunwayEnd) -> f64 {
        end.dir * (end.threshold + end.dir * aim - x)
    }
    /// Farthest useful aim point (any part of the runway will do when high): far enough from the far end
    /// to flare and stop, more so in a tailwind.
    pub fn far_aim(&self, wind: Vec2, end: &RunwayEnd) -> f64 {
        let v = self.spec.vapp_eo;
        let gs = (v + along(wind, end.heading)).max(0.3 * v);
        self.runway.length - STOP_ALLOWANCE * (gs / v).powi(2)
    }
    /// Ground distance floated in the hold-off from the approach speed `v` down to the touchdown speed:
    /// the glide reaches the ground this far before the touchdown point.
    pub fn float_distance(&self, ac: &Aircraft, v: f64, flap: f64, wind: Vec2, end: &RunwayEnd) -> f64 {
        let vtd = TD_SPEED * ac.stall_speed(flap);
        // idle, nearly level: the deceleration is g over the glide ratio at the mean speed (normal
        // landings measure 1.5 m/s² with full flap: 65 → 49 kt over 156 m)
        let vm = 0.5 * (v + vtd);
        let decel = G / ac.glide_ratio(vm, flap);
        let air = ((v * v - vtd * vtd) / (2.0 * decel)).max(0.0);
        // over the ground with the wind
        air * ((vm + along(wind, end.heading)) / vm).max(0.3)
    }
    /// Distance from the final fix to where the glide must reach the ground, for a touchdown `aim` past
    /// the threshold (the float is planned at the power-off approach speed).
    fn glide_final(&self, ac: &Aircraft, wind: Vec2, end: &RunwayEnd, fix_d: f64, aim: f64) -> f64 {
        (fix_d + aim - self.float_distance(ac, self.spec.vapp_eo, 30.0, wind, end)).max(0.0)
    }

    /// Glide speed at a bank angle: best glide, or more when the bank needs it for stall margin.
    pub fn turn_speed(&self, ac: &Aircraft, bank: f64, flap: f64) -> f64 {
        let base = if flap >= 20.0 { self.spec.vapp_eo } else { self.spec.vbg };
        let n = 1.0 / bank.abs().min(60.0 * D2R).cos();
        base.max(TURN_STALL_MARGIN * ac.stall_speed(flap) * n.sqrt())
    }
    /// Radius of the planned power-off turn, m (air mass).
    pub fn turn_radius(&self, ac: &Aircraft) -> f64 {
        let v = self.turn_speed(ac, self.spec.turn_bank, 0.0);
        v * v / (G * self.spec.turn_bank.tan())
    }
    /// Height per metre of straight clean glide, and per metre of arc in a planned turn.
    fn height_rates(&self, ac: &Aircraft) -> (f64, f64) {
        let b = self.spec.turn_bank;
        let n = 1.0 / b.cos();
        let vt = self.turn_speed(ac, b, 0.0);
        (1.0 / ac.glide_ratio(self.spec.vbg, 0.0), n / ac.glide_ratio(vt / n.sqrt(), 0.0))
    }
    fn path_height(&self, p: &DubinsPath, rates: (f64, f64)) -> f64 {
        let arcs = [p.a0, p.a1].iter().filter(|a| **a > 3.0 * D2R).count() as f64;
        p.straight * rates.0 + p.r * p.turn() * rates.1 + ROLL_LOSS * arcs
    }

    /// Air heading that holds the runway's ground track at best glide (into the crosswind).
    fn final_air_heading(&self, wind: Vec2, end: &RunwayEnd) -> f64 {
        end.heading - clamp(across(wind, end.heading) / self.spec.vbg, -0.6, 0.6).asin()
    }

    /// The Dubins path in the air mass from the aircraft to the final fix `fix_d` before the threshold,
    /// the time it takes, and the straight roll-in flown before it starts (m). The fix moves upwind in the
    /// air-mass frame by wind × time (3 iterations).
    pub fn path_to_fix(&self, ac: &Aircraft, wind: Vec2, end: &RunwayEnd, fix_d: f64) -> (DubinsPath, f64, f64) {
        let r = self.turn_radius(ac);
        let rates = self.height_rates(ac);
        let vt = self.turn_speed(ac, self.spec.turn_bank, 0.0);
        // the arc cannot start until the bank is on: begin the path where the remaining roll-in ends
        // (half the roll time at the current heading)
        let roll_left = (1.0 - ac.phi.abs() / self.spec.turn_bank).clamp(0.0, 1.0);
        let adv = vt * (self.spec.turn_bank / ROLL_RATE) * 0.5 * roll_left;
        let start = Pose { x: ac.x + adv * ac.psi.cos(), y: ac.y + adv * ac.psi.sin(), th: ac.psi };
        let [fx, fy] = self.fix_at(fix_d, end);
        let th = self.final_air_heading(wind, end);
        let mut t = 0.0;
        let mut path = dubins::shortest(start, Pose { x: fx, y: fy, th }, r);
        for _ in 0..3 {
            let fix = Pose { x: fx - wind.x * t, y: fy - wind.y * t, th };
            path = dubins::shortest_by(start, fix, r, |p| self.path_height(p, rates));
            t = (adv + path.straight) / self.spec.vbg + path.r * path.turn() / vt;
        }
        (path, t, adv)
    }

    /// Assess one option: final fix `fix_d` before the threshold, aim point `aim` past it.
    pub fn assess(&self, ac: &Aircraft, wind: Vec2, end: &RunwayEnd, fix_d: f64, aim: f64) -> GlideAssessment {
        let rates = self.height_rates(ac);
        let (path, t, adv) = self.path_to_fix(ac, wind, end, fix_d);
        // final: straight down the centreline, crabbed, to the aim point
        let v = self.spec.vbg;
        let retract = if ac.flap_pos > 15.0 { RETRACT_LOSS } else { 0.0 };
        let required = adv * rates.0 + self.path_height(&path, rates) + self.glide_final(ac, wind, end, fix_d, aim) * self.final_height_per_m(ac, wind, end) + retract;
        // speed is energy: above best glide it can be traded for height, below it costs height
        let available = ac.h + (ac.v * ac.v - v * v) / (2.0 * G);
        let surplus = available - (required * self.spec.margin_factor + self.spec.margin_height);
        // route over the ground: each air-mass point drifts with the wind for the time taken to reach it
        let vt = self.turn_speed(ac, self.spec.turn_bank, 0.0);
        let mut route: Vec<Point> = vec![[ac.x, ac.y]];
        let (l0, l1) = (path.r * path.a0, path.r * path.a0 + path.straight);
        let n = (path.length() / 40.0).ceil().max(1.0) as usize;
        for i in 0..=n {
            let s = path.length() * i as f64 / n as f64;
            let tau = if s <= l0 {
                s / vt
            } else if s <= l1 {
                l0 / vt + (s - l0) / v
            } else {
                l0 / vt + path.straight / v + (s - l1) / vt
            };
            let q = path.at(s);
            let tau = tau + adv / v;
            route.push([q.x + wind.x * tau, q.y + wind.y * tau]);
        }
        debug_assert!((tau_total(&path, vt, v) + adv / v - t).abs() < 1e-6);
        route.push(self.fix_at(fix_d, end));
        route.push(self.aim_at(aim, end));
        GlideAssessment { reachable: surplus >= 0.0, required, available, route, ip_distance: fix_d, aim, end: *end, surplus, headroom: surplus }
    }

    /// The plan being flown, assessed along what is left of it: `arc_left` m of turn and `straight_left`
    /// m of straight still to the join point, then the final to the aim point. Consistent with the path
    /// follower (a fresh path from the current pose would start with a new roll-in).
    #[allow(clippy::too_many_arguments)]
    pub fn assess_along(&self, ac: &Aircraft, wind: Vec2, end: &RunwayEnd, fix_d: f64, aim: f64, arc_left: f64, straight_left: f64) -> GlideAssessment {
        let rates = self.height_rates(ac);
        let v = self.spec.vbg;
        let retract = if ac.flap_pos > 15.0 { RETRACT_LOSS } else { 0.0 };
        let rolls = if arc_left > 1.0 { ROLL_LOSS } else { 0.0 };
        let required = straight_left * rates.0 + arc_left * rates.1 + rolls + self.glide_final(ac, wind, end, fix_d, aim) * self.final_height_per_m(ac, wind, end) + retract;
        let available = ac.h + (ac.v * ac.v - v * v) / (2.0 * G);
        let surplus = available - (required * self.spec.margin_factor + self.spec.margin_height);
        let route = vec![[ac.x, ac.y], self.fix_at(fix_d, end), self.aim_at(aim, end)];
        GlideAssessment { reachable: surplus >= 0.0, required, available, route, ip_distance: fix_d, aim, end: *end, surplus, headroom: surplus }
    }

    /// Best option for one runway direction. The path may join the centreline anywhere from `fix_max`
    /// before the threshold to over the runway itself (any part of it will do), with at least
    /// `MIN_FINAL` of straight final before the touchdown, which may be anywhere from `aim_min` past the
    /// threshold to the farthest point that still leaves room to stop. Of the options reachable with the
    /// planning margins, the most height in hand (relative to the path) wins up to `reserve`; past that, the
    /// predicted touchdown (aim point, plus whatever height full flap cannot burn on the final, flown off
    /// long) nearest the nominal aim point, then the longest final. With none reachable, the closest miss.
    pub fn plan_for(&self, ac: &Aircraft, wind: Vec2, end: &RunwayEnd) -> GlideAssessment {
        let far = self.far_aim(wind, end);
        let nominal = self.spec.aim_point.min(far);
        let headroom = self.assess(ac, wind, end, self.spec.fix_min, nominal).surplus;
        let finish = |mut a: GlideAssessment| {
            a.headroom = headroom;
            if far < self.spec.aim_min {
                a.reachable = false; // tailwind too strong to stop on this runway
            }
            a
        };
        // joins from far out to over the runway; aims on a grid, plus the nominal one
        let mut aims: Vec<f64> = Vec::new();
        let mut x = self.spec.aim_min;
        while x <= far {
            aims.push(x);
            x += AIM_STEP;
        }
        aims.push(nominal);
        // ranking, lowest first: height in hand up to the reserve; then the touchdown nearest the nominal
        // aim point, predicted with the drag the final can add: height beyond what full flap burns over
        // the final is flown off long (so a high arrival prefers the longer final); then the longest final
        let reserve = self.spec.reserve;
        let (g_clean, g_full) = (self.gradient(ac, wind, 0.0, 1.0, end), self.gradient(ac, wind, 30.0, 1.0, end));
        let predicted = |a: &GlideAssessment| {
            let fin = self.glide_final(ac, wind, end, a.ip_distance, a.aim);
            let burnable = fin * (g_full - g_clean).max(0.0);
            let excess = (a.available - a.required - burnable).max(0.0);
            a.aim + excess / g_full
        };
        let rank = |a: &GlideAssessment| ((-(a.surplus / a.required.max(1.0)).min(reserve)), (predicted(a) - nominal).abs(), -(a.ip_distance + a.aim));
        let better = |a: (f64, f64, f64), b: (f64, f64, f64)| {
            a.0 < b.0 - 1e-6 || (a.0 <= b.0 + 1e-6 && (a.1 < b.1 - 1e-6 || (a.1 <= b.1 + 1e-6 && a.2 < b.2)))
        };
        let mut best: Option<GlideAssessment> = None;
        let mut miss: Option<GlideAssessment> = None;
        let mut d = self.spec.fix_max;
        while d >= -(far - MIN_FINAL) {
            // the path's height does not depend on the aim, only the final does: assess once per join
            let base = self.assess(ac, wind, end, d, (-d + MIN_FINAL).max(self.spec.aim_min));
            for &aim in &aims {
                if aim < -d + MIN_FINAL - 1e-6 {
                    continue;
                }
                let a = self.with_aim(&base, ac, wind, end, aim);
                if a.reachable {
                    if best.as_ref().map_or(true, |b| better(rank(&a), rank(b))) {
                        best = Some(a);
                    }
                } else if miss.as_ref().map_or(true, |m| a.surplus > m.surplus) {
                    miss = Some(a);
                }
            }
            d -= FIX_STEP;
        }
        finish(best.or(miss).expect("at least one option"))
    }

    /// The same path with a different aim point: only the final's length changes.
    fn with_aim(&self, a: &GlideAssessment, ac: &Aircraft, wind: Vec2, end: &RunwayEnd, aim: f64) -> GlideAssessment {
        let per_m = self.final_height_per_m(ac, wind, end);
        let d0 = self.glide_final(ac, wind, end, a.ip_distance, a.aim);
        let d1 = self.glide_final(ac, wind, end, a.ip_distance, aim);
        let required = a.required + (d1 - d0) * per_m;
        let surplus = a.available - (required * self.spec.margin_factor + self.spec.margin_height);
        let mut route = a.route.clone();
        if let Some(last) = route.last_mut() {
            *last = self.aim_at(aim, end);
        }
        GlideAssessment { reachable: surplus >= 0.0, required, surplus, aim, route, ..a.clone() }
    }

    /// Height per metre of final (crabbed, straight, clean at best glide), m/m.
    fn final_height_per_m(&self, ac: &Aircraft, wind: Vec2, end: &RunwayEnd) -> f64 {
        let v = self.spec.vbg;
        let crab = clamp(across(wind, end.heading) / v, -0.6, 0.6).asin();
        let gs = (v * crab.cos() + along(wind, end.heading)).max(0.3 * v);
        (v / gs) / ac.glide_ratio(v, 0.0)
    }

    /// Plan both directions: keep the preferred end while it still works, else the reachable end with
    /// the most height in hand, else the one that comes closest.
    pub fn plan(&self, ac: &Aircraft, wind: Vec2, prefer: Option<&RunwayEnd>) -> GlideAssessment {
        let mut options: Vec<GlideAssessment> = self.ends.iter().map(|e| self.plan_for(ac, wind, e)).collect();
        if let Some(p) = prefer {
            if let Some(k) = options.iter().find(|o| o.end.name == p.name && o.reachable) {
                return k.clone();
            }
        }
        options.sort_by(|a, b| {
            (b.reachable as i32).cmp(&(a.reachable as i32)).then(b.headroom.partial_cmp(&a.headroom).unwrap_or(std::cmp::Ordering::Equal))
        });
        options.swap_remove(0)
    }

    /// Glide footprint: where a straight power-off glide from here could end, in any direction — a
    /// circle of radius (height × glide ratio) whose centre drifts with the wind for the time the glide
    /// takes. Uses the flap already out (above 15°), and ignores turn losses.
    pub fn footprint(&self, ac: &Aircraft, wind: Vec2) -> (Point, f64) {
        let flap = if ac.flap_pos > 15.0 { ac.flap_pos } else { 0.0 };
        let v = if flap >= 20.0 { self.spec.vapp_eo } else { self.spec.vbg };
        let ld = ac.glide_ratio(v, flap);
        let h = ac.h.max(0.0);
        let t = h * ld / v;
        ([ac.x + wind.x * t, ac.y + wind.y * t], h * ld)
    }

    /// Still-air reach in metres from the current height (clean, best glide).
    pub fn still_air_reach(&self, ac: &Aircraft) -> f64 {
        ac.h * ac.glide_ratio(self.spec.vbg, 0.0)
    }

    /// Ground glide gradient (height lost per metre over the ground) along the landing direction for a
    /// flap setting, optionally in a forward slip (`slip_gain` × the sink rate).
    pub fn gradient(&self, ac: &Aircraft, wind: Vec2, flap_pos: f64, slip_gain: f64, end: &RunwayEnd) -> f64 {
        let v = if flap_pos >= 20.0 { self.spec.vapp_eo } else { self.spec.vbg };
        slip_gain / (ac.glide_ratio(v, flap_pos) * ((v + along(wind, end.heading)) / v).max(0.3))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::aircraft::{c172_like, ModelKind};

    fn planner(len: f64) -> GlidePlanner {
        GlidePlanner::new(Runway::new(len, 30.0, 300.0, 3.0 * D2R), EngineOutSpec::new(&Tuning::default(), len))
    }
    fn airborne(x: f64, y: f64, h: f64, psi: f64, p: &GlidePlanner) -> Aircraft {
        let mut ac = Aircraft::new(c172_like(), ModelKind::SixDof);
        ac.on_ground = false;
        ac.flap_pos = 0.0;
        (ac.x, ac.y, ac.h, ac.psi, ac.v) = (x, y, h, psi, p.spec.vbg);
        ac
    }

    #[test]
    fn lined_up_on_final_matches_the_glide_ratio() {
        // straight in from 2 km on the centreline: the required height is the distance over L/D
        let p = planner(1200.0);
        let ac = airborne(-2000.0, 0.0, 300.0, 0.0, &p);
        let a = p.assess(&ac, Vec2::default(), &p.ends[0], 150.0, 300.0);
        let ld = ac.glide_ratio(p.spec.vbg, 0.0);
        // the glide meets the ground a hold-off float before the touchdown point
        let fl = p.float_distance(&ac, p.spec.vapp_eo, 30.0, Vec2::default(), &p.ends[0]);
        assert!(fl > 50.0 && fl < 400.0, "float {fl}");
        assert!((a.required - (2300.0 - fl) / ld).abs() < 1.0, "{} vs {}", a.required, (2300.0 - fl) / ld);
    }

    #[test]
    fn a_turn_back_costs_more_than_flying_straight() {
        let p = planner(1200.0);
        let ahead = airborne(-2000.0, 0.0, 300.0, 0.0, &p);
        let away = airborne(-2000.0, 0.0, 300.0, PI, &p);
        let a = p.assess(&ahead, Vec2::default(), &p.ends[0], 150.0, 300.0);
        let b = p.assess(&away, Vec2::default(), &p.ends[0], 150.0, 300.0);
        assert!(b.required > a.required + 30.0, "{} vs {}", b.required, a.required);
    }

    #[test]
    fn reachability_switches_at_the_glide_limit() {
        let p = planner(1200.0);
        let probe = airborne(0.0, 0.0, 0.0, 0.0, &p);
        let ld = probe.glide_ratio(p.spec.vbg, 0.0);
        let fl = p.float_distance(&probe, p.spec.vapp_eo, 30.0, Vec2::default(), &p.ends[0]);
        // 2.85 km out on the centreline, no turn: the nearest acceptable aim point (60 m in) is reachable
        // with just enough height (margins included) and not with a little less
        let need = (2850.0 + p.spec.aim_min - fl) / ld * p.spec.margin_factor + p.spec.margin_height;
        let ok = p.plan_for(&airborne(-2850.0, 0.0, need + 0.5, 0.0, &p), Vec2::default(), &p.ends[0]);
        let short = p.plan_for(&airborne(-2850.0, 0.0, need - 1.0, 0.0, &p), Vec2::default(), &p.ends[0]);
        assert!((ok.aim - p.spec.aim_min).abs() < 1.0, "aim {}", ok.aim);
        assert!(ok.reachable && !short.reachable, "{} {}", ok.surplus, short.surplus);
    }

    #[test]
    fn a_headwind_on_final_needs_more_height() {
        let p = planner(1200.0);
        let ac = airborne(-2000.0, 0.0, 300.0, 0.0, &p);
        let calm = p.assess(&ac, Vec2::default(), &p.ends[0], 150.0, 300.0);
        let head = p.assess(&ac, Vec2 { x: -5.0, y: 0.0 }, &p.ends[0], 150.0, 300.0);
        assert!(head.required > calm.required * 1.1);
    }
}
