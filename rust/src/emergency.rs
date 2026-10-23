//! Power-off energy planning: can the runway (either direction) be reached in a glide, and by which route?

use crate::aircraft::Aircraft;
use crate::airfield::Runway;
use crate::atmosphere::Wind;
use crate::units::{wrap_pi, Point, D2R, FT, G, KT, PI};
use serde::Serialize;

/// Runway kept beyond the farthest aim point, for the flare's float and the ground roll, m.
const FAR_AIM_STOP: f64 = 450.0;
/// Stretch to the runway (last resort): intercept distance and touchdown aim past the threshold, m.
/// Height lost raising the flap from 20–30° to up at the power-off approach speed, m.
const RETRACT_LOSS: f64 = 8.0;
const STRETCH_IP: f64 = 150.0;
const STRETCH_AIM: f64 = 60.0;
const REVERSAL_COST_FROM: f64 = 2.1;
const REVERSAL_COST: f64 = 0.7;
/// Extra path, in turn radii, to realign after a reversal at a waypoint.
const REALIGN: f64 = 1.0;

#[derive(Clone, Debug, Serialize)]
pub struct EngineOutSpec {
    /// Best-glide speed, clean.
    pub vbg: f64,
    /// Power-off approach speed once flaps are out.
    pub vapp_eo: f64,
    /// Below this height after take-off, never turn back: land ahead.
    pub min_turnback_height: f64,
    /// Range of intercept points on the extended centreline, metres before the threshold.
    pub ip_min: f64,
    pub ip_max: f64,
    /// Lateral offset of the base key point from the centreline.
    pub base_key_offset: f64,
    /// Power-off aim point, metres past the threshold.
    pub aim_point: f64,
    /// Required height is multiplied by this, plus `margin_height`, before calling the runway reachable.
    pub margin_factor: f64,
    pub margin_height: f64,
    /// Base key beyond the turn-onto-final reach, m.
    pub key_margin: f64,
    /// Height kept in hand for a margin-free stretch to the runway, m.
    pub stretch_margin: f64,
    /// Pitch slew while gliding.
    pub pitch_rate: f64,
}

pub fn default_eo() -> EngineOutSpec {
    EngineOutSpec {
        vbg: 70.0 * KT,
        vapp_eo: 66.0 * KT,
        min_turnback_height: 700.0 * FT,
        ip_min: 500.0,
        ip_max: 3000.0,
        aim_point: 300.0,
        base_key_offset: 450.0,
        margin_factor: 1.1,
        margin_height: 20.0,
        key_margin: 150.0,
        stretch_margin: 10.0,
        pitch_rate: 5.0 * D2R,
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
    /// Height the route needs, before margins, m.
    pub required: f64,
    /// Height available above the runway, m.
    pub available: f64,
    pub route: Vec<Point>,
    pub ip_distance: f64,
    pub end: RunwayEnd,
    /// Height left over after margins (negative = short).
    pub surplus: f64,
}

pub struct Waypoints {
    pub base_key: Option<Point>,
    pub ip: Point,
}

#[derive(Clone, Debug)]
pub struct GlidePlanner {
    pub runway: Runway,
    pub spec: EngineOutSpec,
    pub ends: [RunwayEnd; 2],
}

impl GlidePlanner {
    pub fn new(runway: Runway, spec: EngineOutSpec) -> Self {
        let ends = runway_ends(&runway);
        GlidePlanner { runway, spec, ends }
    }

    pub fn ip_at(&self, distance: f64, end: &RunwayEnd) -> Point {
        [end.threshold - end.dir * distance, 0.0]
    }
    pub fn aim_for(&self, end: &RunwayEnd) -> Point {
        [end.threshold + end.dir * self.spec.aim_point, 0.0]
    }
    /// Along-approach distance still to go to the aim point (positive before it).
    pub fn to_aim(&self, x: f64, end: &RunwayEnd) -> f64 {
        end.dir * (self.aim_for(end)[0] - x)
    }
    /// Farthest useful aim point when high (any part of the runway will do): far enough from the far end
    /// to flare and stop.
    pub fn far_aim_distance(&self) -> f64 {
        (self.runway.length - FAR_AIM_STOP).max(self.spec.aim_point)
    }

    /// Ground distance a 90° turn (25° bank, roll-in included) carries the aircraft along its initial
    /// direction, in the air mass plus the wind's drift over the turn. A tailwind on base stretches it.
    pub fn turn_reach(&self, v_air: f64, wind_along: f64) -> f64 {
        let bank = 25.0 * D2R;
        let r = v_air * v_air / (G * bank.tan());
        let t_turn = (PI / 2.0) * r / v_air;
        let t_roll = bank / (8.0 * D2R);
        r + wind_along * t_turn + (v_air + wind_along) * t_roll * 0.5
    }

    /// Base key point abeam the intercept (on the aircraft's side), then the intercept.
    /// The base key is skipped when the aircraft is already out beyond the intercept point. It sits far
    /// enough out for the turn onto final to finish on the centreline, wind included (with a margin).
    pub fn waypoints(&self, ac: &Aircraft, wind: &Wind, ip_distance: f64, end: &RunwayEnd) -> Waypoints {
        let ip = self.ip_at(ip_distance, end);
        if end.dir * (ac.x - ip[0]) < 200.0 {
            return Waypoints { base_key: None, ip };
        }
        let side = if ac.y >= 0.0 { 1.0 } else { -1.0 };
        // base is flown toward the centreline (−side in y): the wind along it
        let w_base = -side * wind.v.y;
        let offset = self.spec.base_key_offset.max(self.turn_reach(self.spec.vbg, w_base) + self.spec.key_margin);
        Waypoints { base_key: Some([ip[0], side * offset]), ip }
    }

    /// Longest final the energy allows: scan intercepts from far to near (50 m steps), first reachable wins.
    pub fn plan_for(&self, ac: &Aircraft, wind: &Wind, end: &RunwayEnd) -> GlideAssessment {
        let mut last: Option<GlideAssessment> = None;
        let mut d = self.spec.ip_max;
        while d >= self.spec.ip_min {
            let a = self.assess(ac, wind, d, end);
            if a.reachable {
                return a;
            }
            last = Some(a);
            d -= 50.0;
        }
        last.unwrap()
    }

    /// Plan both directions: keep the preferred end while it still works, else the reachable end with
    /// the most height in hand, else the one that comes closest.
    pub fn plan(&self, ac: &Aircraft, wind: &Wind, prefer: Option<&RunwayEnd>) -> GlideAssessment {
        let mut options: Vec<GlideAssessment> = self.ends.iter().map(|e| self.plan_for(ac, wind, e)).collect();
        if let Some(p) = prefer {
            if let Some(k) = options.iter().find(|o| o.end.name == p.name && o.reachable) {
                return k.clone();
            }
        }
        // stable sort like Array.prototype.sort: reachable first, then most surplus
        options.sort_by(|a, b| {
            let r = (b.reachable as i32).cmp(&(a.reachable as i32));
            if r != std::cmp::Ordering::Equal {
                return r;
            }
            b.surplus.partial_cmp(&a.surplus).unwrap_or(std::cmp::Ordering::Equal)
        });
        options.swap_remove(0)
    }

    /// Glide footprint: where a straight power-off glide from here could end, in any direction — a
    /// circle of radius (height × glide ratio) whose centre drifts with the wind for the time the glide
    /// takes. Uses the flap already out (above 15°), and ignores turn losses.
    pub fn footprint(&self, ac: &Aircraft, wind: &Wind) -> (Point, f64) {
        let flap = if ac.flap_pos > 15.0 { ac.flap_pos } else { 0.0 };
        let v = if flap >= 20.0 { self.spec.vapp_eo } else { self.spec.vbg };
        let ld = ac.glide_ratio(v, flap);
        let h = ac.h.max(0.0);
        let t = h * ld / v;
        ([ac.x + wind.v.x * t, ac.y + wind.v.y * t], h * ld)
    }

    /// Still-air reach in metres from the current height (clean, best glide).
    pub fn still_air_reach(&self, ac: &Aircraft) -> f64 {
        ac.h * ac.glide_ratio(self.spec.vbg, 0.0)
    }

    pub fn assess(&self, ac: &Aircraft, wind: &Wind, ip_distance: f64, end: &RunwayEnd) -> GlideAssessment {
        self.assess_to(ac, wind, ip_distance, end, self.spec.aim_point, true)
    }

    /// Last resort before a field: can the aircraft reach the runway at all — a short final to a
    /// touchdown just past the threshold, no planning margins? Any part of the runway beats a field.
    pub fn stretch(&self, ac: &Aircraft, wind: &Wind, prefer: Option<&RunwayEnd>) -> Option<GlideAssessment> {
        let mut best: Option<GlideAssessment> = None;
        for e in self.ends.iter() {
            let a = self.assess_to(ac, wind, STRETCH_IP, e, STRETCH_AIM, false);
            if !a.reachable {
                continue;
            }
            let better = match &best {
                None => true,
                Some(b) => prefer.map(|p| p.name == e.name).unwrap_or(false) || a.surplus > b.surplus,
            };
            if better {
                best = Some(a);
            }
        }
        best
    }

    fn assess_to(&self, ac: &Aircraft, wind: &Wind, ip_distance: f64, end: &RunwayEnd, aim_dist: f64, margins: bool) -> GlideAssessment {
        // flap already out (e.g. a failure on base or final): plan with the glide it gives, and with the
        // flatter clean glide after raising it (the retraction itself costs RETRACT_LOSS of height);
        // the better of the two is the plan, and the autopilot raises the flap when short
        let flap = if ac.flap_pos > 15.0 { ac.flap_pos } else { 0.0 };
        let out = self.assess_cfg(ac, wind, ip_distance, end, aim_dist, margins, flap, 0.0);
        if flap == 0.0 || out.reachable {
            return out;
        }
        let clean = self.assess_cfg(ac, wind, ip_distance, end, aim_dist, margins, 0.0, RETRACT_LOSS);
        if clean.surplus > out.surplus { clean } else { out }
    }

    #[allow(clippy::too_many_arguments)]
    fn assess_cfg(&self, ac: &Aircraft, wind: &Wind, ip_distance: f64, end: &RunwayEnd, aim_dist: f64, margins: bool, flap: f64, extra: f64) -> GlideAssessment {
        let v = if flap >= 20.0 { self.spec.vapp_eo } else { self.spec.vbg };
        let ld = ac.glide_ratio(v, flap);
        // planning judges reachability to the nominal aim point with margins (a touchdown just past the
        // threshold left no margin for the flare and failed more often); `stretch` is the margin-free
        // last resort
        let aim = [end.threshold + end.dir * aim_dist, 0.0];
        let wp = self.waypoints(ac, wind, ip_distance, end);
        let gv0 = ac.ground_velocity(wind);
        // already established inbound inside the intercept: straight in to the aim point
        let straight_in = ac.y.abs() < 150.0
            && end.dir * (ac.x - wp.ip[0]) > 0.0
            && self.to_aim(ac.x, end) > 0.0
            && wrap_pi(gv0.y.atan2(gv0.x) - end.heading).abs() < 45.0 * D2R;
        let route: Vec<Point> = match (straight_in, wp.base_key) {
            (true, _) => vec![[ac.x, ac.y], aim],
            (false, Some(k)) => vec![[ac.x, ac.y], k, wp.ip, aim],
            (false, None) => vec![[ac.x, ac.y], wp.ip, aim],
        };
        // Turns: the first (from the current track) adds ~0.3·R per radian; at each waypoint the turn cuts
        // the corner by R·(2·tan(Δ/2) − Δ) and the arc is flown ~10% less efficiently.
        let r = (v * v) / (G * (25.0 * D2R).tan());
        let gv = ac.ground_velocity(wind);
        let mut track = gv.y.atan2(gv.x);
        let mut required = 0.0;
        for i in 1..route.len() {
            let [ax, ay] = route[i - 1];
            let [bx, by] = route[i];
            let len = (bx - ax).hypot(by - ay);
            if len < 1.0 {
                continue;
            }
            let brg = (by - ay).atan2(bx - ax);
            let turn = wrap_pi(brg - track).abs();
            let along = wind.along(brg);
            let ground_ld = ld * ((v + along) / v).max(0.3);
            // a reversal is mostly flown as arc, not a corner: count most of it
            let first = if turn > REVERSAL_COST_FROM { r * turn * REVERSAL_COST } else { r * turn * 0.3 };
            // at a waypoint a turn of up to 90° cuts the corner (fly-by); anything more cannot: a reversal
            // is flown as arc and then a realignment of up to a turn diameter (the old formula credited a
            // near-180° turn with a huge shortcut, so a turn-back after take-off looked easy)
            let at_wp = if turn <= PI / 2.0 {
                r * turn * 1.1 - 2.0 * r * (turn / 2.0).tan()
            } else {
                // continuous with the fly-by value at 90°: the extra turn is flown as arc, plus realignment
                let at90 = r * (PI / 2.0) * 1.1 - 2.0 * r;
                at90 + r * (turn - PI / 2.0) * 1.1 + REALIGN * r * (turn - PI / 2.0) / (PI / 2.0)
            };
            let turn_adj = if i == 1 { first } else { at_wp };
            required += (len + turn_adj).max(0.0) / ground_ld;
            track = brg;
        }
        // speed above best glide is energy too: it can be traded for height
        let required = required + extra;
        let available = ac.h + (ac.v * ac.v - v * v).max(0.0) / (2.0 * G);
        let surplus = if margins {
            available - (required * self.spec.margin_factor + self.spec.margin_height)
        } else {
            available - (required + self.spec.stretch_margin)
        };
        GlideAssessment { reachable: surplus >= 0.0, required, available, route, ip_distance, end: *end, surplus }
    }

    /// Ground glide gradient (height lost per metre over the ground) for a flap setting, along the landing direction.
    pub fn gradient(&self, ac: &Aircraft, wind: &Wind, flap_pos: f64, end: &RunwayEnd) -> f64 {
        let v = if flap_pos >= 20.0 { self.spec.vapp_eo } else { self.spec.vbg };
        1.0 / (ac.glide_ratio(v, flap_pos) * ((v + wind.along(end.heading)) / v).max(0.3))
    }
}
