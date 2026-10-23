//! Power-off energy planning: can the runway (either direction) be reached in a glide, and by which route?

use crate::aircraft::Aircraft;
use crate::airfield::Runway;
use crate::atmosphere::Wind;
use crate::units::{wrap_pi, Point, D2R, FT, G, KT, PI};
use serde::Serialize;

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
    /// Pitch slew while gliding.
    pub pitch_rate: f64,
}

pub fn default_eo() -> EngineOutSpec {
    EngineOutSpec {
        vbg: 68.0 * KT,
        vapp_eo: 63.0 * KT,
        min_turnback_height: 700.0 * FT,
        ip_min: 500.0,
        ip_max: 3000.0,
        aim_point: 300.0,
        base_key_offset: 450.0,
        margin_factor: 1.1,
        margin_height: 20.0,
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

    /// Base key point abeam the intercept (on the aircraft's side), then the intercept.
    /// The base key is skipped when the aircraft is already out beyond the intercept point.
    pub fn waypoints(&self, ac: &Aircraft, ip_distance: f64, end: &RunwayEnd) -> Waypoints {
        let ip = self.ip_at(ip_distance, end);
        if end.dir * (ac.x - ip[0]) < 200.0 {
            return Waypoints { base_key: None, ip };
        }
        let side = if ac.y >= 0.0 { 1.0 } else { -1.0 };
        Waypoints { base_key: Some([ip[0], side * self.spec.base_key_offset]), ip }
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

    /// Still-air reach in metres from the current height (clean, best glide).
    pub fn still_air_reach(&self, ac: &Aircraft) -> f64 {
        ac.h * ac.glide_ratio(self.spec.vbg, 0.0)
    }

    pub fn assess(&self, ac: &Aircraft, wind: &Wind, ip_distance: f64, end: &RunwayEnd) -> GlideAssessment {
        // flap already out (e.g. a failure on final) stays out: plan with the glide that configuration gives
        let flap = if ac.flap_pos > 15.0 { ac.flap_pos } else { 0.0 };
        let v = if flap >= 20.0 { self.spec.vapp_eo } else { self.spec.vbg };
        let ld = ac.glide_ratio(v, flap);
        let aim = self.aim_for(end);
        let wp = self.waypoints(ac, ip_distance, end);
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
            let t = turn.min(2.5);
            let turn_adj = if i == 1 { r * turn * 0.3 } else { r * t * 1.1 - 2.0 * r * (t / 2.0).tan() };
            required += (len + turn_adj).max(0.0) / ground_ld;
            track = brg;
        }
        // speed above best glide is energy too: it can be traded for height
        let available = ac.h + (ac.v * ac.v - v * v).max(0.0) / (2.0 * G);
        let surplus = available - (required * self.spec.margin_factor + self.spec.margin_height);
        GlideAssessment { reachable: surplus >= 0.0, required, available, route, ip_distance, end: *end, surplus }
    }

    /// Ground glide gradient (height lost per metre over the ground) for a flap setting, along the landing direction.
    pub fn gradient(&self, ac: &Aircraft, wind: &Wind, flap_pos: f64, end: &RunwayEnd) -> f64 {
        let v = if flap_pos >= 20.0 { self.spec.vapp_eo } else { self.spec.vbg };
        1.0 / (ac.glide_ratio(v, flap_pos) * ((v + wind.along(end.heading)) / v).max(0.3))
    }
}
