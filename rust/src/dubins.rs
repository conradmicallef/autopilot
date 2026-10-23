//! Dubins paths: the shortest way between two poses (position and heading) for a vehicle with a minimum
//! turn radius, as turn–straight–turn (the CSC family: LSL, RSR, LSR, RSL).
//!
//! Angles use the airfield's math convention: heading 0 = +x (east), counter-clockwise positive. A turn
//! direction `s` is +1 for left (counter-clockwise) and −1 for right. The turn–turn–turn family (CCC) is
//! left out: it is only shorter when the two poses are within about two radii of each other, which a
//! glide plan to a final fix hundreds of metres away does not need.

use crate::units::{wrap_pi, PI};

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Pose {
    pub x: f64,
    pub y: f64,
    /// Heading, rad.
    pub th: f64,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DubinsPath {
    pub start: Pose,
    /// Turn radius, m.
    pub r: f64,
    /// Turn direction of the first and the second arc: +1 left, −1 right.
    pub s0: f64,
    pub s1: f64,
    /// Arc angles, rad, in [0, 2π), and the straight between them, m.
    pub a0: f64,
    pub straight: f64,
    pub a1: f64,
}

/// Angle in [0, 2π).
fn mod2pi(a: f64) -> f64 {
    let t = a.rem_euclid(2.0 * PI);
    if t >= 2.0 * PI - 1e-12 { 0.0 } else { t }
}

/// Pose after flying an arc of `angle` (≥ 0) in direction `s` with radius `r`.
pub fn arc(p: Pose, s: f64, angle: f64, r: f64) -> Pose {
    // centre of the turn: one radius to the left (s = +1) or right of the heading
    let (cx, cy) = (p.x - s * r * p.th.sin(), p.y + s * r * p.th.cos());
    let th = p.th + s * angle;
    Pose { x: cx + s * r * th.sin(), y: cy - s * r * th.cos(), th: wrap_pi(th) }
}

impl DubinsPath {
    pub fn length(&self) -> f64 {
        self.r * (self.a0 + self.a1) + self.straight
    }
    /// Total heading change, rad.
    pub fn turn(&self) -> f64 {
        self.a0 + self.a1
    }
    /// Pose at the start of the straight, and at its end (the start of the second arc).
    pub fn tangent_points(&self) -> (Pose, Pose) {
        let t0 = arc(self.start, self.s0, self.a0, self.r);
        let t1 = Pose { x: t0.x + self.straight * t0.th.cos(), y: t0.y + self.straight * t0.th.sin(), th: t0.th };
        (t0, t1)
    }
    /// Pose after travelling `s` metres along the path (clamped to its ends).
    pub fn at(&self, s: f64) -> Pose {
        let s = s.clamp(0.0, self.length());
        let l0 = self.r * self.a0;
        if s <= l0 {
            return arc(self.start, self.s0, s / self.r, self.r);
        }
        let (t0, t1) = self.tangent_points();
        if s <= l0 + self.straight {
            let d = s - l0;
            return Pose { x: t0.x + d * t0.th.cos(), y: t0.y + d * t0.th.sin(), th: t0.th };
        }
        arc(t1, self.s1, (s - l0 - self.straight) / self.r, self.r)
    }
    /// Points along the path every `step` metres (both ends included), for drawing.
    pub fn sample(&self, step: f64) -> Vec<Pose> {
        let n = (self.length() / step).ceil().max(1.0) as usize;
        (0..=n).map(|i| self.at(self.length() * i as f64 / n as f64)).collect()
    }
}

/// Centre of a turn of direction `s` and radius `r` that starts at pose `p`.
pub fn centre(p: Pose, s: f64, r: f64) -> (f64, f64) {
    (p.x - s * r * p.th.sin(), p.y + s * r * p.th.cos())
}

/// Following a Dubins path: which segment the aircraft is on, and the heading and bank to fly there.
///
/// One vector-field law for all three segments (Nelson et al., 2007). On an arc of centre c, radius r,
/// direction λ, the desired heading is the tangent heading turned toward the circle by
/// atan(2(d − r)/r) where d is the distance from c; on the straight it is the line's heading turned toward
/// the line by up to 60° (atan of the cross-track over a capture distance). Arcs are tracked by heading:
/// an arc ends when the heading still to turn is down to the roll-out lead (the heading the aircraft
/// keeps turning through while it rolls level), and the feed-forward bank tapers to zero over that lead.
/// The straight hands over to the second arc a roll-in distance before its end. So a slow roll only ever
/// leaves a correction to fly, never a different path.
#[derive(Clone, Copy, Debug)]
pub struct Follower {
    pub path: DubinsPath,
    /// 0 = first arc, 1 = straight, 2 = second arc, 3 = done.
    pub seg: u8,
}

pub struct Steer {
    /// Desired heading, rad.
    pub heading: f64,
    /// Feed-forward bank as a fraction of the path's bank, signed (+ left); 0 on the straight.
    pub ff: f64,
    /// Turn direction of the current arc (0 on the straight or when done).
    pub turn: f64,
    /// Heading still to turn on the current arc, rad (0 on the straight).
    pub to_go: f64,
    /// Distance off the current segment, m.
    pub offset: f64,
    /// Path still to fly: arc length (both arcs) and straight length, m.
    pub arc_left: f64,
    pub straight_left: f64,
}

/// Kinematics the follower needs: speed (m/s), current bank (rad, + left), roll rate (rad/s), path bank (rad).
#[derive(Clone, Copy, Debug)]
pub struct Motion {
    pub v: f64,
    pub phi: f64,
    pub roll: f64,
    pub bank: f64,
}

impl Motion {
    /// Heading turned through while rolling from bank `phi` to level: half the roll time at the mean turn rate.
    fn rollout_lead(&self) -> f64 {
        let p = self.phi.abs().max(5.0 * D2R_);
        0.5 * (9.80665 * p.tan() / self.v) * (p / self.roll)
    }
    /// Distance flown while rolling from level to the path's bank (half the roll time).
    pub fn rollin_distance(&self) -> f64 {
        self.v * (self.bank / self.roll) * 0.5
    }
}

impl Follower {
    pub fn new(path: DubinsPath) -> Self {
        let mut f = Follower { path, seg: 0 };
        if path.a0 < 1e-6 {
            f.seg = 1;
        }
        f
    }

    /// Steering from the aircraft's position `q` and heading `psi` (both in the path's frame).
    pub fn steer(&mut self, qx: f64, qy: f64, psi: f64, m: Motion) -> Steer {
        let p = self.path;
        let (t0, t1) = p.tangent_points();
        let lead = m.rollout_lead();
        // an arc that ends at heading `end`: heading still to turn (an overshoot of up to 60° counts as done)
        let arc = |c: (f64, f64), s: f64, end: f64| {
            let (dx, dy) = (qx - c.0, qy - c.1);
            let d = dx.hypot(dy).max(1.0);
            let gam = dy.atan2(dx);
            let heading = gam + s * (PI / 2.0 + (2.0 * (d - p.r) / p.r).atan());
            let mut to_go = mod2pi(s * (end - psi));
            if to_go > 2.0 * PI - 60.0 * D2R_ {
                to_go = 0.0;
            }
            (heading, to_go, (d - p.r).abs())
        };
        loop {
            match self.seg {
                0 | 2 => {
                    let (c, s, end) = if self.seg == 0 {
                        (centre(p.start, p.s0, p.r), p.s0, t0.th)
                    } else {
                        (centre(t1, p.s1, p.r), p.s1, p.at(p.length()).th)
                    };
                    let (heading, to_go, off) = arc(c, s, end);
                    if to_go <= lead {
                        self.seg += 1;
                        continue;
                    }
                    let ff = s * (to_go / lead.max(1e-3)).min(1.0);
                    let (arc_left, straight_left) =
                        if self.seg == 0 { (p.r * (to_go + p.a1), p.straight) } else { (p.r * to_go, 0.0) };
                    return Steer { heading, ff, turn: s, to_go, offset: off, arc_left, straight_left };
                }
                1 => {
                    let (ux, uy) = (t0.th.cos(), t0.th.sin());
                    let along = (qx - t0.x) * ux + (qy - t0.y) * uy;
                    let rem = p.straight - along;
                    if p.a1 > 1e-6 && rem < m.rollin_distance() {
                        self.seg = 2;
                        continue;
                    }
                    if p.a1 <= 1e-6 && rem < 0.0 {
                        self.seg = 3;
                        continue;
                    }
                    let e = ux * (qy - t0.y) - uy * (qx - t0.x);
                    let heading = t0.th - (PI / 3.0) * (2.0 / PI) * (e / (0.8 * p.r)).atan();
                    return Steer { heading, ff: 0.0, turn: 0.0, to_go: 0.0, offset: e.abs(), arc_left: p.r * p.a1, straight_left: rem.max(0.0) };
                }
                _ => {
                    let end = p.at(p.length());
                    return Steer { heading: end.th, ff: 0.0, turn: 0.0, to_go: 0.0, offset: 0.0, arc_left: 0.0, straight_left: 0.0 };
                }
            }
        }
    }
}

const D2R_: f64 = PI / 180.0;

/// Bank command for following: the feed-forward share of the path's bank plus a correction from the
/// heading error, sized to take it out in `tau` seconds; limited to the path's bank + 5° (more bank on
/// the way in only lengthens the roll-out).
/// Bank is + for a left (counter-clockwise) turn.
pub fn bank_command(st: &Steer, m: Motion, psi: f64, tau: f64) -> f64 {
    let corr = ((m.v * wrap_pi(st.heading - psi)) / (9.80665 * tau)).atan();
    let lim = m.bank + 5.0 * D2R_;
    (st.ff * m.bank + corr).clamp(-lim, lim)
}

/// One CSC candidate, or None where the geometry has no such path (an inner tangent between circles that overlap).
fn candidate(p0: Pose, p1: Pose, r: f64, s0: f64, s1: f64) -> Option<DubinsPath> {
    let c0 = (p0.x - s0 * r * p0.th.sin(), p0.y + s0 * r * p0.th.cos());
    let c1 = (p1.x - s1 * r * p1.th.sin(), p1.y + s1 * r * p1.th.cos());
    let (dx, dy) = (c1.0 - c0.0, c1.1 - c0.1);
    let d = dx.hypot(dy);
    let psi = dy.atan2(dx);
    // heading of the straight, and its length
    let (phi, straight) = if s0 == s1 {
        (psi, d)
    } else {
        if d < 2.0 * r {
            return None;
        }
        (psi + s0 * (2.0 * r / d).asin(), (d * d - 4.0 * r * r).sqrt())
    };
    Some(DubinsPath {
        start: p0,
        r,
        s0,
        s1,
        a0: mod2pi(s0 * (phi - p0.th)),
        straight,
        a1: mod2pi(s1 * (p1.th - phi)),
    })
}

/// Shortest turn–straight–turn path from `p0` to `p1` with turn radius `r`, or the shortest of those
/// that a `cost` function prefers (lower is better) when given.
pub fn shortest_by(p0: Pose, p1: Pose, r: f64, cost: impl Fn(&DubinsPath) -> f64) -> DubinsPath {
    [(1.0, 1.0), (-1.0, -1.0), (1.0, -1.0), (-1.0, 1.0)]
        .iter()
        .filter_map(|&(s0, s1)| candidate(p0, p1, r, s0, s1))
        .min_by(|a, b| cost(a).partial_cmp(&cost(b)).unwrap_or(std::cmp::Ordering::Equal))
        .expect("LSL and RSR always exist")
}

pub fn shortest(p0: Pose, p1: Pose, r: f64) -> DubinsPath {
    shortest_by(p0, p1, r, |p| p.length())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn close(a: Pose, b: Pose) -> bool {
        (a.x - b.x).hypot(a.y - b.y) < 1e-6 && wrap_pi(a.th - b.th).abs() < 1e-9
    }

    #[test]
    fn every_candidate_ends_on_the_target() {
        let r = 120.0;
        let starts = [Pose { x: 0.0, y: 0.0, th: 0.0 }, Pose { x: 300.0, y: -200.0, th: 2.0 }, Pose { x: -50.0, y: 900.0, th: -2.6 }];
        let ends = [Pose { x: 1500.0, y: 0.0, th: PI }, Pose { x: -800.0, y: 400.0, th: 0.3 }, Pose { x: 0.0, y: -1000.0, th: -PI / 2.0 }];
        for &p0 in &starts {
            for &p1 in &ends {
                for (s0, s1) in [(1.0, 1.0), (-1.0, -1.0), (1.0, -1.0), (-1.0, 1.0)] {
                    if let Some(p) = candidate(p0, p1, r, s0, s1) {
                        assert!(close(p.at(p.length()), p1), "{s0} {s1}: {:?} vs {:?}", p.at(p.length()), p1);
                    }
                }
            }
        }
    }

    #[test]
    fn following_a_path_flown_ideally_ends_at_the_target() {
        // a point that moves along the path at the planned heading should step through every segment
        let (p0, p1) = (Pose { x: 0.0, y: 0.0, th: PI }, Pose { x: -400.0, y: -900.0, th: 0.0 });
        let path = shortest(p0, p1, 150.0);
        let mut f = Follower::new(path);
        let n = 400;
        let m = Motion { v: 39.0, phi: 45.0 * D2R_, roll: 1e3, bank: 45.0 * D2R_ };
        for i in 0..=n {
            let q = path.at(path.length() * i as f64 / n as f64);
            let st = f.steer(q.x, q.y, q.th, m);
            assert!(wrap_pi(st.heading - q.th).abs() < 0.2, "step {i} seg {} heading {} vs {}", f.seg, st.heading, q.th);
        }
        assert_eq!(f.seg, 3);
    }

    #[test]
    fn off_the_circle_the_heading_points_back_to_it() {
        // left orbit round (0, 150): 50 m outside it the desired heading turns in toward the centre
        let path = DubinsPath { start: Pose { x: 0.0, y: 0.0, th: 0.0 }, r: 150.0, s0: 1.0, s1: 1.0, a0: PI, straight: 0.0, a1: 0.0 };
        let mut f = Follower::new(path);
        let st = f.steer(0.0, -50.0, 0.0, Motion { v: 39.0, phi: 0.0, roll: 8.0 * D2R_, bank: 45.0 * D2R_ });
        assert!(st.heading > 0.1, "{}", st.heading);
    }

    /// Kinematic glide: constant speed, bank slewed at a roll rate, turn rate g·tan(φ)/V; returns the
    /// pose when the follower reports the path done (or after a time limit).
    fn fly(path: DubinsPath, v: f64, bank: f64) -> (Pose, f64) {
        let (dt, roll) = (0.02, 8.0 * D2R_);
        let mut f = Follower::new(path);
        let (mut x, mut y, mut psi, mut phi) = (path.start.x, path.start.y, path.start.th, 0.0f64);
        let mut t = 0.0;
        let mut after = 0.0;
        // follow the path; once it is done, hold the target line (as the final-approach law does) for 20 s
        while after < 20.0 && t < 400.0 {
            let m = Motion { v, phi, roll, bank };
            let st = if f.seg < 3 {
                f.steer(x, y, psi, m)
            } else {
                after += dt;
                let end = path.at(path.length());
                let e = end.th.cos() * (y - end.y) - end.th.sin() * (x - end.x);
                Steer { heading: end.th - (PI / 3.0) * (2.0 / PI) * (e / 120.0).atan(), ff: 0.0, turn: 0.0, to_go: 0.0, offset: e.abs(), arc_left: 0.0, straight_left: 0.0 }
            };
            let cmd = bank_command(&st, m, psi, 3.0);
            phi += (cmd - phi).clamp(-roll * dt, roll * dt);
            psi = wrap_pi(psi + 9.80665 * phi.tan() / v * dt);
            x += v * psi.cos() * dt;
            y += v * psi.sin() * dt;
            t += dt;
        }
        (Pose { x, y, th: psi }, t)
    }

    #[test]
    fn a_flown_path_arrives_on_the_target_line() {
        let (v, bank) = (39.0, 45.0 * D2R_);
        let r = v * v / (9.80665 * bank.tan());
        let target = Pose { x: 0.0, y: 0.0, th: 0.0 };
        let mut worst = (0.0f64, 0.0f64);
        for i in 0..72 {
            // starts on rings 600–2400 m out, every heading
            let a = i as f64 * 0.7;
            let rr = 600.0 + 600.0 * (i % 4) as f64;
            let start = Pose { x: rr * a.cos() - 800.0, y: rr * a.sin(), th: wrap_pi(a * 2.3) };
            let p = shortest(start, target, r);
            let (end, _) = fly(p, v, bank);
            // distance off the final line (y) and heading error where the path ends
            worst.0 = worst.0.max(end.y.abs());
            worst.1 = worst.1.max(wrap_pi(end.th).abs());
            assert!(end.y.abs() < 25.0 && wrap_pi(end.th).abs() < 10.0 * D2R_, "case {i}: start {start:?} ended {end:?}");
        }
        eprintln!("worst: {:.1} m off the line, {:.1}° off heading", worst.0, worst.1 / D2R_);
    }

    #[test]
    fn straight_ahead_is_a_straight_line() {
        let p = shortest(Pose { x: 0.0, y: 0.0, th: 0.0 }, Pose { x: 1000.0, y: 0.0, th: 0.0 }, 100.0);
        assert!((p.length() - 1000.0).abs() < 1e-9 && p.turn() < 1e-9);
    }

    #[test]
    fn reversal_onto_a_parallel_line_costs_half_a_circle_each_side() {
        // heading east, target 2r to the left heading west: a single left semicircle
        let r = 100.0;
        let p = shortest(Pose { x: 0.0, y: 0.0, th: 0.0 }, Pose { x: 0.0, y: 2.0 * r, th: PI }, r);
        assert!((p.length() - PI * r).abs() < 1e-6, "{}", p.length());
        assert_eq!(p.s0, 1.0);
    }

    #[test]
    fn shorter_than_or_equal_to_any_candidate() {
        let (p0, p1) = (Pose { x: 0.0, y: 0.0, th: 1.0 }, Pose { x: 700.0, y: -300.0, th: -2.0 });
        let best = shortest(p0, p1, 150.0);
        for (s0, s1) in [(1.0, 1.0), (-1.0, -1.0), (1.0, -1.0), (-1.0, 1.0)] {
            if let Some(c) = candidate(p0, p1, 150.0, s0, s1) {
                assert!(best.length() <= c.length() + 1e-9);
            }
        }
    }
}

