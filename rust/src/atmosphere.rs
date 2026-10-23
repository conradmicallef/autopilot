//! Wind: steady component plus optional Ornstein–Uhlenbeck gusts, driven by a seeded PRNG.

use crate::units::{Vec2, D2R, KT, PI};
use serde::{Deserialize, Serialize};

/// Park–Miller PRNG so a run can be replayed from its seed (bit-identical to the TypeScript version).
#[derive(Clone, Debug)]
pub struct Rng {
    seed: f64,
}

impl Rng {
    pub fn new(seed: f64) -> Self {
        Rng { seed: if seed <= 0.0 { 1.0 } else { seed } }
    }
    pub fn next(&mut self) -> f64 {
        self.seed = (self.seed * 16807.0) % 2147483647.0;
        self.seed / 2147483647.0
    }
    pub fn gauss(&mut self) -> f64 {
        let a = self.next();
        let b = self.next();
        (-2.0 * (a + 1e-9).ln()).sqrt() * (2.0 * PI * b).cos()
    }
}

#[derive(Clone, Copy, Debug, Serialize, Deserialize, PartialEq)]
pub struct WindSettings {
    /// Direction the wind blows FROM, degrees true.
    pub from_deg: f64,
    pub speed_kt: f64,
    pub gusts: bool,
}

#[derive(Clone, Debug)]
pub struct Wind {
    pub settings: WindSettings,
    /// Correlation time of the gusts, s.
    pub gust_tau: f64,
    /// Standard deviation of each gust axis, m/s.
    pub gust_sigma: f64,
    gust: Vec2,
    /// Current wind vector in m/s (the air's velocity over the ground).
    pub v: Vec2,
    rng: Rng,
}

impl Wind {
    pub fn new(settings: WindSettings, rng: Rng) -> Self {
        Wind { settings, gust_tau: 3.0, gust_sigma: 1.8, gust: Vec2::default(), v: Vec2::default(), rng }
    }

    pub fn update(&mut self, dt: f64) {
        let s = self.settings;
        let to_angle = (90.0 - (s.from_deg + 180.0)) * D2R;
        let ws = s.speed_kt * KT;
        if s.gusts {
            let k = self.gust_sigma * ((2.0 * dt) / self.gust_tau).sqrt();
            let gx = self.rng.gauss();
            self.gust.x += (-self.gust.x / self.gust_tau) * dt + k * gx;
            let gy = self.rng.gauss();
            self.gust.y += (-self.gust.y / self.gust_tau) * dt + k * gy;
        } else {
            self.gust.x *= 0.98;
            self.gust.y *= 0.98;
        }
        self.v.x = ws * to_angle.cos() + self.gust.x;
        self.v.y = ws * to_angle.sin() + self.gust.y;
    }

    /// A steady wind with vector `v` (no gusts): what the autopilot believes, built from its estimate.
    pub fn steady(v: Vec2) -> Self {
        let speed = v.x.hypot(v.y);
        // the direction the wind blows from, degrees true (math angle of the vector + 180°, to compass)
        let from = (90.0 - (v.y.atan2(v.x) / D2R + 180.0)).rem_euclid(360.0);
        Wind {
            settings: WindSettings { from_deg: from, speed_kt: speed / KT, gusts: false },
            gust_tau: 3.0,
            gust_sigma: 0.0,
            gust: Vec2::default(),
            v,
            rng: Rng::new(1.0),
        }
    }

    /// Wind component along a track angle (positive = tailwind).
    pub fn along(&self, psi: f64) -> f64 {
        self.v.x * psi.cos() + self.v.y * psi.sin()
    }
}
