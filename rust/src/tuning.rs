//! Tunable parameters and model perturbations.
//!
//! `Tuning` gathers the control-law gains and the landing parameters that `circuitsim autotune`
//! searches over; its defaults are the values the suites and the page use. `Perturb` scales parts of
//! the airframe away from nominal, so tuning can be scored on models that are slightly wrong, as any
//! real aircraft model is.

use serde::Serialize;

#[derive(Clone, Debug, Serialize, PartialEq)]
pub struct Tuning {
    // --- 6DOF inner loops (see `sixdof::fcs`)
    pub k_theta: f64,
    pub k_q: f64,
    pub ki_theta: f64,
    pub k_phi: f64,
    pub k_p: f64,
    pub k_beta: f64,
    pub k_r: f64,
    // --- landing
    /// Normal-technique flare start, seconds of sink above the ground.
    pub flare_lead: f64,
    /// Short-field flare start, s.
    pub flare_lead_short: f64,
    /// Power-off flare start, s.
    pub flare_lead_eo: f64,
    /// Power-off approach speed with flap 20 or more, kt.
    pub vapp_eo_kt: f64,
    /// Approach-speed gust margin, × measured σ.
    pub gust_k: f64,
    /// Power-off gust margin, × σ.
    pub gust_k_eo: f64,
    /// Flare lull protection engages below this multiple of the touchdown speed.
    pub lull_vs: f64,
    /// Power-off planning: required height multiplier.
    pub eo_margin_factor: f64,
    /// Power-off planning: extra height margin, m.
    pub eo_margin_height: f64,
    /// Clean power-off glide speed, kt.
    pub vbg_kt: f64,
    /// Bank for power-off turns, deg: both the turn the glide planner costs and the one flown.
    pub eo_turn_bank: f64,
    /// Power-off plan reserve: height in hand, as a fraction of the path's need, kept before the plan
    /// optimises its aim point.
    pub eo_reserve: f64,
}

impl Default for Tuning {
    fn default() -> Self {
        // from `circuitsim autotune 18 120` then `autotune 14 120` (23 Oct 2026): strict runway-only
        // scoring, a penalty for any hard arrival, 20 parameters; power-off approach speed then reset to
        // 67.2 kt (65.7 gave four 4.2–4.5 ft/s arrivals) and the gust margin to 1.33 σ (0 cost gusty short-field
        // landings); hold-out 169/240 → 175/240, no hard arrivals
        Tuning {
            k_theta: 2.14,
            k_q: 4.71,
            ki_theta: 0.95,
            k_phi: 3.72,
            k_p: 1.11,
            k_beta: 2.09,
            k_r: 0.62,
            flare_lead: 2.2,
            flare_lead_short: 1.9,
            flare_lead_eo: 2.6,
            vapp_eo_kt: 67.2,
            gust_k: 1.33,
            gust_k_eo: 1.48,
            lull_vs: 0.97,
            eo_margin_factor: 1.04,
            eo_margin_height: 5.0,
            // the trimmed model's glide is flat (L/D 8.4–9.0 over 68–82 kt, best at 76): 70 kt gives 8.7 and
            // costs less speed to reach from the 65 kt climb, with a smaller turn radius
            vbg_kt: 70.0,
            eo_turn_bank: 45.0,
            eo_reserve: 0.2,
        }
    }
}

/// How a parameter is searched: multiplicatively (gains) or additively, within bounds.
#[derive(Clone, Copy, Debug)]
pub struct Dim {
    pub name: &'static str,
    pub log: bool,
    /// One unit of search step: a factor of e^scale (log) or ± scale (linear).
    pub scale: f64,
    pub lo: f64,
    pub hi: f64,
}

pub const NT: usize = 19;

pub const DIMS: [Dim; NT] = [
    Dim { name: "k_theta", log: true, scale: 0.35, lo: 0.8, hi: 5.0 },
    Dim { name: "k_q", log: true, scale: 0.35, lo: 0.6, hi: 5.0 },
    Dim { name: "ki_theta", log: true, scale: 0.4, lo: 0.3, hi: 4.0 },
    Dim { name: "k_phi", log: true, scale: 0.35, lo: 1.0, hi: 6.0 },
    Dim { name: "k_p", log: true, scale: 0.35, lo: 0.15, hi: 1.2 },
    Dim { name: "k_beta", log: true, scale: 0.35, lo: 0.5, hi: 4.0 },
    Dim { name: "k_r", log: true, scale: 0.35, lo: 0.3, hi: 2.5 },
    Dim { name: "flare_lead", log: false, scale: 0.3, lo: 2.2, hi: 3.6 },
    Dim { name: "flare_lead_short", log: false, scale: 0.3, lo: 1.9, hi: 3.2 },
    Dim { name: "flare_lead_eo", log: false, scale: 0.3, lo: 2.0, hi: 3.6 },
    Dim { name: "vapp_eo_kt", log: false, scale: 2.0, lo: 60.0, hi: 72.0 },
    Dim { name: "gust_k", log: false, scale: 0.5, lo: 0.0, hi: 3.0 },
    Dim { name: "gust_k_eo", log: false, scale: 0.5, lo: 0.0, hi: 3.0 },
    Dim { name: "lull_vs", log: false, scale: 0.02, lo: 0.9, hi: 1.02 },
    Dim { name: "eo_margin_factor", log: false, scale: 0.05, lo: 0.9, hi: 1.3 },
    Dim { name: "eo_margin_height", log: false, scale: 8.0, lo: -10.0, hi: 60.0 },
    Dim { name: "vbg_kt", log: false, scale: 2.0, lo: 66.0, hi: 84.0 },
    Dim { name: "eo_turn_bank", log: false, scale: 4.0, lo: 25.0, hi: 45.0 },
    Dim { name: "eo_reserve", log: false, scale: 0.1, lo: 0.1, hi: 0.8 },
];

impl Tuning {
    /// The hand-tuned values before the first autotune run (kept for comparison, and used for the
    /// point-mass model, whose landing behaviour the 6DOF-tuned values do not suit).
    pub fn hand_tuned() -> Self {
        Tuning {
            k_theta: 2.0,
            k_q: 2.0,
            ki_theta: 1.2,
            k_phi: 3.0,
            k_p: 0.4,
            k_beta: 1.5,
            k_r: 0.8,
            flare_lead: 2.7,
            flare_lead_short: 2.3,
            flare_lead_eo: 2.75,
            vapp_eo_kt: 66.0,
            gust_k: 1.33,
            gust_k_eo: 1.6,
            lull_vs: 1.3,
            eo_margin_factor: 1.1,
            eo_margin_height: 20.0,
            vbg_kt: 70.0,
            eo_turn_bank: 45.0,
            eo_reserve: 0.3,
        }
    }
    /// Default parameters for a flight model.
    pub fn for_model(model: crate::aircraft::ModelKind) -> Self {
        match model {
            crate::aircraft::ModelKind::SixDof => Tuning::default(),
            crate::aircraft::ModelKind::PointMass => Tuning::hand_tuned(),
        }
    }
    pub fn to_vec(&self) -> [f64; NT] {
        [
            self.k_theta,
            self.k_q,
            self.ki_theta,
            self.k_phi,
            self.k_p,
            self.k_beta,
            self.k_r,
            self.flare_lead,
            self.flare_lead_short,
            self.flare_lead_eo,
            self.vapp_eo_kt,
            self.gust_k,
            self.gust_k_eo,
            self.lull_vs,
            self.eo_margin_factor,
            self.eo_margin_height,
            self.vbg_kt,
            self.eo_turn_bank,
            self.eo_reserve,
        ]
    }
    pub fn from_vec(v: &[f64; NT]) -> Self {
        Tuning {
            k_theta: v[0],
            k_q: v[1],
            ki_theta: v[2],
            k_phi: v[3],
            k_p: v[4],
            k_beta: v[5],
            k_r: v[6],
            flare_lead: v[7],
            flare_lead_short: v[8],
            flare_lead_eo: v[9],
            vapp_eo_kt: v[10],
            gust_k: v[11],
            gust_k_eo: v[12],
            lull_vs: v[13],
            eo_margin_factor: v[14],
            eo_margin_height: v[15],
            vbg_kt: v[16],
            eo_turn_bank: v[17],
            eo_reserve: v[18],
        }
    }
    /// The parameters at normalised search coordinates `z` around `base` (z = 0 is `base`), clamped to bounds.
    pub fn at(base: &Tuning, z: &[f64; NT]) -> Tuning {
        let b = base.to_vec();
        let mut v = [0.0; NT];
        for i in 0..NT {
            let d = DIMS[i];
            let x = if d.log { b[i] * (d.scale * z[i]).exp() } else { b[i] + d.scale * z[i] };
            v[i] = x.clamp(d.lo, d.hi);
        }
        Tuning::from_vec(&v)
    }
}

/// Model error applied to the nominal airframe.
#[derive(Clone, Copy, Debug, Serialize, PartialEq)]
pub struct Perturb {
    /// Mass multiplier.
    pub mass: f64,
    /// Multiplier on every control-surface moment (elevator, aileron, rudder).
    pub control_power: f64,
    /// Multiplier on the actuator lag.
    pub actuator_lag: f64,
    /// Multiplier on the static pitch stability Cmα.
    pub pitch_stability: f64,
}

impl Default for Perturb {
    fn default() -> Self {
        Perturb { mass: 1.0, control_power: 1.0, actuator_lag: 1.0, pitch_stability: 1.0 }
    }
}
