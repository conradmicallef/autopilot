//! Cessna 172P aerodynamic data, from the open JSBSim model (`aircraft/c172p/c172p.xml`,
//! JSBSim-Team/jsbsim, LGPL). Tables are copied as published and looked up with clamped linear
//! interpolation, as JSBSim does. Angles in rad, flap in degrees, h/b is wing height over span.
//!
//! Sign conventions are this crate's, not JSBSim's: elevator + = trailing edge down (same),
//! aileron + = right roll (same as JSBSim's left-aileron position), rudder + = nose right
//! (JSBSim's is nose left, so the rudder terms change sign).

use crate::units::clamp;

/// 1-D table lookup, clamped at the ends.
pub fn lookup(t: &[[f64; 2]], x: f64) -> f64 {
    let n = t.len();
    if x <= t[0][0] {
        return t[0][1];
    }
    if x >= t[n - 1][0] {
        return t[n - 1][1];
    }
    let mut i = 0;
    while t[i + 1][0] < x {
        i += 1;
    }
    let f = (x - t[i][0]) / (t[i + 1][0] - t[i][0]);
    t[i][1] + (t[i + 1][1] - t[i][1]) * f
}

/// Row/column bracket for a clamped lookup: index and fraction.
fn bracket(xs: &[f64], x: f64) -> (usize, f64) {
    let n = xs.len();
    if x <= xs[0] {
        return (0, 0.0);
    }
    if x >= xs[n - 1] {
        return (n - 2, 1.0);
    }
    let mut i = 0;
    while xs[i + 1] < x {
        i += 1;
    }
    (i, (x - xs[i]) / (xs[i + 1] - xs[i]))
}

/// 2-D lookup: rows by `r`, columns by `c`.
fn lookup2<const C: usize>(rows: &[f64], cols: &[f64; C], data: &[[f64; C]], r: f64, c: f64) -> f64 {
    let (i, fr) = bracket(rows, r);
    let (j, fc) = bracket(cols, c);
    let a = data[i][j] + (data[i][j + 1] - data[i][j]) * fc;
    let b = data[i + 1][j] + (data[i + 1][j + 1] - data[i + 1][j]) * fc;
    a + (b - a) * fr
}

// --- geometry and mass properties (SI)
pub const WING_AREA: f64 = 174.0 * 0.092903;
pub const SPAN: f64 = 35.8 * 0.3048;
pub const CHORD: f64 = 4.9 * 0.3048;
const SLUG_FT2: f64 = 1.355818;
pub const IXX: f64 = 948.0 * SLUG_FT2;
pub const IYY: f64 = 1346.0 * SLUG_FT2;
pub const IZZ: f64 = 1967.0 * SLUG_FT2;
const IN: f64 = 0.0254;
/// JSBSim structural frame (in, x aft, z up) to body axes about the CG (m, x fwd, z down).
pub const fn body(x: f64, y: f64, z: f64) -> [f64; 3] {
    [-(x - CG[0]) * IN, (y - CG[1]) * IN, -(z - CG[2]) * IN]
}
/// Loaded centre of gravity at the model's 1,000 kg (2,205 lb): JSBSim empty aircraft (1,500 lb at
/// x 41, z 36.5 in), pilot and front passenger (180 + 170 lb at x 36, z 24), one rear passenger
/// (155 lb at x 70, z 24) and 200 lb of fuel in the wing tanks (x 56, z 59.4).
pub const CG: [f64; 3] = [43.6, 0.0, 35.7];
/// Aerodynamic reference point (force application), body axes.
pub const AERO_RP: [f64; 3] = body(43.2, 0.0, 59.4);
/// Propeller hub, body axes.
pub const PROP: [f64; 3] = body(-37.7, 0.0, 26.6);
/// Propeller disc area (75 in diameter), m², for the slipstream dynamic pressure.
pub const PROP_DISC: f64 = std::f64::consts::PI * (75.0 * IN / 2.0) * (75.0 * IN / 2.0);
pub const NOSE: [f64; 3] = body(-6.8, 0.0, -19.5);
pub const MAIN_L: [f64; 3] = body(58.2, -43.0, -15.5);
pub const MAIN_R: [f64; 3] = body(58.2, 43.0, -15.5);
pub const TAIL: [f64; 3] = body(188.0, 0.0, 8.0);
/// Gear spring and damper, N/m and N·s/m (5400 / 1800 lbf/ft, 1600 / 600 lbf·s/ft).
pub const MAIN_K: f64 = 5400.0 * 14.5939;
pub const MAIN_C: f64 = 1600.0 * 14.5939;
pub const NOSE_K: f64 = 1800.0 * 14.5939;
pub const NOSE_C: f64 = 600.0 * 14.5939;
/// Height of the aero reference point above the wheel contact plane when parked, m.
pub const WING_H: f64 = (59.4 + 15.5) * IN;

/// Pitch at which the tail skid touches with the mains on the ground (struts at rest), rad.
pub fn tail_strike_pitch() -> f64 {
    ((MAIN_L[2] - TAIL[2]) / (MAIN_L[0] - TAIL[0])).atan()
}

// --- surface limits (rad)
pub const DE_MIN: f64 = -28.0 * crate::units::D2R;
pub const DE_MAX: f64 = 23.0 * crate::units::D2R;
pub const DA_MAX: f64 = 20.0 * crate::units::D2R;
pub const DR_MAX: f64 = 16.0 * crate::units::D2R;
pub const STEER_MAX: f64 = 10.0 * crate::units::D2R;

// --- stall hysteresis: on above α 0.36 rad, off below 0.09 rad
pub const HYST_ON: f64 = 0.36;
pub const HYST_OFF: f64 = 0.09;
/// α of maximum lift (unstalled branch).
pub const ALPHA_CLMAX: f64 = 0.28;

const KCDGE: [[f64; 2]; 13] = [
    [0.0, 0.48], [0.1, 0.515], [0.15, 0.629], [0.2, 0.709], [0.3, 0.815], [0.4, 0.882], [0.5, 0.928],
    [0.6, 0.962], [0.7, 0.988], [0.8, 1.0], [0.9, 1.0], [1.0, 1.0], [1.1, 1.0],
];
const KCLGE: [[f64; 2]; 13] = [
    [0.0, 1.203], [0.1, 1.127], [0.15, 1.09], [0.2, 1.073], [0.3, 1.046], [0.4, 1.055], [0.5, 1.019],
    [0.6, 1.013], [0.7, 1.008], [0.8, 1.006], [0.9, 1.003], [1.0, 1.002], [1.1, 1.0],
];
pub fn k_cdge(hb: f64) -> f64 {
    lookup(&KCDGE, hb)
}
pub fn k_clge(hb: f64) -> f64 {
    lookup(&KCLGE, hb)
}

// --- drag
pub const CDO: f64 = 0.027;
const CDDF: [[f64; 2]; 4] = [[0.0, 0.0], [10.0, 0.007], [20.0, 0.012], [30.0, 0.018]];
const FLAPS4: [f64; 4] = [0.0, 10.0, 20.0, 30.0];
const CDWBH_A: [f64; 26] = [
    -0.0873, -0.0698, -0.0524, -0.0349, -0.0175, 0.0, 0.0175, 0.0349, 0.0524, 0.0698, 0.0873, 0.1047, 0.1222,
    0.1396, 0.1571, 0.1745, 0.192, 0.2094, 0.2269, 0.2443, 0.2618, 0.2793, 0.2967, 0.3142, 0.3316, 0.3491,
];
const CDWBH: [[f64; 4]; 26] = [
    [0.0041, 0.0, 0.0005, 0.0014],
    [0.0013, 0.0004, 0.0025, 0.0041],
    [0.0001, 0.0023, 0.0059, 0.0084],
    [0.0003, 0.0057, 0.0108, 0.0141],
    [0.0020, 0.0105, 0.0172, 0.0212],
    [0.0052, 0.0168, 0.0251, 0.0299],
    [0.0099, 0.0248, 0.0346, 0.0402],
    [0.0162, 0.0342, 0.0457, 0.0521],
    [0.0240, 0.0452, 0.0583, 0.0655],
    [0.0334, 0.0577, 0.0724, 0.0804],
    [0.0442, 0.0718, 0.0881, 0.0968],
    [0.0566, 0.0874, 0.1053, 0.1148],
    [0.0706, 0.1045, 0.1240, 0.1343],
    [0.0860, 0.1232, 0.1442, 0.1554],
    [0.0962, 0.1353, 0.1573, 0.1690],
    [0.1069, 0.1479, 0.1708, 0.1830],
    [0.1180, 0.1610, 0.1849, 0.1975],
    [0.1298, 0.1746, 0.1995, 0.2126],
    [0.1424, 0.1892, 0.2151, 0.2286],
    [0.1565, 0.2054, 0.2323, 0.2464],
    [0.1727, 0.2240, 0.2521, 0.2667],
    [0.1782, 0.2302, 0.2587, 0.2735],
    [0.1716, 0.2227, 0.2507, 0.2653],
    [0.1618, 0.2115, 0.2388, 0.2531],
    [0.1475, 0.1951, 0.2214, 0.2351],
    [0.1097, 0.1512, 0.1744, 0.1866],
];
pub const CD_DE: f64 = 0.06;
pub const CD_BETA: f64 = 0.17;

// --- lift
const CLWBH_A: [f64; 17] =
    [-0.09, 0.0, 0.09, 0.1, 0.12, 0.14, 0.16, 0.17, 0.19, 0.21, 0.24, 0.26, 0.28, 0.30, 0.32, 0.34, 0.36];
/// Columns: normal, stalled (hysteresis branch).
/// JSBSim's c172p has CL 0.73 at α 0.09 and 0.83 at 0.10: a slope of 10/rad over that step, twice the
/// slope either side, which put a kink in the glide polar right at best glide (L/D 8.74 at 70 kt,
/// 8.13 at 72 kt). The 0.09 point is raised onto the line through the points above it (0.78), which
/// removes the jump and keeps the glide performance: best L/D 9.0 at 76 kt, minimum sink 811 fpm at 70.
const CLWBH: [[f64; 2]; 17] = [
    [-0.22, -0.22],
    [0.25, 0.25],
    [0.78, 0.73],
    [0.83, 0.78],
    [0.92, 0.79],
    [1.02, 0.81],
    [1.08, 0.82],
    [1.13, 0.83],
    [1.19, 0.85],
    [1.25, 0.86],
    [1.35, 0.88],
    [1.44, 0.90],
    [1.47, 0.92],
    [1.43, 0.95],
    [1.38, 0.99],
    [1.30, 1.05],
    [1.15, 1.15],
];
const CLDF: [[f64; 2]; 4] = [[0.0, 0.0], [10.0, 0.2], [20.0, 0.3], [30.0, 0.35]];
pub const CL_DE: f64 = 0.43;
pub const CL_ADOT: f64 = 1.7;
pub const CL_Q: f64 = 3.9;

// --- side force
pub const CY_DR: f64 = -0.187;
const A2: [f64; 2] = [0.0, 0.094];
const F2: [f64; 2] = [0.0, 30.0];
const CYP: [[f64; 2]; 2] = [[-0.075, -0.161], [-0.145, -0.231]];
const CYR: [[f64; 2]; 2] = [[0.214, 0.162], [0.267, 0.215]];

// --- roll
pub const CL_P: f64 = -0.484;
const CLR: [[f64; 2]; 2] = [[0.0798, 0.1246], [0.1869, 0.2317]];
pub const CL_DA: f64 = 0.229;
pub const CL_DR: f64 = -0.0147;

// --- pitch
pub const CM0: f64 = 0.1;
pub const CM_ALPHA: f64 = -1.8;
pub const CM_Q: f64 = -12.4;
pub const CM_ADOT: f64 = -7.27;
/// Multiplies the slipstream dynamic pressure.
pub const CM_DE: f64 = -1.122;
const CMDF: [[f64; 2]; 4] = [[0.0, 0.0], [10.0, -0.0654], [20.0, -0.0981], [30.0, -0.114]];

// --- yaw
pub const CN_P: f64 = -0.0278;
pub const CN_R: f64 = -0.0937;
pub const CN_DA: f64 = -0.0053;
/// Multiplies the slipstream dynamic pressure.
pub const CN_DR: f64 = 0.043;

/// CL of wing–body–tail (before ground effect), α in rad; `hyst` 0 normal, 1 stalled branch.
pub fn cl_wbh(alpha: f64, hyst: f64) -> f64 {
    lookup2(&CLWBH_A, &[0.0, 1.0], &CLWBH, alpha, hyst)
}
pub fn cd_wbh(alpha: f64, flap: f64) -> f64 {
    lookup2(&CDWBH_A, &FLAPS4, &CDWBH, alpha, flap)
}
pub fn cl_df(flap: f64) -> f64 {
    lookup(&CLDF, flap)
}
pub fn cd_df(flap: f64) -> f64 {
    lookup(&CDDF, flap)
}
pub fn cm_df(flap: f64) -> f64 {
    lookup(&CMDF, flap)
}
/// Side force coefficient from sideslip (the table is linear to ±0.349 rad and flat beyond).
pub fn cy_b(beta: f64, flap: f64) -> f64 {
    let slope = -0.137 + (0.137 - 0.106) * clamp(flap / 30.0, 0.0, 1.0);
    slope * clamp(beta, -0.349, 0.349) / 0.349
}
pub fn cy_beta_slope(flap: f64) -> f64 {
    cy_b(0.01, flap) / 0.01
}
pub fn cl_b(beta: f64) -> f64 {
    -0.0322 * clamp(beta, -0.349, 0.349) / 0.349
}
pub const CL_BETA: f64 = -0.0322 / 0.349;
pub fn cn_b(beta: f64) -> f64 {
    0.0205 * clamp(beta, -0.349, 0.349) / 0.349
}
pub const CN_BETA: f64 = 0.0205 / 0.349;
pub fn cy_p(alpha: f64, flap: f64) -> f64 {
    lookup2(&A2, &F2, &CYP, alpha, flap)
}
/// Rudder sign does not enter: these are rate derivatives (r + = nose right in both).
pub fn cy_r(alpha: f64, flap: f64) -> f64 {
    lookup2(&A2, &F2, &CYR, alpha, flap)
}
pub fn cl_r(alpha: f64, flap: f64) -> f64 {
    lookup2(&A2, &F2, &CLR, alpha, flap)
}

/// Static lift and drag of the airframe (no rates, no controls): what both models share.
/// `hb`: wing height over span (use a large value out of ground effect).
pub fn polar(alpha: f64, flap: f64, hb: f64, hyst: f64) -> (f64, f64) {
    let kl = k_clge(hb);
    let kd = k_cdge(hb);
    let cl = (cl_wbh(alpha, hyst) + cl_df(flap)) * kl;
    let cd = CDO + (cd_df(flap) + cd_wbh(alpha, flap)) * kd;
    (cl, cd)
}

/// α that gives `cl` on the unstalled branch out of ground effect (bisection; clamps to the table).
pub fn alpha_for_cl(cl: f64, flap: f64) -> f64 {
    let (mut lo, mut hi) = (-0.09, ALPHA_CLMAX);
    for _ in 0..40 {
        let mid = 0.5 * (lo + hi);
        if polar(mid, flap, 9.0, 0.0).0 < cl {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    0.5 * (lo + hi)
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn tables() {
        assert!((cl_wbh(0.0, 0.0) - 0.25).abs() < 1e-12);
        assert!((cl_wbh(0.29, 0.0) - 1.45).abs() < 1e-9);
        assert!((cd_wbh(0.0, 15.0) - (0.0168 + 0.0251) / 2.0).abs() < 1e-12);
        assert!((k_cdge(5.0) - 1.0).abs() < 1e-12);
        let a = alpha_for_cl(0.8, 0.0);
        assert!((polar(a, 0.0, 9.0, 0.0).0 - 0.8).abs() < 1e-6);
        // structural → body: the mains are behind and below the CG
        assert!(MAIN_L[0] < 0.0 && MAIN_L[2] > 1.0 && MAIN_L[1] < 0.0);
    }
}
