//! Physical constants, unit conversions and small math helpers. All internal state is SI.

use serde::Serialize;

pub const G: f64 = 9.81;
/// Sea-level ISA air density, kg/m³.
pub const RHO: f64 = 1.225;
/// m/s per knot.
pub const KT: f64 = 0.514444;
/// m per foot.
pub const FT: f64 = 0.3048;
pub const D2R: f64 = std::f64::consts::PI / 180.0;
pub const PI: f64 = std::f64::consts::PI;

/// Same semantics as `Math.max(lo, Math.min(hi, v))`.
#[inline]
pub fn clamp(v: f64, lo: f64, hi: f64) -> f64 {
    lo.max(hi.min(v))
}

/// Wrap an angle to (-π, π].
#[inline]
pub fn wrap_pi(mut a: f64) -> f64 {
    while a > PI {
        a -= 2.0 * PI;
    }
    while a <= -PI {
        a += 2.0 * PI;
    }
    a
}

/// Moves `current` toward `target` by at most `rate * dt`.
#[inline]
pub fn slew(current: f64, target: f64, rate: f64, dt: f64) -> f64 {
    current + clamp(target - current, -rate * dt, rate * dt)
}

/// Compass heading (deg, 0 = north) from math angle (rad, 0 = east, CCW positive).
pub fn heading_deg(psi: f64) -> f64 {
    (((90.0 - psi / D2R) % 360.0) + 360.0) % 360.0
}

#[derive(Clone, Copy, Debug, Default, Serialize, PartialEq)]
pub struct Vec2 {
    pub x: f64,
    pub y: f64,
}

/// (x, y) point in the airfield frame: x east along runway 09, y north, metres.
pub type Point = [f64; 2];

/// Integer rendering of a rounded value ("-0" never appears). Avoids the float formatter in the WASM build.
pub fn r0(v: f64) -> String {
    let n = v.round() as i64;
    n.to_string()
}

/// One-decimal rendering via integer arithmetic (half away from zero).
pub fn f1(v: f64) -> String {
    let n = (v * 10.0).round() as i64;
    let a = n.unsigned_abs();
    format!("{}{}.{}", if n < 0 { "-" } else { "" }, a / 10, a % 10)
}
