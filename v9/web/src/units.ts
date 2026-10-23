/** Physical constants, unit conversions and small math helpers. All internal state is SI. */
export const G = 9.81;
export const RHO = 1.225;          // sea-level ISA air density, kg/m³
export const KT = 0.514444;        // m/s per knot
export const FT = 0.3048;          // m per foot
export const D2R = Math.PI / 180;

export const clamp = (v: number, lo: number, hi: number): number => Math.max(lo, Math.min(hi, v));

/** Wrap an angle to (-π, π]. */
export function wrapPi(a: number): number {
  while (a > Math.PI) a -= 2 * Math.PI;
  while (a <= -Math.PI) a += 2 * Math.PI;
  return a;
}

/** Moves `current` toward `target` by at most `rate * dt`. */
export const slew = (current: number, target: number, rate: number, dt: number): number =>
  current + clamp(target - current, -rate * dt, rate * dt);

/** Compass heading (deg, 0 = north) from math angle (rad, 0 = east, CCW positive). */
export const headingDeg = (psi: number): number => (((90 - psi / D2R) % 360) + 360) % 360;

export interface Vec2 { x: number; y: number; }
