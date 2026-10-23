import { Aircraft } from './aircraft';
import { Point } from './airfield';
import { Wind } from './atmosphere';
import { D2R, G, clamp, wrapPi } from './units';

/** Proportional-integral controller: out = i + kp·e, with a clamped integrator and optional conditional integration. */
export class PI {
  i = 0;
  constructor(readonly kp: number, readonly ki: number, readonly iMin: number, readonly iMax: number,
              readonly out: [number, number] = [-Infinity, Infinity], readonly antiWindup = false) {}
  reset(i: number): void { this.i = clamp(i, this.iMin, this.iMax); }
  /** Integrate first, then compute output (used where the integrator leads). */
  update(e: number, dt: number): number {
    this.i = clamp(this.i + this.ki * e * dt, this.iMin, this.iMax);
    return clamp(this.i + this.kp * e, this.out[0], this.out[1]);
  }
  /** Output from the current integrator, then integrate only if that does not deepen saturation. */
  updateConditional(e: number, dt: number): number {
    const u = this.i + this.kp * e;
    const [lo, hi] = this.out;
    if ((u < hi || e < 0) && (u > lo || e > 0)) this.i = clamp(this.i + this.ki * e * dt, this.iMin, this.iMax);
    return clamp(u, lo, hi);
  }
}

/** Lateral guidance: vector-field line following plus a bank command to fly the desired ground track. */
export class TrackGuidance {
  constructor(readonly approachAngle = 60 * D2R, readonly gainDistance = 180, readonly turnTimeConst = 3) {}

  follow(ac: Aircraft, A: Point, B: Point): { crossTrack: number; remaining: number; desiredTrack: number } {
    const dx = B[0] - A[0], dy = B[1] - A[1], L = Math.hypot(dx, dy), ux = dx / L, uy = dy / L;
    const e = ux * (ac.y - A[1]) - uy * (ac.x - A[0]);                // + = left of track
    const remaining = (B[0] - ac.x) * ux + (B[1] - ac.y) * uy;
    const desiredTrack = Math.atan2(uy, ux) - this.approachAngle * (2 / Math.PI) * Math.atan(e / this.gainDistance);
    return { crossTrack: e, remaining, desiredTrack };
  }

  bankFor(ac: Aircraft, wind: Wind, desiredTrack: number, limit: number): number {
    const gv = ac.groundVelocity(wind), chi = Math.atan2(gv.y, gv.x), Vg = Math.hypot(gv.x, gv.y);
    return clamp(Math.atan((Vg * wrapPi(desiredTrack - chi)) / (G * this.turnTimeConst)), -limit, limit);
  }

  /** Distance before a leg's end to start a 90° turn: radius plus roll-in distance. */
  turnLead(ac: Aircraft, wind: Wind, bankMax: number, rollRate: number): number {
    const gv = ac.groundVelocity(wind), Vg = Math.hypot(gv.x, gv.y);
    return (Vg * Vg) / (G * Math.tan(bankMax)) + Vg * (bankMax / rollRate) * 0.5;
  }
}

/** Vertical path control: flight-path angle → pitch, with V/S and altitude holds layered on top. */
export class FlightPathController {
  readonly gammaPI = new PI(0.8, 0.4, -0.08, 0.08);

  gamma(ac: Aircraft, gCmd: number, dt: number): number {
    const err = gCmd - ac.gamma;
    this.gammaPI.i = clamp(this.gammaPI.i + this.gammaPI.ki * err * dt, this.gammaPI.iMin, this.gammaPI.iMax);
    return clamp(ac.alphaF + gCmd + this.gammaPI.kp * err + this.gammaPI.i, ac.spec.minPitch, ac.spec.maxPitch);
  }
  verticalSpeed(ac: Aircraft, vs: number, dt: number): number {
    return this.gamma(ac, Math.asin(clamp(vs / Math.max(ac.V, 10), -0.25, 0.25)), dt);
  }
  altitude(ac: Aircraft, hCmd: number, dt: number, vsMax = 3): number {
    return this.verticalSpeed(ac, clamp(0.15 * (hCmd - ac.h), -vsMax, vsMax), dt);
  }
}

/** Auto-throttle: airspeed error → throttle, with conditional integration (anti-windup). */
export class AutoThrottle {
  readonly pi = new PI(0.10, 0.03, 0, 1, [0, 1]);
  /** Bumpless engage from the current throttle. */
  engage(currentThr: number): void { this.pi.reset(currentThr); }
  update(ac: Aircraft, Vtarget: number, dt: number): number { return this.pi.updateConditional(Vtarget - ac.V, dt); }
}

/** Climb at full power holding airspeed with pitch (speed on elevator). */
export class SpeedOnPitch {
  readonly pi: PI;
  constructor(kp = 0.015, ki = 0.004, iMin = -0.05, iMax = 0.2) { this.pi = new PI(kp, ki, iMin, iMax); }
  engage(gamma: number): void { this.pi.reset(gamma); }
  /** Returns a flight-path angle command. */
  update(ac: Aircraft, Vtarget: number, dt: number, gMin: number): number {
    const e = ac.V - Vtarget;
    this.pi.i = clamp(this.pi.i + this.pi.ki * e * dt, this.pi.iMin, this.pi.iMax);
    return clamp(this.pi.i + this.pi.kp * e, gMin, 0.2);
  }
}
