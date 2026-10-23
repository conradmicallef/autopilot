import { Wind } from './atmosphere';
import { D2R, G, RHO, Vec2, clamp, slew, wrapPi } from './units';

export interface FlapIncrement { dCL: number; dCD: number; dCLmax: number; }

/** Static description of an airframe. */
export interface AircraftSpec {
  name: string;
  mass: number;          // kg
  wingArea: number;      // m²
  CLalpha: number;       // per rad
  CL0: number;
  CLmax: number;
  CD0: number;
  inducedK: number;      // CD = CD0 + k·CL²
  power: number;         // W at the propeller (already × efficiency)
  staticThrust: number;  // N
  rollingMu: number;
  brakeMu: number;
  /** Side-force slope with sideslip, per rad (magnitude; force acts toward the nose). */
  CYbeta: number;
  /** Extra drag coefficient from a windmilling propeller after an engine failure. */
  windmillCD: number;
  maxSideslip: number;
  maxPitch: number; minPitch: number; tailStrikePitch: number;
  /** Flap increments at 0/10/20/30°, interpolated in between. */
  flaps: Record<0 | 10 | 20 | 30, FlapIncrement>;
}

export const C172_LIKE: AircraftSpec = {
  name: 'Light single, C172-class',
  mass: 1000, wingArea: 16.2, CLalpha: 5.0, CL0: 0.30, CLmax: 1.55, CD0: 0.032, inducedK: 0.057,
  power: 70000, staticThrust: 2800, rollingMu: 0.03, brakeMu: 0.35, CYbeta: 0.35, maxSideslip: 15 * D2R, windmillCD: 0.012,
  maxPitch: 15 * D2R, minPitch: -10 * D2R, tailStrikePitch: 12 * D2R,
  flaps: {
    0: { dCL: 0, dCD: 0, dCLmax: 0 },
    10: { dCL: 0.20, dCD: 0.008, dCLmax: 0.15 },
    20: { dCL: 0.35, dCD: 0.020, dCLmax: 0.30 },
    30: { dCL: 0.45, dCD: 0.035, dCLmax: 0.40 }
  }
};

/** What the autopilot asks for each step. The aircraft never applies these directly: see CommandShaper. */
export interface Commands {
  theta: number;   // pitch attitude, rad
  phi: number;     // bank, rad
  beta: number;    // sideslip (rudder), rad: nose heading minus air-path heading, + = nose left (CCW)
  thr: number;     // 0..1
  flaps: number;   // deg
  brake: number;   // 0..1
  /** Rotation phase: the wheels may leave the ground. */
  allowLiftoff: boolean;
  /** Just after lift-off or during a go-around a brief settle must not count as a landing. */
  noTouchdown: boolean;
}

/** Rate limits applied to every command so no mode change can step a control. */
export interface SlewRates { theta: number; phi: number; thr: number; flaps: number; beta: number; }
export const DEFAULT_SLEW: SlewRates = { theta: 3 * D2R, phi: 8 * D2R, thr: 0.3, flaps: 3, beta: 4 * D2R };

export class CommandShaper {
  theta = 0; phi = 0; thr = 0; flaps = 10; beta = 0;
  constructor(public rates: SlewRates = DEFAULT_SLEW) {}
  shape(c: Commands, dt: number): void {
    this.theta = slew(this.theta, c.theta, this.rates.theta, dt);
    this.phi = slew(this.phi, c.phi, this.rates.phi, dt);
    this.thr = slew(this.thr, c.thr, this.rates.thr, dt);
    this.flaps = slew(this.flaps, c.flaps, this.rates.flaps, dt);
    this.beta = slew(this.beta, c.beta, this.rates.beta, dt);
  }
}

export type ContactEvent =
  | { kind: 'liftoff'; V: number }
  | { kind: 'touchdown'; sink: number; /** nose heading minus ground track, rad */ crab: number; V: number; groundSpeed: number };

/** Point-mass aircraft with first-order attitude dynamics and inner attitude-hold loops. */
export class Aircraft {
  x = 0; y = 0; h = 0;
  V = 0;           // true airspeed (sea level, so = IAS)
  Vg = 0;          // ground speed while on the ground
  psi = 0;         // air-path heading, rad (0 = east); equals the nose heading in coordinated flight
  beta = 0;        // sideslip, rad: nose heading = psi + beta
  gamma = 0;       // flight-path angle
  theta = 0; q = 0;
  phi = 0; p = 0;
  alphaF = 0;      // filtered angle of attack, used as autopilot feed-forward
  thr = 0; flapPos = 10; brake = 0;
  onGround = true; stalled = false;
  engineFailed = false;
  /** Heading held on the ground after a landing away from the runway; null = steer to the runway centreline. */
  rolloutHeading: number | null = null;
  /** Runway direction steered to on the ground: 0 for 09, π for 27. */
  runwayHeading = 0;
  readonly shaper = new CommandShaper();

  constructor(readonly spec: AircraftSpec = C172_LIKE) {}

  get flaps(): number { return Math.round(this.flapPos); }
  /** Where the nose points (what a compass reads). */
  get noseHeading(): number { return wrapPi(this.psi + this.beta); }
  /** Ground track angle, rad. */
  track(wind: Wind): number { const gv = this.groundVelocity(wind); return Math.atan2(gv.y, gv.x); }
  /** Side force from sideslip, N, positive toward the nose (CCW). */
  sideForce(): number { return 0.5 * RHO * this.V * this.V * this.spec.wingArea * this.spec.CYbeta * this.beta; }
  /** Bank that cancels the sideslip side force: the wing-low angle for a straight path. */
  wingLowBank(): number {
    if (this.onGround) return 0;
    const L = this.aero(this.V, this.theta - this.gamma).L;
    return Math.asin(clamp(-this.sideForce() / Math.max(L, 1), -0.3, 0.3));
  }
  get alpha(): number { return this.onGround ? this.theta : this.theta - this.gamma; }
  get verticalSpeed(): number { return this.onGround ? 0 : this.V * Math.sin(this.gamma); }

  groundVelocity(wind: Wind): Vec2 {
    if (this.onGround) return { x: this.Vg * Math.cos(this.psi), y: this.Vg * Math.sin(this.psi) };
    const hv = this.V * Math.cos(this.gamma);
    return { x: hv * Math.cos(this.psi) + wind.v.x, y: hv * Math.sin(this.psi) + wind.v.y };
  }

  private flapIncrement(pos = this.flapPos): FlapIncrement {
    const f = this.spec.flaps, p = clamp(pos, 0, 30);
    const i = Math.min(2, Math.floor(p / 10)), lo = (i * 10) as 0 | 10 | 20, hi = (i * 10 + 10) as 10 | 20 | 30;
    const t = (p - lo) / 10, a = f[lo], b = f[hi];
    return { dCL: a.dCL + (b.dCL - a.dCL) * t, dCD: a.dCD + (b.dCD - a.dCD) * t, dCLmax: a.dCLmax + (b.dCLmax - a.dCLmax) * t };
  }

  aero(V: number, alpha: number): { L: number; D: number; stalled: boolean } {
    const s = this.spec, f = this.flapIncrement();
    let CL = s.CL0 + f.dCL + s.CLalpha * alpha;
    const CLmax = s.CLmax + f.dCLmax;
    let stalled = false;
    if (CL > CLmax) { stalled = true; CL = CLmax - (CL - CLmax) * 1.5; }
    CL = Math.max(CL, -0.8);
    const CD = s.CD0 + f.dCD + s.inducedK * CL * CL + 0.3 * this.beta * this.beta + (this.engineFailed ? s.windmillCD : 0);
    const qS = 0.5 * RHO * V * V * s.wingArea;
    return { L: qS * CL, D: qS * CD, stalled };
  }

  thrust(V: number): number {
    return this.engineFailed ? 0 : this.thr * Math.min(this.spec.staticThrust, this.spec.power / Math.max(V, 1));
  }

  /** Still-air glide ratio (L/D) in trimmed, power-off, coordinated flight at airspeed V. */
  glideRatio(V: number, flapPos: number): number {
    const s = this.spec, f = this.flapIncrement(flapPos);
    const CL = (s.mass * G) / (0.5 * RHO * V * V * s.wingArea);
    return CL / (s.CD0 + f.dCD + s.inducedK * CL * CL + s.windmillCD);
  }

  step(dt: number, cmd: Commands, wind: Wind): ContactEvent | null {
    const sh = this.shaper;
    sh.shape(cmd, dt);
    // inner loops: attitude error → rate demand → rate with actuator lag
    const qCmd = clamp(1.5 * (sh.theta - this.theta), -5 * D2R, 5 * D2R);
    this.q += ((qCmd - this.q) * dt) / 0.3; this.theta += this.q * dt;
    const pCmd = clamp(2.0 * (sh.phi - this.phi), -15 * D2R, 15 * D2R);
    this.p += ((pCmd - this.p) * dt) / 0.2; this.phi += this.p * dt;
    this.thr = slew(this.thr, sh.thr, 0.8, dt);          // engine spool
    this.flapPos = sh.flaps; this.brake = cmd.brake;
    this.beta = this.onGround ? 0 : clamp(sh.beta, -this.spec.maxSideslip, this.spec.maxSideslip);

    return this.onGround ? this.stepGround(dt, cmd, wind) : this.stepAir(dt, cmd, wind);
  }

  private stepGround(dt: number, cmd: Commands, wind: Wind): ContactEvent | null {
    const s = this.spec;
    this.phi = 0; this.p = 0; this.shaper.phi = 0; this.shaper.beta = 0; this.gamma = 0;
    this.theta = clamp(this.theta, 0, s.tailStrikePitch);
    if (this.theta === 0 && this.q < 0) this.q = 0;
    this.V = Math.max(0, this.Vg - wind.along(this.psi));
    const a = this.aero(this.V, this.theta), T = this.thrust(this.V);
    const W = s.mass * G, N = Math.max(0, W - a.L), mu = s.rollingMu + this.brake * s.brakeMu;
    const acc = (T - a.D - mu * N) / s.mass;
    this.Vg = Math.max(0, this.Vg + acc * dt);
    this.alphaF = this.theta;
    // nose-wheel steering: back to the centreline (runway axis is psi = 0), or straight ahead in a field
    // runway (centreline y = 0, either direction): steer back onto it; field: hold the heading
    const psiCmd = this.rolloutHeading ?? this.runwayHeading + clamp(-0.03 * this.y * Math.cos(this.runwayHeading), -0.08, 0.08);
    this.psi = wrapPi(this.psi + (wrapPi(psiCmd - this.psi) * dt) / 0.6);
    this.x += this.Vg * Math.cos(this.psi) * dt; this.y += this.Vg * Math.sin(this.psi) * dt;
    if (cmd.allowLiftoff && a.L > W * 1.01) {
      this.onGround = false; this.h = 0.01;
      return { kind: 'liftoff', V: this.V };
    }
    return null;
  }

  private stepAir(dt: number, cmd: Commands, wind: Wind): ContactEvent | null {
    const s = this.spec, alpha = this.theta - this.gamma, a = this.aero(this.V, alpha), T = this.thrust(this.V);
    this.stalled = a.stalled;
    const Vs = Math.max(this.V, 5), mV = s.mass * Vs;
    const Vdot = (T * Math.cos(alpha) - a.D) / s.mass - G * Math.sin(this.gamma);
    const Lt = a.L + T * Math.sin(alpha);
    const gdot = (Lt * Math.cos(this.phi)) / mV - (G * Math.cos(this.gamma)) / Vs;
    // horizontal turn: lift tilted by bank, plus the sideslip side force pulling the path toward the nose
    const psidot = (Lt * Math.sin(this.phi) + this.sideForce()) / (mV * Math.cos(this.gamma));
    this.V += Vdot * dt; this.gamma += gdot * dt; this.psi = wrapPi(this.psi + psidot * dt);
    this.alphaF += ((alpha - this.alphaF) * dt) / 0.5;
    const gv = this.groundVelocity(wind);
    this.x += gv.x * dt; this.y += gv.y * dt; this.h += this.V * Math.sin(this.gamma) * dt;

    if (this.h > 0) return null;
    if (cmd.noTouchdown) { this.h = 0; this.gamma = Math.max(this.gamma, 0); return null; }
    const sink = -this.V * Math.sin(this.gamma);
    const crab = wrapPi(this.noseHeading - Math.atan2(gv.y, gv.x));
    this.psi = this.noseHeading; this.beta = 0;   // wheels track where the nose points
    const near09 = Math.abs(wrapPi(this.psi)) < 30 * D2R, near27 = Math.abs(wrapPi(this.psi - Math.PI)) < 30 * D2R;
    const onRunwayLine = (near09 || near27) && Math.abs(this.y) < 60;
    this.runwayHeading = near27 ? Math.PI : 0;
    this.rolloutHeading = onRunwayLine ? null : this.psi;
    this.h = 0; this.onGround = true;
    this.Vg = this.V * Math.cos(this.gamma) + wind.along(this.psi);
    this.gamma = 0;
    return { kind: 'touchdown', sink, crab, V: this.V, groundSpeed: this.Vg };
  }
}
