import { Aircraft, Commands } from './aircraft';
import { Circuit, Leg, Point, Runway } from './airfield';
import { Wind } from './atmosphere';
import { AutoThrottle, FlightPathController, SpeedOnPitch, TrackGuidance } from './control';
import { DEFAULT_EO, EngineOutPlan, GlideAssessment, GlidePlanner, RunwayEnd } from './emergency';
import { D2R, FT, G, KT, clamp, wrapPi } from './units';

export type Phase =
  | 'TAKEOFF_ROLL' | 'ROTATE' | 'CLIMB' | 'CROSSWIND' | 'DOWNWIND'
  | 'BASE' | 'FINAL' | 'FLARE' | 'ROLLOUT' | 'STOPPED' | 'GO_AROUND'
  | 'REJECTED' | 'EO_GLIDE' | 'EO_FINAL' | 'FORCED_LANDING';

export const ENGINE_OUT_PHASES: Phase[] = ['REJECTED', 'EO_GLIDE', 'EO_FINAL', 'FORCED_LANDING'];

/** Flight mode annunciator: what each channel is doing, as a pilot would read it. */
export interface Fma { thr: string; vert: string; lat: string; }

export interface ModeEvent { t: number; phase: Phase | 'DESCENT' | 'ENGINE_FAILURE'; h: number; V: number; x: number; y: number; note?: string; }

/** Stabilised-approach criteria, checked below the gate height on final. */
export interface StabilityGates {
  gateHeight: number; speedLow: number; speedHigh: number; glidepathDeg: number;
  lateral: number; sinkRate: number; bank: number; persistence: number;
}
export const DEFAULT_GATES: StabilityGates = {
  gateHeight: 300 * FT, speedLow: 5 * KT, speedHigh: 10 * KT, glidepathDeg: 0.7,
  lateral: 30, sinkRate: (1000 * FT) / 60, bank: 15 * D2R, persistence: 2
};

export class StabilityMonitor {
  private unstableFor = 0;
  constructor(readonly gates: StabilityGates, readonly runway: Runway, readonly Vapp: number) {}
  reset(): void { this.unstableFor = 0; }
  /** Returns a reason once a criterion has been violated for `persistence` seconds. */
  check(ac: Aircraft, dt: number): string | null {
    const g = this.gates;
    if (ac.h > g.gateHeight) { this.unstableFor = 0; return null; }
    const dev = this.runway.glidepathDeviationDeg(ac.x, ac.h);
    let why: string | null = null;
    if (ac.V < this.Vapp - g.speedLow) why = 'Slow on approach';
    else if (ac.V > this.Vapp + g.speedHigh) why = 'Fast on approach';
    else if (ac.h > 60 * FT && Math.abs(dev) > g.glidepathDeg) why = dev > 0 ? 'High on glidepath' : 'Low on glidepath';
    else if (Math.abs(ac.y) > g.lateral) why = 'Off centreline';
    else if (ac.verticalSpeed < -g.sinkRate) why = 'Sink rate';
    else if (Math.abs(ac.phi) > g.bank) why = 'Excessive bank';
    this.unstableFor = why ? this.unstableFor + dt : 0;
    return this.unstableFor > g.persistence ? why : null;
  }
}

/**
 * Mode manager plus the outer loops it switches between.
 * Each phase sets the targets; the controllers turn them into attitude/throttle commands;
 * the aircraft's CommandShaper rate-limits whatever comes out.
 */
export class Autopilot {
  phase: Phase = 'TAKEOFF_ROLL';
  fma: Fma = { thr: 'TOGA', vert: 'TO', lat: 'RWY' };
  descending = false;
  goArounds = 0;
  goAroundReason = '';
  goAroundRequested = false;
  touchdownTime = 0;
  /** Set when the engine fails: what was decided, why, and where. */
  engineOut: { plan: EngineOutPlan; why: string; t: number; x: number; y: number; h: number; track: number } | null = null;
  /** Landing direction chosen for a power-off return. */
  get eoEnd(): RunwayEnd { return this.glide?.end ?? this.planner.ends[0]; }
  glide: GlideAssessment | null = null;
  readonly planner: GlidePlanner;
  readonly glideSpeed = new SpeedOnPitch(0.02, 0.01, -0.35, 0.1);
  private eoFlaps = 0;
  private passedBaseKey = false;
  private slipSide = 0;
  private lastAssess = 0;
  private flareK = 0.28;

  readonly guidance = new TrackGuidance();
  readonly path = new FlightPathController();
  readonly autoThrottle = new AutoThrottle();
  readonly climb = new SpeedOnPitch();
  readonly stability: StabilityMonitor;

  private cmd: Commands = { theta: 0, phi: 0, beta: 0, thr: 1, flaps: 10, brake: 0, allowLiftoff: false, noTouchdown: false };
  private gaStart = 0;
  private gaGamma = 0;
  private t = 0;

  constructor(readonly circuit: Circuit, readonly onEvent: (e: ModeEvent) => void, gates = DEFAULT_GATES) {
    this.stability = new StabilityMonitor(gates, circuit.runway, circuit.spec.Vapp);
    this.planner = new GlidePlanner(circuit.runway, DEFAULT_EO);
  }

  private log(ac: Aircraft, phase: ModeEvent['phase'], note?: string): void {
    this.onEvent({ t: this.t, phase, h: ac.h, V: ac.V, x: ac.x, y: ac.y, note });
  }
  setPhase(ac: Aircraft, p: Phase): void {
    if (this.phase === p) return;
    // flare law: sink rate shrinks linearly with height from whatever it was at entry (≈0.28/s on a normal approach)
    if (p === 'FLARE' && !ac.onGround) this.flareK = clamp((-ac.verticalSpeed - 0.4) / Math.max(ac.h, 0.5), 0.28, 2);
    this.phase = p; this.log(ac, p);
  }

  /**
   * Engine failure decision:
   * on the ground → reject; low after take-off → land ahead (no turn-back);
   * otherwise return to the runway if the glide planner says it is reachable with margin, else land ahead.
   */
  failEngine(ac: Aircraft, wind: Wind): boolean {
    if (this.engineOut || this.phase === 'STOPPED' || this.phase === 'ROLLOUT') return false;
    ac.engineFailed = true;
    ac.shaper.rates = { ...ac.shaper.rates, theta: this.planner.spec.pitchRate };
    this.goAroundRequested = false;
    const E = this.planner.spec, track = ac.track(wind);
    this.glide = ac.onGround ? null : this.planner.plan(ac, wind);
    let plan: EngineOutPlan, why: string;
    if (ac.onGround) { plan = 'REJECT'; why = 'On the runway: reject take-off'; }
    else if (this.phase === 'FLARE') { plan = 'RETURN'; why = 'Already in the flare'; }
    else if ((this.phase === 'CLIMB' || this.phase === 'GO_AROUND') && ac.h < E.minTurnbackHeight) { plan = 'LAND_AHEAD'; why = 'Below 700 ft: no turn-back'; }
    else if (this.glide!.reachable) { plan = 'RETURN'; why = `Runway ${this.glide!.end.name} within glide range`; }
    else { plan = 'LAND_AHEAD'; why = 'Both runway directions beyond glide range'; }
    this.engineOut = { plan, why, t: this.t, x: ac.x, y: ac.y, h: ac.h, track };
    this.log(ac, 'ENGINE_FAILURE', why);
    // bumpless-ish: start the glide loop at the expected best-glide path angle; the pitch slew does the rest
    this.glideSpeed.engage(-1 / ac.glideRatio(E.Vbg, 0));
    this.eoFlaps = 0; this.lastAssess = this.t; this.passedBaseKey = false;
    if (plan === 'REJECT') this.setPhase(ac, 'REJECTED');
    else if (plan === 'LAND_AHEAD') this.setPhase(ac, 'FORCED_LANDING');
    else if (this.phase === 'FLARE') { /* keep flaring */ }
    else if (Math.abs(ac.y) < 100 && Math.abs(wrapPi(track - this.eoEnd.heading)) < 30 * D2R && this.planner.toAim(ac.x, this.eoEnd) > 0) {
      this.eoFlaps = Math.round(ac.flapPos / 10) * 10; this.setPhase(ac, 'EO_FINAL');
    } else this.setPhase(ac, 'EO_GLIDE');
    return true;
  }

  private turnRadius(ac: Aircraft, wind: Wind): number {
    const gv = ac.groundVelocity(wind), Vg = Math.hypot(gv.x, gv.y);
    return (Vg * Vg) / (G * Math.tan(this.circuit.spec.bankMax));
  }
  /** Vertical speed to follow a path whose height is `target` here: feed-forward along the track plus correction. */
  private pathVS(ac: Aircraft, wind: Wind, target: number): number {
    const gv = ac.groundVelocity(wind), Vg = Math.hypot(gv.x, gv.y);
    return clamp(-Vg * Math.tan(this.circuit.spec.approachAngle) + 0.2 * (target - ac.h), -5.5, 0.5);
  }

  private glidePitch(ac: Aircraft, V: number, dt: number): number {
    return this.path.gamma(ac, this.glideSpeed.update(ac, V, dt, -0.35), dt);
  }
  private flareIfLow(ac: Aircraft): void {
    // a power-off glide is steep: start the round-out higher, about 3 s of sink above the ground
    if (ac.h < Math.max(this.circuit.spec.flareHeight, 3.0 * -ac.verticalSpeed)) this.setPhase(ac, 'FLARE');
  }

  private startGoAround(ac: Aircraft, why: string): void {
    this.goArounds++; this.goAroundReason = why; this.goAroundRequested = false;
    this.stability.reset();
    this.circuit.crosswindX = null; this.descending = false;
    this.gaStart = this.t; this.gaGamma = ac.gamma;   // flight-path ramp starts where we are: bumpless
    this.phase = 'GO_AROUND'; this.log(ac, 'GO_AROUND', why);
  }

  private legBank(ac: Aircraft, wind: Wind, leg: Leg, limit: number) {
    const [A, B] = this.circuit.legEnds(leg);
    const g = this.guidance.follow(ac, A, B);
    return { ...g, bank: this.guidance.bankFor(ac, wind, g.desiredTrack, limit) };
  }
  private turnLead(ac: Aircraft, wind: Wind): number {
    return this.guidance.turnLead(ac, wind, this.circuit.spec.bankMax, ac.shaper.rates.phi);
  }
  private altHoldWithSpeed(ac: Aircraft, V: number, dt: number): void {
    if (this.fma.vert !== 'ALT') { this.fma.vert = 'ALT'; this.autoThrottle.engage(ac.thr); }
    this.cmd.theta = this.path.altitude(ac, this.circuit.spec.patternAlt, dt);
    this.cmd.thr = this.autoThrottle.update(ac, V, dt); this.fma.thr = 'SPD';
  }

  /**
   * Crab → wing-low: rudder swings the nose onto the runway heading; the side force that creates
   * is cancelled by banking into wind (feed-forward), and the track loop trims what is left.
   */
  private align(ac: Aircraft, wind: Wind, c: Commands, heading: number, line?: [Point, Point]): void {
    const lim = this.circuit.spec.alignBankMax;
    c.beta = clamp(wrapPi(heading - ac.psi), -ac.spec.maxSideslip, ac.spec.maxSideslip);
    // on a runway: hold its centreline; in a field: hold the track
    const trackBank = line ? this.guidance.bankFor(ac, wind, this.guidance.follow(ac, line[0], line[1]).desiredTrack, 6 * D2R)
      : this.guidance.bankFor(ac, wind, heading, 6 * D2R);
    c.phi = clamp(ac.wingLowBank() + trackBank, -lim, lim);
    this.fma.lat = 'ALIGN';
  }
  private get landingAhead(): boolean { return this.engineOut?.plan === 'LAND_AHEAD'; }
  /** Centreline for a landing direction, from far out on the approach to the far end. */
  private endLine(end: RunwayEnd): [Point, Point] {
    return [[end.threshold - end.dir * 5000, 0], [end.threshold + end.dir * this.circuit.runway.length, 0]];
  }
  private lineBank(ac: Aircraft, wind: Wind, end: RunwayEnd, limit: number): number {
    return this.guidance.bankFor(ac, wind, this.guidance.follow(ac, ...this.endLine(end)).desiredTrack, limit);
  }

  update(ac: Aircraft, wind: Wind, t: number, dt: number): Commands {
    this.t = t;
    const c = this.cmd, S = this.circuit.spec, rwy = this.circuit.runway;
    c.allowLiftoff = false; c.noTouchdown = false; c.beta = 0;   // coordinated unless aligning

    switch (this.phase) {
      case 'TAKEOFF_ROLL': {
        c.thr = 1; c.flaps = 10; c.theta = 0; c.brake = 0; c.phi = 0;
        this.fma = { thr: 'TOGA', vert: 'TO', lat: 'RWY' };
        if (ac.V >= S.Vr) this.setPhase(ac, 'ROTATE');
        break;
      }
      case 'ROTATE': {
        c.thr = 1; c.theta = 8 * D2R; c.phi = 0; c.allowLiftoff = true; c.noTouchdown = true; this.fma.vert = 'ROT';
        if (!ac.onGround) { this.climb.engage(0.06); this.path.gammaPI.i = 0; this.setPhase(ac, 'CLIMB'); }
        break;
      }
      case 'CLIMB':
      case 'CROSSWIND': {
        c.noTouchdown = this.phase === 'CLIMB';
        if (ac.h > S.flapsUpAlt) c.flaps = 0;
        const capture = ac.h > S.patternAlt - Math.max(15, ac.verticalSpeed * 6);
        if (this.fma.vert === 'ALT' || capture) this.altHoldWithSpeed(ac, S.Vdownwind, dt);
        else {
          this.fma.vert = 'PITCH-SPD'; this.fma.thr = 'TOGA'; c.thr = 1;
          const gMin = ac.h < 30 ? 0.05 : -0.03;                // never pitch down near the ground
          c.theta = this.path.gamma(ac, this.climb.update(ac, S.Vy, dt, gMin), dt);
        }
        if (this.phase === 'CLIMB') {
          this.fma.lat = 'TRK 090';
          c.phi = this.legBank(ac, wind, 'UPWIND', ac.h < 60 ? 10 * D2R : S.bankMax).bank;
          if (ac.h >= S.crosswindTurnAlt) {
            const lead = this.turnLead(ac, wind);
            if (this.circuit.crosswindX === null) this.circuit.crosswindX = Math.max(ac.x + lead, rwy.length + S.crosswindClearance);
            if (ac.x >= this.circuit.crosswindX - lead) this.setPhase(ac, 'CROSSWIND');
          }
        } else {
          this.fma.lat = 'LEG XWND';
          const g = this.legBank(ac, wind, 'CROSSWIND', S.bankMax);
          c.phi = g.bank;
          if (g.remaining < this.turnLead(ac, wind)) this.setPhase(ac, 'DOWNWIND');
        }
        break;
      }
      case 'DOWNWIND': {
        this.fma.lat = 'LEG DNWD';
        const g = this.legBank(ac, wind, 'DOWNWIND', S.bankMax);
        c.phi = g.bank;
        const profH = this.circuit.profileHeight('DOWNWIND', ac.x, ac.y, this.turnRadius(ac, wind));
        const startDescent = S.profileDescent ? profH < S.patternAlt - 3 : ac.x < 0;
        if (startDescent && !this.descending) { this.descending = true; this.log(ac, 'DESCENT'); }
        if (this.descending) {
          c.flaps = 10; this.fma.vert = S.profileDescent ? 'PATH' : 'V/S'; this.fma.thr = 'SPD';
          c.theta = S.profileDescent ? this.path.verticalSpeed(ac, this.pathVS(ac, wind, profH), dt)
            : this.path.verticalSpeed(ac, clamp(0.15 * (S.baseFloor - ac.h), S.descentVS, 0.5), dt);
          c.thr = this.autoThrottle.update(ac, S.Vdescent, dt);
        } else this.altHoldWithSpeed(ac, S.Vdownwind, dt);
        if (g.remaining < this.turnLead(ac, wind)) this.setPhase(ac, 'BASE');
        break;
      }
      case 'BASE': {
        this.fma.lat = 'LEG BASE'; this.fma.vert = 'V/S'; c.flaps = 20;
        const g = this.legBank(ac, wind, 'BASE', S.bankMax);
        c.phi = g.bank;
        if (S.profileDescent) {
          this.fma.vert = 'PATH';
          c.theta = this.path.verticalSpeed(ac, this.pathVS(ac, wind, this.circuit.profileHeight('BASE', ac.x, ac.y, this.turnRadius(ac, wind))), dt);
        } else {
          const floor = Math.min(rwy.glidepathHeight(ac.x), S.baseFloor);
          c.theta = this.path.verticalSpeed(ac, clamp(0.15 * (floor - ac.h), S.descentVS, 0.5), dt);
        }
        c.thr = this.autoThrottle.update(ac, S.Vbase, dt);
        if (this.goAroundRequested) { this.startGoAround(ac, 'Pilot request'); break; }
        if (g.remaining < this.turnLead(ac, wind)) this.setPhase(ac, 'FINAL');
        break;
      }
      case 'FINAL': {
        this.fma.lat = 'LOC'; this.fma.vert = 'G/S'; c.flaps = 30;
        if (ac.h < S.alignHeight) this.align(ac, wind, c, 0, this.circuit.legEnds('FINAL'));
        else c.phi = this.legBank(ac, wind, 'FINAL', ac.h < 30 ? 10 * D2R : S.bankMax).bank;
        const gvx = ac.groundVelocity(wind).x;
        const vs = clamp(-gvx * Math.tan(rwy.glideslope) + 0.25 * (rwy.glidepathHeight(ac.x) - ac.h), -5.5, 1);
        c.theta = this.path.verticalSpeed(ac, vs, dt);
        c.thr = this.autoThrottle.update(ac, S.Vapp, dt);
        const why = this.goAroundRequested ? 'Pilot request' : this.stability.check(ac, dt);
        if (why) { this.startGoAround(ac, why); break; }
        if (ac.h < Math.max(S.flareHeight, 3.3 * -ac.verticalSpeed)) this.setPhase(ac, 'FLARE');
        break;
      }
      case 'FLARE': {
        this.fma.vert = 'FLARE'; this.fma.thr = this.engineOut ? 'ENG OUT' : 'IDLE'; c.thr = 0;
        if (this.landingAhead) this.align(ac, wind, c, this.engineOut!.track);
        else if (this.engineOut) this.align(ac, wind, c, this.eoEnd.heading, this.endLine(this.eoEnd));
        else this.align(ac, wind, c, 0, this.circuit.legEnds('FINAL'));
        c.theta = this.path.verticalSpeed(ac, -(0.4 + this.flareK * ac.h), dt);   // sink rate shrinks with height
        if (ac.onGround) { this.touchdownTime = t; this.setPhase(ac, 'ROLLOUT'); break; }
        if (this.engineOut) break;                             // no power: committed to land
        if (this.goAroundRequested) this.startGoAround(ac, 'Pilot request');
        else if (ac.x > rwy.length / 3 + 50) this.startGoAround(ac, 'Long landing');
        break;
      }
      case 'ROLLOUT': {
        this.fma = { thr: this.engineOut ? 'ENG OUT' : 'IDLE', vert: 'ROLLOUT', lat: this.engineOut?.plan === 'LAND_AHEAD' ? 'FIELD' : 'RWY' };
        c.thr = 0; c.theta = 0; c.phi = 0; c.flaps = 0;
        c.brake = t - this.touchdownTime > 2 ? 1 : 0;
        if (ac.Vg < 0.5) { this.setPhase(ac, 'STOPPED'); this.fma = { thr: '—', vert: '—', lat: '—' }; }
        break;
      }
      case 'GO_AROUND': {
        // Power and pitch come up together but gently: throttle and pitch are slew-limited by the
        // CommandShaper, and the flight-path target ramps from its current value at 1.5°/s.
        this.fma = { thr: 'TOGA', vert: 'GA', lat: 'TRK 090' };
        c.thr = 1; c.noTouchdown = true;
        this.gaGamma = Math.min(this.gaGamma + 1.5 * D2R * dt, 4 * D2R);
        const slow = S.Vapp - 3 * KT - ac.V;               // speed protection: trade climb for airspeed
        c.theta = this.path.gamma(ac, clamp(this.gaGamma - (slow > 0 ? 0.02 * slow : 0), -0.06, 0.12), dt);
        const vs = ac.verticalSpeed;
        if (c.flaps > 20) c.flaps = 20;                    // first stage at once, then clean up in steps
        if (c.flaps > 10 && vs > 1 && ac.V > 63 * KT && t - this.gaStart > 4) c.flaps = 10;
        c.phi = this.legBank(ac, wind, 'UPWIND', 10 * D2R).bank;
        if (vs > 1.5 && ac.h > 150 * FT && ac.flapPos <= 10.5) {
          this.climb.engage(ac.gamma); this.fma.vert = 'PITCH-SPD'; this.setPhase(ac, 'CLIMB');
        }
        break;
      }
      case 'REJECTED': {
        this.fma = { thr: 'ENG OUT', vert: 'REJECT', lat: 'RWY' };
        c.thr = 0; c.theta = 0; c.phi = 0; c.brake = 1;
        if (ac.Vg < 0.5) { this.setPhase(ac, 'STOPPED'); this.fma = { thr: '—', vert: '—', lat: '—' }; }
        break;
      }
      case 'EO_GLIDE': {
        // Best glide, clean: direct to a base key point, down a short base leg, then turn final at the intercept.
        const E = this.planner.spec, end = this.eoEnd, wp = this.planner.waypoints(ac, this.glide!.ipDistance, end);
        this.fma = { thr: 'ENG OUT', vert: 'GLIDE', lat: `DIR IP ${end.name}` };
        c.thr = 0; c.flaps = 0;
        c.theta = this.glidePitch(ac, E.Vbg, dt);
        const gv = ac.groundVelocity(wind), Vg = Math.hypot(gv.x, gv.y);
        const R = (Vg * Vg) / (G * Math.tan(S.bankMax)), rollIn = Vg * (S.bankMax / ac.shaper.rates.phi) * 0.5;
        if (wp.baseKey && !this.passedBaseKey) {
          const [bx, by] = wp.baseKey, brg = Math.atan2(by - ac.y, bx - ac.x), dist = Math.hypot(bx - ac.x, by - ac.y);
          const baseBrg = Math.atan2(wp.ip[1] - by, wp.ip[0] - bx);
          const lead = R * Math.tan(Math.min(Math.abs(wrapPi(baseBrg - brg)), 2.5) / 2) + rollIn;
          this.fma.lat = 'DIR KEY';
          c.phi = this.guidance.bankFor(ac, wind, brg, S.bankMax);
          if (dist < lead || end.dir * (ac.x - bx) < 0) this.passedBaseKey = true;
        } else if (wp.baseKey) {
          const g = this.guidance.follow(ac, wp.baseKey, wp.ip);
          this.fma.lat = 'BASE';
          c.phi = this.guidance.bankFor(ac, wind, g.desiredTrack, S.bankMax);
          if (g.remaining < R + rollIn) { this.setPhase(ac, 'EO_FINAL'); break; }
        } else {
          const [ipx, ipy] = wp.ip, brg = Math.atan2(ipy - ac.y, ipx - ac.x), dist = Math.hypot(ipx - ac.x, ipy - ac.y);
          const lead = R * Math.tan(Math.min(Math.abs(wrapPi(brg - end.heading)), 2.5) / 2) + rollIn;
          c.phi = this.guidance.bankFor(ac, wind, brg, S.bankMax);
          if (dist < lead || (end.dir * (ac.x - ipx) > 0 && Math.abs(ac.y) < 150)) { this.setPhase(ac, 'EO_FINAL'); break; }
        }
        if (t - this.lastAssess > 1) {
          // re-plan each second: take the longest reachable final, keep the current one if only that works,
          // and give up on the runway only when nothing reaches
          this.lastAssess = t;
          const fresh = this.planner.plan(ac, wind, end), keep = this.planner.assess(ac, wind, this.glide!.ipDistance, end);
          this.glide = fresh.reachable ? fresh : keep.available >= keep.required ? keep : fresh;
          if (this.glide.available < this.glide.required) {         // lost the energy race: take what is ahead
            this.engineOut!.plan = 'LAND_AHEAD'; this.engineOut!.track = ac.track(wind);
            this.log(ac, 'FORCED_LANDING', 'Runway no longer reachable'); this.phase = 'FORCED_LANDING';
          }
        }
        break;
      }
      case 'EO_FINAL': {
        // Energy management on a power-off final: add flap in stages when still high (never retract),
        // then forward-slip if even full flap will not get it down; straighten for touchdown below 60 ft.
        const E = this.planner.spec, end = this.eoEnd, dist = this.planner.toAim(ac.x, end), greq = dist > 30 ? ac.h / dist : Infinity;
        // lined up on the extended centreline? Flap beyond 10° and slipping wait for that, so the turn onto final
        // is flown with the flatter clean glide and does not run out of height before it is lined up
        const aligned = Math.abs(wrapPi(ac.track(wind) - end.heading)) < 15 * D2R && Math.abs(ac.y) < 80;
        const flapCap = aligned || ac.h < 300 * FT ? 30 : 10;
        while (this.eoFlaps < flapCap && greq > this.planner.gradient(ac, wind, this.eoFlaps + 10, end) * 1.05) this.eoFlaps += 10;
        c.thr = 0; c.flaps = this.eoFlaps;
        const V = this.eoFlaps >= 20 ? E.VappEO : E.Vbg;
        c.theta = this.glidePitch(ac, V, dt);
        this.fma = { thr: 'ENG OUT', vert: `GLIDE F${this.eoFlaps}`, lat: `LOC ${end.name}` };
        // only slip once lined up: a slip restricts bank, so the turn onto final must be finished first
        const slip = aligned && this.eoFlaps === 30 && greq > this.planner.gradient(ac, wind, 30, end) * 1.1 && ac.h > S.alignHeight;
        if (ac.h < S.alignHeight) this.align(ac, wind, c, end.heading, this.endLine(end));
        else if (slip) {
          // forward slip: into-wind wing down, opposite rudder, track held on the centreline
          // pick the slip direction once (into wind if there is a crab), so it does not flip back and forth
          const rel = wrapPi(ac.psi - end.heading);
          if (!this.slipSide) this.slipSide = Math.abs(rel) < 1 * D2R ? 1 : Math.sign(rel);
          c.beta = -this.slipSide * 12 * D2R;
          c.phi = clamp(ac.wingLowBank() + this.lineBank(ac, wind, end, 6 * D2R), -12 * D2R, 12 * D2R);
          this.fma.lat = 'SLIP';
        } else c.phi = this.lineBank(ac, wind, end, ac.h < 30 ? 10 * D2R : S.bankMax);
        this.flareIfLow(ac);
        break;
      }
      case 'FORCED_LANDING': {
        // Land ahead: hold the track at failure, best glide clean, landing flap below 300 ft.
        const E = this.planner.spec, eo = this.engineOut!;
        this.fma = { thr: 'ENG OUT', vert: 'GLIDE', lat: 'TRK HOLD' };
        c.thr = 0; c.flaps = ac.h < 300 * FT ? 20 : 0;
        c.theta = this.glidePitch(ac, c.flaps >= 20 ? E.VappEO : E.Vbg, dt);
        if (ac.h < S.alignHeight) this.align(ac, wind, c, eo.track);
        else c.phi = this.guidance.bankFor(ac, wind, eo.track, 15 * D2R);
        this.flareIfLow(ac);
        break;
      }
      case 'STOPPED': c.thr = 0; c.brake = 1; break;
    }
    return c;
  }

  /** Called by the simulation when the wheels touch outside the flare (e.g. a short approach). */
  onUnexpectedTouchdown(ac: Aircraft): void { if (this.phase !== 'FLARE') this.setPhase(ac, 'FLARE'); }
}
