import { Aircraft, AircraftSpec, C172_LIKE } from './aircraft';
import { Circuit, CircuitSpec, Runway, STANDARD_CIRCUIT } from './airfield';
import { Rng, Wind, WindSettings } from './atmosphere';
import { Autopilot, ModeEvent, Phase } from './autopilot';
import { D2R, FT, KT } from './units';

export interface TrackPoint { t: number; x: number; y: number; h: number; V: number; phase: Phase; reach: boolean | null; }
export interface Touchdown { t: number; x: number; y: number; sink: number; V: number; crab: number; onRunway: boolean; }
export interface Criterion { label: string; target: string; value?: string; pass?: boolean | null; info?: boolean; }

export interface SimOptions {
  wind: WindSettings;
  seed?: number;
  aircraft?: AircraftSpec;
  circuit?: CircuitSpec;
  runway?: Runway;
}

/** Owns the world: aircraft, wind, airfield, autopilot, and the flight record. Fixed-step. */
export class Simulation {
  static readonly DT = 0.02;   // 50 Hz
  t = 0;
  readonly aircraft: Aircraft;
  readonly wind: Wind;
  readonly runway: Runway;
  readonly circuit: Circuit;
  readonly autopilot: Autopilot;
  readonly track: TrackPoint[] = [];
  readonly events: ModeEvent[] = [];
  touchdown: Touchdown | null = null;
  maxBank = 0; maxAlt = 0;
  private nextRecord = 0;
  /** Is the runway within power-off gliding reach right now? null on the ground or once the engine has failed. */
  runwayReachable: boolean | null = null;
  /** Which runway direction the glide planner would use right now. */
  reachEnd: string | null = null;
  /** Seconds airborne with the engine running, and how many of those had the runway within glide reach. */
  airTime = 0; reachTime = 0;
  private nextReachCheck = 0;

  constructor(opts: SimOptions) {
    this.aircraft = new Aircraft(opts.aircraft ?? C172_LIKE);
    this.wind = new Wind(opts.wind, new Rng(opts.seed ?? 1));
    const spec = opts.circuit ?? STANDARD_CIRCUIT;
    this.runway = opts.runway ?? new Runway(1200, 30, 250, spec.approachAngle);
    this.circuit = new Circuit(spec, this.runway);
    this.autopilot = new Autopilot(this.circuit, e => this.events.push(e));
    this.events.push({ t: 0, phase: 'TAKEOFF_ROLL', h: 0, V: 0, x: 0, y: 0 });
  }

  get phase(): Phase { return this.autopilot.phase; }
  get finished(): boolean { return this.phase === 'STOPPED'; }

  requestGoAround(): boolean {
    const ok = this.phase === 'BASE' || this.phase === 'FINAL' || this.phase === 'FLARE';
    if (ok) this.autopilot.goAroundRequested = true;
    return ok;
  }

  /** Stop the engine now; the autopilot decides how to handle it. */
  failEngine(): boolean { return this.autopilot.failEngine(this.aircraft, this.wind); }
  get engineFailed(): boolean { return this.aircraft.engineFailed; }

  step(dt = Simulation.DT): void {
    const ac = this.aircraft;
    if (this.finished) { ac.Vg = 0; ac.V = 0; this.t += dt; return; }
    this.wind.update(dt);
    const cmd = this.autopilot.update(ac, this.wind, this.t, dt);
    const contact = ac.step(dt, cmd, this.wind);
    if (contact?.kind === 'touchdown') {
      this.touchdown = { t: this.t, x: ac.x, y: ac.y, sink: contact.sink, V: contact.V, crab: contact.crab, onRunway: this.runway.contains(ac.x, ac.y) };
      this.autopilot.onUnexpectedTouchdown(ac);
    }
    if (!ac.onGround) { this.maxBank = Math.max(this.maxBank, Math.abs(ac.phi)); this.maxAlt = Math.max(this.maxAlt, ac.h); }
    this.updateReach(dt);
    this.t += dt;
    if (this.t >= this.nextRecord) {
      this.track.push({ t: this.t, x: ac.x, y: ac.y, h: ac.h, V: ac.V, phase: this.phase, reach: this.runwayReachable });
      this.nextRecord = this.t + 0.25;
    }
  }

  /**
   * Glide-reach bookkeeping: twice a second, would a power-off return reach the runway from here?
   * Climbing below the turn-back height counts as no (the policy is to land ahead there).
   */
  private updateReach(dt: number): void {
    const ac = this.aircraft, ap = this.autopilot;
    const tracking = !ac.onGround && !ac.engineFailed && this.phase !== 'FLARE';
    if (!tracking) { this.runwayReachable = null; return; }
    if (this.t >= this.nextReachCheck) {
      this.nextReachCheck = this.t + 0.5;
      const climbingLow = (this.phase === 'CLIMB' || this.phase === 'GO_AROUND') && ac.h < ap.planner.spec.minTurnbackHeight;
      const g = ap.planner.plan(ac, this.wind);
      this.runwayReachable = !climbingLow && g.reachable;
      this.reachEnd = this.runwayReachable ? g.end.name : null;
    }
    this.airTime += dt;
    if (this.runwayReachable) this.reachTime += dt;
  }
  get reachFraction(): number { return this.airTime > 0 ? this.reachTime / this.airTime : 0; }

  /** Landing pass/fail check against the proof-of-concept criteria. */
  evaluate(): Criterion[] {
    if (this.autopilot.engineOut) return this.evaluateEngineOut();
    const td = this.touchdown, third = this.runway.length / 3, stopped = this.finished;
    const rows: Criterion[] = [
      { label: 'Touchdown sink rate', target: '< 3 ft/s' },
      { label: 'Touchdown point', target: `first third (0–${Math.round(third)} m)` },
      { label: 'Centerline error', target: '< 5 m' },
      { label: 'Nose aligned at touchdown', target: '< 5° crab' },
      { label: 'Stopped on runway', target: `before ${this.runway.length} m` }
    ];
    if (!td) return rows;
    const sinkFps = td.sink / FT;
    rows[0] = { ...rows[0], value: sinkFps.toFixed(1) + ' ft/s', pass: sinkFps < 3 };
    rows[1] = { ...rows[1], value: Math.round(td.x) + ' m past threshold', pass: td.x >= 0 && td.x <= third };
    rows[2] = { ...rows[2], value: Math.abs(td.y).toFixed(1) + ' m', pass: Math.abs(td.y) < 5 };
    const crabDeg = Math.abs(td.crab) / D2R;
    rows[3] = { ...rows[3], value: crabDeg.toFixed(1) + '° crab', pass: crabDeg < 5 };
    rows[4] = { ...rows[4], value: stopped ? Math.round(this.aircraft.x) + ' m mark' : 'rolling…', pass: stopped ? this.aircraft.x <= this.runway.length : null };
    rows.push({ label: 'Runway within glide reach', target: `${this.circuit.spec.name} circuit, share of time airborne`, value: Math.round(this.reachFraction * 100) + '%', info: true });
    return rows;
  }

  /** Engine-out outcome: was the decision carried out, and was the touchdown survivable and controlled? */
  private evaluateEngineOut(): Criterion[] {
    const eo = this.autopilot.engineOut!, td = this.touchdown, ac = this.aircraft, stopped = this.finished;
    const end = this.autopilot.eoEnd;
    const decided = eo.plan === 'REJECT' ? 'Reject take-off' : eo.plan === 'RETURN' ? `Glide back to runway ${end.name}` : 'Land ahead';
    const rows: Criterion[] = [{ label: 'Decision', target: eo.why, value: `${decided} · failed at ${Math.round(eo.h / FT)} ft`, info: true }];
    if (eo.plan === 'REJECT') {
      rows.push({ label: 'Stopped on runway', target: `before ${this.runway.length} m`,
        value: stopped ? Math.round(ac.x) + ' m mark' : 'braking…', pass: stopped ? ac.x <= this.runway.length : null });
      return rows;
    }
    const sinkFps = td ? td.sink / FT : 0, crabDeg = td ? Math.abs(td.crab) / D2R : 0;
    rows.push({ label: 'Touchdown sink rate', target: '< 3 ft/s', value: td ? sinkFps.toFixed(1) + ' ft/s' : undefined, pass: td ? sinkFps < 3 : null });
    if (eo.plan === 'RETURN') {
      rows.push({ label: 'Touched down on runway', target: 'within the paved surface',
        value: td ? (() => { const past = end.dir * (td.x - end.threshold);
          return td.onRunway ? `${Math.round(past)} m past the ${end.name} threshold` : past < 0 ? Math.round(-past) + ' m short' : 'off the runway'; })() : undefined,
        pass: td ? td.onRunway : null });
      rows.push({ label: 'Stopped on runway', target: `within the ${this.runway.length} m runway`,
        value: stopped ? Math.round(end.dir * (ac.x - end.threshold)) + ' m from the ' + end.name + ' threshold' : td ? 'rolling…' : undefined,
        pass: stopped ? this.runway.contains(ac.x, ac.y) : null });
    } else {
      rows.push({ label: 'Wings level, nose straight', target: '< 5° crab', value: td ? crabDeg.toFixed(1) + '° crab' : undefined, pass: td ? crabDeg < 5 : null });
      const dist = td ? Math.hypot(td.x - eo.x, td.y - eo.y) : 0;
      rows.push({ label: 'Field landing', target: 'touchdown point', value: td ? `${(dist / 1000).toFixed(1)} km from failure, ${Math.round(td.V / KT)} kt` : undefined, info: true });
    }
    return rows;
  }
}
