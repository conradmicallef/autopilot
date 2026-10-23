import { Aircraft } from './aircraft';
import { Point, Runway } from './airfield';
import { Wind } from './atmosphere';
import { D2R, FT, G, KT, wrapPi } from './units';

/** What the autopilot decides to do when the engine stops. */
export type EngineOutPlan = 'REJECT' | 'LAND_AHEAD' | 'RETURN';

export interface EngineOutSpec {
  /** Best-glide speed, clean. */
  Vbg: number;
  /** Power-off approach speed once flaps are out. */
  VappEO: number;
  /** Below this height after take-off, never turn back: land ahead. */
  minTurnbackHeight: number;
  /** Range of intercept points on the extended centreline, metres before the threshold. */
  ipMin: number; ipMax: number;
  /** Lateral offset of the base key point from the centreline, so the turn onto final is about 90°. */
  baseKeyOffset: number;
  /** Power-off aim point, metres past the threshold: aim long, since height can be lost but not gained. */
  aimPoint: number;
  /** Required height is multiplied by this, plus `marginHeight`, before calling the runway reachable. */
  marginFactor: number;
  marginHeight: number;
  /** Pitch slew while gliding: faster than normal, because airspeed must be held after the failure. */
  pitchRate: number;
}

export const DEFAULT_EO: EngineOutSpec = {
  Vbg: 68 * KT, VappEO: 63 * KT, minTurnbackHeight: 700 * FT, ipMin: 500, ipMax: 3000, aimPoint: 300, baseKeyOffset: 450,
  marginFactor: 1.1, marginHeight: 20, pitchRate: 5 * D2R
};

/** One landing direction of the runway. 09 lands toward +x from x = 0; 27 lands toward −x from x = length. */
export interface RunwayEnd {
  name: '09' | '27';
  heading: number;      // track angle, rad
  threshold: number;    // x of the landing threshold
  dir: 1 | -1;          // +1 when landing toward +x
}
export function runwayEnds(runway: Runway): RunwayEnd[] {
  return [
    { name: '09', heading: 0, threshold: 0, dir: 1 },
    { name: '27', heading: Math.PI, threshold: runway.length, dir: -1 }
  ];
}

export interface GlideAssessment {
  reachable: boolean;
  /** Height the route needs, before margins, metres. */
  required: number;
  /** Height available above the runway, metres. */
  available: number;
  route: Point[];
  /** Intercept distance the route uses. */
  ipDistance: number;
  end: RunwayEnd;
  /** Height left over after margins (negative = short). */
  surplus: number;
}

/** Energy planning for a power-off return: route length, wind, turns and glide ratio → required height. */
export class GlidePlanner {
  constructor(readonly runway: Runway, readonly spec: EngineOutSpec = DEFAULT_EO) { this.ends = runwayEnds(runway); }

  readonly ends: RunwayEnd[];

  ipAt(distance: number, end: RunwayEnd): Point { return [end.threshold - end.dir * distance, 0]; }
  aimFor(end: RunwayEnd): Point { return [end.threshold + end.dir * this.spec.aimPoint, 0]; }
  /** Along-approach distance still to go to the aim point (positive before it). */
  toAim(x: number, end: RunwayEnd): number { return end.dir * (this.aimFor(end)[0] - x); }

  /**
   * Waypoints for a power-off return: a base key point abeam the intercept (on the aircraft's side), then the
   * intercept. The base key is skipped when the aircraft is already out beyond the intercept point.
   */
  waypoints(ac: Aircraft, ipDistance: number, end: RunwayEnd): { baseKey: Point | null; ip: Point } {
    const ip = this.ipAt(ipDistance, end);
    if (end.dir * (ac.x - ip[0]) < 200) return { baseKey: null, ip };   // already out beyond the intercept
    const side = ac.y >= 0 ? 1 : -1;
    return { baseKey: [ip[0], side * this.spec.baseKeyOffset], ip };
  }

  /**
   * Choose the longest final the energy allows, so surplus height is spent on a long straight final
   * instead of arriving high: scan intercept points from far to near (50 m steps) and take the first
   * that is reachable with margin. Returns the closest-in attempt (unreachable) when none is.
   */
  planFor(ac: Aircraft, wind: Wind, end: RunwayEnd): GlideAssessment {
    let last: GlideAssessment | null = null;
    for (let d = this.spec.ipMax; d >= this.spec.ipMin; d -= 50) {
      last = this.assess(ac, wind, d, end);
      if (last.reachable) return last;
    }
    return last!;
  }

  /**
   * Plan for both landing directions and pick one: the preferred end if it still works (no switching back and
   * forth mid-glide), otherwise whichever reachable end leaves the most height in hand; if neither is reachable,
   * the one that comes closest.
   */
  plan(ac: Aircraft, wind: Wind, prefer?: RunwayEnd): GlideAssessment {
    const options = this.ends.map(e => this.planFor(ac, wind, e));
    const kept = prefer && options.find(o => o.end.name === prefer.name && o.reachable);
    if (kept) return kept;
    return options.sort((a, b) => Number(b.reachable) - Number(a.reachable) || b.surplus - a.surplus)[0];
  }

  /** Still-air reach in metres from the current height (clean, best glide). */
  stillAirReach(ac: Aircraft): number { return ac.h * ac.glideRatio(this.spec.Vbg, 0); }

  assess(ac: Aircraft, wind: Wind, ipDistance: number, end: RunwayEnd): GlideAssessment {
    const V = this.spec.Vbg, LD = ac.glideRatio(V, 0), aim = this.aimFor(end);
    const wp = this.waypoints(ac, ipDistance, end);
    const route: Point[] = wp.baseKey ? [[ac.x, ac.y], wp.baseKey, wp.ip, aim] : [[ac.x, ac.y], wp.ip, aim];
    // Turns. The first one (from the current track) adds roughly 0.3·R per radian. At each waypoint the turn
    // cuts the corner: the path is shorter by R·(2·tan(Δ/2) − Δ), and the arc is flown ~10% less efficiently.
    const R = (V * V) / (G * Math.tan(25 * D2R));
    const gv = ac.groundVelocity(wind);
    let track = Math.atan2(gv.y, gv.x), required = 0;
    for (let i = 1; i < route.length; i++) {
      const [ax, ay] = route[i - 1], [bx, by] = route[i];
      const len = Math.hypot(bx - ax, by - ay);
      if (len < 1) continue;
      const brg = Math.atan2(by - ay, bx - ax), turn = Math.abs(wrapPi(brg - track));
      const along = wind.along(brg), groundLD = LD * Math.max(0.3, (V + along) / V);
      const t = Math.min(turn, 2.5);
      const turnAdj = i === 1 ? R * turn * 0.3 : R * t * 1.1 - 2 * R * Math.tan(t / 2);
      required += Math.max(0, len + turnAdj) / groundLD;
      track = brg;
    }
    // speed above best glide is energy too: it can be traded for height
    const available = ac.h + Math.max(0, ac.V * ac.V - V * V) / (2 * G);
    const surplus = available - (required * this.spec.marginFactor + this.spec.marginHeight);
    return { reachable: surplus >= 0, required, available, route, ipDistance, end, surplus };
  }

  /** Ground glide gradient (height lost per metre over the ground) for a flap setting, along the landing direction. */
  gradient(ac: Aircraft, wind: Wind, flapPos: number, end: RunwayEnd): number {
    const V = flapPos >= 20 ? this.spec.VappEO : this.spec.Vbg;
    return 1 / (ac.glideRatio(V, flapPos) * Math.max(0.3, (V + wind.along(end.heading)) / V));
  }
}
