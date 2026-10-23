import { D2R, FT, KT } from './units';

/** Runway 09/27. Threshold of 09 at x = 0, heading 090 along +x, centreline y = 0. */
export class Runway {
  constructor(
    readonly length = 1200,
    readonly width = 30,
    /** Glidepath aim point, metres past the threshold. */
    readonly aimPoint = 250,
    readonly glideslope = 3 * D2R
  ) {}
  get departureEnd(): number { return this.length; }
  /** Height of the glidepath above the runway at along-track position x. */
  glidepathHeight(x: number): number { return Math.max(0, (this.aimPoint - x) * Math.tan(this.glideslope)); }
  /** Angular deviation from the glidepath, degrees (positive = high). */
  glidepathDeviationDeg(x: number, h: number): number {
    const d = Math.max(this.aimPoint - x, 1);
    return (Math.atan2(h, d) - this.glideslope) / D2R;
  }
  contains(x: number, y: number): boolean { return x >= 0 && x <= this.length && Math.abs(y) <= this.width / 2; }
}

/** Left-hand traffic pattern flown to the north of runway 09. */
export interface CircuitSpec {
  downwindOffset: number;   // m north of the centreline
  baseX: number;            // m, along-track position of the base leg (negative = before threshold)
  patternAlt: number; crosswindTurnAlt: number; flapsUpAlt: number;
  Vr: number; Vy: number; Vdownwind: number; Vdescent: number; Vbase: number; Vapp: number;
  bankMax: number; descentVS: number; baseFloor: number; flareHeight: number;
  /** Height below which the crab is taken out (wing-low), and the bank allowed while doing it. */
  alignHeight: number; alignBankMax: number;
  /** Minimum distance beyond the departure end before turning crosswind. */
  crosswindClearance: number;
  /** Final approach path angle. */
  approachAngle: number;
  /**
   * true: hold pattern altitude on downwind and descend only when the approach path (at approachAngle,
   * measured along the remaining downwind + base + final) meets it. false: start a fixed descent abeam the threshold.
   */
  profileDescent: boolean;
  name: string;
}

export const STANDARD_CIRCUIT: CircuitSpec = {
  downwindOffset: 1200, baseX: -2600,
  patternAlt: 1000 * FT, crosswindTurnAlt: 500 * FT, flapsUpAlt: 300 * FT,
  Vr: 55 * KT, Vy: 75 * KT, Vdownwind: 90 * KT, Vdescent: 75 * KT, Vbase: 70 * KT, Vapp: 65 * KT,
  bankMax: 25 * D2R, descentVS: -2.5, baseFloor: 150, flareHeight: 4.8,
  alignHeight: 60 * FT, alignBankMax: 10 * D2R,
  crosswindClearance: 400,
  approachAngle: 3 * D2R, profileDescent: false, name: 'Standard'
};

/**
 * High circuit: tighter pattern, pattern altitude held until the descent point, then a 5.5° path flown
 * along base and final. 5.5° is close to the clean power-off glide angle (~5.7°), so the runway stays
 * within gliding reach for much more of the circuit, while still needing a little power on the approach.
 */
export const HIGH_CIRCUIT: CircuitSpec = {
  ...STANDARD_CIRCUIT,
  downwindOffset: 1000, baseX: -1500,
  approachAngle: 5.5 * D2R, profileDescent: true, name: 'High'
};

export type Leg = 'UPWIND' | 'CROSSWIND' | 'DOWNWIND' | 'BASE' | 'FINAL';
export type Point = [number, number];

export class Circuit {
  /** Set once the aircraft decides where to turn crosswind; null until then. */
  crosswindX: number | null = null;
  constructor(readonly spec: CircuitSpec, readonly runway: Runway) {}

  legEnds(leg: Leg, nominalCrossX = 2300): [Point, Point] {
    const cx = this.crosswindX ?? nominalCrossX, s = this.spec, dy = s.downwindOffset;
    switch (leg) {
      case 'UPWIND': return [[0, 0], [20000, 0]];
      case 'CROSSWIND': return [[cx, 0], [cx, dy]];
      case 'DOWNWIND': return [[cx, dy], [s.baseX, dy]];
      case 'BASE': return [[s.baseX, dy], [s.baseX, 0]];
      case 'FINAL': return [[s.baseX, 0], [this.runway.length, 0]];
    }
  }

  /**
   * Distance still to fly along the circuit to the aim point, from a point on the downwind or base leg.
   * Used to place the aircraft on the approach profile before it reaches final.
   */
  remainingToAim(leg: 'DOWNWIND' | 'BASE', x: number, y: number, turnRadius = 0): number {
    const s = this.spec, final = this.runway.aimPoint - s.baseX;
    // each 90° turn still ahead cuts its corner by (2 − π/2)·R
    const cut = (2 - Math.PI / 2) * turnRadius;
    return leg === 'DOWNWIND' ? Math.max(0, x - s.baseX) + s.downwindOffset + final - 2 * cut
      : Math.max(0, y) + final - (y > turnRadius ? cut : 0);
  }
  profileHeight(leg: 'DOWNWIND' | 'BASE', x: number, y: number, turnRadius = 0): number {
    return Math.max(0, this.remainingToAim(leg, x, y, turnRadius)) * Math.tan(this.spec.approachAngle);
  }

  /** Planned route for display, from brake release to the aim point. */
  plannedRoute(): Point[] {
    const cx = this.crosswindX ?? 2300, s = this.spec;
    return [[0, 0], [cx, 0], [cx, s.downwindOffset], [s.baseX, s.downwindOffset], [s.baseX, 0], [this.runway.aimPoint, 0]];
  }
}
