import { D2R, KT, Vec2 } from './units';

/** Park–Miller PRNG so a run can be replayed from its seed. */
export class Rng {
  constructor(private seed: number) { if (this.seed <= 0) this.seed = 1; }
  next(): number { this.seed = (this.seed * 16807) % 2147483647; return this.seed / 2147483647; }
  gauss(): number { return Math.sqrt(-2 * Math.log(this.next() + 1e-9)) * Math.cos(2 * Math.PI * this.next()); }
}

export interface WindSettings {
  /** Direction the wind blows FROM, degrees true. */
  fromDeg: number;
  speedKt: number;
  gusts: boolean;
}

/** Steady wind plus optional gusts modelled as an Ornstein–Uhlenbeck process on each axis. */
export class Wind {
  settings: WindSettings;
  readonly gustTau = 3;     // s, correlation time
  readonly gustSigma = 1.8; // m/s, standard deviation
  private gust: Vec2 = { x: 0, y: 0 };
  /** Current wind vector in m/s (the air's velocity over the ground). */
  readonly v: Vec2 = { x: 0, y: 0 };

  constructor(settings: WindSettings, private rng: Rng) { this.settings = { ...settings }; }

  update(dt: number): void {
    const { fromDeg, speedKt, gusts } = this.settings;
    const toAngle = (90 - (fromDeg + 180)) * D2R, ws = speedKt * KT;
    if (gusts) {
      const k = this.gustSigma * Math.sqrt((2 * dt) / this.gustTau);
      this.gust.x += (-this.gust.x / this.gustTau) * dt + k * this.rng.gauss();
      this.gust.y += (-this.gust.y / this.gustTau) * dt + k * this.rng.gauss();
    } else {
      this.gust.x *= 0.98; this.gust.y *= 0.98;
    }
    this.v.x = ws * Math.cos(toAngle) + this.gust.x;
    this.v.y = ws * Math.sin(toAngle) + this.gust.y;
  }

  /** Wind component along a track angle (positive = tailwind). */
  along(psi: number): number { return this.v.x * Math.cos(psi) + this.v.y * Math.sin(psi); }
}
