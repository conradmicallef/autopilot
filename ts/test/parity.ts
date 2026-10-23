import { HIGH_CIRCUIT, STANDARD_CIRCUIT } from '../src/airfield';
import { Simulation } from '../src/simulation';
import { D2R, FT, KT } from '../src/units';
type W = { fromDeg: number; speedKt: number; gusts: boolean };
const w = (fromDeg: number, speedKt: number, gusts: boolean): W => ({ fromDeg, speedKt, gusts });
const normal: [W, number, string | null][] = [
  [w(270, 0, false), 1, null], [w(270, 8, false), 1, null], [w(300, 15, false), 1, null], [w(360, 15, false), 1, null],
  [w(180, 15, false), 1, null], [w(360, 20, false), 1, null], [w(200, 12, false), 1, null], [w(220, 12, true), 7, null],
  [w(90, 6, false), 1, null], [w(300, 20, true), 3, null], [w(360, 25, true), 11, null],
  [w(270, 8, false), 5, 'final'], [w(270, 8, false), 5, 'flare'], [w(270, 8, false), 5, 'base']];
const trig: [string, (s: Simulation) => boolean][] = [
  ['roll 40kt', s => s.phase === 'TAKEOFF_ROLL' && s.aircraft.V > 40 * KT],
  ['climb 200ft', s => s.phase === 'CLIMB' && s.aircraft.h > 200 * FT],
  ['climb 450ft', s => s.phase === 'CLIMB' && s.aircraft.h > 450 * FT],
  ['crosswind', s => s.phase === 'CROSSWIND' && s.aircraft.h > 700 * FT],
  ['downwind early', s => s.phase === 'DOWNWIND' && s.aircraft.x < 1800],
  ['downwind mid', s => s.phase === 'DOWNWIND' && s.aircraft.x < 1000],
  ['abeam thr', s => s.phase === 'DOWNWIND' && s.aircraft.x < 0],
  ['base', s => s.phase === 'BASE' && s.aircraft.y < 700],
  ['final 400ft', s => s.phase === 'FINAL' && s.aircraft.h < 400 * FT],
  ['final 150ft', s => s.phase === 'FINAL' && s.aircraft.h < 150 * FT]];
const eoW = [w(270, 0, false), w(270, 12, false), w(360, 12, true)];
const f = (x: number, d: number) => Number.isNaN(x) ? 'NaN' : x.toFixed(d);
const line = (name: string, s: Simulation) => {
  const td = s.touchdown;
  console.log([name, f(s.t, 2), s.phase, s.autopilot.goArounds, s.events.length, f(td ? td.x : NaN, 2), f(td ? td.y : NaN, 3),
    f(td ? td.sink / FT : NaN, 3), f(td ? td.crab / D2R : NaN, 3), f(s.aircraft.x, 2), f(s.aircraft.y, 3), f(s.reachFraction * 100, 1)].join(','));
};
console.log('case,t_end,phase,go_arounds,events,td_x,td_y,td_sink_fps,td_crab_deg,stop_x,stop_y,reach_pct');
for (const circuit of [STANDARD_CIRCUIT, HIGH_CIRCUIT]) {
  for (const [wd, seed, ga] of normal) {
    const s = new Simulation({ wind: wd, seed, circuit }); let asked = false;
    while (!s.finished && s.t < 1500) {
      if (ga && !asked) { const p = s.phase, h = s.aircraft.h;
        if ((ga === 'final' && p === 'FINAL' && h < 200 * FT) || (ga === 'flare' && p === 'FLARE' && h < 2) || (ga === 'base' && p === 'BASE')) asked = s.requestGoAround(); }
      s.step();
    }
    line(`${circuit.name}-${wd.fromDeg}-${wd.speedKt}-${wd.gusts ? 1 : 0}-${ga ?? 'none'}`, s);
  }
  for (const wd of eoW) for (const [n, when] of trig) {
    const s = new Simulation({ wind: wd, seed: 4, circuit }); let done = false;
    while (!s.finished && s.t < 1200) { if (!done && when(s)) done = s.failEngine(); s.step(); }
    line(`${circuit.name}-${wd.fromDeg}-${wd.speedKt}-${wd.gusts ? 1 : 0}-EO ${n}`, s);
  }
}
