import { Simulation } from '../src/simulation';
import { FT } from '../src/units';

const cases: [number, number, boolean, number, string?][] = [
  [270, 0, false, 1], [270, 8, false, 1], [300, 15, false, 1], [360, 15, false, 1], [180, 15, false, 1], [360, 20, false, 1], [200, 12, false, 1], [220, 12, true, 7], [90, 6, false, 1],
  [300, 20, true, 3], [360, 25, true, 11], [270, 8, false, 5, 'final'], [270, 8, false, 5, 'flare'], [270, 8, false, 5, 'base']
];
let fails = 0;
for (const [fromDeg, speedKt, gusts, seed, ga] of cases) {
  const sim = new Simulation({ wind: { fromDeg, speedKt, gusts }, seed });
  let asked = false, maxThrRate = 0, prevThr = 0;
  while (!sim.finished && sim.t < 1500) {
    const p = sim.phase, h = sim.aircraft.h;
    if (ga && !asked && ((ga === 'final' && p === 'FINAL' && h < 200 * FT) || (ga === 'flare' && p === 'FLARE' && h < 2) || (ga === 'base' && p === 'BASE')))
      asked = sim.requestGoAround();
    sim.step();
    maxThrRate = Math.max(maxThrRate, Math.abs(sim.aircraft.thr - prevThr) / Simulation.DT); prevThr = sim.aircraft.thr;
  }
  const ev = sim.evaluate(), ok = sim.finished && ev.every(r => r.info || r.pass);
  if (!ok && !gusts) fails++;
  console.log(`${String(fromDeg).padStart(3, '0')}/${speedKt}${gusts ? 'G' : ' '} ${ga ?? '     '} ${ok ? 'PASS' : 'FAIL'} t=${sim.t.toFixed(0)} GA=${sim.autopilot.goArounds} thrRate≤${maxThrRate.toFixed(2)} ` +
    ev.map(r => r.value).join(' | ') + ' ' + sim.events.filter(e => e.note).map(e => e.note).join(','));
}
(globalThis as any).process?.exit(fails ? 1 : 0);
