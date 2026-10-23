import { HIGH_CIRCUIT, STANDARD_CIRCUIT } from '../src/airfield';
import { Simulation } from '../src/simulation';
import { FT, KT } from '../src/units';
type Trig = { name: string; when: (s: Simulation) => boolean };
const trigs: Trig[] = [
  { name: 'roll 40kt', when: s => s.phase === 'TAKEOFF_ROLL' && s.aircraft.V > 40 * KT },
  { name: 'climb 200ft', when: s => s.phase === 'CLIMB' && s.aircraft.h > 200 * FT },
  { name: 'climb 450ft', when: s => s.phase === 'CLIMB' && s.aircraft.h > 450 * FT },
  { name: 'crosswind', when: s => s.phase === 'CROSSWIND' && s.aircraft.h > 700 * FT },
  { name: 'downwind early', when: s => s.phase === 'DOWNWIND' && s.aircraft.x < 1800 },
  { name: 'downwind mid', when: s => s.phase === 'DOWNWIND' && s.aircraft.x < 1000 },
  { name: 'abeam thr', when: s => s.phase === 'DOWNWIND' && s.aircraft.x < 0 },
  { name: 'base', when: s => s.phase === 'BASE' && s.aircraft.y < 700 },
  { name: 'final 400ft', when: s => s.phase === 'FINAL' && s.aircraft.h < 400 * FT },
  { name: 'final 150ft', when: s => s.phase === 'FINAL' && s.aircraft.h < 150 * FT },
];
let pass = 0, total = 0;
for (const circuit of [STANDARD_CIRCUIT, HIGH_CIRCUIT])
for (const w of [{ fromDeg: 270, speedKt: 0, gusts: false }, { fromDeg: 270, speedKt: 12, gusts: false }, { fromDeg: 360, speedKt: 12, gusts: true }])
for (const tr of trigs) {
  const s = new Simulation({ wind: w, seed: 4, circuit });
  let done = false, minV = 999, stall = 0;
  while (!s.finished && s.t < 1200) { if (!done && tr.when(s)) done = s.failEngine(); s.step(); if (s.engineFailed && !s.aircraft.onGround) { minV = Math.min(minV, s.aircraft.V / KT); if (s.aircraft.stalled) stall++; } }
  const ev = s.evaluate(), ok = s.finished && ev.every(r => r.info || r.pass);
  total++; if (ok) pass++;
  console.log(`${circuit.name.padEnd(8)} ${w.fromDeg}/${w.speedKt}${w.gusts ? 'G' : ''} ${tr.name.padEnd(13)} ${ok ? 'PASS' : 'FAIL'} t=${s.t.toFixed(0)} minIAS=${minV === 999 ? '-' : minV.toFixed(0)} stall=${stall} | ` + ev.map(r => r.value).join(' | ') + ' | ' + s.events.filter(e => ['ENGINE_FAILURE','EO_GLIDE','EO_FINAL','FORCED_LANDING','FLARE'].includes(e.phase)).map(e => e.phase + '@' + Math.round(e.h / FT) + 'ft' + (e.note ? '(' + e.note + ')' : '')).join(' '));
}

console.log(`${pass}/${total} passed`);
