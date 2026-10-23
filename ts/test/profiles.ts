import { HIGH_CIRCUIT, STANDARD_CIRCUIT } from '../src/airfield';
import { Simulation } from '../src/simulation';
import { FT } from '../src/units';
const winds: [number, number, boolean][] = [[270, 0, false], [270, 10, false], [90, 8, false], [360, 15, false], [220, 12, true], [300, 20, true]];
for (const circuit of [STANDARD_CIRCUIT, HIGH_CIRCUIT]) for (const [fromDeg, speedKt, gusts] of winds) {
  const s = new Simulation({ wind: { fromDeg, speedKt, gusts }, seed: 3, circuit });
  let maxSink = 0;
  while (!s.finished && s.t < 1200) { s.step(); if (s.phase === 'FINAL') maxSink = Math.max(maxSink, -s.aircraft.verticalSpeed); }
  const ev = s.evaluate(), ok = ev.every(r => r.info || r.pass);
  const d = s.events.find(e => e.phase === 'DESCENT');
  console.log(`${circuit.name.padEnd(8)} ${String(fromDeg).padStart(3, '0')}/${speedKt}${gusts ? 'G' : ' '} ${ok ? 'PASS' : 'FAIL'} t=${s.t.toFixed(0)} GA=${s.autopilot.goArounds} reach=${Math.round(s.reachFraction * 100)}% descent@x=${d ? Math.round(d.x) : '-'} maxFinalSink=${Math.round(maxSink / FT * 60)}fpm | ` + ev.slice(0, 5).map(r => r.value).join(' | ') + ' ' + s.events.filter(e => e.note).map(e => e.note).join(','));
}
