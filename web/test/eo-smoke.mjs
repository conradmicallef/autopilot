// Browser smoke test: fail the engine at several points on the crosswind leg and check the page never
// throws or freezes, and that the failure drops the speed to 1×. Run from web/: node test/eo-smoke.mjs (needs the playwright package).
import { chromium } from 'playwright';
const b = await chromium.launch({ args: ['--no-sandbox'] });
const results = [];
for (const delay of [0, 1500, 3000, 4500, 6000]) {
  const p = await b.newPage({ viewport: { width: 1280, height: 900 } });
  const errs = []; p.on('pageerror', e => errs.push(e.message)); p.on('console', m => { if (m.type() === 'error') errs.push(m.text()); });
  await p.goto('file://' + new URL('../../circuit-autopilot-sim.html', import.meta.url).pathname); await p.waitForTimeout(800);
  for (const bt of await p.$$('button')) if ((await bt.textContent()).trim() === '10×') await bt.click();
  for (let i = 0; i < 400; i++) { const ph = (await p.textContent('#phaseNow')).trim(); if (ph.startsWith('CROSSWIND')) break; await p.waitForTimeout(50); }
  await p.waitForTimeout(delay);
  await p.click('#eng');
  const warp = (await p.textContent('#warp button[aria-pressed="true"]')).trim();
  let last = '', same = 0;
  for (let i = 0; i < 240; i++) { await p.waitForTimeout(500); const c = await p.textContent('#clock'); if (c === last) same++; else same = 0; last = c; const ph = (await p.textContent('#phaseNow')).trim(); if (ph === 'STOPPED') break; }
  const ph = (await p.textContent('#phaseNow')).trim();
  results.push({ delay, warp, ph, clock: last, frozen: same > 4, errs: errs.filter(e => !e.includes('ERR_TUNNEL')) });
  await p.close();
}
console.log(JSON.stringify(results, null, 1));
await b.close();
