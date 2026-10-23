// ---- Flight model + autopilot core (no DOM) ----
const G = 9.81, RHO = 1.225, KT = 0.514444, FT = 0.3048, D2R = Math.PI / 180;
const clamp = (v, a, b) => Math.max(a, Math.min(b, v));
const wrapPi = a => { while (a > Math.PI) a -= 2 * Math.PI; while (a <= -Math.PI) a += 2 * Math.PI; return a; };

// Light single-engine trainer, roughly C172-class
const AC = {
  m: 1000, S: 16.2, CLa: 5.0, CL0: 0.30, CLmax: 1.55, CD0: 0.032, k: 0.057,
  P: 70000, Tstatic: 2800, mu: 0.03, muBrake: 0.35,
  flaps: {
    0: { dCL: 0, dCD: 0, dCLmax: 0 },
    10: { dCL: 0.20, dCD: 0.008, dCLmax: 0.15 },
    20: { dCL: 0.35, dCD: 0.020, dCLmax: 0.30 },
    30: { dCL: 0.45, dCD: 0.035, dCLmax: 0.40 }
  }
};
// Runway 09/27, 1200 m, threshold at x=0, heading 090 = +x. Left-hand circuit (north side).
const RWY = { x0: 0, x1: 1200, aim: 250, width: 30 };
const CIRC = {
  downY: 1200, baseX: -2600, patternAlt: 1000 * FT, crossAlt: 500 * FT, flapsUpAlt: 300 * FT,
  Vr: 55 * KT, Vy: 75 * KT, Vdown: 90 * KT, Vdesc: 75 * KT, Vbase: 70 * KT, Vapp: 65 * KT,
  gs: 3 * D2R, flareH: 4.8, bankMax: 25 * D2R, descVS: -2.5, baseFloor: 150
};
const PHASES = ['TAKEOFF_ROLL', 'ROTATE', 'CLIMB', 'CROSSWIND', 'DOWNWIND', 'BASE', 'FINAL', 'FLARE', 'ROLLOUT', 'STOPPED', 'GO_AROUND'];
// Command rate limits: every mode change, including a go-around, ramps rather than steps
const SLEW = { theta: 3 * D2R, phi: 8 * D2R, thr: 0.3, flaps: 3 };   // per second
// Stabilised-approach gates, checked below 300 ft on final
const STAB = { gateH: 300 * FT, spdLo: 5 * KT, spdHi: 10 * KT, gsDev: 0.7, latDev: 30, sink: 1000 * FT / 60, bank: 15 * D2R, persist: 2 };

function createSim(opts = {}) {
  const s = {
    t: 0, x: 0, y: 0, h: 0, V: 0, Vg: 0, psi: 0, gamma: 0, theta: 0, q: 0, phi: 0, p: 0, alphaF: 0,
    thr: 0, flaps: 10, flapPos: 10, thetaRef: 0, phiRef: 0, thrRef: 0, brake: 0, onGround: true, stalled: false,
    phase: 'TAKEOFF_ROLL', vmode: 'TO', lmode: 'RWY', tmode: 'TOGA', descending: false,
    crossX: null, gaCount: 0, gaRequest: false, gaReason: '', gaT: 0, gaG: 0, unstableT: 0, maxBank: 0, maxAlt: 0,
    windDir: opts.windDir ?? 270, windKt: opts.windKt ?? 8, gusts: !!opts.gusts,
    gust: { x: 0, y: 0 }, w: { x: 0, y: 0 },
    cmd: { theta: 0, phi: 0, thr: 1, flaps: 10, brake: 0 },
    ap: { gI: 0, thrI: 0, climbI: 0 },
    track: [], events: [], td: null, liftoff: null, nextRec: 0, seed: opts.seed ?? 1
  };
  logEvent(s, 'TAKEOFF_ROLL');
  return s;
}
function rand(s) { s.seed = (s.seed * 16807) % 2147483647; return s.seed / 2147483647; }
function gauss(s) { return Math.sqrt(-2 * Math.log(rand(s) + 1e-9)) * Math.cos(2 * Math.PI * rand(s)); }

function logEvent(s, phase, note) {
  s.events.push({ t: s.t, phase, h: s.h, V: s.V, x: s.x, y: s.y, note });
}
function setPhase(s, p) { if (s.phase !== p) { s.phase = p; logEvent(s, p); } }

function flapCoeffs(pos) {
  const p = clamp(pos, 0, 30), i = Math.min(2, Math.floor(p / 10)), a = AC.flaps[i * 10], b = AC.flaps[i * 10 + 10], t = (p - i * 10) / 10;
  return { dCL: a.dCL + (b.dCL - a.dCL) * t, dCD: a.dCD + (b.dCD - a.dCD) * t, dCLmax: a.dCLmax + (b.dCLmax - a.dCLmax) * t };
}
function aero(s, V, alpha) {
  const f = flapCoeffs(s.flapPos);
  let CL = AC.CL0 + f.dCL + AC.CLa * alpha;
  const CLmax = AC.CLmax + f.dCLmax;
  let stalled = false;
  if (CL > CLmax) { stalled = true; CL = CLmax - (CL - CLmax) * 1.5; }
  CL = Math.max(CL, -0.8);
  const CD = AC.CD0 + f.dCD + AC.k * CL * CL;
  const qS = 0.5 * RHO * V * V * AC.S;
  return { L: qS * CL, D: qS * CD, stalled };
}
const thrust = (thr, V) => thr * Math.min(AC.Tstatic, AC.P / Math.max(V, 1));

function updateWind(s, dt) {
  const toDeg = s.windDir + 180, a = (90 - toDeg) * D2R, ws = s.windKt * KT;
  if (s.gusts) {
    const tau = 3, sig = 1.8;
    s.gust.x += (-s.gust.x / tau) * dt + sig * Math.sqrt(2 * dt / tau) * gauss(s);
    s.gust.y += (-s.gust.y / tau) * dt + sig * Math.sqrt(2 * dt / tau) * gauss(s);
  } else { s.gust.x *= 0.98; s.gust.y *= 0.98; }
  s.w.x = ws * Math.cos(a) + s.gust.x; s.w.y = ws * Math.sin(a) + s.gust.y;
}

function groundVel(s) {
  if (s.onGround) return { x: s.Vg * Math.cos(s.psi), y: s.Vg * Math.sin(s.psi) };
  const hv = s.V * Math.cos(s.gamma);
  return { x: hv * Math.cos(s.psi) + s.w.x, y: hv * Math.sin(s.psi) + s.w.y };
}

// ---- Autopilot ----
function legGuide(s, A, B) {
  const dx = B[0] - A[0], dy = B[1] - A[1], L = Math.hypot(dx, dy), ux = dx / L, uy = dy / L;
  const e = ux * (s.y - A[1]) - uy * (s.x - A[0]);           // + = left of track
  const remain = (B[0] - s.x) * ux + (B[1] - s.y) * uy;
  const chiLeg = Math.atan2(uy, ux);
  const chiD = chiLeg - (60 * D2R) * (2 / Math.PI) * Math.atan(e / 180);
  return { e, remain, chiD };
}
function bankFor(s, chiD, limit) {
  const gv = groundVel(s), chi = Math.atan2(gv.y, gv.x), Vg = Math.hypot(gv.x, gv.y);
  const err = wrapPi(chiD - chi);
  return clamp(Math.atan(Vg * err / (G * 3)), -limit, limit);
}
function turnLead(s) {
  const gv = groundVel(s), Vg = Math.hypot(gv.x, gv.y);
  // turn radius plus the distance covered while rolling into the bank (roll is slew-limited)
  return Vg * Vg / (G * Math.tan(CIRC.bankMax)) + Vg * (CIRC.bankMax / SLEW.phi) * 0.5;
}
// gamma (flight-path) loop -> pitch command
function gammaHold(s, gCmd, dt) {
  const err = gCmd - s.gamma;
  s.ap.gI = clamp(s.ap.gI + 0.4 * err * dt, -0.08, 0.08);
  return clamp(s.alphaF + gCmd + 0.8 * err + s.ap.gI, -10 * D2R, 15 * D2R);
}
function vsHold(s, vs, dt) { return gammaHold(s, Math.asin(clamp(vs / Math.max(s.V, 10), -0.25, 0.25)), dt); }
function altHold(s, hc, dt, vmax = 3) { return vsHold(s, clamp(0.15 * (hc - s.h), -vmax, vmax), dt); }
function speedThrottle(s, Vt, dt) {
  const e = Vt - s.V;
  let u = s.ap.thrI + 0.10 * e;
  if ((u < 1 || e < 0) && (u > 0 || e > 0)) s.ap.thrI = clamp(s.ap.thrI + 0.03 * e * dt, 0, 1);
  return clamp(u, 0, 1);
}

function checkStable(s, dt) {
  if (s.h > STAB.gateH) { s.unstableT = 0; return null; }
  const d = Math.max(RWY.aim - s.x, 1), dev = (Math.atan2(s.h, d) - CIRC.gs) / D2R, vs = s.V * Math.sin(s.gamma);
  let why = null;
  if (s.V < CIRC.Vapp - STAB.spdLo) why = 'Slow on approach';
  else if (s.V > CIRC.Vapp + STAB.spdHi) why = 'Fast on approach';
  else if (s.h > 60 * FT && Math.abs(dev) > STAB.gsDev) why = dev > 0 ? 'High on glidepath' : 'Low on glidepath';
  else if (Math.abs(s.y) > STAB.latDev) why = 'Off centreline';
  else if (vs < -STAB.sink) why = 'Sink rate';
  else if (Math.abs(s.phi) > STAB.bank) why = 'Excessive bank';
  s.unstableT = why ? s.unstableT + dt : 0;
  return s.unstableT > STAB.persist ? why : null;
}
function startGoAround(s, why) {
  s.gaCount++; s.gaReason = why; s.gaRequest = false; s.unstableT = 0;
  s.crossX = null; s.descending = false;
  s.gaT = s.t; s.gaG = s.gamma;                 // flight-path ramp starts from where we are: bumpless
  s.phase = 'GO_AROUND'; logEvent(s, 'GO_AROUND', why);
}

function autopilot(s, dt) {
  const c = s.cmd, gv = groundVel(s);
  switch (s.phase) {
    case 'TAKEOFF_ROLL': {
      c.thr = 1; c.flaps = 10; c.theta = 0; c.brake = 0; c.phi = 0;
      s.tmode = 'TOGA'; s.vmode = 'TO'; s.lmode = 'RWY';
      if (s.V >= CIRC.Vr) setPhase(s, 'ROTATE');
      break;
    }
    case 'ROTATE': {
      c.thr = 1; c.theta = 8 * D2R; c.phi = 0; s.vmode = 'ROT';
      if (!s.onGround) { s.ap.climbI = 0.06; s.ap.gI = 0; setPhase(s, 'CLIMB'); }
      break;
    }
    case 'CLIMB':
    case 'CROSSWIND': {
      // vertical: pitch-for-Vy at full throttle, then capture pattern altitude
      if (s.h > CIRC.flapsUpAlt) c.flaps = 0;
      const capture = s.h > CIRC.patternAlt - Math.max(15, s.V * Math.sin(s.gamma) * 6);
      if (s.vmode === 'ALT' || capture) {
        if (s.vmode !== 'ALT') { s.vmode = 'ALT'; s.ap.thrI = s.thr; }
        c.theta = altHold(s, CIRC.patternAlt, dt); c.thr = speedThrottle(s, CIRC.Vdown, dt); s.tmode = 'SPD';
      } else {
        s.vmode = 'PITCH-SPD'; s.tmode = 'TOGA'; c.thr = 1;
        const e = s.V - CIRC.Vy;
        s.ap.climbI = clamp(s.ap.climbI + 0.004 * e * dt, -0.05, 0.2);
        const gMin = s.h < 30 ? 0.05 : -0.03;               // no pitching down near the ground
        c.theta = gammaHold(s, clamp(s.ap.climbI + 0.015 * e, gMin, 0.2), dt);
      }
      // lateral
      if (s.phase === 'CLIMB') {
        s.lmode = 'TRK 090';
        c.phi = bankFor(s, legGuide(s, [0, 0], [20000, 0]).chiD, s.h < 60 ? 10 * D2R : CIRC.bankMax);
        if (s.h >= CIRC.crossAlt) {
          const lead = turnLead(s);
          if (s.crossX === null) s.crossX = Math.max(s.x + lead, RWY.x1 + 400);
          if (s.x >= s.crossX - lead) setPhase(s, 'CROSSWIND');
        }
      } else {
        s.lmode = 'LEG XWND';
        const g = legGuide(s, [s.crossX, 0], [s.crossX, CIRC.downY]);
        c.phi = bankFor(s, g.chiD, CIRC.bankMax);
        if (g.remain < turnLead(s)) setPhase(s, 'DOWNWIND');
      }
      break;
    }
    case 'DOWNWIND': {
      s.lmode = 'LEG DNWD';
      const g = legGuide(s, [s.crossX, CIRC.downY], [CIRC.baseX, CIRC.downY]);
      c.phi = bankFor(s, g.chiD, CIRC.bankMax);
      if (s.x < RWY.x0 && !s.descending) { s.descending = true; logEvent(s, 'DESCENT'); }
      if (s.descending) {
        c.flaps = 10; s.vmode = 'V/S'; s.tmode = 'SPD';
        c.theta = vsHold(s, clamp(0.15 * (CIRC.baseFloor - s.h), CIRC.descVS, 0.5), dt);
        c.thr = speedThrottle(s, CIRC.Vdesc, dt);
      } else {
        if (s.vmode !== 'ALT') { s.vmode = 'ALT'; s.ap.thrI = s.thr; }
        c.theta = altHold(s, CIRC.patternAlt, dt); c.thr = speedThrottle(s, CIRC.Vdown, dt); s.tmode = 'SPD';
      }
      if (g.remain < turnLead(s)) setPhase(s, 'BASE');
      break;
    }
    case 'BASE': {
      s.lmode = 'LEG BASE'; c.flaps = 20; s.vmode = 'V/S';
      const g = legGuide(s, [CIRC.baseX, CIRC.downY], [CIRC.baseX, 0]);
      c.phi = bankFor(s, g.chiD, CIRC.bankMax);
      const hgs = (RWY.aim - s.x) * Math.tan(CIRC.gs);
      c.theta = vsHold(s, clamp(0.15 * (Math.min(hgs, CIRC.baseFloor) - s.h), CIRC.descVS, 0.5), dt);
      c.thr = speedThrottle(s, CIRC.Vbase, dt);
      if (s.gaRequest) { startGoAround(s, 'Pilot request'); break; }
      if (g.remain < turnLead(s)) setPhase(s, 'FINAL');
      break;
    }
    case 'FINAL': {
      s.lmode = 'LOC'; s.vmode = 'G/S'; c.flaps = 30;
      const g = legGuide(s, [CIRC.baseX, 0], [RWY.x1, 0]);
      c.phi = bankFor(s, g.chiD, s.h < 30 ? 10 * D2R : CIRC.bankMax);
      const hgs = Math.max(0, (RWY.aim - s.x) * Math.tan(CIRC.gs));
      const vs = clamp(-gv.x * Math.tan(CIRC.gs) + 0.25 * (hgs - s.h), -4, 1);
      c.theta = vsHold(s, vs, dt);
      c.thr = speedThrottle(s, CIRC.Vapp, dt);
      const why = s.gaRequest ? 'Pilot request' : checkStable(s, dt);
      if (why) { startGoAround(s, why); break; }
      if (s.h < CIRC.flareH) setPhase(s, 'FLARE');
      break;
    }
    case 'FLARE': {
      s.vmode = 'FLARE'; s.tmode = 'IDLE'; c.thr = 0;
      const g = legGuide(s, [CIRC.baseX, 0], [RWY.x1, 0]);
      c.phi = bankFor(s, g.chiD, 5 * D2R);
      c.theta = vsHold(s, -(0.4 + 0.28 * s.h), dt);
      if (s.onGround) { setPhase(s, 'ROLLOUT'); break; }
      if (s.gaRequest) startGoAround(s, 'Pilot request');
      else if (s.x > RWY.x1 / 3 + 50) startGoAround(s, 'Long landing');
      break;
    }
    case 'ROLLOUT': {
      s.vmode = 'ROLLOUT'; s.lmode = 'RWY'; s.tmode = 'IDLE';
      c.thr = 0; c.theta = 0; c.phi = 0; c.flaps = 0;
      c.brake = (s.t - s.td.t > 2) ? 1 : 0;
      if (s.Vg < 0.5) { setPhase(s, 'STOPPED'); s.vmode = '—'; s.lmode = '—'; s.tmode = '—'; }
      break;
    }
    case 'GO_AROUND': {
      // Power and pitch come up together but gently: throttle and pitch are slew-limited,
      // and the flight-path target ramps from the current value at 1.5 deg/s.
      s.tmode = 'TOGA'; s.vmode = 'GA'; s.lmode = 'TRK 090';
      c.thr = 1;
      s.gaG = Math.min(s.gaG + 1.5 * D2R * dt, 4 * D2R);
      const slow = CIRC.Vapp - 3 * KT - s.V;          // speed protection: trade climb for airspeed
      c.theta = gammaHold(s, clamp(s.gaG - (slow > 0 ? 0.02 * slow : 0), -0.06, 0.12), dt);
      const vs = s.V * Math.sin(s.gamma);
      if (c.flaps > 20) c.flaps = 20;                 // first stage at once, then clean up in steps
      if (c.flaps > 10 && vs > 1 && s.V > 63 * KT && s.t - s.gaT > 4) c.flaps = 10;
      c.phi = bankFor(s, legGuide(s, [0, 0], [20000, 0]).chiD, 10 * D2R);
      if (vs > 1.5 && s.h > 150 * FT && s.flapPos <= 10.5) {
        s.ap.climbI = s.gamma; s.vmode = 'PITCH-SPD'; setPhase(s, 'CLIMB');
      }
      break;
    }
    case 'STOPPED': c.thr = 0; c.brake = 1; break;
  }
}

// ---- Physics ----
function step(s, dt) {
  if (s.phase === 'STOPPED') { s.Vg = 0; s.V = 0; s.t += dt; return; }
  updateWind(s, dt);
  autopilot(s, dt);
  const c = s.cmd;
  // inner loops: attitude hold -> rate -> attitude (with actuator lag)
  s.thetaRef += clamp(c.theta - s.thetaRef, -SLEW.theta * dt, SLEW.theta * dt);
  s.phiRef += clamp(c.phi - s.phiRef, -SLEW.phi * dt, SLEW.phi * dt);
  s.thrRef += clamp(c.thr - s.thrRef, -SLEW.thr * dt, SLEW.thr * dt);
  const qCmd = clamp(1.5 * (s.thetaRef - s.theta), -5 * D2R, 5 * D2R);
  s.q += (qCmd - s.q) * dt / 0.3; s.theta += s.q * dt;
  const pCmd = clamp(2.0 * (s.phiRef - s.phi), -15 * D2R, 15 * D2R);
  s.p += (pCmd - s.p) * dt / 0.2; s.phi += s.p * dt;
  s.thr += clamp(s.thrRef - s.thr, -0.8 * dt, 0.8 * dt);
  s.flapPos += clamp(c.flaps - s.flapPos, -SLEW.flaps * dt, SLEW.flaps * dt);
  s.flaps = Math.round(s.flapPos); s.brake = c.brake;

  if (s.onGround) {
    s.phi = 0; s.p = 0; s.phiRef = 0; s.gamma = 0;
    s.theta = clamp(s.theta, 0, 12 * D2R); if (s.theta === 0 && s.q < 0) s.q = 0;
    const along = s.w.x * Math.cos(s.psi) + s.w.y * Math.sin(s.psi);
    s.V = Math.max(0, s.Vg - along);
    const a = aero(s, s.V, s.theta), T = thrust(s.thr, s.V);
    const N = Math.max(0, AC.m * G - a.L), mu = AC.mu + s.brake * AC.muBrake;
    const acc = (T - a.D - mu * N * Math.sign(s.Vg || 1)) / AC.m;
    s.Vg = Math.max(0, s.Vg + acc * dt);
    s.alphaF = s.theta;
    const psiCmd = clamp(-0.03 * s.y, -0.08, 0.08);
    s.psi = wrapPi(s.psi + wrapPi(psiCmd - s.psi) * dt / 0.6);
    s.x += s.Vg * Math.cos(s.psi) * dt; s.y += s.Vg * Math.sin(s.psi) * dt;
    if (s.phase === 'ROTATE' && a.L > AC.m * G * 1.01) {
      s.onGround = false; s.h = 0.01; s.liftoff = { t: s.t, x: s.x, V: s.V };
    }
  } else {
    const alpha = s.theta - s.gamma, a = aero(s, s.V, alpha), T = thrust(s.thr, s.V);
    s.stalled = a.stalled;
    const mV = AC.m * Math.max(s.V, 5);
    const Vdot = (T * Math.cos(alpha) - a.D) / AC.m - G * Math.sin(s.gamma);
    const Lt = a.L + T * Math.sin(alpha);
    const gdot = (Lt * Math.cos(s.phi)) / mV - G * Math.cos(s.gamma) / Math.max(s.V, 5);
    const psidot = Lt * Math.sin(s.phi) / (mV * Math.cos(s.gamma));
    s.V += Vdot * dt; s.gamma += gdot * dt; s.psi = wrapPi(s.psi + psidot * dt);
    s.alphaF += (alpha - s.alphaF) * dt / 0.5;
    const gv = groundVel(s);
    s.x += gv.x * dt; s.y += gv.y * dt; s.h += s.V * Math.sin(s.gamma) * dt;
    s.maxBank = Math.max(s.maxBank, Math.abs(s.phi)); s.maxAlt = Math.max(s.maxAlt, s.h);
    if (s.h <= 0 && (s.phase === 'CLIMB' || s.phase === 'ROTATE' || s.phase === 'GO_AROUND')) {
      s.h = 0; s.gamma = Math.max(s.gamma, 0);          // settling just after lift-off: stay airborne
    } else if (s.h <= 0) {
      const sink = -s.V * Math.sin(s.gamma);
      const crab = wrapPi(s.psi - Math.atan2(gv.y, gv.x));
      s.td = { t: s.t, x: s.x, y: s.y, sink, V: s.V, crab, onRunway: s.x >= RWY.x0 && s.x <= RWY.x1 && Math.abs(s.y) <= RWY.width / 2 };
      s.h = 0; s.onGround = true;
      s.Vg = s.V * Math.cos(s.gamma) + (s.w.x * Math.cos(s.psi) + s.w.y * Math.sin(s.psi));
      s.gamma = 0;
      if (s.phase !== 'FLARE') setPhase(s, 'FLARE');
    }
  }
  s.t += dt;
  if (s.t >= s.nextRec) { s.track.push({ t: s.t, x: s.x, y: s.y, h: s.h, V: s.V, ph: s.phase }); s.nextRec = s.t + 0.25; }
}

function evaluate(s) {
  if (!s.td) return null;
  const sinkFps = s.td.sink / FT;
  const rollout = s.phase === 'STOPPED' ? s.x - s.td.x : null;
  return [
    { k: 'Touchdown sink rate', v: sinkFps.toFixed(1) + ' ft/s', ok: sinkFps < 3, crit: '< 3 ft/s' },
    { k: 'Touchdown point', v: Math.round(s.td.x) + ' m past threshold', ok: s.td.x >= 0 && s.td.x <= RWY.x1 / 3, crit: 'first third (0–400 m)' },
    { k: 'Centerline error', v: Math.abs(s.td.y).toFixed(1) + ' m', ok: Math.abs(s.td.y) < 5, crit: '< 5 m' },
    { k: 'Stopped on runway', v: s.phase === 'STOPPED' ? Math.round(s.x) + ' m mark' : 'rolling…', ok: s.phase === 'STOPPED' ? s.x <= RWY.x1 : null, crit: 'before 1200 m' }
  ];
}

if (typeof module !== 'undefined') module.exports = { createSim, step, evaluate, RWY, CIRC, FT, KT, D2R, SLEW };
