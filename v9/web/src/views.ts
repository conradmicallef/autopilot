import { Frame, TrackPoint } from './frame';
import { D2R, FT, KT } from './units';
import { PHASES, Palette, fitCanvas } from './theme';

/** Top-down chart: planned circuit, runway, flown track coloured by phase, aircraft, wind. */
export class PlanView {
  constructor(private cv: HTMLCanvasElement, private pal: Palette) {}

  draw(sim: Frame, track: TrackPoint[]): void {
    const { ctx, w, h } = fitCanvas(this.cv), P = this.pal, rwy = sim.runway, spec = sim.circuit;
    ctx.clearRect(0, 0, w, h);
    const route = sim.circuit.plannedRoute, cx = route[1][0];
    let minX = spec.baseX - 500, maxX = Math.max(cx, rwy.length) + 600, minY = -500, maxY = spec.downwindOffset + 450;
    for (const p of track) { minX = Math.min(minX, p.x - 200); maxX = Math.max(maxX, p.x + 200); minY = Math.min(minY, p.y - 200); maxY = Math.max(maxY, p.y + 200); }
    const sc = Math.min(w / (maxX - minX), h / (maxY - minY));
    const ox = (w - (maxX - minX) * sc) / 2 - minX * sc, oy = h - ((h - (maxY - minY) * sc) / 2 - minY * sc);
    const X = (x: number) => ox + x * sc, Y = (y: number) => oy - y * sc;
    const fDisp = P.get('--f-display'), fMono = P.get('--f-mono');

    // 500 m grid
    ctx.lineWidth = 1; ctx.strokeStyle = P.get('--line'); ctx.globalAlpha = 0.55; ctx.beginPath();
    for (let gx = Math.ceil(minX / 500) * 500; gx <= maxX; gx += 500) { ctx.moveTo(X(gx), 0); ctx.lineTo(X(gx), h); }
    for (let gy = Math.ceil(minY / 500) * 500; gy <= maxY; gy += 500) { ctx.moveTo(0, Y(gy)); ctx.lineTo(w, Y(gy)); }
    ctx.stroke(); ctx.globalAlpha = 1;

    // planned circuit, magenta like a chart route
    ctx.strokeStyle = P.get('--route'); ctx.lineWidth = 1.5; ctx.setLineDash([6, 5]); ctx.beginPath();
    route.forEach((p, i) => (i ? ctx.lineTo(X(p[0]), Y(p[1])) : ctx.moveTo(X(p[0]), Y(p[1]))));
    ctx.stroke(); ctx.setLineDash([]);
    ctx.fillStyle = P.get('--route'); ctx.textAlign = 'center';
    ctx.font = `600 ${w < 500 ? 10 : 12}px ${fDisp}`;
    const label = (t: string, x: number, y: number, rot = 0) => { ctx.save(); ctx.translate(X(x), Y(y)); ctx.rotate(rot); ctx.fillText(t, 0, 0); ctx.restore(); };
    label('DOWNWIND · 1000 FT', (cx + spec.baseX) / 2, spec.downwindOffset + 70);
    label('CROSSWIND', cx + 80, spec.downwindOffset / 2, Math.PI / 2);
    label('BASE', spec.baseX - 80, spec.downwindOffset / 2, -Math.PI / 2);
    label('FINAL', spec.baseX / 2, -110); label('UPWIND', (rwy.length + cx) / 2, -110);

    // runway
    const rw = Math.max(rwy.width * sc, 7);
    ctx.fillStyle = P.get('--runway'); ctx.fillRect(X(0), Y(0) - rw / 2, rwy.length * sc, rw);
    ctx.strokeStyle = P.get('--rwmark'); ctx.lineWidth = 1; ctx.setLineDash([4, 4]);
    ctx.beginPath(); ctx.moveTo(X(0) + 6, Y(0)); ctx.lineTo(X(rwy.length) - 6, Y(0)); ctx.stroke(); ctx.setLineDash([]);
    ctx.fillStyle = P.get('--rwmark'); ctx.fillRect(X(rwy.aimPoint) - 1, Y(0) - rw / 2 + 1, 3, rw - 2);
    ctx.fillStyle = P.get('--ink'); ctx.font = `700 12px ${fDisp}`;
    ctx.textAlign = 'right'; ctx.fillText('09', X(0) - 5, Y(0) + 4);
    ctx.textAlign = 'left'; ctx.fillText('27', X(rwy.length) + 5, Y(0) + 4);

    // 50 ft trees beyond each end of a short strip (drawn as a short row of canopies across the extended centreline)
    if (rwy.obstacles) {
      ctx.fillStyle = P.get('--ok');
      for (const ox of [-rwy.obstacleOffset, rwy.length + rwy.obstacleOffset]) {
        for (let k = -2; k <= 2; k++) { ctx.beginPath(); ctx.arc(X(ox), Y(k * 28), Math.max(3, 9 * sc), 0, 7); ctx.fill(); }
      }
      ctx.font = `600 10px ${fDisp}`; ctx.textAlign = 'center';
      ctx.fillText('TREES 50 FT', X(-rwy.obstacleOffset), Y(-90)); ctx.fillText('TREES 50 FT', X(rwy.length + rwy.obstacleOffset), Y(-90));
    }

    // flown track
    ctx.lineWidth = 2.5; ctx.lineCap = 'round';
    for (let i = 1; i < track.length; i++) {
      const a = track[i - 1], b = track[i];
      ctx.strokeStyle = P.phase(b.phase); ctx.beginPath(); ctx.moveTo(X(a.x), Y(a.y)); ctx.lineTo(X(b.x), Y(b.y)); ctx.stroke();
    }
    // go-around points
    ctx.font = `700 11px ${fDisp}`; ctx.textAlign = 'center';
    for (const e of sim.events) if (e.phase === 'GO_AROUND') {
      ctx.fillStyle = P.get('--ph-ga'); ctx.beginPath(); ctx.arc(X(e.x), Y(e.y), 4.5, 0, 7); ctx.fill();
      ctx.fillText('GA', X(e.x), Y(e.y) - 9);
    }
    this.drawEngineOut(sim, ctx, X, Y, sc);
    const td = sim.touchdown;
    if (td) { ctx.strokeStyle = P.get('--ink'); ctx.lineWidth = 1.5; ctx.beginPath(); ctx.arc(X(td.x), Y(td.y), 6, 0, 7); ctx.stroke(); }

    // aircraft silhouette, nose along +x before rotation
    const ac = sim.aircraft, k = 1.1;
    ctx.save(); ctx.translate(X(ac.x), Y(ac.y)); ctx.rotate(-ac.noseHeading); ctx.beginPath();
    const pts = [[10, 0], [4, 1.6], [2, 11], [-1, 11], [-1, 1.6], [-7, 1.4], [-9, 4.5], [-10.5, 4.5], [-10, 0],
      [-10.5, -4.5], [-9, -4.5], [-7, -1.4], [-1, -1.6], [-1, -11], [2, -11], [4, -1.6]];
    pts.forEach(([px, py], i) => (i ? ctx.lineTo(px * k, py * k) : ctx.moveTo(px * k, py * k)));
    ctx.closePath(); ctx.fillStyle = P.get('--ink'); ctx.strokeStyle = P.get('--paper'); ctx.lineWidth = 2; ctx.stroke(); ctx.fill();
    ctx.restore();

    // wind, north, scale
    const ws = sim.wind, wx = w - 44, wy = 44, toA = (90 - (ws.fromDeg + 180)) * D2R;
    ctx.strokeStyle = P.get('--ink'); ctx.fillStyle = P.get('--ink'); ctx.lineWidth = 1;
    ctx.globalAlpha = 0.25; ctx.beginPath(); ctx.arc(wx, wy, 22, 0, 7); ctx.stroke(); ctx.globalAlpha = 1;
    if (ws.speedKt > 0) {
      ctx.save(); ctx.translate(wx, wy); ctx.rotate(-toA); ctx.lineWidth = 2;
      ctx.beginPath(); ctx.moveTo(-18, 0); ctx.lineTo(14, 0); ctx.stroke();
      ctx.beginPath(); ctx.moveTo(18, 0); ctx.lineTo(10, -5); ctx.lineTo(10, 5); ctx.closePath(); ctx.fill(); ctx.restore();
    }
    ctx.font = `500 11px ${fMono}`; ctx.textAlign = 'center';
    ctx.fillText(ws.speedKt ? `${String(ws.fromDeg).padStart(3, '0')}°/${ws.speedKt}kt${ws.gusts ? ' G' : ''}` : 'CALM', wx, wy + 38);
    ctx.font = `700 11px ${fDisp}`; ctx.fillText('N ↑', wx, wy - 28);
    const bar = 1000 * sc, bx = 14, by = h - 16;
    ctx.lineWidth = 2; ctx.beginPath(); ctx.moveTo(bx, by - 4); ctx.lineTo(bx, by); ctx.lineTo(bx + bar, by); ctx.lineTo(bx + bar, by - 4); ctx.stroke();
    ctx.textAlign = 'left'; ctx.font = `500 11px ${fMono}`; ctx.fillText('1 km', bx + bar + 6, by + 3);
  }

  /** Failure point, the planned power-off route (base key, intercept) or the field ahead, and a still-air reach ring. */
  private drawEngineOut(sim: Frame, ctx: CanvasRenderingContext2D, X: (x: number) => number, Y: (y: number) => number, sc: number): void {
    const eo = sim.autopilot.engineOut; if (!eo) return;
    const P = this.pal, col = P.get('--ph-eo'), ac = sim.aircraft, fDisp = P.get('--f-display');
    ctx.save();
    ctx.strokeStyle = col; ctx.fillStyle = col; ctx.lineWidth = 2;
    const fx = X(eo.x), fy = Y(eo.y);
    ctx.beginPath(); ctx.moveTo(fx - 5, fy - 5); ctx.lineTo(fx + 5, fy + 5); ctx.moveTo(fx + 5, fy - 5); ctx.lineTo(fx - 5, fy + 5); ctx.stroke();
    ctx.font = `700 11px ${fDisp}`; ctx.textAlign = 'center'; ctx.fillText('ENG', fx, fy - 9);
    if (!ac.onGround && sim.phase !== 'FLARE') {
      ctx.globalAlpha = 0.5; ctx.setLineDash([2, 4]); ctx.lineWidth = 1.2;
      ctx.beginPath(); ctx.arc(X(ac.x), Y(ac.y), ac.stillAirReach * sc, 0, 7); ctx.stroke();
      ctx.globalAlpha = 1;
    }
    const g = sim.autopilot.glide;
    if (eo.plan === 'RETURN' && g && (sim.phase === 'EO_GLIDE' || sim.phase === 'EO_FINAL')) {
      ctx.setLineDash([7, 4]); ctx.lineWidth = 1.8; ctx.beginPath();
      const pts = sim.phase === 'EO_FINAL' ? [[ac.x, ac.y], sim.autopilot.eoEnd.aim] : g.route;
      pts.forEach((p, i) => (i ? ctx.lineTo(X(p[0]), Y(p[1])) : ctx.moveTo(X(p[0]), Y(p[1])))); ctx.stroke();
      ctx.setLineDash([]); ctx.font = `700 10px ${fDisp}`;
      if (sim.phase === 'EO_GLIDE') g.route.slice(1, -1).forEach((p, i, a) => {
        ctx.beginPath(); ctx.arc(X(p[0]), Y(p[1]), 3.5, 0, 7); ctx.fill();
        ctx.fillText(i === a.length - 1 ? 'IP' : 'KEY', X(p[0]), Y(p[1]) + 15);
      });
    } else if (eo.plan === 'LAND_AHEAD' && !sim.touchdown) {
      ctx.setLineDash([7, 4]); ctx.lineWidth = 1.8; ctx.beginPath();
      const len = Math.max(400, ac.stillAirReach);
      ctx.moveTo(X(ac.x), Y(ac.y)); ctx.lineTo(X(ac.x + Math.cos(eo.track) * len), Y(ac.y + Math.sin(eo.track) * len)); ctx.stroke();
    }
    ctx.restore();
  }
}

/** Altitude and airspeed against time, with phase bands. */
export class ProfileView {
  constructor(private cv: HTMLCanvasElement, private pal: Palette) {}

  draw(sim: Frame, track: TrackPoint[]): void {
    const { ctx, w, h } = fitCanvas(this.cv), P = this.pal;
    ctx.clearRect(0, 0, w, h);
    const L = 46, R = 12, T0 = 12, gap = 26, ph = (h - T0 - 28 - gap) / 2;
    const Tmax = Math.max(360, Math.ceil((sim.t + 20) / 60) * 60);
    const X = (t: number) => L + (t / Tmax) * (w - L - R);
    const fMono = P.get('--f-mono'), fDisp = P.get('--f-display');
    const panels = [
      { top: T0, max: 1200, ticks: [0, 400, 800, 1200], val: (p: { h: number }) => p.h / FT, label: 'ALT (ft)', color: P.get('--ink') },
      { top: T0 + ph + gap, max: 100, ticks: [0, 25, 50, 75, 100], val: (p: { V: number }) => p.V / KT, label: 'IAS (kt)', color: P.get('--route') }
    ];
    const ev = sim.events.filter(e => e.phase !== 'DESCENT' && e.phase !== 'ENGINE_FAILURE');
    ev.forEach((e, i) => {
      const t1 = i + 1 < ev.length ? ev[i + 1].t : sim.t;
      ctx.fillStyle = P.phase(e.phase as never); ctx.globalAlpha = e.phase === 'GO_AROUND' || P.get(PHASES[e.phase as never as keyof typeof PHASES].token) === P.get('--ph-eo') ? 0.22 : 0.10;
      for (const p of panels) ctx.fillRect(X(e.t), p.top, X(t1) - X(e.t), ph);
      ctx.globalAlpha = 1; ctx.fillRect(X(e.t), panels[0].top - 6, Math.max(1, X(t1) - X(e.t)), 3);
    });
    for (const p of panels) {
      const Y = (v: number) => p.top + ph - (v / p.max) * ph;
      ctx.strokeStyle = P.get('--line'); ctx.lineWidth = 1; ctx.fillStyle = P.get('--muted'); ctx.textAlign = 'right';
      ctx.font = `500 10.5px ${fMono}`;
      for (const t of p.ticks) { ctx.beginPath(); ctx.moveTo(L, Y(t) + 0.5); ctx.lineTo(w - R, Y(t) + 0.5); ctx.stroke(); ctx.fillText(String(t), L - 6, Y(t) + 3.5); }
      ctx.textAlign = 'left'; ctx.fillStyle = p.color; ctx.font = `600 11px ${fDisp}`; ctx.fillText(p.label, L + 4, p.top + 11);
      ctx.strokeStyle = p.color; ctx.lineWidth = 2; ctx.beginPath();
      track.forEach((q, i) => { const x = X(q.t), y = Y(Math.min(p.max * 1.02, p.val(q))); i ? ctx.lineTo(x, y) : ctx.moveTo(x, y); });
      ctx.stroke();
      const last = track[track.length - 1];
      if (last) { ctx.fillStyle = p.color; ctx.beginPath(); ctx.arc(X(last.t), Y(Math.min(p.max, p.val(last))), 3.5, 0, 7); ctx.fill(); }
    }
    // glide-reach strip under the altitude panel: green while a power-off return would make the runway
    const sy = panels[0].top + ph + 5;
    for (let i = 1; i < track.length; i++) {
      const q = track[i]; if (q.reach === null) continue;
      ctx.fillStyle = q.reach ? P.get('--ok') : P.get('--bad'); ctx.globalAlpha = q.reach ? 0.85 : 0.45;
      ctx.fillRect(X(track[i - 1].t), sy, X(q.t) - X(track[i - 1].t) + 0.5, 4);
    }
    ctx.globalAlpha = 1; ctx.textAlign = 'right'; ctx.fillStyle = P.get('--muted'); ctx.font = `600 9.5px ${fDisp}`;
    ctx.fillText('GLIDE REACH', w - R, sy + 14);
    ctx.fillStyle = P.get('--muted'); ctx.textAlign = 'center'; ctx.font = `500 10.5px ${fMono}`;
    const step = Tmax > 720 ? 120 : 60;
    for (let t = 0; t <= Tmax; t += step) ctx.fillText(`${t / 60}:00`, X(t), h - 8);
  }
}

/** Attitude indicator. Deliberately fixed instrument colours: it lives on the always-dark panel. */
export class AttitudeIndicator {
  constructor(private cv: HTMLCanvasElement, private pal: Palette) {}

  draw(sim: Frame): void {
    const cv = this.cv, dpr = Math.min(window.devicePixelRatio || 1, 2), size = 150;
    if (cv.width !== size * dpr) { cv.width = size * dpr; cv.height = size * dpr; }
    const ctx = cv.getContext('2d')!; ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    const r = size / 2, ppd = 3.2, ac = sim.aircraft, fMono = this.pal.get('--f-mono');
    ctx.save(); ctx.beginPath(); ctx.arc(r, r, r, 0, 7); ctx.clip();
    ctx.translate(r, r); ctx.rotate(-ac.phi); ctx.translate(0, (ac.theta / D2R) * ppd);
    ctx.fillStyle = '#2c6aa6'; ctx.fillRect(-200, -400, 400, 400);
    ctx.fillStyle = '#7a4f28'; ctx.fillRect(-200, 0, 400, 400);
    ctx.strokeStyle = '#ffffff'; ctx.fillStyle = '#ffffff'; ctx.lineWidth = 1.5;
    ctx.beginPath(); ctx.moveTo(-200, 0); ctx.lineTo(200, 0); ctx.stroke();
    ctx.font = `500 9px ${fMono}`; ctx.lineWidth = 1; ctx.textAlign = 'left';
    for (let d = -20; d <= 20; d += 5) {
      if (!d) continue;
      const y = -d * ppd, half = d % 10 ? 9 : 18;
      ctx.beginPath(); ctx.moveTo(-half, y); ctx.lineTo(half, y); ctx.stroke();
      if (!(d % 10)) ctx.fillText(String(Math.abs(d)), half + 3, y + 3);
    }
    ctx.restore();
    ctx.save(); ctx.translate(r, r); ctx.strokeStyle = '#ffffff'; ctx.lineWidth = 1.5;
    for (const b of [-60, -45, -30, -20, -10, 0, 10, 20, 30, 45, 60]) {
      const a = (b - 90) * D2R, l = b % 30 ? 6 : 10;
      ctx.beginPath(); ctx.moveTo(Math.cos(a) * (r - 2), Math.sin(a) * (r - 2)); ctx.lineTo(Math.cos(a) * (r - 2 - l), Math.sin(a) * (r - 2 - l)); ctx.stroke();
    }
    ctx.rotate(-ac.phi); ctx.fillStyle = '#ffffff';
    ctx.beginPath(); ctx.moveTo(0, -r + 14); ctx.lineTo(-5, -r + 22); ctx.lineTo(5, -r + 22); ctx.closePath(); ctx.fill();
    ctx.restore();
    ctx.strokeStyle = '#f2b33d'; ctx.lineWidth = 3.5; ctx.lineCap = 'round'; ctx.beginPath();
    ctx.moveTo(r - 42, r); ctx.lineTo(r - 16, r); ctx.lineTo(r - 9, r + 7);
    ctx.moveTo(r + 42, r); ctx.lineTo(r + 16, r); ctx.lineTo(r + 9, r + 7); ctx.stroke();
    ctx.fillStyle = '#f2b33d'; ctx.fillRect(r - 2.5, r - 2.5, 5, 5);
  }
}
