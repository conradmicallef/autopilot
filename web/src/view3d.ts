import { Frame, TrackPoint } from './frame';
import { FT, KT } from './units';
import { Palette, fitCanvas } from './theme';

type V3 = [number, number, number];
export type CamMode = 'chase' | 'tower' | 'cockpit';

const add = (a: V3, b: V3): V3 => [a[0] + b[0], a[1] + b[1], a[2] + b[2]];
const sub = (a: V3, b: V3): V3 => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
const mul = (a: V3, k: number): V3 => [a[0] * k, a[1] * k, a[2] * k];
const dot = (a: V3, b: V3) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
const cross = (a: V3, b: V3): V3 => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
const norm = (a: V3): V3 => mul(a, 1 / Math.hypot(a[0], a[1], a[2]));

/** Simple aircraft model, body axes: x forward, y left, z up (m, about the CG). Polygons + colour role. */
const MODEL: { pts: V3[]; role: 'wing' | 'body' | 'tail' }[] = [
  { role: 'wing', pts: [[0.6, 0.4, 0.9], [0.6, 5.5, 1.0], [-0.9, 5.5, 1.0], [-0.9, 0.4, 0.9]] },
  { role: 'wing', pts: [[0.6, -0.4, 0.9], [0.6, -5.5, 1.0], [-0.9, -5.5, 1.0], [-0.9, -0.4, 0.9]] },
  { role: 'body', pts: [[2.2, 0, 0.3], [0.8, 0.55, 0.9], [-1.0, 0.5, 0.8], [-4.6, 0.12, 0.45], [-4.6, -0.12, 0.45], [-1.0, -0.5, 0.8], [0.8, -0.55, 0.9]] },
  { role: 'body', pts: [[2.2, 0, 0.3], [0.8, 0.55, -0.4], [-1.0, 0.45, -0.3], [-4.6, 0.1, 0.35], [-4.6, -0.1, 0.35], [-1.0, -0.45, -0.3], [0.8, -0.55, -0.4]] },
  { role: 'tail', pts: [[-3.8, 0.1, 0.45], [-3.8, 1.7, 0.45], [-4.6, 1.7, 0.45], [-4.7, 0.1, 0.45]] },
  { role: 'tail', pts: [[-3.8, -0.1, 0.45], [-3.8, -1.7, 0.45], [-4.6, -1.7, 0.45], [-4.7, -0.1, 0.45]] },
  { role: 'tail', pts: [[-3.6, 0, 0.5], [-4.7, 0, 2.0], [-5.0, 0, 2.0], [-4.8, 0, 0.45]] }
];

/** Perspective view drawn on a 2D canvas: ground grid, runway, flown track, glide footprint, aircraft. */
export class View3D {
  mode: CamMode = 'chase';
  /** Only the visible view reacts to drag and scroll. */
  active = false;
  /** Chase-camera orbit, set by dragging: azimuth offset and elevation (rad). */
  private orbit = 0;
  private elev = 0.18;
  private dist = 70;

  constructor(private cv: HTMLCanvasElement, private pal: Palette) {
    let dragging = false, lx = 0, ly = 0;
    cv.addEventListener('pointerdown', e => { dragging = true; lx = e.clientX; ly = e.clientY; cv.setPointerCapture(e.pointerId); });
    cv.addEventListener('pointerup', () => { dragging = false; });
    cv.addEventListener('pointermove', e => {
      if (!dragging || !this.active || this.mode !== 'chase') return;
      this.orbit -= (e.clientX - lx) * 0.008;
      this.elev = Math.min(1.3, Math.max(-0.05, this.elev + (e.clientY - ly) * 0.005));
      lx = e.clientX; ly = e.clientY;
    });
    cv.addEventListener('wheel', e => {
      if (!this.active || this.mode !== 'chase') return;
      e.preventDefault();
      this.dist = Math.min(600, Math.max(25, this.dist * (e.deltaY > 0 ? 1.12 : 1 / 1.12)));
    }, { passive: false });
  }

  draw(sim: Frame, track: TrackPoint[]): void {
    const { ctx, w, h } = fitCanvas(this.cv), P = this.pal, ac = sim.aircraft, rwy = sim.runway;
    const fDisp = P.get('--f-display'), fMono = P.get('--f-mono');
    const pos: V3 = [ac.x, ac.y, ac.h];
    const nose = ac.noseHeading;

    // aircraft attitude: body axes in the world (nose heading, pitch, bank + = left wing down)
    const cr = Math.cos(-ac.phi), sr = Math.sin(-ac.phi), cp = Math.cos(-ac.theta), sp = Math.sin(-ac.theta), cy = Math.cos(nose), sy = Math.sin(nose);
    const bodyToWorld = (b: V3, k = 1): V3 => {
      let [x, y, z] = mul(b, k);
      [y, z] = [y * cr - z * sr, y * sr + z * cr];       // roll about x
      [x, z] = [x * cp + z * sp, -x * sp + z * cp];       // pitch about y (left)
      [x, y] = [x * cy - y * sy, x * sy + y * cy];        // yaw about z
      return [x, y, z];
    };
    const cockpit = this.mode === 'cockpit';

    // --- camera
    let cam: V3, target: V3;
    if (cockpit) {
      // pilot's eye: a little forward of and above the CG, looking along the body axis
      cam = add(pos, bodyToWorld([0.6, 0.3, 1.0]));
      target = add(cam, bodyToWorld([1, 0, 0]));
    } else if (this.mode === 'chase') {
      const az = nose + Math.PI + this.orbit;
      const d = this.dist;
      cam = [pos[0] + Math.cos(az) * d * Math.cos(this.elev), pos[1] + Math.sin(az) * d * Math.cos(this.elev), Math.max(2, pos[2] + d * Math.sin(this.elev) + 3)];
      target = [pos[0] + Math.cos(nose) * 25, pos[1] + Math.sin(nose) * 25, pos[2]];
    } else {
      cam = [rwy.length * 0.5, -260, 25];
      target = pos;
    }
    const f = norm(sub(target, cam));
    // the cockpit camera rolls with the aircraft; the others keep the horizon level
    const r = cockpit ? mul(bodyToWorld([0, 1, 0]), -1) : norm(cross(f, [0, 0, 1]));
    const u = cockpit ? bodyToWorld([0, 0, 1]) : cross(r, f);
    const focal = (h / 2) / Math.tan((this.mode === 'tower' ? 22 : cockpit ? 24 : 30) * Math.PI / 180);
    const near = 1;
    const cam2 = (p: V3): V3 => { const d = sub(p, cam); return [dot(d, r), dot(d, u), dot(d, f)]; };
    const scr = (c: V3): [number, number] => [w / 2 + focal * c[0] / c[2], h / 2 - focal * c[1] / c[2]];
    const proj = (p: V3): [number, number] | null => { const c = cam2(p); return c[2] > near ? scr(c) : null; };
    /** Line segment with near-plane clipping. */
    const seg = (a: V3, b: V3): [[number, number], [number, number]] | null => {
      let ca = cam2(a), cb = cam2(b);
      if (ca[2] <= near && cb[2] <= near) return null;
      if (ca[2] <= near) { const t = (near - ca[2]) / (cb[2] - ca[2]); ca = add(ca, mul(sub(cb, ca), t)); }
      if (cb[2] <= near) { const t = (near - cb[2]) / (ca[2] - cb[2]); cb = add(cb, mul(sub(ca, cb), t)); }
      return [scr(ca), scr(cb)];
    };
    const line = (a: V3, b: V3) => { const s = seg(a, b); if (s) { ctx.moveTo(s[0][0], s[0][1]); ctx.lineTo(s[1][0], s[1][1]); } };
    const poly = (pts: V3[]): [number, number][] | null => {
      // clip a ground polygon against the near plane (Sutherland–Hodgman, one plane)
      const cs = pts.map(cam2), out: V3[] = [];
      for (let i = 0; i < cs.length; i++) {
        const a = cs[i], b = cs[(i + 1) % cs.length], ina = a[2] > near, inb = b[2] > near;
        if (ina) out.push(a);
        if (ina !== inb) { const t = (near - a[2]) / (b[2] - a[2]); out.push(add(a, mul(sub(b, a), t))); }
      }
      return out.length >= 3 ? out.map(scr) : null;
    };

    // --- sky and ground, split at the horizon
    const fh = norm([f[0], f[1], 0]);
    const hz = proj(add(cam, mul(fh, 1e6)));
    const hy = hz ? Math.max(-h, Math.min(2 * h, hz[1])) : (f[2] > 0 ? 2 * h : -h);
    const sky = ctx.createLinearGradient(0, 0, 0, Math.max(1, hy));
    sky.addColorStop(0, P.get('--sky-top')); sky.addColorStop(1, P.get('--sky-low'));
    if (cockpit) {
      // banked horizon: sky everywhere, then the ground as a large plane clipped at the near plane
      ctx.fillStyle = P.get('--sky-top'); ctx.fillRect(0, 0, w, h);
      const R = 60000, gp = poly([[cam[0] - R, cam[1] - R, 0], [cam[0] + R, cam[1] - R, 0], [cam[0] + R, cam[1] + R, 0], [cam[0] - R, cam[1] + R, 0]]);
      if (gp) { ctx.fillStyle = P.get('--earth'); ctx.beginPath(); gp.forEach((p, i) => (i ? ctx.lineTo(p[0], p[1]) : ctx.moveTo(p[0], p[1]))); ctx.closePath(); ctx.fill(); }
    } else {
      ctx.fillStyle = sky; ctx.fillRect(0, 0, w, Math.max(0, hy));
      ctx.fillStyle = P.get('--earth'); ctx.fillRect(0, Math.max(0, hy), w, h - Math.max(0, hy));
    }

    // --- ground grid, 250 m, around the aircraft and the field
    ctx.strokeStyle = P.get('--grid3d'); ctx.lineWidth = 1; ctx.beginPath();
    const gx0 = Math.floor((Math.min(pos[0], 0) - 3000) / 250) * 250, gx1 = Math.max(pos[0], rwy.length) + 3000;
    const gy0 = Math.floor((Math.min(pos[1], 0) - 3000) / 250) * 250, gy1 = Math.max(pos[1], 0) + 3000;
    for (let x = gx0; x <= gx1; x += 250) line([x, gy0, 0], [x, gy1, 0]);
    for (let y = gy0; y <= gy1; y += 250) line([gx0, y, 0], [gx1, y, 0]);
    ctx.stroke();

    // --- runway, centreline, threshold bars, trees on short strips
    const hw = rwy.width / 2;
    const rp = poly([[0, -hw, 0], [rwy.length, -hw, 0], [rwy.length, hw, 0], [0, hw, 0]]);
    if (rp) { ctx.fillStyle = P.get('--runway3d'); ctx.beginPath(); rp.forEach((p, i) => (i ? ctx.lineTo(p[0], p[1]) : ctx.moveTo(p[0], p[1]))); ctx.closePath(); ctx.fill(); }
    ctx.strokeStyle = P.get('--rwmark3d'); ctx.lineWidth = 1.5; ctx.beginPath();
    for (let x = 60; x < rwy.length - 60; x += 60) line([x, 0, 0.05], [x + 30, 0, 0.05]);
    for (const tx of [6, rwy.length - 6]) for (let y = -hw + 2; y <= hw - 2; y += 3) line([tx - 4, y, 0.05], [tx + 4, y, 0.05]);
    ctx.stroke();
    if (rwy.obstacles) {
      ctx.strokeStyle = P.get('--tree'); ctx.lineWidth = 3; ctx.beginPath();
      for (const x of [-rwy.obstacleOffset, rwy.length + rwy.obstacleOffset]) for (let y = -60; y <= 60; y += 20) line([x, y, 0], [x, y, rwy.obstacleHeight]);
      ctx.stroke();
    }

    // --- planned circuit on the ground (faint), glide footprint
    ctx.strokeStyle = P.get('--route'); ctx.globalAlpha = 0.45; ctx.lineWidth = 1.2; ctx.setLineDash([6, 6]); ctx.beginPath();
    const route = sim.circuit.plannedRoute;
    for (let i = 1; i < route.length; i++) line([route[i - 1][0], route[i - 1][1], 0], [route[i][0], route[i][1], 0]);
    ctx.stroke(); ctx.setLineDash([]); ctx.globalAlpha = 1;
    const fp = ac.glideFootprint;
    if (!ac.onGround && sim.phase !== 'FLARE' && fp && fp.r > 30) {
      const ok = sim.reach.reachable === true;
      ctx.strokeStyle = ok ? P.get('--ok') : P.get('--bad'); ctx.lineWidth = 1.5; ctx.setLineDash([4, 5]); ctx.beginPath();
      for (let k = 0; k < 72; k++) {
        const a0 = (k / 72) * 2 * Math.PI, a1 = ((k + 1) / 72) * 2 * Math.PI;
        line([fp.c[0] + fp.r * Math.cos(a0), fp.c[1] + fp.r * Math.sin(a0), 0], [fp.c[0] + fp.r * Math.cos(a1), fp.c[1] + fp.r * Math.sin(a1), 0]);
      }
      ctx.stroke(); ctx.setLineDash([]);
    }

    // --- flown track (3D, coloured by phase) with its ground shadow
    ctx.lineWidth = 1; ctx.strokeStyle = P.get('--shadow3d'); ctx.beginPath();
    for (let i = 1; i < track.length; i++) line([track[i - 1].x, track[i - 1].y, 0], [track[i].x, track[i].y, 0]);
    ctx.stroke();
    ctx.lineWidth = 2.2;
    // from the cockpit, the track right behind the eye would smear across the view: leave the last 150 m out
    const nearEye = (q: { x: number; y: number; h: number }) => cockpit && Math.hypot(q.x - cam[0], q.y - cam[1], q.h - cam[2]) < 150;
    for (let i = 1; i < track.length; i++) {
      const a = track[i - 1], b = track[i];
      if (nearEye(a) || nearEye(b)) continue;
      ctx.strokeStyle = P.phase(b.phase); ctx.beginPath(); line([a.x, a.y, a.h], [b.x, b.y, b.h]); ctx.stroke();
    }
    const last = track[track.length - 1];
    if (last && !cockpit) { ctx.strokeStyle = P.phase(sim.phase as never); ctx.beginPath(); line([last.x, last.y, last.h], pos); ctx.stroke(); }

    if (cockpit) {
      this.drawHud(sim, ctx, w, h, focal);
      return;
    }

    // --- aircraft: nose heading, pitch, bank (+ = left wing down); scaled up when far so it stays visible
    const dcam = Math.hypot(...sub(pos, cam));
    const k = this.mode === 'tower' ? Math.max(1, dcam / 160) : 1;
    const toWorld = (b: V3): V3 => add(pos, bodyToWorld(b, k));
    const faces = MODEL.map(m => {
      const wp = m.pts.map(toWorld);
      const cs = wp.map(cam2);
      const depth = cs.reduce((s, c) => s + c[2], 0) / cs.length;
      return { m, cs, depth };
    }).filter(o => o.cs.every(c => c[2] > near)).sort((a, b) => b.depth - a.depth);
    for (const o of faces) {
      const pts = o.cs.map(scr);
      ctx.beginPath(); pts.forEach((p, i) => (i ? ctx.lineTo(p[0], p[1]) : ctx.moveTo(p[0], p[1]))); ctx.closePath();
      ctx.fillStyle = o.m.role === 'wing' ? P.get('--plane-wing') : o.m.role === 'tail' ? P.get('--plane-tail') : P.get('--plane-body');
      ctx.strokeStyle = P.get('--plane-edge'); ctx.lineWidth = 1; ctx.fill(); ctx.stroke();
    }
    // shadow dot under the aircraft
    const sh = proj([pos[0], pos[1], 0]);
    if (sh && ac.h > 1) { ctx.fillStyle = P.get('--shadow3d'); ctx.beginPath(); ctx.ellipse(sh[0], sh[1], 5, 2.5, 0, 0, 7); ctx.fill(); }

    // --- labels
    ctx.font = `700 11px ${fDisp}`; ctx.fillStyle = P.get('--hud3d'); ctx.textAlign = 'left';
    ctx.fillText(this.mode === 'chase' ? 'CHASE CAMERA · drag to orbit, scroll to zoom' : 'TOWER CAMERA · beside runway 09/27', 12, 18);
    ctx.font = `600 11px ${fMono}`; ctx.textAlign = 'right';
    ctx.fillText(`${Math.round(ac.h / FT)} ft  ${sim.phase.replace(/_/g, ' ')}`, w - 12, 18);
    for (const [x, label] of [[0, '09'], [rwy.length, '27']] as [number, string][]) {
      const p = proj([x, 0, 0.1]); if (p && p[1] > hy - 2) { ctx.textAlign = 'center'; ctx.fillText(label, p[0], p[1] + 14); }
    }
  }

  /** Head-up display over the cockpit view, in HUD green: pitch ladder and horizon rolled with the bank,
   *  flight path marker, bank scale, heading tape, airspeed and altitude tapes, vertical speed, FMA. */
  private drawHud(sim: Frame, ctx: CanvasRenderingContext2D, w: number, h: number, focal: number): void {
    const P = this.pal, ac = sim.aircraft, col = P.get('--hud'), fMono = P.get('--f-mono');
    const D = Math.PI / 180, cx = w / 2, cy0 = h / 2;
    const px = (a: number) => focal * Math.tan(a);
    ctx.save();
    ctx.strokeStyle = col; ctx.fillStyle = col; ctx.lineWidth = 1.6; ctx.font = `600 12px ${fMono}`;
    ctx.shadowColor = 'rgba(0,0,0,0.45)'; ctx.shadowBlur = 2;

    // --- pitch ladder and horizon, rotated by the bank (left wing down turns the world clockwise)
    ctx.save();
    ctx.translate(cx, cy0); ctx.rotate(ac.phi);
    ctx.beginPath(); ctx.rect(-w * 0.22, -h * 0.32, w * 0.44, h * 0.64); ctx.clip();
    for (let a = -30; a <= 30; a += 5) {
      const y = px(ac.theta - a * D);
      if (Math.abs(y) > h) continue;
      const half = a === 0 ? w * 0.2 : a % 10 === 0 ? 60 : 32;
      ctx.beginPath();
      if (a < 0) ctx.setLineDash([6, 5]);
      if (a === 0) { ctx.moveTo(-half, y); ctx.lineTo(half, y); }
      else {
        const tick = a > 0 ? 8 : -8;
        ctx.moveTo(-half, y + tick); ctx.lineTo(-half, y); ctx.lineTo(-14, y);
        ctx.moveTo(14, y); ctx.lineTo(half, y); ctx.lineTo(half, y + tick);
      }
      ctx.stroke(); ctx.setLineDash([]);
      if (a !== 0 && a % 10 === 0) { ctx.textAlign = 'right'; ctx.fillText(String(Math.abs(a)), -half - 6, y + 4); ctx.textAlign = 'left'; ctx.fillText(String(Math.abs(a)), half + 6, y + 4); }
    }
    // flight path marker: where the aircraft is going over the ground (drift and flight-path angle)
    const drift = Math.atan2(Math.sin(ac.track - ac.noseHeading), Math.cos(ac.track - ac.noseHeading));
    const fpa = Math.atan2(ac.verticalSpeed, Math.max(1, ac.groundSpeed));
    const fx = -px(drift), fy = px(ac.theta - fpa);
    ctx.beginPath(); ctx.arc(fx, fy, 7, 0, 2 * Math.PI);
    ctx.moveTo(fx - 7, fy); ctx.lineTo(fx - 18, fy); ctx.moveTo(fx + 7, fy); ctx.lineTo(fx + 18, fy); ctx.moveTo(fx, fy - 7); ctx.lineTo(fx, fy - 13);
    ctx.stroke();
    ctx.restore();

    // --- boresight (where the nose points)
    ctx.beginPath(); ctx.moveTo(cx - 22, cy0); ctx.lineTo(cx - 8, cy0); ctx.lineTo(cx, cy0 + 7); ctx.lineTo(cx + 8, cy0); ctx.lineTo(cx + 22, cy0); ctx.stroke();

    // --- bank scale: arc with ticks at 10, 20, 30, 45, 60°, pointer at the current bank
    const R = Math.min(w, h) * 0.3;
    ctx.beginPath(); ctx.arc(cx, cy0, R, -Math.PI / 2 - 60 * D, -Math.PI / 2 + 60 * D); ctx.stroke();
    for (const b of [-60, -45, -30, -20, -10, 0, 10, 20, 30, 45, 60]) {
      const a = -Math.PI / 2 + b * D, l = b === 0 ? 12 : Math.abs(b) % 30 === 0 ? 10 : 6;
      ctx.beginPath(); ctx.moveTo(cx + R * Math.cos(a), cy0 + R * Math.sin(a)); ctx.lineTo(cx + (R + l) * Math.cos(a), cy0 + (R + l) * Math.sin(a)); ctx.stroke();
    }
    const pa = -Math.PI / 2 + ac.phi;
    ctx.beginPath(); ctx.moveTo(cx + (R - 2) * Math.cos(pa), cy0 + (R - 2) * Math.sin(pa));
    ctx.lineTo(cx + (R - 12) * Math.cos(pa + 0.03), cy0 + (R - 12) * Math.sin(pa + 0.03)); ctx.lineTo(cx + (R - 12) * Math.cos(pa - 0.03), cy0 + (R - 12) * Math.sin(pa - 0.03)); ctx.closePath(); ctx.fill();

    // --- heading tape across the top, compass degrees
    const hdg = ((90 - ac.noseHeading / D) % 360 + 360) % 360;
    const ty = 46, span = 40, pxd = (w * 0.44) / span;
    ctx.save(); ctx.beginPath(); ctx.rect(cx - w * 0.22, ty - 22, w * 0.44, 34); ctx.clip();
    ctx.textAlign = 'center';
    for (let d = Math.floor((hdg - span / 2) / 5) * 5; d <= hdg + span / 2; d += 5) {
      const x = cx + (d - hdg) * pxd;
      ctx.beginPath(); ctx.moveTo(x, ty); ctx.lineTo(x, ty - (d % 10 === 0 ? 9 : 5)); ctx.stroke();
      if (d % 10 === 0) ctx.fillText(String(((d % 360) + 360) % 360 / 10 | 0).padStart(2, '0'), x, ty - 12);
    }
    ctx.restore();
    ctx.beginPath(); ctx.moveTo(cx, ty + 2); ctx.lineTo(cx - 5, ty + 10); ctx.lineTo(cx + 5, ty + 10); ctx.closePath(); ctx.fill();

    // --- speed (left) and altitude (right) tapes
    const tape = (x: number, value: number, step: number, label: string, left: boolean) => {
      const th = h * 0.5, top = cy0 - th / 2, ppu = th / (step * 6);
      ctx.save(); ctx.beginPath(); ctx.rect(x - 34, top, 68, th); ctx.clip();
      ctx.textAlign = left ? 'right' : 'left';
      for (let v = Math.floor((value - step * 3) / step) * step; v <= value + step * 3; v += step) {
        const y = cy0 - (v - value) * ppu;
        ctx.beginPath(); ctx.moveTo(x, y); ctx.lineTo(x + (left ? 8 : -8), y); ctx.stroke();
        if (v >= 0) ctx.fillText(String(v), x + (left ? -4 : 4), y + 4);
      }
      ctx.restore();
      ctx.beginPath(); ctx.moveTo(x, top); ctx.lineTo(x, top + th); ctx.stroke();
      // current value box
      const bw = 52, bx = left ? x - bw - 2 : x + 2;
      ctx.clearRect(bx, cy0 - 11, bw, 22);
      ctx.strokeRect(bx, cy0 - 11, bw, 22);
      ctx.textAlign = 'center'; ctx.font = `700 13px ${fMono}`;
      ctx.fillText(String(Math.round(value)), bx + bw / 2, cy0 + 5);
      ctx.font = `600 12px ${fMono}`; ctx.fillText(label, bx + bw / 2, top - 6);
    };
    tape(cx - w * 0.3, ac.V / KT, 5, 'KT', true);
    tape(cx + w * 0.3, ac.h / FT, 50, 'FT', false);
    ctx.textAlign = 'left';
    ctx.fillText(`VS ${Math.round(ac.verticalSpeed / FT * 60 / 10) * 10}`, cx + w * 0.3 + 4, cy0 + h * 0.25 + 18);

    // --- FMA across the top, configuration and phase along the bottom
    const fma = sim.autopilot.fma;
    ctx.textAlign = 'center'; ctx.font = `700 12px ${fMono}`;
    ctx.fillText(`${fma.thr}   ${fma.vert}   ${fma.lat}`, cx, 18);
    ctx.font = `600 12px ${fMono}`;
    ctx.textAlign = 'left';
    ctx.fillText(`FLAP ${ac.flaps}   THR ${Math.round(ac.thr * 100)}%${ac.engineFailed ? '   ENG OUT' : ''}`, 12, h - 14);
    ctx.textAlign = 'right';
    ctx.fillText(`GS ${Math.round(ac.groundSpeed / KT)}   ${sim.phase.replace(/_/g, ' ')}`, w - 12, h - 14);
    ctx.restore();
  }
}
