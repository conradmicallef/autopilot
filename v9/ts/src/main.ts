import { WindSettings } from './atmosphere';
import { ENGINE_OUT_PHASES } from './autopilot';
import { CircuitSpec, HIGH_CIRCUIT, STANDARD_CIRCUIT } from './airfield';
import { Simulation } from './simulation';
import { D2R, FT, KT, clamp, headingDeg } from './units';
import { GO_AROUND_INFO, PHASES, PHASE_SEQUENCE, Palette } from './ui/theme';
import { AttitudeIndicator, PlanView, ProfileView } from './ui/views';

const $ = <T extends HTMLElement = HTMLElement>(id: string) => document.getElementById(id) as T;
const mmss = (t: number) => `${Math.floor(t / 60)}:${String(Math.floor(t % 60)).padStart(2, '0')}`;

class App {
  private sim!: Simulation;
  private warp = 5;
  private running = true;
  private acc = 0;
  private last = performance.now();
  private readonly wind: WindSettings = { fromDeg: 270, speedKt: 8, gusts: false };
  private circuit: CircuitSpec = HIGH_CIRCUIT;
  private readonly pal = new Palette();
  private readonly plan = new PlanView($<HTMLCanvasElement>('map'), this.pal);
  private readonly profile = new ProfileView($<HTMLCanvasElement>('prof'), this.pal);
  private readonly adi = new AttitudeIndicator($<HTMLCanvasElement>('adi'), this.pal);
  private logCount = -1;
  private critKey = '';

  constructor() {
    $('phases').innerHTML = PHASE_SEQUENCE.map(p => `<div class="phase" data-p="${p.phase}" style="--c:var(${p.token})"><i></i>${p.name}</div>`).join('')
      + `<div class="phase ga" data-p="GO_AROUND" style="--c:var(${GO_AROUND_INFO.token})"><i></i><span id="gaChip">Go-around</span></div>`
      + `<div class="phase ga" data-p="ENGINE_OUT" style="--c:var(--ph-eo)"><i></i><span id="eoChip">Engine out</span></div>`;
    this.bindControls();
    this.syncWind();
    this.reset();
    requestAnimationFrame(t => { this.last = t; this.frame(t); });
  }

  private reset(): void {
    this.sim = new Simulation({ wind: { ...this.wind }, seed: (Math.random() * 1e6) | 1, circuit: this.circuit });
    this.acc = 0; this.logCount = -1; this.critKey = '';
  }

  private bindControls(): void {
    $('run').onclick = () => {
      if (this.sim.finished) { this.reset(); this.running = true; }
      else this.running = !this.running;
      this.last = performance.now(); this.updateRunLabel();
    };
    $('reset').onclick = () => { this.reset(); this.running = true; this.updateRunLabel(); };
    $('ga').onclick = () => { this.sim.requestGoAround(); };
    $('eng').onclick = () => { this.sim.failEngine(); };
    $('warp').addEventListener('click', e => {
      const b = (e.target as HTMLElement).closest('button'); if (!b) return;
      this.warp = Number(b.dataset.w);
      $('warp').querySelectorAll('button').forEach(x => x.setAttribute('aria-pressed', String(x === b)));
    });
    for (const id of ['wdir', 'wspd', 'gusts']) $(id).addEventListener('input', () => this.syncWind());
    $('profile').addEventListener('click', e => {
      const b = (e.target as HTMLElement).closest('button'); if (!b) return;
      this.circuit = b.dataset.c === 'standard' ? STANDARD_CIRCUIT : HIGH_CIRCUIT;
      $('profile').querySelectorAll('button').forEach(x => x.setAttribute('aria-pressed', String(x === b)));
      this.reset(); this.running = true; this.updateRunLabel();   // a new profile means a new flight
    });
  }

  private updateRunLabel(): void {
    $('run').textContent = this.sim.finished ? 'Fly again' : this.running ? 'Pause' : 'Resume';
  }

  private syncWind(): void {
    this.wind.fromDeg = Number($<HTMLInputElement>('wdir').value);
    this.wind.speedKt = Number($<HTMLInputElement>('wspd').value);
    this.wind.gusts = $<HTMLInputElement>('gusts').checked;
    $('wdirV').textContent = String(this.wind.fromDeg).padStart(3, '0') + '°';
    $('wspdV').textContent = this.wind.speedKt + ' kt';
    if (this.sim) this.sim.wind.settings = { ...this.wind };   // live: the wind can change mid-flight
  }

  private frame(now: number): void {
    const real = Math.min(0.1, (now - this.last) / 1000); this.last = now;
    if (this.running && !this.sim.finished) {
      this.acc += real * this.warp;
      while (this.acc >= Simulation.DT) { this.sim.step(); this.acc -= Simulation.DT; }
      if (this.sim.finished) { this.running = false; this.updateRunLabel(); }
    }
    this.plan.draw(this.sim); this.profile.draw(this.sim); this.adi.draw(this.sim);
    this.updateInstruments(); this.renderLog(); this.renderCheck();
    requestAnimationFrame(t => this.frame(t));
  }

  private updateInstruments(): void {
    const s = this.sim, ac = s.aircraft, ap = s.autopilot, gv = ac.groundVelocity(s.wind);
    const vsFpm = (ac.verticalSpeed / FT) * 60;
    const set = (id: string, v: string, unit: string) => { $(id).innerHTML = `${v}<em>${unit}</em>`; };
    set('rIas', (ac.V / KT).toFixed(0), 'kt');
    set('rGs', (Math.hypot(gv.x, gv.y) / KT).toFixed(0), 'kt');
    set('rAlt', (ac.h / FT).toFixed(0), 'ft');
    set('rVs', (vsFpm > 5 ? '+' : '') + (Math.round(vsFpm / 10) * 10).toFixed(0), 'fpm');
    const deg3 = (a: number) => String(Math.round(headingDeg(a)) % 360).padStart(3, '0');
    set('rHdg', deg3(ac.noseHeading), '°');
    set('rTrk', deg3(ac.track(s.wind)), '°');
    const slipSide = Math.abs(ac.beta) < 0.05 * D2R ? '' : ac.beta > 0 ? ' L' : ' R';   // which way the nose is yawed
    set('rSlip', (Math.abs(ac.beta) / D2R).toFixed(1), '°' + slipSide);
    set('rFlaps', String(ac.flaps), '°');
    $('bThr').style.width = ac.thr * 100 + '%'; $('vThr').textContent = Math.round(ac.thr * 100) + '%';
    $('bAoa').style.width = clamp(ac.alpha / (16 * D2R), 0, 1) * 100 + '%'; $('vAoa').textContent = (ac.alpha / D2R).toFixed(1) + '°';
    $('fmaThr').textContent = ap.fma.thr; $('fmaVert').textContent = ap.fma.vert; $('fmaLat').textContent = ap.fma.lat;
    const eoPhase = ENGINE_OUT_PHASES.includes(ap.phase) || !!ap.engineOut;
    $('fmaVert').classList.toggle('amber', ap.phase === 'GO_AROUND' || eoPhase);
    $('fmaThr').classList.toggle('red', !!ap.engineOut);
    $('phaseNow').textContent = PHASES[ap.phase].name.toUpperCase() + (ap.phase === 'DOWNWIND' && ap.descending ? ' · DESC' : '');
    $('phaseNow').classList.toggle('amber', ap.phase === 'GO_AROUND' || ENGINE_OUT_PHASES.includes(ap.phase));
    $('clock').textContent = mmss(s.t).padStart(5, '0');
    const eoPlan = ap.engineOut ? { REJECT: 'REJECT TAKE-OFF', LAND_AHEAD: 'LAND AHEAD', RETURN: 'GLIDE TO RUNWAY' }[ap.engineOut.plan] : '';
    $('warn').textContent = ac.stalled ? 'STALL'
      : ap.engineOut && !s.finished ? `ENGINE FAILURE · ${eoPlan}`
      : ap.phase === 'GO_AROUND' ? `GO AROUND · ${ap.goAroundReason.toUpperCase()}`
      : ac.alpha > 13 * D2R && !ac.onGround ? 'HIGH AOA' : '';
    const canGa = ap.phase === 'BASE' || ap.phase === 'FINAL' || ap.phase === 'FLARE';
    const reach = $('reach');
    reach.textContent = s.runwayReachable === null ? (ap.engineOut ? '—' : ac.onGround ? 'ON GROUND' : '—')
      : s.runwayReachable ? `RWY ${s.reachEnd} ✓` : 'NONE';
    reach.className = s.runwayReachable === null ? '' : s.runwayReachable ? 'ok' : 'no';
    $('reachPct').textContent = s.airTime > 1 ? `${Math.round(s.reachFraction * 100)}% of flight` : '';
    $<HTMLButtonElement>('ga').disabled = !canGa || ap.goAroundRequested || !!ap.engineOut;
    $<HTMLButtonElement>('eng').disabled = !!ap.engineOut || s.finished || ap.phase === 'ROLLOUT';

    const cur = PHASES[ap.phase].index, stopped = s.finished;
    document.querySelectorAll<HTMLElement>('.phase').forEach(el => {
      if (el.dataset.p === 'ENGINE_OUT') {
        el.hidden = !ap.engineOut;
        el.classList.toggle('now', ENGINE_OUT_PHASES.includes(ap.phase));
        el.classList.toggle('done', !!ap.engineOut && !ENGINE_OUT_PHASES.includes(ap.phase));
        $('eoChip').textContent = ENGINE_OUT_PHASES.includes(ap.phase) ? PHASES[ap.phase].name : 'Engine out';
        return;
      }
      const p = el.dataset.p as keyof typeof PHASES;
      if (p === 'GO_AROUND') {
        el.classList.toggle('now', ap.phase === 'GO_AROUND');
        el.classList.toggle('done', ap.goArounds > 0 && ap.phase !== 'GO_AROUND');
        el.hidden = ap.goArounds === 0;
        $('gaChip').textContent = ap.goArounds > 1 ? `Go-around ×${ap.goArounds}` : 'Go-around';
        return;
      }
      const i = PHASES[p].index, isNow = ap.phase !== 'GO_AROUND' && !ENGINE_OUT_PHASES.includes(ap.phase) && i === cur && !stopped;
      el.classList.toggle('done', stopped || i < cur);
      el.classList.toggle('now', isNow);
    });
  }

  private renderLog(): void {
    const ev = this.sim.events; if (ev.length === this.logCount) return; this.logCount = ev.length;
    $('log').innerHTML = ev.map(e => {
      const info = e.phase === 'DESCENT' || e.phase === 'ENGINE_FAILURE' ? null : PHASES[e.phase];
      const name = info ? info.name : e.phase === 'ENGINE_FAILURE' ? 'Engine failure' : 'Begin descent';
      const tok = info ? info.token : e.phase === 'ENGINE_FAILURE' ? '--ph-eo' : '--muted';
      return `<span class="mut l">${mmss(e.t)}</span><span class="ph" style="color:var(${tok})">${name}${e.note ? `<small>${e.note}</small>` : ''}</span>`
        + `<span class="mut">${Math.round(e.h / FT)} ft</span><span class="mut">${Math.round(e.V / KT)} kt</span>`;
    }).join('');
    $('log').scrollTop = 1e6;
  }

  private renderCheck(): void {
    const rows = this.sim.evaluate(), ap = this.sim.autopilot;
    const key = JSON.stringify(rows) + ap.goArounds + this.sim.phase; if (key === this.critKey) return; this.critKey = key;
    $('crit').innerHTML = rows.map(r => {
      const cls = r.info ? 'info' : r.pass === true ? 'ok' : r.pass === false ? 'bad' : '';
      const txt = r.info ? 'Info' : r.pass === true ? 'Pass' : r.pass === false ? 'Fail' : 'Pending';
      return `<div class="crit"><span class="k">${r.label}</span><span class="c">${r.info ? '' : 'Target '}${r.target}</span><span class="v">${r.value ?? '—'}</span><span class="chip ${cls}">${txt}</span></div>`;
    }).join('') + (ap.engineOut ? '' : `<div class="crit"><span class="k">Go-arounds</span><span class="c">Unstable approaches rejected</span><span class="v">${ap.goArounds}${ap.goAroundReason ? ' · last: ' + ap.goAroundReason : ''}</span><span class="chip info">Info</span></div>`);
    const v = $('verdict'), done = this.sim.finished;
    if (ap.phase === 'REJECTED') { v.textContent = 'Stopping'; v.style.color = 'var(--muted)'; }
    else if (!this.sim.touchdown && !done) { v.textContent = ap.phase === 'GO_AROUND' ? 'Going around' : ap.engineOut ? 'Gliding' : 'In flight'; v.style.color = 'var(--muted)'; }
    else if (!done) { v.textContent = 'Rolling out'; v.style.color = 'var(--muted)'; }
    else { const all = rows.every(r => r.info || r.pass); v.textContent = all ? 'All passed' : 'Check failed'; v.style.color = all ? 'var(--ok)' : 'var(--bad)'; }
  }
}

new App();
