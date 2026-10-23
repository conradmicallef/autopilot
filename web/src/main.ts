import { Engine, WasmEngine } from './engine';
import { ENGINE_OUT_PHASES, FlightSetup, Frame, TrackPoint, WindSettings } from './frame';
import { GO_AROUND_INFO, PHASES, PHASE_SEQUENCE, Palette } from './theme';
import { D2R, FT, KT, clamp, headingDeg } from './units';
import { AttitudeIndicator, PlanView, ProfileView } from './views';
import { View3D } from './view3d';

declare const CIRCUITSIM_WASM: string;

const $ = <T extends HTMLElement = HTMLElement>(id: string) => document.getElementById(id) as T;
const mmss = (t: number) => `${Math.floor(t / 60)}:${String(Math.floor(t % 60)).padStart(2, '0')}`;
const DT = 0.02;

class App {
  private warp = 5;
  private setWarp(w: number): void {
    this.warp = w;
    $('warp').querySelectorAll<HTMLButtonElement>('button').forEach(x => x.setAttribute('aria-pressed', String(Number(x.dataset.w) === w)));
  }
  private running = true;
  private acc = 0;
  private last = performance.now();
  private readonly wind: WindSettings = { fromDeg: 90, speedKt: 8, gusts: false };
  private setup: FlightSetup;
  private readonly pal = new Palette();
  private readonly plan = new PlanView($<HTMLCanvasElement>('map'), this.pal);
  private readonly profile = new ProfileView($<HTMLCanvasElement>('prof'), this.pal);
  private readonly adi = new AttitudeIndicator($<HTMLCanvasElement>('adi'), this.pal);
  private readonly v3 = new View3D($<HTMLCanvasElement>('map'), this.pal);
  private view: 'plan' | 'chase' | 'tower' | 'cockpit' = 'plan';
  private track: TrackPoint[] = [];
  private f!: Frame;
  private logCount = -1;
  private critKey = '';

  constructor(private readonly engine: Engine) {
    this.setup = { model: '6dof', circuit: 'high', strip: 1200, shortField: false };
    $('core').textContent = engine.label;
    $('phases').innerHTML = PHASE_SEQUENCE.map(p => `<div class="phase" data-p="${p.phase}" style="--c:var(${p.token})"><i></i>${p.name}</div>`).join('')
      + `<div class="phase ga" data-p="GO_AROUND" style="--c:var(${GO_AROUND_INFO.token})"><i></i><span id="gaChip">Go-around</span></div>`
      + `<div class="phase ga" data-p="ENGINE_OUT" style="--c:var(--ph-eo)"><i></i><span id="eoChip">Engine out</span></div>`;
    this.bindControls();
    this.syncWind();
    this.reset();
    requestAnimationFrame(t => { this.last = t; this.frame(t); });
  }

  private reset(): void {
    this.engine.create(this.setup, { ...this.wind }, (Math.random() * 1e6) | 1);
    this.track = []; this.acc = 0; this.logCount = -1; this.critKey = '';
    this.f = this.engine.frame(); this.track.push(...this.f.newTrack);
    $('modelNote').textContent = '6DOF rigid body on the JSBSim Cessna 172P aerodynamic tables (stall hysteresis, ground effect, slipstream over the tail) and gear, RK4 at 200 Hz.';
    $('surfaces').hidden = false;
  }

  private syncToggle(groupId: string, key: string, value: string): void {
    $(groupId).querySelectorAll<HTMLButtonElement>('button').forEach(x => x.setAttribute('aria-pressed', String(x.dataset[key] === value)));
  }

  private bindControls(): void {
    $('run').onclick = () => {
      if (this.f.finished) { this.reset(); this.running = true; }
      else this.running = !this.running;
      this.last = performance.now(); this.updateRunLabel();
    };
    $('reset').onclick = () => { this.reset(); this.running = true; this.updateRunLabel(); };
    $('ga').onclick = () => { this.engine.requestGoAround(); };
    // an engine failure drops to real time, so the glide and the decisions can be followed
    $('eng').onclick = () => { this.engine.failEngine(); this.setWarp(1); };
    // auto-land from a random state: a new flight that starts in the air, possibly upset, at real time
    const randomStart = (engineOut: boolean) => {
      this.reset();
      this.engine.randomStart((Math.random() * 1e6) | 1, engineOut);
      this.track = []; this.logCount = -1; this.critKey = '';
      this.f = this.engine.frame(); this.track.push(...this.f.newTrack);
      this.setWarp(1); this.running = true; this.last = performance.now(); this.updateRunLabel();
    };
    $('rnd').onclick = () => randomStart(false);
    $('rndEo').onclick = () => randomStart(true);
    $('warp').addEventListener('click', e => {
      const b = (e.target as HTMLElement).closest('button'); if (!b) return;
      this.setWarp(Number(b.dataset.w));
    });
    // view switch: plan chart or one of the 3D cameras, on the same canvas; the flight carries on
    $('view').addEventListener('click', e => {
      const b = (e.target as HTMLElement).closest('button'); if (!b) return;
      this.view = b.dataset.v as 'plan' | 'chase' | 'tower' | 'cockpit';
      this.syncToggle('view', 'v', this.view);
      this.v3.active = this.view !== 'plan';
      // dragging orbits the 3D chase camera; elsewhere the page keeps scrolling on touch
      $('map').style.touchAction = this.view === 'chase' ? 'none' : 'auto';
      if (this.view !== 'plan') this.v3.mode = this.view;
      $('viewLabel').textContent = this.view === 'plan' ? 'Plan view' : this.view === 'cockpit' ? 'Cockpit' : '3D view';
      $('viewHint').textContent = this.view === 'plan'
        ? 'Dashed: planned circuit · Solid: flown track, coloured by phase · Dotted ring: glide footprint (green = runway in power-off reach)'
        : this.view === 'cockpit'
          ? 'Head-up display: pitch ladder, flight path marker (circle: where the aircraft is going), bank scale, heading, speed and altitude tapes, modes'
          : 'Track coloured by phase, with its ground shadow · Dotted ring on the ground: glide footprint · Dashed: planned circuit';
    });
    for (const id of ['wdir', 'wspd', 'gusts']) $(id).addEventListener('input', () => this.syncWind());
    // each of these starts a new flight with the changed setup
    const toggle = (id: string, key: string, apply: (v: string) => void) => $(id).addEventListener('click', e => {
      const b = (e.target as HTMLElement).closest('button'); if (!b || b.disabled) return;
      apply(b.dataset[key]!);
      this.syncToggle(id, key, b.dataset[key]!);
      this.reset(); this.running = true; this.updateRunLabel();
    });
    toggle('profile', 'c', v => { this.setup.circuit = v === 'standard' ? 'standard' : 'high'; });
    toggle('strip', 's', v => { this.setup.strip = Number(v); });
    toggle('proc', 'p', v => { this.setup.shortField = v === 'short'; });
  }

  private updateRunLabel(): void {
    $('run').textContent = this.f.finished ? 'Fly again' : this.running ? 'Pause' : 'Resume';
  }

  private syncWind(): void {
    this.wind.fromDeg = Number($<HTMLInputElement>('wdir').value);
    this.wind.speedKt = Number($<HTMLInputElement>('wspd').value);
    this.wind.gusts = $<HTMLInputElement>('gusts').checked;
    $('wdirV').textContent = String(this.wind.fromDeg).padStart(3, '0') + '°';
    $('wspdV').textContent = this.wind.speedKt + ' kt';
    if (this.f) this.engine.setWind({ ...this.wind }); // live: the wind can change mid-flight
  }

  private frame(now: number): void {
    const real = Math.min(0.1, (now - this.last) / 1000); this.last = now;
    if (this.running && !this.f.finished) {
      this.acc += real * this.warp;
      const n = Math.floor(this.acc / DT);
      if (n > 0) { this.engine.step(n); this.acc -= n * DT; }
    }
    this.f = this.engine.frame();
    if (this.f.newTrack.length) this.track.push(...this.f.newTrack);
    if (this.f.finished && this.running) { this.running = false; this.updateRunLabel(); }
    if (this.view === 'plan') this.plan.draw(this.f, this.track); else this.v3.draw(this.f, this.track);
    this.profile.draw(this.f, this.track); this.adi.draw(this.f);
    this.updateInstruments(); this.renderLog(); this.renderCheck();
    requestAnimationFrame(t => this.frame(t));
  }

  private updateInstruments(): void {
    const s = this.f, ac = s.aircraft, ap = s.autopilot, phase = s.phase;
    const vsFpm = (ac.verticalSpeed / FT) * 60;
    const set = (id: string, v: string, unit: string) => { $(id).innerHTML = `${v}<em>${unit}</em>`; };
    set('rIas', (ac.V / KT).toFixed(0), 'kt');
    set('rGs', (ac.groundSpeed / KT).toFixed(0), 'kt');
    set('rAlt', (ac.h / FT).toFixed(0), 'ft');
    set('rVs', (vsFpm > 5 ? '+' : '') + (Math.round(vsFpm / 10) * 10).toFixed(0), 'fpm');
    const deg3 = (a: number) => String(Math.round(headingDeg(a)) % 360).padStart(3, '0');
    set('rHdg', deg3(ac.noseHeading), '°');
    set('rTrk', deg3(ac.track), '°');
    const slipSide = Math.abs(ac.beta) < 0.05 * D2R ? '' : ac.beta > 0 ? ' L' : ' R';
    set('rSlip', (Math.abs(ac.beta) / D2R).toFixed(1), '°' + slipSide);
    set('rFlaps', String(ac.flaps), '°');
    $('bThr').style.width = ac.thr * 100 + '%'; $('vThr').textContent = Math.round(ac.thr * 100) + '%';
    $('bAoa').style.width = clamp(ac.alpha / (16 * D2R), 0, 1) * 100 + '%'; $('vAoa').textContent = (ac.alpha / D2R).toFixed(1) + '°';
    if (ac.six) {
      // centred bars: elevator shown as stick (nose-up = right), aileron + right roll, rudder + nose right
      const bar = (id: string, v: number, max: number) => {
        const k = clamp(v / max, -1, 1), el = $(id);
        el.style.left = (50 + Math.min(0, k) * 50) + '%'; el.style.width = Math.abs(k) * 50 + '%';
      };
      bar('bEle', -ac.six.de, 25 * D2R); $('vEle').textContent = (-ac.six.de / D2R).toFixed(1) + '°';
      bar('bAil', ac.six.da, 20 * D2R); $('vAil').textContent = (ac.six.da / D2R).toFixed(1) + '°';
      bar('bRud', ac.six.dr, 25 * D2R); $('vRud').textContent = (ac.six.dr / D2R).toFixed(1) + '°';
      const l = ac.six.loads;
      $('vGear').textContent = ac.six.loads.some(v => v > 50) ? `${Math.round(l[0] / 9.81)}/${Math.round(l[1] / 9.81)}/${Math.round(l[2] / 9.81)} kg` : 'airborne';
    }
    $('fmaThr').textContent = ap.fma.thr; $('fmaVert').textContent = ap.fma.vert; $('fmaLat').textContent = ap.fma.lat;
    const eoPhase = ENGINE_OUT_PHASES.includes(phase) || !!ap.engineOut;
    $('fmaVert').classList.toggle('amber', phase === 'GO_AROUND' || eoPhase);
    $('fmaThr').classList.toggle('red', !!ap.engineOut);
    $('phaseNow').textContent = PHASES[phase].name.toUpperCase() + (phase === 'DOWNWIND' && ap.descending ? ' · DESC' : '');
    $('phaseNow').classList.toggle('amber', phase === 'GO_AROUND' || ENGINE_OUT_PHASES.includes(phase));
    $('clock').textContent = mmss(s.t).padStart(5, '0');
    const eoPlan = ap.engineOut ? { REJECT: 'REJECT TAKE-OFF', LAND_AHEAD: 'LAND AHEAD', RETURN: 'GLIDE TO RUNWAY' }[ap.engineOut.plan] : '';
    $('warn').textContent = ac.stalled ? 'STALL'
      : ap.engineOut && !s.finished ? `ENGINE FAILURE · ${eoPlan}`
      : phase === 'GO_AROUND' ? `GO AROUND · ${ap.goAroundReason.toUpperCase()}`
      : ac.alpha > 13 * D2R && !ac.onGround ? 'HIGH AOA' : '';
    const canGa = phase === 'BASE' || phase === 'FINAL' || phase === 'FLARE';
    const reach = $('reach');
    reach.textContent = s.reach.reachable === null ? (ap.engineOut ? '—' : ac.onGround ? 'ON GROUND' : '—')
      : s.reach.reachable ? `RWY ${s.reach.end} ✓` : 'NONE';
    reach.className = s.reach.reachable === null ? '' : s.reach.reachable ? 'ok' : 'no';
    $('reachPct').textContent = s.reach.airTime > 1 ? `${Math.round(s.reach.fraction * 100)}% of flight` : '';
    $<HTMLButtonElement>('ga').disabled = !canGa || ap.goAroundRequested || !!ap.engineOut;
    $<HTMLButtonElement>('eng').disabled = !!ap.engineOut || s.finished || phase === 'ROLLOUT';

    const cur = PHASES[phase].index, stopped = s.finished;
    document.querySelectorAll<HTMLElement>('.phase').forEach(el => {
      if (el.dataset.p === 'ENGINE_OUT') {
        el.hidden = !ap.engineOut;
        el.classList.toggle('now', ENGINE_OUT_PHASES.includes(phase));
        el.classList.toggle('done', !!ap.engineOut && !ENGINE_OUT_PHASES.includes(phase));
        $('eoChip').textContent = ENGINE_OUT_PHASES.includes(phase) ? PHASES[phase].name : 'Engine out';
        return;
      }
      const p = el.dataset.p as keyof typeof PHASES;
      if (p === 'GO_AROUND') {
        el.classList.toggle('now', phase === 'GO_AROUND');
        el.classList.toggle('done', ap.goArounds > 0 && phase !== 'GO_AROUND');
        el.hidden = ap.goArounds === 0;
        $('gaChip').textContent = ap.goArounds > 1 ? `Go-around ×${ap.goArounds}` : 'Go-around';
        return;
      }
      const i = PHASES[p].index, isNow = phase !== 'GO_AROUND' && !ENGINE_OUT_PHASES.includes(phase) && i === cur && !stopped;
      el.classList.toggle('done', stopped || i < cur);
      el.classList.toggle('now', isNow);
    });
  }

  private renderLog(): void {
    const ev = this.f.events; if (ev.length === this.logCount) return; this.logCount = ev.length;
    $('log').innerHTML = ev.map(e => {
      // phase changes, plus marker events that are not phases (an unknown one must never break the page)
      const info = (PHASES as Record<string, { name: string; token: string } | undefined>)[e.phase];
      const marker: Record<string, [string, string]> = {
        DESCENT: ['Begin descent', '--muted'], ENGINE_FAILURE: ['Engine failure', '--ph-eo'], STRETCH: ['Stretch to runway', '--ph-eo'],
      REPLAN: ['New glide plan', '--ph-eo'], EXTEND: ['Longer path (high)', '--ph-eo'], ORBIT: ['Orbit down (high)', '--ph-eo']
      };
      const [name, tok] = info ? [info.name, info.token] : marker[e.phase] ?? [e.phase.replace(/_/g, ' ').toLowerCase(), '--muted'];
      return `<span class="mut l">${mmss(e.t)}</span><span class="ph" style="color:var(${tok})">${name}${e.note ? `<small>${e.note}</small>` : ''}</span>`
        + `<span class="mut">${Math.round(e.h / FT)} ft</span><span class="mut">${Math.round(e.V / KT)} kt</span>`;
    }).join('');
    $('log').scrollTop = 1e6;
  }

  private renderCheck(): void {
    const s = this.f, rows = s.evaluation, ap = s.autopilot;
    const key = JSON.stringify(rows) + ap.goArounds + s.phase; if (key === this.critKey) return; this.critKey = key;
    $('crit').innerHTML = rows.map(r => {
      const cls = r.info ? 'info' : r.pass === true ? 'ok' : r.pass === false ? 'bad' : '';
      const txt = r.info ? 'Info' : r.pass === true ? 'Pass' : r.pass === false ? 'Fail' : 'Pending';
      return `<div class="crit"><span class="k">${r.label}</span><span class="c">${r.info ? '' : 'Target '}${r.target}</span><span class="v">${r.value ?? '—'}</span><span class="chip ${cls}">${txt}</span></div>`;
    }).join('') + (ap.engineOut ? '' : `<div class="crit"><span class="k">Go-arounds</span><span class="c">Unstable approaches rejected</span><span class="v">${ap.goArounds}${ap.goAroundReason ? ' · last: ' + ap.goAroundReason : ''}</span><span class="chip info">Info</span></div>`);
    const v = $('verdict'), done = s.finished;
    if (s.phase === 'REJECTED') { v.textContent = 'Stopping'; v.style.color = 'var(--muted)'; }
    else if (!s.touchdown && !done) { v.textContent = s.phase === 'GO_AROUND' ? 'Going around' : ap.engineOut ? 'Gliding' : 'In flight'; v.style.color = 'var(--muted)'; }
    else if (!done) { v.textContent = 'Rolling out'; v.style.color = 'var(--muted)'; }
    else { v.textContent = s.passed ? 'All passed' : 'Check failed'; v.style.color = s.passed ? 'var(--ok)' : 'var(--bad)'; }
  }
}

async function boot(): Promise<void> {
  let engine: Engine;
  try {
    engine = await WasmEngine.load(CIRCUITSIM_WASM);
  } catch (err) {
    console.error('WebAssembly core failed to start:', err);
    $('core').textContent = 'simulation core failed to start';
    $('startError').hidden = false;
    return;
  }
  new App(engine);
}
boot();
