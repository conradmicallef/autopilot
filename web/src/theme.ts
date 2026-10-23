import { Phase } from './frame';

export interface PhaseInfo { phase: Phase; name: string; token: string; }

/** Display order of the normal circuit; GO_AROUND is a branch shown separately. */
export const PHASE_SEQUENCE: PhaseInfo[] = [
  { phase: 'TAKEOFF_ROLL', name: 'Takeoff roll', token: '--ph-ground' },
  { phase: 'ROTATE', name: 'Rotate', token: '--ph-ground' },
  { phase: 'CLIMB', name: 'Climb', token: '--ph-climb' },
  { phase: 'CROSSWIND', name: 'Crosswind', token: '--ph-xwind' },
  { phase: 'DOWNWIND', name: 'Downwind', token: '--ph-down' },
  { phase: 'BASE', name: 'Base', token: '--ph-base' },
  { phase: 'FINAL', name: 'Final', token: '--ph-final' },
  { phase: 'FLARE', name: 'Flare', token: '--ph-flare' },
  { phase: 'ROLLOUT', name: 'Rollout', token: '--ph-ground' },
  { phase: 'STOPPED', name: 'Stopped', token: '--ph-ground' }
];
export const GO_AROUND_INFO: PhaseInfo = { phase: 'GO_AROUND', name: 'Go-around', token: '--ph-ga' };
/** Engine-out branch phases; `index` places them on the main strip for progress shading. */
export const ENGINE_OUT_INFO: (PhaseInfo & { index: number })[] = [
  { phase: 'REJECTED', name: 'Rejected take-off', token: '--ph-eo', index: 1 },
  { phase: 'EO_GLIDE', name: 'Glide to runway', token: '--ph-eo', index: 5 },
  { phase: 'EO_FINAL', name: 'Power-off final', token: '--ph-eo', index: 6 },
  { phase: 'FORCED_LANDING', name: 'Forced landing', token: '--ph-eo', index: 6 },
  // random-state starts: recovery, then a powered path to final (shaded like base)
  { phase: 'RECOVER', name: 'Upset recovery', token: '--ph-ga', index: 4 },
  { phase: 'REJOIN', name: 'Rejoin to final', token: '--ph-base', index: 5 }
];

export const PHASES: Record<Phase, PhaseInfo & { index: number }> = Object.fromEntries([
  ...[...PHASE_SEQUENCE, GO_AROUND_INFO].map((p, i) => [p.phase, { ...p, index: p.phase === 'GO_AROUND' ? 2 : i }]),
  ...ENGINE_OUT_INFO.map(p => [p.phase, p])
]) as Record<Phase, PhaseInfo & { index: number }>;

const TOKENS = ['--paper', '--panel', '--ink', '--muted', '--line', '--route', '--runway', '--rwmark', '--ok', '--bad',
  '--f-display', '--f-mono', '--ph-eo', '--sky-top', '--sky-low', '--earth', '--grid3d', '--runway3d', '--rwmark3d', '--tree',
  '--shadow3d', '--plane-wing', '--plane-body', '--plane-tail', '--plane-edge', '--hud3d', '--hud', ...new Set([...PHASE_SEQUENCE, GO_AROUND_INFO].map(p => p.token))];

/** Canvas can't read CSS variables, so resolved token values are cached and refreshed on theme change. */
export class Palette {
  private values: Record<string, string> = {};
  constructor() {
    this.refresh();
    window.matchMedia?.('(prefers-color-scheme: dark)').addEventListener?.('change', () => this.refresh());
    new MutationObserver(() => this.refresh()).observe(document.documentElement, { attributes: true, attributeFilter: ['data-theme'] });
    document.fonts?.ready.then(() => this.refresh());
  }
  refresh(): void {
    const cs = getComputedStyle(document.documentElement);
    for (const t of TOKENS) this.values[t] = cs.getPropertyValue(t).trim();
  }
  get(token: string): string { return this.values[token] ?? '#888'; }
  phase(p: Phase): string { return this.get(PHASES[p]?.token ?? '--ph-eo'); }
}

/** Sizes a canvas to its CSS box at device pixel ratio and returns a context in CSS pixels. */
export function fitCanvas(cv: HTMLCanvasElement): { ctx: CanvasRenderingContext2D; w: number; h: number } {
  const dpr = Math.min(window.devicePixelRatio || 1, 2), r = cv.getBoundingClientRect();
  const w = Math.max(10, Math.round(r.width)), h = Math.max(10, Math.round(r.height));
  if (cv.width !== w * dpr || cv.height !== h * dpr) { cv.width = w * dpr; cv.height = h * dpr; }
  const ctx = cv.getContext('2d')!;
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  return { ctx, w, h };
}
