/** The JSON frame the simulation core sends to the page each animation frame (see rust/src/wasm.rs). */

export type Phase =
  | 'TAKEOFF_ROLL' | 'ROTATE' | 'CLIMB' | 'CROSSWIND' | 'DOWNWIND'
  | 'BASE' | 'FINAL' | 'FLARE' | 'ROLLOUT' | 'STOPPED' | 'GO_AROUND'
  | 'REJECTED' | 'EO_GLIDE' | 'EO_FINAL' | 'FORCED_LANDING' | 'RECOVER' | 'REJOIN';

export const ENGINE_OUT_PHASES: Phase[] = ['REJECTED', 'EO_GLIDE', 'EO_FINAL', 'FORCED_LANDING'];

export type Point = [number, number];

export interface TrackPoint { t: number; x: number; y: number; h: number; V: number; phase: Phase; reach: boolean | null; }
export interface ModeEvent { t: number; phase: Phase | 'DESCENT' | 'ENGINE_FAILURE'; h: number; V: number; x: number; y: number; note?: string | null; }
export interface Criterion { label: string; target: string; value?: string | null; pass?: boolean | null; info?: boolean; }
export interface Touchdown { t: number; x: number; y: number; sink: number; V: number; crab: number; onRunway: boolean; }
export interface EngineOut { plan: 'REJECT' | 'LAND_AHEAD' | 'RETURN'; why: string; t: number; x: number; y: number; h: number; track: number; }

export interface Frame {
  t: number;
  phase: Phase;
  finished: boolean;
  model: string;
  aircraft: {
    x: number; y: number; h: number; V: number; Vg: number;
    psi: number; beta: number; gamma: number; theta: number; phi: number;
    thr: number; flapPos: number; flaps: number; alpha: number; verticalSpeed: number;
    noseHeading: number; track: number; groundSpeed: number;
    stalled: boolean; onGround: boolean; engineFailed: boolean; stillAirReach: number; glideFootprint: { c: Point; r: number };
    /** Present only for the 6DOF model: control-surface positions (rad) and gear loads (N). */
    six: { de: number; da: number; dr: number; steer: number; loads: [number, number, number, number]; alpha: number; beta: number } | null;
  };
  autopilot: {
    fma: { thr: string; vert: string; lat: string };
    descending: boolean;
    goArounds: number;
    goAroundReason: string;
    goAroundRequested: boolean;
    engineOut: EngineOut | null;
    glide: { route: Point[]; ipDistance: number; end: string; reachable: boolean } | null;
    eoEnd: { name: string; heading: number; threshold: number; dir: number; aim: Point };
  };
  wind: { fromDeg: number; speedKt: number; gusts: boolean; vx: number; vy: number };
  circuit: { name: string; plannedRoute: Point[]; baseX: number; downwindOffset: number };
  runway: { length: number; width: number; aimPoint: number; obstacles: boolean; obstacleOffset: number; obstacleHeight: number };
  procedure: string;
  perf: { liftoff_x: number | null; takeoff_50ft_x: number | null; departure_obstacle_h: number | null; landing_50ft_x: number | null; approach_obstacle_h: number | null };
  touchdown: Touchdown | null;
  reach: { reachable: boolean | null; end: string | null; fraction: number; airTime: number };
  events: ModeEvent[];
  /** Track points recorded since the previous frame. */
  newTrack: TrackPoint[];
  evaluation: Criterion[];
  passed: boolean;
}

export interface WindSettings { fromDeg: number; speedKt: number; gusts: boolean; }
export type ModelChoice = 'pm' | '6dof';
export type CircuitChoice = 'standard' | 'high';
export interface FlightSetup { model: ModelChoice; circuit: CircuitChoice; strip: number; shortField: boolean; }
