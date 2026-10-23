/** The simulation engine: the Rust core compiled to WebAssembly (point-mass and 6DOF models). */
import { FlightSetup, Frame, WindSettings } from './frame';

export interface Engine {
  readonly label: string;
  create(setup: FlightSetup, wind: WindSettings, seed: number): void;
  setWind(w: WindSettings): void;
  /** Advance n 20 ms steps; true once stopped. */
  step(n: number): boolean;
  failEngine(): boolean;
  requestGoAround(): boolean;
  frame(): Frame;
}

interface WasmExports {
  memory: WebAssembly.Memory;
  sim_new(model: number, circuit: number, fromDeg: number, speedKt: number, gusts: number, seed: number, strip: number, shortField: number): void;
  sim_set_wind(fromDeg: number, speedKt: number, gusts: number): void;
  sim_step(n: number): number;
  sim_fail_engine(): number;
  sim_request_go_around(): number;
  sim_frame(): number;
  sim_out_ptr(): number;
}

export class WasmEngine implements Engine {
  readonly label = 'Rust core (WebAssembly)';
  private readonly dec = new TextDecoder();
  private constructor(private readonly x: WasmExports) {}

  static async load(base64: string): Promise<WasmEngine> {
    const bin = atob(base64), bytes = new Uint8Array(bin.length);
    for (let i = 0; i < bin.length; i++) bytes[i] = bin.charCodeAt(i);
    const { instance } = await WebAssembly.instantiate(bytes, {});
    return new WasmEngine(instance.exports as unknown as WasmExports);
  }
  create(f: FlightSetup, w: WindSettings, seed: number): void {
    this.x.sim_new(f.model === '6dof' ? 1 : 0, f.circuit === 'high' ? 1 : 0, w.fromDeg, w.speedKt, w.gusts ? 1 : 0, seed, f.strip, f.shortField ? 1 : 0);
  }
  setWind(w: WindSettings): void { this.x.sim_set_wind(w.fromDeg, w.speedKt, w.gusts ? 1 : 0); }
  step(n: number): boolean { return this.x.sim_step(n) === 1; }
  failEngine(): boolean { return this.x.sim_fail_engine() === 1; }
  requestGoAround(): boolean { return this.x.sim_request_go_around() === 1; }
  frame(): Frame {
    const n = this.x.sim_frame();
    return JSON.parse(this.dec.decode(new Uint8Array(this.x.memory.buffer, this.x.sim_out_ptr(), n)));
  }
}
