# Circuit Autopilot Sim

A C172-class aircraft flies a traffic circuit under autopilot: take-off, circuit, approach and landing. It goes around when the approach is unstable, straightens up for crosswind landings, and handles engine failures.

The simulation core is written in Rust and runs natively, for test suites and Monte Carlo batches, or in the browser as WebAssembly. Two flight models sit behind the same autopilot:

- **6DOF**: a rigid body with aerodynamic forces and moments, control surfaces, propeller torque and slipstream, and spring–damper landing gear, integrated with RK4 at 200 Hz.
- **Point-mass**: the original model, with first-order attitude response.

The page runs the Rust core only. If WebAssembly can't start, it shows an error instead of falling back.

## Layout

| Path | What |
|---|---|
| `rust/` | Simulation core (`circuitsim` crate) and the native CLI `circuitsim` |
| `rust/src/sixdof/` | 6DOF model (`mod.rs`), its control laws (`fcs.rs`), and trim and natural-mode analysis (`analysis.rs`, native only) |
| `rust/src/wasm.rs` | C-ABI exports for the browser: one simulation in module memory, read as a JSON frame each animation frame |
| `web/` | Browser UI in TypeScript; `build.sh` produces the single-file page with the WASM base64-embedded |
| `ts/` | The original TypeScript core, kept as a reference implementation for parity checks (no longer in the page) |

### Rust modules

| Module | Role |
|---|---|
| `units` | SI constants and helpers |
| `atmosphere` | Seeded PRNG, wind with gusts |
| `airfield` | Runway 09/27, Standard and High circuit specifications |
| `aircraft` | What the autopilot sees (`Aircraft`), `CommandShaper` rate limits, point-mass dynamics, dispatch to 6DOF |
| `control` | PI controllers, track guidance, flight-path/V-S/altitude holds, auto-throttle, speed-on-pitch |
| `autopilot` | Phase state machine, flight mode annunciator (FMA), stabilised-approach monitor, go-around, crosswind alignment, engine-out logic |
| `emergency` | `GlidePlanner`: reachability of either runway direction, intercept choice, glide gradients |
| `simulation` | World stepping at 50 Hz, track, glide-reach bookkeeping, landing evaluation |
| `scenarios` | Shared test cases |

## The 6DOF model

**State:** NED position, body velocity (u, v, w), attitude quaternion, and body rates (p, q, r).

**Aerodynamics:** C172-class stability and control derivatives:
- CLq, CLδe
- Cm0, Cmα, Cmq (including the α̇ term), Cmδe, flap pitching moment
- CYβ, CYδr
- Clβ, Clp, Clr, Clδa, Clδr
- Cnβ, Cnp, Cnr, Cnδa (adverse yaw), Cnδr

The lift and drag polar, stall and flap increments are shared with the point-mass model, so both glide the same. The model also includes ground effect on induced drag.

**Propeller:** thrust is power-limited. Torque rolls the aircraft left, and slipstream/P-factor yaws it left.

**Gear:** nose, left and right mains, and a tail skid. Each is a spring–damper with rolling and braking friction and lateral tyre force. The nose wheel steers.

**Natural modes:** from `circuitsim modes`, linearised about trim at sea level and 1,000 kg:

| Condition | Short period | Phugoid | Dutch roll | Roll τ | Spiral τ |
|---|---|---|---|---|---|
| Cruise 100 kt | 5.7 rad/s, ζ 0.70 | 30.8 s, ζ 0.14 | 2.9 rad/s, ζ 0.23 | 0.09 s | 87 s |
| Approach 65 kt, flap 30 | 3.7 rad/s, ζ 0.71 | 20.0 s, ζ 0.14 | 1.9 rad/s, ζ 0.22 | 0.14 s | 39 s |

These are in line with published C172 values. The trimmed power-off glide at 68 kt is 4.9°, a glide ratio of 11.7.

### Control laws (`sixdof/fcs.rs`)

The autopilot still commands pitch, bank and sideslip, rate-limited by the `CommandShaper`. The control laws close the loop on the rigid body, with gains scheduled by dynamic pressure:

- **Pitch:**
  - Attitude error, plus feed-forward of the shaped command's own rate, plus turn compensation, gives a pitch-rate demand.
  - The elevator works on rate error, with an integrator that finds the trim elevator.
  - The elevator stays neutral on the ground until rotation.
- **Roll:**
  - Bank error gives a roll-rate demand.
  - Aileron works on rate error, with an integrator that absorbs propeller torque.
  - A feed-forward cancels the rolling moment from sideslip (dihedral) and rudder.
- **Yaw:** rudder holds the commanded sideslip (zero for coordinated flight), with a yaw damper.
- **Ground:** nose-wheel steering and rudder track the runway heading, or the held heading in a field.

## Autopilot changes made for the 6DOF model

These changes apply to both models:

- **Crosswind alignment:**
  - Below 60 ft, a tight lateral law uses cross-track error and lateral velocity, plus a wing-low feed-forward computed from the actual side force.
  - Allowed bank tapers from 10° at 40 ft to 4° at touchdown.
  - Sideslip is limited to the crab the crosswind needs, plus 3°.
- **Flare:**
  - Pitch may move at 6°/s in the flare only. The 3°/s limit stays everywhere else, including the go-around.
  - The flare starts at about 2.7 s of sink above the ground.
- **Gust margin:** when gusts are reported, the approach speed rises by 4 kt, on power-off approaches too.
- **Glide planning:**
  - Flap that is already down stays down in the reachability sums.
  - When the aircraft is already established inbound, the planned route is straight in.

## Results

| Suite (native) | 6DOF | Point-mass |
|---|---|---|
| `scenarios`: 14 cases × 2 circuits | 23/28 | 23/28 |
| `engine`: 60 engine-failure cases | 57/60 | 54/60 |
| `montecarlo 400`, normal circuits | 94% | 84% |
| `montecarlo 400`, engine failures | 94% | 96% |

The Monte Carlo runs use winds of 0–15 kt from any direction, 40% of them gusty, across both circuits and random failure points.

The 5 normal-circuit failures in `scenarios` are 20–25 kt direct crosswinds. Those exceed the 15° sideslip and rudder limit, so the aircraft lands with 5–8° of crab left. That matches the airframe's limit, not a control fault. Most of the remaining Monte Carlo failures are marginal gusty touchdowns, at 3.0–3.7 ft/s against a 3 ft/s limit, or floats to 405–425 m against 400 m.

**Parity checks:**
- The first Rust port matched the TypeScript core on all 88 cases (normal and engine failure, both circuits), to printed precision: `ts/test/parity.ts` against `circuitsim parity`. The autopilot changes above came afterwards.
- The WASM build gives results identical to the native build.
- A full 6DOF flight computes in about 0.1 s in WASM.

## Build and run

The `wasm32` standard library can't be downloaded in this environment, so `build.sh` builds it from source. That needs `rust-src` for the toolchain version, which was fetched from GitHub.

```
cd rust
cargo build --release
./target/release/circuitsim scenarios [6dof]
./target/release/circuitsim engine [6dof]
./target/release/circuitsim profiles [6dof]
./target/release/circuitsim montecarlo 400 [6dof]
./target/release/circuitsim modes
./target/release/circuitsim trace [6dof] <fromDeg> <kt> <gusts 0|1> <std|high> [fail-engine-at-x-on-downwind]
./target/release/circuitsim parity            # CSV for comparing implementations
./target/release/circuitsim stol [6dof]       # normal vs short-field on 1200/600/450 m strips

cd ../web && ./build.sh                         # → ../circuit-autopilot-sim.html
```

`build.sh` runs `cargo build --release --lib --target wasm32-unknown-unknown -Zbuild-std=std,panic_abort` with `RUSTC_BOOTSTRAP=1`. With a normal toolchain, `rustup target add wasm32-unknown-unknown` lets you drop the `-Zbuild-std` flag.

## Short-field take-off and landing

There are two independent options, Technique and Strip, on the same aircraft. The procedure lives in `airfield::Procedure`.

| | Normal | Short-field |
|---|---|---|
| Brakes | Released at once | Held until full power |
| Rotate | 55 kt, 8° pitch | 48 kt, 10° pitch |
| Initial climb | Vy 75 kt | Vx 57 kt to 100 ft, then Vy |
| Approach | 65 kt on the circuit's path | 60 kt on at least a 5.5° power-on path |
| Aim point | A quarter of the way in (max 250 m) | 40 m past the threshold |
| Flare | Settles to 0.4 m/s from 2.7 s of sink | Firmer: 0.7 m/s from 2.3 s, so less float |
| Braking | Full after 2 s | Full after 0.5 s, flaps up |
| Long-landing go-around | Past the first third + 50 m | Past aim + 130 m |

**Strips:** 1,200 m (paved, 30 m wide), or 600 m and 450 m (20 m wide) with 50 ft trees 150 m beyond each end.

**Landing check:** it adds take-off distance (ground roll and to 50 ft) and landing distance (from 50 ft and ground roll), plus tree clearance on departure and approach. For short-field, the touchdown point must be within 100 m of the aim point. After two long-landing go-arounds, the autopilot accepts the long landing rather than circling forever.

**Power-off landings:** with short-field selected, power-off landings also use the short-field approach speed and immediate braking, but keep the gentler normal flare. The power-off aim point is capped at a quarter of the strip.

**Results** (`circuitsim stol [6dof]`, High circuit, 4 winds × 2 seeds). The winds are calm, a 10 kt headwind, a 10 kt crosswind, and 10 kt gusting.

| Strip | Normal (6DOF / point-mass) | Short-field (6DOF / point-mass) |
|---|---|---|
| 1,200 m | 8/8 / 7/8 | 7/8 / 7/8 |
| 600 m | 0/8 / 0/8 | 7/8 / 7/8 |
| 450 m | 0/8 / 0/8 | 7/8 / 7/8 |

**Typical 6DOF short-field figures in calm air:**
- **Take-off:** 206 m ground roll, 339 m to 50 ft.
- **Landing:** 185 m ground roll, 408 m from 50 ft.

These are in line with C172 handbook figures for a light aircraft. Normal technique touches down beyond the first third of the short strips, which is why short-field technique exists. The remaining short-field failure is a gusty case that floats past the touchdown window.

On the 450 m strip with short-field technique, 17/20 engine failures pass on both models. The three that fail arrive with too much energy and run past the end. The planner's safety margin works against a very short runway.

## Next

- **Energy management on short strips:** after a power-off return to a short strip, spend surplus height before the threshold, with S-turns or a later flap-and-slip decision.
- **Gusty landings:** gust-adaptive flare timing, for gusty short-field landings.
