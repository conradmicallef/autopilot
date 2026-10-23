# Circuit Autopilot Sim

A proof-of-concept flight simulator. A light-single autopilot takes off from runway 09, flies a left-hand circuit at 1,000 ft, and lands. It goes around when the approach is unstable.

## Object model (`src/`)

| File | Classes | Role |
|---|---|---|
| `units.ts` | – | SI constants, conversions, `clamp`, `slew`, `wrapPi` |
| `atmosphere.ts` | `Rng`, `Wind` | Steady wind plus Ornstein–Uhlenbeck gusts (seeded, replayable) |
| `aircraft.ts` | `AircraftSpec`, `Aircraft`, `CommandShaper` | Point-mass flight model with sideslip, ground roll, inner attitude loops; `CommandShaper` rate-limits every command |
| `airfield.ts` | `Runway`, `CircuitSpec`, `Circuit` | Runway geometry, glidepath, circuit legs |
| `control.ts` | `PI`, `TrackGuidance`, `FlightPathController`, `AutoThrottle`, `SpeedOnPitch` | Outer loops and lateral guidance |
| `autopilot.ts` | `Autopilot`, `StabilityMonitor` | Mode manager (phase state machine), flight mode annunciator (FMA), go-around and engine-out logic |
| `emergency.ts` | `GlidePlanner` | Power-off energy planning: reachability, intercept choice, glide gradients |
| `simulation.ts` | `Simulation` | Owns the world, steps at a fixed 50 Hz, records the track and events, evaluates the landing |
| `ui/` | `Palette`, `PlanView`, `ProfileView`, `AttitudeIndicator` | Canvas rendering |
| `main.ts` | `App` | Controls and the animation loop |

Control flow on each step:

`Simulation.step` → `Wind.update` → `Autopilot.update` (returns `Commands`) → `Aircraft.step` (`CommandShaper` slews the commands, the inner loops run, then integration) → a liftoff or touchdown event, if any.

## Go-around

A go-around can be triggered in three ways:

- **Pilot request**, on base, final or in the flare.
- **Stabilised-approach gate**, checked below 300 ft on final. It trips when any one limit is breached for 2 seconds:
  - speed more than 5 kt below or 10 kt above the approach speed
  - glidepath deviation more than 0.7°
  - more than 30 m off the centreline
  - sink rate over 1,000 fpm
  - bank over 15°
- **Long landing**: the flare floats past the first third of the runway.

Nothing in the go-around steps a control:

- Throttle slews at 30%/s, pitch at 3°/s and bank at 8°/s.
- The flight-path target ramps up from its current value at 1.5°/s.
- Flaps retract in stages, travelling at 3°/s: 30 → 20 straight away, → 10 once climbing above 63 kt, and → 0 above 300 ft.
- Once it is climbing cleanly, control hands over to the normal climb and the circuit starts again.

## Crosswind landing

The approach is flown crabbed, with the nose into wind and the track on the centreline. Below 60 ft (`alignHeight`) the autopilot changes to a wing-low approach:

- Rudder sideslip `beta` swings the nose onto the runway heading. It slews at 4°/s, up to a 15° limit.
- The sideslip side force (`CYbeta`) would push the aircraft downwind. A feed-forward bank cancels it (`Aircraft.wingLowBank`), and the track loop trims what is left. The bank is limited to 10°.
- At touchdown the wheels take the nose heading. The landing check scores alignment as crab under 5°.

When a crosswind needs more than 15° of crab (about 20 kt at 65 kt approach speed), the aircraft lands with the remaining crab. That is beyond the airframe's crosswind limit.

## Circuit profiles

| | Standard | High (default) |
|---|---|---|
| Downwind | 1,200 m out, 1,000 ft | 1,000 m out, 1,000 ft |
| Descent | Fixed −500 fpm, starting abeam the threshold | Holds 1,000 ft until the approach path meets it |
| Base leg | 2.6 km before the threshold | 1.5 km before the threshold |
| Approach path | 3° | 5.5° |

The High circuit flies its descent as a path along the remaining downwind, base and final legs (`Circuit.profileHeight`). The path is corrected for the corners each turn cuts. The flare starts at about 3 s of sink above the ground, so the steeper approach still touches down gently.

`Simulation` checks twice a second whether a power-off return would reach either runway direction. The answer drives the GLIDE REACH readout, the strip under the altitude trace, and the "Runway within glide reach" row in the landing check. In the tests (calm air and winds up to 20 kt), the runway was within reach for:
- **Standard circuit:** about 30–39% of the airborne time.
- **High circuit:** about 55–66% of the airborne time.

## Engine failure

`Simulation.failEngine()` (the **Fail engine** button) stops the engine. The thrust drops to zero and the windmilling propeller adds drag. Pitch may slew at 5°/s, because airspeed must be held. `Autopilot.failEngine` then decides what to do:

| Situation | Plan | Phases |
|---|---|---|
| On the runway | Reject the take-off: idle, full brakes | `REJECTED` |
| Climbing below 700 ft | Land ahead; never turn back | `FORCED_LANDING` |
| Higher, and `GlidePlanner` finds runway 09 or 27 reachable | Return to the one with the most height in hand | `EO_GLIDE` → `EO_FINAL` |
| Higher, both directions out of reach | Land ahead | `FORCED_LANDING` |

### How `GlidePlanner` (`emergency.ts`) plans a return

**Route.** A base key point, abeam the intercept, on the aircraft's side. Then the intercept on the extended centreline. Then an aim point 300 m into the runway.

**Height required.** Each leg's length is divided by the ground glide ratio. That ratio is the clean, best-glide (68 kt) L/D from the model, adjusted for wind along the leg. The calculation also allows for:
- the initial turn
- corner-cutting at each turn
- turn load factor

**Height available.** The height, plus any speed above best glide, converted to height.

**Choosing the runway direction.** Both directions are planned (`RunwayEnd`: 09 lands toward +x from x = 0, and 27 lands toward −x from x = 1,200 m). It picks the reachable one with the most height left over. Once a direction is chosen it sticks with it while it still works, so it doesn't switch back and forth.

**Choosing the intercept.** It scans intercepts from 3 km down to 500 m. It takes the longest final it can reach, with a 10% + 20 m margin, so surplus height goes into a long final rather than arriving high. It re-plans every second, and switches to landing ahead if nothing reaches.

**On a power-off final.**
- Flaps go out in stages, and never retract, while the required gradient is steeper than the next setting's. Until it is lined up (or below 300 ft), it holds at 10°, so the turn onto final is flown on the flatter glide.
- If it is still high with full flap, it forward-slips (into-wind wing down, opposite rudder).
- It straightens up below 60 ft.
- The flare starts at about 3 s of sink above the ground.

**Field landings** hold the track at failure, take 20° of flap below 300 ft, straighten up, and roll out straight ahead.

## Build

```
npm i
npx tsc -p .                                    # type-check (strict)
npx esbuild src/main.ts --bundle --format=iife --outfile=dist/sim.js
npx esbuild test/scenarios.ts --bundle --platform=node --outfile=dist/test.js && node dist/test.js
npx esbuild test/engine-failure.ts --bundle --platform=node --outfile=dist/eo.js && node dist/eo.js   # both circuits, 60 cases
npx esbuild test/profiles.ts --bundle --platform=node --outfile=dist/prof.js && node dist/prof.js        # Standard vs High, glide reach
```

`test/scenarios.ts` flies these cases headless:

- calm, headwind, crosswind and tailwind
- gusty crosswinds up to 25 kt
- a manual go-around from base, from final and from the flare

It also checks that throttle never moves faster than 30%/s.
