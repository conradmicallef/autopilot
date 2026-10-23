# Applying the circuit autopilot to a foam RC model

Notes from the design discussion, 22 Oct 2026.

## What transfers and what doesn't

The structure carries over almost unchanged:
- phase state machine
- shaped attitude commands driving inner rate loops
- stabilised-approach gates, go-around and glide planner
- `Procedure` parameters
- the 6DOF plant for testing

Every number changes, and some phases change shape:

| C172 | Foam model (~1–1.5 m span, ~1 kg, 10–15 m/s) |
|---|---|
| Re ≈ 5 million | Re ≈ 100–200 k. CLmax is lower (~1.0–1.2), CD0 higher (0.04–0.07), stall is more abrupt, glide ratio ~6–8 |
| Wheels, brakes, rollout | Usually hand launch and belly landing. The take-off phase becomes launch detection (an acceleration spike), then a fixed-pitch climb. Rollout is "skid to a stop" |
| Piston, power-limited thrust | Electric: T ≈ T₀·thr·(1 − V/V_pitch). Thrust-to-weight is high, so torque roll and the pitch-with-power trim change are large. Battery sag reduces thrust. "Engine failure" means the motor is cut or the battery is low |
| Wind ≈ 10–20% of airspeed | Wind ≈ 30–60% of airspeed. Gusts dominate. Crosswind limits are low; with a weak or missing rudder, land in a crab on grass |
| Modes in seconds | Dynamic scaling: times shrink roughly with √(length scale). The short period can be 15–25 rad/s, so the loops need 200–400 Hz, and servo speed and latency become the bandwidth limit |

**Sensors:**
- **Flare height:** use a rangefinder (lidar). Baro drift (±1 m) and GPS vertical error (±2–5 m) are too coarse.
- **Airspeed:** a pitot at 10 m/s reads only ~60 Pa, so filter it, or estimate wind and use GPS speed minus wind.

## Getting the model parameters

The C172 tables (`rust/src/c172p.rs`) get replaced by a foam-model table.

1. **Measure geometry and mass:** span, chord, area, mass, CG, and control throws.
2. **Inertia:** use a bifilar or trifilar pendulum, one swing test per axis.
3. **First-guess aerodynamics:**
   - XFLR5 for low-Re airfoil polars plus vortex-lattice stability derivatives; AVL or OpenVSP also work.
   - These give CLα, Cmα, Cmq, Clp, Clβ, Cnβ, Cnr and the control derivatives.
   - Add 20–40% to CD0.
4. **Propulsion:**
   - Thrust stand for static thrust against throttle.
   - Published prop data (e.g. APC) for thrust against airspeed.
   - Measure servo rate and latency.
5. **Correct the model from flight data** (high-rate IMU logging):
   - **Doublets or 3-2-1-1 inputs:** fit short period, Dutch roll and roll mode. Compare with `circuitsim modes` and adjust Cmq, Cnr and Clp until they match.
   - **Motor-off glides at several speeds:** give the drag polar, glide ratio and stall speed, which feed the glide planner.
   - **Trim elevator against speed:** gives the static margin. The CG matters: the C172 model ran out of elevator in the flare until a realistic loaded CG was used.
6. **Optional:** write the same data as a JSBSim XML for ArduPilot/PX4 SITL.

## Tuning the controllers

**Inner loops:**
- Keep the dynamic-pressure scheduling, with `v_ref` moved to cruise speed (~12 m/s).
- Tune in this order: roll rate, then pitch rate, then yaw damper, then the attitude loops.
- Rate-loop crossover at no more than about ⅓–⅕ of 1/(servo delay + lag). Attitude loop about 4–5× slower. Integrators small.
- Check margins on the linearised model before flying.

**Shaper rates:** pitch 3°/s → 10–20°/s; bank 8°/s → 40–60°/s. The flare lead stays defined in seconds of sink.

**Circuit geometry** follows turn radius, V²/(g·tan φ), which is ~25 m at 12 m/s and 30° bank:
- downwind 60–80 m out
- circuit height 30–50 m
- final 100–150 m
- aim point ~10 m

**Speeds:** approach at about 1.3 Vs. Make the gust increment proportional to the measured gust spread.

**Engine-out:** use the measured polar, a turn-back height of ~15–20 m, and a battery-voltage trigger.

**Robustness:** run Monte Carlo with the parameters randomised as well as the wind:
- derivatives ±20–30%
- CG ±10 mm
- servo delay ×2
- thrust −20%

**Flight-test ladder:**
1. Stabilised manual mode (inner loops only).
2. Speed and altitude hold.
3. Circuits at height.
4. Approach and flare to a virtual runway 50 m up.
5. Real landing.

At every step a pilot on the transmitter must be able to take over instantly.

## Implementation paths

- **(Recommended) ArduPilot Plane or PX4** for sensors, EKF, failsafes and actuators. Use this project as the design and test bench. ArduPilot already has auto take-off, landing and flare (TECS + L1).
- **Companion computer:** run the Rust autopilot on a Pi-class board, sending attitude or rate targets over MAVLink. Check that your firmware version supports fixed-wing attitude targets in guided mode.
- **Fully custom:** `no_std` port on an STM32H7. You would also have to write the state estimator, which is the hard part.

## Rules (Italy / EU)

EASA Open category:
- registration and A1/A3 competency
- 120 m height limit
- visual line of sight
- a remote pilot able to intervene
