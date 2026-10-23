# Circuit Autopilot Sim

**▶ Try it in your browser: https://conradmicallef.github.io/autopilot/**

A Cessna 172P flies a traffic circuit under autopilot: take-off, circuit, approach and landing. It goes around when the approach is unstable, straightens up for crosswind landings, and handles engine failures.

The simulation core is written in Rust. It runs natively for test suites and Monte Carlo batches, or in the browser as WebAssembly.

**Flight model: 6DOF only.** A rigid body on the open JSBSim Cessna 172P aerodynamic tables, with control surfaces, propeller torque and slipstream, and the JSBSim spring–damper landing gear, integrated with RK4 at 200 Hz.

The page, the `circuitsim` command line and the test tools use only this model. The CLI still accepts and ignores a `6dof` argument. The original point-mass model is still in the crate (`ModelKind::PointMass`) but nothing uses it. Point-mass figures further down are historical.

The page runs the Rust core only. If WebAssembly can't start, it shows an error instead of falling back.

## Layout

| Path | What |
|---|---|
| `rust/` | Simulation core (`circuitsim` crate) and the native CLI `circuitsim` |
| `rust/src/sixdof/` | 6DOF model (`mod.rs`), its control laws (`fcs.rs`), and trim and natural-mode analysis (`analysis.rs`, native only) |
| `rust/src/wasm.rs` | C-ABI exports for the browser: one simulation in module memory, read as a JSON frame each animation frame |
| `web/` | Browser UI in TypeScript; `build.sh` produces the single-file page with the WASM base64-embedded |

### Rust modules

| Module | Role |
|---|---|
| `units` | SI constants and helpers |
| `atmosphere` | Seeded PRNG, wind with gusts |
| `airfield` | Runway 09/27, Standard and High circuit specifications |
| `c172p` | The JSBSim Cessna 172P data: lift, drag, ground-effect and derivative tables, geometry, inertia, gear, loaded CG |
| `aircraft` | What the autopilot sees (`Aircraft`), `CommandShaper` rate limits, point-mass dynamics, dispatch to 6DOF |
| `control` | PI controllers, track guidance, flight-path/V-S/altitude holds, auto-throttle, speed-on-pitch, wind and gust estimator |
| `dubins` | Dubins paths (turn–straight–turn) and the vector-field path follower used for power-off glides |
| `autopilot` | Phase state machine, flight mode annunciator (FMA), stabilised-approach monitor, go-around, crosswind alignment, engine-out logic |
| `emergency` | `GlidePlanner`: Dubins glide paths in the air mass to either runway direction, join point and aim choice, height model from the trimmed glide, hold-off float |
| `tuning` | `Tuning` (gains and landing parameters, with the autotuned defaults) and `Perturb` (model error for robustness tests) |
| `autotune` | Native only: fixed flight sets, soft scoring, inner-loop margin check, CMA-ES search |
| `simulation` | World stepping at 50 Hz, track, glide-reach bookkeeping, landing evaluation |
| `scenarios` | Shared test cases |

## The 6DOF model

**State:** NED position, body velocity (u, v, w), attitude quaternion, and body rates (p, q, r).

**Aerodynamics:** the Cessna 172P tables from the open [JSBSim](https://github.com/JSBSim-Team/jsbsim) model (`aircraft/c172p/c172p.xml`, LGPL), looked up with clamped linear interpolation as JSBSim does (`rust/src/c172p.rs`):
- **Lift:** CL(α) for wing, body and tail, with the stall-hysteresis branch (stalled above α 0.36 rad, recovered below 0.09 rad); flap increments; CLδe, CLq and CLα̇.
- **Drag:** CD0, a full α × flap table, flap increments, and drag from elevator and sideslip.
- **Ground effect:** the kCL and kCD tables against wing height over span.
- **Side force, roll and yaw:** CYβ by flap; CYp, CYr and Clr by α and flap; CYδr, Clβ, Clp, Clδa, Clδr, Cnβ, Cnp, Cnr, Cnδa (adverse yaw), Cnδr.
- **Pitch:** Cm0, Cmα, Cmq, Cmα̇, Cmδe, and flap pitching moment by flap.
- **Slipstream:** Cmδe and Cnδr act on the propwash dynamic pressure (momentum theory, 75 in propeller). This is why the elevator and rudder still work at low speed with power on.
- **Force and thrust lines:** forces act at the JSBSim aerodynamic reference point, and thrust along the propeller hub line, both relative to the CG.
- **α̇ terms:** these are folded into the q terms (α̇ ≈ q in the short-period motion), because the state has no α̇.

Sign conventions are converted to this model's: the rudder is positive nose-right, so the JSBSim rudder terms change sign.

**Mass and balance:** 1,000 kg (2,205 lb), made up of:
- the JSBSim empty aircraft (1,500 lb)
- pilot and front passenger
- one rear passenger
- 200 lb of fuel

That puts the CG at 43.6 in, close to the aerodynamic reference point. With the empty-aircraft CG (41 in, well forward), the elevator ran out at its −28° stop in power-off flares. Inertias are the JSBSim values.

**Surface limits:** JSBSim's elevator −28/+23°, rudder ±16°, nose-wheel steering ±10°. Ailerons are ±20°.

**Propeller:** thrust is power-limited. Torque rolls the aircraft left, and slipstream/P-factor yaws it left.

**Gear:** the JSBSim C172P nose and main contact points, springs and dampers, plus a tail skid. Each is a spring–damper with rolling and braking friction and lateral tyre force. The nose wheel steers. The tail touches at 10.3° of pitch.

**Natural modes:** from `circuitsim modes`, linearised about trim at sea level and 1,000 kg:

| Condition | Short period | Phugoid | Dutch roll | Roll τ | Spiral τ |
|---|---|---|---|---|---|
| Cruise 100 kt | 7.3 rad/s, ζ 0.60 | 26.1 s, ζ 0.12 | 2.7 rad/s, ζ 0.23 | 0.09 s | 48 s |
| Approach 65 kt, flap 30 | 4.9 rad/s, ζ 0.60 | 19.5 s, ζ 0.12 | 1.9 rad/s, ζ 0.24 | 0.13 s | 132 s, divergent |

**Trim:**
- Cruise at 100 kt takes 79% throttle.
- A 75 kt climb at 5° (660 fpm) takes full throttle.
- The power-off glide at 68 kt is 6.0°, a glide ratio of 9.5, close to the POH figure.

The spiral mode is mildly divergent at low speed, as is typical of the C172.

The earlier hand-picked coefficients glided at 11.7:1. The JSBSim drag is higher, so the glide planner, power-off speeds and flare timing were re-tuned.

### Control laws (`sixdof/fcs.rs`)

The autopilot still commands pitch, bank and sideslip, rate-limited by the `CommandShaper`. The control laws close the loop on the rigid body, with gains scheduled by dynamic pressure:

- **Pitch:**
  - Attitude error, plus feed-forward of the shaped command's own rate, plus turn compensation, gives a pitch-rate demand.
  - The elevator works on rate error, with an integrator that finds the trim elevator. Both elevator and rudder gains are divided by the slipstream factor.
  - The elevator stays neutral on the ground until rotation.
- **Roll:**
  - Bank error gives a roll-rate demand.
  - Aileron works on rate error, with an integrator that absorbs propeller torque. The integrator runs only within 4° of the target bank (anti-windup). Without that, it wound up during the roll-in and gave 20% bank overshoot with a slow settle; with it, overshoot is 5–7%.
  - A feed-forward cancels the rolling moment from sideslip (dihedral) and rudder.
- **Yaw:** rudder holds the commanded sideslip (zero for coordinated flight), with a yaw damper.
- **Ground:** nose-wheel steering and rudder track the runway heading, or the held heading in a field.

The gains come from the autotuner (see below): k_θ 3.52, k_q 2.94, k_iθ 1.28, k_φ 3.25, k_p 0.53, k_β 2.35, k_r 1.07. `circuitsim tunecheck` prints their step responses.

## Autopilot changes made for the 6DOF model

These changes apply to both models:

- **Crosswind alignment:**
  - Below 60 ft, a tight lateral law uses cross-track error and lateral velocity, plus a wing-low feed-forward computed from the actual side force.
  - Allowed bank tapers from 10° at 40 ft to 4° at touchdown.
  - Sideslip is limited to the crab the crosswind needs, plus 3°.
- **Flare:**
  - Pitch may move at 6°/s in the flare only. The 3°/s limit stays everywhere else, including the go-around.
  - The flare starts at about 2.7 s of sink above the ground.
- **Turbulence (measured, not given):**
  - The autopilot estimates the wind the way a real one does, as GPS velocity minus air velocity.
  - It high-passes that estimate and tracks the spread, σ: about 1.2–1.3 m/s in the gusty runs and 0 in steady wind.
  - The approach speed adds a multiple of σ, up to 5 kt (3 kt for short-field). Power-off approaches add their own multiple of σ, up to 5 kt. The multiples are autotuned (see below).
  - The short-field flare softens in turbulence: its 0.7 m/s floor moves toward 0.4 m/s, and it starts up to 0.3 s earlier.
  - **Lull protection:** with the engine running, if airspeed sags below about 1.3 Vs in the flare, the autopilot adds throttle, up to 60% (FMA "SPD"). In calm air this is mostly dormant.
- **Flare limits:** pitch in the flare stops 0.5° short of the tail-strike attitude.
- **Power-off flare:** starts at about 2.6 s of sink (was 3 s), from a 67 kt approach with flap 20 or more (was 63 kt), whatever the technique.
- **Glide planning:**
  - Flap that is already down stays down in the reachability sums.
  - When the aircraft is already established inbound, the planned route is straight in.
- **Power-off turn onto final in strong wind.** In 25 kt with a tailwind on base, the turn onto final used to swing up to 300 m through the centreline, because the capture law turned in as if the aircraft could turn on the spot. Four changes fixed it:
  - **Turn prediction.** Once past the base key, the autopilot predicts where a turn onto final begun now would end. The prediction uses air-mass turn geometry, the wind's drift over the turn, and any roll reversal still to come. The turn starts as soon as it would only just reach the centreline (`final_turn_due`).
  - **Wind-aware base key.** The base key sits far enough out for that turn to finish, wind included (`GlidePlanner::turn_reach`). The base-to-final trigger uses the same estimate.
  - **Capture distance.** The final-capture distance scales with the ground-speed turn radius at the circuit bank, instead of a fixed 90 m. It still uses the circuit bank during a slip, so the slip keeps the centreline.
  - **Cross-controls (forward slip), and the order of energy measures.** When high on a power-off final, the autopilot uses energy measures in this order:
    1. Flap in stages to 20°. It goes beyond 10° only once within 30° and 150 m of the line.
    2. A forward slip at flap 20 (FMA "SLIP"): into-wind wing down, opposite rudder, track held on the centreline. It starts once within 15° and 60 m of the line, and runs down to 100 ft.
    3. S-turns, if even that won't get it down.
    4. Flap 30 only when it isn't slipping.

    The C172 POH advises against steep slips with more than 20° of flap, hence the order. Before, the slip waited for full flap and tight alignment, so it often came too late to help.

    How much a slip buys in this model (`examples/slip_test.rs`, 66 kt, power off):

    | | Flap 0 | Flap 20 | Flap 30 |
    |---|---|---|---|
    | No slip, sink | 850 fpm | 950 fpm | 990 fpm |
    | Full slip, sink | 1,110 fpm | 1,200 fpm | 1,240 fpm |
    | Full slip, glide angle | — | 10.4° (from 8.2°) | 10.7° (from 8.5°) |

    The rudder stop (±16°, JSBSim) caps the sideslip at 11.5°. Beyond that, commanding more does nothing, and JSBSim's sideslip drag (CDβ 0.17·|β|) is modest. A slip adds about 25% to the sink rate, which is enough to fix a moderately high final but not one that arrives hundreds of feet high.
  - **Energy.** The planner counts most of a reversal turn as arc, so it no longer over-promises energy to a key behind the aircraft. If the final is still far too high (over 1.4 × the full-flap gradient, more than 500 m out), the autopilot flies S-turns of ±150 m about the centreline (FMA "S-TURN") before the slip.

## Auto-tuning (`circuitsim autotune`)

The control-law gains and the landing parameters are found by search, not by hand: 14 parameters in all (`tuning::DIMS`).
- **Gains:** k_θ, k_q, k_iθ, k_φ, k_p, k_β, k_r. These are searched multiplicatively.
- **Flare timing:** flare lead for normal, short-field and power-off landings.
- **Speeds and margins:** the power-off approach speed, the two gust-margin multiples, and the lull-protection threshold.

How a candidate is scored:
- **Fixed flights.** Every candidate flies the same 120 training flights, so a difference in score comes from the parameters, not from luck:
  - 40% normal circuits, 40% engine failures, 20% short-field on 600 m and 450 m strips
  - winds of 0–15 kt from anywhere, 40% gusty
- **Model error.** Half the flights use a perturbed airframe: mass 0.9–1.08×, control power 0.8–1.2×, actuator lag 1–2×, pitch stability Cmα 0.75–1.25×. The tuning has to work on a model that is a little wrong, which any real one is.
- **Soft cost.** A failed flight costs 1. Every flight also adds up to 0.6 for how close it came to the sink, crab, centreline and touchdown-point limits. That rewards margin, and gives the search a slope to follow between pass and fail.
- **Margin check before flying.** The pitch (+4°) and bank (20°) responses are simulated at 65 kt with flap 30 and at 100 kt clean, with commands slewed at three times the operational rates. Two models are used:
  - the nominal one
  - a "hot" one, with control power ×1.6 (≈ 4 dB of gain margin) and actuator lag ×2.5 (phase margin)

  The candidate is rejected if any response overshoots more than 25% (50% when hot), is slow to settle, or rings by more than 5% about its trend.
- **Search.** Separable CMA-ES, 8 candidates per generation, flights run on all cores.
- **Hold-out.** The winner is judged on 240 different flights.

**First run** (`circuitsim autotune 6dof 14 120`, 11 minutes on 2 cores): 104 candidates flown, 9 rejected by the margin check.

| | Hand-tuned | Autotuned |
|---|---|---|
| Training (120 flights) | 108/120 (cost 0.147) | 118/120 (cost 0.046) |
| Hold-out, nominal airframe (120) | 107/120 | 115/120 |
| Hold-out, perturbed airframe (120) | 111/120 | 110/120 |

The tuned values are the 6DOF defaults (`Tuning::default`). The hand-tuned set is kept as `Tuning::hand_tuned`. The point-mass model keeps it, because the 6DOF-tuned landing values cut its engine-failure Monte Carlo from 93% to 85%: landing parameters are model-specific.

**What changed:**
- Tighter pitch and yaw loops, still inside the margin check.
- Slightly earlier flares: 2.5 s normal, 2.1 s short-field, 2.56 s power-off.
- A power-off speed of 67 kt.
- Smaller gust margins, especially power-off (0.87 σ). More speed was costing float, not saving lulls.

**What it didn't fix:** on the fixed suites the picture is mixed. Some suites gain a case or two and others lose one. In particular, the one gusty short-field case in `stol` now floats again, to 165 m against a 140 m window. Those suites are small and were not in the training set. The hold-out is the fairer measure.

## Review and rework (23 Oct, afternoon)

A Fable review (`notes/fable-review-2026-10-23.md`) checked the code against good practice, over-complicated logic and what should be physically possible. All eight of its recommendations are done:

1. **Dubins glide planner and one path-following law** (`dubins.rs`, `emergency.rs`). The power-off plan is a turn–straight–turn path at the bank actually flown, solved in the air mass so the fix drifts upwind by wind × time. It joins the centreline anywhere from 1.2 km out to over the runway, and the touchdown can be anywhere from 60 m in to where a stop still fits. One height model covers it all:
   - straight: distance ÷ L/D
   - turn at bank φ flown at max(best glide, 1.2 × the stall speed × √n): n × arc length ÷ L/D at the 1 g equivalent speed
   - roll-in, hold-off float and flap retraction included

   Plans rank by height in hand relative to the path's need (up to the tuned `eo_reserve`), then by touchdown nearest the nominal aim point, then by the longest final. The path is followed in the air mass with a vector-field law: orbit capture on the arcs, line capture on the straight, arcs tracked by heading with a roll-out lead. Each second the energy is re-checked along what is left of that path, not on a fresh path, which would start with a new roll-in.

   This deletes the base key, intercept-point corner costs, `final_turn_due`, the DIR KEY/BASE branches, S-turns and the separate margin-free "stretch". A kinematic test flies 72 start/target pairs: worst case 8.3 m off the line and 2.1° off heading.
2. **Scoring.**
   - Every landing now has a touchdown speed row (≤ 1.15 Vs, + ¾ of the crosswind, + 3.5 kt when gusty) and a nose-up row (≥ 5°).
   - Glide-back returns also have the centreline and crab rows.
   - "Stopped on runway" checks the runway area.
   - The flare holds off just above the runway until the speed bleeds to 1.05 Vs (+ ¾ crosswind, + half the gust margin). Touchdowns are now 49 kt, 1.09 Vs, 9° nose-up, with a 173 m ground roll (POH 168 m). Before they were 58 kt, 1.29 Vs, 4°.
   - The normal aim point moves to 150 m in so the float stays in the first third.
3. **One energy decision on a power-off final**: a drag ladder (flap 0/10/20, slip at 20, flap 30). Of the levels allowed (flap beyond 10° once lined up or low, the slip when aligned above 100 ft, no flap raised below 100 ft), it takes the one whose glide plus float lands nearest the planned aim point, never short of the runway. When even full drag lands far beyond the farthest aim point, it plans a longer path.
4. **Estimated wind.** The autopilot flies on a 15 s mean of GPS velocity minus airspeed along the nose heading, frozen while a sideslip is commanded. It no longer reads the simulation's wind. The state is still noise-free.
5. **Competing mechanisms removed.**
   - The planner's turn bank is the one flown (`eo_turn_bank`).
   - The stretch fallback is folded into the plan's aim-point choice, so the tuned margins matter again.
   - `VBG_HEAD_K`, `key_margin` and `stretch_margin` are gone.
   - Every tuned value enters through one constructor: `Autopilot::with_procedure(…, Tuning)`, `EngineOutSpec::new`, `FcsGains::from_tuning`.
   - The autopilot no longer writes to the aircraft. The glide pitch rate travels in `Commands`, and only the flap lever carries over between steps.
   - Heights are named in feet, and `vy` is renamed `v_climb`.
6. **Tests.** `cargo test --release --lib --test suites`. 17 unit tests: Dubins geometry, the follower, the planner, the flare law, frame conventions through `observe()`, glide table against trim. 4 regression tests: three fixed flights, and suite totals against `tests/golden.txt`.
7. **Results against a physical ceiling.**
   - At every engine failure the simulation records whether an ideal plan (no margins, true wind, any aim point) reaches the runway.
   - `engine` and `montecarlo` report each failure point against that ceiling.
   - The tuner no longer charges the parameters for failures beyond it.
8. **The lift-table kink.** JSBSim's c172p has CL 0.73 at α 0.09 and 0.83 at 0.10, a slope twice that either side, which put a kink at best glide (L/D 8.74 at 70 kt, 8.13 at 72). The 0.09 point is raised onto the line above (0.78). The polar is now smooth: best L/D 9.0 at 76 kt, minimum sink 811 fpm at 70 kt. The planner reads the model's own trimmed glide, elevator trim drag included (`GlideTable`), instead of an untrimmed polar that was up to 11% optimistic below 70 kt.

**Re-tune.** `autotune 14 120 2` improved the training cost from 0.419 to 0.294, but on the 240 hold-out flights only from 0.399 to 0.371, with no more passes. On the suites the tuned set was mixed: normal scenarios 20 → 17, short-field 19 → 22, engine 25 → 24, Monte Carlo normal 77 → 80%. The defaults were kept.

**Also new:** a cockpit view with a head-up display (pitch ladder, flight path marker, bank scale, heading/speed/altitude tapes, vertical speed, modes). The page drops to 1× when the engine fails.

## Crosswind, base and final engine failures (23 Oct)

Crosswind-leg failures almost never reached the runway. Several separate causes:

1. **Accelerated near-stall in the turn back.** The autotuner had pushed the power-off turn bank to its 50° limit. Gliding at about 62 kt, the stall speed at 50° bank (about 60 kt) left no margin. The elevator went to full up, α reached 15.7°, and the recovery dive gave a 4.4:1 glide. About 360 ft went in one 135° turn. Fixes:
   - a bank limit from the speed actually flown (airspeed ≥ 1.2 × the stall speed at the turn's load factor, never below 15°)
   - the glide speed in a turn rises with √(load factor), so best-glide α is held and the nose goes down as the bank goes on
   - power-off turn bank capped at 45°
2. **Geometry.** The aircraft turned crosswind 0.9–1.6 km past the far end of the 1,200 m runway. A Vy climb (about 10:1) gains height more slowly than an 8.9:1 glide uses it, so every metre flown away put the runway further out of reach. In calm air at 700 ft on the crosswind leg, the runway needs about 236 m of height against 213 m available. The climb is now flown at 65 kt, between Vx 62 and Vy 74, which stays closer to the runway. That is the best of 62, 65, 68 and 75 kt in a crosswind-leg sweep.
3. **"Already on final" when heading away.** A failure just as the crosswind turn began was within 30 m of the extended centreline, and the turn-lead test switched it to final while it was flying away from the runway. The lead test now counts only when flying toward the intercept point. While reversing onto final, no more flap goes out and the turn is flown at the 45° power-off bank, not the 25° circuit bank.
4. **Base and final: raise the flap.** The planner now also costs a flap-up glide (8 m to raise the flap) when flap is out, and takes the better plan. On final, the flap comes up in stages while lined up, above 100 ft, and short of a point 60 m past the threshold (not short of the normal aim point, which wrongly raised the flap on a gusty approach). A failure lined up on final always goes for the runway: landing ahead points the same way.

`examples/xw_sweep.rs` fails the engine at six points along the crosswind leg in six winds on both circuits. Run it with `cargo run --release --example xw_sweep`. `xw_case`, `xw_trace`, `eo_gust` and `glide_oracle` are the tracing tools used.

| Suite | Before | After |
|---|---|---|
| Crosswind sweep (both circuits) | 4/65 | 24/71 |
| Engine suite (`circuitsim engine`) | 28/60 | 34/60 |
| Monte Carlo engine failures | 46% | 50% |
| Monte Carlo normal circuits | 98% | 99% |
| Normal scenarios | 24/28 | 24/28 |
| Short-field (`stol`) | 23/24 | 24/24 |

Still out of reach, on physics rather than flying:
- calm-air and tailwind-on-return failures early on the crosswind leg
- final at 400 ft on the Standard circuit (2 km out)
- the climb-out failures

The page now drops to 1× when the engine is failed.

## Page freeze after an engine failure (fixed 23 Oct)

An engine failure on the crosswind leg could freeze the page mid-glide. The stretch-to-runway logic logs a `STRETCH` event. The page's profile chart and event log looked up every event as a flight phase, so a non-phase event threw in the drawing loop and stopped the animation.

The fix has three parts:
- Only real phase changes draw profile bands.
- The log names marker events (begin descent, engine failure, stretch) and shows any unknown one as plain text.
- The palette falls back to a default colour for an unknown phase.

`web/test/eo-smoke.mjs` fails the engine at five points on the crosswind leg in a headless browser. It checks that each flight reaches STOPPED with no page errors. It reproduced the freeze in 3 of 5 runs before the fix and passes 5/5 after.

## Tuning on 6DOF with strict scoring (23 Oct)

The autotuner now searches 20 parameters. To the 14 control-law and landing parameters it adds six for the power-off planner:
- planning margin factor and margin height
- best-glide speed
- base-key margin
- turn-back bank
- the stretch margin

A candidate scores well only for engine failures that end on the runway. Any touchdown above 4 ft/s, on or off the runway, costs extra (more above 7 ft/s), so the search can't buy runway landings with hard ones.

How the run went:
1. **First pass** (18 generations): hold-out 169 → 176/240. It pushed the flare leads and the stretch and planning margins to their lower bounds, and gave six hard (4.2–4.5 ft/s) arrivals where there had been none.
2. **Retune.** The hard-arrival penalty was extended to all touchdowns, and the flare leads (2.2 s, 1.9 s short-field) and margins (5 m) were given floors. A second pass of 14 generations followed.
3. **Two settings reset by hand.** The power-off approach speed went back to 67.2 kt: the tuned 65.7 kt was behind four of the hard arrivals. The gust margin went back to 1.33 σ: the tuned 0 lost a gusty short-field landing on every strip.

Final values (`Tuning::default`):

| Group | Values |
|---|---|
| Gains | k_θ 2.14, k_q 4.71, k_iθ 0.95, k_φ 3.72, k_p 1.11, k_β 2.09, k_r 0.62 |
| Flare leads | 2.2 s, short-field 1.9 s, power-off 2.6 s |
| Power-off | approach 67.2 kt, best glide 68.6 kt |
| Gust margins | 1.33 σ, power-off 1.48 σ |
| Lull protection | 1.34 Vs |
| Planner | margin ×1.04 + 5 m, base-key margin 262 m, turn-back bank 50°, stretch margin 5 m |

The step-response margin check passes.

| | Before | After |
|---|---|---|
| Hold-out, 240 flights | 169 | 175 (nominal 83/120, perturbed 92/120) |
| Off-runway touchdowns | 62 | 60 |
| Hard arrivals | 0 | 0 |
| `engine` | 27/60 | 27/60 |
| `scenarios` | 24/28 | 24/28 |
| Short-field `stol` | 7/8 | 7/8 |
| 450 m-strip engine failures | 5/20 | 8/20 |
| Strong-wind sweep | 133/144 | 130/144 |
| Monte Carlo, engine failures | 43% | 45% |
| Monte Carlo, normal circuits | 97% | 98% |

`examples/tune_compare.rs` reproduces the hold-out comparison.

## Engine-failure scoring: only a runway counts

An engine failure now passes only if the aircraft touches down on a runway, any part of it, and stops on it. A rejected take-off that stops on the runway also passes. A field landing is a fail, however gentle. The touchdown limit is still 3 ft/s, and a return must also stop on the runway. This is much stricter than before, when a controlled field landing counted as a pass.

**Engine suite** (`circuitsim engine`), by failure point, 6 cases each. Each cell is 6DOF / point-mass.

| Failure point | Passes | Why the rest fail |
|---|---|---|
| Take-off roll, 40 kt | 6/6 · 6/6 | Rejected on the runway |
| Climb, 200 ft and 450 ft | 0/6 · 0/6 | Already past the runway end. The climb is shallower than the glide, so the runway is never back in reach |
| Crosswind, 700 ft | 0/6 · 0/6 | Planner says out of reach: field |
| Downwind early (x < 1,800 m) | 5/6 · 6/6 | Returns |
| Downwind mid (x < 1,000 m) | 4/6 · 6/6 | Returns. This was 0/6 before a margin bug was fixed (see below) |
| Abeam the threshold | 6/6 · 5/6 | Returns |
| Base | 0/6 · 0/6 | Field |
| Final, 400 ft | 0/6 · 0/6 | The 3° approach is far shallower than the power-off glide (about 7:1 with flap 30 and a windmilling prop), so it lands short |
| Final, 150 ft | 1/6 · 1/6 | Same; only the High circuit's 5.5° approach makes it |

Totals:

| | 6DOF | Point-mass |
|---|---|---|
| `engine`, 60 cases | 22/60 | 24/60 |
| Monte Carlo, engine failures | 36% | 42% |
| Strong-wind sweep, 144 cases | 85/144 | 91/144 |

**Stretch to the runway, and steeper turn backs.** These were added after a failure on the crosswind leg at 700 ft, in a 15 kt easterly, that landed in a field.

The plan to runway 27 was marginal, with 1 m of surplus. The 136° left turn back at 25° of bank took about 20 s and used up that margin. At 580 ft the autopilot gave up on the runway, though the runway was still within a margin-free glide. Two changes:
- **Stretch (`GlidePlanner::stretch`).** Before settling for a field, the autopilot checks a margin-free short final: a 150 m intercept, a touchdown 60 m past the threshold, and a 10 m height margin. It checks this both at the failure and whenever the plan is lost. If either runway direction passes, it keeps the return (FMA event "STRETCH").
- **Steeper turn backs.** A power-off turn of more than 60° toward the base key or intercept is flown at 45° of bank, as turn-back studies recommend, instead of 25°. A steeper turn loses less height per degree of heading change.

| | Before | Stretch only | Stretch + 45° turns |
|---|---|---|---|
| `engine`, 6DOF / point-mass | 22 / 24 | 25 / 27 | 26 / 25 |
| Strong-wind sweep, 6DOF / point-mass | 85 / 91 | 118 / 126 | 131 / 136 |
| Monte Carlo engine failures, 6DOF / point-mass | 36% / 42% | — | 43% / 47% |

Not every stretch makes it. One gusty case in the 6DOF engine suite (High circuit, base, 360/12G) lands 150 m short of runway 09 at 5.5 ft/s, where a field landing would have been gentler. That is the price of preferring the runway.

Engine suite by failure point now, 6DOF / point-mass:
- downwind early: 6 / 6
- downwind mid: 5 / 5
- abeam the threshold: 6 / 5
- final 150 ft: 2 / 3
- base: 1 / 0
- crosswind: 0 / 0
- climb: 0 / 0
- final 400 ft: 0 / 0

**Power-off speed, measured rather than assumed.** `examples/glide_speed.rs` trims the 6DOF model in a steady power-off glide (prop windmilling) across the speed range.

| | Best glide | Glide ratio | Notes |
|---|---|---|---|
| Flap 0 | about 70 kt | 8.7 | The 68 kt used before gave 8.4. The optimum is flat out to about 80 kt (8.5). Minimum sink is 805 fpm, also near 70 kt |
| Flap 20 | 74–76 kt | 7.5 | The 67 kt power-off approach speed gives 7.1. That speed is kept for the landing; flap only goes out when there is height to burn |

- **Planner versus trim.** The planner's glide ratio at best glide is about 6% optimistic against the trim (8.9 against 8.4 at 68 kt). The 10% planning margin covers it.
- **Wind.** In a 15–25 kt headwind the trim says fly 80–82 kt, which is about half the headwind added. In a tailwind, stay at 70 kt.
- **What was adopted.** Best glide is now 70 kt. A headwind correction of half the headwind component above 5 kt was tried and dropped: it cost the point-mass model 11 cases in the strong-wind sweep and gained nothing on 6DOF.

| | 68 kt | 70 kt |
|---|---|---|
| `engine`, 6DOF / point-mass | 26 / 25 | 27 / 26 |
| Strong-wind sweep, 6DOF / point-mass | 131 / 136 | 133 / 137 |
| 450 m-strip engine failures, 6DOF | 7/20 | 5/20 |

**Flap limit speeds.** The C172P limits are 110 KIAS for 10° of flap and 85 KIAS beyond 10°. They weren't enforced. `examples/flap_speed.rs` found flap above 10° beyond 85 kt in 23 of 88 suite flights, up to 99 kt, mostly flap 20 commanded on entering base. Now any flap command beyond 10° waits until the airspeed is 2 kt inside the limit, and any flap at all waits under 110 kt. That leaves one marginal case at 85 kt, and the normal-circuit results barely move: Monte Carlo 97% / 84%, from 98% / 85%.

**A leftover from a parameter sweep.** The margin that places the base key beyond the turn-onto-final reach had been left at 450 m, instead of the intended 150 m. That pushed every base key about 300 m further from the centreline. The extra path made the planner reject returns it could make, such as a failure abeam the runway at 1,000 ft on downwind, which then landed in a field. Back at 150 m, a sweep of 50–350 m shows 150 m is best on the strict score.

The strong-wind figures quoted further down (143/144 and 139/144 passed, worst overshoot 36 m / 311 m) were measured with the 450 m margin and the old lenient score. With 150 m, the overshoot rises again: worst 318 m / 310 m, mean 19 m / 25 m. A larger margin gives more room to turn onto final but costs reach. Getting both needs a key placed by energy, not a fixed margin.

Scored the same way, the code before today's changes managed 19/60 and 20/60 on `engine`, 65/144 and 60/144 on the strong-wind sweep, and 33% in the 6DOF Monte Carlo.

**What can and can't be recovered:**
- **Physically out of reach:** the climb and on-final cases. They need a different procedure, such as a steeper, power-off-capable approach.
- **Worth winning back:** crosswind and base. The downwind-early returns arrive with hundreds of metres of runway to spare, which suggests the planner is too pessimistic in some geometries. Making the margins more aggressive (margin factor 1.0, height 0–10 m, cheaper reversals) didn't help (16–19/60): more returns were attempted, and they overran or fell short. The next step is to tune the planner on this strict score, with an extra penalty for any return that ends short of the runway.

## Engine failure below 700 ft, field landings, and the runway as a whole

**The turn back ("impossible turn") under 700 ft.** I tested this with `examples/turnback.rs`, allowing a turn back at any height, flown with 25°, 35° and 45° of bank.

That exposed a planner bug. A turn of 180° or more at a waypoint was credited with a big shortcut, `R·(Δ·1.1 − 2·tan(Δ/2))`, which goes strongly negative near 180°. So a turn back from 300 ft looked reachable, and ended in a 22 ft/s impact 600 m short. Turns over 90° at a waypoint are now flown as arc plus a realignment, continuous with the fly-by value at 90°.

With the honest planner, a straight-out climb on the 1,200 m runway never has a turn back in reach at any height. The climb gradient (about 8.6% at Vy) is shallower than the power-off glide gradient (about 11%), so the higher the aircraft climbs, the further outside its glide reach the runway falls. A window exists only when the failure comes just past the end of a long runway:

| Runway | Failure height | Position at failure | Outcome |
|---|---|---|---|
| 3,000 m | 800–900 ft | 450–800 m past the end | Turn back succeeds |
| 3,000 m | 700 ft and below | Over or just past the end | Lands ahead |

That matches the shape of published turn-back analyses: a narrow window that depends on runway length. One analysis gives roughly 330–485 ft for a C172M on a 4,400 ft runway, assuming a perfect 45° turn near the stall and no reaction delay. The simulated window sits higher, because the planner budgets the turn at the circuit bank (25°) and adds margins. Instructors typically set a personal minimum of about 1,000 ft. The 700 ft rule is kept, but the planner, not the rule, now makes the decision.

**Field landings into wind.** With height to spare, a field landing now turns toward the wind:
- any amount above 700 ft
- up to 90° from 400 ft
- up to 30° below that

Before, it kept the track it had at the failure. In 20–25 kt crosswinds that left 9–16° of crab at touchdown.

**The whole runway is usable.** When high on a power-off final, the aim point moves down the runway before slips and S-turns are used. It can go as far as 450 m before the far end, leaving room to flare and stop. Judging reachability to a touchdown just past the threshold was tried and dropped: it removed the flare margin and failed more often.

**Earlier flap.** Once heading for the intercept, flap 10 goes out when the plan shows more than 60 m of surplus height, and flap 20 above 120 m. On a power-off final that is clearly high, flap 20 goes out before the aircraft is lined up.

**Glide footprint, all flight.** The plan view and 3D views now show where a straight power-off glide from here could end:
- **Size:** a circle of radius height × glide ratio.
- **Wind:** its centre drifts with the wind for the time the glide takes.
- **Colour:** green when the planner (turns and margins included) says the runway is in reach.

## 3D view

The map canvas switches between **Plan**, **3D chase** and **3D tower**. In the chase view, drag to orbit and scroll to zoom.

The view is a perspective renderer drawn on the 2D canvas, so there is no dependency. It shows:
- sky and ground split at the horizon
- a 250 m ground grid
- the runway with centreline and threshold bars, plus trees on the short strips
- the planned circuit and the glide footprint on the ground
- the flown track in 3D, coloured by phase, with its ground shadow
- a low-poly C172 drawn at the real attitude: nose heading, pitch and bank

## Random-state auto-land, touchdown accuracy and turn overshoot (23 Oct, evening)

**Random airborne start** (page buttons *Random state* and *Random engine out*; `circuitsim randomland N`). The aircraft is placed 1.5–6 km from the field at a random height, heading, speed and attitude, with enough energy to reach the runway. Up to 70° bank and ±25° pitch are allowed. The autopilot then:
1. **Recover:** rolls wings level at 20°/s. It unloads if α > 0.2 and never pulls beyond 60° bank. It holds level for 2 s. With the engine out, it pitches for best glide instead.
2. **Rejoin:** flies a Dubins path at 80 kt and 25° bank to a final fix 1.5–6 km out. The fix distance follows the height to lose (descent gradient 0.09) or gain (climb 0.06). The aircraft slows to the base speed with flap 20 in the last 1.5 km.
3. **Final:** flies the normal final and landing. With the engine out, it hands over to the glide planner after recovery. If the aircraft is far too high, the plan spends the height: gentler turns, then a stretched path joining further out on the extended centreline.

**Touchdown accuracy.** Every runway landing now scores 100 points on the target touchdown point, minus 1 point per 3 m of miss in either direction. It is reported per suite as an average and counts in the autotune closeness term. The target is:
- normal circuits: the aim point plus the planned float
- glide-backs: the planner's chosen aim

**Turn overshoot** (`cargo run --release --example overshoot`): the largest distance outside the planned ground track in the turn onto final.
- A bank limit now keeps every turn at 1.2× the stall speed for that load factor (`BANK_STALL_MARGIN`). It is applied to powered and power-off turns alike.
- Two tighter power-off settings were tried and dropped. Planning turns at 1.25 Vs√n halved the power-off median overshoot (38 → 18 m), but crosswind-leg failures became unreachable (8/71 → 0/71). Flying turns faster than planned was worse on every count.

| Turn onto final | n | Median | 90% | Worst |
|---|---|---|---|---|
| Normal circuits | 22 | 12 m | 22 m | 27 m |
| Power-off returns | 34 | 1 m | 15 m | 31 m |
| Powered rejoins from a random state | 30 | 5 m | 24 m | 77 m |

**Circuit turns started earlier (23 Oct, later).** The worst normal-circuit overshoots were all a tailwind on base, on the High circuit's 1,000 m base: the aircraft started the base turn at 90 kt, the tailwind carried it through the final, and it ended up to 80 m beyond the centreline. Three changes:
1. **Turn lead in the air mass.** The distance from the next leg's line at which to start a turn is now R(1 − cos Δ) + w·T + half the roll-in distance. Δ is the heading still to turn, R the air-mass radius at the planning bank of 25°, and w·T the wind's drift toward the line during the turn. The old lead used R = vg²/(g·tan φ) for a fixed 90°. On a short base the turn onto final now starts before the turn onto base is finished, and is flown as one turn. Final's speed and glide path take over within 100° of the runway heading.
2. **Closing bank.** During the turn the bank is the one that ends the circle on the line, R = e / ((1 − cos Δ) + w·Δ/v). It starts at 25° and steepens up to 30° if the wind or the speed widens the turn. The stall-margin limit still applies.
3. **Slower into base.** The aircraft slows to 75 kt with flap 10 abeam the threshold. On the base descent, at idle and more than 5 kt above the base speed, it descends less until the speed comes back. Otherwise flap 20 can't come out, and a steep path only adds speed.

Normal-circuit overshoot fell from a median of 16 m and a worst case of 80 m to 12 m and 27 m. Monte Carlo normal circuits went from 81% to 83%, engine failures from 34% to 37% (67% within the ceiling), and stol from 18/24 to 20/24.

Two changes to power-off turns were tried and dropped:
- Rolling into the arcs earlier (0.7 of the roll time instead of 0.5) cut their overshoot (median 38 → 28 m). But Monte Carlo power-off landings fell to 31%, and crosswind-leg returns to 2/71.
- A pitch feed-forward for the bank held the speed through the turn (median 31 m), but power-off landings fell to 33%.

The power-off plan's height budget has no slack for a wider or earlier turn, so those turns stay as planned.

**Orbit down beside the field, not where the engine stopped (23 Oct, later).** With more than 150 m of height to spare, the glide used to orbit wherever the aircraft was. It could be anywhere within 3 km of the field, so it spent height far out, with the runway at the edge of reach. Now it glides straight to an orbit point 700 m abeam the runway's middle, on its own side. It orbits there at 30° until the surplus is under 70 m, then flies the planned path. Random engine-out starts that need an orbit: before, 45 of 47 orbited more than 1.5 km out and 37 landed. Now 34 of 34 orbit within 1.2 km and all land (`examples/orbit_check.rs`). Random engine-out starts overall: 49 → 57 of 100, with 96 runway landings, up from 88.

**Glide footprint with the turn included (23 Oct, later).** The footprint used to be a circle of radius height × L/D, drifted by the wind. That is only true if the aircraft could point in any direction for free. Now it is computed for each final heading, every 5°:
- the turn onto that heading, the short way, at the planning bank and speed, which costs height for the load factor and the roll-in
- then a straight glide at best glide with the height that is left
- all in the air mass, then drifted by the wind for the time it takes

At 1,000 ft in calm air it reaches about 2.6 km ahead but only 1.8 km behind, offset sideways by the turn. In wind it stretches downwind. The planner's own decision, the green or red colour, is unchanged.

**Lined-up final before touchdown** (`examples/lined_final.rs`): the distance flown before touchdown within 5° of the runway heading and 15 m of the centreline.

| Flights | Median | Worst 10% |
|---|---|---|
| Normal circuits | 2,066 m | 1,120 m |
| Power-off returns | 399 m | 206 m |
| Random engine-out starts | 799 m | 364 m |

Two causes, both seen in the traces:
- The planner, when height is tight, picks a join point only 50–150 m before the threshold, because a further-out join costs height.
- The final turn runs 40–50 m wide and is then corrected in an S over the next few hundred metres.

Ranking a 300 m lined-up final above the second half of the height reserve made little difference to the line-up (median 425 m), and cost some landings (Monte Carlo power-off 37% → 35%), so it was dropped.

**No orbits: the plan spends the height (23 Oct, evening).** The orbit beside the field is gone. A high aircraft now follows a plan that uses its height:
1. **Gentler turns.** The planner tries 30°, then 37.5°, before the tuned 45°. It takes the gentlest that still reaches with the full reserve. A gentler turn spends more height on longer arcs. It also leaves bank in hand below the stall-margin limit, so the follower can tighten a turn that runs wide, the one onto final above all. That was the cause of the power-off overshoot: at 45° the turn was flown right at the limit, and a small speed sag held the bank below plan.
2. **A stretched path.** Joins are searched out to 6 km on the extended centreline (200 m steps beyond 1.2 km). They rank after the predicted touchdown in 30 m steps, and the least stretch wins among equals, so the path goes further out only when flap couldn't burn the height on final.

| Measure | Before | After |
|---|---|---|
| Power-off turn overshoot, median / worst | 38 m / 79 m | 1 m / 31 m |
| Lined-up final before touchdown, median: power-off returns | 399 m | 645 m |
| Lined-up final before touchdown, median: random engine-out starts | 799 m | 2,107 m |
| Random engine-out starts landed | 57/100 | 69/100 |
| Random engine-out touchdown accuracy | 22 | 48 |
| Monte Carlo engine failures | 37% | 38% |
| `engine` suite | 21/60 | 22/60 |

The cost: some stretched paths go out to 4–5 km before turning back on a long final, always on a plan that reaches with the reserve.

**Flap first, slip last (23 Oct, evening).** Power-off finals slipped on 98% of landings, about 40 s each (`examples/slip_use.rs`). Two causes:
- **The target point.** The final steered its drag to land on the plan's aim point, and that was usually the runway's first 60 m. The planner aims short to make up for height it expects to burn, and with that target even full flap landed long, so the slip came out.
- **The plan's spare height.** The margins and the 20% reserve are kept all the way down, and they arrive on a short final where flap alone can't burn them.

Four changes:
1. **Nominal target.** The final steers to the nominal touchdown point, or the plan's aim if that is further.
2. **Slip only when far too high.** The slip is used only when full flap would still land more than 150 m past that point, and it must do clearly better. It never retracts flap to slip.
3. **Spare height spent on the way.** A fresh plan replaces the one being flown when it keeps the full reserve and lands at least 80 m nearer the nominal touchdown. The reserve is a share of what is still to fly, so the spare height is spent on the path as the aircraft closes in.
4. **Accuracy scored against the real target.** Power-off touchdown accuracy is now scored against the point the final aims for.

| Measure | Before | After |
|---|---|---|
| Power-off landings that slipped | 98% | 13% |
| Slip time per landing | 40 s | 2.5 s |
| Monte Carlo engine failures | 38% | 41% (73% within the ceiling) |
| `engine` suite | 22/60 | 26/60 |
| `xw_sweep` | 8/71 | 14/71 |
| Power-off touchdown accuracy | 28 | 65 |

**Less high on power-off finals, flap limits, flap and sideslip on the chart (23 Oct, night).**
- **Flap run time.** The final's touchdown prediction now counts the 10 s the flap takes to run, plus 3 s to settle, at the glide actually measured. Before, a selection of full flap at 300 ft landed about 90 m past where it was predicted to.
- **Tapered reserve.** The plan's height reserve is kept in full above 1,000 ft and tapers to a quarter of it lower down. Close in, the final's flap covers what the reserve was for, and a reserve held to the end only arrives as height to burn.
- **Slip threshold.** The slip is used when full flap would still land more than 100 m past the target, down from 150 m.
- **Flap limits.** Flap beyond 10° is now selected only below 80 KIAS, a 5 kt margin under the 85 kt limit, because extending flap pitches the nose down and the speed rises while it runs. With flap out, the nose comes up to hold the speed below the limit. Across 148 flights of every suite (`examples/flap_limits.rs`), 4 flights went over 85 kt with flap beyond 10°, for 0.8 s in total, at most 2.4 kt over, with the flap at 11°. Before: 8 flights, 3.9 s.
- **Chart.** The profile chart has a third panel with flap (°) and sideslip (°) against time.

Power-off landings (`examples/slip_use.rs`, engine suite plus 60 random engine-out starts):

| Measure | Before | After |
|---|---|---|
| Touchdown vs target, median | +147 m long | +115 m long |
| Touchdown vs target, 90th percentile | +401 m long | +325 m long |
| Landings that slipped | 13% | 13% |
| Passed | 82 | 85 |

Other suites:

| Suite | Before | After |
|---|---|---|
| `engine` | 26/60 | 27/60 |
| Random engine-out starts | 67/100 | 77/100 |
| Monte Carlo engine failures | 41% | 37% |
| `xw_sweep` | 14/71 | 12/71 |

The Monte Carlo figure was 37–38% with each change undone in turn, so the earlier 41% looks like a lucky run.

**Why it is still long.** The planning margins (×1.09 + 15 m) and the reserve are kept until the turn onto final. That height arrives on a final that is often under 1 km, where full flap burns only about 2 m per 100 m more than a clean glide. Landing long is the safe side. Landing on the point would need the margins spent before the final, which costs runway landings when the model is wrong.

**Take-off flap** is 10°, retracted at 300 ft (seen in the traces as `fl10`).

## Results

With the scoring from the 23 Oct review: every landing is scored on touchdown speed and nose-up attitude. Glide-back returns are also scored on centreline and crab. These figures can't be compared with earlier ones, which allowed 58 kt flat touchdowns.

| Suite (native, 6DOF) | Now | Touchdown accuracy |
|---|---|---|
| `scenarios`: 14 cases × 2 circuits | 22/28 | 81 |
| `stol`: 4 winds × 2 seeds per strip | 20/24 | |
| `engine`: 60 cases | 27/60; 27/35 within the physical ceiling | |
| `montecarlo 400`, normal circuits | 83% | 84 |
| `montecarlo 400`, engine failures | 37%; 77/113 (68%) within the ceiling | 71 |
| `randomland 100`, engine running | 85/100 (upsets 48/56) | 83 |
| `randomland 100`, engine out | 77/100 | 59 |
| `xw_sweep` (crosswind-leg failures) | 12/71 | |

The stall-margin bank limit costs some glide-back reach. Before it was added, the results were engine 25/60, Monte Carlo engine failures 36%, and xw_sweep 14/71. The limit is kept because a turn at 45° bank and best-glide speed has only about 1.1× the stall margin.

Monte Carlo engine failures by failure point:

| Failure point | Cases | Ceiling | Passed |
|---|---|---|---|
| Take-off roll 40 kt | 21 | 21 | 21 |
| Climb 200 / 450 ft | 40 | 0 | 0 |
| Crosswind leg | 22 | 15 | 5 |
| Downwind early | 25 | 22 | 10 |
| Downwind mid | 19 | 19 | 12 |
| Abeam threshold | 19 | 19 | 12 |
| Base | 18 | 6 | 1 |
| Final 400 ft | 19 | 3 | 4 |
| Final 150 ft | 31 | 11 | 8 |

**Where it fails now:**
- **Gusty landings, normal and power-off.** The hold-off makes them harder: a lull in the float drops the aircraft in. This is the largest group: 13 hard, 13 flat and 12 crabbed among the gusty normal circuits; 24 hard and 11 flat among the power-off ones. The old scoring hid it by letting every landing hit the runway at approach speed.
- **Power-off overruns** (27). The hold-off float from the power-off approach speed is planned, but not always accurately.
- **Crosswind-leg and base failures** reachable in principle but planned or flown short. The margins and reserve are what the tuner adjusts.
- **Direct crosswinds of 15 kt and more.** The rudder holds about 11° of sideslip at touchdown speed, close to the C172's demonstrated 15 kt.
- **Beyond the ceiling.** Climb-out failures and the Standard circuit's base and final can't be saved by any plan.

The Monte Carlo runs use winds of 0–15 kt from any direction, 40% of them gusty, across both circuits and random failure points.

Earlier results (strong-wind power-off sweep, the pre-review failure analysis) are in the dated sections above. They used the old planner and the old scoring.

**Parity checks:**
- The first Rust port matched the original TypeScript core (since removed; it is in the git history) on all 88 cases (normal and engine failure, both circuits), to printed precision: `ts/test/parity.ts` against `circuitsim parity`. The autopilot changes above came afterwards.
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
./target/release/circuitsim tunecheck         # inner-loop step responses + margin check
./target/release/circuitsim autotune 6dof 14 120 2   # generations, training flights, threads
cargo test --release --lib --test suites       # unit tests + whole-flight regressions (tests/golden.txt)
cargo run --release --example xw_sweep         # engine failures along the crosswind leg
cargo run --release --example energy_check     # planner energy model against the flown glide

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
| Flare | Settles to 0.4 m/s from 2.7 s of sink | Firmer: 0.7 m/s from 2.3 s, so less float. Softer and earlier in measured turbulence |
| Braking | Full after 2 s | Full after 0.5 s, flaps up |
| Long-landing go-around | Past the first third + 50 m | Past aim + 130 m |

**Strips:** 1,200 m (paved, 30 m wide), or 600 m and 450 m (20 m wide) with 50 ft trees 150 m beyond each end.

**Landing check:** it adds take-off distance (ground roll and to 50 ft) and landing distance (from 50 ft and ground roll), plus tree clearance on departure and approach. For short-field, the touchdown point must be within 100 m of the aim point. After two long-landing go-arounds, the autopilot accepts the long landing rather than circling forever.

**Power-off landings:** with short-field selected, power-off landings brake at once. They keep the normal 66 kt power-off speed and the gentler flare: with the C172P drag, the 60 kt short-field speed left too little energy to flare without power (11/20 instead of 19/20 on the 450 m strip). The power-off aim point is capped at a quarter of the strip.

**Results** (`circuitsim stol [6dof]`, High circuit, 4 winds × 2 seeds). The winds are calm, a 10 kt headwind, a 10 kt crosswind, and 10 kt gusting.

| Strip | Normal (6DOF / point-mass) | Short-field (6DOF / point-mass) |
|---|---|---|
| 1,200 m | 8/8 / 7/8 | 7/8 / 7/8 |
| 600 m | 0/8 / 0/8 | 7/8 / 7/8 |
| 450 m | 0/8 / 0/8 | 7/8 / 7/8 |

**Typical 6DOF short-field figures in calm air:**
- **Take-off:** 199 m ground roll, 359 m to 50 ft.
- **Landing:** 204 m ground roll, 409 m from 50 ft, touching down 87 m past the threshold at 2.1 ft/s.

The POH short-field figures at 2,400 lb are about 270 m ground roll and 495 m to 50 ft for take-off, and about 165 m ground roll and 380 m from 50 ft for landing. This model is 10% lighter.

Normal technique touches down beyond the first third of the short strips, which is why short-field technique exists. The gusty short-field case used to land hard: 6.0 ft/s after a lull at 5 ft. With lull protection and the hand-tuned values it passed, at 2.4 ft/s. With the autotuned values it lands softly, at 1.4 ft/s, but 165 m past the threshold, 25 m beyond the touchdown window.

**Engine failures on the 450 m strip** with short-field technique (20 cases):

| | Before | Now |
|---|---|---|
| 6DOF | 17/20 | 18/20 (19/20 hand-tuned) |
| Point-mass | 17/20 | 18/20 |

Nothing now runs off the end. The remaining failures are hard arrivals.

## Next

- **Auto-land from a random state:** an unusual-attitude recovery phase, then the Dubins planner with power to a final fix, scored by the same rows against a "recoverable" ceiling (discussed 23 Oct).
- **Gusty landings with the hold-off:** a shorter hold-off in turbulence, or a gust-aware sink floor.

- **A longer tuning run:** more training flights (240+), the `stol` and `engine` suites added to the training mix, and a start from the current values.
- **Gusty power-off arrivals:** these are the largest remaining failure group, at 3.2–4.4 ft/s. Options are a flare that reads the gust estimate and holds a little more speed into the last few feet, or a predicted-lull pitch feed-forward.
- **Foam RC model:** see `notes/foam-rc-autopilot.md` for how this design transfers to a small electric model, and `notes/vertical-landing-and-autotune.md` for tuning a real one.
