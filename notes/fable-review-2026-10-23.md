# Review: `circuitsim` autopilot (Rust, 6DOF C172P)

Scope: `rust/src/{autopilot,emergency,control,aircraft,airfield,simulation,tuning,autotune}.rs`, `sixdof/{mod,fcs}.rs`, `c172p.rs`, README. All suites were rebuilt and rerun (`engine` 34/60, `scenarios` 24/28, `stol` 24/24 pass-rate table, `montecarlo 400` 50% / 99%, `xw_sweep` 24/71), matching the README's 23 Oct figures. To judge physical limits I wrote a Dubins-path glide oracle (45° bank, measured L/D 8.5, turn loss 26 m/rad) in a scratch copy and ran it over the engine suite, the crosswind sweep and the Monte Carlo set; it is quoted below with its caveats.

Findings are ordered by importance.

---

## 1. The engine-out route structure, not the energy margins, is what loses the recoverable cases

**Evidence.** The planner (`emergency.rs:144-154, 246-307`) can only express one route shape: *base key abeam an intercept point → intercept point on the extended centreline (500–3000 m out) → aim point*. Reachability is the height needed for that polyline plus hand-fitted turn costs. The autopilot then flies it as `DIR KEY → BASE → final_turn_due → EoFinal` (`autopilot.rs:839-884`).

Crosswind-leg failure, High circuit, wind 090/8, 25 s into the leg (`examples/xw_case 90 8 25`): aircraft at (1781, 630) m, 780 ft, heading north; runway 27's threshold is 580 m west. A 90° left turn onto a long final needs ~130 m of height against 238 m available (oracle surplus +105 m). The planner's best route scores *surplus 0*, sends the aircraft north-east to a base key at y ≈ 700 m ("DIR KEY", track 358→25°, bank −22°, 240 ft burned in 10 s), then re-plans to "lost", and lands in a field 1.5 km away. Same wind at 15 kt, 20 s: it reaches the key, then `line_bank` → `bank_for` (`control.rs:77-82`) takes the *shorter* heading change of a ~200° requirement and flies a 270° right turn (track 38→95→177→250) to reach a left-hand final; it arrives at 50 ft still 116 m off the centreline and touches down 24 m off, "off the runway", with 2 m of planner surplus. Both are reachable states lost to geometry logic.

Across the crosswind sweep the oracle says 27/71 reachable and the sim passes 24, but only 12 are the same cases. In the 400-flight Monte Carlo (214 engine failures), 18 flights the oracle calls reachable with ≥ 2 m surplus failed; 12 of them were `LandAhead` decisions with 11–91 m of surplus (6 crosswind, 3 downwind-early, 3 base).

**Impact.** The crosswind, base and part of the downwind-early gap (section 7) is in the planner's vocabulary and the lateral law, not in energy. Tuning `eo_margin_factor`/`key_margin` (the README's "more aggressive margins didn't help") cannot fix a route that goes the wrong way first.

**Change.** Replace the waypoint polyline with a Dubins (turn–straight–turn) path, at the bank actually flown, from the current ground position/track to a *final fix* (distance d before the threshold on the extended centreline, heading = runway heading), for each end and a handful of d values; compute it in the air-mass frame with the target shifted by wind × estimated time (two iterations are enough). Energy required = straight length / L/D(config, wind) + total turn angle × (height loss per radian, measured once from the model). This deletes `waypoints`, `turn_reach`, `KEY_CUT_MAX`, `REVERSAL_COST*`, `REALIGN`, the `at_wp` corner formulas, `passed_base_key`, `final_turn_due`, the three-way `match wp.base_key` and `eo_turn_bank`, and gives the lateral channel an explicit path to follow (one vector-field law along the arcs and the straight) instead of DIR KEY / BASE / line_bank / S-turn. Risk: Dubins assumes instantaneous roll and constant radius; the roll-in costs a few metres and wind turns circles into trochoids — both are second order at ≤ 15 kt and covered by a fixed height allowance; the first implementation should be validated against the existing suites and against the trace tools.

## 2. Scoring is asymmetric and lets the landing technique be wrong

**Engine-out returns have no alignment criteria.** `simulation.rs:452-474` scores a return on sink, "touched down on runway" and "stopped on runway" only; the crab and centreline rows exist only for field landings (`476-481`). Passing returns in the engine suite include 7.6° crab / 5.5 m off, 6.8° / 6.7 m and 3.6° / 12.4 m on a 30 m runway (scratch `examples/crab.rs`). Normal circuits require < 5° and < 5 m; four of these passes would fail under those rules. The README's "only a runway counts" strictness is partly offset by this.

**Normal landings are fast and flat.** Trace of the calm Standard case (`circuitsim trace 270 0 0 std`): touchdown at **58 kt, pitch 4°, flap 30**, elevator −7° of −28° available. Flap-30 stall is 45 kt in this model, so touchdown is at 1.29 Vs0, versus roughly 1.05–1.1 Vs0 and 8–10° nose-up for a real C172. This is why the measured landing distance (597 m from 50 ft, 250 m roll in calm) is ~55% longer than the POH 381 m / 158 m at a *heavier* weight, and why normal technique cannot use a 600 m strip. The flare law (`autopilot.rs:701`, sink = floor + k·h) controls vertical speed only; there is no hold-off, no speed target, and the scoring (`simulation.rs:339-345`) checks sink, position, centreline, crab and stop — never speed or attitude — so the tuner was never asked to fix it. Short-field technique gets its 2.3 ft/s arrivals by aiming for a 0.7 m/s floor, i.e. by design it lands at ~77% of the limit.

**Minor.** "Stopped on runway" for normal circuits checks only `x <= length` (`simulation.rs:362`): no `x >= 0`, no lateral check (the EO path uses `contains`). The Standard-circuit 20–25 kt direct-crosswind cases are beyond the airframe (README agrees) but still sit in a pass/fail suite.

**Change.** Add touchdown speed (≤ 1.15 Vs0) and pitch (≥ 5°) rows and a crab/centreline row for returns; add a speed-bleed term to the flare (target V decreasing toward 1.1 Vs0 as h → 0, or a TECS-style flare); retune the flare leads afterwards, since the autotuned 2.2 s lead was fitted to the current law.

## 3. The autopilot reads the true wind, and sees a perfect state

The README states that turbulence is "measured, not given", which is true for σ (`GustEstimator`), but everything else reads the simulation's wind object: the crab feed-forward uses the instantaneous true wind including gusts (`autopilot.rs:463`), `final_turn_due` uses `wind.v.y` (`:976`), the planner uses `wind.along()` and `wind.v` throughout (`emergency.rs:151, 200, 280, 312`), and `field_track` reads the scenario's *settings* (`autopilot.rs:938-943`). The rest of the state (γ, α-filtered, h to the centimetre, β) is copied from the rigid body (`aircraft.rs:457-474`) with no noise, lag or bias. Perfect wing-low feed-forward from a perfect crosswind is one reason gusty approaches pass at 99%; a real installation gets wind from GPS−air data with seconds of lag. This is acceptable for a proof of concept but should be stated, and the planner should be fed the *estimated* mean wind, which already exists in `GustEstimator::w_lp` (`control.rs:189`) but is private.

## 4. Mode logic and the plant are not cleanly separated

- `fail_engine(&mut Aircraft)` sets `ac.engine_failed` and rewrites the aircraft's command-shaper rate (`autopilot.rs:346-347`); the autopilot mutates the plant.
- `update()` starts from `let mut c = self.cmd` (`:515`): commands not written by a phase are inherited from the previous step, so a phase's output depends on history. The go-around flap logic (`:743-748`) relies on this.
- Phase code reaches into controller internals: `self.path.gamma_pi.i = 0.0` (`:545`), `self.climb.engage(0.06)`.
- `eo_glide` switches to `ForcedLanding` by assigning `self.phase` directly (`:909`), bypassing `set_phase`, and does not reset `eo_flaps`, so `ForcedLanding` (`:775`) can retract flap that `eo_glide` extended.
- Engine-out behaviour is spread over `fail_engine`, `eo_glide`, `eo_final`, and the EO branches inside `Flare`, `Rollout` and `ForcedLanding`.
- Two sources of truth for parameters: `default_eo()` (`emergency.rs:47-62`: vbg 70, vapp 66, margin 1.1/20 m, key margin 150) is overwritten field by field in `Simulation::new` (`simulation.rs:152-157`) from `Tuning`, and the FCS gains are copied by hand (`:136-148`) on top of `FcsGains::default()` which already reads `Tuning::default()`. `Autopilot::new` alone yields a planner that differs from what the suites test.
- Three angle conventions (JSBSim, NED body, math CCW) meet in `observe()` (`sixdof/mod.rs:545-575`: `phi: -phi`, `p: -p`, `psi = π/2 − ψ`, `wind_ned = [v.y, v.x, 0]`). It is correct, but nothing asserts it: there are no tests of sign conventions, nor of anything else except one table lookup (`c172p.rs:284-298`). The planner and the geometry helpers are pure functions and would be trivial to unit-test.
- Mixed units in one function with no suffix: `ac.h < 60.0` and `ac.h < 30.0` are metres (`autopilot.rs:563, 569, 648, 1065`) next to `100.0 * FT` and `s.align_height` (60 ft). The 10° bank limit "below 60" in the climb is therefore below 197 ft; if 60 ft was intended, it is a bug.
- `CircuitSpec.vy` is 65 kt (`airfield.rs:75`) after the crosswind-leg change; it is no longer Vy (74–76 kt) and is used as the climb target everywhere.

## 5. `eo_final` is a threshold ladder that should be one energy error

`autopilot.rs:983-1069` decides flap (extend loop, retract loop, `flap_cap` from `lined_up || low || clearly_high`, `reversing`), aim-point shift (`along_more`), slip (`aligned && flaps ≥ 20 && h > 100 ft && greq > g20·1.05`), S-turns (`h > 350 ft && dist > 500 && greq > g20·1.25·1.4`) and flap 30, with factors 0.95, 1.05, 1.2, 1.25, 1.4 and heights 100/300/350 ft that all interact. Sections 1–2 of the README's "order of energy measures" are a pilot's rule of thumb encoded as nested gates.

**Change.** Compute one signal: height error relative to the reference glide to the chosen aim point in the *current* configuration, with the aim point itself chosen by the planner (any point from threshold+60 m to length−450 m). Map that error monotonically onto a drag ladder (flap 10 → 20 → slip → 30) with hysteresis, and keep S-turns only as a path-length tool from the Dubins planner (longer final fix) rather than a lateral special case. Flap retraction when short follows from the same error with the opposite sign. This removes roughly 50 lines and five tunable-but-untuned constants (`S_TURN_*`, `SLIP_SINK_GAIN`, `EARLY_FLAP_SURPLUS`, `RETRACT_FLOOR`).

## 6. Tuning parameters compete with hand-coded constants

- The planner costs every turn at 25° bank with an 8°/s roll (`emergency.rs:134-137, 267`), while the aircraft flies turn-backs at `tune.eo_turn_bank` = 45° (`autopilot.rs:929-932, 1065`). The searched parameter changes the flying but not the plan that decides whether to fly.
- `STRETCH = true` (`autopilot.rs:193`) re-checks reachability with margins removed whenever the margined plan fails (`:360, :894-902`). With that fallback, `eo_margin_factor` and `eo_margin_height` barely matter, which is why the tuner drove them to their floors (README, "Tuning on 6DOF with strict scoring"): two mechanisms, one of which makes the other's parameter meaningless.
- `VBG_HEAD_K = 0.0` (`:190`) is a dead experiment left compiled in; `tune.vbg_kt` then sits at 68.6 kt because the trim polar has a step there (section 8).
- `DIMS` caps `eo_turn_bank` at 45° (`tuning.rs:112`) while the README's final-values table says 50°. The defaults table and `Tuning::default` disagree.
- Lull protection mixes a tuned threshold (`lull_vs`) with hard-coded 0.3/0.6/0.3/0.7 shaping (`autopilot.rs:669-674`).
- The autotuner trains 20 parameters on 120 fixed flights with 8 candidates per generation and ~150 evaluations; 2 of the 9 engine-failure triggers in the training mix are unwinnable (section 7) and contribute only soft-cost noise. Fine for a demonstration, under-sampled for a claim of robustness.

## 7. Where the pass rates stand against physics

Method: for each failure state record (x, y, h, V, track, wind) and test a Dubins path at 45° bank to any aim point on either runway, with L/D 8.5 (measured trim, `examples/glide_speed`) and 26 m of height per radian of turn (sink × n^1.5 at n = 1.41, ÷ turn rate). Wind is handled by shifting the target in the air mass. It is an estimate: it is pessimistic for lined-up finals with a crosswind (it demands a Dubins turn to absorb the drift) and ignores flap, so treat ±40 m of surplus as noise.

Engine suite (6 cases per trigger, winds ≤ 12 kt), oracle-reachable / sim-passed: abeam 6/5, base 3/2, climb 200 and 450 ft 0/0, crosswind 4/4, downwind early 6/6, downwind mid 6/6, final 150 ft 2/3, final 400 ft 1/2, reject 6/6. On this suite the autopilot is already at the geometric ceiling; the two misses are a gusty 3.6 ft/s flare and a 10 m-surplus base case.

Monte Carlo (214 failures, winds ≤ 15 kt, 40% gusty), reachable / passed: abeam 19/19, base 8/4, climb 0/0 (40 cases), crosswind 12/9 (only 5 common), downwind early 25/20, downwind mid 19/17, final 150 ft —/12 of 31 (oracle not valid here), final 400 ft 1/4, reject 21/21.

What the geometry says per failure point, and an achievable rate:

| Failure point | Physics | Ceiling | Now |
|---|---|---|---|
| Take-off roll | trivial | 100% | 100% |
| Climb 200 / 450 ft, straight out | At 450 ft the aircraft is 450–800 m past the end of a 1,200 m runway; a 180°+ turn costs ~270 ft, leaving ~470 m of glide for ≥ 700 m of path. Oracle −73 to −223 m. Agrees with the turn-back literature (C172 needs ~600–1,000 ft, and a long runway) | 0% on this runway | 0% |
| Crosswind leg | Reachable only in the second half of the leg (oracle: delays ≥ 15–20 s; first-half surplus −50 to −190 m) | ~55–60% | 43% (MC), 34% (sweep) |
| Downwind early / mid, abeam | 130–250 m of surplus; failures are gusty flares and three pessimistic `LandAhead` calls | ~95% (gust-limited) | 80 / 89 / 100% |
| Base | Standard circuit: 2.6 km out at 490 ft — needs 17:1, impossible. High: 1.5 km at 700 ft, 10–40 m surplus, headwind-dependent | ~40% (0% Std, ~75% High) | 22% |
| Final 400 ft | Standard: 2.1 km at 122 m, impossible. High: 1 km, needs 8.3:1 against 8.5 clean — calm/tailwind only | ~20% | 21% |
| Final 150 ft | Standard: 620 m at 46 m, needs 13.5:1, impossible. High: 225 m, easy | ~50% | 39% |

Weighted over uniform triggers, the Monte Carlo ceiling is ~57–60% against the current 50%. The gap sits in crosswind (+15 pts of that trigger), base on the High circuit (+20), and gusty power-off flares on downwind (+15). Half of the Standard-circuit failure points are unrecoverable by any technique because the circuit is flown 2.6 km out on a 3° slope; the High circuit demonstrates the fix (engine suite 19/30 vs 15/30). The README's "physically out of reach" list (climb, Standard final 400) is correct; "crosswind and base are worth winning back" is also correct, and section 1 is where the gain is.

Sanity checks against the real aircraft: take-off 233 m / 478 m to 50 ft at 1,000 kg against POH 270/495 m at 1,089 kg — plausible. Short-field landing 204/409 m against POH 165/380 — plausible. Normal landing is not (section 2). Climb 660 fpm at 75 kt matches the POH. Approach speeds (65 kt normal, 60 short-field, 67 power-off with flap) are POH-like. Sink rates: 805 fpm minimum sink at 70 kt is ~15% worse than the POH-implied ~700 fpm (65 KIAS, 9:1), so the reachability conclusions are slightly conservative relative to the real aircraft.

## 8. Model artefacts worth knowing

- The trim polar (`glide_speed`) jumps from 805 fpm at 70 kt to 890 fpm at 72 kt and L/D 8.74 → 8.13. That is not physical; the JSBSim CL table has a slope break at α = 0.09 rad (0.73 → 0.83 over 0.01 rad, `c172p.rs:161-182`), and the 70 kt trim sits exactly on it. "Best glide 70 kt" and the tuned `vbg_kt` 68.6 are fitted to this knot. The README's "9.5 at 68 kt, close to the POH" (from `modes`, apparently without windmill drag) and "8.7" (with) are both quoted; only the latter governs the planner.
- The gust model is horizontal OU noise, σ 1.8 m/s, τ 3 s (`atmosphere.rs:46`); no vertical gusts, no wind gradient with height, no shear. The boundary-layer gradient is the dominant real effect on the last 200 ft of a landing and is absent, so the 3 ft/s gusty flare results are optimistic for a 1:1 transfer.
- Numerics are sound: RK4 at 200 Hz, quaternion renormalised each step, clamped tables, guarded divisions (`v.max(20.0)`, `l.max(1.0)`), conditional integration in the PI. `follow_with` divides by leg length with no guard (`control.rs:68-70`), harmless for the fixed legs. `wrap_pi` loops rather than using `rem_euclid`; fine.
- The inner loops (`fcs.rs`) are tidy: rate-on-attitude with turn compensation, dynamic-pressure scheduling divided by the slipstream factor, dihedral/rudder feed-forward, conditional roll integrator. The margin check (`autotune.rs:278-298`) is a genuine robustness gate (gain ×1.6, lag ×2.5) and is the best piece of test engineering in the crate.

## Prioritised changes

1. **Dubins glide planner + single path-following law** (section 1). Replaces `waypoints`/`turn_reach`/`assess_cfg` turn heuristics, `final_turn_due`, `eo_turn_bank`, DIR KEY/BASE/S-turn branches. Expected: crosswind sweep from 24/71 toward ~35–40/71, Monte Carlo EO +5–8 points. Validate with the existing trace tools before deleting the old path.
2. **Fix the scoring gaps** (section 2): crab/centreline rows for returns, touchdown speed and pitch rows for every landing, `contains()` for the normal stop check. Then re-run the tuner; expect the flare to need a speed term.
3. **One energy-error signal in `eo_final`** with a monotone drag ladder (section 5); fold aim-point selection into the planner.
4. **Feed the planner and feed-forwards the estimated wind** (`GustEstimator::w_lp`), not `wind.v` or `wind.settings` (section 3), and state the remaining perfect-state assumptions in the README.
5. **Remove competing mechanisms**: either keep stretch and drop the margin parameters from `DIMS`, or drop stretch and let one margin set be tuned; make the planner's turn bank the one actually flown; delete `VBG_HEAD_K`, align `default_eo()` with `Tuning` (single constructor path).
6. **Unit tests** for the pure parts: sign conventions through `observe()`/`quat_from_euler` round trips, planner reachability on a few hand-computable geometries, flare law monotonicity, and a golden-file regression of the `parity` CSV so that suite drift is caught by `cargo test`, not by reading the README.
7. **Report engine-failure results per failure point against a stated ceiling**, and move the unwinnable triggers (climb; Standard base/final) out of the headline Monte Carlo rate, or change the Standard circuit so that they are not unwinnable (tighter base, steeper approach — the High circuit already is that).
8. **Investigate the 70–72 kt trim discontinuity** (section 8) before trusting any tuned speed near 68–70 kt; if it is the CL table knot, smooth it or move `vbg` off it.
