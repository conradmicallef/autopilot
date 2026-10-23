# Vertical landing on a powerful foamie, and auto-tuning

Notes from the design discussion, 22 Oct 2026.

## 1. Prop-hang / tail-sitter vertical landing

**Physics:**
- Needs thrust/weight of about 1.5–2 (3D-style power).
- Elevator and rudder work only in the propwash. The slipstream term from the JSBSim C172 (Cmδe and Cnδr on propwash q̄) becomes all the authority there is in a hover, and it depends on throttle, not airspeed.
- Ailerons in the wash counter motor torque, which is large in a hover.
- Hover current is high: 30–90 s of hover, so check the battery before committing.

**Model changes:**
- **Aerodynamics:** cover α from 0 to 90°, with a post-stall flat-plate model (CN ≈ 1.2·sin α).
- **Propeller:** thrust at zero and negative axial speed, and prop gyroscopic moments.
- **Descent:** keep it slow (≲0.5 m/s, well under the induced velocity) to stay out of vortex-ring-like wash.
- **Weathervaning:** gusts push the large side area around.

**Control changes:**
- Euler commands break at θ = 90°, so use quaternion attitude targets.
- The axes swap in the hover: ailerons give heading; elevator and rudder give tilt.
- Throttle controls vertical speed; tilt controls position.
- Schedule gains on propwash q̄ instead of airspeed.

**Phases:**
1. Approach into wind.
2. Transition: pitch to 70–90° while throttle rises to hover.
3. Hover over the touchdown point.
4. Vertical descent on the lidar at 0.3–0.5 m/s.
5. Touchdown on tail legs, or cut power at ~0.5 m and drop onto the belly.

The go-around becomes "power up, pitch down, dive out". An engine failure in the hover is a drop, so commit to the transition only above a safe height with the battery checked.

**Easier alternative:** a quadplane (four lift motors plus the wing), well supported by ArduPilot's QuadPlane modes. ArduPilot supports tail-sitters too.

## 2. Auto-tuning

**In the sim:**
- **Parameters:** FCS gains, flare lead and floor, approach and gust speeds, shaper rates.
- **Objective:** Monte Carlo pass rate, plus margins (sink, touchdown dispersion, float).
- **Optimiser:** random search, then CMA-ES or Bayesian optimisation.
- **Fair comparisons:** fixed seeds, so candidates are compared without noise; separate training and test seeds.
- **Robustness:** randomise the model too (derivatives ±20%, CG, servo delay).
- **Hard constraint:** gain margin ≥ ~6 dB and phase margin ≥ ~45° on the linear model.
- **Cost:** with rayon, 400 flights take ~5 s on 8 cores, so a few hundred candidates take under an hour.
- **Starting point:** model-based gains (LQR or loop-shaping) give a good start and a gain schedule.

**On the real aircraft:**
- **Inner loops:** in-flight autotune (ArduPilot AUTOTUNE adapts FF/P/I/D from rate steps), or system ID from chirps or doublets, then recompute the gains. Do it at altitude, with oscillation abort and pilot override.
- **Landing parameters:**
  1. Identify the model from logs.
  2. Tune in the sim with randomisation.
  3. Then make small, bounded iterative-learning steps across real landings, using the measured sink and float.
