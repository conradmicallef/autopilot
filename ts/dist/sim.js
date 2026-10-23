"use strict";
(() => {
  // src/units.ts
  var G = 9.81;
  var RHO = 1.225;
  var KT = 0.514444;
  var FT = 0.3048;
  var D2R = Math.PI / 180;
  var clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));
  function wrapPi(a) {
    while (a > Math.PI) a -= 2 * Math.PI;
    while (a <= -Math.PI) a += 2 * Math.PI;
    return a;
  }
  var slew = (current, target, rate, dt) => current + clamp(target - current, -rate * dt, rate * dt);
  var headingDeg = (psi) => ((90 - psi / D2R) % 360 + 360) % 360;

  // src/control.ts
  var PI = class {
    constructor(kp, ki, iMin, iMax, out = [-Infinity, Infinity], antiWindup = false) {
      this.kp = kp;
      this.ki = ki;
      this.iMin = iMin;
      this.iMax = iMax;
      this.out = out;
      this.antiWindup = antiWindup;
      this.i = 0;
    }
    reset(i) {
      this.i = clamp(i, this.iMin, this.iMax);
    }
    /** Integrate first, then compute output (used where the integrator leads). */
    update(e, dt) {
      this.i = clamp(this.i + this.ki * e * dt, this.iMin, this.iMax);
      return clamp(this.i + this.kp * e, this.out[0], this.out[1]);
    }
    /** Output from the current integrator, then integrate only if that does not deepen saturation. */
    updateConditional(e, dt) {
      const u = this.i + this.kp * e;
      const [lo, hi] = this.out;
      if ((u < hi || e < 0) && (u > lo || e > 0)) this.i = clamp(this.i + this.ki * e * dt, this.iMin, this.iMax);
      return clamp(u, lo, hi);
    }
  };
  var TrackGuidance = class {
    constructor(approachAngle = 60 * D2R, gainDistance = 180, turnTimeConst = 3) {
      this.approachAngle = approachAngle;
      this.gainDistance = gainDistance;
      this.turnTimeConst = turnTimeConst;
    }
    follow(ac, A, B) {
      const dx = B[0] - A[0], dy = B[1] - A[1], L = Math.hypot(dx, dy), ux = dx / L, uy = dy / L;
      const e = ux * (ac.y - A[1]) - uy * (ac.x - A[0]);
      const remaining = (B[0] - ac.x) * ux + (B[1] - ac.y) * uy;
      const desiredTrack = Math.atan2(uy, ux) - this.approachAngle * (2 / Math.PI) * Math.atan(e / this.gainDistance);
      return { crossTrack: e, remaining, desiredTrack };
    }
    bankFor(ac, wind, desiredTrack, limit) {
      const gv = ac.groundVelocity(wind), chi = Math.atan2(gv.y, gv.x), Vg = Math.hypot(gv.x, gv.y);
      return clamp(Math.atan(Vg * wrapPi(desiredTrack - chi) / (G * this.turnTimeConst)), -limit, limit);
    }
    /** Distance before a leg's end to start a 90° turn: radius plus roll-in distance. */
    turnLead(ac, wind, bankMax, rollRate) {
      const gv = ac.groundVelocity(wind), Vg = Math.hypot(gv.x, gv.y);
      return Vg * Vg / (G * Math.tan(bankMax)) + Vg * (bankMax / rollRate) * 0.5;
    }
  };
  var FlightPathController = class {
    constructor() {
      this.gammaPI = new PI(0.8, 0.4, -0.08, 0.08);
    }
    gamma(ac, gCmd, dt) {
      const err = gCmd - ac.gamma;
      this.gammaPI.i = clamp(this.gammaPI.i + this.gammaPI.ki * err * dt, this.gammaPI.iMin, this.gammaPI.iMax);
      return clamp(ac.alphaF + gCmd + this.gammaPI.kp * err + this.gammaPI.i, ac.spec.minPitch, ac.spec.maxPitch);
    }
    verticalSpeed(ac, vs, dt) {
      return this.gamma(ac, Math.asin(clamp(vs / Math.max(ac.V, 10), -0.25, 0.25)), dt);
    }
    altitude(ac, hCmd, dt, vsMax = 3) {
      return this.verticalSpeed(ac, clamp(0.15 * (hCmd - ac.h), -vsMax, vsMax), dt);
    }
  };
  var AutoThrottle = class {
    constructor() {
      this.pi = new PI(0.1, 0.03, 0, 1, [0, 1]);
    }
    /** Bumpless engage from the current throttle. */
    engage(currentThr) {
      this.pi.reset(currentThr);
    }
    update(ac, Vtarget, dt) {
      return this.pi.updateConditional(Vtarget - ac.V, dt);
    }
  };
  var SpeedOnPitch = class {
    constructor(kp = 0.015, ki = 4e-3, iMin = -0.05, iMax = 0.2) {
      this.pi = new PI(kp, ki, iMin, iMax);
    }
    engage(gamma) {
      this.pi.reset(gamma);
    }
    /** Returns a flight-path angle command. */
    update(ac, Vtarget, dt, gMin) {
      const e = ac.V - Vtarget;
      this.pi.i = clamp(this.pi.i + this.pi.ki * e * dt, this.pi.iMin, this.pi.iMax);
      return clamp(this.pi.i + this.pi.kp * e, gMin, 0.2);
    }
  };

  // src/emergency.ts
  var DEFAULT_EO = {
    Vbg: 68 * KT,
    VappEO: 63 * KT,
    minTurnbackHeight: 700 * FT,
    ipMin: 500,
    ipMax: 3e3,
    aimPoint: 300,
    baseKeyOffset: 450,
    marginFactor: 1.1,
    marginHeight: 20,
    pitchRate: 5 * D2R
  };
  function runwayEnds(runway) {
    return [
      { name: "09", heading: 0, threshold: 0, dir: 1 },
      { name: "27", heading: Math.PI, threshold: runway.length, dir: -1 }
    ];
  }
  var GlidePlanner = class {
    constructor(runway, spec = DEFAULT_EO) {
      this.runway = runway;
      this.spec = spec;
      this.ends = runwayEnds(runway);
    }
    ipAt(distance, end) {
      return [end.threshold - end.dir * distance, 0];
    }
    aimFor(end) {
      return [end.threshold + end.dir * this.spec.aimPoint, 0];
    }
    /** Along-approach distance still to go to the aim point (positive before it). */
    toAim(x, end) {
      return end.dir * (this.aimFor(end)[0] - x);
    }
    /**
     * Waypoints for a power-off return: a base key point abeam the intercept (on the aircraft's side), then the
     * intercept. The base key is skipped when the aircraft is already out beyond the intercept point.
     */
    waypoints(ac, ipDistance, end) {
      const ip = this.ipAt(ipDistance, end);
      if (end.dir * (ac.x - ip[0]) < 200) return { baseKey: null, ip };
      const side = ac.y >= 0 ? 1 : -1;
      return { baseKey: [ip[0], side * this.spec.baseKeyOffset], ip };
    }
    /**
     * Choose the longest final the energy allows, so surplus height is spent on a long straight final
     * instead of arriving high: scan intercept points from far to near (50 m steps) and take the first
     * that is reachable with margin. Returns the closest-in attempt (unreachable) when none is.
     */
    planFor(ac, wind, end) {
      let last = null;
      for (let d = this.spec.ipMax; d >= this.spec.ipMin; d -= 50) {
        last = this.assess(ac, wind, d, end);
        if (last.reachable) return last;
      }
      return last;
    }
    /**
     * Plan for both landing directions and pick one: the preferred end if it still works (no switching back and
     * forth mid-glide), otherwise whichever reachable end leaves the most height in hand; if neither is reachable,
     * the one that comes closest.
     */
    plan(ac, wind, prefer) {
      const options = this.ends.map((e) => this.planFor(ac, wind, e));
      const kept = prefer && options.find((o) => o.end.name === prefer.name && o.reachable);
      if (kept) return kept;
      return options.sort((a, b) => Number(b.reachable) - Number(a.reachable) || b.surplus - a.surplus)[0];
    }
    /** Still-air reach in metres from the current height (clean, best glide). */
    stillAirReach(ac) {
      return ac.h * ac.glideRatio(this.spec.Vbg, 0);
    }
    assess(ac, wind, ipDistance, end) {
      const V = this.spec.Vbg, LD = ac.glideRatio(V, 0), aim = this.aimFor(end);
      const wp = this.waypoints(ac, ipDistance, end);
      const route = wp.baseKey ? [[ac.x, ac.y], wp.baseKey, wp.ip, aim] : [[ac.x, ac.y], wp.ip, aim];
      const R = V * V / (G * Math.tan(25 * D2R));
      const gv = ac.groundVelocity(wind);
      let track = Math.atan2(gv.y, gv.x), required = 0;
      for (let i = 1; i < route.length; i++) {
        const [ax, ay] = route[i - 1], [bx, by] = route[i];
        const len = Math.hypot(bx - ax, by - ay);
        if (len < 1) continue;
        const brg = Math.atan2(by - ay, bx - ax), turn = Math.abs(wrapPi(brg - track));
        const along = wind.along(brg), groundLD = LD * Math.max(0.3, (V + along) / V);
        const t = Math.min(turn, 2.5);
        const turnAdj = i === 1 ? R * turn * 0.3 : R * t * 1.1 - 2 * R * Math.tan(t / 2);
        required += Math.max(0, len + turnAdj) / groundLD;
        track = brg;
      }
      const available = ac.h + Math.max(0, ac.V * ac.V - V * V) / (2 * G);
      const surplus = available - (required * this.spec.marginFactor + this.spec.marginHeight);
      return { reachable: surplus >= 0, required, available, route, ipDistance, end, surplus };
    }
    /** Ground glide gradient (height lost per metre over the ground) for a flap setting, along the landing direction. */
    gradient(ac, wind, flapPos, end) {
      const V = flapPos >= 20 ? this.spec.VappEO : this.spec.Vbg;
      return 1 / (ac.glideRatio(V, flapPos) * Math.max(0.3, (V + wind.along(end.heading)) / V));
    }
  };

  // src/autopilot.ts
  var ENGINE_OUT_PHASES = ["REJECTED", "EO_GLIDE", "EO_FINAL", "FORCED_LANDING"];
  var DEFAULT_GATES = {
    gateHeight: 300 * FT,
    speedLow: 5 * KT,
    speedHigh: 10 * KT,
    glidepathDeg: 0.7,
    lateral: 30,
    sinkRate: 1e3 * FT / 60,
    bank: 15 * D2R,
    persistence: 2
  };
  var StabilityMonitor = class {
    constructor(gates, runway, Vapp) {
      this.gates = gates;
      this.runway = runway;
      this.Vapp = Vapp;
      this.unstableFor = 0;
    }
    reset() {
      this.unstableFor = 0;
    }
    /** Returns a reason once a criterion has been violated for `persistence` seconds. */
    check(ac, dt) {
      const g = this.gates;
      if (ac.h > g.gateHeight) {
        this.unstableFor = 0;
        return null;
      }
      const dev = this.runway.glidepathDeviationDeg(ac.x, ac.h);
      let why = null;
      if (ac.V < this.Vapp - g.speedLow) why = "Slow on approach";
      else if (ac.V > this.Vapp + g.speedHigh) why = "Fast on approach";
      else if (ac.h > 60 * FT && Math.abs(dev) > g.glidepathDeg) why = dev > 0 ? "High on glidepath" : "Low on glidepath";
      else if (Math.abs(ac.y) > g.lateral) why = "Off centreline";
      else if (ac.verticalSpeed < -g.sinkRate) why = "Sink rate";
      else if (Math.abs(ac.phi) > g.bank) why = "Excessive bank";
      this.unstableFor = why ? this.unstableFor + dt : 0;
      return this.unstableFor > g.persistence ? why : null;
    }
  };
  var Autopilot = class {
    constructor(circuit, onEvent, gates = DEFAULT_GATES) {
      this.circuit = circuit;
      this.onEvent = onEvent;
      this.phase = "TAKEOFF_ROLL";
      this.fma = { thr: "TOGA", vert: "TO", lat: "RWY" };
      this.descending = false;
      this.goArounds = 0;
      this.goAroundReason = "";
      this.goAroundRequested = false;
      this.touchdownTime = 0;
      /** Set when the engine fails: what was decided, why, and where. */
      this.engineOut = null;
      this.glide = null;
      this.glideSpeed = new SpeedOnPitch(0.02, 0.01, -0.35, 0.1);
      this.eoFlaps = 0;
      this.passedBaseKey = false;
      this.slipSide = 0;
      this.lastAssess = 0;
      this.flareK = 0.28;
      this.guidance = new TrackGuidance();
      this.path = new FlightPathController();
      this.autoThrottle = new AutoThrottle();
      this.climb = new SpeedOnPitch();
      this.cmd = { theta: 0, phi: 0, beta: 0, thr: 1, flaps: 10, brake: 0, allowLiftoff: false, noTouchdown: false };
      this.gaStart = 0;
      this.gaGamma = 0;
      this.t = 0;
      this.stability = new StabilityMonitor(gates, circuit.runway, circuit.spec.Vapp);
      this.planner = new GlidePlanner(circuit.runway, DEFAULT_EO);
    }
    /** Landing direction chosen for a power-off return. */
    get eoEnd() {
      return this.glide?.end ?? this.planner.ends[0];
    }
    log(ac, phase, note) {
      this.onEvent({ t: this.t, phase, h: ac.h, V: ac.V, x: ac.x, y: ac.y, note });
    }
    setPhase(ac, p) {
      if (this.phase === p) return;
      if (p === "FLARE" && !ac.onGround) this.flareK = clamp((-ac.verticalSpeed - 0.4) / Math.max(ac.h, 0.5), 0.28, 2);
      this.phase = p;
      this.log(ac, p);
    }
    /**
     * Engine failure decision:
     * on the ground → reject; low after take-off → land ahead (no turn-back);
     * otherwise return to the runway if the glide planner says it is reachable with margin, else land ahead.
     */
    failEngine(ac, wind) {
      if (this.engineOut || this.phase === "STOPPED" || this.phase === "ROLLOUT") return false;
      ac.engineFailed = true;
      ac.shaper.rates = { ...ac.shaper.rates, theta: this.planner.spec.pitchRate };
      this.goAroundRequested = false;
      const E = this.planner.spec, track = ac.track(wind);
      this.glide = ac.onGround ? null : this.planner.plan(ac, wind);
      let plan, why;
      if (ac.onGround) {
        plan = "REJECT";
        why = "On the runway: reject take-off";
      } else if (this.phase === "FLARE") {
        plan = "RETURN";
        why = "Already in the flare";
      } else if ((this.phase === "CLIMB" || this.phase === "GO_AROUND") && ac.h < E.minTurnbackHeight) {
        plan = "LAND_AHEAD";
        why = "Below 700 ft: no turn-back";
      } else if (this.glide.reachable) {
        plan = "RETURN";
        why = `Runway ${this.glide.end.name} within glide range`;
      } else {
        plan = "LAND_AHEAD";
        why = "Both runway directions beyond glide range";
      }
      this.engineOut = { plan, why, t: this.t, x: ac.x, y: ac.y, h: ac.h, track };
      this.log(ac, "ENGINE_FAILURE", why);
      this.glideSpeed.engage(-1 / ac.glideRatio(E.Vbg, 0));
      this.eoFlaps = 0;
      this.lastAssess = this.t;
      this.passedBaseKey = false;
      if (plan === "REJECT") this.setPhase(ac, "REJECTED");
      else if (plan === "LAND_AHEAD") this.setPhase(ac, "FORCED_LANDING");
      else if (this.phase === "FLARE") {
      } else if (Math.abs(ac.y) < 100 && Math.abs(wrapPi(track - this.eoEnd.heading)) < 30 * D2R && this.planner.toAim(ac.x, this.eoEnd) > 0) {
        this.eoFlaps = Math.round(ac.flapPos / 10) * 10;
        this.setPhase(ac, "EO_FINAL");
      } else this.setPhase(ac, "EO_GLIDE");
      return true;
    }
    turnRadius(ac, wind) {
      const gv = ac.groundVelocity(wind), Vg = Math.hypot(gv.x, gv.y);
      return Vg * Vg / (G * Math.tan(this.circuit.spec.bankMax));
    }
    /** Vertical speed to follow a path whose height is `target` here: feed-forward along the track plus correction. */
    pathVS(ac, wind, target) {
      const gv = ac.groundVelocity(wind), Vg = Math.hypot(gv.x, gv.y);
      return clamp(-Vg * Math.tan(this.circuit.spec.approachAngle) + 0.2 * (target - ac.h), -5.5, 0.5);
    }
    glidePitch(ac, V, dt) {
      return this.path.gamma(ac, this.glideSpeed.update(ac, V, dt, -0.35), dt);
    }
    flareIfLow(ac) {
      if (ac.h < Math.max(this.circuit.spec.flareHeight, 3 * -ac.verticalSpeed)) this.setPhase(ac, "FLARE");
    }
    startGoAround(ac, why) {
      this.goArounds++;
      this.goAroundReason = why;
      this.goAroundRequested = false;
      this.stability.reset();
      this.circuit.crosswindX = null;
      this.descending = false;
      this.gaStart = this.t;
      this.gaGamma = ac.gamma;
      this.phase = "GO_AROUND";
      this.log(ac, "GO_AROUND", why);
    }
    legBank(ac, wind, leg, limit) {
      const [A, B] = this.circuit.legEnds(leg);
      const g = this.guidance.follow(ac, A, B);
      return { ...g, bank: this.guidance.bankFor(ac, wind, g.desiredTrack, limit) };
    }
    turnLead(ac, wind) {
      return this.guidance.turnLead(ac, wind, this.circuit.spec.bankMax, ac.shaper.rates.phi);
    }
    altHoldWithSpeed(ac, V, dt) {
      if (this.fma.vert !== "ALT") {
        this.fma.vert = "ALT";
        this.autoThrottle.engage(ac.thr);
      }
      this.cmd.theta = this.path.altitude(ac, this.circuit.spec.patternAlt, dt);
      this.cmd.thr = this.autoThrottle.update(ac, V, dt);
      this.fma.thr = "SPD";
    }
    /**
     * Crab → wing-low: rudder swings the nose onto the runway heading; the side force that creates
     * is cancelled by banking into wind (feed-forward), and the track loop trims what is left.
     */
    align(ac, wind, c, heading, line) {
      const lim = this.circuit.spec.alignBankMax;
      c.beta = clamp(wrapPi(heading - ac.psi), -ac.spec.maxSideslip, ac.spec.maxSideslip);
      const trackBank = line ? this.guidance.bankFor(ac, wind, this.guidance.follow(ac, line[0], line[1]).desiredTrack, 6 * D2R) : this.guidance.bankFor(ac, wind, heading, 6 * D2R);
      c.phi = clamp(ac.wingLowBank() + trackBank, -lim, lim);
      this.fma.lat = "ALIGN";
    }
    get landingAhead() {
      return this.engineOut?.plan === "LAND_AHEAD";
    }
    /** Centreline for a landing direction, from far out on the approach to the far end. */
    endLine(end) {
      return [[end.threshold - end.dir * 5e3, 0], [end.threshold + end.dir * this.circuit.runway.length, 0]];
    }
    lineBank(ac, wind, end, limit) {
      return this.guidance.bankFor(ac, wind, this.guidance.follow(ac, ...this.endLine(end)).desiredTrack, limit);
    }
    update(ac, wind, t, dt) {
      this.t = t;
      const c = this.cmd, S = this.circuit.spec, rwy = this.circuit.runway;
      c.allowLiftoff = false;
      c.noTouchdown = false;
      c.beta = 0;
      switch (this.phase) {
        case "TAKEOFF_ROLL": {
          c.thr = 1;
          c.flaps = 10;
          c.theta = 0;
          c.brake = 0;
          c.phi = 0;
          this.fma = { thr: "TOGA", vert: "TO", lat: "RWY" };
          if (ac.V >= S.Vr) this.setPhase(ac, "ROTATE");
          break;
        }
        case "ROTATE": {
          c.thr = 1;
          c.theta = 8 * D2R;
          c.phi = 0;
          c.allowLiftoff = true;
          c.noTouchdown = true;
          this.fma.vert = "ROT";
          if (!ac.onGround) {
            this.climb.engage(0.06);
            this.path.gammaPI.i = 0;
            this.setPhase(ac, "CLIMB");
          }
          break;
        }
        case "CLIMB":
        case "CROSSWIND": {
          c.noTouchdown = this.phase === "CLIMB";
          if (ac.h > S.flapsUpAlt) c.flaps = 0;
          const capture = ac.h > S.patternAlt - Math.max(15, ac.verticalSpeed * 6);
          if (this.fma.vert === "ALT" || capture) this.altHoldWithSpeed(ac, S.Vdownwind, dt);
          else {
            this.fma.vert = "PITCH-SPD";
            this.fma.thr = "TOGA";
            c.thr = 1;
            const gMin = ac.h < 30 ? 0.05 : -0.03;
            c.theta = this.path.gamma(ac, this.climb.update(ac, S.Vy, dt, gMin), dt);
          }
          if (this.phase === "CLIMB") {
            this.fma.lat = "TRK 090";
            c.phi = this.legBank(ac, wind, "UPWIND", ac.h < 60 ? 10 * D2R : S.bankMax).bank;
            if (ac.h >= S.crosswindTurnAlt) {
              const lead = this.turnLead(ac, wind);
              if (this.circuit.crosswindX === null) this.circuit.crosswindX = Math.max(ac.x + lead, rwy.length + S.crosswindClearance);
              if (ac.x >= this.circuit.crosswindX - lead) this.setPhase(ac, "CROSSWIND");
            }
          } else {
            this.fma.lat = "LEG XWND";
            const g = this.legBank(ac, wind, "CROSSWIND", S.bankMax);
            c.phi = g.bank;
            if (g.remaining < this.turnLead(ac, wind)) this.setPhase(ac, "DOWNWIND");
          }
          break;
        }
        case "DOWNWIND": {
          this.fma.lat = "LEG DNWD";
          const g = this.legBank(ac, wind, "DOWNWIND", S.bankMax);
          c.phi = g.bank;
          const profH = this.circuit.profileHeight("DOWNWIND", ac.x, ac.y, this.turnRadius(ac, wind));
          const startDescent = S.profileDescent ? profH < S.patternAlt - 3 : ac.x < 0;
          if (startDescent && !this.descending) {
            this.descending = true;
            this.log(ac, "DESCENT");
          }
          if (this.descending) {
            c.flaps = 10;
            this.fma.vert = S.profileDescent ? "PATH" : "V/S";
            this.fma.thr = "SPD";
            c.theta = S.profileDescent ? this.path.verticalSpeed(ac, this.pathVS(ac, wind, profH), dt) : this.path.verticalSpeed(ac, clamp(0.15 * (S.baseFloor - ac.h), S.descentVS, 0.5), dt);
            c.thr = this.autoThrottle.update(ac, S.Vdescent, dt);
          } else this.altHoldWithSpeed(ac, S.Vdownwind, dt);
          if (g.remaining < this.turnLead(ac, wind)) this.setPhase(ac, "BASE");
          break;
        }
        case "BASE": {
          this.fma.lat = "LEG BASE";
          this.fma.vert = "V/S";
          c.flaps = 20;
          const g = this.legBank(ac, wind, "BASE", S.bankMax);
          c.phi = g.bank;
          if (S.profileDescent) {
            this.fma.vert = "PATH";
            c.theta = this.path.verticalSpeed(ac, this.pathVS(ac, wind, this.circuit.profileHeight("BASE", ac.x, ac.y, this.turnRadius(ac, wind))), dt);
          } else {
            const floor = Math.min(rwy.glidepathHeight(ac.x), S.baseFloor);
            c.theta = this.path.verticalSpeed(ac, clamp(0.15 * (floor - ac.h), S.descentVS, 0.5), dt);
          }
          c.thr = this.autoThrottle.update(ac, S.Vbase, dt);
          if (this.goAroundRequested) {
            this.startGoAround(ac, "Pilot request");
            break;
          }
          if (g.remaining < this.turnLead(ac, wind)) this.setPhase(ac, "FINAL");
          break;
        }
        case "FINAL": {
          this.fma.lat = "LOC";
          this.fma.vert = "G/S";
          c.flaps = 30;
          if (ac.h < S.alignHeight) this.align(ac, wind, c, 0, this.circuit.legEnds("FINAL"));
          else c.phi = this.legBank(ac, wind, "FINAL", ac.h < 30 ? 10 * D2R : S.bankMax).bank;
          const gvx = ac.groundVelocity(wind).x;
          const vs = clamp(-gvx * Math.tan(rwy.glideslope) + 0.25 * (rwy.glidepathHeight(ac.x) - ac.h), -5.5, 1);
          c.theta = this.path.verticalSpeed(ac, vs, dt);
          c.thr = this.autoThrottle.update(ac, S.Vapp, dt);
          const why = this.goAroundRequested ? "Pilot request" : this.stability.check(ac, dt);
          if (why) {
            this.startGoAround(ac, why);
            break;
          }
          if (ac.h < Math.max(S.flareHeight, 3.3 * -ac.verticalSpeed)) this.setPhase(ac, "FLARE");
          break;
        }
        case "FLARE": {
          this.fma.vert = "FLARE";
          this.fma.thr = this.engineOut ? "ENG OUT" : "IDLE";
          c.thr = 0;
          if (this.landingAhead) this.align(ac, wind, c, this.engineOut.track);
          else if (this.engineOut) this.align(ac, wind, c, this.eoEnd.heading, this.endLine(this.eoEnd));
          else this.align(ac, wind, c, 0, this.circuit.legEnds("FINAL"));
          c.theta = this.path.verticalSpeed(ac, -(0.4 + this.flareK * ac.h), dt);
          if (ac.onGround) {
            this.touchdownTime = t;
            this.setPhase(ac, "ROLLOUT");
            break;
          }
          if (this.engineOut) break;
          if (this.goAroundRequested) this.startGoAround(ac, "Pilot request");
          else if (ac.x > rwy.length / 3 + 50) this.startGoAround(ac, "Long landing");
          break;
        }
        case "ROLLOUT": {
          this.fma = { thr: this.engineOut ? "ENG OUT" : "IDLE", vert: "ROLLOUT", lat: this.engineOut?.plan === "LAND_AHEAD" ? "FIELD" : "RWY" };
          c.thr = 0;
          c.theta = 0;
          c.phi = 0;
          c.flaps = 0;
          c.brake = t - this.touchdownTime > 2 ? 1 : 0;
          if (ac.Vg < 0.5) {
            this.setPhase(ac, "STOPPED");
            this.fma = { thr: "\u2014", vert: "\u2014", lat: "\u2014" };
          }
          break;
        }
        case "GO_AROUND": {
          this.fma = { thr: "TOGA", vert: "GA", lat: "TRK 090" };
          c.thr = 1;
          c.noTouchdown = true;
          this.gaGamma = Math.min(this.gaGamma + 1.5 * D2R * dt, 4 * D2R);
          const slow = S.Vapp - 3 * KT - ac.V;
          c.theta = this.path.gamma(ac, clamp(this.gaGamma - (slow > 0 ? 0.02 * slow : 0), -0.06, 0.12), dt);
          const vs = ac.verticalSpeed;
          if (c.flaps > 20) c.flaps = 20;
          if (c.flaps > 10 && vs > 1 && ac.V > 63 * KT && t - this.gaStart > 4) c.flaps = 10;
          c.phi = this.legBank(ac, wind, "UPWIND", 10 * D2R).bank;
          if (vs > 1.5 && ac.h > 150 * FT && ac.flapPos <= 10.5) {
            this.climb.engage(ac.gamma);
            this.fma.vert = "PITCH-SPD";
            this.setPhase(ac, "CLIMB");
          }
          break;
        }
        case "REJECTED": {
          this.fma = { thr: "ENG OUT", vert: "REJECT", lat: "RWY" };
          c.thr = 0;
          c.theta = 0;
          c.phi = 0;
          c.brake = 1;
          if (ac.Vg < 0.5) {
            this.setPhase(ac, "STOPPED");
            this.fma = { thr: "\u2014", vert: "\u2014", lat: "\u2014" };
          }
          break;
        }
        case "EO_GLIDE": {
          const E = this.planner.spec, end = this.eoEnd, wp = this.planner.waypoints(ac, this.glide.ipDistance, end);
          this.fma = { thr: "ENG OUT", vert: "GLIDE", lat: `DIR IP ${end.name}` };
          c.thr = 0;
          c.flaps = 0;
          c.theta = this.glidePitch(ac, E.Vbg, dt);
          const gv = ac.groundVelocity(wind), Vg = Math.hypot(gv.x, gv.y);
          const R = Vg * Vg / (G * Math.tan(S.bankMax)), rollIn = Vg * (S.bankMax / ac.shaper.rates.phi) * 0.5;
          if (wp.baseKey && !this.passedBaseKey) {
            const [bx, by] = wp.baseKey, brg = Math.atan2(by - ac.y, bx - ac.x), dist = Math.hypot(bx - ac.x, by - ac.y);
            const baseBrg = Math.atan2(wp.ip[1] - by, wp.ip[0] - bx);
            const lead = R * Math.tan(Math.min(Math.abs(wrapPi(baseBrg - brg)), 2.5) / 2) + rollIn;
            this.fma.lat = "DIR KEY";
            c.phi = this.guidance.bankFor(ac, wind, brg, S.bankMax);
            if (dist < lead || end.dir * (ac.x - bx) < 0) this.passedBaseKey = true;
          } else if (wp.baseKey) {
            const g = this.guidance.follow(ac, wp.baseKey, wp.ip);
            this.fma.lat = "BASE";
            c.phi = this.guidance.bankFor(ac, wind, g.desiredTrack, S.bankMax);
            if (g.remaining < R + rollIn) {
              this.setPhase(ac, "EO_FINAL");
              break;
            }
          } else {
            const [ipx, ipy] = wp.ip, brg = Math.atan2(ipy - ac.y, ipx - ac.x), dist = Math.hypot(ipx - ac.x, ipy - ac.y);
            const lead = R * Math.tan(Math.min(Math.abs(wrapPi(brg - end.heading)), 2.5) / 2) + rollIn;
            c.phi = this.guidance.bankFor(ac, wind, brg, S.bankMax);
            if (dist < lead || end.dir * (ac.x - ipx) > 0 && Math.abs(ac.y) < 150) {
              this.setPhase(ac, "EO_FINAL");
              break;
            }
          }
          if (t - this.lastAssess > 1) {
            this.lastAssess = t;
            const fresh = this.planner.plan(ac, wind, end), keep = this.planner.assess(ac, wind, this.glide.ipDistance, end);
            this.glide = fresh.reachable ? fresh : keep.available >= keep.required ? keep : fresh;
            if (this.glide.available < this.glide.required) {
              this.engineOut.plan = "LAND_AHEAD";
              this.engineOut.track = ac.track(wind);
              this.log(ac, "FORCED_LANDING", "Runway no longer reachable");
              this.phase = "FORCED_LANDING";
            }
          }
          break;
        }
        case "EO_FINAL": {
          const E = this.planner.spec, end = this.eoEnd, dist = this.planner.toAim(ac.x, end), greq = dist > 30 ? ac.h / dist : Infinity;
          const aligned = Math.abs(wrapPi(ac.track(wind) - end.heading)) < 15 * D2R && Math.abs(ac.y) < 80;
          const flapCap = aligned || ac.h < 300 * FT ? 30 : 10;
          while (this.eoFlaps < flapCap && greq > this.planner.gradient(ac, wind, this.eoFlaps + 10, end) * 1.05) this.eoFlaps += 10;
          c.thr = 0;
          c.flaps = this.eoFlaps;
          const V = this.eoFlaps >= 20 ? E.VappEO : E.Vbg;
          c.theta = this.glidePitch(ac, V, dt);
          this.fma = { thr: "ENG OUT", vert: `GLIDE F${this.eoFlaps}`, lat: `LOC ${end.name}` };
          const slip = aligned && this.eoFlaps === 30 && greq > this.planner.gradient(ac, wind, 30, end) * 1.1 && ac.h > S.alignHeight;
          if (ac.h < S.alignHeight) this.align(ac, wind, c, end.heading, this.endLine(end));
          else if (slip) {
            const rel = wrapPi(ac.psi - end.heading);
            if (!this.slipSide) this.slipSide = Math.abs(rel) < 1 * D2R ? 1 : Math.sign(rel);
            c.beta = -this.slipSide * 12 * D2R;
            c.phi = clamp(ac.wingLowBank() + this.lineBank(ac, wind, end, 6 * D2R), -12 * D2R, 12 * D2R);
            this.fma.lat = "SLIP";
          } else c.phi = this.lineBank(ac, wind, end, ac.h < 30 ? 10 * D2R : S.bankMax);
          this.flareIfLow(ac);
          break;
        }
        case "FORCED_LANDING": {
          const E = this.planner.spec, eo = this.engineOut;
          this.fma = { thr: "ENG OUT", vert: "GLIDE", lat: "TRK HOLD" };
          c.thr = 0;
          c.flaps = ac.h < 300 * FT ? 20 : 0;
          c.theta = this.glidePitch(ac, c.flaps >= 20 ? E.VappEO : E.Vbg, dt);
          if (ac.h < S.alignHeight) this.align(ac, wind, c, eo.track);
          else c.phi = this.guidance.bankFor(ac, wind, eo.track, 15 * D2R);
          this.flareIfLow(ac);
          break;
        }
        case "STOPPED":
          c.thr = 0;
          c.brake = 1;
          break;
      }
      return c;
    }
    /** Called by the simulation when the wheels touch outside the flare (e.g. a short approach). */
    onUnexpectedTouchdown(ac) {
      if (this.phase !== "FLARE") this.setPhase(ac, "FLARE");
    }
  };

  // src/airfield.ts
  var Runway = class {
    constructor(length = 1200, width = 30, aimPoint = 250, glideslope = 3 * D2R) {
      this.length = length;
      this.width = width;
      this.aimPoint = aimPoint;
      this.glideslope = glideslope;
    }
    get departureEnd() {
      return this.length;
    }
    /** Height of the glidepath above the runway at along-track position x. */
    glidepathHeight(x) {
      return Math.max(0, (this.aimPoint - x) * Math.tan(this.glideslope));
    }
    /** Angular deviation from the glidepath, degrees (positive = high). */
    glidepathDeviationDeg(x, h) {
      const d = Math.max(this.aimPoint - x, 1);
      return (Math.atan2(h, d) - this.glideslope) / D2R;
    }
    contains(x, y) {
      return x >= 0 && x <= this.length && Math.abs(y) <= this.width / 2;
    }
  };
  var STANDARD_CIRCUIT = {
    downwindOffset: 1200,
    baseX: -2600,
    patternAlt: 1e3 * FT,
    crosswindTurnAlt: 500 * FT,
    flapsUpAlt: 300 * FT,
    Vr: 55 * KT,
    Vy: 75 * KT,
    Vdownwind: 90 * KT,
    Vdescent: 75 * KT,
    Vbase: 70 * KT,
    Vapp: 65 * KT,
    bankMax: 25 * D2R,
    descentVS: -2.5,
    baseFloor: 150,
    flareHeight: 4.8,
    alignHeight: 60 * FT,
    alignBankMax: 10 * D2R,
    crosswindClearance: 400,
    approachAngle: 3 * D2R,
    profileDescent: false,
    name: "Standard"
  };
  var HIGH_CIRCUIT = {
    ...STANDARD_CIRCUIT,
    downwindOffset: 1e3,
    baseX: -1500,
    approachAngle: 5.5 * D2R,
    profileDescent: true,
    name: "High"
  };
  var Circuit = class {
    constructor(spec, runway) {
      this.spec = spec;
      this.runway = runway;
      /** Set once the aircraft decides where to turn crosswind; null until then. */
      this.crosswindX = null;
    }
    legEnds(leg, nominalCrossX = 2300) {
      const cx = this.crosswindX ?? nominalCrossX, s = this.spec, dy = s.downwindOffset;
      switch (leg) {
        case "UPWIND":
          return [[0, 0], [2e4, 0]];
        case "CROSSWIND":
          return [[cx, 0], [cx, dy]];
        case "DOWNWIND":
          return [[cx, dy], [s.baseX, dy]];
        case "BASE":
          return [[s.baseX, dy], [s.baseX, 0]];
        case "FINAL":
          return [[s.baseX, 0], [this.runway.length, 0]];
      }
    }
    /**
     * Distance still to fly along the circuit to the aim point, from a point on the downwind or base leg.
     * Used to place the aircraft on the approach profile before it reaches final.
     */
    remainingToAim(leg, x, y, turnRadius = 0) {
      const s = this.spec, final = this.runway.aimPoint - s.baseX;
      const cut = (2 - Math.PI / 2) * turnRadius;
      return leg === "DOWNWIND" ? Math.max(0, x - s.baseX) + s.downwindOffset + final - 2 * cut : Math.max(0, y) + final - (y > turnRadius ? cut : 0);
    }
    profileHeight(leg, x, y, turnRadius = 0) {
      return Math.max(0, this.remainingToAim(leg, x, y, turnRadius)) * Math.tan(this.spec.approachAngle);
    }
    /** Planned route for display, from brake release to the aim point. */
    plannedRoute() {
      const cx = this.crosswindX ?? 2300, s = this.spec;
      return [[0, 0], [cx, 0], [cx, s.downwindOffset], [s.baseX, s.downwindOffset], [s.baseX, 0], [this.runway.aimPoint, 0]];
    }
  };

  // src/aircraft.ts
  var C172_LIKE = {
    name: "Light single, C172-class",
    mass: 1e3,
    wingArea: 16.2,
    CLalpha: 5,
    CL0: 0.3,
    CLmax: 1.55,
    CD0: 0.032,
    inducedK: 0.057,
    power: 7e4,
    staticThrust: 2800,
    rollingMu: 0.03,
    brakeMu: 0.35,
    CYbeta: 0.35,
    maxSideslip: 15 * D2R,
    windmillCD: 0.012,
    maxPitch: 15 * D2R,
    minPitch: -10 * D2R,
    tailStrikePitch: 12 * D2R,
    flaps: {
      0: { dCL: 0, dCD: 0, dCLmax: 0 },
      10: { dCL: 0.2, dCD: 8e-3, dCLmax: 0.15 },
      20: { dCL: 0.35, dCD: 0.02, dCLmax: 0.3 },
      30: { dCL: 0.45, dCD: 0.035, dCLmax: 0.4 }
    }
  };
  var DEFAULT_SLEW = { theta: 3 * D2R, phi: 8 * D2R, thr: 0.3, flaps: 3, beta: 4 * D2R };
  var CommandShaper = class {
    constructor(rates = DEFAULT_SLEW) {
      this.rates = rates;
      this.theta = 0;
      this.phi = 0;
      this.thr = 0;
      this.flaps = 10;
      this.beta = 0;
    }
    shape(c, dt) {
      this.theta = slew(this.theta, c.theta, this.rates.theta, dt);
      this.phi = slew(this.phi, c.phi, this.rates.phi, dt);
      this.thr = slew(this.thr, c.thr, this.rates.thr, dt);
      this.flaps = slew(this.flaps, c.flaps, this.rates.flaps, dt);
      this.beta = slew(this.beta, c.beta, this.rates.beta, dt);
    }
  };
  var Aircraft = class {
    constructor(spec = C172_LIKE) {
      this.spec = spec;
      this.x = 0;
      this.y = 0;
      this.h = 0;
      this.V = 0;
      // true airspeed (sea level, so = IAS)
      this.Vg = 0;
      // ground speed while on the ground
      this.psi = 0;
      // air-path heading, rad (0 = east); equals the nose heading in coordinated flight
      this.beta = 0;
      // sideslip, rad: nose heading = psi + beta
      this.gamma = 0;
      // flight-path angle
      this.theta = 0;
      this.q = 0;
      this.phi = 0;
      this.p = 0;
      this.alphaF = 0;
      // filtered angle of attack, used as autopilot feed-forward
      this.thr = 0;
      this.flapPos = 10;
      this.brake = 0;
      this.onGround = true;
      this.stalled = false;
      this.engineFailed = false;
      /** Heading held on the ground after a landing away from the runway; null = steer to the runway centreline. */
      this.rolloutHeading = null;
      /** Runway direction steered to on the ground: 0 for 09, π for 27. */
      this.runwayHeading = 0;
      this.shaper = new CommandShaper();
    }
    get flaps() {
      return Math.round(this.flapPos);
    }
    /** Where the nose points (what a compass reads). */
    get noseHeading() {
      return wrapPi(this.psi + this.beta);
    }
    /** Ground track angle, rad. */
    track(wind) {
      const gv = this.groundVelocity(wind);
      return Math.atan2(gv.y, gv.x);
    }
    /** Side force from sideslip, N, positive toward the nose (CCW). */
    sideForce() {
      return 0.5 * RHO * this.V * this.V * this.spec.wingArea * this.spec.CYbeta * this.beta;
    }
    /** Bank that cancels the sideslip side force: the wing-low angle for a straight path. */
    wingLowBank() {
      if (this.onGround) return 0;
      const L = this.aero(this.V, this.theta - this.gamma).L;
      return Math.asin(clamp(-this.sideForce() / Math.max(L, 1), -0.3, 0.3));
    }
    get alpha() {
      return this.onGround ? this.theta : this.theta - this.gamma;
    }
    get verticalSpeed() {
      return this.onGround ? 0 : this.V * Math.sin(this.gamma);
    }
    groundVelocity(wind) {
      if (this.onGround) return { x: this.Vg * Math.cos(this.psi), y: this.Vg * Math.sin(this.psi) };
      const hv = this.V * Math.cos(this.gamma);
      return { x: hv * Math.cos(this.psi) + wind.v.x, y: hv * Math.sin(this.psi) + wind.v.y };
    }
    flapIncrement(pos = this.flapPos) {
      const f = this.spec.flaps, p = clamp(pos, 0, 30);
      const i = Math.min(2, Math.floor(p / 10)), lo = i * 10, hi = i * 10 + 10;
      const t = (p - lo) / 10, a = f[lo], b = f[hi];
      return { dCL: a.dCL + (b.dCL - a.dCL) * t, dCD: a.dCD + (b.dCD - a.dCD) * t, dCLmax: a.dCLmax + (b.dCLmax - a.dCLmax) * t };
    }
    aero(V, alpha) {
      const s = this.spec, f = this.flapIncrement();
      let CL = s.CL0 + f.dCL + s.CLalpha * alpha;
      const CLmax = s.CLmax + f.dCLmax;
      let stalled = false;
      if (CL > CLmax) {
        stalled = true;
        CL = CLmax - (CL - CLmax) * 1.5;
      }
      CL = Math.max(CL, -0.8);
      const CD = s.CD0 + f.dCD + s.inducedK * CL * CL + 0.3 * this.beta * this.beta + (this.engineFailed ? s.windmillCD : 0);
      const qS = 0.5 * RHO * V * V * s.wingArea;
      return { L: qS * CL, D: qS * CD, stalled };
    }
    thrust(V) {
      return this.engineFailed ? 0 : this.thr * Math.min(this.spec.staticThrust, this.spec.power / Math.max(V, 1));
    }
    /** Still-air glide ratio (L/D) in trimmed, power-off, coordinated flight at airspeed V. */
    glideRatio(V, flapPos) {
      const s = this.spec, f = this.flapIncrement(flapPos);
      const CL = s.mass * G / (0.5 * RHO * V * V * s.wingArea);
      return CL / (s.CD0 + f.dCD + s.inducedK * CL * CL + s.windmillCD);
    }
    step(dt, cmd, wind) {
      const sh = this.shaper;
      sh.shape(cmd, dt);
      const qCmd = clamp(1.5 * (sh.theta - this.theta), -5 * D2R, 5 * D2R);
      this.q += (qCmd - this.q) * dt / 0.3;
      this.theta += this.q * dt;
      const pCmd = clamp(2 * (sh.phi - this.phi), -15 * D2R, 15 * D2R);
      this.p += (pCmd - this.p) * dt / 0.2;
      this.phi += this.p * dt;
      this.thr = slew(this.thr, sh.thr, 0.8, dt);
      this.flapPos = sh.flaps;
      this.brake = cmd.brake;
      this.beta = this.onGround ? 0 : clamp(sh.beta, -this.spec.maxSideslip, this.spec.maxSideslip);
      return this.onGround ? this.stepGround(dt, cmd, wind) : this.stepAir(dt, cmd, wind);
    }
    stepGround(dt, cmd, wind) {
      const s = this.spec;
      this.phi = 0;
      this.p = 0;
      this.shaper.phi = 0;
      this.shaper.beta = 0;
      this.gamma = 0;
      this.theta = clamp(this.theta, 0, s.tailStrikePitch);
      if (this.theta === 0 && this.q < 0) this.q = 0;
      this.V = Math.max(0, this.Vg - wind.along(this.psi));
      const a = this.aero(this.V, this.theta), T = this.thrust(this.V);
      const W = s.mass * G, N = Math.max(0, W - a.L), mu = s.rollingMu + this.brake * s.brakeMu;
      const acc = (T - a.D - mu * N) / s.mass;
      this.Vg = Math.max(0, this.Vg + acc * dt);
      this.alphaF = this.theta;
      const psiCmd = this.rolloutHeading ?? this.runwayHeading + clamp(-0.03 * this.y * Math.cos(this.runwayHeading), -0.08, 0.08);
      this.psi = wrapPi(this.psi + wrapPi(psiCmd - this.psi) * dt / 0.6);
      this.x += this.Vg * Math.cos(this.psi) * dt;
      this.y += this.Vg * Math.sin(this.psi) * dt;
      if (cmd.allowLiftoff && a.L > W * 1.01) {
        this.onGround = false;
        this.h = 0.01;
        return { kind: "liftoff", V: this.V };
      }
      return null;
    }
    stepAir(dt, cmd, wind) {
      const s = this.spec, alpha = this.theta - this.gamma, a = this.aero(this.V, alpha), T = this.thrust(this.V);
      this.stalled = a.stalled;
      const Vs = Math.max(this.V, 5), mV = s.mass * Vs;
      const Vdot = (T * Math.cos(alpha) - a.D) / s.mass - G * Math.sin(this.gamma);
      const Lt = a.L + T * Math.sin(alpha);
      const gdot = Lt * Math.cos(this.phi) / mV - G * Math.cos(this.gamma) / Vs;
      const psidot = (Lt * Math.sin(this.phi) + this.sideForce()) / (mV * Math.cos(this.gamma));
      this.V += Vdot * dt;
      this.gamma += gdot * dt;
      this.psi = wrapPi(this.psi + psidot * dt);
      this.alphaF += (alpha - this.alphaF) * dt / 0.5;
      const gv = this.groundVelocity(wind);
      this.x += gv.x * dt;
      this.y += gv.y * dt;
      this.h += this.V * Math.sin(this.gamma) * dt;
      if (this.h > 0) return null;
      if (cmd.noTouchdown) {
        this.h = 0;
        this.gamma = Math.max(this.gamma, 0);
        return null;
      }
      const sink = -this.V * Math.sin(this.gamma);
      const crab = wrapPi(this.noseHeading - Math.atan2(gv.y, gv.x));
      this.psi = this.noseHeading;
      this.beta = 0;
      const near09 = Math.abs(wrapPi(this.psi)) < 30 * D2R, near27 = Math.abs(wrapPi(this.psi - Math.PI)) < 30 * D2R;
      const onRunwayLine = (near09 || near27) && Math.abs(this.y) < 60;
      this.runwayHeading = near27 ? Math.PI : 0;
      this.rolloutHeading = onRunwayLine ? null : this.psi;
      this.h = 0;
      this.onGround = true;
      this.Vg = this.V * Math.cos(this.gamma) + wind.along(this.psi);
      this.gamma = 0;
      return { kind: "touchdown", sink, crab, V: this.V, groundSpeed: this.Vg };
    }
  };

  // src/atmosphere.ts
  var Rng = class {
    constructor(seed) {
      this.seed = seed;
      if (this.seed <= 0) this.seed = 1;
    }
    next() {
      this.seed = this.seed * 16807 % 2147483647;
      return this.seed / 2147483647;
    }
    gauss() {
      return Math.sqrt(-2 * Math.log(this.next() + 1e-9)) * Math.cos(2 * Math.PI * this.next());
    }
  };
  var Wind = class {
    constructor(settings, rng) {
      this.rng = rng;
      this.gustTau = 3;
      // s, correlation time
      this.gustSigma = 1.8;
      // m/s, standard deviation
      this.gust = { x: 0, y: 0 };
      /** Current wind vector in m/s (the air's velocity over the ground). */
      this.v = { x: 0, y: 0 };
      this.settings = { ...settings };
    }
    update(dt) {
      const { fromDeg, speedKt, gusts } = this.settings;
      const toAngle = (90 - (fromDeg + 180)) * D2R, ws = speedKt * KT;
      if (gusts) {
        const k = this.gustSigma * Math.sqrt(2 * dt / this.gustTau);
        this.gust.x += -this.gust.x / this.gustTau * dt + k * this.rng.gauss();
        this.gust.y += -this.gust.y / this.gustTau * dt + k * this.rng.gauss();
      } else {
        this.gust.x *= 0.98;
        this.gust.y *= 0.98;
      }
      this.v.x = ws * Math.cos(toAngle) + this.gust.x;
      this.v.y = ws * Math.sin(toAngle) + this.gust.y;
    }
    /** Wind component along a track angle (positive = tailwind). */
    along(psi) {
      return this.v.x * Math.cos(psi) + this.v.y * Math.sin(psi);
    }
  };

  // src/simulation.ts
  var _Simulation = class _Simulation {
    constructor(opts) {
      // 50 Hz
      this.t = 0;
      this.track = [];
      this.events = [];
      this.touchdown = null;
      this.maxBank = 0;
      this.maxAlt = 0;
      this.nextRecord = 0;
      /** Is the runway within power-off gliding reach right now? null on the ground or once the engine has failed. */
      this.runwayReachable = null;
      /** Which runway direction the glide planner would use right now. */
      this.reachEnd = null;
      /** Seconds airborne with the engine running, and how many of those had the runway within glide reach. */
      this.airTime = 0;
      this.reachTime = 0;
      this.nextReachCheck = 0;
      this.aircraft = new Aircraft(opts.aircraft ?? C172_LIKE);
      this.wind = new Wind(opts.wind, new Rng(opts.seed ?? 1));
      const spec = opts.circuit ?? STANDARD_CIRCUIT;
      this.runway = opts.runway ?? new Runway(1200, 30, 250, spec.approachAngle);
      this.circuit = new Circuit(spec, this.runway);
      this.autopilot = new Autopilot(this.circuit, (e) => this.events.push(e));
      this.events.push({ t: 0, phase: "TAKEOFF_ROLL", h: 0, V: 0, x: 0, y: 0 });
    }
    get phase() {
      return this.autopilot.phase;
    }
    get finished() {
      return this.phase === "STOPPED";
    }
    requestGoAround() {
      const ok = this.phase === "BASE" || this.phase === "FINAL" || this.phase === "FLARE";
      if (ok) this.autopilot.goAroundRequested = true;
      return ok;
    }
    /** Stop the engine now; the autopilot decides how to handle it. */
    failEngine() {
      return this.autopilot.failEngine(this.aircraft, this.wind);
    }
    get engineFailed() {
      return this.aircraft.engineFailed;
    }
    step(dt = _Simulation.DT) {
      const ac = this.aircraft;
      if (this.finished) {
        ac.Vg = 0;
        ac.V = 0;
        this.t += dt;
        return;
      }
      this.wind.update(dt);
      const cmd = this.autopilot.update(ac, this.wind, this.t, dt);
      const contact = ac.step(dt, cmd, this.wind);
      if (contact?.kind === "touchdown") {
        this.touchdown = { t: this.t, x: ac.x, y: ac.y, sink: contact.sink, V: contact.V, crab: contact.crab, onRunway: this.runway.contains(ac.x, ac.y) };
        this.autopilot.onUnexpectedTouchdown(ac);
      }
      if (!ac.onGround) {
        this.maxBank = Math.max(this.maxBank, Math.abs(ac.phi));
        this.maxAlt = Math.max(this.maxAlt, ac.h);
      }
      this.updateReach(dt);
      this.t += dt;
      if (this.t >= this.nextRecord) {
        this.track.push({ t: this.t, x: ac.x, y: ac.y, h: ac.h, V: ac.V, phase: this.phase, reach: this.runwayReachable });
        this.nextRecord = this.t + 0.25;
      }
    }
    /**
     * Glide-reach bookkeeping: twice a second, would a power-off return reach the runway from here?
     * Climbing below the turn-back height counts as no (the policy is to land ahead there).
     */
    updateReach(dt) {
      const ac = this.aircraft, ap = this.autopilot;
      const tracking = !ac.onGround && !ac.engineFailed && this.phase !== "FLARE";
      if (!tracking) {
        this.runwayReachable = null;
        return;
      }
      if (this.t >= this.nextReachCheck) {
        this.nextReachCheck = this.t + 0.5;
        const climbingLow = (this.phase === "CLIMB" || this.phase === "GO_AROUND") && ac.h < ap.planner.spec.minTurnbackHeight;
        const g = ap.planner.plan(ac, this.wind);
        this.runwayReachable = !climbingLow && g.reachable;
        this.reachEnd = this.runwayReachable ? g.end.name : null;
      }
      this.airTime += dt;
      if (this.runwayReachable) this.reachTime += dt;
    }
    get reachFraction() {
      return this.airTime > 0 ? this.reachTime / this.airTime : 0;
    }
    /** Landing pass/fail check against the proof-of-concept criteria. */
    evaluate() {
      if (this.autopilot.engineOut) return this.evaluateEngineOut();
      const td = this.touchdown, third = this.runway.length / 3, stopped = this.finished;
      const rows = [
        { label: "Touchdown sink rate", target: "< 3 ft/s" },
        { label: "Touchdown point", target: `first third (0\u2013${Math.round(third)} m)` },
        { label: "Centerline error", target: "< 5 m" },
        { label: "Nose aligned at touchdown", target: "< 5\xB0 crab" },
        { label: "Stopped on runway", target: `before ${this.runway.length} m` }
      ];
      if (!td) return rows;
      const sinkFps = td.sink / FT;
      rows[0] = { ...rows[0], value: sinkFps.toFixed(1) + " ft/s", pass: sinkFps < 3 };
      rows[1] = { ...rows[1], value: Math.round(td.x) + " m past threshold", pass: td.x >= 0 && td.x <= third };
      rows[2] = { ...rows[2], value: Math.abs(td.y).toFixed(1) + " m", pass: Math.abs(td.y) < 5 };
      const crabDeg = Math.abs(td.crab) / D2R;
      rows[3] = { ...rows[3], value: crabDeg.toFixed(1) + "\xB0 crab", pass: crabDeg < 5 };
      rows[4] = { ...rows[4], value: stopped ? Math.round(this.aircraft.x) + " m mark" : "rolling\u2026", pass: stopped ? this.aircraft.x <= this.runway.length : null };
      rows.push({ label: "Runway within glide reach", target: `${this.circuit.spec.name} circuit, share of time airborne`, value: Math.round(this.reachFraction * 100) + "%", info: true });
      return rows;
    }
    /** Engine-out outcome: was the decision carried out, and was the touchdown survivable and controlled? */
    evaluateEngineOut() {
      const eo = this.autopilot.engineOut, td = this.touchdown, ac = this.aircraft, stopped = this.finished;
      const end = this.autopilot.eoEnd;
      const decided = eo.plan === "REJECT" ? "Reject take-off" : eo.plan === "RETURN" ? `Glide back to runway ${end.name}` : "Land ahead";
      const rows = [{ label: "Decision", target: eo.why, value: `${decided} \xB7 failed at ${Math.round(eo.h / FT)} ft`, info: true }];
      if (eo.plan === "REJECT") {
        rows.push({
          label: "Stopped on runway",
          target: `before ${this.runway.length} m`,
          value: stopped ? Math.round(ac.x) + " m mark" : "braking\u2026",
          pass: stopped ? ac.x <= this.runway.length : null
        });
        return rows;
      }
      const sinkFps = td ? td.sink / FT : 0, crabDeg = td ? Math.abs(td.crab) / D2R : 0;
      rows.push({ label: "Touchdown sink rate", target: "< 3 ft/s", value: td ? sinkFps.toFixed(1) + " ft/s" : void 0, pass: td ? sinkFps < 3 : null });
      if (eo.plan === "RETURN") {
        rows.push({
          label: "Touched down on runway",
          target: "within the paved surface",
          value: td ? (() => {
            const past = end.dir * (td.x - end.threshold);
            return td.onRunway ? `${Math.round(past)} m past the ${end.name} threshold` : past < 0 ? Math.round(-past) + " m short" : "off the runway";
          })() : void 0,
          pass: td ? td.onRunway : null
        });
        rows.push({
          label: "Stopped on runway",
          target: `within the ${this.runway.length} m runway`,
          value: stopped ? Math.round(end.dir * (ac.x - end.threshold)) + " m from the " + end.name + " threshold" : td ? "rolling\u2026" : void 0,
          pass: stopped ? this.runway.contains(ac.x, ac.y) : null
        });
      } else {
        rows.push({ label: "Wings level, nose straight", target: "< 5\xB0 crab", value: td ? crabDeg.toFixed(1) + "\xB0 crab" : void 0, pass: td ? crabDeg < 5 : null });
        const dist = td ? Math.hypot(td.x - eo.x, td.y - eo.y) : 0;
        rows.push({ label: "Field landing", target: "touchdown point", value: td ? `${(dist / 1e3).toFixed(1)} km from failure, ${Math.round(td.V / KT)} kt` : void 0, info: true });
      }
      return rows;
    }
  };
  _Simulation.DT = 0.02;
  var Simulation = _Simulation;

  // src/ui/theme.ts
  var PHASE_SEQUENCE = [
    { phase: "TAKEOFF_ROLL", name: "Takeoff roll", token: "--ph-ground" },
    { phase: "ROTATE", name: "Rotate", token: "--ph-ground" },
    { phase: "CLIMB", name: "Climb", token: "--ph-climb" },
    { phase: "CROSSWIND", name: "Crosswind", token: "--ph-xwind" },
    { phase: "DOWNWIND", name: "Downwind", token: "--ph-down" },
    { phase: "BASE", name: "Base", token: "--ph-base" },
    { phase: "FINAL", name: "Final", token: "--ph-final" },
    { phase: "FLARE", name: "Flare", token: "--ph-flare" },
    { phase: "ROLLOUT", name: "Rollout", token: "--ph-ground" },
    { phase: "STOPPED", name: "Stopped", token: "--ph-ground" }
  ];
  var GO_AROUND_INFO = { phase: "GO_AROUND", name: "Go-around", token: "--ph-ga" };
  var ENGINE_OUT_INFO = [
    { phase: "REJECTED", name: "Rejected take-off", token: "--ph-eo", index: 1 },
    { phase: "EO_GLIDE", name: "Glide to runway", token: "--ph-eo", index: 5 },
    { phase: "EO_FINAL", name: "Power-off final", token: "--ph-eo", index: 6 },
    { phase: "FORCED_LANDING", name: "Forced landing", token: "--ph-eo", index: 6 }
  ];
  var PHASES = Object.fromEntries([
    ...[...PHASE_SEQUENCE, GO_AROUND_INFO].map((p, i) => [p.phase, { ...p, index: p.phase === "GO_AROUND" ? 2 : i }]),
    ...ENGINE_OUT_INFO.map((p) => [p.phase, p])
  ]);
  var TOKENS = [
    "--paper",
    "--panel",
    "--ink",
    "--muted",
    "--line",
    "--route",
    "--runway",
    "--rwmark",
    "--ok",
    "--bad",
    "--f-display",
    "--f-mono",
    "--ph-eo",
    ...new Set([...PHASE_SEQUENCE, GO_AROUND_INFO].map((p) => p.token))
  ];
  var Palette = class {
    constructor() {
      this.values = {};
      this.refresh();
      window.matchMedia?.("(prefers-color-scheme: dark)").addEventListener?.("change", () => this.refresh());
      new MutationObserver(() => this.refresh()).observe(document.documentElement, { attributes: true, attributeFilter: ["data-theme"] });
      document.fonts?.ready.then(() => this.refresh());
    }
    refresh() {
      const cs = getComputedStyle(document.documentElement);
      for (const t of TOKENS) this.values[t] = cs.getPropertyValue(t).trim();
    }
    get(token) {
      return this.values[token] ?? "#888";
    }
    phase(p) {
      return this.get(PHASES[p].token);
    }
  };
  function fitCanvas(cv) {
    const dpr = Math.min(window.devicePixelRatio || 1, 2), r = cv.getBoundingClientRect();
    const w = Math.max(10, Math.round(r.width)), h = Math.max(10, Math.round(r.height));
    if (cv.width !== w * dpr || cv.height !== h * dpr) {
      cv.width = w * dpr;
      cv.height = h * dpr;
    }
    const ctx = cv.getContext("2d");
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    return { ctx, w, h };
  }

  // src/ui/views.ts
  var PlanView = class {
    constructor(cv, pal) {
      this.cv = cv;
      this.pal = pal;
    }
    draw(sim) {
      const { ctx, w, h } = fitCanvas(this.cv), P = this.pal, rwy = sim.runway, spec = sim.circuit.spec;
      ctx.clearRect(0, 0, w, h);
      const route = sim.circuit.plannedRoute(), cx = route[1][0];
      let minX = spec.baseX - 500, maxX = Math.max(cx, rwy.length) + 600, minY = -500, maxY = spec.downwindOffset + 450;
      for (const p of sim.track) {
        minX = Math.min(minX, p.x - 200);
        maxX = Math.max(maxX, p.x + 200);
        minY = Math.min(minY, p.y - 200);
        maxY = Math.max(maxY, p.y + 200);
      }
      const sc = Math.min(w / (maxX - minX), h / (maxY - minY));
      const ox = (w - (maxX - minX) * sc) / 2 - minX * sc, oy = h - ((h - (maxY - minY) * sc) / 2 - minY * sc);
      const X = (x) => ox + x * sc, Y = (y) => oy - y * sc;
      const fDisp = P.get("--f-display"), fMono = P.get("--f-mono");
      ctx.lineWidth = 1;
      ctx.strokeStyle = P.get("--line");
      ctx.globalAlpha = 0.55;
      ctx.beginPath();
      for (let gx = Math.ceil(minX / 500) * 500; gx <= maxX; gx += 500) {
        ctx.moveTo(X(gx), 0);
        ctx.lineTo(X(gx), h);
      }
      for (let gy = Math.ceil(minY / 500) * 500; gy <= maxY; gy += 500) {
        ctx.moveTo(0, Y(gy));
        ctx.lineTo(w, Y(gy));
      }
      ctx.stroke();
      ctx.globalAlpha = 1;
      ctx.strokeStyle = P.get("--route");
      ctx.lineWidth = 1.5;
      ctx.setLineDash([6, 5]);
      ctx.beginPath();
      route.forEach((p, i) => i ? ctx.lineTo(X(p[0]), Y(p[1])) : ctx.moveTo(X(p[0]), Y(p[1])));
      ctx.stroke();
      ctx.setLineDash([]);
      ctx.fillStyle = P.get("--route");
      ctx.textAlign = "center";
      ctx.font = `600 ${w < 500 ? 10 : 12}px ${fDisp}`;
      const label = (t, x, y, rot = 0) => {
        ctx.save();
        ctx.translate(X(x), Y(y));
        ctx.rotate(rot);
        ctx.fillText(t, 0, 0);
        ctx.restore();
      };
      label("DOWNWIND \xB7 1000 FT", (cx + spec.baseX) / 2, spec.downwindOffset + 70);
      label("CROSSWIND", cx + 80, spec.downwindOffset / 2, Math.PI / 2);
      label("BASE", spec.baseX - 80, spec.downwindOffset / 2, -Math.PI / 2);
      label("FINAL", spec.baseX / 2, -110);
      label("UPWIND", (rwy.length + cx) / 2, -110);
      const rw = Math.max(rwy.width * sc, 7);
      ctx.fillStyle = P.get("--runway");
      ctx.fillRect(X(0), Y(0) - rw / 2, rwy.length * sc, rw);
      ctx.strokeStyle = P.get("--rwmark");
      ctx.lineWidth = 1;
      ctx.setLineDash([4, 4]);
      ctx.beginPath();
      ctx.moveTo(X(0) + 6, Y(0));
      ctx.lineTo(X(rwy.length) - 6, Y(0));
      ctx.stroke();
      ctx.setLineDash([]);
      ctx.fillStyle = P.get("--rwmark");
      ctx.fillRect(X(rwy.aimPoint) - 1, Y(0) - rw / 2 + 1, 3, rw - 2);
      ctx.fillStyle = P.get("--ink");
      ctx.font = `700 12px ${fDisp}`;
      ctx.textAlign = "right";
      ctx.fillText("09", X(0) - 5, Y(0) + 4);
      ctx.textAlign = "left";
      ctx.fillText("27", X(rwy.length) + 5, Y(0) + 4);
      ctx.lineWidth = 2.5;
      ctx.lineCap = "round";
      for (let i = 1; i < sim.track.length; i++) {
        const a = sim.track[i - 1], b = sim.track[i];
        ctx.strokeStyle = P.phase(b.phase);
        ctx.beginPath();
        ctx.moveTo(X(a.x), Y(a.y));
        ctx.lineTo(X(b.x), Y(b.y));
        ctx.stroke();
      }
      ctx.font = `700 11px ${fDisp}`;
      ctx.textAlign = "center";
      for (const e of sim.events) if (e.phase === "GO_AROUND") {
        ctx.fillStyle = P.get("--ph-ga");
        ctx.beginPath();
        ctx.arc(X(e.x), Y(e.y), 4.5, 0, 7);
        ctx.fill();
        ctx.fillText("GA", X(e.x), Y(e.y) - 9);
      }
      this.drawEngineOut(sim, ctx, X, Y, sc);
      const td = sim.touchdown;
      if (td) {
        ctx.strokeStyle = P.get("--ink");
        ctx.lineWidth = 1.5;
        ctx.beginPath();
        ctx.arc(X(td.x), Y(td.y), 6, 0, 7);
        ctx.stroke();
      }
      const ac = sim.aircraft, k = 1.1;
      ctx.save();
      ctx.translate(X(ac.x), Y(ac.y));
      ctx.rotate(-ac.noseHeading);
      ctx.beginPath();
      const pts = [
        [10, 0],
        [4, 1.6],
        [2, 11],
        [-1, 11],
        [-1, 1.6],
        [-7, 1.4],
        [-9, 4.5],
        [-10.5, 4.5],
        [-10, 0],
        [-10.5, -4.5],
        [-9, -4.5],
        [-7, -1.4],
        [-1, -1.6],
        [-1, -11],
        [2, -11],
        [4, -1.6]
      ];
      pts.forEach(([px, py], i) => i ? ctx.lineTo(px * k, py * k) : ctx.moveTo(px * k, py * k));
      ctx.closePath();
      ctx.fillStyle = P.get("--ink");
      ctx.strokeStyle = P.get("--paper");
      ctx.lineWidth = 2;
      ctx.stroke();
      ctx.fill();
      ctx.restore();
      const ws = sim.wind.settings, wx = w - 44, wy = 44, toA = (90 - (ws.fromDeg + 180)) * D2R;
      ctx.strokeStyle = P.get("--ink");
      ctx.fillStyle = P.get("--ink");
      ctx.lineWidth = 1;
      ctx.globalAlpha = 0.25;
      ctx.beginPath();
      ctx.arc(wx, wy, 22, 0, 7);
      ctx.stroke();
      ctx.globalAlpha = 1;
      if (ws.speedKt > 0) {
        ctx.save();
        ctx.translate(wx, wy);
        ctx.rotate(-toA);
        ctx.lineWidth = 2;
        ctx.beginPath();
        ctx.moveTo(-18, 0);
        ctx.lineTo(14, 0);
        ctx.stroke();
        ctx.beginPath();
        ctx.moveTo(18, 0);
        ctx.lineTo(10, -5);
        ctx.lineTo(10, 5);
        ctx.closePath();
        ctx.fill();
        ctx.restore();
      }
      ctx.font = `500 11px ${fMono}`;
      ctx.textAlign = "center";
      ctx.fillText(ws.speedKt ? `${String(ws.fromDeg).padStart(3, "0")}\xB0/${ws.speedKt}kt${ws.gusts ? " G" : ""}` : "CALM", wx, wy + 38);
      ctx.font = `700 11px ${fDisp}`;
      ctx.fillText("N \u2191", wx, wy - 28);
      const bar = 1e3 * sc, bx = 14, by = h - 16;
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.moveTo(bx, by - 4);
      ctx.lineTo(bx, by);
      ctx.lineTo(bx + bar, by);
      ctx.lineTo(bx + bar, by - 4);
      ctx.stroke();
      ctx.textAlign = "left";
      ctx.font = `500 11px ${fMono}`;
      ctx.fillText("1 km", bx + bar + 6, by + 3);
    }
    /** Failure point, the planned power-off route (base key, intercept) or the field ahead, and a still-air reach ring. */
    drawEngineOut(sim, ctx, X, Y, sc) {
      const eo = sim.autopilot.engineOut;
      if (!eo) return;
      const P = this.pal, col = P.get("--ph-eo"), ac = sim.aircraft, fDisp = P.get("--f-display");
      ctx.save();
      ctx.strokeStyle = col;
      ctx.fillStyle = col;
      ctx.lineWidth = 2;
      const fx = X(eo.x), fy = Y(eo.y);
      ctx.beginPath();
      ctx.moveTo(fx - 5, fy - 5);
      ctx.lineTo(fx + 5, fy + 5);
      ctx.moveTo(fx + 5, fy - 5);
      ctx.lineTo(fx - 5, fy + 5);
      ctx.stroke();
      ctx.font = `700 11px ${fDisp}`;
      ctx.textAlign = "center";
      ctx.fillText("ENG", fx, fy - 9);
      if (!ac.onGround && sim.phase !== "FLARE") {
        ctx.globalAlpha = 0.5;
        ctx.setLineDash([2, 4]);
        ctx.lineWidth = 1.2;
        ctx.beginPath();
        ctx.arc(X(ac.x), Y(ac.y), sim.autopilot.planner.stillAirReach(ac) * sc, 0, 7);
        ctx.stroke();
        ctx.globalAlpha = 1;
      }
      const g = sim.autopilot.glide;
      if (eo.plan === "RETURN" && g && (sim.phase === "EO_GLIDE" || sim.phase === "EO_FINAL")) {
        ctx.setLineDash([7, 4]);
        ctx.lineWidth = 1.8;
        ctx.beginPath();
        const pts = sim.phase === "EO_FINAL" ? [[ac.x, ac.y], sim.autopilot.planner.aimFor(sim.autopilot.eoEnd)] : g.route;
        pts.forEach((p, i) => i ? ctx.lineTo(X(p[0]), Y(p[1])) : ctx.moveTo(X(p[0]), Y(p[1])));
        ctx.stroke();
        ctx.setLineDash([]);
        ctx.font = `700 10px ${fDisp}`;
        if (sim.phase === "EO_GLIDE") g.route.slice(1, -1).forEach((p, i, a) => {
          ctx.beginPath();
          ctx.arc(X(p[0]), Y(p[1]), 3.5, 0, 7);
          ctx.fill();
          ctx.fillText(i === a.length - 1 ? "IP" : "KEY", X(p[0]), Y(p[1]) + 15);
        });
      } else if (eo.plan === "LAND_AHEAD" && !sim.touchdown) {
        ctx.setLineDash([7, 4]);
        ctx.lineWidth = 1.8;
        ctx.beginPath();
        const len = Math.max(400, ac.h * ac.glideRatio(sim.autopilot.planner.spec.Vbg, 0));
        ctx.moveTo(X(ac.x), Y(ac.y));
        ctx.lineTo(X(ac.x + Math.cos(eo.track) * len), Y(ac.y + Math.sin(eo.track) * len));
        ctx.stroke();
      }
      ctx.restore();
    }
  };
  var ProfileView = class {
    constructor(cv, pal) {
      this.cv = cv;
      this.pal = pal;
    }
    draw(sim) {
      const { ctx, w, h } = fitCanvas(this.cv), P = this.pal;
      ctx.clearRect(0, 0, w, h);
      const L = 46, R = 12, T0 = 12, gap = 26, ph = (h - T0 - 28 - gap) / 2;
      const Tmax = Math.max(360, Math.ceil((sim.t + 20) / 60) * 60);
      const X = (t) => L + t / Tmax * (w - L - R);
      const fMono = P.get("--f-mono"), fDisp = P.get("--f-display");
      const panels = [
        { top: T0, max: 1200, ticks: [0, 400, 800, 1200], val: (p) => p.h / FT, label: "ALT (ft)", color: P.get("--ink") },
        { top: T0 + ph + gap, max: 100, ticks: [0, 25, 50, 75, 100], val: (p) => p.V / KT, label: "IAS (kt)", color: P.get("--route") }
      ];
      const ev = sim.events.filter((e) => e.phase !== "DESCENT" && e.phase !== "ENGINE_FAILURE");
      ev.forEach((e, i) => {
        const t1 = i + 1 < ev.length ? ev[i + 1].t : sim.t;
        ctx.fillStyle = P.phase(e.phase);
        ctx.globalAlpha = e.phase === "GO_AROUND" || P.get(PHASES[e.phase].token) === P.get("--ph-eo") ? 0.22 : 0.1;
        for (const p of panels) ctx.fillRect(X(e.t), p.top, X(t1) - X(e.t), ph);
        ctx.globalAlpha = 1;
        ctx.fillRect(X(e.t), panels[0].top - 6, Math.max(1, X(t1) - X(e.t)), 3);
      });
      for (const p of panels) {
        const Y = (v) => p.top + ph - v / p.max * ph;
        ctx.strokeStyle = P.get("--line");
        ctx.lineWidth = 1;
        ctx.fillStyle = P.get("--muted");
        ctx.textAlign = "right";
        ctx.font = `500 10.5px ${fMono}`;
        for (const t of p.ticks) {
          ctx.beginPath();
          ctx.moveTo(L, Y(t) + 0.5);
          ctx.lineTo(w - R, Y(t) + 0.5);
          ctx.stroke();
          ctx.fillText(String(t), L - 6, Y(t) + 3.5);
        }
        ctx.textAlign = "left";
        ctx.fillStyle = p.color;
        ctx.font = `600 11px ${fDisp}`;
        ctx.fillText(p.label, L + 4, p.top + 11);
        ctx.strokeStyle = p.color;
        ctx.lineWidth = 2;
        ctx.beginPath();
        sim.track.forEach((q, i) => {
          const x = X(q.t), y = Y(Math.min(p.max * 1.02, p.val(q)));
          i ? ctx.lineTo(x, y) : ctx.moveTo(x, y);
        });
        ctx.stroke();
        const last = sim.track[sim.track.length - 1];
        if (last) {
          ctx.fillStyle = p.color;
          ctx.beginPath();
          ctx.arc(X(last.t), Y(Math.min(p.max, p.val(last))), 3.5, 0, 7);
          ctx.fill();
        }
      }
      const sy = panels[0].top + ph + 5;
      for (let i = 1; i < sim.track.length; i++) {
        const q = sim.track[i];
        if (q.reach === null) continue;
        ctx.fillStyle = q.reach ? P.get("--ok") : P.get("--bad");
        ctx.globalAlpha = q.reach ? 0.85 : 0.45;
        ctx.fillRect(X(sim.track[i - 1].t), sy, X(q.t) - X(sim.track[i - 1].t) + 0.5, 4);
      }
      ctx.globalAlpha = 1;
      ctx.textAlign = "right";
      ctx.fillStyle = P.get("--muted");
      ctx.font = `600 9.5px ${fDisp}`;
      ctx.fillText("GLIDE REACH", w - R, sy + 14);
      ctx.fillStyle = P.get("--muted");
      ctx.textAlign = "center";
      ctx.font = `500 10.5px ${fMono}`;
      const step = Tmax > 720 ? 120 : 60;
      for (let t = 0; t <= Tmax; t += step) ctx.fillText(`${t / 60}:00`, X(t), h - 8);
    }
  };
  var AttitudeIndicator = class {
    constructor(cv, pal) {
      this.cv = cv;
      this.pal = pal;
    }
    draw(sim) {
      const cv = this.cv, dpr = Math.min(window.devicePixelRatio || 1, 2), size = 150;
      if (cv.width !== size * dpr) {
        cv.width = size * dpr;
        cv.height = size * dpr;
      }
      const ctx = cv.getContext("2d");
      ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
      const r = size / 2, ppd = 3.2, ac = sim.aircraft, fMono = this.pal.get("--f-mono");
      ctx.save();
      ctx.beginPath();
      ctx.arc(r, r, r, 0, 7);
      ctx.clip();
      ctx.translate(r, r);
      ctx.rotate(-ac.phi);
      ctx.translate(0, ac.theta / D2R * ppd);
      ctx.fillStyle = "#2c6aa6";
      ctx.fillRect(-200, -400, 400, 400);
      ctx.fillStyle = "#7a4f28";
      ctx.fillRect(-200, 0, 400, 400);
      ctx.strokeStyle = "#ffffff";
      ctx.fillStyle = "#ffffff";
      ctx.lineWidth = 1.5;
      ctx.beginPath();
      ctx.moveTo(-200, 0);
      ctx.lineTo(200, 0);
      ctx.stroke();
      ctx.font = `500 9px ${fMono}`;
      ctx.lineWidth = 1;
      ctx.textAlign = "left";
      for (let d = -20; d <= 20; d += 5) {
        if (!d) continue;
        const y = -d * ppd, half = d % 10 ? 9 : 18;
        ctx.beginPath();
        ctx.moveTo(-half, y);
        ctx.lineTo(half, y);
        ctx.stroke();
        if (!(d % 10)) ctx.fillText(String(Math.abs(d)), half + 3, y + 3);
      }
      ctx.restore();
      ctx.save();
      ctx.translate(r, r);
      ctx.strokeStyle = "#ffffff";
      ctx.lineWidth = 1.5;
      for (const b of [-60, -45, -30, -20, -10, 0, 10, 20, 30, 45, 60]) {
        const a = (b - 90) * D2R, l = b % 30 ? 6 : 10;
        ctx.beginPath();
        ctx.moveTo(Math.cos(a) * (r - 2), Math.sin(a) * (r - 2));
        ctx.lineTo(Math.cos(a) * (r - 2 - l), Math.sin(a) * (r - 2 - l));
        ctx.stroke();
      }
      ctx.rotate(-ac.phi);
      ctx.fillStyle = "#ffffff";
      ctx.beginPath();
      ctx.moveTo(0, -r + 14);
      ctx.lineTo(-5, -r + 22);
      ctx.lineTo(5, -r + 22);
      ctx.closePath();
      ctx.fill();
      ctx.restore();
      ctx.strokeStyle = "#f2b33d";
      ctx.lineWidth = 3.5;
      ctx.lineCap = "round";
      ctx.beginPath();
      ctx.moveTo(r - 42, r);
      ctx.lineTo(r - 16, r);
      ctx.lineTo(r - 9, r + 7);
      ctx.moveTo(r + 42, r);
      ctx.lineTo(r + 16, r);
      ctx.lineTo(r + 9, r + 7);
      ctx.stroke();
      ctx.fillStyle = "#f2b33d";
      ctx.fillRect(r - 2.5, r - 2.5, 5, 5);
    }
  };

  // src/main.ts
  var $ = (id) => document.getElementById(id);
  var mmss = (t) => `${Math.floor(t / 60)}:${String(Math.floor(t % 60)).padStart(2, "0")}`;
  var App = class {
    constructor() {
      this.warp = 5;
      this.running = true;
      this.acc = 0;
      this.last = performance.now();
      this.wind = { fromDeg: 270, speedKt: 8, gusts: false };
      this.circuit = HIGH_CIRCUIT;
      this.pal = new Palette();
      this.plan = new PlanView($("map"), this.pal);
      this.profile = new ProfileView($("prof"), this.pal);
      this.adi = new AttitudeIndicator($("adi"), this.pal);
      this.logCount = -1;
      this.critKey = "";
      $("phases").innerHTML = PHASE_SEQUENCE.map((p) => `<div class="phase" data-p="${p.phase}" style="--c:var(${p.token})"><i></i>${p.name}</div>`).join("") + `<div class="phase ga" data-p="GO_AROUND" style="--c:var(${GO_AROUND_INFO.token})"><i></i><span id="gaChip">Go-around</span></div><div class="phase ga" data-p="ENGINE_OUT" style="--c:var(--ph-eo)"><i></i><span id="eoChip">Engine out</span></div>`;
      this.bindControls();
      this.syncWind();
      this.reset();
      requestAnimationFrame((t) => {
        this.last = t;
        this.frame(t);
      });
    }
    reset() {
      this.sim = new Simulation({ wind: { ...this.wind }, seed: Math.random() * 1e6 | 1, circuit: this.circuit });
      this.acc = 0;
      this.logCount = -1;
      this.critKey = "";
    }
    bindControls() {
      $("run").onclick = () => {
        if (this.sim.finished) {
          this.reset();
          this.running = true;
        } else this.running = !this.running;
        this.last = performance.now();
        this.updateRunLabel();
      };
      $("reset").onclick = () => {
        this.reset();
        this.running = true;
        this.updateRunLabel();
      };
      $("ga").onclick = () => {
        this.sim.requestGoAround();
      };
      $("eng").onclick = () => {
        this.sim.failEngine();
      };
      $("warp").addEventListener("click", (e) => {
        const b = e.target.closest("button");
        if (!b) return;
        this.warp = Number(b.dataset.w);
        $("warp").querySelectorAll("button").forEach((x) => x.setAttribute("aria-pressed", String(x === b)));
      });
      for (const id of ["wdir", "wspd", "gusts"]) $(id).addEventListener("input", () => this.syncWind());
      $("profile").addEventListener("click", (e) => {
        const b = e.target.closest("button");
        if (!b) return;
        this.circuit = b.dataset.c === "standard" ? STANDARD_CIRCUIT : HIGH_CIRCUIT;
        $("profile").querySelectorAll("button").forEach((x) => x.setAttribute("aria-pressed", String(x === b)));
        this.reset();
        this.running = true;
        this.updateRunLabel();
      });
    }
    updateRunLabel() {
      $("run").textContent = this.sim.finished ? "Fly again" : this.running ? "Pause" : "Resume";
    }
    syncWind() {
      this.wind.fromDeg = Number($("wdir").value);
      this.wind.speedKt = Number($("wspd").value);
      this.wind.gusts = $("gusts").checked;
      $("wdirV").textContent = String(this.wind.fromDeg).padStart(3, "0") + "\xB0";
      $("wspdV").textContent = this.wind.speedKt + " kt";
      if (this.sim) this.sim.wind.settings = { ...this.wind };
    }
    frame(now) {
      const real = Math.min(0.1, (now - this.last) / 1e3);
      this.last = now;
      if (this.running && !this.sim.finished) {
        this.acc += real * this.warp;
        while (this.acc >= Simulation.DT) {
          this.sim.step();
          this.acc -= Simulation.DT;
        }
        if (this.sim.finished) {
          this.running = false;
          this.updateRunLabel();
        }
      }
      this.plan.draw(this.sim);
      this.profile.draw(this.sim);
      this.adi.draw(this.sim);
      this.updateInstruments();
      this.renderLog();
      this.renderCheck();
      requestAnimationFrame((t) => this.frame(t));
    }
    updateInstruments() {
      const s = this.sim, ac = s.aircraft, ap = s.autopilot, gv = ac.groundVelocity(s.wind);
      const vsFpm = ac.verticalSpeed / FT * 60;
      const set = (id, v, unit) => {
        $(id).innerHTML = `${v}<em>${unit}</em>`;
      };
      set("rIas", (ac.V / KT).toFixed(0), "kt");
      set("rGs", (Math.hypot(gv.x, gv.y) / KT).toFixed(0), "kt");
      set("rAlt", (ac.h / FT).toFixed(0), "ft");
      set("rVs", (vsFpm > 5 ? "+" : "") + (Math.round(vsFpm / 10) * 10).toFixed(0), "fpm");
      const deg3 = (a) => String(Math.round(headingDeg(a)) % 360).padStart(3, "0");
      set("rHdg", deg3(ac.noseHeading), "\xB0");
      set("rTrk", deg3(ac.track(s.wind)), "\xB0");
      const slipSide = Math.abs(ac.beta) < 0.05 * D2R ? "" : ac.beta > 0 ? " L" : " R";
      set("rSlip", (Math.abs(ac.beta) / D2R).toFixed(1), "\xB0" + slipSide);
      set("rFlaps", String(ac.flaps), "\xB0");
      $("bThr").style.width = ac.thr * 100 + "%";
      $("vThr").textContent = Math.round(ac.thr * 100) + "%";
      $("bAoa").style.width = clamp(ac.alpha / (16 * D2R), 0, 1) * 100 + "%";
      $("vAoa").textContent = (ac.alpha / D2R).toFixed(1) + "\xB0";
      $("fmaThr").textContent = ap.fma.thr;
      $("fmaVert").textContent = ap.fma.vert;
      $("fmaLat").textContent = ap.fma.lat;
      const eoPhase = ENGINE_OUT_PHASES.includes(ap.phase) || !!ap.engineOut;
      $("fmaVert").classList.toggle("amber", ap.phase === "GO_AROUND" || eoPhase);
      $("fmaThr").classList.toggle("red", !!ap.engineOut);
      $("phaseNow").textContent = PHASES[ap.phase].name.toUpperCase() + (ap.phase === "DOWNWIND" && ap.descending ? " \xB7 DESC" : "");
      $("phaseNow").classList.toggle("amber", ap.phase === "GO_AROUND" || ENGINE_OUT_PHASES.includes(ap.phase));
      $("clock").textContent = mmss(s.t).padStart(5, "0");
      const eoPlan = ap.engineOut ? { REJECT: "REJECT TAKE-OFF", LAND_AHEAD: "LAND AHEAD", RETURN: "GLIDE TO RUNWAY" }[ap.engineOut.plan] : "";
      $("warn").textContent = ac.stalled ? "STALL" : ap.engineOut && !s.finished ? `ENGINE FAILURE \xB7 ${eoPlan}` : ap.phase === "GO_AROUND" ? `GO AROUND \xB7 ${ap.goAroundReason.toUpperCase()}` : ac.alpha > 13 * D2R && !ac.onGround ? "HIGH AOA" : "";
      const canGa = ap.phase === "BASE" || ap.phase === "FINAL" || ap.phase === "FLARE";
      const reach = $("reach");
      reach.textContent = s.runwayReachable === null ? ap.engineOut ? "\u2014" : ac.onGround ? "ON GROUND" : "\u2014" : s.runwayReachable ? `RWY ${s.reachEnd} \u2713` : "NONE";
      reach.className = s.runwayReachable === null ? "" : s.runwayReachable ? "ok" : "no";
      $("reachPct").textContent = s.airTime > 1 ? `${Math.round(s.reachFraction * 100)}% of flight` : "";
      $("ga").disabled = !canGa || ap.goAroundRequested || !!ap.engineOut;
      $("eng").disabled = !!ap.engineOut || s.finished || ap.phase === "ROLLOUT";
      const cur = PHASES[ap.phase].index, stopped = s.finished;
      document.querySelectorAll(".phase").forEach((el) => {
        if (el.dataset.p === "ENGINE_OUT") {
          el.hidden = !ap.engineOut;
          el.classList.toggle("now", ENGINE_OUT_PHASES.includes(ap.phase));
          el.classList.toggle("done", !!ap.engineOut && !ENGINE_OUT_PHASES.includes(ap.phase));
          $("eoChip").textContent = ENGINE_OUT_PHASES.includes(ap.phase) ? PHASES[ap.phase].name : "Engine out";
          return;
        }
        const p = el.dataset.p;
        if (p === "GO_AROUND") {
          el.classList.toggle("now", ap.phase === "GO_AROUND");
          el.classList.toggle("done", ap.goArounds > 0 && ap.phase !== "GO_AROUND");
          el.hidden = ap.goArounds === 0;
          $("gaChip").textContent = ap.goArounds > 1 ? `Go-around \xD7${ap.goArounds}` : "Go-around";
          return;
        }
        const i = PHASES[p].index, isNow = ap.phase !== "GO_AROUND" && !ENGINE_OUT_PHASES.includes(ap.phase) && i === cur && !stopped;
        el.classList.toggle("done", stopped || i < cur);
        el.classList.toggle("now", isNow);
      });
    }
    renderLog() {
      const ev = this.sim.events;
      if (ev.length === this.logCount) return;
      this.logCount = ev.length;
      $("log").innerHTML = ev.map((e) => {
        const info = e.phase === "DESCENT" || e.phase === "ENGINE_FAILURE" ? null : PHASES[e.phase];
        const name = info ? info.name : e.phase === "ENGINE_FAILURE" ? "Engine failure" : "Begin descent";
        const tok = info ? info.token : e.phase === "ENGINE_FAILURE" ? "--ph-eo" : "--muted";
        return `<span class="mut l">${mmss(e.t)}</span><span class="ph" style="color:var(${tok})">${name}${e.note ? `<small>${e.note}</small>` : ""}</span><span class="mut">${Math.round(e.h / FT)} ft</span><span class="mut">${Math.round(e.V / KT)} kt</span>`;
      }).join("");
      $("log").scrollTop = 1e6;
    }
    renderCheck() {
      const rows = this.sim.evaluate(), ap = this.sim.autopilot;
      const key = JSON.stringify(rows) + ap.goArounds + this.sim.phase;
      if (key === this.critKey) return;
      this.critKey = key;
      $("crit").innerHTML = rows.map((r) => {
        const cls = r.info ? "info" : r.pass === true ? "ok" : r.pass === false ? "bad" : "";
        const txt = r.info ? "Info" : r.pass === true ? "Pass" : r.pass === false ? "Fail" : "Pending";
        return `<div class="crit"><span class="k">${r.label}</span><span class="c">${r.info ? "" : "Target "}${r.target}</span><span class="v">${r.value ?? "\u2014"}</span><span class="chip ${cls}">${txt}</span></div>`;
      }).join("") + (ap.engineOut ? "" : `<div class="crit"><span class="k">Go-arounds</span><span class="c">Unstable approaches rejected</span><span class="v">${ap.goArounds}${ap.goAroundReason ? " \xB7 last: " + ap.goAroundReason : ""}</span><span class="chip info">Info</span></div>`);
      const v = $("verdict"), done = this.sim.finished;
      if (ap.phase === "REJECTED") {
        v.textContent = "Stopping";
        v.style.color = "var(--muted)";
      } else if (!this.sim.touchdown && !done) {
        v.textContent = ap.phase === "GO_AROUND" ? "Going around" : ap.engineOut ? "Gliding" : "In flight";
        v.style.color = "var(--muted)";
      } else if (!done) {
        v.textContent = "Rolling out";
        v.style.color = "var(--muted)";
      } else {
        const all = rows.every((r) => r.info || r.pass);
        v.textContent = all ? "All passed" : "Check failed";
        v.style.color = all ? "var(--ok)" : "var(--bad)";
      }
    }
  };
  new App();
})();
