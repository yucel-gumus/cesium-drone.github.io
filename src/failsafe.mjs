import { clamp } from "./flight-math.mjs";
import {
  referenceAt,
  roundedPath,
  horizontalReference,
  phaseSequence,
  appendLanding,
  verticalDuration,
  JERK_SMOOTHING,
} from "./trajectory.mjs";
import {
  terrainFollowingProfile,
  corridorFloor,
  rasterHeight,
  stationAt,
  indexStations,
} from "./terrain.mjs";
import { OBSTACLE_DEFAULTS, gridIndex, obstaclesNear } from "./obstacles.mjs";

// In-flight fault scenarios the operator can inject at a mission time. The
// flight is solved in advance, so a fault is a time-triggered event inside
// simulate(); the autopilot's failsafes below react to its symptoms, not
// to the scenario name.
export const FAULTS = Object.freeze({
  // A damaged cell: the pack's remaining charge drops by `loss` of its
  // capacity at once (the weakest cell limits a series pack).
  battery: Object.freeze({ label: "Batarya hücresi arızası", loss: 0.4 }),
  // GNSS outage (jamming, canyon shading) of `duration` seconds.
  "gnss-outage": Object.freeze({ label: "GNSS kesintisi", duration: 15 }),
  "gnss-loss": Object.freeze({ label: "GNSS kaybı", duration: Infinity }),
  // Loss of effectiveness of one rotor (damaged propeller): it keeps its
  // commanded speed but gives `effectiveness` of the thrust and torque.
  motor: Object.freeze({
    label: "Motor verim kaybı",
    motor: 0,
    effectiveness: 0.7,
  }),
});

// Autopilot failsafe parameters, in the spirit of PX4/ArduPilot defaults.
export const FAILSAFE = Object.freeze({
  // Seconds without a GNSS fix before the position is dead reckoning.
  gnssTimeout: 1,
  // Dead-reckoning position hold before landing in place, s.
  holdTimeout: 20,
  // Fix age after a GNSS return before the mission resumes, s.
  gnssSettle: 3,
  // Safety factor on the predicted charge of a return to home.
  batteryMargin: 1.25,
  // State of charge that forces a landing in place.
  batteryCritical: 0.1,
  // Low-passed relative speed excess of one motor over the mean of the
  // others that flags it as failing, the filter time constant and how long
  // the excess must persist.
  motorImbalance: 0.1,
  motorFilter: 2,
  motorPersistence: 1,
  // A field landing descends this far below the elevation-model ground,
  // as the real surface is only known to the model's accuracy, m.
  fieldOvershoot: 10,
  // Below this distance a return lands straight down, m.
  homeRadius: 3,
  // A landing away from home moves at most this far to touch down clear of
  // obstacles, searching rings this far apart, m.
  landingSearch: 150,
  landingSearchStep: 10,
});

// Operator commands during a flight (Turkish labels for the log).
export const OPERATOR_COMMANDS = Object.freeze({
  rtl: "eve dönüş (RTL)",
  hold: "konum tut",
  resume: "göreve devam",
  land: "yerinde iniş",
});
export function validateCommands(commands) {
  if (!Array.isArray(commands)) throw new Error("Operatör komutları bir liste olmalı.");
  return commands
    .map((c) => {
      if (!c || !Object.hasOwn(OPERATOR_COMMANDS, c.action))
        throw new Error(`Bilinmeyen operatör komutu: ${c?.action}`);
      if (!Number.isFinite(c.time) || c.time < 0)
        throw new Error("Operatör komutunun zamanı negatif olmayan bir sayı olmalı.");
      return { time: c.time, action: c.action, done: false };
    })
    .sort((a, b) => a.time - b.time);
}

export function validateFault(fault) {
  if (fault === null || fault === undefined) return null;
  const definition = FAULTS[fault.type];
  if (!definition) throw new Error(`Bilinmeyen arıza senaryosu: ${fault.type}`);
  if (!Number.isFinite(fault.time) || fault.time < 0)
    throw new Error("Arıza zamanı negatif olmayan bir sayı olmalı.");
  return { ...definition, ...fault };
}

// Smootherstep and its derivative and integral: a C2 speed ramp.
const ramp = (u) => u * u * u * (10 + u * (-15 + 6 * u)),
  rampRate = (u) => 30 * u * u * (1 - u) * (1 - u),
  rampIntegral = (u) => u * u * u * u * (2.5 + u * (-3 + u));

// Ground height of the elevation model at (x, y): the raster surface, or
// the corridor's centre line where no raster covers the point.
export function groundHeight(terrain, x, y) {
  if (!terrain) return NaN;
  const h = rasterHeight(terrain.raster, x, y);
  if (Number.isFinite(h)) return h;
  return stationAt(terrain, [x, y])?.height ?? NaN;
}

// Obstacle index of an elevation model from terrainForWorker(), built once.
function obstaclesOf(terrain) {
  const data = terrain?.obstacles;
  if (!data?.items?.length) return null;
  terrain.obstacleIndex ??= gridIndex(data.items, data.cell);
  return terrain.obstacleIndex;
}
// Tallest obstacle within `radius` of p, or null.
function tallestObstacle(index, p, radius) {
  let tallest = null;
  for (const hit of obstaclesNear(index, p, radius))
    if (!tallest || hit.obstacle.height > tallest.height) tallest = hit.obstacle;
  return tallest;
}
// What the checklist and the log name of an obstacle.
const describe = (o) =>
  o && {
    kind: o.kind,
    ...(o.name && { name: o.name }),
    height: o.height,
    heightSource: o.heightSource,
  };

// Whether the straight leg from p to q, with its corridor sides, lies on
// the sampled elevation model (corridorFloor() throws where it does not).
function legOnModel(terrain, p, q) {
  const d = Math.hypot(q[0] - p[0], q[1] - p[1]),
    tangent = [(q[0] - p[0]) / d, (q[1] - p[1]) / d],
    n = Math.ceil(d / 10);
  try {
    for (let i = 0; i <= n; i++)
      corridorFloor(
        terrain,
        [p[0] + ((q[0] - p[0]) * i) / n, p[1] + ((q[1] - p[1]) * i) / n],
        tangent,
      );
  } catch {
    return false;
  }
  return true;
}

// Where a landing away from home touches down: the point itself when no
// obstacle footprint lies within the lateral buffer, otherwise the nearest
// point of the elevation model that is clear, searched on rings
// FAILSAFE.landingSearchStep apart out to FAILSAFE.landingSearch (east
// first, counter-clockwise, so the choice is deterministic). The buffer
// grows by the navigation's horizontal uncertainty (EPH): on dead
// reckoning the vehicle only knows roughly where it is, so it needs a
// wider gap. Only sites whose leg stays on the sampled elevation model
// count. With nothing clear in reach the vehicle lands where it is and
// the checklist reports the obstacle. Returns { xy, moved (m), obstacle
// (the one avoided or landed on) }.
export function clearLandingSite(terrain, p, settings, uncertainty = 0) {
  const index = obstaclesOf(terrain),
    buffer = (settings.obstacleLateral ?? OBSTACLE_DEFAULTS.lateral) + uncertainty,
    here = { xy: [p[0], p[1]], moved: 0, obstacle: null };
  if (!index) return here;
  const below = tallestObstacle(index, p, buffer);
  if (!below) return here;
  const step = FAILSAFE.landingSearchStep;
  for (let r = step; r <= FAILSAFE.landingSearch + 1e-9; r += step) {
    const n = Math.max(8, Math.ceil((2 * Math.PI * r) / step));
    for (let k = 0; k < n; k++) {
      const a = (2 * Math.PI * k) / n,
        q = [p[0] + r * Math.cos(a), p[1] + r * Math.sin(a)];
      if (
        Number.isFinite(groundHeight(terrain, q[0], q[1])) &&
        !obstaclesNear(index, q, buffer).length &&
        legOnModel(terrain, p, q)
      )
        return { xy: q, moved: r, obstacle: describe(below) };
    }
  }
  return { ...here, obstacle: describe(below) };
}

// CG rest height and landing options for a safe point: the pad at home, the
// elevation model's ground at a rally point (or wherever the vehicle lands
// in place), which is only known to the model's accuracy.
function landingAt(target, terrain, settings) {
  if (target.site === "home") return { base: 0, settle: 4, xy: target.xy };
  const ground = terrain
    ? groundHeight(terrain, target.xy[0], target.xy[1])
    : -settings.landingOffset;
  if (!Number.isFinite(ground))
    throw new Error("İniş noktasının arazi yüksekliği bilinmiyor.");
  return {
    base: ground + settings.landingOffset,
    ground,
    settle: 4,
    xy: target.xy,
    overshoot: FAILSAFE.fieldOvershoot,
  };
}

// Return to a safe point along a direct, terrain-following leg, as
// autopilots with a terrain database fly it: from the stop point `p` (ENU,
// at rest) climb in place to the leg's height while turning towards the
// target, fly the leg at the target AGL over the elevation model, then land:
// on the pad at home, on the modelled ground at a rally point or a clear
// field site. The leg's requirement follows the planned route's rule: the
// raster/corridor floor plus the target AGL, or plus the obstacle clearance
// over the tallest obstacle within the corridor and the lateral buffer
// where that is higher. It is never below the stop height at its start, so
// the climb-out is the only height change before the leg profile takes
// over.
export function buildReturn(
  p,
  yaw,
  terrain,
  settings,
  target = { site: "home", xy: [0, 0] },
) {
  const sequence = phaseSequence(),
    land = landingAt(target, terrain, settings),
    distance = Math.hypot(p[0] - target.xy[0], p[1] - target.xy[1]);
  if (distance < FAILSAFE.homeRadius) {
    appendLanding(sequence, p[2], yaw, settings, { ...land, xy: p });
    return { ref: finish(sequence, null), leg: null, land };
  }
  const path = roundedPath(
      [
        [p[0], p[1]],
        [target.xy[0], target.xy[1]],
      ],
      settings.turnTrim,
      2,
    ),
    table = path.table,
    // Without an elevation model (synthetic tests) the ground is level with
    // the pad.
    floor = Float64Array.from(table, (k) =>
      terrain
        ? corridorFloor(terrain, k.p, k.tangent)
        : -settings.landingOffset,
    ),
    requirement = floor.map((f) => f + settings.agl),
    obstacles = obstaclesOf(terrain),
    reach =
      (settings.corridorHalfWidth ?? terrain?.halfWidth ?? 0) +
      (settings.obstacleLateral ?? OBSTACLE_DEFAULTS.lateral),
    clearance = settings.obstacleClearance ?? OBSTACLE_DEFAULTS.clearance;
  // The obstacle that lifts the leg most above its target AGL.
  let raised = null;
  if (obstacles)
    table.forEach((k, i) => {
      const o = tallestObstacle(obstacles, k.p, reach);
      if (!o || o.height + clearance <= settings.agl) return;
      requirement[i] = floor[i] + o.height + clearance;
      const lift = o.height + clearance - settings.agl;
      if (!raised || lift > raised.lift) raised = { lift, obstacle: describe(o) };
    });
  requirement[0] = Math.max(requirement[0], p[2]);
  const spacing = table.length > 1 ? table[1].s - table[0].s : 1,
    profile = terrainFollowingProfile(requirement, spacing, settings),
    leg = horizontalReference(path, profile, settings, {
      jerkSmoothing: JERK_SMOOTHING,
    });
  sequence.vertical(
    "rtl",
    verticalDuration(profile[0] - p[2], settings.climbSpeed, settings),
    p[2],
    profile[0],
    leg.initialYaw,
    0,
    0,
    p,
  );
  const legStart = sequence.clock;
  sequence.horizontal("rtl", leg.duration);
  appendLanding(sequence, leg.endHeight, leg.finalYaw, settings, land);
  return {
    ref: finish(sequence, leg),
    land,
    leg: {
      start: legStart,
      points: table.map((k) => k.p),
      floor,
      profile,
      distance,
      climb: profile[0] - p[2],
      target,
      raised,
    },
  };
}

// Landing away from home, onto the elevation model's ground under the point
// the autopilot believes it holds. With `avoid` (the autopilot's own
// failsafe landings) a point over an obstacle is replaced by the nearest
// clear site, flown to over the obstacle profile; an operator's "land"
// lands exactly where commanded.
export function buildFieldLanding(
  p,
  yaw,
  terrain,
  settings,
  { avoid = false, uncertainty = 0 } = {},
) {
  const site =
    avoid && terrain ? clearLandingSite(terrain, p, settings, uncertainty) : null;
  if (site?.moved) {
    const built = buildReturn(p, yaw, terrain, settings, {
      site: "field",
      xy: site.xy,
    });
    return {
      ref: built.ref,
      base: built.land.base,
      ground: built.land.ground,
      leg: built.leg,
      site,
    };
  }
  const sequence = phaseSequence(),
    land = landingAt({ site: "field", xy: p }, terrain, settings);
  appendLanding(sequence, p[2], yaw, settings, { ...land, settle: 2 });
  return {
    ref: finish(sequence, null),
    base: land.base,
    ground: land.ground,
    leg: null,
    site,
  };
}

function finish(sequence, leg) {
  const horizontal = sequence.phases.find((p) => p.horizontal);
  return {
    horizontal: leg?.horizontal ?? [],
    phases: sequence.phases,
    duration: sequence.clock,
    surveyStart: horizontal?.startTime ?? 0,
    horizontalDuration: leg?.duration ?? 0,
    initialYaw: leg?.initialYaw ?? 0,
    finalYaw: leg?.finalYaw ?? 0,
  };
}

const FINAL_PHASES = new Set(["settle", "landing", "touchdown", "disarm"]);

// The reference the controller follows during a flight with failsafes: the
// active reference (the mission, then perhaps a return or a landing) played
// on its own clock τ. Any of them can be slowed to a stop with a C2 time
// warp: the vehicle brakes along its current path (for the mission, inside
// the validated corridor), holds on it, and either speeds up again on
// resume or starts a new reference from the stop point: a return to the
// nearest safe point or a landing in place. `context` = { settings,
// terrain, rally: [{ name, xy }], rtlTarget: "nearest" | "home" }.
export function makeGuidance(mission, context = {}) {
  const settings = context.settings,
    terrain = context.terrain ? indexStations(context.terrain) : null,
    rally = context.rtlTarget === "home" ? [] : (context.rally ?? []);
  let active = { ref: mission, base: 0, kind: "mission" },
    mode = "mission",
    warp = null,
    pending = null,
    hold = 0,
    surface = 0,
    rtl = null,
    landing = { site: "home", xy: [0, 0] },
    avoidObstacles = false,
    transitions = [];
  const guidance = {
    get mode() {
      return mode;
    },
    get pending() {
      return pending;
    },
    // End of the active reference; open while its clock is warped or held,
    // as only a later decision ends a hold.
    get endTime() {
      return warp || mode === "hold" || mode === "brake" || mode === "resume"
        ? Infinity
        : active.base + active.ref.duration;
    },
    // CG rest height of the landing surface below the current reference.
    get surface() {
      return surface;
    },
    get rtl() {
      return rtl;
    },
    get landing() {
      return landing;
    },
    transitions,
    // The safe point a return from `p` would fly to: home or the nearest
    // rally point.
    returnTarget(p) {
      let best = { site: "home", xy: [0, 0], name: "Kalkış noktası" },
        distance = Math.hypot(p[0], p[1]);
      for (const point of rally) {
        const d = Math.hypot(p[0] - point.xy[0], p[1] - point.xy[1]);
        if (d < distance) {
          distance = d;
          best = { site: "rally", xy: point.xy, name: point.name };
        }
      }
      return best;
    },
    // Active clock and its rate at time t.
    clock(t) {
      if (mode === "hold") return { tau: hold, rate: 0, rateDot: 0 };
      if (!warp) return { tau: t - active.base, rate: 1, rateDot: 0 };
      const u = clamp((t - warp.t0) / warp.T, 0, 1),
        dr = warp.r1 - warp.r0;
      return {
        tau:
          warp.tau0 +
          warp.T * (warp.r0 * u + dr * rampIntegral(u)) +
          warp.r1 * Math.max(0, t - warp.t0 - warp.T),
        rate: warp.r0 + dr * ramp(u),
        rateDot: u < 1 ? (dr * rampRate(u)) / warp.T : 0,
        done: u >= 1,
      };
    },
    // Phase of the mission reference at time t (null once replaced).
    missionPhase(t) {
      return active.kind === "mission"
        ? referenceAt(mission, guidance.clock(t).tau).phase
        : null;
    },
    // Phase of whatever reference is active at time t.
    activePhase(t) {
      return referenceAt(active.ref, guidance.clock(t).tau).phase;
    },
    at(t) {
      let c = guidance.clock(t);
      if (warp && c.done) {
        // The ramp has finished: a resume returns to the active clock, a
        // brake stops it and performs what it was braking for.
        if (warp.r1 > 0) {
          active.base = t - c.tau;
          warp = null;
          setMode(active.kind, t);
        } else {
          hold = c.tau;
          warp = null;
          const action = pending;
          pending = null;
          if (action === "hold") setMode("hold", t);
          else return start(action, t);
        }
        c = guidance.clock(t);
      }
      const n = referenceAt(active.ref, c.tau);
      if (c.rate === 1 && c.rateDot === 0) return n;
      // Time warp: same geometry, velocity × rate, acceleration × rate² plus
      // the path velocity times the rate's derivative.
      return {
        ...n,
        v: n.v.map((x) => x * c.rate),
        a: n.a.map((x, i) => x * c.rate * c.rate + n.v[i] * c.rateDot),
        yawRate: n.yawRate * c.rate,
        // The camera keeps its distance trigger while the vehicle brakes or
        // speeds up on the mission path; a return switches it off at once.
        // An aborted take-off brakes its climb and stays a take-off.
        phase:
          mode === "hold"
            ? "hold"
            : pending === "rtl" && n.phase !== "takeoff"
              ? "rtl"
              : n.phase,
      };
    },
    // Slow the active clock to a stop along the path, then do `action`
    // ("hold", "rtl" or "land"). Refused once the vehicle is in its final
    // landing. `avoid` lets a landing move clear of obstacles (failsafes),
    // with the buffer widened by the position `uncertainty` (EPH, m).
    stop(t, action, { avoid = false, uncertainty = 0 } = {}) {
      if (FINAL_PHASES.has(guidance.activePhase(t)) && mode !== "hold")
        return false;
      if (action === "land") avoidObstacles = avoid && { uncertainty };
      if (mode === "hold") {
        if (action !== "hold") start(action, t);
        return true;
      }
      if (warp && warp.r1 === 0) {
        // Already braking: the latest, stronger action wins.
        if (action !== "hold") pending = action;
        return true;
      }
      const c = guidance.clock(t),
        n = referenceAt(active.ref, c.tau),
        horizontal = Math.hypot(n.v[0], n.v[1]) * c.rate,
        vertical = Math.abs(n.v[2]) * c.rate;
      warp = {
        t0: t,
        tau0: c.tau,
        r0: c.rate,
        r1: 0,
        T: Math.max(
          2,
          (1.875 * horizontal) / settings.acceleration,
          (1.875 * vertical) / (settings.verticalAcceleration ?? 1),
        ),
      };
      pending = action;
      setMode("brake", t);
      return true;
    },
    resume(t) {
      if (mode !== "hold") return false;
      const n = referenceAt(active.ref, hold);
      warp = {
        t0: t,
        tau0: hold,
        r0: 0,
        r1: 1,
        T: Math.max(
          2,
          (1.875 * Math.hypot(n.v[0], n.v[1])) / settings.acceleration,
          (1.875 * Math.abs(n.v[2])) / (settings.verticalAcceleration ?? 1),
        ),
      };
      setMode("resume", t);
      return true;
    },
  };
  function setMode(next, t) {
    mode = next;
    transitions.push({ time: t, mode: next });
  }
  // Replace the active reference with a return or a field landing from the
  // stop point.
  function start(action, t) {
    const n = referenceAt(active.ref, hold),
      p = n.p;
    if (action === "rtl") {
      const target = guidance.returnTarget(p),
        built = buildReturn(p, n.yaw, terrain, settings, target);
      active = { ref: built.ref, base: t, kind: "rtl" };
      rtl = built.leg && {
        ...built.leg,
        start: t + built.leg.start,
        from: [p[0], p[1], p[2]],
      };
      landing =
        target.site === "home"
          ? { site: "home", xy: [0, 0] }
          : {
              site: "rally",
              name: target.name,
              xy: [target.xy[0], target.xy[1]],
              ground: built.land.ground,
            };
      surface = built.land.base;
    } else {
      const built = buildFieldLanding(p, n.yaw, terrain, settings, {
        avoid: Boolean(avoidObstacles),
        uncertainty: avoidObstacles?.uncertainty ?? 0,
      });
      active = { ref: built.ref, base: t, kind: "land" };
      surface = built.base;
      landing = {
        site: "field",
        xy: built.site?.moved ? built.site.xy : [p[0], p[1]],
        ground: built.ground,
        // Distance moved clear of an obstacle and the obstacle avoided (or,
        // with nothing clear in reach, the one landed on).
        moved: built.site?.moved ?? 0,
        obstacle: built.site?.obstacle ?? null,
      };
      if (built.leg)
        rtl = { ...built.leg, start: t + built.leg.start, from: [p[0], p[1], p[2]] };
    }
    setMode(action, t);
    return referenceAt(active.ref, 0);
  }
  return guidance;
}

// The planner's own timing of a return, which the battery check needs at
// every control step and cannot rebuild a leg for: the direct leg's
// duration over level ground at a few distances, as buildReturn() plans it
// with the jerk smoothing of its speed profile (linear between them, the
// last slope beyond), the brake to a stop of the guidance time warp, the
// turn and climb-out, and the rest-to-rest quintic descent and touchdown
// crawl of appendLanding(). Cached per settings object.
const timings = new WeakMap();
export function returnTiming(settings) {
  let timing = timings.get(settings);
  if (timing) return timing;
  const height = settings.agl - settings.landingOffset,
    distances = [0, 10, 25, 50, 100, 200, 400, 800, 1600, 3200],
    legs = distances.map((d) =>
      d < FAILSAFE.homeRadius
        ? 0
        : buildReturn([d, 0, height], 0, null, settings).ref.horizontalDuration,
    ),
    crawl = settings.landingCrawlHeight ?? 3,
    creep = settings.landingSpeed ?? 0.35,
    below = settings.landingOvershoot ?? 2.5,
    ramp = 2,
    rampEnd = crawl - (creep * ramp) / 2;
  timing = {
    leg(d) {
      let i = 0;
      while (i < distances.length - 2 && d > distances[i + 1]) i++;
      const f = (d - distances[i]) / (distances[i + 1] - distances[i]);
      return Math.max(0, legs[i] + (legs[i + 1] - legs[i]) * f);
    },
    brake: (speed) => Math.max(2, (1.875 * speed) / settings.acceleration),
    climbOut: verticalDuration(0, settings.climbSpeed, settings),
    // Settle, quintic descent to the crawl height, ramp and crawl to
    // contact; the overshoot below the surface covers the land detector.
    landing: (drop) =>
      4 +
      verticalDuration(Math.max(0, drop - crawl), settings.descentSpeed, settings) +
      ramp +
      (rampEnd + below) / creep,
  };
  timings.set(settings, timing);
  return timing;
}

// Predicted charge (fraction of capacity) to return from `p` (ENU, relative
// to the return target, height above home in p[2]). The phases are timed as
// the planner will fly them (returnTiming()), so the prediction is unbiased
// and the failsafe safety factor covers only the model's error. The leg's
// mean ground speed and the estimated wind set its airspeed: a headwind leg
// needs more power than a tailwind one. Power follows the simulation's own
// rotor model, shaft power ∝ thrust^1.5, with the thrust tilted to hold the
// leg's drag, T = m √(g² + (F_d / m g)²): P_leg = P_hover (1 + (F_d / m
// g)²)^¾, from the hover-equivalent shaft power measured in flight
// (`hoverShaft`). Brake, turn, descent from the target AGL (or the planned
// return height over home, if higher) and the landing crawl are flown near
// hover power; `speed` is the current ground speed the brake starts from.
export function returnCharge({
  p,
  wind = [0, 0],
  hoverShaft,
  voltage,
  homeHeight = 0,
  speed,
  settings,
  battery,
  airframe,
}) {
  const timing = returnTiming(settings),
    distance = Math.hypot(p[0], p[1]),
    leg = distance < FAILSAFE.homeRadius ? 0 : timing.leg(distance),
    legSpeed = leg > 0 ? distance / leg : 0,
    toward = distance > 1e-6 ? [-p[0] / distance, -p[1] / distance] : [0, 0],
    airspeed = Math.hypot(
      legSpeed * toward[0] - wind[0],
      legSpeed * toward[1] - wind[1],
    ),
    drag =
      0.5 * airframe.airDensity * airframe.dragArea * airspeed * airspeed +
      airframe.linearDrag * airspeed,
    tilt = (1 + (drag / (airframe.mass * airframe.gravity)) ** 2) ** 0.75,
    electrical = (shaft) => shaft / battery.propulsionEfficiency + battery.avionicsPower,
    rest =
      timing.brake(speed ?? settings.cruiseSpeed) +
      timing.climbOut +
      timing.landing(Math.max(settings.agl, homeHeight)),
    joules = electrical(hoverShaft * tilt) * leg + electrical(hoverShaft) * rest;
  return (
    (FAILSAFE.batteryMargin * joules) /
    (voltage * 3600 * battery.capacityAh)
  );
}
