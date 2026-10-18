import test from "node:test";
import assert from "node:assert/strict";
import fs from "node:fs";
import * as C from "cesium";
import { createTerrainRoute, buildMission } from "../src/flight-planner.mjs";
import {
  simulate,
  derivative,
  initialState,
  hoverRotors,
  inducedVelocity,
  vortexRingLoss,
  AIRFRAME,
} from "../src/quadrotor.mjs";
import { analyzeMission } from "../src/mission-analysis.mjs";

const data = JSON.parse(
  fs.readFileSync(new URL("../public/points.json", import.meta.url)),
);
const near = (a, b, t = 1e-6) =>
  assert.ok(Math.abs(a - b) <= t, `${a} != ${b} (±${t})`);
const g = AIRFRAME.gravity,
  R = AIRFRAME.rotorRadius,
  hover = hoverRotors(AIRFRAME) / 4;
// Level airframe at hover rotor speeds, CG `height` above the surface whose
// rest height is 0, moving with ENU velocity `v` through still air.
function accelerations(height, v = [0, 0, 0], q = [1, 0, 0, 0]) {
  const s = initialState(0);
  s[2] = height;
  s.set(v, 3);
  s.set(q, 6);
  s.fill(hover, 13, 17);
  return derivative(s, [hover, hover, hover, hover], 0, { ...AIRFRAME, groundLevel: 0 }, 0);
}

test("ground effect raises thrust by 1 / (1 − (R / 4h)²) near the surface only", () => {
  // Rotor plane one radius above the ground: T / T∞ = 1 / (1 − 1/16).
  const atRadius = accelerations(R - AIRFRAME.rotorClearance);
  near(atRadius[5], g * (1 / (1 - 1 / 16) - 1), 1e-9);
  // Resting on the gear (rotor plane 0.15 m up): about 5 %.
  const resting = accelerations(0)[5] / g;
  assert.ok(resting > 0.04 && resting < 0.06, `${resting}`);
  // Two metres up the effect is below 0.1 %, at altitude it is gone.
  assert.ok(accelerations(2)[5] / g < 1e-3);
  near(accelerations(50)[5], 0, 1e-12);
});

test("vortex ring state: thrust loss only in the descent band and only with little in-plane airflow", () => {
  const vh = inducedVelocity(AIRFRAME);
  near(vh, Math.sqrt((AIRFRAME.mass * g) / 4 / (2 * AIRFRAME.airDensity * Math.PI * R * R)), 1e-12);
  assert.equal(vortexRingLoss(0.25 * vh, 0, vh), 0);
  assert.equal(vortexRingLoss(1.7 * vh, 0, vh), 0);
  const peak = vortexRingLoss(0.95 * vh, 0, vh);
  near(peak, AIRFRAME.vrsLoss, 1e-12);
  // Forward speed carries the wake away.
  near(vortexRingLoss(0.95 * vh, 0.5 * vh, vh), peak / Math.E, 1e-12);
  // In the dynamics: descending vertically at v_h at altitude loses about
  // a quarter of the thrust; body drag pushes back up.
  const descending = accelerations(50, [0, 0, -vh]),
    up = (0.5 * AIRFRAME.airDensity * AIRFRAME.dragArea * vh * vh) / AIRFRAME.mass;
  near(descending[5], -g * vortexRingLoss(vh, 0, vh) + up, 1e-9);
  assert.ok(descending[5] < -0.1 * g);
  // Same descent at 8 m/s forward: almost no loss (body drag aside).
  const forward = accelerations(50, [8, 0, -vh]),
    drag = (0.5 * AIRFRAME.airDensity * AIRFRAME.dragArea * Math.hypot(8, vh) * vh) / AIRFRAME.mass;
  near(forward[5], drag, 0.05);
});

test("blade flapping pitches the nose up in forward flight and rolls away from a sideslip; rotor drag scales with rotor speed", () => {
  const k = AIRFRAME.flappingMoment * 4 * hover,
    [ix, iy] = AIRFRAME.inertia;
  const forward = accelerations(50, [5, 0, 0]);
  // Heading east (yaw 0): body x is east; nose-up is a negative pitch rate.
  near(forward[11], (-k * 5) / iy, 1e-9);
  near(forward[10], 0, 1e-12);
  const sideslip = accelerations(50, [0, 4, 0]);
  near(sideslip[10], (k * 4) / ix, 1e-9);
  // Half the rotor speed: half the rotor drag (body drag unchanged).
  const s = initialState(0);
  s[2] = 50;
  s[3] = 6;
  s.fill(hover / 2, 13, 17);
  const half = derivative(s, [0, 0, 0, 0], 0, AIRFRAME, 0),
    body = (0.5 * AIRFRAME.airDensity * AIRFRAME.dragArea * 36) / AIRFRAME.mass;
  near(half[3], -body - (AIRFRAME.linearDrag * 6) / 2 / AIRFRAME.mass, 1e-9);
});

test("the checklist flags a vertical descent fast enough for the vortex ring state", () => {
  const plan = (descentSpeed) => {
    const route = createTerrainRoute(C, data, { descentSpeed });
    route.samples.forEach((p) => (p.height = 1150));
    const mission = buildMission(C, route);
    mission.simulation = simulate(mission.reference);
    return { mission, check: () => analyzeMission(C, attach(mission)).checks.find((c) => c.id === "vrs") };
  };
  const attach = (m) => Object.assign(m, { nominal: { simulation: m.simulation, minClearance: 80, maxAgl: 80, maxCorridorDeviation: 0 }, minClearance: 80, maxAgl: 80, maxCorridorDeviation: 0 });
  const normal = plan(1.5);
  assert.equal(normal.check().ok, true, normal.check().detail);
  assert.equal(normal.mission.simulation.metrics.maxVrsLoss, 0);
  // 4 m/s straight down (0.6 v_h): the rotors meet their own wake.
  const fast = plan(4);
  const check = fast.check();
  assert.equal(check.ok, false, check.detail);
  assert.equal(check.blocking, false);
  assert.ok(fast.mission.simulation.metrics.maxVrsLoss > 0.05);
  assert.ok(fast.mission.simulation.metrics.disarmed);
});
