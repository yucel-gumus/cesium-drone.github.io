import test from "node:test";
import assert from "node:assert/strict";
import fs from "node:fs";
import * as C from "cesium";
import { createTerrainRoute, buildMission, attachSimulation } from "../src/flight-planner.mjs";
import { simulate, AIRFRAME, meanWind } from "../src/quadrotor.mjs";
import { BATTERY } from "../src/battery.mjs";
import { toCSV, toULog } from "../src/exports.mjs";
import {
  leastSquares,
  instrumental,
  fromSimulation,
  fromCSV,
  parseULog,
  fromULog,
  identify,
} from "../src/sysid.mjs";

const data = JSON.parse(
  fs.readFileSync(new URL("../public/points.json", import.meta.url)),
);
const valley = (p) =>
  1146 + 30 * Math.max(0, (C.Math.toDegrees(p.latitude) - 40.6195) / 0.001);
const route = createTerrainRoute(C, data, { wind: 3 });
route.samples.forEach((p) => (p.height = valley(p)));
const mission = buildMission(C, route);
// The "real" vehicle differs from the nominal model in every identifiable
// parameter.
const TRUTH = {
  ...AIRFRAME,
  thrustCoefficient: 1.95e-5,
  dragArea: 0.1,
  linearDrag: 0.14,
  inertia: [0.04, 0.043, 0.07],
  flappingMoment: 3.5e-6,
  rotorInertia: 8e-5,
  angularDrag: [0.004, 0.004, 0.005],
};
const PACK = { ...BATTERY, internalResistance: 0.045, propulsionEfficiency: 0.74, avionicsPower: 22 };
const flown = simulate(mission.reference, { airframe: TRUTH, battery: PACK, wind: 3 });
const relative = (estimate, truth, tolerance, label) =>
  assert.ok(
    Math.abs(estimate / truth - 1) <= tolerance,
    `${label}: ${estimate} vs ${truth} (${(100 * (estimate / truth - 1)).toFixed(1)} %)`,
  );

test("least squares recovers a linear law; instrumental variables remove the errors-in-variables bias", () => {
  const X = [],
    y = [];
  for (let i = 0; i < 200; i++) {
    X.push([Math.sin(i), 1]);
    y.push(3 * Math.sin(i) - 2);
  }
  const fit = leastSquares(X, y);
  assert.ok(Math.abs(fit.beta[0] - 3) < 1e-9 && Math.abs(fit.beta[1] + 2) < 1e-9);
  // y = 2 x with x observed through noise as large as its own spread: least
  // squares attenuates the slope by about half, an instrument sharing the
  // signal but not the noise does not.
  let seed = 1;
  const noise = () => {
    seed = (seed * 16807) % 2147483647;
    return (seed / 2147483647 - 0.5) * 2 * Math.sqrt(3);
  };
  const noisy = [],
    instrument = [],
    out = [];
  for (let i = 0; i < 20000; i++) {
    const x = Math.sin(i * 0.01) * Math.SQRT2;
    noisy.push([x + noise()]);
    instrument.push([x + noise()]);
    out.push(2 * x);
  }
  const ls = leastSquares(noisy, out).beta[0],
    iv = instrumental(noisy, instrument, out).beta[0];
  assert.ok(ls < 1.5, `${ls}`);
  assert.ok(Math.abs(iv - 2) < 0.05, `${iv}`);
});

test("identification recovers the airframe and pack from a noisy, gusty mission log", () => {
  const result = identify(fromSimulation(flown), AIRFRAME, BATTERY);
  const value = (key) => result.report.find((r) => r.key === key);
  relative(value("thrustCoefficient").estimate, TRUTH.thrustCoefficient, 0.01, "k");
  relative(value("dragArea").estimate, TRUTH.dragArea, 0.05, "CdA");
  relative(value("linearDrag").estimate, TRUTH.linearDrag, 0.08, "rotor drag");
  relative(value("inertiaX").estimate, TRUTH.inertia[0], 0.06, "Ix");
  relative(value("inertiaY").estimate, TRUTH.inertia[1], 0.06, "Iy");
  relative(value("flappingMoment").estimate, TRUTH.flappingMoment, 0.03, "flapping");
  relative(value("propulsionEfficiency").estimate, PACK.propulsionEfficiency, 0.02, "η");
  relative(value("avionicsPower").estimate, PACK.avionicsPower, 0.05, "avionics");
  relative(value("internalResistance").estimate, PACK.internalResistance, 0.03, "R");
  // The mean of the gusty wind.
  const wind = meanWind(3);
  assert.ok(Math.hypot(result.wind[0] - wind[0], result.wind[1] - wind[1]) < 0.3, `${result.wind}`);
  // Applied parameters agree between the interleaved halves; I_z (yaw,
  // weakly excited) is reported but held at its nominal value here.
  for (const r of result.report.filter((x) => x.applied)) assert.ok(r.spread <= 0.15, r.label);
  assert.equal(value("inertiaZ").applied, false);
  assert.equal(result.airframe.inertia[2], AIRFRAME.inertia[2]);
  // Closed loop: the calibrated model predicts the landing charge of the
  // logged flight within 0.3 %, the nominal one misses it by about 2 %.
  const nominal = simulate(mission.reference, { wind: 3 }),
    calibrated = simulate(mission.reference, {
      airframe: result.airframe,
      battery: result.battery,
      wind: 3,
    });
  const truthSoc = flown.metrics.finalSoc;
  assert.ok(Math.abs(calibrated.metrics.finalSoc - truthSoc) < 0.003, `${calibrated.metrics.finalSoc} ${truthSoc}`);
  assert.ok(Math.abs(nominal.metrics.finalSoc - truthSoc) > 0.015);
});

test("PX4 ULog: the simulator's .ulg parses back with nested ESC reports, and calibrates like the direct log", () => {
  const flight = attachSimulation({ ...mission }, flown),
    bytes = toULog(flight),
    log = parseULog(bytes);
  assert.equal(log.info.sys_name, "cesium-drone-sim");
  assert.deepEqual(Object.keys(log.topics), [
    "vehicle_attitude",
    "vehicle_local_position",
    "sensor_combined",
    "esc_status",
    "battery_status",
  ]);
  const esc = log.topics.esc_status;
  // Padding is skipped, the nested array is flattened.
  assert.ok(!Object.keys(esc).some((k) => k.startsWith("_padding")));
  assert.equal(esc["esc[7].esc_rpm"].length, flown.rows);
  assert.equal(esc.esc_count[0], 4);
  // NED: level and at rest on the pad the accelerometer reads −g on z.
  near(log.topics.sensor_combined["accelerometer_m_s2[2]"][0], -9.80665, 0.05);
  const { rows } = fromULog(bytes);
  const result = identify(rows, AIRFRAME, BATTERY),
    value = (key) => result.report.find((r) => r.key === key).estimate;
  relative(value("thrustCoefficient"), TRUTH.thrustCoefficient, 0.01, "k");
  relative(value("dragArea"), TRUTH.dragArea, 0.05, "CdA");
  // Roll and pitch come out right only with the motor order mapped.
  relative(value("inertiaX"), TRUTH.inertia[0], 0.06, "Ix");
  relative(value("inertiaY"), TRUTH.inertia[1], 0.06, "Iy");
  relative(value("internalResistance"), PACK.internalResistance, 0.03, "R");
  assert.throws(() => parseULog(new Uint8Array(32)), /ULog başlığı/);
  assert.throws(() => fromULog(bytes.subarray(0, 16)), /konusu yok/);
});

test("the CSV export calibrates the aerodynamics too, from its 10 Hz rows and without its truth columns", () => {
  const flight = attachSimulation({ ...mission }, flown),
    csv = toCSV(flight),
    rows = fromCSV(csv);
  assert.equal(rows.length, Math.ceil(flown.rows / 3));
  // Wipe the true wind, thrust and torque columns: nothing changes.
  const header = csv.split("\n")[0].split(","),
    hidden = ["wind_e_ms", "wind_n_ms", "wind_u_ms", "thrust_N", "torque_x_Nm", "torque_y_Nm", "torque_z_Nm"].map((k) => header.indexOf(k)),
    blind = csv
      .split("\n")
      .map((line, i) => (i ? line.split(",").map((x, j) => (hidden.includes(j) ? "0" : x)).join(",") : line))
      .join("\n");
  const result = identify(fromCSV(blind), AIRFRAME, BATTERY),
    value = (key) => result.report.find((r) => r.key === key).estimate;
  relative(value("thrustCoefficient"), TRUTH.thrustCoefficient, 0.01, "k");
  relative(value("dragArea"), TRUTH.dragArea, 0.1, "CdA");
  relative(value("internalResistance"), PACK.internalResistance, 0.03, "R");
  assert.throws(() => fromCSV("time_s,east_m\n1,2"), /CSV sütunu eksik/);
});

function near(a, b, t) {
  assert.ok(Math.abs(a - b) <= t, `${a} != ${b} (±${t})`);
}
