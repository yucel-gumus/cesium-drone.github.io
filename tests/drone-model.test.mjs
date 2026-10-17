import test from "node:test";
import assert from "node:assert/strict";
import fs from "node:fs";
import {
  ROTOR_NODES,
  MODEL_ROOT_NODE,
  MODEL_MOTOR_SPACING,
  MODEL_BASE_Y,
  modelScale,
  modelGroundOffset,
} from "../src/drone-model.mjs";
import { AIRFRAME, SPIN } from "../src/quadrotor.mjs";
import { FLIGHT_DEFAULTS } from "../src/flight-planner.mjs";

const glb = fs.readFileSync(new URL("../CesiumDrone.glb", import.meta.url));
const gltf = JSON.parse(
  glb.subarray(20, 20 + glb.readUInt32LE(12)).toString(),
);
const node = (name) => gltf.nodes.find((n) => n.name === name);

test("rotor nodes map physics motor order to the model's positions and spin names", () => {
  // Physics order is front right, back right, back left, front left (FLU).
  // In glTF space +X is the airframe's left and +Z its forward.
  const expected = [
    [-1, 1],
    [-1, -1],
    [1, -1],
    [1, 1],
  ];
  ROTOR_NODES.forEach((name, i) => {
    const n = node(name);
    assert.ok(n, `${name} missing from GLB`);
    assert.equal(Math.sign(n.translation[0]), expected[i][0], `${name} side`);
    assert.equal(Math.sign(n.translation[2]), expected[i][1], `${name} fore/aft`);
    const clockwise = !name.includes("CounterClockwise");
    assert.equal(SPIN[i], clockwise ? 1 : -1, `${name} spin direction`);
  });
  assert.ok(node(MODEL_ROOT_NODE)?.children?.length > 0);
});

test("model scale and ground offset follow the GLB geometry", () => {
  const xs = ROTOR_NODES.map((name) => node(name).translation[0]);
  assert.ok(
    Math.abs(Math.max(...xs) - Math.min(...xs) - MODEL_MOTOR_SPACING) < 1e-6,
  );
  const base = node("Base_low");
  const { min } =
    gltf.accessors[gltf.meshes[base.mesh].primitives[0].attributes.POSITION];
  assert.ok(Math.abs(min[1] - MODEL_BASE_Y) < 1e-6);
  const scale = modelScale(AIRFRAME.armLength);
  assert.ok(
    Math.abs(scale * MODEL_MOTOR_SPACING - (2 * AIRFRAME.armLength) / Math.SQRT2) <
      1e-12,
  );
  const offset = modelGroundOffset(
    AIRFRAME.armLength,
    FLIGHT_DEFAULTS.landingOffset,
  );
  // The lowest model point ends exactly landingOffset below the CG.
  assert.ok(
    Math.abs((MODEL_BASE_Y + offset) * scale + FLIGHT_DEFAULTS.landingOffset) <
      1e-12,
  );
});
