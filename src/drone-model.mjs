// Geometry of the bundled CesiumDrone.glb. glTF axes are +X left, +Y up,
// +Z forward; Cesium converts them to the FLU body frame (+X forward, +Y left,
// +Z up), so a node at glTF (x > 0, z > 0) sits front-left on the airframe.
// Rotor nodes in physics motor order: front right, back right, back left,
// front left. Their names state the spin direction and match SPIN in
// quadrotor.mjs (+1 clockwise seen from above); tests check both against the
// GLB node positions.
export const ROTOR_NODES = Object.freeze([
  "PropCounterClockwiseFront_low",
  "PropClockwiseBack_low",
  "PropCounterClockwiseBack_low",
  "PropClockwiseFront_low",
]);
// glTF scene root; a translation here moves the whole model in glTF space.
export const MODEL_ROOT_NODE = "Scene";
// Left-right motor spacing and lowest base-plate point, in glTF units.
export const MODEL_MOTOR_SPACING = 3.4;
export const MODEL_BASE_Y = -0.08;

// Scale so the model's lateral motor spacing equals the airframe's. An X quad
// with arm length L has adjacent motors 2L/√2 apart.
export function modelScale(armLength) {
  return (2 * armLength) / Math.SQRT2 / MODEL_MOTOR_SPACING;
}
// Vertical root translation (glTF units) placing the base plate landingOffset
// metres below the CG, so the airframe rests on the pad instead of hovering.
export function modelGroundOffset(armLength, landingOffset) {
  return -landingOffset / modelScale(armLength) - MODEL_BASE_Y;
}
