// Cesium 1.120 predates CallbackPositionProperty. Supply its PositionProperty
// interface explicitly so the renderer and tracked camera use the exact path.
export function createPositionProperty(C, callback) {
  const property = new C.CallbackProperty(callback, false);
  Object.defineProperty(property, "referenceFrame", {
    value: C.ReferenceFrame.FIXED,
  });
  property.getValueInReferenceFrame = (time, referenceFrame, result) =>
    C.PositionProperty.convertToReferenceFrame(
      time,
      callback(time),
      C.ReferenceFrame.FIXED,
      referenceFrame,
      result,
    );
  return property;
}
