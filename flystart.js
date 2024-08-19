import { viewer, entity } from "./dronefocus.js";

document.getElementById("flyButton").addEventListener("click", function () {
  const selectedAltitude = parseFloat(document.getElementById("altitudeSelect").value);
  startDroneFlight(selectedAltitude);
});

function startDroneFlight(altitude) {
  viewer.clock.shouldAnimate = true;
  entity.model.runAnimations = true;

  const startPosition = entity.position.getValue(Cesium.JulianDate.now());
  const endPosition = Cesium.Cartesian3.fromDegrees(40.293544977054204, 40.61760805958316, altitude);

  const start = Cesium.JulianDate.now();
  const stop = Cesium.JulianDate.addSeconds(start, 5, new Cesium.JulianDate());

  viewer.clock.startTime = start.clone();
  viewer.clock.stopTime = stop.clone();
  viewer.clock.currentTime = start.clone();
  viewer.clock.multiplier = 1;

  const flightPath = new Cesium.SampledPositionProperty();
  flightPath.addSample(start, startPosition);
  flightPath.addSample(stop, endPosition);
  entity.position = flightPath;
  viewer.trackedEntity = entity;

  viewer.scene.preUpdate.addEventListener(updateDrone);

  function updateDrone() {
    if (Cesium.JulianDate.compare(viewer.clock.currentTime, stop) >= 0) {
      viewer.scene.preUpdate.removeEventListener(updateDrone);
      entity.position = new Cesium.ConstantPositionProperty(endPosition);
      entity.model.runAnimations = altitude !== 1;
      localStorage.setItem("endPosition", JSON.stringify(endPosition));
    }
  }
}
