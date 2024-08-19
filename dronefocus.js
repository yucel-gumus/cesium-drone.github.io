
const viewer = new Cesium.Viewer("cesiumContainer", {
  shouldAnimate: true,
  infoBox: false,
  terrain: Cesium.Terrain.fromWorldTerrain(),
  selectionIndicator: false,
  shadows: true,
  baseLayerPicker: false,
  navigationHelpButton: false,
  animation: false,
  timeline: true,
  fullscreenButton: false,
  geocoder: false,
  homeButton: false,
  sceneModePicker: false,
  projectionPicker: false,
});

viewer.cesiumWidget.creditContainer.style.display = "none";
viewer.scene.fog.enabled = false;
viewer.scene.globe.enableLighting = true;
viewer.scene.globe.depthTestAgainstTerrain = true;

viewer.scene.screenSpaceCameraController.minimumZoomDistance = 10;
viewer.scene.screenSpaceCameraController.maximumZoomDistance = 1000;

const initialPosition = Cesium.Cartesian3.fromDegrees(40.293544977054204, 40.61760805958316, 1);
const initialHeading = Cesium.Math.toRadians(135);
const hpr = new Cesium.HeadingPitchRoll(initialHeading, 0, 0);
const orientation = Cesium.Transforms.headingPitchRollQuaternion(initialPosition, hpr);

const entity = viewer.entities.add({
  name: "Drone",
  position: initialPosition,
  orientation: orientation,
  model: {
    uri: "./CesiumDrone.glb",
    heightReference: Cesium.HeightReference.RELATIVE_TO_GROUND,
    minimumPixelSize: 256,
    maximumScale: 10,
    scale: 2.0,
    runAnimations: false,
  },
});

viewer.scene.camera.flyTo({
  destination: Cesium.Cartesian3.fromDegrees(40.293544977054204, 40.61760805958316, 101),
  orientation: {
    heading: initialHeading,
    pitch: Cesium.Math.toRadians(-20),
    roll: 0,
  },
  duration: 5.0,
  complete: function () {
    viewer.trackedEntity = entity;
  },
});

export { viewer, entity };
