Cesium.Ion.defaultAccessToken = "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJqdGkiOiI1MmI2MjkzYS1mMmE5LTRkODEtYTZiMC00YTZiMDgzZWU1YjAiLCJpZCI6ODYyNTEsImlhdCI6MTc0NjMxMjgxM30.OK5gINXOFgBwFNachWVbWWwYDpFeJXxNMS2_Ot6nMRg";

window.viewer = new Cesium.Viewer("cesiumContainer", {
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

const viewer = window.viewer;

viewer.cesiumWidget.creditContainer.style.display = "none";
viewer.scene.fog.enabled = false;
viewer.scene.globe.enableLighting = true;
viewer.scene.globe.depthTestAgainstTerrain = true;

const mainJsAnimationConfig = {
    start: Cesium.JulianDate.fromDate(new Date(2015, 2, 25, 16)),
    duration: 40,
    get stop() { return Cesium.JulianDate.addSeconds(this.start, this.duration, new Cesium.JulianDate()); }
};

viewer.clock.startTime = mainJsAnimationConfig.start.clone();
viewer.clock.stopTime = mainJsAnimationConfig.stop.clone();
viewer.clock.currentTime = mainJsAnimationConfig.start.clone();
viewer.clock.multiplier = 2.0;
viewer.clock.clockRange = Cesium.ClockRange.CLAMPED;
viewer.clock.shouldAnimate = false;

viewer.timeline.zoomTo(mainJsAnimationConfig.start, mainJsAnimationConfig.stop);

import './style.css';

let points = [];
fetch('/points.json', {
    headers: {
        'Accept': 'application/json'
    }
})
    .then(response => {
        if (!response.ok) {
        }
        return response.json();
    })
    .then(data => {
        if (!data.dronePoints || !Array.isArray(data.dronePoints)) {
        }

        points = data.dronePoints.map((point, index) => {
            if (!point.longitude || !point.latitude || !point.height) {
            }
            const cartesian = Cesium.Cartesian3.fromDegrees(point.longitude, point.latitude, point.height);
            return cartesian;
        });

        if (points.length < 2) {
        }

        initializeDronePath();
    })
    .catch(error => {
    });

function initializeDronePath() {
    if (!points || points.length < 2) {
        return;
    }
    const firstTime = 1.0;
    const lastTime = 50.0;
    const delta = lastTime - firstTime;

    const numPoints = points.length;
    const times = [];
    for (let i = 0; i < numPoints; i++) {
        times.push(firstTime + (delta * i) / (numPoints - 1));
    }
    const sampledPosition = new Cesium.SampledPositionProperty();
    const numberOfSamples = points.length;

    for (let i = 0; i < numberOfSamples; i++) {
        const time = Cesium.JulianDate.addSeconds(
            mainJsAnimationConfig.start,
            (i * mainJsAnimationConfig.duration) / (numberOfSamples - 1),
            new Cesium.JulianDate()
        );
        sampledPosition.addSample(time, points[i]);
    }

    sampledPosition.setInterpolationOptions({
        interpolationDegree: 3,
        interpolationAlgorithm: Cesium.LinearApproximation
    });

    const orientation = new Cesium.VelocityOrientationProperty(sampledPosition);

    for (let i = 0; i < points.length; ++i) {
        viewer.entities.add({
            position: points[i],
        });
    }

    const entity = viewer.entities.add({
        availability: new Cesium.TimeIntervalCollection([
            new Cesium.TimeInterval({
                start: mainJsAnimationConfig.start,
                stop: mainJsAnimationConfig.stop,
            }),
        ]),
        position: sampledPosition,
        orientation: orientation,
        model: {
            uri: "./CesiumDrone.glb",
            minimumPixelSize: 256,
            maximumScale: 10,
            scale: 2.0,
        },
        path: {
            material: new Cesium.PolylineGlowMaterialProperty({
                glowPower: 0.1,
                color: Cesium.Color.RED,
            }),
            width: 10,
            resolution: 0.01,
            leadTime: 1,
            trailTime: 0.1,
        },

        viewFrom: new Cesium.Cartesian3(-10, 5, 5),
    });

    viewer.scene.camera.flyTo({
        destination: Cesium.Cartesian3.fromDegrees(40.2925729751587, 40.61789421298509, 1500),
        complete: function () {
            setTimeout(function () {
                viewer.scene.camera.flyTo({
                    destination: Cesium.Cartesian3.fromDegrees(40.2925729751587, 40.61789421298509, 1200),
                    duration: 3.0,
                    complete: function () {
                        viewer.trackedEntity = entity;
                    }
                });
            }, 1000);
        }
    });
    viewer.clock.onTick.addEventListener(function (clock) {
        if (Cesium.JulianDate.equals(clock.startTime, mainJsAnimationConfig.start) &&
            Cesium.JulianDate.equals(clock.stopTime, mainJsAnimationConfig.stop) &&
            Cesium.JulianDate.compare(clock.currentTime, mainJsAnimationConfig.stop) >= 0) {
            viewer.clock.shouldAnimate = false;
        }
    });
    document.getElementById('startButton').addEventListener('click', function () {

        entity.viewFrom = new Cesium.Cartesian3(-10, 0, -10);
        viewer.trackedEntity = undefined;
        viewer.trackedEntity = entity;
        viewer.clock.startTime = mainJsAnimationConfig.start.clone();
        viewer.clock.stopTime = mainJsAnimationConfig.stop.clone();
        viewer.clock.currentTime = mainJsAnimationConfig.start.clone();
        viewer.clock.multiplier = 2.0;
        viewer.clock.clockRange = Cesium.ClockRange.CLAMPED;
        viewer.clock.shouldAnimate = true;
        entity.position.setInterpolationOptions({
            interpolationDegree: 5,
            interpolationAlgorithm: Cesium.LagrangePolynomialApproximation,
        });
        entity.trackingReferenceFrame = Cesium.TrackingReferenceFrame.VELOCITY;
    });
}