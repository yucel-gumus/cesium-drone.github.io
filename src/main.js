import { CESIUM_ION_TOKEN } from './config.js';

Cesium.Ion.defaultAccessToken = CESIUM_ION_TOKEN;
import DroneUIController from './ui-controller.js';
import './style.css';

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
    start: Cesium.JulianDate.fromDate(new Date(2025, 2, 25, 16)),
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
        velocity: new Cesium.VelocityVectorProperty(sampledPosition),

        orientation: orientation,
        model: {
            uri: "./CesiumDrone.glb",
            minimumPixelSize: 256,
            maximumScale: 10,
            scale: 2.0,
        },
        path: {
            show: false,
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
    if (!window.droneUI) {
        window.droneUI = new DroneUIController();
    }
    viewer.scene.camera.flyTo({
        destination: Cesium.Cartesian3.fromDegrees(40.2925729751587, 40.61789421298509, 1500),
        complete: function () {
            setTimeout(function () {
                viewer.scene.camera.flyTo({
                    destination: Cesium.Cartesian3.fromDegrees(40.2925729751587, 40.61789421298509, 1200),
                    duration: 3.0,
                    complete: function () {
                        viewer.trackedEntity = entity;
                        window.droneUI.updateConnectionStatus('Bağlantı Kuruldu');

                    }
                });
            }, 1000);
        }
    });




 document.getElementById('startButton').addEventListener('click', function () {
        window.droneUI.startUpdates();
        window.droneUI.updateConnectionStatus('Aktif');
        entity.path.show = true;
        entity.viewFrom = new Cesium.Cartesian3(-10, 0, -10);
        viewer.trackedEntity = undefined;
        viewer.trackedEntity = entity;
        viewer.clock.startTime = mainJsAnimationConfig.start.clone();
        viewer.clock.stopTime = mainJsAnimationConfig.stop.clone();
        viewer.clock.currentTime = mainJsAnimationConfig.start.clone();
        viewer.clock.multiplier = 1.0;
        viewer.clock.clockRange = Cesium.ClockRange.CLAMPED;
        viewer.clock.shouldAnimate = true;

        // Linear interpolation (lerp) fonksiyonu
        function lerp(start, end, t) {
            return start * (1 - t) + end * t;
        }

        // Tek bir onTick listener kullanıyoruz
        viewer.clock.onTick.addEventListener(function (clock) {
            const currentTime = Cesium.JulianDate.secondsDifference(clock.currentTime, mainJsAnimationConfig.start);
            const totalDuration = mainJsAnimationConfig.duration;
            const transitionDuration = totalDuration * 0.1;

            let currentSpeed;
            if (currentTime < totalDuration * 0.3) {
                if (currentTime < transitionDuration) {
                    currentSpeed = lerp(0, 4, currentTime / transitionDuration);
                } else {
                    currentSpeed = 4;
                }
                viewer.clock.multiplier = 1.0;
            }
            else if (currentTime < totalDuration * 0.7) {
                const accelerationTime = (currentTime - totalDuration * 0.3) / transitionDuration;
                if (accelerationTime < 1) {
                    currentSpeed = lerp(4, 12, accelerationTime);
                } else {
                    currentSpeed = 12;
                }
                viewer.clock.multiplier = 3.0;
            }
            else if (currentTime < totalDuration) {
                const decelerationTime = (currentTime - totalDuration * 0.7) / transitionDuration;
                if (currentTime > totalDuration - transitionDuration) {
                    const finalSlowdown = (currentTime - (totalDuration - transitionDuration)) / transitionDuration;
                    currentSpeed = lerp(4, 0, finalSlowdown);
                } else if (decelerationTime < 1) {
                    currentSpeed = lerp(12, 4, decelerationTime);
                } else {
                    currentSpeed = 4;
                }
                viewer.clock.multiplier = 1.0;
            }
            else {
                currentSpeed = 0;
                viewer.clock.shouldAnimate = false;
                window.droneUI.updateConnectionStatus('Bağlantı Kesildi');
                window.droneUI.updateAltitude(0);
                window.droneUI.updateBatteryStatus(100);
                window.droneUI.stopUpdates();
            }

            window.droneUI.updateSpeed(Math.round(currentSpeed));
        });

        entity.position.setInterpolationOptions({
            interpolationDegree: 5,
            interpolationAlgorithm: Cesium.LagrangePolynomialApproximation,
        });
        entity.trackingReferenceFrame = Cesium.TrackingReferenceFrame.VELOCITY;
    });




}