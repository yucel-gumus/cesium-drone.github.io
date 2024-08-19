import { viewer,entity } from "./dronefocus.js";

class DroneRouteManager {
  constructor(droneEntity) {
    this.droneEntity = droneEntity;
    this.circularFlightProperty = null;
    this.initEventListeners();
  }

  initEventListeners() {
    document.getElementById("createRota").addEventListener("click", async () => {
      const retrievedEndPositionData = JSON.parse(localStorage.getItem("endPosition"));
      this.removeAllEntitiesExceptFirst();
      await this.createRota(retrievedEndPositionData);
    });

    document.getElementById("followRota").addEventListener("click", () => {
      this.followRota();
    });
  }

  removeAllEntitiesExceptFirst() {
    const entities = viewer.entities.values;
    for (let i = entities.length - 1; i >= 1; i--) {
      viewer.entities.remove(entities[i]);
    }
  }

  async createRota(endPosition) {
    const cartesian = new Cesium.Cartesian3(endPosition.x, endPosition.y, endPosition.z);
    const cartographic = Cesium.Cartographic.fromCartesian(cartesian);
    const longitude = Cesium.Math.toDegrees(cartographic.longitude);
    const latitude = Cesium.Math.toDegrees(cartographic.latitude);
    await this.computeCircularFlight(longitude, latitude, 0.03, cartesian);
  }

  async computeCircularFlight(lon, lat, radius, endCartesian) {
    const start = Cesium.JulianDate.now();
    const stop = Cesium.JulianDate.addSeconds(start, 360, new Cesium.JulianDate());

    this.setupClock(start, stop);

    const positions = this.generateCircularPositions(lon, lat, radius, endCartesian);
    const clampedPositions = await this.clampPositionsToTerrain(positions);
    
    this.createCircularFlightProperty(clampedPositions, start, stop);
    this.addRouteVisualization(clampedPositions);
  }

  setupClock(start, stop) {
    viewer.clock.startTime = start.clone();
    viewer.clock.stopTime = stop.clone();
    viewer.clock.currentTime = start.clone();
    viewer.clock.multiplier = 1;
  }

  generateCircularPositions(lon, lat, radius, endCartesian) {
    const count = 7; 
    const circularPositions = Array.from({ length: count }, (_, i) => {
      const angle = Cesium.Math.toRadians((i * 360) / count);
      return Cesium.Cartographic.fromDegrees(
        lon + radius * Math.cos(angle),
        lat + radius * Math.sin(angle)
      );
    });

    const startCartographic = Cesium.Cartographic.fromCartesian(endCartesian);
    return [startCartographic, ...circularPositions, startCartographic];
  }

  async clampPositionsToTerrain(positions) {
    const clampedPositions = await Cesium.sampleTerrainMostDetailed(viewer.terrainProvider, positions);
    return clampedPositions.map(p => 
      Cesium.Cartesian3.fromRadians(p.longitude, p.latitude, p.height + 1000)
    );
  }

  createCircularFlightProperty(clampedPositions, start, stop) {
    this.circularFlightProperty = new Cesium.SampledPositionProperty();
    const duration = Cesium.JulianDate.secondsDifference(stop, start);
    
    clampedPositions.forEach((position, i) => {
      const time = Cesium.JulianDate.addSeconds(start, (i / (clampedPositions.length - 1)) * duration, new Cesium.JulianDate());
      this.circularFlightProperty.addSample(time, position);
    });
  }

  addRouteVisualization(clampedPositions) {
    clampedPositions.forEach((position, i) => {
      const color = (i === 0 || i === clampedPositions.length - 1) ? Cesium.Color.RED : Cesium.Color.YELLOW;
      this.addPointEntity(position, color, i);
    });

    this.addPathEntity(clampedPositions);
  }

  addPointEntity(position, color, index) {
    viewer.entities.add({
      id: `point_${index}`,
      position: position,
      point: {
        pixelSize: 8,
        color: Cesium.Color.TRANSPARENT,
        outlineColor: color,
        outlineWidth: 3,
      },
    });
  }

  addPathEntity(positions) {
    viewer.entities.add({
      id: "path_",
      polyline: {
        positions: positions,
        width: 5,
        material: new Cesium.PolylineGlowMaterialProperty({
          glowPower: 0.2,
          color: Cesium.Color.YELLOW,
        }),
        clampToGround: false,
      },
    });
  }

  followRota() {
    if (!this.circularFlightProperty) {
      console.log("Rota oluşturulmamış. Önce rotayı oluşturun.");
      return;
    }
    
    const start = viewer.clock.startTime;
    const stop = viewer.clock.stopTime;
  
    this.setupClockForAnimation(start, stop);
  
    const entity = this.droneEntity;
    entity.position = this.circularFlightProperty;
    entity.orientation = new Cesium.VelocityOrientationProperty(this.circularFlightProperty);

    entity.path = {
      resolution: 1,
      material: new Cesium.PolylineGlowMaterialProperty({
        glowPower: 0.2,
        color: Cesium.Color.BLACK,
      }),
      width: 10,
    };
  
    viewer.trackedEntity = entity;
  
    entity.position.setInterpolationOptions({
      interpolationDegree: 2,
      interpolationAlgorithm: Cesium.HermitePolynomialApproximation,
    });
  
    this.animateDrone(entity, stop);
  }

  setupClockForAnimation(start, stop) {
    viewer.clock.startTime = start.clone();
    viewer.clock.stopTime = stop.clone();
    viewer.clock.currentTime = start.clone();
    viewer.clock.multiplier = 10;
    viewer.timeline.zoomTo(start, stop);
  }

  animateDrone(entity, stop) {
    let isFlightComplete = false;
    let finalPosition;

    viewer.clock.onTick.addEventListener(() => {
      const currentTime = viewer.clock.currentTime;
      const dronePosition = entity.position.getValue(currentTime);

      if (dronePosition) {
        finalPosition = dronePosition;
      }

      if (Cesium.JulianDate.compare(currentTime, stop) >= 0 && !isFlightComplete) {
        this.completeFlight(entity, finalPosition);
        isFlightComplete = true;
      }
    });
  }

  completeFlight(entity, finalPosition) {
    viewer.clock.shouldAnimate = true;
    entity.model.runAnimations = true;

    if (finalPosition) {
      entity.position = new Cesium.ConstantPositionProperty(finalPosition);
      const hpr = new Cesium.HeadingPitchRoll(0, 0, 0);
      const orientation = Cesium.Transforms.headingPitchRollQuaternion(finalPosition, hpr);
      entity.orientation = new Cesium.ConstantProperty(orientation);
    }
  }
}

const routeManager = new DroneRouteManager(entity);
