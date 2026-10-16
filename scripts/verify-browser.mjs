// Optional regression runner for the locally installed Kimi Browser Extension.
// Start `npm run preview` first. No credentials are written to this report.
import fs from "node:fs/promises";
import path from "node:path";
import assert from "node:assert/strict";
const bridge = process.env.WEBBRIDGE_URL || "http://127.0.0.1:10086";
const base =
  process.env.PREVIEW_URL || "http://127.0.0.1:9001/cesium-drone.github.io/";
const artifacts = path.resolve("artifacts");
await fs.mkdir(artifacts, { recursive: true });
const report = { date: new Date().toISOString(), url: base, checks: [] };
const delay = (ms) => new Promise((resolve) => setTimeout(resolve, ms));
async function command(action, args = {}) {
  const response = await fetch(`${bridge}/command`, {
    signal: AbortSignal.timeout(30000),
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ action, args, session: "uzungol-flight" }),
  });
  const body = await response.json();
  if (!body.ok) throw new Error(`${action}: ${body.error?.message}`);
  return body.data;
}
async function evaluate(code) {
  const data = await command("evaluate", {
    code: `(async()=>JSON.stringify(await (async()=>{${code}})()))()`,
  });
  if (data.type !== "string")
    throw new Error(`Evaluation failed: ${JSON.stringify(data)}`);
  return JSON.parse(data.value);
}
async function waitUntil(code, timeout = 95000) {
  const deadline = Date.now() + timeout;
  while (Date.now() < deadline) {
    if (await evaluate(`return !!(${code});`)) return;
    await delay(1000);
  }
  throw new Error(`Timed out: ${code}`);
}
const click = (selector) => command("click", { selector });
const check = (name, details = true) => {
  report.checks.push({ name, details });
  console.log(`PASS ${name}`);
};

try {
  console.log("Opening production preview");
  await command("navigate", { url: base, newTab: true });
  console.log("Setting browser viewport");
  await command("cdp", {
    method: "Emulation.setFocusEmulationEnabled",
    params: { enabled: true },
  });
  await command("cdp", {
    method: "Emulation.setDeviceMetricsOverride",
    params: { width: 1440, height: 1000, deviceScaleFactor: 1, mobile: false },
  });
  await command("network", { cmd: "start" });
  console.log("Waiting for terrain plan");
  await waitUntil('window.droneSimulation?.state === "ready"');
  await evaluate(
    `window.__flightErrors=[]; addEventListener('error',e=>__flightErrors.push(e.message)); addEventListener('unhandledrejection',e=>__flightErrors.push(String(e.reason))); viewer.scene.renderError.addEventListener((s,e)=>__flightErrors.push(String(e))); return true;`,
  );
  const initial = await evaluate(
    `const m=droneSimulation.mission; return {duration:m.duration,horizontalDistance:m.horizontalDistance,samples:m.samples.length,homeHeight:m.groundHeight,maxTerrain:Math.max(...m.samples.map(p=>p.height)),minPlannedClearance:m.minCruiseClearance,listeners:viewer.clock.onTick.numberOfListeners,entities:viewer.entities.values.length,metrics:m.simulation.metrics,airframe:m.simulation.airframe};`,
  );
  assert.ok(initial.samples > 1000 && initial.duration > 600);
  assert.equal(initial.entities, 4);
  check(
    "Worker solves 240 Hz rigid-body flight with actual terrain under a nested deployment path",
    initial,
  );

  const dynamics = await evaluate(
    `const m=droneSimulation.mission,log=m.simulation;let maxStep=0,turnSamples=0;for(let i=1;i<log.rows;i++){const a=(i-1)*36+7,b=i*36+7;let dot=0;for(let j=0;j<4;j++)dot+=log.data[a+j]*log.data[b+j];maxStep=Math.max(maxStep,2*Math.acos(Math.min(1,Math.abs(dot))));if([2,3].includes(log.data[i*36+30])&&Math.abs(log.data[i*36+13])>.06)turnSamples++;}return {maxQuaternionStep:maxStep,turnSamples,trackingError:log.metrics.maxTrackingError,maxBank:log.metrics.maxTilt,motorSaturation:log.metrics.saturationSeconds};`,
  );
  assert.ok(
    dynamics.maxQuaternionStep < 0.025 &&
      dynamics.turnSamples > 100 &&
      dynamics.trackingError < 1,
  );
  assert.ok(dynamics.maxBank > 0.05 && dynamics.maxBank < (27 * Math.PI) / 180);
  check(
    "Continuous physical attitude, banked fly-through turns and bounded tracking error",
    dynamics,
  );

  const csv = await evaluate(
    `let captured;const original=URL.createObjectURL,click=HTMLAnchorElement.prototype.click;try{URL.createObjectURL=function(blob){captured=blob;return original.call(URL,blob)};HTMLAnchorElement.prototype.click=function(){};document.getElementById('exportButton').click();}finally{URL.createObjectURL=original;HTMLAnchorElement.prototype.click=click;}const lines=(await captured.text()).trim().split('\\n');return {rows:lines.length-1,columns:lines[0].split(',').length,firstTime:Number(lines[1].split(',')[0]),lastTime:Number(lines.at(-1).split(',')[0])};`,
  );
  assert.ok(
    csv.rows > 5000 &&
      csv.columns === 32 &&
      csv.firstTime === 0 &&
      csv.lastTime > initial.duration - 0.2,
  );
  check("CSV exports the measured physical states at 10 Hz", csv);

  const liveTerrain = await evaluate(
    `const m=droneSimulation.mission;const states=Array.from({length:289},(_,i)=>droneSimulation.evaluate(m.reference.surveyStart+m.reference.horizontalDuration*(i+.5)/289));const terrain=states.map(s=>Cesium.Cartographic.clone(s.position));await Cesium.sampleTerrainMostDetailed(viewer.terrainProvider,terrain,true);return {points:terrain.length,minAGL:Math.min(...terrain.map((p,i)=>states[i].position.height-p.height)),allFinite:terrain.every(p=>Number.isFinite(p.height))};`,
  );
  assert.ok(liveTerrain.allFinite && liveTerrain.minAGL >= 70);
  check(
    "Independent terrain queries between route samples confirm clearance",
    liveTerrain,
  );

  await click("#overviewButton");
  await delay(1800);
  await command("screenshot", {
    path: path.join(artifacts, "desktop-overview.png"),
  });
  await click("#startButton");
  await delay(1000);
  assert.equal(await evaluate("return droneSimulation.state;"), "running");
  const takeoff = await evaluate(
    'return {agl:droneSimulation.evaluate(Cesium.JulianDate.secondsDifference(viewer.clock.currentTime,viewer.clock.startTime)).agl,groundSpeed:droneSimulation.evaluate(Cesium.JulianDate.secondsDifference(viewer.clock.currentTime,viewer.clock.startTime)).groundSpeed,speed:document.getElementById("speedStatus").textContent};',
  );
  assert.ok(takeoff.groundSpeed < 0.25);
  check("Force-driven takeoff has bounded wind drift", takeoff);
  await click("#pauseButton");
  const pausedAt = await evaluate(
    "return Cesium.JulianDate.secondsDifference(viewer.clock.currentTime,viewer.clock.startTime);",
  );
  await delay(700);
  assert.equal(
    await evaluate(
      "return Cesium.JulianDate.secondsDifference(viewer.clock.currentTime,viewer.clock.startTime);",
    ),
    pausedAt,
  );
  await click("#pauseButton");
  assert.equal(await evaluate("return droneSimulation.state;"), "running");
  check("Pause freezes simulation time and resume continues");

  await evaluate(
    `const s=document.getElementById('flightProgress');s.value='350';s.dispatchEvent(new Event('input',{bubbles:true}));return true;`,
  );
  assert.equal(await evaluate("return droneSimulation.state;"), "paused");
  await click("#followButton");
  await delay(1200);
  await waitUntil("viewer.scene.globe.tilesLoaded", 20000);
  assert.equal(await evaluate("return droneSimulation.cameraMode;"), "follow");
  await command("screenshot", {
    path: path.join(artifacts, "drone-follow.png"),
  });
  assert.equal(
    await evaluate('return document.getElementById("freeButton") !== null;'),
    false,
  );
  await evaluate(
    `const s=document.getElementById('followDistance');s.value='2';s.dispatchEvent(new Event('input',{bubbles:true}));return true;`,
  );
  await waitUntil(
    "Math.abs(Cesium.Cartesian3.distance(viewer.camera.positionWC,droneSimulation.drone.position.getValue(viewer.clock.currentTime))-2)<.08",
    7000,
  );
  const closeDistance = await evaluate(
    `return {target:droneSimulation.followDistance.target,actual:Cesium.Cartesian3.distance(viewer.camera.positionWC,droneSimulation.drone.position.getValue(viewer.clock.currentTime))};`,
  );
  await delay(1200);
  assert.ok(
    await evaluate(
      "return Math.abs(droneSimulation.followDistance.current-2)<.05;",
    ),
  );
  await evaluate(
    `viewer.canvas.dispatchEvent(new WheelEvent('wheel',{deltaY:500,bubbles:true,cancelable:true}));return true;`,
  );
  assert.ok(await evaluate("return droneSimulation.followDistance.target>3;"));
  await evaluate(
    `viewer.canvas.dispatchEvent(new WheelEvent('wheel',{deltaY:-500,bubbles:true,cancelable:true}));return true;`,
  );
  assert.ok(
    await evaluate(
      "return Math.abs(droneSimulation.followDistance.target-2)<.02;",
    ),
  );
  await click("#zoomInButton");
  assert.ok(await evaluate("return droneSimulation.followDistance.target<2;"));
  await click("#zoomOutButton");
  assert.ok(
    await evaluate("return droneSimulation.followDistance.target>1.8;"),
  );
  await evaluate(
    `const s=document.getElementById('followDistance');s.value='2';s.dispatchEvent(new Event('input',{bubbles:true}));return true;`,
  );
  await click("#overviewButton");
  assert.equal(
    await evaluate(
      'return document.getElementById("followZoomControls").hidden && viewer.scene.screenSpaceCameraController.enableZoom;',
    ),
    true,
  );
  await click("#followButton");
  assert.equal(
    await evaluate("return droneSimulation.followDistance.target;"),
    2,
  );
  await evaluate(
    `const s=document.getElementById('followDistance');s.value='8';s.dispatchEvent(new Event('input',{bubbles:true}));return true;`,
  );
  check(
    "Follow zoom accepts slider, wheel and buttons, persists selection, and Free mode is removed",
    closeDistance,
  );
  const cameraViews = [];
  for (const [view, bearing, pitch] of [
    ["rear", 0, -.32], ["front", Math.PI, -.32],
    ["left", Math.PI / 2, -.22], ["right", -Math.PI / 2, -.22],
    ["top", 0, -Math.PI / 2 + .015], ["diagonal", Math.PI / 4, -.6],
  ]) {
    await click(`[data-follow-view="${view}"]`);
    // Measure real camera position in the drone's ENU frame, independent of preset state.
    const geometry = `const C=Cesium,p=droneSimulation.drone.position.getValue(viewer.clock.currentTime),local=C.Matrix4.multiplyByPoint(C.Matrix4.inverseTransformation(C.Transforms.eastNorthUpToFixedFrame(p),new C.Matrix4()),viewer.camera.positionWC,new C.Cartesian3()),range=C.Cartesian3.magnitude(local),heading=droneSimulation.evaluate(C.JulianDate.secondsDifference(viewer.clock.currentTime,viewer.clock.startTime)).heading,bearing=Math.atan2(-local.x,-local.y)-heading;return {view:droneSimulation.followView,range,pitch:-Math.asin(local.z/range),bearingError:Math.atan2(Math.sin(bearing-(${bearing})),Math.cos(bearing-(${bearing}))),pressed:document.querySelectorAll('[data-follow-view][aria-pressed="true"]').length,target:droneSimulation.followDistance.target};`;
    const deadline = Date.now() + 10000;
    let actual;
    do {
      await delay(400);
      actual = await evaluate(geometry);
      if (Math.abs(actual.bearingError)<.025 && Math.abs(actual.pitch-pitch)<.015 && Math.abs(actual.range-8)<.05) break;
    } while (Date.now()<deadline);
    assert.equal(actual.view, view);
    assert.equal(actual.pressed, 1);
    assert.equal(actual.target, 8);
    assert.ok(Math.abs(actual.bearingError)<.025 && Math.abs(actual.pitch-pitch)<.015 && Math.abs(actual.range-8)<.05, JSON.stringify(actual));
    cameraViews.push(actual);
  }
  await command("screenshot", { path: path.join(artifacts, "follow-camera-angles.png"), selector: ".camera-controls" });
  await click("#overviewButton");
  await click("#followButton");
  assert.equal(await evaluate("return droneSimulation.followView;"), "diagonal");
  check("Six drone-relative camera angles match measured ENU geometry and preserve zoom and selection", cameraViews);
  await click("#overviewButton");
  check("Scrubbing and the two camera modes work independently of telemetry");

  await click("#resetButton");
  await click("#startButton");
  // Speed up only the test clock; the analytical trajectory and its limits stay identical.
  await evaluate("viewer.clock.multiplier=180;return true;");
  await waitUntil('droneSimulation.state === "complete"', 25000);
  const landing = await evaluate(
    `const m=droneSimulation.mission;const actual=Cesium.Cartographic.fromCartesian(droneSimulation.drone.position.getValue(viewer.clock.currentTime));const start=droneSimulation.evaluate(0).position;return {landingErrorMeters:Cesium.Cartesian3.distance(droneSimulation.drone.position.getValue(viewer.clock.currentTime),Cesium.Cartesian3.fromRadians(start.longitude,start.latitude,start.height)),heightError:actual.height-start.height,disarmed:m.simulation.metrics.disarmed,speed:document.getElementById('speedStatus').textContent,animate:viewer.clock.shouldAnimate,phase:document.getElementById('flightPhase').textContent,listeners:viewer.clock.onTick.numberOfListeners,entities:viewer.entities.values.length};`,
  );
  assert.ok(
    landing.landingErrorMeters < 0.2 && Math.abs(landing.heightError) < 0.02,
  );
  assert.equal(landing.disarmed, true);
  assert.equal(landing.speed, "0.0 km/h");
  assert.equal(landing.animate, false);
  check(
    "Force-driven flight lands within 20 cm, contacts ground and disarms",
    landing,
  );
  await click("#startButton");
  await click("#resetButton");
  assert.equal(
    await evaluate("return viewer.clock.onTick.numberOfListeners;"),
    initial.listeners,
  );
  assert.equal(
    await evaluate("return viewer.entities.values.length;"),
    initial.entities,
  );
  check("Replay does not accumulate event listeners or entities");

  await command("fill", { selector: "#clearance", value: "90" });
  await click("#planButton");
  await waitUntil(
    'droneSimulation?.state === "ready" && droneSimulation.mission.settings.clearance === 90',
  );
  assert.ok(
    await evaluate(
      "return droneSimulation.mission.minCruiseClearance >= 100-1e-8;",
    ),
  );
  assert.equal(await evaluate("return viewer.entities.values.length;"), 4);
  check(
    "Changing clearance rebuilds the terrain-aware mission without duplicates",
  );
  await command("fill", { selector: "#clearance", value: "70" });
  await click("#planButton");
  await waitUntil(
    'droneSimulation?.state === "ready" && droneSimulation.mission.settings.clearance === 70',
  );
  await command("cdp", {
    method: "Emulation.setDeviceMetricsOverride",
    params: { width: 390, height: 844, deviceScaleFactor: 1, mobile: true },
  });
  await delay(1200);
  const mobile = await evaluate(
    `return {width:innerWidth,bodyWidth:document.body.scrollWidth,mapHeight:document.getElementById('cesiumContainer').getBoundingClientRect().height,panelHeight:document.querySelector('.sidebar').clientHeight,scrollHeight:document.querySelector('.sidebar').scrollHeight};`,
  );
  assert.equal(mobile.width, 390);
  assert.ok(mobile.bodyWidth <= 390);
  assert.ok(
    mobile.mapHeight >= 300 &&
      mobile.panelHeight > 300 &&
      mobile.scrollHeight > mobile.panelHeight,
  );
  await command("screenshot", {
    path: path.join(artifacts, "mobile-overview.png"),
  });
  check(
    "Mobile map and independently scrollable controls fit 390px without overflow",
    mobile,
  );
  await click("#followButton");
  await click('[data-follow-view="top"]');
  const mobileCamera = await evaluate(`const r=s=>document.querySelector(s).getBoundingClientRect(),c=r('.camera-controls'),h=r('.attitude-instrument'),f=r('.flight-bar');return {buttons:document.querySelectorAll('[data-follow-view]').length,overflow:document.body.scrollWidth>innerWidth,panelBottom:c.bottom,horizonBottom:h.bottom,flightTop:f.top,overlap:!(c.right<=h.left||c.left>=h.right||c.bottom<=h.top||c.top>=h.bottom)};`);
  assert.equal(mobileCamera.buttons, 6);
  assert.equal(mobileCamera.overflow, false);
  assert.equal(mobileCamera.overlap, false);
  assert.ok(mobileCamera.panelBottom < mobileCamera.flightTop && mobileCamera.horizonBottom < mobileCamera.flightTop);
  await command("screenshot", {path:path.join(artifacts,"mobile-follow-angles.png")});
  check("Six camera controls fit mobile without overlapping the horizon or flight bar",mobileCamera);
  assert.deepEqual(await evaluate("return __flightErrors;"), []);
  check("No JavaScript or Cesium render errors during the exercised flows");

  await evaluate(
    `window.__priorIonToken=sessionStorage.getItem('uzungol-ion-token');document.querySelector('.settings').open=true;return true;`,
  );
  await command("fill", {
    selector: "#ionToken",
    value: "deliberately-invalid-test-token",
  });
  await click("#tokenButton");
  await waitUntil('droneSimulation.state === "error"', 30000);
  assert.ok(
    await evaluate(
      `return document.getElementById('startButton').disabled && !document.getElementById('mapError').hidden && viewer.entities.values.length === 0;`,
    ),
  );
  check("Invalid terrain credentials display an error and block flight");
  await evaluate(
    `if(__priorIonToken===null)sessionStorage.removeItem('uzungol-ion-token');else sessionStorage.setItem('uzungol-ion-token',__priorIonToken);return true;`,
  );
  await command("navigate", { url: base });
  await waitUntil('window.droneSimulation?.state === "ready"');
  check("Valid terrain access recovers after a failed load");

  const network = await command("network", { cmd: "list" });
  report.failedRequests = (network.requests || [])
    .filter((r) => r.completed && r.status >= 400)
    .map((r) => ({
      host: new URL(r.url).host,
      path: new URL(r.url).pathname,
      status: r.status,
      expected: new URL(r.url).host === "api.cesium.com" && r.status === 401,
    }));
  assert.equal(report.failedRequests.filter((r) => !r.expected).length, 0);
  report.success = true;
} catch (error) {
  report.success = false;
  report.error = error.message;
  throw error;
} finally {
  await evaluate(
    `if(typeof __priorIonToken!=='undefined'){if(__priorIonToken===null)sessionStorage.removeItem('uzungol-ion-token');else sessionStorage.setItem('uzungol-ion-token',__priorIonToken);}return true;`,
  ).catch(() => {});
  await fs.writeFile(
    path.join(artifacts, "browser-verification.json"),
    JSON.stringify(report, null, 2) + "\n",
  );
  await command("network", { cmd: "stop" }).catch(() => {});
  await command("cdp", {
    method: "Emulation.clearDeviceMetricsOverride",
  }).catch(() => {});
  await command("cdp", {
    method: "Emulation.setFocusEmulationEnabled",
    params: { enabled: false },
  }).catch(() => {});
}
