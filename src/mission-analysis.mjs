import { STRIDE, PHASE_NAMES } from "./quadrotor.mjs";
import { capturePhotos, coverageSummary } from "./payload.mjs";
import { GNSS_MODES } from "./navigation.mjs";

// ENU point of the mission frame to WGS84 longitude/latitude (degrees) and
// ellipsoidal height (m).
export function toGeodetic(C, mission, p) {
  const cartesian = C.Matrix4.multiplyByPoint(
    mission.frame,
    new C.Cartesian3(p[0], p[1], p[2]),
    new C.Cartesian3(),
  );
  const g = C.Cartographic.fromCartesian(cartesian);
  return {
    longitude: C.Math.toDegrees(g.longitude),
    latitude: C.Math.toDegrees(g.latitude),
    height: g.height,
  };
}

// Every flown position inside the route box (the mission geofence).
function geofence(C, mission) {
  const b = mission.bounds,
    sim = mission.simulation;
  let inside = true,
    worst = null;
  for (let i = 0; i < sim.rows; i += 15) {
    const r = i * STRIDE,
      g = toGeodetic(C, mission, [
        sim.data[r + 1],
        sim.data[r + 2],
        sim.data[r + 3],
      ]);
    if (
      g.longitude < b.west ||
      g.longitude > b.east ||
      g.latitude < b.south ||
      g.latitude > b.north
    ) {
      inside = false;
      worst = g;
      break;
    }
  }
  return { inside, worst };
}

const percent = (x) => `%${Math.round(x * 100)}`;
const metres = (x, digits = 1) => `${x.toFixed(digits)} m`;

// Preflight checklist in the order an operator walks it. Blocking items
// keep the mission from being flown; the others are advisories.
export function preflightChecks(C, mission, capture, coverage) {
  const s = mission.settings,
    sim = mission.simulation,
    m = sim.metrics,
    battery = sim.battery,
    fence = geofence(C, mission),
    minVoltage = battery.cells * battery.minCellVoltage,
    gnss = GNSS_MODES[sim.gnss];
  const checks = [
    {
      id: "terrain",
      label: "Arazi modeli",
      detail: `${mission.samples.length} örnek · ±${s.corridorHalfWidth} m koridor`,
      ok: mission.samples.every((p) => Number.isFinite(p.height)),
      blocking: true,
    },
    {
      id: "clearance",
      label: "Asgari arazi açıklığı",
      detail: `${metres(mission.minClearance)} · hedef ${s.agl} m, tolerans ${s.clearanceTolerance} m`,
      ok: mission.minClearance >= s.agl - s.clearanceTolerance,
      blocking: true,
    },
    {
      id: "ceiling",
      label: "Yasal irtifa tavanı",
      detail: `en yüksek ${metres(mission.maxAgl)} AGL · sınır ${s.maxAgl} m`,
      ok: mission.maxAgl <= s.maxAgl,
      blocking: true,
    },
    {
      id: "geofence",
      label: "Coğrafi sınır",
      detail: fence.inside
        ? "uçulan iz rota kutusunun içinde"
        : `sınır dışı: ${fence.worst.latitude.toFixed(5)}°, ${fence.worst.longitude.toFixed(5)}°`,
      ok: fence.inside,
      blocking: true,
    },
    {
      id: "corridor",
      label: "Koridor sapması",
      detail: `${metres(mission.maxCorridorDeviation)} · sınır ${s.corridorHalfWidth - 2} m`,
      ok: mission.maxCorridorDeviation <= s.corridorHalfWidth - 2,
      blocking: true,
    },
    {
      id: "reserve",
      label: "İniş batarya rezervi",
      detail: `inişte ${percent(m.finalSoc)} · asgari ${percent(battery.reserve)}`,
      ok: m.finalSoc >= battery.reserve,
      blocking: true,
    },
    {
      id: "voltage",
      label: "Pil gerilimi",
      detail: `en düşük ${m.minVoltage.toFixed(2)} V · sınır ${minVoltage.toFixed(1)} V`,
      ok: m.minVoltage >= minVoltage && m.batteryLimitedSeconds === 0,
      blocking: true,
    },
    {
      id: "authority",
      label: "Motor yetkisi",
      detail: `doygunluk ${m.saturationSeconds.toFixed(1)} s · en yüksek ${Math.round((m.maxRotorSpeed * 60) / (2 * Math.PI))} rpm`,
      ok: m.saturationSeconds <= 0.5,
      blocking: true,
    },
    {
      id: "tracking",
      label: "Rota takibi",
      detail: `en büyük sapma ${metres(m.maxTrackingError, 2)}`,
      ok: m.maxTrackingError <= 3,
      blocking: false,
    },
    {
      id: "navigation",
      label: "Navigasyon",
      detail: gnss
        ? `${gnss.label} · ${gnss.satellites} uydu · kestirim hatası ≤ ${metres(m.maxNavigationError, 2)}`
        : "ideal durum bilgisi",
      ok: m.maxNavigationError <= 3,
      blocking: false,
    },
    {
      id: "coverage",
      label: "Fotoğraf bindirmesi",
      detail: `${coverage.count} kare · asgari ${percent(coverage.minOverlap)} · hedef ${percent(capture.overlap)}`,
      ok: coverage.count > 0 && coverage.minOverlap >= capture.overlap - 0.05,
      blocking: false,
    },
    {
      id: "landing",
      label: "İniş doğruluğu",
      detail: `${metres(m.landingError, 2)} · temas ${m.contactSpeed.toFixed(2)} m/s`,
      ok: m.landingError <= (sim.gnss === "standard" ? 3 : 0.5),
      blocking: false,
    },
  ];
  return checks;
}

// Mission timeline reconstructed from the flight record.
export function missionEvents(mission, capture) {
  const sim = mission.simulation,
    m = sim.metrics,
    events = [];
  const add = (time, label, detail = "") =>
    events.push({ time, label, detail });
  add(0, "Motorlar hazırlanıyor", "ön kontroller tamam, rotorlar rölantide");
  let lastPhase = -1,
    halfBattery = false,
    contact = false;
  for (let i = 0; i < sim.rows; i++) {
    const r = i * STRIDE,
      t = sim.data[r],
      phase = sim.data[r + 30],
      name = PHASE_NAMES[phase];
    if (phase !== lastPhase) {
      if (name === "takeoff") add(t, "Kalkış", "dikey tırmanış");
      if (name === "survey")
        add(t, "Görev irtifası", `${metres(sim.data[r + 3])} · keşif başladı`);
      if (name === "return") add(t, "Dönüş bacağı", "eve dönüş rotası");
      if (name === "settle") add(t, "Ev noktası üzerinde", "konum tutma");
      if (name === "landing") add(t, "İniş alçalması", "3 m sürünme irtifasına");
      if (name === "touchdown")
        add(t, "Son yaklaşma", `${mission.settings.landingSpeed} m/s sabit iniş hızı`);
      lastPhase = phase;
    }
    if (!halfBattery && sim.data[r + 36] <= 0.5) {
      halfBattery = true;
      add(t, "Batarya %50", `${sim.data[r + 37].toFixed(1)} V`);
    }
    if (!contact && phase >= 6 && sim.data[r + 3] <= 0.002) {
      contact = true;
      add(t, "Zemin teması", `${m.contactSpeed.toFixed(2)} m/s`);
    }
  }
  if (capture.photos.length) {
    add(capture.photos[0].time, "İlk fotoğraf", `GSD ${(capture.photos[0].gsd * 100).toFixed(2)} cm`);
    add(
      capture.photos.at(-1).time,
      "Son fotoğraf",
      `${capture.photos.length} kare`,
    );
  }
  if (m.disarmTime !== null)
    add(m.disarmTime, "Motorlar durdu", "iniş dedektörü");
  add(sim.duration, "Görev tamamlandı", `inişte batarya ${percent(m.finalSoc)}`);
  return events.sort((a, b) => a.time - b.time);
}

export function analyzeMission(C, mission, payloadSettings = {}) {
  const capture = capturePhotos(mission, payloadSettings),
    coverage = coverageSummary(capture),
    checks = preflightChecks(C, mission, capture, coverage),
    events = missionEvents(mission, capture);
  return {
    capture,
    coverage,
    checks,
    events,
    blocked: checks.some((c) => c.blocking && !c.ok),
  };
}
