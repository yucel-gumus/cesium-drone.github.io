const PHASES = {
  ready: "Kalkışa hazır",
  arming: "Motorlar hazırlanıyor",
  takeoff: "Dikey kalkış",
  survey: "Keşif uçuşu",
  return: "Kalkış noktasına dönüş",
  landing: "Kontrollü iniş",
  settle: "İniş öncesi konum tutma",
  touchdown: "Zemine yaklaşma",
  disarm: "Temas · motorlar duruyor",
  complete: "Görev tamamlandı",
};
export const formatTime = (seconds) => {
  const t = Math.max(0, Math.round(seconds));
  return `${String(Math.floor(t / 60)).padStart(2, "0")}:${String(t % 60).padStart(2, "0")}`;
};
export const formatDistance = (distance) =>
  distance >= 1000
    ? `${(distance / 1000).toFixed(2)} km`
    : `${Math.round(distance)} m`;

export default class DroneUIController {
  constructor(handlers) {
    this.elements = Object.fromEntries(
      [...document.querySelectorAll("[id]")].map((el) => [el.id, el]),
    );
    this.handlers = handlers;
    const e = this.elements;
    e.startButton.addEventListener("click", handlers.start);
    e.pauseButton.addEventListener("click", handlers.pause);
    e.resetButton.addEventListener("click", handlers.reset);
    e.exportButton.addEventListener("click", handlers.export);
    e.planButton.addEventListener("click", () => {
      if (
        e.clearance.reportValidity() &&
        e.massInput.reportValidity() &&
        e.windInput.reportValidity() &&
        e.seedInput.reportValidity()
      )
        handlers.plan(Number(e.clearance.value));
    });
    e.tokenButton.addEventListener("click", () =>
      handlers.token(e.ionToken.value.trim()),
    );
    e.flightProgress.addEventListener("input", () =>
      handlers.seek(Number(e.flightProgress.value) / 1000),
    );
    e.playbackSpeed.addEventListener("change", () =>
      handlers.rate(Number(e.playbackSpeed.value)),
    );
    ["overview", "follow"].forEach((mode) =>
      e[`${mode}Button`].addEventListener("click", () => handlers.camera(mode)),
    );
    document.querySelectorAll("[data-follow-view]").forEach((button) =>
      button.addEventListener("click", () =>
        handlers.followView(button.dataset.followView),
      ),
    );
    e.followDistance.addEventListener("input", () =>
      handlers.followDistance(Number(e.followDistance.value)),
    );
    e.zoomInButton.addEventListener("click", () =>
      handlers.followDistance(Number(e.followDistance.value) / 1.3),
    );
    e.zoomOutButton.addEventListener("click", () =>
      handlers.followDistance(Number(e.followDistance.value) * 1.3),
    );
  }

  message(text, error = false) {
    this.elements.message.textContent = text;
    this.elements.message.classList.toggle("error", error);
  }

  setState(state) {
    const e = this.elements;
    const busy = state === "loading";
    const available = ["ready", "running", "paused", "complete"].includes(
      state,
    );
    const active = state === "running" || state === "paused";
    e.startButton.disabled = !available || active;
    e.startButton.innerHTML = `<span aria-hidden="true">↗</span> ${state === "complete" ? "Yeniden uçur" : "Keşif uçuşunu başlat"}`;
    e.pauseButton.disabled = !active;
    e.pauseButton.textContent = state === "paused" ? "Devam et" : "Duraklat";
    e.resetButton.disabled = !available;
    e.flightProgress.disabled = !available;
    e.exportButton.disabled = !available;
    e.clearance.disabled = busy || active;
    e.massInput.disabled = busy || active;
    e.windInput.disabled = busy || active;
    e.seedInput.disabled = busy || active;
    e.turbulenceInput.disabled = busy || active;
    e.planButton.disabled = busy || active;
    e.tokenButton.disabled = busy || active;
    e.ionToken.disabled = busy || active;
    e.connectionStatus.textContent = {
      loading: "Hazırlanıyor",
      ready: "Hazır",
      running: "Uçuşta",
      paused: "Duraklatıldı",
      complete: "İniş tamam",
      error: "Veri gerekli",
    }[state];
    if (!available) {
      e.flightPhase.textContent = busy
        ? "Rota hazırlanıyor"
        : "Uçuş hazırlanamadı";
      e.terrainStatus.textContent = busy
        ? "ARAZİ ÖRNEKLENİYOR"
        : "ARAZİ DOĞRULANMADI";
    }
    this.state = state;
  }

  setMission(mission) {
    this.mission = mission;
    this.elements.routeDistance.textContent = formatDistance(
      mission.horizontalDistance,
    );
    this.elements.routeDuration.textContent = formatTime(mission.duration);
    this.elements.terrainStatus.textContent = `${mission.samples.length} ARAZİ ÖRNEĞİ · WGS84`;
    const s = mission.simulation,
      m = s.metrics,
      c = s.airframe;
    this.elements.physicsSummary.innerHTML = `<dl><dt>Çözüm</dt><dd>RK4 · ${c.physicsHz} Hz / kontrol ${c.controlHz} Hz</dd><dt>Kütle / kol</dt><dd>${c.mass.toFixed(1)} kg / ${(c.armLength * 100).toFixed(0)} cm</dd><dt>Atalet Ixx / Iyy / Izz</dt><dd>${c.inertia.map((x) => x.toFixed(3)).join(" / ")} kg·m²</dd><dt>Motor gecikmesi</dt><dd>${(c.motorTimeConstant * 1000).toFixed(0)} ms</dd><dt>Hava yoğunluğu</dt><dd>${c.airDensity.toFixed(2)} kg/m³</dd><dt>Azami rotor / yatış</dt><dd>${Math.round((c.maxRotorSpeed * 60) / (2 * Math.PI))} rpm / ${((c.maxTilt * 180) / Math.PI).toFixed(0)}°</dd><dt>Ortalama rüzgâr</dt><dd>${s.wind.toFixed(1)} m/s</dd><dt>Dryden türbülansı</dt><dd>${s.turbulence ? `açık · tohum ${s.seed}` : "kapalı"}</dd><dt>En büyük takip hatası</dt><dd>${m.maxTrackingError.toFixed(2)} m</dd><dt>İniş konum hatası</dt><dd>${(m.landingError * 100).toFixed(1)} cm</dd><dt>Temas alçalma hızı</dt><dd>${m.contactSpeed.toFixed(2)} m/s</dd><dt>Motor doygunluğu</dt><dd>${m.saturationSeconds.toFixed(2)} s</dd></dl><p>MIL-F-8785C Dryden türbülansı, sabit hava yoğunluğu, ideal durum bilgisi ve noktasal zemin teması. Gerçek araç için tanımlanmış katsayılar veya PX4 SITL değildir.</p>`;
  }

  setCamera(mode) {
    for (const name of ["overview", "follow"]) {
      const button = this.elements[`${name}Button`];
      button.classList.toggle("active", mode === name);
      button.setAttribute("aria-pressed", String(mode === name));
    }
    this.elements.followZoomControls.hidden = mode !== "follow";
    this.elements.cesiumContainer
      .closest(".map-area")
      .classList.toggle("follow-camera", mode === "follow");
  }

  setFollowDistance(distance) {
    this.elements.followDistance.value = distance.toFixed(1);
    this.elements.followDistanceValue.textContent = `${distance.toFixed(1)} m`;
  }

  setFollowView(view) {
    document.querySelectorAll("[data-follow-view]").forEach((button) => {
      const selected = button.dataset.followView === view;
      button.classList.toggle("active", selected);
      button.setAttribute("aria-pressed", String(selected));
    });
  }

  render(sample, C) {
    const e = this.elements;
    const phase = this.state === "ready" ? "ready" : sample.phase;
    e.altitudeStatus.textContent = `${sample.agl.toFixed(1)} m`;
    e.speedStatus.textContent = `${(sample.groundSpeed * 3.6).toFixed(1)} km/h`;
    e.absoluteAltitude.textContent = `${sample.position.height.toFixed(1)} m`;
    e.mslAltitude.textContent = Number.isFinite(sample.mslAltitude)
      ? `${sample.mslAltitude.toFixed(1)} m`
      : "—";
    e.relativeAltitude.textContent = `${sample.relativeAltitude.toFixed(1)} m`;
    e.verticalSpeed.textContent = `${sample.verticalSpeed.toFixed(1)} m/s`;
    e.rollStatus.textContent = `${C.Math.toDegrees(sample.roll).toFixed(1)}°`;
    e.pitchStatus.textContent = `${C.Math.toDegrees(sample.pitch).toFixed(1)}°`;
    e.rateStatus.textContent = `${C.Math.toDegrees(Math.hypot(...sample.omega)).toFixed(1)}°/s`;
    e.thrustStatus.textContent = `${sample.thrust.toFixed(1)} N`;
    e.trackingStatus.textContent = `${sample.trackingError.toFixed(2)} m`;
    e.windStatus.textContent = `${Math.hypot(...sample.wind).toFixed(1)} m/s`;
    e.motorStatus.textContent = sample.motors
      .map((w, i) => `M${i + 1} ${Math.round((w * 60) / (2 * Math.PI))}`)
      .join("  ·  ");
    e.headingStatus.textContent = `${Math.round(C.Math.toDegrees(sample.heading)) % 360}°`;
    e.horizonGroup.setAttribute(
      "transform",
      `rotate(${-C.Math.toDegrees(sample.roll)} 60 57) translate(0 ${C.Math.toDegrees(sample.pitch) * 1.3})`,
    );
    e.horizonHeading.textContent = `${Math.round(C.Math.toDegrees(sample.heading)) % 360}° · ${C.Math.toDegrees(sample.roll).toFixed(1)}° roll`;
    // Hemisphere suffix follows the sign so southern/western routes read right.
    const lat = C.Math.toDegrees(sample.position.latitude),
      lon = C.Math.toDegrees(sample.position.longitude);
    e.coordinates.textContent = `${Math.abs(lat).toFixed(6)}° ${lat >= 0 ? "K" : "G"} / ${Math.abs(lon).toFixed(6)}° ${lon >= 0 ? "D" : "B"}`;
    e.flightPhase.textContent =
      this.state === "paused"
        ? `Duraklatıldı · ${PHASES[phase]}`
        : PHASES[phase];
    e.flightTime.textContent = `${formatTime(sample.elapsed)} / ${formatTime(this.mission.duration)}`;
    e.flightProgress.value = String(Math.round(sample.progress * 1000));
    e.flightDistance.textContent = `${formatDistance(sample.traveled)} kat edildi`;
    const activePhase =
      phase === "arming"
        ? "takeoff"
        : ["settle", "touchdown", "disarm", "complete"].includes(phase)
          ? "landing"
          : phase;
    document
      .querySelectorAll("[data-phase]")
      .forEach((el) =>
        el.classList.toggle("active", el.dataset.phase === activePhase),
      );
  }
}
