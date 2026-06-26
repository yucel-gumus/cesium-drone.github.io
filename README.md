# 3D Drone Simülasyonu (CesiumJS)

**CesiumJS** ile 3D dünya görünümünde drone modeli, telemetri paneli ve önceden tanımlı uçuş rotası animasyonu.

**Canlı:** [yucel-gumus.github.io/cesium-drone.github.io](https://yucel-gumus.github.io/cesium-drone.github.io/)  
**GitHub:** [yucel-gumus/cesium-drone.github.io](https://github.com/yucel-gumus/cesium-drone.github.io)

---

## Özellikler

- Cesium globe + `CesiumDrone.glb` 3D model
- Batarya, irtifa, hız, bağlantı göstergeleri
- Kamera açıları ve smooth path animasyonu
- Webpack 5 dev server ve production bundle
- GitHub Pages (`gh-pages -d dist`)

---

## Teknoloji

- Cesium 1.120, Webpack, dotenv-webpack (ion token vb.)
- ES6+ kaynak `src/`, çıktı `dist/`

---

## Kurulum

```bash
git clone https://github.com/yucel-gumus/cesium-drone.github.io.git
cd cesium-drone.github.io
npm install
```

### Cesium Ion (gerekirse)

`.env` içinde `CESIUM_ION_ACCESS_TOKEN` — [cesium.com/ion](https://cesium.com/ion) ücretsiz token.

```bash
npm start          # webpack serve --open
npm run build
npm run deploy     # GitHub Pages
```

---

## Asset'ler

- `CesiumDrone.glb` — drone mesh
- `public/` — statik dosyalar

---

## Lisans

MIT; Cesium kullanım şartları geçerlidir.