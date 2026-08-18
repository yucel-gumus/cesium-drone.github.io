# 🛸 Cesium Drone - 3D Geospatial Drone Flight & Telemetry Simulator

[![JavaScript](https://img.shields.io/badge/JavaScript-ES6+-F7DF1E?style=for-the-badge&logo=javascript&logoColor=black)](https://developer.mozilla.org/)
[![CesiumJS](https://img.shields.io/badge/CesiumJS-3D_Globe-00A3E0?style=for-the-badge&logo=cesium&logoColor=white)](https://cesium.com/platform/cesiumjs/)
[![Webpack 5](https://img.shields.io/badge/Webpack_5-Bundler-8DD6F9?style=for-the-badge&logo=webpack&logoColor=black)](https://webpack.js.org/)
[![3D GLTF](https://img.shields.io/badge/3D_Model-GLTF_%2F_GLB-FF5722?style=for-the-badge)](https://www.khronos.org/gltf/)
[![Portfolio](https://img.shields.io/badge/Portfolio-yucelgumus.dev-2563EB?style=for-the-badge&logo=google-chrome&logoColor=white)](https://www.yucelgumus.dev/)

> **CesiumJS** 3D küre motoru ve **Webpack 5** mimarisi üzerinde çalışan; önceden tanımlanmış coğrafi koordinat rotaları boyunca 3 boyutlu drone uçuşunu simüle eden, anlık telemetri (hız, irtifa, yön) ve dinamik kamera takip modları sunan interaktif CBS simülasyonu.

---

## 🌟 Öne Çıkan Özellikler

- 🌐 **3 Boyutlu Küresel Harita (CesiumJS 3D Globe):** Yüksek çözünürlüklü uydu görüntüleri ve gerçek dünya arazi yükseltisi (terrain) üzerinde simülasyon.
- 🚁 **3D Drone Modeli & Rota Takibi:** Özel `.glb` 3D drone modelinin arazi koordinatları (`points.json`) boyunca pürüzsüz interpolasyon ile uçuş yapması.
- 📟 **Canlı Uçuş Telemetrisi Paneli:** Anlık yükseklik (irtifa), uçuş hızı, koordinat konumu ve pusula yönü bilgileri (`ui-controller.js`).
- 🎥 **Çoklu Kamera Takip Modları:**
  - *Takip Kamerası (Chase Cam):* Drone arkasından birinci şahıs uçuş hissi.
  - *Serbest Kamera (Free Cam):* Kullanıcının haritada serbestçe dolaşabilmesi.
  - *Kuşbakışı (Top-Down):* Rota genel görünümü.
- ⚡ **Optimize Webpack 5 Yapılandırması:** CesiumJS statik asset'lerini ve worker'larını sorunsuz paketleyen modern derleyici yapısı.

---

## 🏗️ Mimari & Simülasyon Akışı

```mermaid
graph TD
    Waypoints[(points.json: Koordinatlar & İrtifa)] --> MainEngine[main.js: Cesium Viewer & Path Interpolation]
    Model[(CesiumDrone.glb: 3D Model)] --> MainEngine
    MainEngine --> UI[ui-controller.js: Telemetri HUD & Kamera Kontrolleri]
    MainEngine --> CesiumCanvas[Cesium 3D WebGL Canvas View]
```

---

## 🚀 Hızlı Başlangıç

### Gereksinimler
- **Node.js**: v16.0 veya üstü
- **Cesium Ion Access Token** ([Cesium Ion](https://ion.cesium.com/)'dan temin edilebilir)

### Kurulum

```bash
git clone https://github.com/yucel-gumus/cesium-drone.github.io.git
cd cesium-drone.github.io

npm install
```

### Ortam Değişkenleri (`.env`)

```env
CESIUM_ION_TOKEN=your_cesium_ion_token_here
```

### Çalıştırma

```bash
npm start
```

Uygulama varsayılan olarak `http://localhost:8080` adresinde açılacaktır.

---

## 📂 Proje Dizin Yapısı

```
cesium-drone.github.io/
├── CesiumDrone.glb                 # 3D Drone modeli
├── webpack.config.js               # Webpack Cesium yapılandırması
├── package.json
├── public/
│   └── points.json                 # Uçuş rota noktaları ve koordinatları
└── src/
    ├── index.html                  # Ana sayfa ve HUD katmanı
    ├── main.js                     # CesiumJS simülasyon başlatıcı
    ├── ui-controller.js            # Telemetri ve buton kontrolleri
    ├── config.js                   # Simülasyon sabitleri ve token
    └── style.css                   # HUD stilleri
```

---

## 📄 Lisans
Bu proje [MIT Lisansı](LICENSE) ile lisanslanmıştır.

---

## 👨‍💻 Geliştirici & İletişim

**Yücel Gümüş** - Full Stack Developer

- 🌐 **Web Sitesi / Portfolyo:** [yucelgumus.dev](https://www.yucelgumus.dev/)
- 💼 **LinkedIn:** [linkedin.com/in/yucel-gumus](https://www.linkedin.com/in/yucel-gumus/)
- 🐙 **GitHub:** [@yucel-gumus](https://github.com/yucel-gumus)

<p align="left">
  <a href="https://www.yucelgumus.dev/" target="_blank" rel="noopener noreferrer">
    <img src="https://img.shields.io/badge/Developed%20by-Yücel%20Gümüş-blue?style=for-the-badge&logo=google-chrome&logoColor=white" alt="Yücel Gümüş Portfolio" />
  </a>
</p>