# 🛸 3D Drone Uçuş Simülasyonu (CesiumJS 3D Geospatial Simulation)

3D Drone Uçuş Simülasyonu; dünya genelinde yüksek çözünürlüklü uydu görüntüleri ve yükseklik (terrain) haritaları üzerinde önceden tanımlanmış bir uçuş rotası boyunca hareket eden bir 3D drone modelini ve anlık uçuş verilerini (telemetri) simüle eden, **CesiumJS & Webpack 5** tabanlı gelişmiş bir coğrafi bilgi sistemi (CBS) ve grafik uygulamasıdır.

---

## 🌟 Öne Çıkan Özellikler

* 🌍 **CesiumJS 3D Sanal Yer Küre Motoru:** Dünyayı 3 boyutlu olarak render eder. Dağlar, vadiler ve binalar gibi coğrafi detaylar Cesium 3D Tiles ve arazi (terrain) servisleri ile haritaya yansıtılır.
* 🛸 **3D GLB Model Yükleme (`CesiumDrone.glb`):** Uygulama, kök dizinde bulunan detaylı bir drone 3D modelini (gLTF/GLB formatında) harita üzerinde doğru ölçekte ve açıda konumlandırır.
* 📈 **Smooth Path Uçuş Rota Animasyonu:** Uçuş rotası (enlem, boylam, yükseklik koordinatları) boyunca drone'un pürüzsüz hareket etmesini sağlayan Lagrange/Bezier benzeri eğrisel rota interpolasyonları.
* 🖥️ **Canlı Telemetri HUD (Heads-Up Display) Paneli:**
  * **İrtifa (Altitude):** Deniz seviyesinden yükseklik.
  * **Hız (Speed):** Anlık hareket sürati (knot/km/h cinsinden).
  * **Batarya Durumu:** Yüzdesel batarya tüketim göstergesi.
  * **Bağlantı Kalitesi (RSSI):** Kumanda sinyal gücü simülasyonu.
* 🛠️ **Webpack 5 & Cesium Asset Yönetimi:** Cesium'un Web Workers, CSS ve statik dosyalarının tarayıcıda optimize çalışmasını sağlayan özel Webpack konfigürasyonu.

---

## 🏗️ Simülasyon ve Derleme Akışı

```
[ Cesium Ion Token ] ──► [ 3D Dünya Küresi (Tiles & Terrain) ]
                                      │
                                      ▼
[ CesiumDrone.glb ] ──► [ Rota Koordinatları (Interpolated Path) ]
                                      │
                                      ▼
[ Webpack 5 Build ] ──► [ Telemetri HUD Paneli ] ──► [ Web Tarayıcı (60 FPS) ]
```

---

## 🛠️ Teknoloji Stack

* **3D Grafik Motoru:** CesiumJS v1.120+ (WebGL tabanlı 3D CBS motoru).
* **Modül Paketleyici:** Webpack 5, Webpack CLI, Webpack Dev Server.
* **Webpack Eklentileri:** `copy-webpack-plugin` (Cesium kütüphanesinin statik varlıklarını ve Web Worker'larını kopyalar), `dotenv-webpack`.
* **Dağıtım Pipeline:** gh-pages.

---

## 📂 Proje Klasör Yapısı

```
cesium-drone.github.io/
├── src/
│   ├── index.js            # Cesium Viewer kurulumu, GLB model yükleme ve telemetri döngüsü
│   └── styles.css          # Telemetri HUD panelinin şık CSS yerleşimi
├── dist/                   # Webpack tarafından derlenen statik dosyalar
├── CesiumDrone.glb         # Harita üzerinde hareket eden 3D drone mesh modeli
├── webpack.config.js       # Cesium static asset kopyalama kurallarını içeren paketleyici ayarları
└── package.json            # gh-pages deploy scriptleri ve bağımlılıklar
```

---

## 🚀 Kurulum ve Yerel Çalıştırma

### 1. Bağımlılıkları Yükleyin
```bash
git clone https://github.com/yucel-gumus/cesium-drone.github.io.git
cd cesium-drone.github.io
npm install
```

### 2. Cesium Ion Token Ayarı (`.env`)
Cesium harita servislerinin yüklenmesi için ücretsiz bir Cesium Ion anahtarına ihtiyacınız vardır.
1. [Cesium Ion](https://cesium.com/ion/) adresinden ücretsiz üye olup token alın.
2. Proje kök dizininde `.env` dosyası oluşturun ve ekleyin:
   ```env
   CESIUM_ION_ACCESS_TOKEN=your_cesium_ion_token_here
   ```

### 3. Uygulamayı Başlatın
```bash
npm start
```
Uygulama tarayıcınızda otomatik olarak açılacaktır (varsayılan: `http://localhost:8080`).

### 4. GitHub Pages Dağıtımı (Deploy)
```bash
npm run deploy
```

---

## 🔗 Canlı Bağlantılar
* **Canlı Demo:** [https://yucel-gumus.github.io/cesium-drone.github.io/](https://yucel-gumus.github.io/cesium-drone.github.io/)
* **Geliştirici GitHub:** [https://github.com/yucel-gumus](https://github.com/yucel-gumus)