# 🚁 3D Drone Simülasyon Sistemi

<div align="center">
  <img src="https://img.icons8.com/fluency/96/drone.png" alt="Drone Simülasyon Logo"/>
  <p><strong>CesiumJS ile Geliştirilmiş 3D Drone Kontrol ve İzleme Sistemi</strong></p>
</div>

## 📋 İçerik Tablosu
- [🌟 Özellikler](#-özellikler)
- [🛠️ Teknolojiler](#️-teknolojiler)
- [⚡ Hızlı Başlangıç](#-hızlı-başlangıç)
- [📝 Kullanım Kılavuzu](#-kullanım-kılavuzu)
- [🔧 Geliştirme](#-geliştirme)
- [📱 Desteklenen Platformlar](#-desteklenen-platformlar)
- [📄 Lisans](#-lisans)

## 🌟 Özellikler

- 🌍 CesiumJS ile gerçekçi 3D harita görüntüleme
- 🎮 Gerçek zamanlı drone kontrol arayüzü
- 📊 Canlı telemetri veri görüntüleme
  - Batarya durumu
  - Yükseklik bilgisi
  - Hız göstergesi
  - Bağlantı durumu
- 🛣️ Önceden planlanmış uçuş rotası
- 🎥 Çoklu kamera açıları
- 🔄 Smooth animasyonlar ve geçişler
- 📱 Tüm cihazlara uyumlu responsive tasarım

## 🛠️ Teknolojiler

- **Frontend**: JavaScript (ES6+), HTML5, CSS3
- **3D Görselleştirme**: CesiumJS
- **Build Tool**: Webpack
- **Package Manager**: npm
- **Version Control**: Git

## ⚡ Hızlı Başlangıç

### Gereksinimler
- Node.js (16.x veya üzeri)
- npm (Node Package Manager)
- Cesium ion hesabı ve access token

### Kurulum

1. **Projeyi klonlayın**
\`\`\`bash
git clone https://github.com/yourusername/cesium-drone.github.io.git
cd cesium-drone.github.io
\`\`\`

2. **Bağımlılıkları yükleyin**
\`\`\`bash
npm install
\`\`\`

3. **Cesium ion token'ınızı ayarlayın**
\`\`\`bash
cp src/config.template.js src/config.js
\`\`\`
- \`config.js\` dosyasını açın ve \`YOUR_CESIUM_ION_TOKEN_HERE\` yerine kendi token'ınızı yazın

4. **Geliştirme sunucusunu başlatın**
\`\`\`bash
npm start
\`\`\`

5. Tarayıcınızda \`http://localhost:8080\` adresine gidin

## 📝 Kullanım Kılavuzu

### Temel Kontroller

1. **Drone Kontrolü**
   - "Keşif Uçuşu Başlat" butonuna tıklayarak simülasyonu başlatın
   - Gerçek zamanlı telemetri verilerini sol panelden takip edin
   - Uçuş rotasını harita üzerinde görüntüleyin

2. **Kamera Kontrolleri**
   - Sol Tık + Sürükle: Kamerayı döndür
   - Sağ Tık + Sürükle: Yakınlaştır/Uzaklaştır
   - Orta Tık + Sürükle: Kaydır

### Uçuş Rotası Ayarları

\`points.json\` dosyasını düzenleyerek özel uçuş rotaları oluşturabilirsiniz:

\`\`\`json
{
  "dronePoints": [
    {
      "latitude": 40.2925729751587,
      "longitude": 40.61789421298509,
      "height": 1500
    }
  ]
}
\`\`\`

## 🔧 Geliştirme

### Proje Yapısı
\`\`\`
cesium-drone.github.io/
├── src/
│   ├── index.html      # Ana HTML dosyası
│   ├── main.js         # Ana uygulama mantığı
│   ├── style.css       # Stil tanımlamaları
│   ├── ui-controller.js # UI kontrolcüsü
│   └── config.js       # Yapılandırma dosyası
├── public/
│   ├── CesiumDrone.glb # 3D drone modeli
│   └── points.json     # Uçuş rotası verileri
├── package.json        # Proje bağımlılıkları
└── README.md          # Dokümantasyon
\`\`\`

### Production Build

Production ortamı için optimize edilmiş bir build oluşturmak için:

\`\`\`bash
npm run build
\`\`\`

## 📱 Desteklenen Platformlar

- 💻 Masaüstü Bilgisayarlar (Windows, macOS, Linux)
- 📱 Mobil Cihazlar (iOS, Android)
- 🌐 Modern Web Tarayıcıları (Chrome, Firefox, Safari, Edge)

## 📄 Lisans

Bu proje MIT lisansı altında lisanslanmıştır. Detaylı bilgi için [LICENSE](LICENSE) dosyasına bakınız.

---

### 🤝 İletişim ve Katkıda Bulunma

- 🐛 Hata bildirimleri için [Issues](https://github.com/yourusername/cesium-drone.github.io/issues) sayfasını kullanın
- 💡 Yeni özellik önerileri için [Pull Request](https://github.com/yourusername/cesium-drone.github.io/pulls) gönderin
- 📧 İletişim: your-email@example.com

---

<div align="center">
  <sub>Built with ❤️ by Your Name</sub>
</div>
