# Uzungöl · Dinamik Quadrotor Simülasyonu

Uzungöl'ün gerçek arazi verisi üzerinde **motor hazırlığı → kalkış → eğrisel keşif → dönüş → yaklaşma → zemin teması → motor durdurma** görevini çözen, tarayıcı tabanlı bir **6 serbestlik dereceli (6-DOF) rijit cisim simülasyonu**.

Drone'un konumu veya gövde açısı bir rota animasyonuna zorlanmaz. Rota yalnızca kontrolcüye hedef verir; gerçek konum, hız, quaternion yönelimi, gövde açısal hızı ve dört motor devri kuvvet/tork denklemleriyle birlikte entegre edilir. Model, ölçümlerle tanımlanmış belirli bir ticari drone değildir: açık parametreli **genel 2 kg X quad** kullanılır.

## Başlatma

Node.js **22.15+** ve Cesium World Terrain (asset 1) erişimli tarayıcı tokenı gerekir.

```sh
npm ci
cp .env.example .env
```

`.env` içinde:

```dotenv
CESIUM_ION_TOKEN=your_cesium_ion_token
```

Token tarayıcıda kullanılır; Cesium ion üzerinden izinlerini ve kullanılacağı alan adlarını kısıtlayın. `.env` Git'e eklenmez. Arayüzün **Harita erişim ayarı** bölümüne girilen alternatif token yalnızca sekme oturumunda saklanır.

```sh
npm start         # http://localhost:9000
npm run build
npm run preview   # http://127.0.0.1:9001/cesium-drone.github.io/
```

Cesium motoru, worker ve stiller aynı sabit **1.120.0** sürümünden paketlenir. Arazi ve Esri uydu görüntüsü internet bağlantısı gerektirir. Göreli dosya adresleri GitHub Pages alt diziniyle uyumludur.

## Fizik çekirdeği

Referanslar [PX4 kontrol mimarisi](https://docs.px4.io/main/en/flight_stack/controller_diagrams), [PX4 jerk sınırlı hedef üretimi](https://docs.px4.io/main/en/config_mc/mc_jerk_limited_type_trajectory) ve [SE(3) üzerinde quadrotor kontrolü](https://arxiv.org/abs/1003.2005) üzerine birincil kaynaklardır. Bu uygulama kendi kontrolcüsünü kullanır; **PX4 çalıştırmaz** ve PX4/SITL uyumluluğu iddia etmez.

Koordinat ve birim sözleşmesi:

- Dünya: kalkış noktasına sabitlenmiş, sağ elli **ENU** (doğu, kuzey, yukarı), metre.
- Gövde: sağ elli **FLU** (ileri, sol, yukarı).
- Quaternion: `w,x,y,z`, gövdeden ENU'ya dönüşüm. Render sırasında ENU'dan ECEF'e çevrilir.
- Açısal hız rad/s, tork N·m, itki N, motor hızı rad/s. Arayüzdeki RPM dönüşümü `ω × 60 / (2π)`.
- Motor sırası: ön sağ, arka sağ, arka sol, ön sol. Komşu rotorlar ters yönde döner.

Entegrasyon denklemleri:

```text
p_dot = v
m v_dot = R(q) [0, 0, Σf_i] + F_drag + [0, 0, -mg]
q_dot = 1/2 q ⊗ [0, Ω]
I Ω_dot = τ - Ω × (I Ω) - D_Ω Ω
ω_i_dot = (ω_cmd_i - ω_i) / motor_time_constant
f_i = k_f × (ρ / 1.225) × ω_i²
F_drag = -(1/2 ρ CdA |v - wind| + linear_drag) × (v - wind)
τ = Σ r_i × [0,0,f_i] + [0,0,Σ spin_i × reaction_ratio × f_i]
```

Dört motor ayrı ayrı doygunluğa girer. Komut hızının değişmesi rotor devrini anında değiştirmez; motor zaman sabiti uygulanır. Açısal hız gövde ataletinden, quaternion da açısal hızdan oluşur. Rotor görselleri de entegre edilmiş rotor fazlarını kullanır. Kameraya göre modeli büyüten `minimumPixelSize` kaldırılmıştır; GLB motor aralığı fizik profilinin kol uzunluğuna göre ölçeklenir.

Derleme, kaynak `CesiumDrone.glb` dosyasından yazısız `drone.glb` üretir. `scripts/clean-drone-model.cjs`, üst gövdedeki yazı ve simge bölgesinin üçgenlerini UV sınırlarında böler ve yalnızca bu bölgeyi aynı kaplamanın yazısız gövde dokusuna eşler. Böylece renk, normal ve ORM kanallarındaki marka izi birlikte kaldırılır. Kaynak dosya, gövde yüzeyi, diğer parçalar ve rotor düğümleri korunur.

**RK4 integratör 240 Hz**, kontrolcü **120 Hz** çalışır. Kontrolcü; konum/hız geri beslemesi, sınırlı integral, referans ivmesi, quaternion yönelim hatası, yönelim açısal hız ileri beslemesi ve gövde açısal hız geri beslemesi kullanır. Yaw hedefinin açısal hızı ve açısal ivmesi sınırlandırılır. Kontrolcü komutları limitlidir; fizik çözümünün gerçek sonucu referansa kopyalanmaz.

Varsayılan genel profil:

| Parametre                                | Değer                                               |
| ---------------------------------------- | --------------------------------------------------- |
| Kütle                                    | 2 kg; arayüzde 1–3 kg                               |
| Atalet Ixx / Iyy / Izz                   | 0,035 / 0,035 / 0,060 kg·m²                         |
| Merkez–motor kolu                        | 0,25 m                                              |
| İtki katsayısı, 1,225 kg/m³ referansında | 1,8 × 10⁻⁵ N/(rad/s)²                               |
| Reaksiyon torku / itki                   | 0,018 m                                             |
| Rotor devir sınırı                       | 850 rad/s ≈ 8.117 rpm                               |
| Motor zaman sabiti                       | 55 ms                                               |
| Sabit hava yoğunluğu                     | 1,09 kg/m³; bölge yüksekliği için yaklaşık varsayım |
| CdA / doğrusal sürükleme                 | 0,08 m² / 0,10 N·s/m                                |
| Yatış komut sınırı                       | 25°                                                 |
| Yaw komut hızı / ivmesi                  | 40°/s / 60°/s²                                      |
| Ortalama rüzgâr                          | 2 m/s; arayüzde 0–6 m/s                             |

Kütle değiştirildiğinde sabit geometrili genel araç varsayımıyla atalet de kütleyle orantılı ölçeklenir. Rüzgâr doğu-kuzeydoğu yönünde, deterministik sinüs bileşenli sentetik hamleler içerir. Gerçek meteoroloji verisi veya kalibre edilmiş türbülans spektrumu değildir.

## Dönüş ve rota üretimi

Eski köşede durup yönlenme modeli kaldırılmıştır. WGS84 koordinatları kalkışın yerel ENU çerçevesine çevrilir. Köşeler, başlangıç ve bitişinde sıfır eğrilikli **beşinci derece Bézier geçişlerle** yuvarlatılır. Düz çizgi/eğri birleşimleri geometrik C2 sürekliliğe sahiptir. Tam 180° geri dönüş gibi tekil rota geometrileri reddedilir.

1. Eğrilik `κ` üzerinden hız sınırı `v ≤ sqrt(a_lateral / |κ|)` hesaplanır. Drone keskin viraja **önceden yavaşlayarak** girer.
2. İleri/geri hız planlaması ve yerel yeniden zamanlama yapılır.
3. Konum, hız ve ivme uç koşullarını paylaşan quintic Hermite bölümleri kullanılır. Referans konum/hız/ivmesi süreklidir; hız, ivme ve jerk sınırları bölüm içlerinde denetlenir.
4. Bu referans yalnızca kontrolcü hedefidir. Dönüş için gereken yanal kuvveti, gerçek motor itkisiyle yatmış gövde üretir. Gövde yaw'ı, uçuş doğrultusundan doğrudan atanmaz.

Referans üst sınırları: yer hızı **8 m/s**, tırmanma **2,5 m/s**, alçalma **1,5 m/s**, yatay ivme **1,5 m/s²**, viraj yanal ivmesi **1,2 m/s²**, jerk **2 m/s³**. Bunlar hedef üretim sınırlarıdır; kapalı çevrimdeki gerçek hız/ivme/jerk küçük takip sapmaları gösterebilir ve ölçülür. Özellikle zemin teması, ideal rijit temas impulsu içerdiğinden havadaki jerk sınırı temas anına uygulanmaz.

## Arazi, yükseklik ve iniş

**Yuvarlatılmış gerçek referans yolu** en fazla 6 m aralıkla, merkez ve her iki yanda 12 m koridorla Cesium World Terrain üzerinden örneklenir. Eski köşeli rotanın arazi sorgularını eğrisel yola aktarmak gibi bir varsayım yapılmaz.

Keşif tavanı, bu koridordaki en yüksek yerel zemin + seçilen açıklık + **10 m arazi / 5 m takip payı** ile hesaplanır. Bu sürüm sabit AGL takibi yerine bütün koridorun üstünde sabit yerel keşif tavanı kullanır. Zemin alçaldığında AGL artar. Kullanıcı açıklığı **30–150 m** arasında seçebilir. Dinamik çözümden sonra uçulan iz yeniden denetlenir; açıklık veya koridor sınırı bozulmuşsa görev etkinleştirilmez. Arazi örneği eksikse de uçuş açılmaz.

- AGL: gerçek ENU konumunun, yakın merkez çizgi zemin profiliyle farkı.
- Elipsoit irtifası: gerçek ECEF konumunun WGS84 Cartographic yüksekliği.
- Kalkışa göre: bu elipsoit yüksekliği − kalkış zemininin yüksekliği.

Elipsoit irtifası **MSL değildir**. Jeoit modeli olmadan deniz seviyesi yüksekliği gösterilmez. AGL, örneklenmiş zemin profilinin interpolasyonudur; harita DEM'inin gerçek dünya doğruluğuna dair bir garanti değildir.

İnişte drone önce kalkışın üstünde frenler, yaklaşır, son 3 metrede yavaşlar. Kalkış alanı düz ve rijit bir temas düzlemi olarak modellenir: normal impuls, kayma/statik sürtünme uygulanır. CG için 0,10 m zemin ofseti kullanılır. Motorlar yalnızca ölçülen temas/yakınlık, düşük hız ve konum koşulları sağlanınca kademeli kapatılır. **XY konumu eve kopyalanmaz**; santimetre düzeyindeki son hata raporda kalır. Esnek iniş takımı, çoklu ayak teması ve zemin eğimiyle temas henüz modellenmez.

## Kullanım ve tekrarlanabilirlik

Başlatmadan önce uçuş, **Web Worker içinde** sabit zaman adımıyla tamamen çözülür ve 30 Hz durum kaydına alınır. Harita bu fizik çözümünü oynatır; önceden çizilmiş konum/yön animasyonu üretmez. Bu yaklaşım deneyin FPS'ten, tarayıcı sekmesinin yavaşlamasından ve 1×/10× oynatmadan etkilenmemesini sağlar. Parametre değiştirip **Rotayı hesapla** dediğinizde bütün fizik deneyi tekrar çalışır. Şu an uçuş ortasında manuel kumanda/fizik girdisi yoktur.

- **Başlat** drone'u takip eder. Duraklat/devam/başa al ve zaman çubuğu aynı fizik kaydını kullanır.
- Duraklatıldığında telemetri, dondurulan andaki gerçek durum hızını göstermeye devam eder; hız yapay olarak sıfırlanmaz.
- **Rota / Drone takibi**: takip kamerası sürekli ENU çerçevesinde ve yumuşatılmış yön takibiyle çalışır. EntityView'in hızla değişebilen takip referansına dayanmaz. Takipte fare tekerleği/touchpad, iki parmakla açma-kapama veya takip mesafesi sürgüsü ve +/− düğmeleriyle **1,2–80 m** aralığında yakınlaşılır. Seçilen mesafe uçuş ve kamera modu geçişlerinde korunur.
- **Altı takip açısı**: arkadan, önden, sol yandan, sağ yandan, üstten ve çapraz. Açılar drone'un yönüne göredir ve drone döndükçe birlikte döner. Seçim sırasında kamera yönü ve eğimi yumuşak geçer; takip mesafesi korunur. Seçili açı rota görünümünden dönüşte ve yeniden uçuşta korunur.
- Yapay ufuk, gerçek quaternion'dan yatış/tangaj gösterir. Motor RPM, toplam itki, gövde açısal hızı, rüzgâr ve takip hatası aynı fizik kaydındandır.
- Yeşil çizgi referans, turkuaz çizgi son 120 saniyenin gerçekleşen izidir.
- **Fizik modeli ve uçuş doğrulaması** bölümünde parametreler, takip/iniş hatası, temas hızı ve motor doygunluğu görülebilir.
- **CSV** 240 Hz çözümün 30 Hz kaydından 10 Hz örnekler indirir; ENU konumu/hızı, quaternion, gövde hızları, ivme, motor rad/s, itki/tork, rüzgâr, takip hatası ve faz kodu içerir. Faz sırası `quadrotor.mjs` içindeki `PHASE_NAMES` ile aynıdır; ENU orijini kalkışın CG konumudur.

## Doğrulama

```sh
npm test
npm run build
```

Testler, analitik serbest düşüş, dengeli hover, yatık itkinin yanal kuvveti, motor zaman sabiti, üç eksen tork dağılımı, torksuz dönmede enerji korunumu, RK4 zaman adımı yakınsaması, quaternion normu, rüzgâr/kütle tepkisi ve tekrarlanabilirlik içerir. Ayrıca tüm Uzungöl görevi; C2 referans birleşimleri, hız/ivme/jerk sınırları, durmadan viraj geçişi, gövde yönelim sürekliliği, arazi koridoru ve zorla konum düzeltmeden iniş açısından denetlenir.

Gerçek tarayıcı için yerel Kimi Browser Extension köprüsü ve `npm run preview` açıkken:

```sh
node scripts/verify-browser.mjs
```

Bu test worker'ın alt dizinde çalışmasını, gerçek araziyi, bağımsız 289 arazi sorgusunu, fizik yönelim sürekliliğini, tüm uçuşu, kamera/zaman kontrollerini, yeniden planlamayı, mobil görünümü ve token hata/kurtarma davranışını kontrol eder. Rapor ve ekran görüntüleri `artifacts/` altında tutulur. Yalnızca geçersiz token denemesindeki beklenen 401 kabul edilir; diğer başarısız ağ istekleri testi düşürür.

## Doğruluk sınırı ve sonraki kalibrasyon

Bu, kinematik animasyondan gerçek kuvvet/tork entegrasyonuna geçirilmiş bir simülasyon çekirdeğidir. **Sertifikalı veya uçuş verisiyle doğrulanmış bir profesyonel eğitim/FDM ürünü olduğu iddia edilmez.** Gerçek bir drone'a yakınlık için kütle/CG/atalet ölçümleri, pervane itki-tork testleri, motor/ESC tanımlaması ve gerçek uçuş kayıtlarıyla model doğrulaması gerekir.

Şu varsayımlar açıkça korunur: ideal ve gecikmesiz durum bilgisi; sabit yerçekimi/hava yoğunluğu; tek rijit gövde; karesel rotor itkisi; basitleştirilmiş sürükleme; noktasal düz pad teması. Rotor jiroskopik momenti ve hızlanma reaksiyonu, blade flapping, ground effect, downwash/vortex ring, pil/ESC gerilim düşümü, sensör gürültüsü/EKF, Coriolis ve bina/ağaç/elektrik hattı çarpışmaları modellenmez. Bu etkiler ölçülmüş veri ve ek doğrulama gerektirir. Mevcut geniş ölçekli DEM, güncel engel haritası değildir. Gerçek drone'a komut gönderilmez.

## Dosyalar

```text
src/flight-math.mjs           Vektör/quaternion ve Hermite matematiği
src/trajectory.mjs            C2 köşe geçişleri ve sınırlandırılmış referans
src/quadrotor.mjs              Rijit cisim, motor, kontrolcü, RK4, temas, kayıt
src/simulation-worker.mjs      Ana UI'yi kilitlemeden fizik çözümü
src/flight-planner.mjs         WGS84/ENU, eğrisel arazi sorgusu, iz denetimi
src/main.js                   Harita, quaternion render, rotorlar, kamera, CSV
src/ui-controller.js           Gerçek durum telemetrisi ve yapay ufuk
public/points.json             Mevcut coğrafi keşif noktaları
scripts/verify-browser.mjs     Gerçek tarayıcı/terrain regresyonu
```

Yayın için `npm run deploy` üretim build'ini geçmişi koruyarak `gh-pages` dalına gönderir ve `.nojekyll` ekler. GitHub Pages kaynağı `gh-pages` / `/` olmalıdır. Tokenın build ortamında tanımlanması gerekir; yalnızca tarayıcı kullanımı için hazırlanmış, okuma yetkili ve alan adı kısıtlamalı Cesium ion tokenı kullanın. GitHub kimlik doğrulaması sistem anahtarlığı üzerinden yapılmalı; erişim anahtarını uzak depo URL'sine yazmayın.

Geliştirici: [Yücel Gümüş](https://www.yucelgumus.dev/)
