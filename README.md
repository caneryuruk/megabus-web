# MEGABUS

Çizgi takip eden otonom metrobüs robotu + Firebase web paneli.

## Mimari

```
┌─────────────────────────────┐     Serial (9600)    ┌──────────────────────┐
│      Arduino Uno            │ ◄──────────────────► │    ESP8266 NodeMCU   │
│  - 5x QTR sensör (PCF8574) │   >C komut / >P PID   │  - Wi-Fi bağlantısı  │
│  - L298N motor sürücü       │   <T telemetri        │  - Firebase köprüsü  │
│  - HC-SR04 mesafe           │   >S segment (ML)     │  - mlTrainingData    │
│  - Hall istasyon algılama   │                       └──────────┬───────────┘
│  - Encoder (×2)             │                                  │ HTTPS
│  - ADS1115 FSR doluluk      │                       ┌──────────▼───────────┐
│  - Segment ölçüm + ETA      │                       │  Firebase RTDB       │
└─────────────────────────────┘                       └──────────┬───────────┘
                                                                 │
                                              ┌──────────────────▼──────────┐
                                              │   Web Paneli (Vercel)        │
                                              │   admin.html — yönetici      │
                                              │   guest.html — yolcu         │
                                              │   ETA modeli — TensorFlow.js │
                                              └─────────────────────────────┘
```

## Dosya Yapısı

> Bu repo Vercel'e deploy edilir. Web dosyaları kökte; `firmware/` ve `docs/`
> aynı repoda durur ama `.vercelignore` ile siteye gönderilmez.

```
.
├── index.html  admin.html  guest.html      ← web paneli (Vercel kökü)
├── vercel.json  .vercelignore
├── firebase_structure_example.json
├── firmware/
│   ├── arduino_vehicle_controller/
│   │   ├── arduino_vehicle_controller.ino   ← Ana Arduino firmware
│   │   └── stepN_*/                          ← Donanım test sketch'leri
│   └── esp8266_firebase_bridge/
│       ├── esp8266_firebase_bridge.ino      ← Ana ESP firmware
│       ├── secrets.example.h                ← Şablon (repoda)
│       └── secrets.h                        ← Gerçek bilgiler (.gitignore'da)
└── docs/
    ├── wiring.md          ← Pin tabloları, güç, voltaj bölücü
    ├── protocol.md        ← Arduino ↔ ESP Serial protokolü (>C, >P, <T, >S)
    ├── firebase-schema.md ← Firebase node yapısı
    └── setup.md           ← Arduino IDE, Firebase, Vercel kurulumu
```

## Hızlı Başlangıç

1. `firmware/esp8266_firebase_bridge/secrets.example.h` → kopyala → `secrets.h` → Wi-Fi ve Firebase bilgilerini gir
2. Arduino'ya `arduino_vehicle_controller.ino` yükle
3. ESP'ye `esp8266_firebase_bridge.ino` yükle
4. Firebase / admin panelinde araç **Connected** görünmeli
5. Admin panel → araç → mod/manuel/kalibrasyon kontrollerini kullan

Detaylı kurulum: [docs/setup.md](docs/setup.md)

## Özellikler

| Özellik | Durum |
|---|---|
| 5 sensörlü PID çizgi takip | ✅ |
| HC-SR04 engel durma + hız ayarı | ✅ |
| Hall sensörü istasyon algılama | ✅ |
| Encoder mesafe ölçümü | ✅ |
| ADS1115 FSR doluluk (2 koltuk) | ✅ |
| Firebase manuel kontrol (basılı tut sür) | ✅ |
| Web'den canlı PID ayarı | ✅ |
| Kalibrasyon/ölçüm modu (Hall→segment mesafe/süre) | ✅ |
| Manuel modda ölçüm duraklatma | ✅ |
| ML ETA eğitimi (TensorFlow.js, araç başına) | ✅ |
| Admin/guest web paneli (İngilizce) | ✅ |
| Recovery modu | ⏸ sırada |
| Çoklu araç sefer senkronizasyonu | ⏸ sırada |

## Güvenlik

- `secrets.h` `.gitignore`'da — repoya/siteye gitmez (WiFi şifresi içerir)
- `firmware/` ve `docs/` `.vercelignore`'da — Vercel'de yayınlanmaz
- Firebase Web API key client-side'da görünür, bu normaldir — güvenlik Firebase Security Rules ile sağlanır
- Admin paneli Firebase Auth (email/password) ile korunmalı
- Manuel kontrol buton bırakılınca otomatik STOP gönderilir (güvenlik)

## Donanım

| Parça | Adet |
|---|---|
| Arduino Uno | 1 |
| ESP8266 NodeMCU | 1 |
| QTR reflektans sensör | 5 |
| PCF8574 I2C genişletici | 1 |
| L298N motor sürücü | 1 |
| DC motor | 2 |
| HC-SR04 mesafe sensörü | 1 |
| Hall etkisi sensörü | 1 |
| Encoder disk + sensör | 2 |
| ADS1115 ADC | 1 |
| FSR (kuvvet sensörü) | 2 |
| LM2596 Buck dönüştürücü | 1 |
| Li-ion pil (seri ×2) | 2 |
