# MEGABUS — Kurulum Rehberi

## 1. Arduino IDE Kurulumu

### Arduino Uno (Vehicle Controller)

**Board:** Arduino Uno (Tools → Board → Arduino AVR Boards → Arduino Uno)

**Gerekli kütüphaneler** (Library Manager → Search):
- `Adafruit ADS1X15` (FSR/ADS1115 için)

**Upload:** 115200 baud

**Dosya:** `firmware/arduino_vehicle_controller/arduino_vehicle_controller.ino`

---

### ESP8266 (Firebase Bridge)

**Board paketi** (henüz yoksa):
File → Preferences → Additional Boards Manager URLs:
```
https://arduino.esp8266.com/stable/package_esp8266com_index.json
```
Tools → Board Manager → `esp8266` → Install

**Board:** NodeMCU 1.0 (ESP-12E Module)
**Upload Speed:** 115200
**CPU Frequency:** 80 MHz
**Flash Size:** 4MB (FS:2MB OTA:~1019KB)

**Gerekli kütüphaneler:** ESP8266WiFi, ESP8266HTTPClient, WiFiClientSecureBearSSL
(bunlar esp8266 board paketi ile birlikte gelir, ayrıca kurulum gerekmez)

**Dosya:** `firmware/esp8266_firebase_bridge/esp8266_firebase_bridge.ino`

---

## 2. secrets.h Kurulumu

```
firmware/esp8266_firebase_bridge/secrets.example.h   ← repoda (şablon)
firmware/esp8266_firebase_bridge/secrets.h           ← lokal (gerçek değerler)
```

`secrets.example.h` dosyasını kopyala, `secrets.h` adını ver, gerçek değerleri gir:

```cpp
#define WIFI_SSID     "wifi_adin"
#define WIFI_PASS     "wifi_sifren"
#define FIREBASE_URL  "https://PROJE-ADIN-default-rtdb.europe-west1.firebasedatabase.app"
#define CAR_ID        "car1"
```

---

## 3. Firebase Kurulumu

1. [Firebase Console](https://console.firebase.google.com) → Projeyi aç
2. Realtime Database → Kurallar → `firebase-schema.md` içindeki örnek kuralları yapıştır
3. Authentication → Sign-in method → Email/Password → Etkinleştir
4. Admin kullanıcı ekle (Authentication → Add user)

---

## 4. Web Panel (Vercel)

Vercel repo kökünden deploy alıyor. `MEGABUS WEB/` içindeki dosyalar:
- `index.html` — giriş
- `admin.html` — yönetici paneli
- `guest.html` — yolcu paneli
- `vercel.json` — yönlendirme kuralları

Deploy:
```
vercel --prod
```

---

## 5. İlk Açılış Test Sırası

1. Arduino'ya `arduino_vehicle_controller.ino` yükle
2. ESP'ye `esp8266_firebase_bridge.ino` yükle
3. Güç ver → Firebase konsolunda `cars/car1/online: true` görünmeli
4. Admin paneli aç → Commands → Mode: `auto` gönder
5. Arabayı çizgiye koy → hareket etmeli
6. Manuel kontrol butonlarını test et (basılı tut → hareket, bırak → dur)
7. Hall sensörüne mıknatıs yaklaştır → 3 sn durmalı, devam etmeli

---

## 6. PID Ayarı

Admin panelinden veya `arduino_vehicle_controller.ino` içindeki define'lardan:

```cpp
#define DEFAULT_KP  35.0f   // Oransal — hızlı tepki
#define DEFAULT_KI   0.0f   // İntegral — şimdilik 0 (PD modu)
#define DEFAULT_KD  20.0f   // Türevsel — salınım sönümleme
#define DEFAULT_BASE_SPEED 180  // 0-255
```

- Araç çok sallanıyorsa → Kd artır
- Çizgiye yavaş dönüyorsa → Kp artır
- Düz gidişte kayıyorsa → Ki çok küçük bir değer ver (0.5–2.0)
