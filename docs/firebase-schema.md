# MEGABUS — Firebase Realtime Database Şeması

Proje: `megabus-8ff11` (europe-west1)

---

## cars/{carId}

ESP8266 tarafından güncellenir (PATCH, ~800 ms aralıkla).

```json
{
  "carId": "car1",
  "online": true,
  "wifiConnected": true,
  "firebaseConnected": true,
  "lastSeen": 1718000000000,
  "mode": "auto",
  "leftPWM": 175,
  "rightPWM": 165,
  "distanceCm": 42.3,
  "hallRaw": 0,
  "stationDetected": false,
  "lineCase": "CENTER",
  "action": "PID_FOLLOW",
  "encL": 340,
  "encR": 338,
  "occupancy": {
    "fsr1Raw": 0,
    "fsr2Raw": 0,
    "status": "empty",
    "label": "Bos",
    "color": "green",
    "adsReady": false
  }
}
```

**guest.html'nin okuduğu alanlar:** `mode`, `occupancy.status/color/label`, `stationDetected`, `lineCase`

---

## manualControls/{carId}

Admin panel yazar, ESP okur (polling ~600 ms).

```json
{
  "enabled": false,
  "command": "STOP",
  "speed": 180,
  "commandVersion": 1718000000000,
  "updatedAt": 1718000000000,
  "updatedBy": "admin"
}
```

**Kurallar:**
- Buton basılıyken: `enabled: true`, `command: "FORWARD"`, `commandVersion: Date.now()`
- Buton bırakılınca: `enabled: false`, `command: "STOP"` — ESP STOP iletir (güvenlik)
- ESP 2 sn içinde yeni komut görmezse otomatik STOP

---

## commands/{carId}

Admin panel yazar, ESP okur (polling ~400 ms).

```json
{
  "mode": "idle",
  "command": "NONE",
  "expectedNextStop": null,
  "commandVersion": 0,
  "updatedAt": 0,
  "updatedBy": "admin"
}
```

**mode değerleri:** `idle` | `auto` | `stopped`
(v2'de eklenecek: `calibration`, `recovery`, `service`)

---

## stations/{stationId}

İstasyon ESP'leri tarafından güncellenir (v1'de pasif).

```json
{
  "stationId": "station1",
  "online": false,
  "lastSeen": 0
}
```

---

## system

Admin panel okur/yazar.

```json
{
  "mode": "idle",
  "tripsStarted": false,
  "allDevicesOnline": false,
  "routeCalibrated": false,
  "activeRouteVersion": 0
}
```

---

## route (v2)

Rota kalibrasyonu tamamlandığında ESP yazar.

```json
{
  "stopCount": 6,
  "routeVersion": 1,
  "segments": {
    "0": { "from": 0, "to": 1, "distance": 124.5, "duration": 8200 }
  }
}
```

---

## mlTrainingData/{carId} (v2)

Her segment tamamlandığında ESP yazar.

---

## Firebase Security Rules (özet)

- `cars`, `stations`, `system`: herkese okuma, yazma → auth gerekli
- `manualControls`, `commands`: yazma → auth gerekli (admin)
- guest.html anonim okuma yapıyor — kurallar buna izin vermeli

Örnek kural (Realtime DB):
```json
{
  "rules": {
    "cars": { ".read": true, ".write": "auth != null" },
    "stations": { ".read": true, ".write": "auth != null" },
    "manualControls": { ".read": "auth != null", ".write": "auth != null" },
    "commands": { ".read": "auth != null", ".write": "auth != null" },
    "system": { ".read": true, ".write": "auth != null" }
  }
}
```
