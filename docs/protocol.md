# MEGABUS — Arduino ↔ ESP8266 Serial Protokolü

Baud rate: **9600** (SoftwareSerial güvenilirliği için)
Encoding: ASCII, satır sonu `\n`
Yön: her iki taraf da yalnızca kendi çıkışını basar; çakışma yok.

---

## ESP → Arduino: Komut

```
>C,<mode>,<manEn>,<cmd>,<speed>\n
```

| Alan | Tip | Değerler |
|---|---|---|
| mode | string | `idle` `auto` `manual` `stopped` |
| manEn | int | `0` = devre dışı, `1` = etkin |
| cmd | string | `STOP` `FORWARD` `BACKWARD` `LEFT` `RIGHT` |
| speed | int | 0–255 (Arduino 8-bit PWM) |

**Örnekler:**
```
>C,idle,0,STOP,0
>C,auto,0,STOP,0
>C,manual,1,FORWARD,180
>C,manual,1,LEFT,150
```

**Kurallar:**
- ESP her loop'ta `pushCommandToArduino()` ile gönderir (sık).
- Manuel etkin değilse `manEn=0` ve `cmd=STOP` gönderilir.
- Güvenlik: yalnızca bağlantı koparsa (WiFi düşer veya stream >2sn sağlıksız) STOP.
  "Komut vermezsen dur" zamanlayıcısı YOK — latch'lenmiş komut bağlantı varken sürer.

### Manuel komut kaynağı: Firebase Streaming (SSE)
Manuel komutlar artık polling yerine **kalıcı SSE bağlantısı** ile gelir:
ESP, `manualControls/{carId}.json`'a `Accept: text/event-stream` ile bağlanır,
Firebase değişiklikleri `event: put/patch` olarak **anında push** eder → gecikme
~200-400ms. Stream bağlanamazsa polling fallback (~1sn) devreye girer.

---

## ESP → Arduino: PID Ayarları

```
>P,<kp>,<ki>,<kd>,<baseSpeed>,<maxSpeed>\n
```

| Alan | Tip | Açıklama |
|---|---|---|
| kp | float | Oransal katsayı |
| ki | float | İntegral katsayı |
| kd | float | Türevsel katsayı |
| baseSpeed | int | Taban hız (0–255) |
| maxSpeed | int | Maksimum hız (0–255) |

**Örnek:**
```
>P,35.000,0.000,20.000,180,255
```

**Kurallar:**
- ESP `cars/{carId}/pid` düğümünü 1.5 sn'de bir okur.
- `updatedAt` değiştiğinde (web panelden Save) Arduino'ya `>P` satırı gönderilir.
- Arduino bu değerleri anında uygular (`Kp`, `Ki`, `Kd`, `configuredBaseSpeed`, `maxSpeed`).

---

## Arduino → ESP: Telemetri

```
<T,<lPWM>,<rPWM>,<dist>,<hall>,<station>,<lineCase>,<fsr1>,<fsr2>,<occ>,<encL>,<encR>,<action>,<mode>,<posKnown>,<curStop>,<nextStop>,<etaSec>,<seg>,<lap>\n
```

| Alan | Tip | Açıklama |
|---|---|---|
| lPWM | int | Sol motor PWM (0–255) |
| rPWM | int | Sağ motor PWM (0–255) |
| dist | float | HC-SR04 mesafe cm (1 ondalık), -1 = okunamadı |
| hall | int | Hall sensörü ham değer (0/1) |
| station | int | İstasyon tespiti (0/1) |
| lineCase | string | CENTER / SLIGHT_LEFT / LEFT / FAR_LEFT / SLIGHT_RIGHT / RIGHT / FAR_RIGHT / ALL_BLACK / LOST |
| fsr1 | int | FSR 1 ham ADS değeri |
| fsr2 | int | FSR 2 ham ADS değeri |
| occ | string | `empty` / `partial` / `full` |
| encL | long | Sol encoder toplam tik sayısı |
| encR | long | Sağ encoder toplam tik sayısı |
| action | string | PID_FOLLOW / DISTANCE_STOP / MEAS_START / MANUAL_FORWARD / … |
| mode | string | Arduino'nun mevcut modu |
| posKnown | int | Konum güvenilir mi (0/1) — ilk durağa ulaşınca 1 |
| curStop | int | Mevcut durak indeksi (0–5) |
| nextStop | int | Sıradaki durak indeksi (0–5) |
| etaSec | int | Sıradaki durağa tahmini varış (sn) |
| seg | int | Ölçülen segment indeksi (0–5) |
| lap | int | Tamamlanan tam tur sayısı |

**Örnek:**
```
<T,175,165,42.3,0,0,CENTER,0,0,empty,340,338,PID_FOLLOW,calibration,1,2,3,18,2,0
```

**Gönderim aralığı:** 1000 ms (SoftwareSerial yarı-çift yönlü; uzun aralık RX kesilmesini önler)

---

## Arduino → ESP: Segment Event (ölçüm/kalibrasyon modu)

```
>S,<segIdx>,<distCm>,<durationMs>,<avgPwm>,<lineLost>,<occLevel>,<distActLevel>,<lap>\n
```

| Alan | Tip | Açıklama |
|---|---|---|
| segIdx | int | Tamamlanan segment indeksi (0–5) |
| distCm | float | Segment mesafesi (encoder, cm) — manuel mesafe düşülmüş |
| durationMs | unsigned long | Segment süresi (ms) — manuel duraklamalar düşülmüş |
| avgPwm | int | Segment boyunca ortalama motor PWM |
| lineLost | int | Segmentte çizgi kaybedildi mi (0/1) |
| occLevel | int | Doluluk seviyesi (0=boş, 1=yarı, 2=dolu) |
| distActLevel | int | Mesafe aksiyon seviyesi (0=normal, 1=yavaş, 2=çok yavaş, 3=dur) |
| lap | int | Tur numarası |

Arduino bir istasyondan diğerine geçişi tamamlayınca (Hall→Hall) gönderir.
ESP bunu `mlTrainingData/{carId}` altına POST eder; web paneli (TensorFlow.js)
bu örneklerle ETA modelini eğitir. **Manuel modda ölçüm duraklatılır**, çıkınca
kaldığı yerden devam eder.

**Örnek:**
```
>S,0,124.5,8200,182,0,1,0,3
```
