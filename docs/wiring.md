# MEGABUS — Kablo Bağlantıları

## Güç Dağıtımı

```
2× Li-ion seri → LM2596 Buck → 5V
                               ├── Arduino Uno VIN (veya 5V pini)
                               ├── ESP8266 VIN/5V
                               ├── PCF8574 VCC
                               ├── HC-SR04 VCC
                               └── L298N motor güç girişi (VS)

UYARI: L298N'in "5V çıkış" pini Arduino'ya BAĞLANMAZ.
       ESP8266, Arduino'nun 3.3V pininden BESLENMİR — harici 5V'dan.
```

**Ortak GND — ŞART:**
Batarya (−), Buck GND, Arduino GND, ESP GND, L298N GND, PCF8574 GND,
HC-SR04 GND, QTR sensör GND, Encoder GND, Hall GND, ADS1115 GND
→ hepsi tek ortak noktada buluşmalı.

---

## Arduino Uno Pin Tablosu

| Arduino Pin | Bağlantı | Açıklama |
|---|---|---|
| A0 | HC-SR04 TRIG | Mesafe tetik |
| A1 | HC-SR04 ECHO | Mesafe yankı |
| A4 | I2C SDA | PCF8574 + ADS1115 |
| A5 | I2C SCL | PCF8574 + ADS1115 |
| D2 | Sağ encoder OUT | INT0 |
| D3 | Sol encoder OUT | INT1 |
| D4 | Hall sensörü OUT | INPUT_PULLUP |
| D5 | L298N ENA | Sağ motor PWM |
| D6 | L298N ENB | Sol motor PWM |
| D7 | L298N IN1 | Sağ yön A |
| D8 | L298N IN2 | Sağ yön B |
| D10 | SoftwareSerial RX | ← ESP TX (voltaj bölücü üzerinden) |
| D11 | SoftwareSerial TX | → ESP RX (3.3V uyumlu, direkt) |
| D12 | L298N IN3 | Sol yön A |
| D13 | L298N IN4 | Sol yön B |

---

## PCF8574 @ 0x20

| PCF Pin | Bağlantı |
|---|---|
| P0 | QTR Sol (S0) |
| P1 | QTR Sol-Orta (S1) |
| P2 | QTR Orta (S2) |
| P3 | QTR Sağ-Orta (S3) |
| P4 | QTR Sağ (S4) |
| P5–P7 | Boş |
| A0/A1/A2 | GND'e çekili → adres 0x20 |
| VCC | 5V |
| GND | Ortak GND |

**Sensör mantığı:** Siyah çizgi → bit = 0 (LOW). Beyaz zemin → bit = 1 (HIGH).

---

## ADS1115 @ 0x48

| ADS1115 Pin | Bağlantı |
|---|---|
| VDD | 3.3V veya 5V |
| GND | Ortak GND |
| SCL | Arduino A5 |
| SDA | Arduino A4 |
| ADDR | GND → adres 0x48 |
| A0 | FSR 1 çıkışı |
| A1 | FSR 2 çıkışı |

---

## L298N Motor Sürücü

| L298N Pin | Bağlantı |
|---|---|
| ENA | Arduino D5 (PWM) — jumper çıkarık |
| ENB | Arduino D6 (PWM) — jumper çıkarık |
| IN1 | Arduino D7 |
| IN2 | Arduino D8 |
| IN3 | Arduino D12 |
| IN4 | Arduino D13 |
| OUT1/OUT2 | Sağ motor |
| OUT3/OUT4 | Sol motor |
| VS | Batarya + (motor gücü) |
| VSS | 5V (mantık gücü) |
| GND | Ortak GND |

**Yön:** `INVERT_RIGHT = true`, `INVERT_LEFT = false` (Step 5 testinde belirlendi).

---

## ESP8266 ↔ Arduino Voltaj Bölücü

Arduino D11 (TX, 5V) → ESP RX (3.3V) için:

```
Arduino D11 ──┬── 1kΩ ──── ESP RX
              └── 2kΩ ──── GND
```

ESP TX (3.3V) → Arduino D10 (RX): **Direkt bağlantı**, voltaj bölücü gerekmez.

---

## HC-SR04

| HC-SR04 | Arduino |
|---|---|
| VCC | 5V |
| GND | GND |
| TRIG | A0 |
| ECHO | A1 |

---

## Hall Sensörü

| Hall | Arduino |
|---|---|
| VCC | 5V |
| GND | GND |
| OUT | D4 (INPUT_PULLUP) |

Mıknatıs yaklaşınca OUT → LOW (HALL_ACTIVE_LOW = true).

---

## Encoder (×2)

| Encoder | Arduino |
|---|---|
| VCC | 5V |
| GND | GND |
| OUT (Sağ) | D2 |
| OUT (Sol) | D3 |
