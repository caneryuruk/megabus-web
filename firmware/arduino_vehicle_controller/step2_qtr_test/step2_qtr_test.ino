// =====================================================================
// MEGABUS — Step 2: QTR Sensör Okuma Testi (PCF8574 üzerinden)
//
// PCF8574 @ 0x20
//   P0 = QTR Sol      (S0)
//   P1 = QTR Sol-Orta (S1)
//   P2 = QTR Orta     (S2)
//   P3 = QTR Sağ-Orta (S3)
//   P4 = QTR Sağ      (S4)
//   P5-P7 = boş (henüz kullanılmıyor)
//
// Beklenti: sensör SİYAH çizgi üzerindeyken ilgili bit 0 (LOW),
//           BEYAZ/boş zeminde bit 1 (HIGH).
// Eğer tersiyse INVERT_SENSORS true yap → bitler ters yorumlanır.
//
// Serial Monitor: 115200 baud, satır sonu: Her ikisi (CR+LF)
// =====================================================================

#include <Wire.h>

const uint8_t PCF_ADDR  = 0x20;

// Sensör bit pozisyonları
const uint8_t BIT_S0 = 0;  // Sol
const uint8_t BIT_S1 = 1;  // Sol-Orta
const uint8_t BIT_S2 = 2;  // Orta
const uint8_t BIT_S3 = 3;  // Sağ-Orta
const uint8_t BIT_S4 = 4;  // Sağ

// Eğer sensörler beyazta 0, siyahta 1 dönüyorsa true yap:
const bool INVERT_SENSORS = false;

uint8_t pcfRead() {
  Wire.requestFrom(PCF_ADDR, (uint8_t)1);
  return Wire.available() ? Wire.read() : 0xFF;
}

bool sensorBlack(uint8_t raw, uint8_t bit) {
  bool bitVal = bitRead(raw, bit);
  // default: 0 = siyah, 1 = beyaz
  return INVERT_SENSORS ? bitVal : !bitVal;
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {}
  delay(200);

  Wire.begin();

  // PCF8574: P0-P4 giriş olarak kullanmak için 0xFF yaz
  Wire.beginTransmission(PCF_ADDR);
  Wire.write(0xFF);
  Wire.endTransmission();

  Serial.println(F("=== MEGABUS Step 2: QTR Sensor Test ==="));
  Serial.println(F("Sensoru cizgi uzerine getir / beyaz zemine al."));
  Serial.println(F("Format: [raw_hex] S0 S1 S2 S3 S4 | PATTERN | LINE_CASE"));
  Serial.println(F("S=B(siyah) W(beyaz)"));
  Serial.println();
}

// Belirlenen 5-sensör lineCase'i hesapla (çizgi takip mantığı için referans)
// 0=beyaz, 1=siyah (sensorBlack() sonucu)
String computeLineCase(bool s0, bool s1, bool s2, bool s3, bool s4) {
  int black = (s0?1:0)+(s1?1:0)+(s2?1:0)+(s3?1:0)+(s4?1:0);

  if (black == 0) return "LOST(none)";
  if (black == 5) return "ALL_BLACK";

  // Orta ağırlıklı merkez kontrolü
  if (!s0 && !s1 && s2 && !s3 && !s4) return "CENTER";
  if (!s0 && s1 && s2 && !s3 && !s4)  return "SLIGHT_LEFT";
  if (!s0 && !s1 && s2 && s3 && !s4)  return "SLIGHT_RIGHT";
  if (s0 && s1 && !s2 && !s3 && !s4)  return "HARD_LEFT";
  if (!s0 && !s1 && !s2 && s3 && s4)  return "HARD_RIGHT";
  if (s0 && !s1 && !s2 && !s3 && !s4) return "FAR_LEFT";
  if (!s0 && !s1 && !s2 && !s3 && s4) return "FAR_RIGHT";
  if (s0 && s1 && s2 && !s3 && !s4)   return "LEFT_HEAVY";
  if (!s0 && !s1 && s2 && s3 && s4)   return "RIGHT_HEAVY";

  // Genel PID hata skoru (ağırlık merkezi -2..+2)
  float weight = 0;
  int cnt = 0;
  const float w[5] = {-2.0, -1.0, 0.0, 1.0, 2.0};
  bool s[5] = {s0,s1,s2,s3,s4};
  for (int i=0;i<5;i++) if(s[i]){weight+=w[i];cnt++;}
  if (cnt == 0) return "LOST(calc)";
  float pos = weight / cnt;
  if (pos < -1.0) return "LEFT";
  if (pos > 1.0)  return "RIGHT";
  return "NEAR_CENTER";
}

void loop() {
  uint8_t raw = pcfRead();

  bool s0 = sensorBlack(raw, BIT_S0);
  bool s1 = sensorBlack(raw, BIT_S1);
  bool s2 = sensorBlack(raw, BIT_S2);
  bool s3 = sensorBlack(raw, BIT_S3);
  bool s4 = sensorBlack(raw, BIT_S4);

  // Ham byte (hex)
  Serial.print(F("[0x"));
  if (raw < 0x10) Serial.print('0');
  Serial.print(raw, HEX);
  Serial.print(F("] "));

  // Sensör değerleri
  Serial.print(s0 ? F("B ") : F("W "));
  Serial.print(s1 ? F("B ") : F("W "));
  Serial.print(s2 ? F("B ") : F("W "));
  Serial.print(s3 ? F("B ") : F("W "));
  Serial.print(s4 ? F("B ") : F("W "));

  // Görsel çizgi pattern (5 karakter)
  Serial.print(F("| "));
  Serial.print(s0 ? '#' : '.');
  Serial.print(s1 ? '#' : '.');
  Serial.print(s2 ? '#' : '.');
  Serial.print(s3 ? '#' : '.');
  Serial.print(s4 ? '#' : '.');
  Serial.print(F(" | "));

  Serial.println(computeLineCase(s0,s1,s2,s3,s4));

  delay(150);
}
