// =====================================================================
// MEGABUS — Step 1: I2C Scanner
// Yükle, Serial Monitor'ü 115200 baud'a aç, çıktıyı bana ilet.
// Beklenen adresler: 0x20 (PCF8574 QTR) ve 0x48 (ADS1115 FSR)
// =====================================================================

#include <Wire.h>

void setup() {
  Serial.begin(115200);
  while (!Serial) {}
  delay(200);

  Wire.begin();  // Arduino Uno: SDA=A4, SCL=A5
  Serial.println(F("=== MEGABUS I2C Scanner ==="));
  Serial.println(F("Scanning 0x01 - 0x7F ..."));
  Serial.println();

  uint8_t found = 0;

  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    uint8_t err = Wire.endTransmission();

    if (err == 0) {
      Serial.print(F("  [OK]  0x"));
      if (addr < 16) Serial.print('0');
      Serial.print(addr, HEX);

      if (addr == 0x20) Serial.print(F("  <-- PCF8574 (QTR sensörler)"));
      if (addr == 0x48) Serial.print(F("  <-- ADS1115 (FSR doluluk)"));
      Serial.println();
      found++;
    } else if (err == 4) {
      Serial.print(F("  [ERR] 0x"));
      if (addr < 16) Serial.print('0');
      Serial.println(addr, HEX);
    }
  }

  Serial.println();
  Serial.print(F("Toplam bulunan cihaz: "));
  Serial.println(found);

  if (found == 0) {
    Serial.println(F("HATA: Hicbir cihaz bulunamadi."));
    Serial.println(F("  -> SDA=A4, SCL=A5 baglantilarini kontrol et."));
    Serial.println(F("  -> Ortak GND oldugundan emin ol."));
    Serial.println(F("  -> PCF8574 VCC=5V, ADS1115 VCC=3.3V veya 5V."));
  }

  bool pcfOk  = false;
  bool adsOk  = false;

  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      if (addr == 0x20) pcfOk = true;
      if (addr == 0x48) adsOk = true;
    }
  }

  Serial.println();
  Serial.print(F("PCF8574 @ 0x20 : "));
  Serial.println(pcfOk  ? F("BULUNDU ✓") : F("YOK ✗"));
  Serial.print(F("ADS1115 @ 0x48 : "));
  Serial.println(adsOk  ? F("BULUNDU ✓") : F("YOK ✗"));
  Serial.println();
  Serial.println(F("Sonucu Claude'a ilet, bir sonraki adima gecilecek."));
}

void loop() {
  // Tek sefer scan, loop'ta bir sey yok.
}
