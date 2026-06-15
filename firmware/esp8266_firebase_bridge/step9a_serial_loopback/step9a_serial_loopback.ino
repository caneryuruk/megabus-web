// =====================================================================
// MEGABUS — Step 9a: Arduino ↔ ESP Serial Loopback Testi
//
// Arduino tarafı: step8_linefollow_test yüklü olmalı (Serial üzerinden
//                 telemetry (<T,...) basıyor, 9600 baud SoftwareSerial).
//
// Bu sketch ESP'ye yüklenir:
//   - Arduino'dan gelen her satırı Serial1 (GPIO2/D4) debug'a basar
//   - Serial1'e gelen her karakteri Arduino'ya iletir
//
// Serial1 için USB-TTL adaptörü veya ikinci bir Arduino'nun
// TX/RX'ini kullanabilirsin. Eğer yoksa sadece Serial1.print
// satırlarını göremezsin ama Arduino → ESP yönü çalışıyorsa
// ESP kendi Serial'inden okuduğunda sorun çıkmaz.
//
// Bağlantı (hatırlatma):
//   ESP TX (D10'a) → 1k → Arduino D10 RX → 2k → GND (voltaj bölücü)
//   Arduino D11 TX → ESP RX (direkt, 3.3V uyumlu)
//   BAUD: 9600
//
// Serial1 (debug): GPIO2 / D4 pini → USB-TTL RX
// =====================================================================

#define BAUD_ARDUINO  9600
#define BAUD_DEBUG    115200

// Arduino'dan gelen satır buffer'ı
char buf[96];
uint8_t idx = 0;

void setup() {
  // Hardware UART ↔ Arduino
  Serial.begin(BAUD_ARDUINO);

  // Debug UART (GPIO2/D4) — opsiyonel USB-TTL ile izle
  Serial1.begin(BAUD_DEBUG);
  delay(100);

  Serial1.println(F("\n=== MEGABUS Step 9a: Serial Loopback ==="));
  Serial1.println(F("Arduino'dan gelen satirlar asagida gorunecek."));
  Serial1.println(F("Hicbir sey gelmiyorsa: kablo + baud (9600) kontrol et."));
  Serial1.println();
}

void loop() {
  // Arduino → ESP: her satırı debug'a bas
  while (Serial.available()) {
    char c = (char)Serial.read();
    Serial1.write(c);   // direkt yansıt

    if (c == '\n') {
      buf[idx] = '\0';
      // Telemetry satırı mı?
      if (idx > 3 && buf[0] == '<' && buf[1] == 'T') {
        Serial1.print(F("  [TELEMETRİ OK] "));
        Serial1.println(buf);
      }
      idx = 0;
    } else if (idx < sizeof(buf) - 1) {
      buf[idx++] = c;
    }
  }

  // Serial1 debug'dan geleni Arduino'ya ilet (manuel test için)
  while (Serial1.available()) {
    char c = (char)Serial1.read();
    Serial.write(c);
    Serial1.print(F("[ESP→ARD] ")); Serial1.println(c);
  }
}
