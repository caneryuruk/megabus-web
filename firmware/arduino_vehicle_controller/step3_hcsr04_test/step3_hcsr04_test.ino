// =====================================================================
// MEGABUS — Step 3: HC-SR04 Mesafe Sensörü Testi
//
// Bağlantı:
//   TRIG → A0
//   ECHO → A1
//   VCC  → 5V
//   GND  → GND
//
// Serial Monitor: 115200 baud
// =====================================================================

#define PIN_TRIG A0
#define PIN_ECHO A1

// Mesafe eşikleri (asıl firmware ile aynı)
const float DIST_STOP      =  8.0;  // cm → dur
const float DIST_VERY_SLOW = 18.0;  // cm → çok yavaş
const float DIST_SLOW      = 35.0;  // cm → yavaş

float readDistanceCm() {
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);

  // 18000 µs timeout ≈ ~310 cm max; ötesi NO_READ
  unsigned long duration = pulseIn(PIN_ECHO, HIGH, 18000);
  if (duration == 0) return -1.0;
  return duration / 58.0;
}

const char* distanceAction(float cm) {
  if (cm < 0)             return "NO_READ";
  if (cm <= DIST_STOP)    return "STOP";
  if (cm <= DIST_VERY_SLOW) return "VERY_SLOW";
  if (cm <= DIST_SLOW)    return "SLOW";
  return "NORMAL";
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {}
  delay(200);

  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  digitalWrite(PIN_TRIG, LOW);

  Serial.println(F("=== MEGABUS Step 3: HC-SR04 Test ==="));
  Serial.println(F("TRIG=A0  ECHO=A1"));
  Serial.println(F("Onune engel getir / uzaklastir."));
  Serial.println(F("Format: dist_cm | ACTION"));
  Serial.println();
}

void loop() {
  float cm = readDistanceCm();
  const char* act = distanceAction(cm);

  if (cm < 0) {
    Serial.println(F("NO_READ  (nesne yok veya cok uzak >310cm)"));
  } else {
    Serial.print(cm, 1);
    Serial.print(F(" cm  | "));
    Serial.println(act);
  }

  delay(200);
}
