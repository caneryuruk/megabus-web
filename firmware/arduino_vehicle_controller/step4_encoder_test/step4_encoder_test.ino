// =====================================================================
// MEGABUS — Step 4: Encoder Interrupt Testi
//
// Bağlantı:
//   Sağ encoder OUT → D2  (INT0)
//   Sol encoder OUT → D3  (INT1)
//   VCC → 5V,  GND → GND
//
// Tekerlekleri elle çevir veya motoru kısa süre döndür.
// Her iki encoder de sayı artırmalı.
//
// Serial Monitor: 115200 baud
// =====================================================================

#define PIN_ENC_RIGHT 2   // INT0
#define PIN_ENC_LEFT  3   // INT1

// Tekerlek parametreleri (asıl firmware ile aynı)
const float WHEEL_DIAMETER_CM       = 6.5;
const int   TICKS_PER_REVOLUTION    = 20;

volatile long rightTicks = 0;
volatile long leftTicks  = 0;

void onRightTick() { rightTicks++; }
void onLeftTick()  { leftTicks++;  }

float ticksToCm(long ticks) {
  return (ticks * 3.14159f * WHEEL_DIAMETER_CM) / TICKS_PER_REVOLUTION;
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {}
  delay(200);

  pinMode(PIN_ENC_RIGHT, INPUT_PULLUP);
  pinMode(PIN_ENC_LEFT,  INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(PIN_ENC_RIGHT), onRightTick, RISING);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_LEFT),  onLeftTick,  RISING);

  Serial.println(F("=== MEGABUS Step 4: Encoder Test ==="));
  Serial.println(F("Sag encoder=D2 (INT0)  Sol encoder=D3 (INT1)"));
  Serial.println(F("Tekerlekleri elle cevir veya motoru kisaca dondur."));
  Serial.println(F("Format: R_ticks  L_ticks | R_cm   L_cm"));
  Serial.println(F("Serial'e 'r' yazarsan sayaclar sifirlanir."));
  Serial.println();
}

void loop() {
  // Sıfırlama komutu
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'r' || c == 'R') {
      noInterrupts();
      rightTicks = 0;
      leftTicks  = 0;
      interrupts();
      Serial.println(F("--- Sifirlandı ---"));
    }
  }

  long r, l;
  noInterrupts();
  r = rightTicks;
  l = leftTicks;
  interrupts();

  Serial.print(F("R:"));
  Serial.print(r);
  Serial.print(F("  L:"));
  Serial.print(l);
  Serial.print(F(" | R:"));
  Serial.print(ticksToCm(r), 1);
  Serial.print(F("cm  L:"));
  Serial.print(ticksToCm(l), 1);
  Serial.println(F("cm"));

  delay(300);
}
