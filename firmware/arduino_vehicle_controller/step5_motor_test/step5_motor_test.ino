// =====================================================================
// MEGABUS — Step 5: L298N Motor Testi
//
// Pin planı:
//   D5  = ENA → Sağ motor PWM
//   D6  = ENB → Sol motor PWM
//   D7  = IN1 → Sağ yön A
//   D8  = IN2 → Sağ yön B
//   D12 = IN3 → Sol yön A
//   D13 = IN4 → Sol yön B
//
// L298N ENA/ENB jumper'ları ÇIKARILMIŞ olmalı (PWM D5/D6'dan geliyor).
// Motor gücü harici bataryadan; L298N 5V pini Arduino'ya BAĞLANMAZ.
//
// Yön ters dönüyorsa ilgili define'ı true yap:
#define INVERT_RIGHT true
#define INVERT_LEFT  false
//
// Serial Monitor: 115200 baud
// Komutlar (büyük/küçük fark etmez):
//   f → İleri (2 sn)
//   b → Geri  (2 sn)
//   l → Sol pivot (2 sn)
//   r → Sağ pivot (2 sn)
//   s → Dur
//   + → Hızı artır (50 adım)
//   - → Hızı azalt (50 adım)
// =====================================================================

#define PIN_ENA  5    // Sağ PWM
#define PIN_ENB  6    // Sol PWM
#define PIN_IN1  7    // Sağ A
#define PIN_IN2  8    // Sağ B
#define PIN_IN3  12   // Sol A
#define PIN_IN4  13   // Sol B

const int MIN_PWM  = 100;   // Motorun dönmeye başladığı minimum
const int MAX_PWM  = 255;   // analogWrite max (Arduino Uno 8-bit)
int speed = 180;

// ---------- düşük seviye ----------

void setRight(bool fwd, int pwm) {
  bool a = INVERT_RIGHT ? !fwd : fwd;
  digitalWrite(PIN_IN1, a  ? HIGH : LOW);
  digitalWrite(PIN_IN2, a  ? LOW  : HIGH);
  analogWrite(PIN_ENA, constrain(pwm, 0, MAX_PWM));
}

void setLeft(bool fwd, int pwm) {
  bool a = INVERT_LEFT ? !fwd : fwd;
  digitalWrite(PIN_IN3, a  ? HIGH : LOW);
  digitalWrite(PIN_IN4, a  ? LOW  : HIGH);
  analogWrite(PIN_ENB, constrain(pwm, 0, MAX_PWM));
}

void stopMotors() {
  analogWrite(PIN_ENA, 0);
  analogWrite(PIN_ENB, 0);
  digitalWrite(PIN_IN1, LOW); digitalWrite(PIN_IN2, LOW);
  digitalWrite(PIN_IN3, LOW); digitalWrite(PIN_IN4, LOW);
}

// ---------- hareketler ----------

void driveForward(int pwm)  { setRight(true,  pwm); setLeft(true,  pwm); }
void driveBackward(int pwm) { setRight(false, pwm); setLeft(false, pwm); }
void pivotLeft(int pwm)     { setRight(true,  pwm); setLeft(false, pwm); }
void pivotRight(int pwm)    { setRight(false, pwm); setLeft(true,  pwm); }

void runFor(void (*fn)(int), int pwm, unsigned long ms) {
  fn(pwm);
  delay(ms);
  stopMotors();
}

void printStatus(const char* cmd) {
  Serial.print(F(">> "));
  Serial.print(cmd);
  Serial.print(F("  speed="));
  Serial.println(speed);
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {}
  delay(200);

  pinMode(PIN_IN1, OUTPUT); pinMode(PIN_IN2, OUTPUT);
  pinMode(PIN_IN3, OUTPUT); pinMode(PIN_IN4, OUTPUT);
  pinMode(PIN_ENA, OUTPUT); pinMode(PIN_ENB, OUTPUT);
  stopMotors();

  Serial.println(F("=== MEGABUS Step 5: Motor Test ==="));
  Serial.println(F("f=ileri  b=geri  l=sol  r=sag  s=dur  +=hiz+  -=hiz-"));
  Serial.print(F("Baslangic hizi: ")); Serial.println(speed);
  Serial.println(F("NOT: Motorlar 2 sn calısıp durur."));
  Serial.println();
}

void loop() {
  if (!Serial.available()) return;
  char c = tolower(Serial.read());

  switch (c) {
    case 'f':
      printStatus("ILERI");
      runFor(driveForward, speed, 2000);
      break;
    case 'b':
      printStatus("GERI");
      runFor(driveBackward, speed, 2000);
      break;
    case 'l':
      printStatus("SOL PIVOT");
      runFor(pivotLeft, speed, 2000);
      break;
    case 'r':
      printStatus("SAG PIVOT");
      runFor(pivotRight, speed, 2000);
      break;
    case 's':
      stopMotors();
      Serial.println(F(">> DUR"));
      break;
    case '+':
      speed = min(speed + 50, MAX_PWM);
      Serial.print(F(">> Hiz: ")); Serial.println(speed);
      break;
    case '-':
      speed = max(speed - 50, MIN_PWM);
      Serial.print(F(">> Hiz: ")); Serial.println(speed);
      break;
    default:
      break;
  }
}
