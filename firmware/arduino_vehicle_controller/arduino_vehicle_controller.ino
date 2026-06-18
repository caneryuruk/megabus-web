// =====================================================================
// MEGABUS — Arduino Vehicle Controller  v1
//
// Donanım:
//   PCF8574 @0x20  : 5x QTR sensör (P0-P4)
//   ADS1115 @0x48  : 2x FSR doluluk sensörü (CH0, CH1)
//   HC-SR04        : TRIG=A0, ECHO=A1
//   Encoder Sağ    : D2 (INT0)
//   Encoder Sol    : D3 (INT1)
//   Hall sensörü   : D4
//   L298N          : ENA=D5(PWM), ENB=D6(PWM)
//                    IN1=D7, IN2=D8, IN3=D12, IN4=D13
//   ESP8266        : SoftwareSerial RX=D10, TX=D11
//
// v1 kapsamı: AUTO çizgi takip + Manuel kontrol + Hall istasyon durağı
// v2'de eklenecek: rota kalibrasyonu, recovery, service, ML pipeline
// =====================================================================

#include <Wire.h>
#include <SoftwareSerial.h>
#include <Adafruit_ADS1X15.h>

// ==================== YÖN TERSLEME ====================
// Step 5 testinde belirlendi:
#define INVERT_RIGHT true
#define INVERT_LEFT  false

// Step 2 testinde belirlendi: 0=siyah, 1=beyaz → tersleme yok
#define INVERT_SENSORS false

// Hall sensörü mıknatıs gelince LOW çekiyor mu?
#define HALL_ACTIVE_LOW true

// ==================== PİNLER ====================
#define PIN_TRIG      A0
#define PIN_ECHO      A1
#define PIN_ENC_R     2     // INT0
#define PIN_ENC_L     3     // INT1
#define PIN_HALL      4
#define PIN_ENA       5     // Sağ motor PWM
#define PIN_ENB       6     // Sol motor PWM
#define PIN_IN1       7     // Sağ A
#define PIN_IN2       8     // Sağ B
#define PIN_IN3       12    // Sol A
#define PIN_IN4       13    // Sol B
#define PIN_ESP_RX    10
#define PIN_ESP_TX    11

// ==================== PCF8574 ====================
#define PCF_ADDR   0x20
#define BIT_S0     0    // Sol
#define BIT_S1     1    // Sol-Orta
#define BIT_S2     2    // Orta
#define BIT_S3     3    // Sağ-Orta
#define BIT_S4     4    // Sağ

// ==================== ADS1115 ====================
#define ADS_ADDR   0x48

// ==================== VARSAYILAN AYARLAR ====================
// Firmware sürümü — boot ve debug çıktısında görünür; doğru firmware yüklü mü diye bak.
#define FW_VERSION "v3.5-dist3stage"
#define DEFAULT_BASE_SPEED   110    // 0-255. Düz gidiş hızı -25 (135→110, kullanıcı: düz çok hızlı). NOT: 130 altı; takılırsa yükselt.
#define DEFAULT_MAX_SPEED    255
// ORANSAL PD: Kp = hatayla orantılı düzeltme (yüksek = sıkı takip ama overshoot riski),
// Kd = sönümleme (overshoot/zigzag'ı azaltır). Panelden ayarlanır VE artık gerçekten kullanılır.
// KULLANICI ONAYLADI (mükemmel/smooth): Kp=9, Kd=51. AKSİ SÖYLENMEDİKÇE DOKUNMA.
#define DEFAULT_KP            9.0f
#define DEFAULT_KI            0.0f
#define DEFAULT_KD           51.0f
#define DEFAULT_FSR_THRESHOLD 1000  // ADS ham değer (0-32767 arası)

// Motor minimum hareket PWM. Eskiden 110'du ama bu, NAZİK dönüşte iç tekeri tabana
// yapıştırıp durduruyordu (kullanıcı şikayeti). Düşürdük (85) ki iç teker dönüşte
// YAVAŞLAYABİLSİN ama dönmeye devam etsin. Kalkış takılması STARTUP_BOOST ile çözülüyor
// (teker zaten yuvarlanırken 85 PWM onu döndürmeye yeter; duran tekeri boost kırar).
#define MIN_MOVE_PWM    95    // düz hız 110'a indi → floor de düştü ki takılmasın (boost kalkışı kırar)
// Başlangıç boost (durağan motorları kırmak için kısa darbe). Floor düştüğü için süreyi
// 90→120ms uzattık ki 0'dan kalkış garanti olsun.
#define STARTUP_BOOST_PWM  195
#define STARTUP_BOOST_MS   120

// ---- AYRIK DİREKSİYON ŞEKİLLENDİRME (kullanıcı isteği) ----
// Orta-yakın sensör (s1/s3, merkezle birlikte olsa da): iç teker YAVAŞLAR (durmaz),
// dış teker NORMAL hızda kalır. Hız farkını aşırı açma → küçük tut.
// NOT: Ayrık (GENTLE/SHARP) direksiyon şekillendirme KALDIRILDI. Direksiyon artık gerçek
// ORANSAL PD kontrol (Kp/Kd ile, centroid hata üzerinden) — movePIDFollow'a bak.
// Yumuşak hızlanma: dönüşten sonra ANİ hız sıçraması zigzag yapar. Taban hız RAMP_START'tan
// baseSpeed'e, her RAMP_INTERVAL_MS'de RAMP_STEP PWM kademeli yükselir. Keskin dönüşte sıfırlanır.
#define RAMP_START       100   // başlangıç hızı (taban 110'un altında, kademeli 110'a çıkar)
#define RAMP_STEP          3
#define RAMP_INTERVAL_MS  20

// Dönüş PWM sabitleri (ALL_BLACK ve LOST senaryoları için)
#define SOFT_TURN_PWM    125
#define MEDIUM_TURN_PWM  155
#define HARD_TURN_PWM    195

// Sideonly (tek yan sensör) evre süreleri
#define SOFT_STAGE_MS    400
#define MEDIUM_STAGE_MS  1000

// Çizgi kayıp arama evre süreleri
#define LOST_SOFT_MS     300
#define LOST_MEDIUM_MS   800

// Hall istasyon durağı
#define HALL_DEBOUNCE_MS     800
#define STATION_STOP_MS     3000

// Tekerlek parametreleri (encoder → cm dönüşümü)
#define WHEEL_DIAMETER_CM    6.5f
#define TICKS_PER_REV        20

// İnterval'lar (ms)
#define DIST_INTERVAL_MS      150
#define FSR_INTERVAL_MS       250
#define TELEMETRY_INTERVAL_MS 1000   // SoftwareSerial half-duplex: TX sırasında RX sağır
                                     // kalır; uzun aralık = komut satırları kesilmez
#define DEBUG_INTERVAL_MS     500

// Güvenlik: ESP'den bu kadar süre komut gelmezse (ESP çökmesi/kablo) motoru durdur.
// (WiFi kesilince ESP zaten STOP gönderir; bu watchdog ESP'nin tamamen susmasına karşı.)
// 3sn marj: ESP HTTPS ile meşgulken komut göndermesi gecikebilir, yanlış durmasın.
#define CMD_TIMEOUT_MS        3500

// Mesafe sensörü 3 KADEME: >DIST_SLOW_CM normal, DIST_STOP_CM..DIST_SLOW_CM yavaş, <=DIST_STOP_CM DUR
#define DIST_STOP_CM      12.0f  // bu kadar yakına gelince DUR (8→12: daha güvenli marj)
#define DIST_SLOW_CM      30.0f  // bu mesafeden itibaren YAVAŞLA
#define DIST_VERY_SLOW_CM 18.0f  // (kullanılmıyor; ETA için sabit duruyor)
#define VERY_SLOW_SPEED    95    // ETA hız ölçeği için (taban altı)
#define SLOW_SPEED        100    // YAVAŞ kademe hızı — taban(110) altında ki gerçekten yavaşlasın

// ==================== GLOBAL NESNELER ====================
SoftwareSerial espSerial(PIN_ESP_RX, PIN_ESP_TX);
Adafruit_ADS1115 ads;

// ==================== DURUM DEĞİŞKENLERİ ====================

// Sensörler
bool  s[5]        = {false};   // QTR sensör sonuçları (true=siyah)
uint8_t rawPCF    = 0xFF;
float distCm      = -1.0f;
bool  distStop    = false;

// PID
float Kp = DEFAULT_KP;
float Ki = DEFAULT_KI;
float Kd = DEFAULT_KD;
float pidError     = 0;
float lastPidError = 0;
float pidIntegral  = 0;
float pidCorrection = 0;
float lineFilt     = 0;   // yumuşatılmış çizgi konumu (EMA) — kademeli sinyali süzer
float derivFilt    = 0;   // yumuşatılmış türev (D terimi) — yüksek Kd'nin dönüşteki tekmesini keser
const float PID_INTEGRAL_LIMIT = 50.0f;
const float PID_CORRECTION_LIMIT = 120.0f;
// Çizgi konumu EMA katsayısı: yeni okuma ağırlığı = (1 - LINE_FILTER). Düşük = daha çok
// yumuşatma/gecikme, yüksek = daha hızlı tepki/az yumuşatma. 0.6 hafif süzme (kademeli
// sensör zıplamasını ve türev tekmesini azaltır, gerçek dönüşlerde gecikme ihmal edilebilir).
const float LINE_FILTER = 0.4f;   // hafif EMA: kademeli sensör jitter'ını yumuşat (yeni okuma %60)
// Türev (D) filtresi: dönüşte çizgi sensörden sensöre zıplarken türev sıçrar; yüksek Kd bunu
// büyütüp aracı çıldırtır. Bu low-pass o sıçramaları yumuşatır (düz yoldaki yavaş sönümlemeyi
// bozmadan). Yüksek = daha çok yumuşatma. 0.5 dengeli.
const float D_FILTER = 0.5f;
// Küçük ölü bölge: |hata| bunun altındayken düzeltme yapma. KÜÇÜK tut → çizgi merkezden çok
// kaymadan (1-1.5cm) sıkı takip; çok büyütürsen Hall mıknatısını kaçırır. 0 jitter'a tepki verir.
const float PID_DEADBAND = 0.25f;

// Motor
int configuredBaseSpeed = DEFAULT_BASE_SPEED;  // PID panelinden gelen taban hız
int baseSpeed   = DEFAULT_BASE_SPEED;          // mesafe ile anlık modüle edilen
int maxSpeed    = DEFAULT_MAX_SPEED;
int leftPWM     = 0;
int rightPWM    = 0;
unsigned long leftBoostUntil  = 0;
unsigned long rightBoostUntil = 0;
int lastLeftCmd  = 0;
int lastRightCmd = 0;

// Çizgi takip durumu
int   lastTurnDir   = 0;   // -1=sol, +1=sağ, 0=bilinmiyor
int   sideOnlyDir   = 0;
unsigned long sideOnlyStartMs = 0;
// Ayrık direksiyon: yumuşak hızlanma rampası
int   rampBase      = 0;          // kademeli yükselen taban hız
unsigned long lastRampMs = 0;
unsigned long lineLostStartMs = 0;
bool  lineFound     = false;
float linePosition  = 0.0f;  // -2..+2, ağırlık merkezi
char  lineCase[16]  = "LOST";
char  action[32]    = "WAITING";

// Encoder
volatile long encTicksR = 0;
volatile long encTicksL = 0;
long  lastEncR = 0;
long  lastEncL = 0;

// Hall / İstasyon
bool  hallActive       = false;
bool  lastHallActive   = false;
unsigned long lastHallTrigMs = 0;
bool  stationDetected  = false;
bool  stationPause     = false;
unsigned long stationPauseUntil = 0;
int   currentStop      = 0;   // 0-based istasyon indeksi

// ==================== ÖLÇÜM / KALİBRASYON ====================
#define STOP_COUNT 6          // dairesel rotada toplam durak (6 segment)

bool  measuring        = false;   // ilk durağa ulaşıldı mı? (ölçüm aktif)
bool  positionKnown    = false;   // konum güvenilir mi?
int   currentSegment   = 0;       // ölçülen segment (0..5): durak i → durak i+1
int   hallCount        = 0;       // kalibrasyon başından beri Hall sayısı
int   lapCount         = 0;       // tamamlanan tam tur sayısı

unsigned long segStartMs    = 0;  // segment başlangıç zamanı
long  segStartEncSum        = 0;  // segment başında encoder toplamı (L+R)

unsigned long lapStartMs      = 0;  // turun başladığı an
unsigned long lastLapDurationMs = 0;// son tamamlanan turun süresi

// Manuel askıya alma: manuelde + manuelden çıkınca ilk Hall'a kadar ölçme/ML yok.
// O Hall'da yeniden senkronize olup taze ölçmeye başlar (yarım segment atılır).
bool  measSuspended         = false;
int   expectedNextStop      = 0;  // ESP'den (>N): manuelden sonraki durak (1-based, 0=bilinmiyor)

// Segment istatistikleri (ML için)
long  segPwmSum             = 0;
unsigned int segPwmSamples  = 0;
bool  segLineLost           = false;

// Son tamamlanan segment (ESP'ye raporlanacak)
bool  segReportPending      = false;
int   lastSegIndex          = -1;
float lastSegDistanceCm     = 0;
unsigned long lastSegDurationMs = 0;
int   lastSegAvgPwm         = 0;
int   lastSegLineLost       = 0;

// Basit ETA: her segmentin son ölçülen süresi (sn) — web ML bunu rafine eder
unsigned long segLastDurationMs[STOP_COUNT] = {0};
int   etaSec                = 0;   // sıradaki durağa tahmini varış (sn)

// FSR / Doluluk
bool  adsReady        = false;
int   fsr1Raw         = 0;
int   fsr2Raw         = 0;
bool  fsr1Occupied    = false;
bool  fsr2Occupied    = false;
char  occupancyStatus[8]  = "empty";
char  occupancyLabel[8]   = "Bos";
char  occupancyColor[8]   = "green";

// Mod ve komut (ESP'den gelir)
char  vehicleMode[12] = "idle";   // idle | auto | manual | stopped
bool  manualEnabled   = false;
char  manualCmd[12]   = "STOP";
int   manualSpeed     = 0;
unsigned long lastCmdRecvMs = 0;  // son geçerli >C komutunun alındığı an (watchdog)

// Zamanlayıcılar
unsigned long lastDistMs      = 0;
unsigned long lastFsrMs       = 0;
unsigned long lastTelemetryMs = 0;
unsigned long lastDebugMs     = 0;

// ESP Serial alma buffer'ı
char  rxBuf[64];
uint8_t rxIdx = 0;

// --- ESP RX teşhis sayaçları ---
unsigned long espRxBytes = 0;       // D10'a gelen toplam bayt
unsigned long espRxLines = 0;       // tamamlanan satır sayısı
char  lastEspLine[64] = "(yok)";    // son alınan satır
char  espFwVersion[20] = "?";       // ESP'nin bildirdiği firmware sürümü (>V ile gelir)

// ==================== ISR ====================
void onEncR() { encTicksR++; }
void onEncL() { encTicksL++; }

// ==================== PCF8574 ====================
uint8_t pcfRead() {
  Wire.requestFrom((uint8_t)PCF_ADDR, (uint8_t)1);
  return Wire.available() ? Wire.read() : 0xFF;
}

bool sensorBlack(uint8_t raw, uint8_t bit) {
  bool v = bitRead(raw, bit);
  return INVERT_SENSORS ? v : !v;
}

// ==================== MOTOR KONTROL ====================
void applyMotorPins(bool rightFwd, int rightPwm, bool leftFwd, int leftPwm) {
  bool rA = INVERT_RIGHT ? !rightFwd : rightFwd;
  bool lA = INVERT_LEFT  ? !leftFwd  : leftFwd;

  digitalWrite(PIN_IN1, rA ? HIGH : LOW);
  digitalWrite(PIN_IN2, rA ? LOW  : HIGH);
  digitalWrite(PIN_IN3, lA ? HIGH : LOW);
  digitalWrite(PIN_IN4, lA ? LOW  : HIGH);

  analogWrite(PIN_ENA, constrain(rightPwm, 0, 255));
  analogWrite(PIN_ENB, constrain(leftPwm,  0, 255));
}

void stopMotors() {
  analogWrite(PIN_ENA, 0);
  analogWrite(PIN_ENB, 0);
  digitalWrite(PIN_IN1, LOW); digitalWrite(PIN_IN2, LOW);
  digitalWrite(PIN_IN3, LOW); digitalWrite(PIN_IN4, LOW);
  leftPWM = 0; rightPWM = 0;
  lastLeftCmd = 0; lastRightCmd = 0;
  leftBoostUntil = 0; rightBoostUntil = 0;
}

int applyMinPWM(int pwm) {
  if (pwm <= 0) return 0;
  return max(pwm, MIN_MOVE_PWM);
}

// İleri yönde, boost destekli PWM yaz
void driveForward(int leftCmd, int rightCmd) {
  unsigned long now = millis();

  if (leftCmd  > 0 && lastLeftCmd  == 0) leftBoostUntil  = now + STARTUP_BOOST_MS;
  if (rightCmd > 0 && lastRightCmd == 0) rightBoostUntil = now + STARTUP_BOOST_MS;

  int lActual = applyMinPWM(leftCmd);
  int rActual = applyMinPWM(rightCmd);

  if (leftCmd  > 0 && now < leftBoostUntil)  lActual = max(lActual, STARTUP_BOOST_PWM);
  if (rightCmd > 0 && now < rightBoostUntil) rActual = max(rActual, STARTUP_BOOST_PWM);

  leftPWM  = constrain(lActual, 0, 255);
  rightPWM = constrain(rActual, 0, 255);
  lastLeftCmd  = leftCmd;
  lastRightCmd = rightCmd;

  applyMotorPins(true, rightPWM, true, leftPWM);
}

// Manuel: yön parametreli, düz PWM (boost yok — kullanıcı isteği).
void driveManual(bool rightFwd, int rightCmd, bool leftFwd, int leftCmd) {
  leftPWM  = constrain(leftCmd,  0, 255);
  rightPWM = constrain(rightCmd, 0, 255);
  lastLeftCmd  = leftCmd;
  lastRightCmd = rightCmd;
  applyMotorPins(rightFwd, rightPWM, leftFwd, leftPWM);
}

// ==================== QTR / ÇİZGİ ====================
// Ağırlık merkezi hesapla (-2..+2), lineCase güncelle
void readLineSensors() {
  rawPCF = pcfRead();
  for (uint8_t i = 0; i < 5; i++) s[i] = sensorBlack(rawPCF, i);

  int activeCount = 0;
  float weightSum = 0;
  const float weights[5] = {-2.0f, -1.0f, 0.0f, 1.0f, 2.0f};
  for (uint8_t i = 0; i < 5; i++) {
    if (s[i]) { weightSum += weights[i]; activeCount++; }
  }

  if (activeCount == 0) {
    lineFound = false;
    linePosition = 0;
    strcpy(lineCase, "LOST");
    if (lineLostStartMs == 0) lineLostStartMs = millis();
    sideOnlyDir = 0;
    sideOnlyStartMs = 0;
    return;
  }

  lineLostStartMs = 0;
  lineFound = true;

  if (activeCount == 5) {
    linePosition = 0;
    strcpy(lineCase, "ALL_BLACK");
    sideOnlyDir = 0; sideOnlyStartMs = 0;
    return;
  }

  linePosition = weightSum / activeCount;

  // lastTurnDir güncelle
  if (linePosition < -0.2f) lastTurnDir = -1;
  else if (linePosition > 0.2f) lastTurnDir = 1;

  // Sideonly zamanlayıcı (tek taraflı sapma için)
  int newSideDir = (linePosition < -0.5f) ? -1 : (linePosition > 0.5f) ? 1 : 0;
  if (newSideDir != 0) {
    if (sideOnlyDir != newSideDir) { sideOnlyDir = newSideDir; sideOnlyStartMs = millis(); }
  } else {
    sideOnlyDir = 0; sideOnlyStartMs = 0;
  }

  // lineCase string
  if      (linePosition >= -0.3f && linePosition <= 0.3f)  strcpy(lineCase, "CENTER");
  else if (linePosition >= -0.8f && linePosition < -0.3f)  strcpy(lineCase, "SLIGHT_LEFT");
  else if (linePosition >  0.3f  && linePosition <= 0.8f)  strcpy(lineCase, "SLIGHT_RIGHT");
  else if (linePosition >= -1.5f && linePosition < -0.8f)  strcpy(lineCase, "LEFT");
  else if (linePosition >  0.8f  && linePosition <= 1.5f)  strcpy(lineCase, "RIGHT");
  else if (linePosition < -1.5f)                            strcpy(lineCase, "FAR_LEFT");
  else                                                       strcpy(lineCase, "FAR_RIGHT");
}

// Sideonly için kademeli dönüş evresi
uint8_t getSideStage() {
  if (sideOnlyStartMs == 0) return 0;  // SOFT
  unsigned long el = millis() - sideOnlyStartMs;
  if (el < SOFT_STAGE_MS)   return 0;  // SOFT
  if (el < MEDIUM_STAGE_MS) return 1;  // MEDIUM
  return 2;                             // HARD
}

// Çizgi kayıp arama evresi
uint8_t getLostStage() {
  if (lineLostStartMs == 0) return 0;
  unsigned long el = millis() - lineLostStartMs;
  if (el < LOST_SOFT_MS)   return 0;
  if (el < LOST_MEDIUM_MS) return 1;
  return 2;
}

// ==================== PID HAREKETİ ====================
void resetPID() {
  pidError = 0; lastPidError = 0; pidIntegral = 0; pidCorrection = 0;
  lineFilt = 0; derivFilt = 0;
}

// Eski çalışan ESP8266 kodunun mantığı (kanıtlanmış, zigzag yok): MERKEZ sensör çizgiyi
// gördüğü sürece DÜZ git (çok geniş ölü bölge). Düzeltmeyi yalnızca çizgi merkezden
// tamamen kayıp yan sensöre geçince yap. Hata kademeli: 0, ±1 (yan-orta), ±2 (en yan).
// s[i]=true → o sensör çizgide (siyah). s[2]=merkez, s[0]=en sol, s[4]=en sağ.
// Yumuşak hızlanma: taban hızı RAMP_START'tan baseSpeed'e kademeli yükselt (ani başlangıç yumuşar).
void rampUpBase() {
  if (rampBase < RAMP_START) rampBase = RAMP_START;
  if (millis() - lastRampMs >= RAMP_INTERVAL_MS) {
    lastRampMs = millis();
    if (rampBase < baseSpeed) rampBase += RAMP_STEP;
  }
  if (rampBase > baseSpeed) rampBase = baseSpeed;
}

void movePIDFollow() {
  // GERÇEK ORANSAL PD KONTROL (Kp/Kd panelden, GERÇEKTEN kullanılır — ayrık sürümde
  // kullanılmıyordu!). 5 sensör ağırlık merkezi (linePosition, -2..+2) = hata.
  //  - hafif EMA → kademeli jitter yumuşar
  //  - küçük ölü bölge → merkeze çok yakın titreşimi yok say (sıkı takip)
  //  - düzeltme HATAYLA ORANTILI → düz yolda küçük hata=küçük düzeltme (overshoot yok=zigzag yok),
  //    dönüşte büyük hata=güçlü düzeltme. İki teker de döner (pivot YOK → yalpalama yok).
  lineFilt = LINE_FILTER * lineFilt + (1.0f - LINE_FILTER) * linePosition;
  float error = lineFilt;
  if (error > -PID_DEADBAND && error < PID_DEADBAND) error = 0;

  if (linePosition < -0.2f) lastTurnDir = -1;
  else if (linePosition > 0.2f) lastTurnDir = 1;

  float rawDeriv = error - lastPidError;
  derivFilt = D_FILTER * derivFilt + (1.0f - D_FILTER) * rawDeriv;  // türevi low-pass et
  float corr = Kp * error + Kd * derivFilt;        // PD (Ki=0) — filtrelenmiş türevle
  corr = constrain(corr, -PID_CORRECTION_LIMIT, PID_CORRECTION_LIMIT);
  lastPidError = error;
  pidError = error;                               // telemetri/debug

  rampUpBase();
  int left  = constrain(rampBase + (int)corr, 0, maxSpeed);   // hata>0 (sağ) → sol hızlı → sağa döner
  int right = constrain(rampBase - (int)corr, 0, maxSpeed);
  driveForward(left, right);
  strcpy(action, "PID_FOLLOW");
}

void moveAllBlack() {
  resetPID();
  // Kavşak veya dur çizgisi → son dönüş yönünün tersine keskin dönüş
  if (lastTurnDir < 0) {
    driveForward(HARD_TURN_PWM, 0);
    strcpy(action, "ALL_BLACK_TURN_RIGHT");
  } else {
    driveForward(0, HARD_TURN_PWM);
    strcpy(action, "ALL_BLACK_TURN_LEFT");
  }
}

void moveLostSearch() {
  resetPID();
  uint8_t stage = getLostStage();
  int searchPWM = (stage == 0) ? SOFT_TURN_PWM : (stage == 1) ? MEDIUM_TURN_PWM : HARD_TURN_PWM;

  if (lastTurnDir < 0) {
    driveForward(0, searchPWM);
    strcpy(action, "SEARCH_LEFT");
  } else if (lastTurnDir > 0) {
    driveForward(searchPWM, 0);
    strcpy(action, "SEARCH_RIGHT");
  } else {
    driveForward(DEFAULT_BASE_SPEED, DEFAULT_BASE_SPEED);
    strcpy(action, "SEARCH_FWD");
  }
}

// ==================== HC-SR04 ====================
void readDistanceIfNeeded() {
  if (millis() - lastDistMs < DIST_INTERVAL_MS) return;
  lastDistMs = millis();

  digitalWrite(PIN_TRIG, LOW);  delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH); delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);

  // 5000µs ≈ ~85cm max — 8cm dur / 35cm yavaş eşikleri için fazlasıyla yeter. Engel
  // yokken pulseIn döngüyü en fazla 5ms bloklar (12ms yerine) → kontrol döngüsü daha
  // akıcı, çizgi takibinde periyodik "kör nokta" azalır → daha az zigzag.
  unsigned long dur = pulseIn(PIN_ECHO, HIGH, 5000);
  if (dur == 0) { distCm = -1; distStop = false; baseSpeed = configuredBaseSpeed; return; }

  distCm = dur / 58.0f;

  // 3 KADEME (kullanıcı isteği):
  if (distCm <= DIST_STOP_CM) {                 // KADEME 3: DUR (engel çok yakın)
    distStop = true;
  } else if (distCm <= DIST_SLOW_CM) {          // KADEME 2: YAVAŞ
    distStop = false;
    baseSpeed = min(SLOW_SPEED, configuredBaseSpeed);
  } else {                                      // KADEME 1: NORMAL
    distStop = false;
    baseSpeed = configuredBaseSpeed;
  }
}

// ==================== HALL / İSTASYON ====================
bool readHall() {
  bool raw = digitalRead(PIN_HALL);
  return HALL_ACTIVE_LOW ? !raw : raw;
}

// Hall kenar algıla; yeni istasyon tespitinde true döner
bool detectHallEvent() {
  bool cur = readHall();
  bool rising = cur && !lastHallActive;
  lastHallActive = cur;
  hallActive = cur;

  if (rising && millis() - lastHallTrigMs > HALL_DEBOUNCE_MS) {
    lastHallTrigMs = millis();
    return true;
  }
  return false;
}

void startStationPause() {
  stopMotors();
  stationPause = true;
  stationDetected = true;
  stationPauseUntil = millis() + STATION_STOP_MS;
  currentStop = (currentStop + 1) % 6;  // 6 istasyon döngüsü
  strcpy(action, "STATION_STOP");
}

// İstasyon duraklaması devam ediyor mu? true ise dışarıda bekle
bool handleStationPause() {
  if (!stationPause) return false;
  if (millis() >= stationPauseUntil) {
    stationPause = false;
    stationDetected = false;
    strcpy(action, "RESUMING");
    return false;
  }
  return true;
}

// ==================== FSR / DOLULUK ====================
void readFsrIfNeeded() {
  if (millis() - lastFsrMs < FSR_INTERVAL_MS) return;
  lastFsrMs = millis();

  if (!adsReady) {
    // ADS1115 bağlı değil: sabit boş
    fsr1Raw = 0; fsr2Raw = 0;
    fsr1Occupied = false; fsr2Occupied = false;
    strcpy(occupancyStatus, "empty");
    strcpy(occupancyLabel,  "Empty");
    strcpy(occupancyColor,  "green");
    return;
  }

  fsr1Raw = (int)ads.readADC_SingleEnded(0);
  fsr2Raw = (int)ads.readADC_SingleEnded(1);
  fsr1Occupied = (fsr1Raw > DEFAULT_FSR_THRESHOLD);
  fsr2Occupied = (fsr2Raw > DEFAULT_FSR_THRESHOLD);

  int count = (fsr1Occupied ? 1 : 0) + (fsr2Occupied ? 1 : 0);

  if (count == 0) {
    strcpy(occupancyStatus, "empty");
    strcpy(occupancyLabel,  "Empty");
    strcpy(occupancyColor,  "green");
  } else if (count == 1) {
    strcpy(occupancyStatus, "partial");
    strcpy(occupancyLabel,  "Partial");
    strcpy(occupancyColor,  "yellow");
  } else {
    strcpy(occupancyStatus, "full");
    strcpy(occupancyLabel,  "Full");
    strcpy(occupancyColor,  "red");
  }
}

// ==================== ENCODER ====================
long getEncR() { noInterrupts(); long v = encTicksR; interrupts(); return v; }
long getEncL() { noInterrupts(); long v = encTicksL; interrupts(); return v; }

// ==================== MANUEL HAREKETİ ====================
// driveManual(rightFwd, rightCmd, leftFwd, leftCmd)
// Nazik direksiyon: dönerken iç tekerlek DURUR (geri gitmez), dış tekerlek sürer.
void updateManualMovement() {
  if (strcmp(manualCmd, "FORWARD") == 0) {
    driveManual(true,  manualSpeed, true,  manualSpeed);   // ikisi ileri
    strcpy(action, "MANUAL_FORWARD");
  } else if (strcmp(manualCmd, "BACKWARD") == 0) {
    driveManual(false, manualSpeed, false, manualSpeed);   // ikisi geri (boost YOK, D13 düzeldi)
    strcpy(action, "MANUAL_BACKWARD");
  } else if (strcmp(manualCmd, "LEFT") == 0) {
    driveManual(true,  manualSpeed, true,  0);             // sağ ileri, SOL DUR → sola döner
    strcpy(action, "MANUAL_LEFT");
  } else if (strcmp(manualCmd, "RIGHT") == 0) {
    driveManual(true,  0,           true,  manualSpeed);   // SAĞ DUR, sol ileri → sağa döner
    strcpy(action, "MANUAL_RIGHT");
  } else {
    stopMotors();
    strcpy(action, "MANUAL_STOP");
  }
}

// ==================== AUTO HAREKETİ ====================
void updateAutoMovement() {
  if (handleStationPause()) return;

  if (detectHallEvent()) {
    startStationPause();
    return;
  }

  if (distStop) {
    stopMotors();
    resetPID();
    strcpy(action, "DISTANCE_STOP");
    return;
  }

  if (lineFound && strcmp(lineCase, "ALL_BLACK") != 0) {
    movePIDFollow();
  } else if (strcmp(lineCase, "ALL_BLACK") == 0) {
    moveAllBlack();
  } else {
    moveLostSearch();
  }
}

// ==================== ÖLÇÜM MOTORU (kalibrasyon modu) ====================
long encSum() { return getEncL() + getEncR(); }

float ticksToCm(long ticks) {
  return ((ticks / 2.0f) * 3.14159f * WHEEL_DIAMETER_CM) / TICKS_PER_REV;
}

void startSegment() {
  segStartMs       = millis();
  segStartEncSum   = encSum();
  segPwmSum        = 0;
  segPwmSamples    = 0;
  segLineLost      = false;
}

void resetMeasurement() {
  measuring      = false;
  positionKnown  = false;
  measSuspended  = false;
  currentSegment = 0;
  hallCount      = 0;
  lapCount       = 0;
  etaSec         = 0;
  lastLapDurationMs = 0;
}

// Bir segment tamamlandı: süre + mesafe hesapla, ESP'ye rapor için sakla
void finalizeSegment() {
  unsigned long dur = millis() - segStartMs;
  long ticks = encSum() - segStartEncSum;
  if (ticks < 0) ticks = 0;

  lastSegIndex      = currentSegment;
  lastSegDistanceCm = ticksToCm(ticks);
  lastSegDurationMs = dur;
  lastSegAvgPwm     = segPwmSamples ? (int)(segPwmSum / segPwmSamples) : 0;
  lastSegLineLost   = segLineLost ? 1 : 0;
  segReportPending  = true;

  segLastDurationMs[currentSegment] = dur;   // basit ETA için sakla

  Serial.print(F("[SEG] idx="));    Serial.print(currentSegment);
  Serial.print(F(" dist="));        Serial.print(lastSegDistanceCm, 1);
  Serial.print(F("cm dur="));       Serial.print(dur / 1000.0f, 1);
  Serial.print(F("s avgPWM="));     Serial.print(lastSegAvgPwm);
  Serial.print(F(" lineLost="));    Serial.print(lastSegLineLost);
  Serial.print(F(" lap="));         Serial.println(lapCount);
}

// İlk Hall → ölçümü başlat; sonraki her Hall → segmenti kapat, yenisini aç
void onMeasurementHall() {
  hallCount++;

  if (!measuring) {
    // İlk durağa ulaşıldı: ölçüm referansı burası
    measuring      = true;
    positionKnown  = true;
    measSuspended  = false;
    currentSegment = 0;
    currentStop    = 0;
    lapStartMs     = millis();
    startSegment();
    strcpy(action, "MEAS_START");
    Serial.println(F("[MEAS] İlk durak — ölçüm başladı"));
    return;
  }

  if (measSuspended) {
    // Manuelden sonraki ilk Hall: yeniden senkronize, TAZE başla (yarım segment atılır)
    measSuspended = false;
    if (expectedNextStop >= 1 && expectedNextStop <= STOP_COUNT) {
      currentStop    = expectedNextStop - 1;   // 1-based → 0-based
      currentSegment = currentStop;
    }
    startSegment();
    strcpy(action, "MEAS_RESYNC");
    Serial.println(F("[MEAS] Manuel sonrası yeniden senkron"));
    return;
  }

  // Mevcut segmenti bitir
  finalizeSegment();

  // Sıradaki segmente geç
  currentSegment = (currentSegment + 1) % STOP_COUNT;
  currentStop    = currentSegment;
  if (currentSegment == 0) {                    // tam tur tamamlandı
    lastLapDurationMs = millis() - lapStartMs;
    lapStartMs        = millis();
    lapCount++;
    Serial.print(F("[LAP] tur="));     Serial.print(lapCount);
    Serial.print(F(" süre="));         Serial.print(lastLapDurationMs / 1000.0f, 1);
    Serial.println(F("s"));
  }
  startSegment();
}

// Her döngü çağrılır. Manuel moda girince ölçüm ASKIYA alınır; manuelden çıkınca
// da askıda kalır → ölçme/ML, manuelden sonraki İLK Hall'da yeniden başlar (resync).
void updateMeasurement() {
  // calibration VE service modunda ölç (service = Start Trips, ETA refine devam eder)
  bool inCalib = (strcmp(vehicleMode, "calibration") == 0 ||
                  strcmp(vehicleMode, "service") == 0);

  if (!inCalib) {
    if (measuring || hallCount > 0) resetMeasurement();
    return;
  }

  // Manuel moda girilirse → askıya al (manuelden sonraki Hall'a kadar askıda kalır)
  if (manualEnabled) measSuspended = true;

  // Aktif ölçümde (askıda değil + manuel değil) istatistik biriktir
  if (measuring && !measSuspended && !manualEnabled) {
    segPwmSum += (leftPWM + rightPWM) / 2;
    segPwmSamples++;
    if (strcmp(lineCase, "LOST") == 0) segLineLost = true;   // sapma/arama tespiti
  }

  // Basit ETA: sıradaki segmentin son ölçülen süresi (sn). Öndeki araca yaklaşıp
  // yavaşlama → kalan mesafe daha uzun sürer, ölçeklenir. (Web ML bunu rafine eder.)
  if (measuring && !measSuspended) {
    unsigned long base = segLastDurationMs[currentSegment];
    if (base > 0) {
      float scale = 1.0f;
      if (distStop)                          scale = 3.0f;   // önde engel: çok yavaş
      else if (baseSpeed <= VERY_SLOW_SPEED) scale = 1.8f;
      else if (baseSpeed <= SLOW_SPEED)      scale = 1.3f;
      etaSec = (int)((base / 1000.0f) * scale);
    } else {
      etaSec = 0;
    }
  }
}

// Kalibrasyon/ölçüm hareketi: çizgi takip + Hall'da segment ölç (istasyonda DURMAZ)
void updateCalibrationMovement() {
  if (detectHallEvent()) {
    onMeasurementHall();
  }

  if (distStop) {
    stopMotors();
    resetPID();
    strcpy(action, "DISTANCE_STOP");
    return;
  }

  if (lineFound && strcmp(lineCase, "ALL_BLACK") != 0) {
    movePIDFollow();
  } else if (strcmp(lineCase, "ALL_BLACK") == 0) {
    moveAllBlack();
  } else {
    moveLostSearch();
  }
}

// ==================== ESP PROTOKOLÜ ====================
// Komut formatı (ESP → Arduino):
//   >C,<mode>,<manEn>,<cmd>,<speed>\n   (mod + manuel komut)
//   >P,<kp>,<ki>,<kd>,<base>,<max>\n    (PID ayarları)
//   örnek: >C,auto,0,STOP,0
//           >C,manual,1,FORWARD,180
//           >P,35.0,0.0,20.0,180,255

// ATOMİK: tüm alanlar gelene kadar hiçbir şeyi uygulama.
// (SoftwareSerial yarı-çift yönlü olduğu için satırlar kesik gelebilir;
//  yarım bir komutu uygulamak tehlikeli olur — ör. kesik STOP.)
void parseEspCommand(char* line) {
  char* p = line + 3;  // mod alanına geç

  char tmpMode[12];
  char tmpCmd[12];
  bool tmpEn;
  int  tmpSpd;

  char* tok = strtok(p, ",");
  if (!tok) return;
  strncpy(tmpMode, tok, sizeof(tmpMode) - 1); tmpMode[sizeof(tmpMode) - 1] = '\0';

  tok = strtok(NULL, ",");
  if (!tok) return;
  tmpEn = (tok[0] == '1');

  tok = strtok(NULL, ",");
  if (!tok) return;
  strncpy(tmpCmd, tok, sizeof(tmpCmd) - 1); tmpCmd[sizeof(tmpCmd) - 1] = '\0';

  tok = strtok(NULL, ",");
  if (!tok) return;
  tmpSpd = constrain(atoi(tok), 0, 255);

  // Tüm alanlar tamam → şimdi uygula
  strncpy(vehicleMode, tmpMode, sizeof(vehicleMode) - 1);
  manualEnabled = tmpEn;
  strncpy(manualCmd, tmpCmd, sizeof(manualCmd) - 1);
  manualSpeed = tmpSpd;
  lastCmdRecvMs = millis();   // watchdog: ESP canlı
}

void parseEspPid(char* line) {
  char* p = line + 3;  // kp alanına geç

  float tKp, tKi, tKd; int tBase, tMax;

  char* tok = strtok(p, ",");   if (!tok) return; tKp = atof(tok);
  tok = strtok(NULL, ",");      if (!tok) return; tKi = atof(tok);
  tok = strtok(NULL, ",");      if (!tok) return; tKd = atof(tok);
  tok = strtok(NULL, ",");      if (!tok) return; tBase = constrain(atoi(tok), 0, 255);
  tok = strtok(NULL, ",");      if (!tok) return; tMax  = constrain(atoi(tok), 0, 255);

  // Tüm alanlar tamam → uygula
  Kp = tKp; Ki = tKi; Kd = tKd;
  configuredBaseSpeed = tBase;
  maxSpeed = tMax;

  Serial.print(F("[PID] Kp=")); Serial.print(Kp);
  Serial.print(F(" Ki="));      Serial.print(Ki);
  Serial.print(F(" Kd="));      Serial.print(Kd);
  Serial.print(F(" base="));    Serial.print(configuredBaseSpeed);
  Serial.print(F(" max="));     Serial.println(maxSpeed);
}

// >N,<expectedNextStop>  — manuelden sonra varılacak durak (1-based)
void parseEspNextStop(char* line) {
  char* p = line + 3;
  char* tok = strtok(p, ",");
  if (!tok) return;
  expectedNextStop = constrain(atoi(tok), 0, STOP_COUNT);
}

// >V,<espVersion>  — ESP firmware sürümü (doğru ESP firmware'i yüklü mü görmek için)
void parseEspVersion(char* line) {
  char* p = line + 3;
  char* tok = strtok(p, ",");
  if (!tok) return;
  strncpy(espFwVersion, tok, sizeof(espFwVersion) - 1);
  espFwVersion[sizeof(espFwVersion) - 1] = '\0';
}

void dispatchEspLine(char* line) {
  if (line[0] != '>') return;
  if (line[1] == 'C') parseEspCommand(line);
  else if (line[1] == 'P') parseEspPid(line);
  else if (line[1] == 'N') parseEspNextStop(line);
  else if (line[1] == 'V') parseEspVersion(line);
}

void readEspIfAvailable() {
  while (espSerial.available()) {
    char c = (char)espSerial.read();
    espRxBytes++;                         // teşhis: gelen her bayt
    if (c == '\n') {
      rxBuf[rxIdx] = '\0';
      if (rxIdx > 3) {
        dispatchEspLine(rxBuf);
        strncpy(lastEspLine, rxBuf, sizeof(lastEspLine) - 1);
        espRxLines++;
      }
      rxIdx = 0;
    } else if (rxIdx < sizeof(rxBuf) - 1) {
      rxBuf[rxIdx++] = c;
    }
  }
}

// ML için seviye yardımcıları
int occupancyLevel() {
  if (strcmp(occupancyStatus, "full") == 0)    return 2;
  if (strcmp(occupancyStatus, "partial") == 0) return 1;
  return 0;
}

int distanceActionLevel() {
  if (distCm < 0)                  return 0;   // okuma yok = normal
  if (distCm <= DIST_STOP_CM)      return 3;   // dur
  if (distCm <= DIST_VERY_SLOW_CM) return 2;   // çok yavaş
  if (distCm <= DIST_SLOW_CM)      return 1;   // yavaş
  return 0;                                     // normal
}

// Segment tamamlandığında ESP'ye gönder:
//   >S,<idx>,<distCm>,<durationMs>,<avgPwm>,<lineLost>,<occLevel>,<distActLevel>,<lap>
void sendSegmentReportIfPending() {
  if (!segReportPending) return;
  segReportPending = false;

  char dbuf[10];
  dtostrf(lastSegDistanceCm, 4, 1, dbuf);

  espSerial.print(F(">S,"));
  espSerial.print(lastSegIndex);       espSerial.print(',');
  espSerial.print(dbuf);               espSerial.print(',');
  espSerial.print(lastSegDurationMs);  espSerial.print(',');
  espSerial.print(lastSegAvgPwm);      espSerial.print(',');
  espSerial.print(lastSegLineLost);    espSerial.print(',');
  espSerial.print(occupancyLevel());   espSerial.print(',');
  espSerial.print(distanceActionLevel()); espSerial.print(',');
  espSerial.println(lapCount);
}

// Telemetry formatı (Arduino → ESP):
//   <T,lPWM,rPWM,dist,hallRaw,station,lineCase,fsr1,fsr2,occ,encL,encR,action,mode,
//      posKnown,curStop,nextStop,etaSec,seg,lap\n
void sendTelemetry() {
  if (millis() - lastTelemetryMs < TELEMETRY_INTERVAL_MS) return;
  lastTelemetryMs = millis();

  // dist: -1 veya gerçek değer (1 ondalık)
  char distBuf[8];
  if (distCm < 0) strcpy(distBuf, "-1");
  else dtostrf(distCm, 4, 1, distBuf);

  int nextStop = measuring ? ((currentStop + 1) % STOP_COUNT) : 0;

  espSerial.print(F("<T,"));
  espSerial.print(leftPWM);          espSerial.print(',');
  espSerial.print(rightPWM);         espSerial.print(',');
  espSerial.print(distBuf);          espSerial.print(',');
  espSerial.print(hallActive ? 1:0); espSerial.print(',');
  espSerial.print(stationDetected?1:0); espSerial.print(',');
  espSerial.print(lineCase);         espSerial.print(',');
  espSerial.print(fsr1Raw);          espSerial.print(',');
  espSerial.print(fsr2Raw);          espSerial.print(',');
  espSerial.print(occupancyStatus);  espSerial.print(',');
  espSerial.print(getEncL());        espSerial.print(',');
  espSerial.print(getEncR());        espSerial.print(',');
  espSerial.print(action);           espSerial.print(',');
  espSerial.print(vehicleMode);      espSerial.print(',');
  espSerial.print(positionKnown?1:0);espSerial.print(',');
  espSerial.print(currentStop);      espSerial.print(',');
  espSerial.print(nextStop);         espSerial.print(',');
  espSerial.print(etaSec);           espSerial.print(',');
  espSerial.print(currentSegment);   espSerial.print(',');
  espSerial.print(lapCount);         espSerial.print(',');
  espSerial.println(lastLapDurationMs);
}

// ==================== DEBUG (Serial USB) ====================
void printDebugIfNeeded() {
  if (millis() - lastDebugMs < DEBUG_INTERVAL_MS) return;
  lastDebugMs = millis();

  Serial.print(F("[LINE] "));
  for (int i=0;i<5;i++) Serial.print(s[i]?'#':'.');
  Serial.print(F(" pos="));  Serial.print(linePosition, 2);
  Serial.print(F(" case="));  Serial.print(lineCase);
  Serial.print(F(" act="));   Serial.println(action);

  Serial.print(F("[MOT]  L="));   Serial.print(leftPWM);
  Serial.print(F(" R="));          Serial.print(rightPWM);
  Serial.print(F(" dist="));       Serial.print(distCm, 1);
  Serial.print(F(" mode="));       Serial.println(vehicleMode);

  // Aktif PID değerleri (panelden geldiyse değişir, gelmediyse firmware default).
  // Zigzag varken buraya bak: Kp/Kd araçta GERÇEKTE ne? FW sürümünü de yazar.
  Serial.print(F("[PID]  "));      Serial.print(F(FW_VERSION));
  Serial.print(F(" Kp="));         Serial.print(Kp, 1);
  Serial.print(F(" Ki="));         Serial.print(Ki, 1);
  Serial.print(F(" Kd="));         Serial.print(Kd, 1);
  Serial.print(F(" base="));       Serial.println(baseSpeed);

  // ESP RX teşhis: D10'dan bayt geliyor mu, ESP sürümü ne, son satır ne?
  Serial.print(F("[ESP]  ver="));     Serial.print(espFwVersion);
  Serial.print(F(" rxBytes="));       Serial.print(espRxBytes);
  Serial.print(F(" lines="));         Serial.print(espRxLines);
  Serial.print(F(" manEn="));         Serial.print(manualEnabled?1:0);
  Serial.print(F(" lastLine="));      Serial.println(lastEspLine);

  // Ölçüm durumu (kalibrasyon modu)
  Serial.print(F("[MEAS] measuring=")); Serial.print(measuring?1:0);
  Serial.print(F(" seg="));             Serial.print(currentSegment);
  Serial.print(F(" hallCnt="));         Serial.print(hallCount);
  Serial.print(F(" lap="));             Serial.print(lapCount);
  Serial.print(F(" susp="));            Serial.print(measSuspended?1:0);
  Serial.print(F(" lapDur="));          Serial.print(lastLapDurationMs / 1000.0f, 1);
  Serial.print(F(" eta="));             Serial.print(etaSec);
  Serial.println(F("s"));

  Serial.print(F("[HALL] active=")); Serial.print(hallActive?1:0);
  Serial.print(F(" stop="));        Serial.print(currentStop);
  Serial.print(F(" pause="));       Serial.println(stationPause?1:0);

  Serial.print(F("[FSR]  1="));    Serial.print(fsr1Raw);
  Serial.print(F(" 2="));           Serial.print(fsr2Raw);
  Serial.print(F(" occ="));         Serial.println(occupancyStatus);

  Serial.print(F("[ENC]  L="));    Serial.print(getEncL());
  Serial.print(F(" R="));           Serial.println(getEncR());
}

// ==================== SETUP ====================
void setup() {
  Serial.begin(115200);
  espSerial.begin(9600);
  delay(100);

  Wire.begin();

  // PCF8574: giriş modu (0xFF yaz)
  Wire.beginTransmission(PCF_ADDR);
  Wire.write(0xFF);
  Wire.endTransmission();

  // ADS1115
  adsReady = ads.begin(ADS_ADDR);
  if (adsReady) ads.setGain(GAIN_ONE);
  Serial.print(F("ADS1115: "));
  Serial.println(adsReady ? F("OK") : F("not found (FSR disabled)"));

  // HC-SR04
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  digitalWrite(PIN_TRIG, LOW);

  // Hall
  pinMode(PIN_HALL, INPUT_PULLUP);

  // Encoder
  pinMode(PIN_ENC_R, INPUT_PULLUP);
  pinMode(PIN_ENC_L, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_R), onEncR, RISING);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_L), onEncL, RISING);

  // Motor pinleri
  pinMode(PIN_IN1, OUTPUT); pinMode(PIN_IN2, OUTPUT);
  pinMode(PIN_IN3, OUTPUT); pinMode(PIN_IN4, OUTPUT);
  pinMode(PIN_ENA, OUTPUT); pinMode(PIN_ENB, OUTPUT);
  stopMotors();

  Serial.println(F("=========================================="));
  Serial.println(F("MEGABUS Arduino " FW_VERSION " ready."));
  Serial.print(F("PID defaults -> Kp=")); Serial.print(Kp);
  Serial.print(F(" Ki="));   Serial.print(Ki);
  Serial.print(F(" Kd="));   Serial.print(Kd);
  Serial.print(F(" base=")); Serial.print(baseSpeed);
  Serial.print(F(" deadband=")); Serial.print(PID_DEADBAND);
  Serial.print(F(" lineFilt=")); Serial.println(LINE_FILTER);
  Serial.println(F("Eger bu satiri gormuyorsan ESKI firmware yuklu demektir!"));
  Serial.println(F("=========================================="));
  Serial.println(F("Mod: idle — ESP'den komut bekleniyor."));
}

// ==================== LOOP ====================
void loop() {
  readEspIfAvailable();
  readDistanceIfNeeded();
  readFsrIfNeeded();
  readLineSensors();

  // Ölçüm durum makinesi her döngü çalışır (manuel duraklatmayı yönetir)
  updateMeasurement();

  // NOT: "komut gelmezse dur" watchdog'u KALDIRILDI (kullanıcı isteği).
  // Durma yalnızca WiFi koparsa olur: ESP, WiFi kesilince STOP gönderir.
  // Latch'lenmiş komut, ESP komut göndermeyi anlık geciktirse bile korunur.

  // ---- Mod yönlendirme ----
  if (manualEnabled) {
    // Manuel her zaman önceliklidir (mod ne olursa olsun manualEnabled=1 ise sür).
    updateManualMovement();
  } else if (strcmp(vehicleMode, "calibration") == 0 ||
             strcmp(vehicleMode, "service") == 0) {
    // calibration VE service: çizgi takip + segment ölç + ML (Start Trips de
    // ETA'yı her turda rafine etsin diye service de ölçer).
    updateCalibrationMovement();
  } else if (strcmp(vehicleMode, "auto") == 0) {
    updateAutoMovement();
  } else {
    // idle / stopped / recovery (v2) / bilinmeyen → dur
    stopMotors();
    resetPID();
    strcpy(action, "WAITING");
  }

  sendSegmentReportIfPending();
  sendTelemetry();
  printDebugIfNeeded();

  delay(1);
}
