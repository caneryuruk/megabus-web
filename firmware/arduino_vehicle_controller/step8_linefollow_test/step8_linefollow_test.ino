// =====================================================================
// MEGABUS — Step 8: Çizgi Takip Testi (ESP olmadan)
//
// USB Serial Monitor üzerinden mod değiştir:
//   a → AUTO (çizgi takip başlar)
//   s → STOP (dur)
//   + → baseSpeed +10
//   - → baseSpeed -10
//   p → PID değerlerini yazdır
//
// Donanım: PCF8574, L298N, HC-SR04, encoder, Hall — hepsi bağlı olmalı.
// ADS1115 olmasa da çalışır (FSR devre dışı).
// Serial Monitor: 115200 baud
// =====================================================================

#include <Wire.h>
#include <Adafruit_ADS1X15.h>

// ==================== YÖN TERSLEME ====================
#define INVERT_RIGHT    true
#define INVERT_LEFT     false
#define INVERT_SENSORS  false
#define HALL_ACTIVE_LOW true

// ==================== PİNLER ====================
#define PIN_TRIG   A0
#define PIN_ECHO   A1
#define PIN_ENC_R  2
#define PIN_ENC_L  3
#define PIN_HALL   4
#define PIN_ENA    5
#define PIN_ENB    6
#define PIN_IN1    7
#define PIN_IN2    8
#define PIN_IN3    12
#define PIN_IN4    13

// ==================== PCF / ADS ====================
#define PCF_ADDR   0x20
#define ADS_ADDR   0x48

// ==================== AYARLAR ====================
#define DEFAULT_BASE_SPEED  180
#define DEFAULT_MAX_SPEED   255
#define DEFAULT_KP          35.0f
#define DEFAULT_KI          0.0f
#define DEFAULT_KD          20.0f
#define FSR_THRESHOLD       1000

#define MIN_MOVE_PWM        100
#define STARTUP_BOOST_PWM   220
#define STARTUP_BOOST_MS    80

#define SOFT_TURN_PWM       160
#define MEDIUM_TURN_PWM     190
#define HARD_TURN_PWM       220

#define SOFT_STAGE_MS       400
#define MEDIUM_STAGE_MS     1000
#define LOST_SOFT_MS        300
#define LOST_MEDIUM_MS      800

#define HALL_DEBOUNCE_MS    800
#define STATION_STOP_MS     3000

#define DIST_INTERVAL_MS    150
#define FSR_INTERVAL_MS     250
#define DEBUG_INTERVAL_MS   400

#define DIST_STOP_CM        8.0f
#define DIST_VERY_SLOW_CM   18.0f
#define DIST_SLOW_CM        35.0f
#define VERY_SLOW_SPEED     130
#define SLOW_SPEED          150

// ==================== GLOBALS ====================
Adafruit_ADS1115 ads;
bool adsReady = false;

bool  s[5]        = {};
uint8_t rawPCF    = 0xFF;
float distCm      = -1;
bool  distStop    = false;

float Kp = DEFAULT_KP, Ki = DEFAULT_KI, Kd = DEFAULT_KD;
float pidError = 0, lastPidError = 0, pidIntegral = 0, pidCorrection = 0;

int baseSpeed = DEFAULT_BASE_SPEED;
int maxSpeed  = DEFAULT_MAX_SPEED;
int leftPWM = 0, rightPWM = 0;
unsigned long leftBoostUntil = 0, rightBoostUntil = 0;
int lastLeftCmd = 0, lastRightCmd = 0;

int   lastTurnDir = 0;
int   sideOnlyDir = 0;
unsigned long sideOnlyStartMs = 0, lineLostStartMs = 0;
bool  lineFound   = false;
float linePos     = 0;
char  lineCase[16] = "LOST";
char  action[32]   = "WAITING";

volatile long encTicksR = 0, encTicksL = 0;

bool  hallActive = false, lastHallActive = false;
unsigned long lastHallTrigMs = 0;
bool  stationPause = false;
unsigned long stationPauseUntil = 0;
int   currentStop = 0;

int   fsr1Raw = 0, fsr2Raw = 0;
char  occupancy[8] = "empty";

bool  autoMode = false;   // a=AUTO, s=STOP

unsigned long lastDistMs = 0, lastFsrMs = 0, lastDebugMs = 0;

// ==================== ISR ====================
void onEncR() { encTicksR++; }
void onEncL() { encTicksL++; }

// ==================== PCF ====================
uint8_t pcfRead() {
  Wire.requestFrom((uint8_t)PCF_ADDR, (uint8_t)1);
  return Wire.available() ? Wire.read() : 0xFF;
}
bool sBlack(uint8_t raw, uint8_t bit) {
  return INVERT_SENSORS ? bitRead(raw, bit) : !bitRead(raw, bit);
}

// ==================== MOTOR ====================
void applyMotorPins(bool rFwd, int rPwm, bool lFwd, int lPwm) {
  bool rA = INVERT_RIGHT ? !rFwd : rFwd;
  bool lA = INVERT_LEFT  ? !lFwd : lFwd;
  digitalWrite(PIN_IN1, rA ? HIGH : LOW);
  digitalWrite(PIN_IN2, rA ? LOW  : HIGH);
  digitalWrite(PIN_IN3, lA ? HIGH : LOW);
  digitalWrite(PIN_IN4, lA ? LOW  : HIGH);
  analogWrite(PIN_ENA, constrain(rPwm, 0, 255));
  analogWrite(PIN_ENB, constrain(lPwm, 0, 255));
}

void stopMotors() {
  analogWrite(PIN_ENA, 0); analogWrite(PIN_ENB, 0);
  digitalWrite(PIN_IN1,LOW); digitalWrite(PIN_IN2,LOW);
  digitalWrite(PIN_IN3,LOW); digitalWrite(PIN_IN4,LOW);
  leftPWM=0; rightPWM=0; lastLeftCmd=0; lastRightCmd=0;
  leftBoostUntil=0; rightBoostUntil=0;
}

int applyMin(int pwm) { return (pwm<=0) ? 0 : max(pwm, MIN_MOVE_PWM); }

void driveForward(int lCmd, int rCmd) {
  unsigned long now = millis();
  if (lCmd>0 && lastLeftCmd==0)  leftBoostUntil  = now + STARTUP_BOOST_MS;
  if (rCmd>0 && lastRightCmd==0) rightBoostUntil = now + STARTUP_BOOST_MS;
  int lA = applyMin(lCmd), rA = applyMin(rCmd);
  if (lCmd>0 && now<leftBoostUntil)  lA = max(lA, STARTUP_BOOST_PWM);
  if (rCmd>0 && now<rightBoostUntil) rA = max(rA, STARTUP_BOOST_PWM);
  leftPWM  = constrain(lA, 0, 255);
  rightPWM = constrain(rA, 0, 255);
  lastLeftCmd=lCmd; lastRightCmd=rCmd;
  applyMotorPins(true, rightPWM, true, leftPWM);
}

// ==================== QTR ====================
void readLineSensors() {
  rawPCF = pcfRead();
  for (uint8_t i=0;i<5;i++) s[i] = sBlack(rawPCF, i);

  int cnt=0; float wSum=0;
  const float w[5]={-2,-1,0,1,2};
  for (uint8_t i=0;i<5;i++) if(s[i]){wSum+=w[i];cnt++;}

  if (cnt==0) {
    lineFound=false; linePos=0; strcpy(lineCase,"LOST");
    if (!lineLostStartMs) lineLostStartMs=millis();
    sideOnlyDir=0; sideOnlyStartMs=0; return;
  }
  lineLostStartMs=0; lineFound=true;

  if (cnt==5) { linePos=0; strcpy(lineCase,"ALL_BLACK"); sideOnlyDir=0; sideOnlyStartMs=0; return; }

  linePos = wSum/cnt;
  if (linePos<-0.2f) lastTurnDir=-1;
  else if (linePos>0.2f) lastTurnDir=1;

  int newSide=(linePos<-0.5f)?-1:(linePos>0.5f)?1:0;
  if (newSide!=0) {
    if (sideOnlyDir!=newSide){sideOnlyDir=newSide;sideOnlyStartMs=millis();}
  } else { sideOnlyDir=0; sideOnlyStartMs=0; }

  if      (linePos>=-0.3f && linePos<=0.3f) strcpy(lineCase,"CENTER");
  else if (linePos>=-0.8f && linePos<-0.3f) strcpy(lineCase,"SLIGHT_LEFT");
  else if (linePos> 0.3f  && linePos<=0.8f) strcpy(lineCase,"SLIGHT_RIGHT");
  else if (linePos>=-1.5f && linePos<-0.8f) strcpy(lineCase,"LEFT");
  else if (linePos> 0.8f  && linePos<=1.5f) strcpy(lineCase,"RIGHT");
  else if (linePos<-1.5f)                   strcpy(lineCase,"FAR_LEFT");
  else                                       strcpy(lineCase,"FAR_RIGHT");
}

uint8_t getLostStage() {
  if (!lineLostStartMs) return 0;
  unsigned long el=millis()-lineLostStartMs;
  if (el<LOST_SOFT_MS) return 0;
  if (el<LOST_MEDIUM_MS) return 1;
  return 2;
}

// ==================== HC-SR04 ====================
void readDistanceIfNeeded() {
  if (millis()-lastDistMs < DIST_INTERVAL_MS) return;
  lastDistMs=millis();
  digitalWrite(PIN_TRIG,LOW);  delayMicroseconds(2);
  digitalWrite(PIN_TRIG,HIGH); delayMicroseconds(10);
  digitalWrite(PIN_TRIG,LOW);
  unsigned long dur=pulseIn(PIN_ECHO,HIGH,18000);
  if (!dur){distCm=-1;distStop=false;baseSpeed=DEFAULT_BASE_SPEED;return;}
  distCm=dur/58.0f;
  distStop=(distCm<=DIST_STOP_CM);
  if (!distStop){
    if      (distCm<=DIST_VERY_SLOW_CM) baseSpeed=min(VERY_SLOW_SPEED,DEFAULT_BASE_SPEED);
    else if (distCm<=DIST_SLOW_CM)      baseSpeed=min(SLOW_SPEED,DEFAULT_BASE_SPEED);
    else                                baseSpeed=DEFAULT_BASE_SPEED;
  }
}

// ==================== HALL ====================
bool readHall(){ bool r=digitalRead(PIN_HALL); return HALL_ACTIVE_LOW?!r:r; }

void checkHall() {
  bool cur=readHall();
  bool rising=cur&&!lastHallActive;
  lastHallActive=cur; hallActive=cur;
  if (rising && millis()-lastHallTrigMs>HALL_DEBOUNCE_MS) {
    lastHallTrigMs=millis();
    stopMotors();
    stationPause=true;
    stationPauseUntil=millis()+STATION_STOP_MS;
    currentStop=(currentStop+1)%6;
    strcpy(action,"STATION_STOP");
    Serial.print(F("[HALL] İstasyon #")); Serial.println(currentStop);
  }
}

bool handleStationPause(){
  if (!stationPause) return false;
  if (millis()>=stationPauseUntil){stationPause=false;return false;}
  return true;
}

// ==================== FSR ====================
void readFsrIfNeeded(){
  if (millis()-lastFsrMs<FSR_INTERVAL_MS) return;
  lastFsrMs=millis();
  if (!adsReady){fsr1Raw=0;fsr2Raw=0;strcpy(occupancy,"empty");return;}
  fsr1Raw=(int)ads.readADC_SingleEnded(0);
  fsr2Raw=(int)ads.readADC_SingleEnded(1);
  int cnt=(fsr1Raw>FSR_THRESHOLD?1:0)+(fsr2Raw>FSR_THRESHOLD?1:0);
  if      (cnt==0) strcpy(occupancy,"empty");
  else if (cnt==1) strcpy(occupancy,"partial");
  else             strcpy(occupancy,"full");
}

// ==================== PID ====================
void resetPID(){ pidError=0;lastPidError=0;pidIntegral=0;pidCorrection=0; }

void movePIDFollow(){
  pidError=linePos;
  float der=pidError-lastPidError;
  pidIntegral=constrain(pidIntegral+pidError,-50,50);
  float raw=Kp*pidError+Ki*pidIntegral+Kd*der;
  pidCorrection=constrain(raw,-120,120);
  lastPidError=pidError;
  int lCmd=constrain((int)(baseSpeed+pidCorrection),0,maxSpeed);
  int rCmd=constrain((int)(baseSpeed-pidCorrection),0,maxSpeed);
  driveForward(lCmd,rCmd);
  strcpy(action,"PID_FOLLOW");
}

void moveAllBlack(){
  resetPID();
  if (lastTurnDir<0){ driveForward(HARD_TURN_PWM,0); strcpy(action,"ALL_BLACK_R"); }
  else              { driveForward(0,HARD_TURN_PWM); strcpy(action,"ALL_BLACK_L"); }
}

void moveLostSearch(){
  resetPID();
  uint8_t st=getLostStage();
  int spd=(st==0)?SOFT_TURN_PWM:(st==1)?MEDIUM_TURN_PWM:HARD_TURN_PWM;
  if      (lastTurnDir<0){ driveForward(0,spd);   strcpy(action,"SEARCH_L"); }
  else if (lastTurnDir>0){ driveForward(spd,0);   strcpy(action,"SEARCH_R"); }
  else                   { driveForward(spd,spd); strcpy(action,"SEARCH_FWD"); }
}

// ==================== DEBUG ====================
void printDebugIfNeeded(){
  if (millis()-lastDebugMs<DEBUG_INTERVAL_MS) return;
  lastDebugMs=millis();

  Serial.print(F("LINE["));
  for(int i=0;i<5;i++) Serial.print(s[i]?'#':'.');
  Serial.print(F("] pos="));   Serial.print(linePos,2);
  Serial.print(F(" case="));   Serial.print(lineCase);
  Serial.print(F(" | L="));    Serial.print(leftPWM);
  Serial.print(F(" R="));      Serial.print(rightPWM);
  Serial.print(F(" | dist="));  Serial.print(distCm,1);
  Serial.print(F("cm | act=")); Serial.print(action);
  Serial.print(F(" | hall="));  Serial.print(hallActive?1:0);
  Serial.print(F(" stop="));    Serial.print(currentStop);
  Serial.print(F(" | occ="));   Serial.print(occupancy);
  noInterrupts(); long eL=encTicksL; long eR=encTicksR; interrupts();
  Serial.print(F(" | encL=")); Serial.print(eL);
  Serial.print(F(" encR="));   Serial.println(eR);
}

// ==================== USB KOMUTLARı ====================
void handleSerialCommand(){
  if (!Serial.available()) return;
  char c=tolower(Serial.read());
  if (c=='a'){
    autoMode=true;
    Serial.println(F(">> AUTO modu basladı"));
  } else if (c=='s'){
    autoMode=false;
    stopMotors(); resetPID();
    Serial.println(F(">> DURDURULDU"));
  } else if (c=='+'){
    baseSpeed=min(baseSpeed+10,DEFAULT_MAX_SPEED);
    Serial.print(F(">> baseSpeed=")); Serial.println(baseSpeed);
  } else if (c=='-'){
    baseSpeed=max(baseSpeed-10,100);
    Serial.print(F(">> baseSpeed=")); Serial.println(baseSpeed);
  } else if (c=='p'){
    Serial.print(F("Kp=")); Serial.print(Kp);
    Serial.print(F(" Ki=")); Serial.print(Ki);
    Serial.print(F(" Kd=")); Serial.print(Kd);
    Serial.print(F(" baseSpeed=")); Serial.println(baseSpeed);
  }
}

// ==================== SETUP ====================
void setup(){
  Serial.begin(115200);
  Wire.begin();

  Wire.beginTransmission(PCF_ADDR); Wire.write(0xFF); Wire.endTransmission();

  adsReady=ads.begin(ADS_ADDR);
  if (adsReady) ads.setGain(GAIN_ONE);
  Serial.print(F("ADS1115: ")); Serial.println(adsReady?F("OK"):F("yok"));

  pinMode(PIN_TRIG,OUTPUT); pinMode(PIN_ECHO,INPUT); digitalWrite(PIN_TRIG,LOW);
  pinMode(PIN_HALL,INPUT_PULLUP);
  pinMode(PIN_ENC_R,INPUT_PULLUP); pinMode(PIN_ENC_L,INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_R),onEncR,RISING);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_L),onEncL,RISING);

  pinMode(PIN_IN1,OUTPUT); pinMode(PIN_IN2,OUTPUT);
  pinMode(PIN_IN3,OUTPUT); pinMode(PIN_IN4,OUTPUT);
  pinMode(PIN_ENA,OUTPUT); pinMode(PIN_ENB,OUTPUT);
  stopMotors();

  Serial.println(F("=== MEGABUS Step 8: Line Follow Test ==="));
  Serial.println(F("Komutlar: a=AUTO  s=DUR  +=hız+  -=hız-  p=PID değerleri"));
}

// ==================== LOOP ====================
void loop(){
  handleSerialCommand();
  readDistanceIfNeeded();
  readFsrIfNeeded();
  readLineSensors();

  if (autoMode){
    if (handleStationPause()){
      printDebugIfNeeded();
      return;
    }
    checkHall();
    if (distStop){
      stopMotors(); resetPID(); strcpy(action,"DISTANCE_STOP");
    } else if (lineFound && strcmp(lineCase,"ALL_BLACK")!=0){
      movePIDFollow();
    } else if (strcmp(lineCase,"ALL_BLACK")==0){
      moveAllBlack();
    } else {
      moveLostSearch();
    }
  } else {
    stopMotors();
    resetPID();
    strcpy(action,"WAITING");
  }

  printDebugIfNeeded();
  delay(3);
}
