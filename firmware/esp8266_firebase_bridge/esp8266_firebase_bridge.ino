// =====================================================================
// MEGABUS — ESP8266 Firebase Bridge  v1
//
// Görev: Yalnızca köprü.
//   - Firebase manualControls/{CAR_ID} → Arduino'ya Serial komut
//   - Firebase commands/{CAR_ID}       → Arduino'ya Serial komut
//   - Arduino'dan gelen telemetry      → Firebase cars/{CAR_ID}
//
// Sensöre / motora dokunmaz. Tüm fiziksel kontrol Arduino'da.
//
// Bağlantı:
//   ESP TX (GPIO1 / D4 değil, seri TX)  → voltaj bölücü → Arduino D10 (RX)
//   ESP RX (GPIO3)                       ← Arduino D11 (TX) direkt 3.3V
//   SoftwareSerial KULLANILMIYOR ESP'de:
//     ESP8266'nın hardware UART'ı (Serial) Arduino'ya bağlı.
//     USB debug için Serial1 (GPIO2/D4) kullanılır.
//
// NOT: ESP8266 Arduino IDE ayarları:
//   Board : NodeMCU 1.0 (ESP-12E Module)
//   Upload Speed: 115200
//   CPU Freq: 80 MHz
//
// Kütüphaneler:
//   ESP8266WiFi (board paketi içinde gelir)
//   ESP8266HTTPClient (board paketi içinde gelir)
// =====================================================================

#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <WiFiClientSecureBearSSL.h>
#include "secrets.h"

// ==================== ZAMANLAMA ====================
// Düşük gecikme: manuel'i HIZLI poll'la + TLS oturum RESUME ile handshake'i kısalt
// (~1sn yerine ~300ms). Diğerleri (mod/pid/telemetri) latency-kritik değil, seyrek.
#define MANUAL_POLL_MS        50    // manualControls — keep-alive ile GET ~100-200ms
// commands GET'i ARTIK YEDEK: mod + expectedNextStop zaten 50ms keep-alive
// (applyManualBody, manualControls) ile geliyor. Bu poll'u 8s→30s yavaşlattık ki
// keep-alive'a handshake baskısı yapan transient TLS op'ları azalsın → "ara ara geç
// algılama" düşsün. (Manuel reaktiflik için 2. madde değişikliği — gerekirse 8000'e
// geri al.) Keep-alive yolu (manualClient/manualHttp/50ms) HİÇ DEĞİŞMEDİ.
#define COMMAND_POLL_MS    30000    // commands — yedek poll (mod keep-alive'dan geliyor)
#define TELEMETRY_PUSH_MS   5000    // Firebase'e telemetry yazma (panel için, kritik değil)
#define WIFI_RETRY_MS      30000    // Wi-Fi yeniden bağlanma denemesi
#define PID_POLL_MS        15000    // PID ayarları (yalnızca Save'de değişir)
#define HTTP_TIMEOUT_MS     4000    // HTTPS çağrıları için zaman aşımı (takılma önler)
#define CONN_LOST_MS        2500    // manuel veri bu kadar süredir gelmiyorsa → STOP (güvenlik)

// ESP firmware sürümü — Arduino'ya >V ile bildirilir, Arduino [ESP] ver=... satırında
// gösterir. Doğru ESP firmware'i yüklü mü buradan anlaşılır. PID keep-alive ile gelir.
#define ESP_FW_VERSION    "ka-pid-v3"
#define VERSION_PUSH_MS    10000     // sürümü her 10sn'de bir Arduino'ya tekrar gönder

// ==================== URL YARDIMCILARI ====================
// Stack'e geçici String basarız; tek seferlik HTTP çağrısı için yeterli.
String fbUrl(const char* path) {
  String u = F(FIREBASE_URL);
  u += path;
  u += F(".json");
  return u;
}

String carUrl()           { return fbUrl("/cars/" CAR_ID); }
String manualControlUrl() { return fbUrl("/manualControls/" CAR_ID); }
String commandUrl()       { return fbUrl("/commands/" CAR_ID); }
String pidUrl()           { return fbUrl("/cars/" CAR_ID "/pid"); }
String mlUrl()            { return fbUrl("/mlTrainingData/" CAR_ID); }

// ==================== DURUM ====================
bool  wifiOk            = false;
unsigned long lastWifiRetry = 0;

// Manuel kontrol durumu (latch: komut STOP'a kadar sürer)
bool  manualEnabled      = false;
char  manualCmd[12]      = "STOP";
int   manualSpeed        = 0;
unsigned long lastManualOkMs = 0;       // son başarılı manuel okuma (bağlantı sağlığı)
String lastManualVersion = "";          // sürüm dedup
unsigned long lastManualPollMs = 0;     // poll zamanlayıcı

// TLS oturumu — transient çağrılarda handshake'i RESUME ile kısaltır.
BearSSL::Session firebaseSession;

// Manuel için KALICI keep-alive bağlantı: ilk istekte handshake (~1sn), sonraki
// istekler bağlantıyı YENİDEN KULLANIR (handshake yok → ~100-200ms). <1sn gecikmenin anahtarı.
BearSSL::WiFiClientSecure manualClient;
HTTPClient manualHttp;
bool manualHttpReady = false;
unsigned long lastManualConnAttempt = 0;

// Araç komutu durumu
char  vehicleMode[12]    = "idle";
String lastCmdVersion    = "";
unsigned long lastCmdPollMs = 0;

// PID ayar durumu
String lastPidVersion    = "";
unsigned long lastPidPollMs = 0;

// Telemetry (Arduino'dan parse edilen son değerler)
struct Telemetry {
  int   leftPWM         = 0;
  int   rightPWM        = 0;
  float distCm          = -1;
  int   hallRaw         = 0;
  int   stationDetected = 0;
  char  lineCase[16]    = "LOST";
  int   fsr1Raw         = 0;
  int   fsr2Raw         = 0;
  char  occupancy[8]    = "empty";
  long  encL            = 0;
  long  encR            = 0;
  char  action[32]      = "WAITING";
  char  mode[12]        = "idle";
  // Ölçüm/konum alanları (telemetri uzantısı)
  int   positionKnown   = 0;
  int   currentStop     = 0;
  int   nextStop        = 0;
  int   etaSec          = 0;
  int   segment         = 0;
  int   lapCount        = 0;
  unsigned long lapDurationMs = 0;
} tele;

unsigned long lastTelemetryPushMs = 0;
String lastNextStopSent = "";   // >N dedup (expectedNextStop)
String lastPidSigKA = "";       // >P dedup — keep-alive (manualControls) üzerinden gelen PID
unsigned long lastPidKaMs = 0;  // keep-alive PID parse hız sınırı (manuel hattını yormamak için)

// Arduino'dan gelen satır buffer'ı (genişletilmiş telemetri ~125 karakter olabilir)
char  arBuf[160];
uint16_t arIdx = 0;

// ==================== Wi-Fi ====================
void startWifi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial1.print(F("[WIFI] Baglanıyor: "));
  Serial1.println(WIFI_SSID);
}

void maintainWifi() {
  if (WiFi.status() == WL_CONNECTED) { wifiOk = true; return; }
  wifiOk = false;
  unsigned long now = millis();
  if (now - lastWifiRetry > WIFI_RETRY_MS) {
    lastWifiRetry = now;
    WiFi.disconnect();
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    Serial1.println(F("[WIFI] Yeniden deneniyor..."));
  }
}

// ==================== HTTP YARDIMCILARI ====================
// Kalıcı manuel keep-alive bağlantısı belleği tuttuğu için, transient çağrılar
// KÜÇÜK buffer kullanır (varsayılan 16KB yerine ~5KB) → yan yana sığsınlar.
// Bellek yine de düşükse çağrıyı atla → manuel keep-alive çökmesin (önceliklidir).
bool heapOkForHttps() { return ESP.getFreeHeap() > 11000; }

void smallTls(BearSSL::WiFiClientSecure& c) {
  c.setInsecure();
  c.setSession(&firebaseSession);   // oturum resume → hızlı handshake
  c.setBufferSizes(4096, 512);      // küçük buffer → az bellek (transient için yeter)
}

// GET → response body döner, hata varsa boş String
String httpGet(const String& url) {
  if (!heapOkForHttps()) return "";
  BearSSL::WiFiClientSecure client;
  smallTls(client);
  HTTPClient https;
  https.setReuse(false);
  https.setTimeout(HTTP_TIMEOUT_MS);
  if (!https.begin(client, url)) return "";
  int code = https.GET();
  String body = (code == HTTP_CODE_OK) ? https.getString() : "";
  https.end();
  return body;
}

// PATCH → gönder, http kodu döner
int httpPatch(const String& url, const String& json) {
  if (!heapOkForHttps()) return -1;
  BearSSL::WiFiClientSecure client;
  smallTls(client);
  HTTPClient https;
  https.setTimeout(HTTP_TIMEOUT_MS);
  if (!https.begin(client, url)) return -1;
  https.addHeader(F("Content-Type"), F("application/json"));
  int code = https.sendRequest("PATCH", json);
  https.end();
  return code;
}

// POST → Firebase'de yeni push anahtarı oluşturur (mlTrainingData için)
int httpPost(const String& url, const String& json) {
  if (!heapOkForHttps()) return -1;
  BearSSL::WiFiClientSecure client;
  smallTls(client);
  HTTPClient https;
  https.setTimeout(HTTP_TIMEOUT_MS);
  if (!https.begin(client, url)) return -1;
  https.addHeader(F("Content-Type"), F("application/json"));
  int code = https.POST(json);
  https.end();
  return code;
}

// ==================== JSON MİNİ PARSER ====================
// Çok basit: ilk eşleşen key'in değerini çeker.
// Sadece düz (nested olmayan) alanlar için.

int jsonFindValue(const String& json, const char* key) {
  int ki = json.indexOf(key);
  if (ki < 0) return -1;
  int colon = json.indexOf(':', ki + strlen(key));
  if (colon < 0) return -1;
  int vi = colon + 1;
  while (vi < (int)json.length() && (json[vi] == ' ' || json[vi] == '\t')) vi++;
  return vi;
}

bool jsonBool(const String& json, const char* key, bool fallback = false) {
  int vi = jsonFindValue(json, key);
  if (vi < 0) return fallback;
  return json[vi] == 't';
}

int jsonInt(const String& json, const char* key, int fallback = 0) {
  int vi = jsonFindValue(json, key);
  if (vi < 0) return fallback;
  return json.substring(vi).toInt();
}

float jsonFloat(const String& json, const char* key, float fallback = 0) {
  int vi = jsonFindValue(json, key);
  if (vi < 0) return fallback;
  return json.substring(vi).toFloat();
}

String jsonStr(const String& json, const char* key, const String& fallback = "") {
  int vi = jsonFindValue(json, key);
  if (vi < 0 || json[vi] != '"') return fallback;
  int end = json.indexOf('"', vi + 1);
  if (end < 0) return fallback;
  return json.substring(vi + 1, end);
}

// ==================== ARDUINO'YA KOMUT GÖNDER ====================
// Protokol: >C,<mode>,<manEn>,<cmd>,<speed>\n
void sendToArduino(const char* mode, bool manEn, const char* cmd, int spd) {
  Serial.print(F(">C,"));
  Serial.print(mode);
  Serial.print(',');
  Serial.print(manEn ? '1' : '0');
  Serial.print(',');
  Serial.print(cmd);
  Serial.print(',');
  Serial.println(spd);
}

// Protokol: >P,<kp>,<ki>,<kd>,<base>,<max>\n
void sendPidToArduino(float kp, float ki, float kd, int base, int maxS) {
  Serial.print(F(">P,"));
  Serial.print(kp, 3); Serial.print(',');
  Serial.print(ki, 3); Serial.print(',');
  Serial.print(kd, 3); Serial.print(',');
  Serial.print(base);  Serial.print(',');
  Serial.println(maxS);
}

// ESP sürümünü Arduino'ya bildir (>V). Arduino [ESP] ver=... satırında gösterir.
unsigned long lastVersionPushMs = 0;
void sendVersionIfNeeded() {
  if (millis() - lastVersionPushMs < VERSION_PUSH_MS) return;
  lastVersionPushMs = millis();
  Serial.print(F(">V,")); Serial.println(F(ESP_FW_VERSION));
}

// ==================== PID AYAR POLLİNG ====================
void readPidIfNeeded() {
  if (!wifiOk) return;
  if (millis() - lastPidPollMs < PID_POLL_MS) return;
  lastPidPollMs = millis();

  String body = httpGet(pidUrl());
  if (body.length() == 0) return;

  // updatedAt'i sürüm olarak kullan (her kaydetmede değişir)
  String ver = String(jsonInt(body, "updatedAt", 0));
  if (ver == lastPidVersion) return;
  lastPidVersion = ver;

  float kp = jsonFloat(body, "kp", 18.0);
  float ki = jsonFloat(body, "ki", 0.0);
  float kd = jsonFloat(body, "kd", 8.0);
  int base = constrain(jsonInt(body, "baseSpeed", 145), 0, 255);
  int maxS = constrain(jsonInt(body, "maxSpeed", 255), 0, 255);

  sendPidToArduino(kp, ki, kd, base, maxS);

  Serial1.print(F("[PID] kp=")); Serial1.print(kp);
  Serial1.print(F(" kd="));      Serial1.print(kd);
  Serial1.print(F(" base="));    Serial1.println(base);
}

// ==================== MANUAL CONTROL — KEEP-ALIVE POLLING ====================
// Tek kalıcı HTTPS bağlantısı üzerinden manualControls'u poll eder. setReuse(true)
// sayesinde ilk GET handshake yapar, sonrakiler bağlantıyı yeniden kullanır → ~100-200ms.
void applyManualBody(const String& body) {
  // MOD + expectedNextStop'u BU hızlı keep-alive okumasından al. (Web bunları
  // manualControls'a da yazıyor.) Ayrı "command" GET'i kalıcı keep-alive bağlantısı
  // belleği yüzünden bloke olabiliyor; bu yüzden mod oradan değil BURADAN geliyor.
  // Bu, sürümden BAĞIMSIZ her okumada uygulanır — manuel komut parse'ı AŞAĞIDA aynen durur.
  String m = jsonStr(body, "mode", "");
  if (m.length() > 0) {
    strncpy(vehicleMode, m.c_str(), sizeof(vehicleMode) - 1);
    vehicleMode[sizeof(vehicleMode) - 1] = '\0';
  }
  int ns = jsonInt(body, "expectedNextStop", -1);
  if (ns >= 0) {
    String nss = String(ns);
    if (nss != lastNextStopSent) {
      lastNextStopSent = nss;
      Serial.print(F(">N,")); Serial.println(ns);
    }
  }

  // PID'i de BURADAN gönder (güvenilir keep-alive). Ayrı PID GET'i heap-block oluyordu →
  // panelden değiştirilen Kp/Kd araca ULAŞMIYORDU. Web bunları manualControls'a da yazar.
  // Yalnızca pid alanları varsa ve değiştiyse >P gönder. Manuel hattını/heap'i yormamak için
  // en fazla saniyede bir parse et (PID değişiminin 1sn'de uygulanması fazlasıyla yeterli).
  if (body.indexOf("\"kp\"") >= 0 && millis() - lastPidKaMs > 1000) {
    lastPidKaMs = millis();
    float kp = jsonFloat(body, "kp", 18.0);
    float ki = jsonFloat(body, "ki", 0.0);
    float kd = jsonFloat(body, "kd", 8.0);
    int base = constrain(jsonInt(body, "baseSpeed", 145), 0, 255);
    int maxS = constrain(jsonInt(body, "maxSpeed", 255), 0, 255);
    String sig = String(kp,1)+","+String(ki,1)+","+String(kd,1)+","+String(base)+","+String(maxS);
    if (sig != lastPidSigKA) {
      lastPidSigKA = sig;
      sendPidToArduino(kp, ki, kd, base, maxS);
      Serial1.print(F("[PID-KA] ")); Serial1.println(sig);
    }
  }

  String ver = jsonStr(body, "commandVersion", "0");
  if (ver == lastManualVersion) return;      // manuel komutta değişiklik yok
  lastManualVersion = ver;

  bool en = jsonBool(body, "enabled");
  manualEnabled = en;
  if (!en) {
    strncpy(manualCmd, "STOP", sizeof(manualCmd));
    manualSpeed = 0;
  } else {
    String cmd = jsonStr(body, "command", "STOP");
    strncpy(manualCmd, cmd.c_str(), sizeof(manualCmd) - 1);
    manualCmd[sizeof(manualCmd) - 1] = '\0';
    manualSpeed = constrain(jsonInt(body, "speed", 0), 0, 255);
  }
  Serial1.print(F("[MANUAL] en=")); Serial1.print(en);
  Serial1.print(F(" cmd=")); Serial1.print(manualCmd);
  Serial1.print(F(" spd=")); Serial1.println(manualSpeed);
}

void readManualControl() {
  if (!wifiOk) {
    if (manualHttpReady) { manualHttp.end(); manualHttpReady = false; }
    return;
  }
  if (millis() - lastManualPollMs < MANUAL_POLL_MS) return;
  lastManualPollMs = millis();

  // Kalıcı bağlantıyı (gerekirse) kur — handshake yalnızca ilk seferde.
  if (!manualHttpReady) {
    if (millis() - lastManualConnAttempt < 1000) return;   // backoff
    lastManualConnAttempt = millis();
    manualClient.setInsecure();
    manualHttp.setReuse(true);                              // KEEP-ALIVE
    manualHttp.setTimeout(HTTP_TIMEOUT_MS);
    if (!manualHttp.begin(manualClient, manualControlUrl())) return;
    manualHttpReady = true;
  }

  int code = manualHttp.GET();
  if (code <= 0) {                       // bağlantı koptu → sıfırla, yeniden kur
    manualHttp.end();
    manualHttpReady = false;
    return;
  }
  if (code != HTTP_CODE_OK) return;      // bağlantıyı koru, bu turu atla

  String body = manualHttp.getString();
  if (body.length() == 0) return;
  lastManualOkMs = millis();             // başarılı okuma → bağlantı sağlam
  applyManualBody(body);
}
// ==================== VEHICLE COMMAND POLLİNG ====================
void readVehicleCommandIfNeeded() {
  if (!wifiOk) return;
  if (millis() - lastCmdPollMs < COMMAND_POLL_MS) return;
  lastCmdPollMs = millis();

  String body = httpGet(commandUrl());
  if (body.length() == 0) return;

  String ver  = jsonStr(body, "commandVersion", "0");
  String mode = jsonStr(body, "mode", "idle");
  String cmd  = jsonStr(body, "command", "NONE");

  if (ver != lastCmdVersion) {
    lastCmdVersion = ver;
    strncpy(vehicleMode, mode.c_str(), sizeof(vehicleMode) - 1);
    Serial1.print(F("[CMD] mode="));
    Serial1.print(mode);
    Serial1.print(F(" cmd="));
    Serial1.println(cmd);
  }

  // expectedNextStop değişince Arduino'ya >N ile bildir (manuel sonrası resync için)
  int ns = jsonInt(body, "expectedNextStop", 0);
  if (String(ns) != lastNextStopSent) {
    lastNextStopSent = String(ns);
    Serial.print(F(">N,"));
    Serial.println(ns);
  }
}

// ==================== ARDUINO'YA GÜNCEL MOD/KOMUT GÖNDER ====================
void pushCommandToArduino() {
  // GÜVENLİK: SADECE WiFi/stream bağlantısı KOPARSA araca STOP (kullanıcı isteği).
  // Bu bir "komut vermezsen dur" zamanlayıcısı DEĞİL — bağlantı sağlamken
  // latch'lenmiş komut sonsuza dek sürer (tıkla, bırak, devam eder).
  //   wifiOk        = WiFi.status() == WL_CONNECTED (fiziksel WiFi kopması)
  //   lastManualOkMs= son başarılı manuel okumanın zamanı
  bool connectionLost = !wifiOk || (millis() - lastManualOkMs > CONN_LOST_MS);
  if (connectionLost) {
    // Latch'i temizle ki bağlantı dönünce araç kendiliğinden harekete geçmesin
    strncpy(manualCmd, "STOP", sizeof(manualCmd));
    sendToArduino(vehicleMode, manualEnabled, "STOP", 0);
    return;
  }

  // Manuel etkinken GERÇEK modu koru (ör. calibration) + manuel bayrağını set et.
  // Böylece Arduino kalibrasyon modunda olduğunu bilir ve manuel sırasında
  // ölçümü SIFIRLAMAZ, sadece DURAKLATIR. (Mod="manual" gönderirsek ölçüm sıfırlanır.)
  if (manualEnabled) {
    sendToArduino(vehicleMode, true, manualCmd, manualSpeed);
  } else {
    sendToArduino(vehicleMode, false, "STOP", 0);
  }
}

// ==================== TELEMETRİ PARSE (Arduino → ESP) ====================
// Format: <T,lPWM,rPWM,dist,hall,station,lineCase,fsr1,fsr2,occ,encL,encR,action,mode,
//            posKnown,curStop,nextStop,etaSec,seg,lap\n

void parseTelemetry(char* line) {
  if (line[0] != '<' || line[1] != 'T') return;
  char* p = line + 3;

  char* tok;
  tok = strtok(p,    ","); if (!tok) return; tele.leftPWM         = atoi(tok);
  tok = strtok(NULL, ","); if (!tok) return; tele.rightPWM        = atoi(tok);
  tok = strtok(NULL, ","); if (!tok) return; tele.distCm          = atof(tok);
  tok = strtok(NULL, ","); if (!tok) return; tele.hallRaw         = atoi(tok);
  tok = strtok(NULL, ","); if (!tok) return; tele.stationDetected = atoi(tok);
  tok = strtok(NULL, ","); if (!tok) return; strncpy(tele.lineCase, tok, 15); tele.lineCase[15]='\0';
  tok = strtok(NULL, ","); if (!tok) return; tele.fsr1Raw         = atoi(tok);
  tok = strtok(NULL, ","); if (!tok) return; tele.fsr2Raw         = atoi(tok);
  tok = strtok(NULL, ","); if (!tok) return; strncpy(tele.occupancy, tok, 7); tele.occupancy[7]='\0';
  tok = strtok(NULL, ","); if (!tok) return; tele.encL            = atol(tok);
  tok = strtok(NULL, ","); if (!tok) return; tele.encR            = atol(tok);
  tok = strtok(NULL, ","); if (!tok) return; strncpy(tele.action, tok, 31); tele.action[31]='\0';
  tok = strtok(NULL, ","); if (!tok) return; strncpy(tele.mode, tok, 11); tele.mode[11]='\0';
  // Genişletilmiş ölçüm alanları (eski format gelirse buradan sonrası atlanır)
  tok = strtok(NULL, ","); if (!tok) return; tele.positionKnown = atoi(tok);
  tok = strtok(NULL, ","); if (!tok) return; tele.currentStop   = atoi(tok);
  tok = strtok(NULL, ","); if (!tok) return; tele.nextStop      = atoi(tok);
  tok = strtok(NULL, ","); if (!tok) return; tele.etaSec        = atoi(tok);
  tok = strtok(NULL, ","); if (!tok) return; tele.segment       = atoi(tok);
  tok = strtok(NULL, ","); if (!tok) return; tele.lapCount      = atoi(tok);
  tok = strtok(NULL, ","); if (!tok) return; tele.lapDurationMs = atol(tok);
}

// Segment raporu: >S,idx,distCm,durMs,avgPwm,lineLost,occLevel,distActLevel,lap
struct SegSample {
  bool pending = false;
  int idx; float distCm; unsigned long durMs;
  int avgPwm; int lineLost; int occLevel; int distAct; int lap;
} segSample;

void parseSegmentReport(char* line) {
  char* p = line + 3;
  int idx, avg, ll, occ, da, lap; float d; unsigned long dur;
  char* tok;
  tok = strtok(p,    ","); if (!tok) return; idx = atoi(tok);
  tok = strtok(NULL, ","); if (!tok) return; d   = atof(tok);
  tok = strtok(NULL, ","); if (!tok) return; dur = atol(tok);
  tok = strtok(NULL, ","); if (!tok) return; avg = atoi(tok);
  tok = strtok(NULL, ","); if (!tok) return; ll  = atoi(tok);
  tok = strtok(NULL, ","); if (!tok) return; occ = atoi(tok);
  tok = strtok(NULL, ","); if (!tok) return; da  = atoi(tok);
  tok = strtok(NULL, ","); if (!tok) return; lap = atoi(tok);

  // Tüm alanlar tamam → kuyruğa al (HTTPS yüklemesi loop'ta yapılır)
  segSample.idx=idx; segSample.distCm=d; segSample.durMs=dur;
  segSample.avgPwm=avg; segSample.lineLost=ll; segSample.occLevel=occ;
  segSample.distAct=da; segSample.lap=lap; segSample.pending=true;

  Serial1.print(F("[SEG] idx=")); Serial1.print(idx);
  Serial1.print(F(" dur="));      Serial1.print(dur/1000.0);
  Serial1.print(F("s dist="));    Serial1.println(d);
}

void readArduinoIfAvailable() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n') {
      arBuf[arIdx] = '\0';
      if (arIdx > 3) {
        if (arBuf[0] == '<' && arBuf[1] == 'T')      parseTelemetry(arBuf);
        else if (arBuf[0] == '>' && arBuf[1] == 'S') parseSegmentReport(arBuf);
      }
      arIdx = 0;
    } else if (arIdx < sizeof(arBuf) - 1) {
      arBuf[arIdx++] = c;
    }
  }
}

// ==================== FIREBASE'E TELEMETRİ YAZ ====================
void pushTelemetryIfNeeded() {
  if (!wifiOk) return;
  if (millis() - lastTelemetryPushMs < TELEMETRY_PUSH_MS) return;
  lastTelemetryPushMs = millis();

  // Doluluk nesnesi (guest.html bunu okuyor, yapıyı koru)
  String occLabel = (strcmp(tele.occupancy, "empty") == 0)   ? F("Empty") :
                    (strcmp(tele.occupancy, "partial") == 0)  ? F("Partially full") :
                                                                 F("Full");
  String occColor = (strcmp(tele.occupancy, "empty") == 0)   ? F("green") :
                    (strcmp(tele.occupancy, "partial") == 0)  ? F("yellow") :
                                                                 F("red");

  String json = F("{");

  json += F("\"online\":true,");
  json += F("\"wifiConnected\":true,");
  json += F("\"firebaseConnected\":true,");
  // Firebase sunucu zaman damgası (gerçek epoch ms) — panel "online" hesabı buna güvenir.
  // millis() yazarsak panel Date.now()-millis() devasa cikar ve hep "offline" gosterir.
  json += F("\"lastSeen\":{\".sv\":\"timestamp\"},");
  // Manuel etkinse panele "manual" göster (guest ETA'yı manuelde gizler)
  json += F("\"mode\":\"");   json += (manualEnabled ? "manual" : tele.mode); json += F("\",");
  json += F("\"leftPWM\":");  json += tele.leftPWM;   json += ',';
  json += F("\"rightPWM\":"); json += tele.rightPWM;  json += ',';
  json += F("\"distanceCm\":"); json += tele.distCm;  json += ',';
  json += F("\"hallRaw\":");  json += tele.hallRaw;   json += ',';
  json += F("\"stationDetected\":"); json += (tele.stationDetected ? F("true") : F("false")); json += ',';
  json += F("\"lineCase\":\""); json += tele.lineCase; json += F("\",");
  json += F("\"action\":\"");  json += tele.action;   json += F("\",");
  json += F("\"encL\":");  json += tele.encL;  json += ',';
  json += F("\"encR\":");  json += tele.encR;  json += ',';

  // Konum / ETA alanları (guest paneli bunları okur)
  bool posKnown = (tele.positionKnown == 1);
  bool etaUsable = posKnown && tele.etaSec > 0 && !manualEnabled;
  json += F("\"positionKnown\":");      json += (posKnown ? F("true") : F("false")); json += ',';
  json += F("\"etaValid\":");           json += (etaUsable ? F("true") : F("false")); json += ',';
  json += F("\"needsPositionConfirm\":"); json += (posKnown ? F("false") : F("true")); json += ',';
  json += F("\"currentStop\":");        json += tele.currentStop; json += ',';
  json += F("\"currentStopIndex\":");   json += tele.currentStop; json += ',';
  json += F("\"nextStop\":");           json += tele.nextStop;    json += ',';
  json += F("\"nextStopIndex\":");      json += tele.nextStop;    json += ',';
  json += F("\"etaToNextStop\":");      json += tele.etaSec;      json += ',';
  json += F("\"segment\":");            json += tele.segment;     json += ',';
  json += F("\"lapCount\":");           json += tele.lapCount;    json += ',';
  json += F("\"lapDurationSec\":");     json += (tele.lapDurationMs / 1000.0); json += ',';

  // occupancy nesnesi (mevcut şemayı koru)
  json += F("\"occupancy\":{");
  json += F("\"fsr1Raw\":"); json += tele.fsr1Raw;  json += ',';
  json += F("\"fsr2Raw\":"); json += tele.fsr2Raw;  json += ',';
  json += F("\"status\":\"");  json += tele.occupancy; json += F("\",");
  json += F("\"label\":\"");   json += occLabel;       json += F("\",");
  json += F("\"color\":\"");   json += occColor;       json += F("\",");
  json += F("\"adsReady\":"); json += (tele.fsr1Raw > 0 || tele.fsr2Raw > 0 ? F("true") : F("false"));
  json += '}';

  json += '}';

  int code = httpPatch(carUrl(), json);
  Serial1.print(F("[FB] PATCH "));
  Serial1.println(code);
}

// ==================== ML SEGMENT ÖRNEĞİNİ YÜKLE ====================
// Arduino'dan gelen her segment → mlTrainingData/{carId} altına POST (push).
// Web paneli (TensorFlow.js) bu örneklerle ETA modelini eğitir.
void uploadSegmentIfPending() {
  if (!segSample.pending || !wifiOk) return;
  segSample.pending = false;

  float durSec = segSample.durMs / 1000.0f;

  String json = F("{");
  json += F("\"carId\":\"" CAR_ID "\",");
  // v1: sabit routeVersion=1. Web paneli system.activeRouteVersion=1 ayarlar;
  // ML eğitimi yalnızca routeVersion == activeRouteVersion örnekleri kullanır.
  json += F("\"routeVersion\":1,");
  json += F("\"segmentIndex\":");        json += segSample.idx;      json += ',';
  json += F("\"segmentDistance\":");     json += String(segSample.distCm, 1); json += ',';
  json += F("\"actualDurationSec\":");   json += String(durSec, 2);  json += ',';
  json += F("\"calibratedDurationSec\":"); json += String(durSec, 2); json += ',';
  json += F("\"occupancyLevel\":");      json += segSample.occLevel; json += ',';
  json += F("\"distanceActionLevel\":"); json += segSample.distAct;  json += ',';
  json += F("\"avgPWM\":");              json += segSample.avgPwm;   json += ',';
  json += F("\"lineLost\":");            json += segSample.lineLost; json += ',';
  json += F("\"lap\":");                 json += segSample.lap;      json += ',';
  json += F("\"createdAt\":{\".sv\":\"timestamp\"}");
  json += '}';

  int code = httpPost(mlUrl(), json);
  Serial1.print(F("[ML] segment idx="));
  Serial1.print(segSample.idx);
  Serial1.print(F(" POST "));
  Serial1.println(code);
}

// ==================== SETUP ====================
void setup() {
  // Serial (Hardware UART) ↔ Arduino
  Serial.begin(9600);

  // Serial1 (GPIO2 / D4) → USB-Serial adaptörü ile debug
  Serial1.begin(115200);
  delay(100);

  Serial1.println(F("\n=== MEGABUS ESP8266 Bridge " ESP_FW_VERSION " ==="));
  Serial1.print(F("CAR_ID: ")); Serial1.println(F(CAR_ID));

  // Sürümü hemen Arduino'ya bildir (>V) — boot'ta görünsün
  Serial.print(F(">V,")); Serial.println(F(ESP_FW_VERSION));

  startWifi();

  // Wi-Fi bağlanana kadar bekle (max 10 sn)
  unsigned long t = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t < 10000) {
    delay(300);
    Serial1.print('.');
  }
  Serial1.println();

  if (WiFi.status() == WL_CONNECTED) {
    wifiOk = true;
    Serial1.print(F("[WIFI] Bağlandı: "));
    Serial1.println(WiFi.localIP());
  } else {
    Serial1.println(F("[WIFI] Bağlanamadı, arka planda denemeye devam."));
  }
}

// ==================== LOOP ====================
void loop() {
  maintainWifi();
  readArduinoIfAvailable();

  // Manuel: hızlı poll (öncelikli) + komutu sık gönder
  pushCommandToArduino();
  readManualControl();
  pushCommandToArduino();

  // Latency-kritik olmayanlar (seyrek)
  readVehicleCommandIfNeeded();
  readPidIfNeeded();
  uploadSegmentIfPending();
  pushTelemetryIfNeeded();
  sendVersionIfNeeded();   // ESP sürümünü Arduino'ya bildir (>V)

  delay(2);
}
