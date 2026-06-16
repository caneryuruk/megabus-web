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
// Manuel kontrol artık POLLING değil STREAMING (SSE) ile geliyor → ~200-400ms gecikme.
// Diğerleri (mod/pid/telemetri) latency-kritik değil, seyrek polling yeterli.
#define COMMAND_POLL_MS     5000    // commands (mod nadiren değişir)
#define TELEMETRY_PUSH_MS   3000    // Firebase'e telemetry yazma (panel için, kritik değil)
#define WIFI_RETRY_MS      30000    // Wi-Fi yeniden bağlanma denemesi
#define PID_POLL_MS        10000    // PID ayarları (yalnızca Save'de değişir)
#define HTTP_TIMEOUT_MS     4000    // HTTPS çağrıları için zaman aşımı (takılma önler)

// ==================== MANUEL STREAMING ====================
#define STREAM_RECONNECT_MS  1500   // stream koptuysa yeniden deneme aralığı
#define STREAM_STALE_MS     60000   // bu kadar süre bayt/keep-alive yoksa yeniden bağlan
#define CONN_LOST_MS         2000   // stream bu kadar süredir sağlıksızsa → STOP (güvenlik)

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
unsigned long lastManualOkMs = 0;       // manuel veri (stream VEYA poll) son sağlıklı an
String lastManualVersion = "";          // poll fallback için sürüm dedup
unsigned long lastManualPollMs = 0;     // poll fallback zamanlayıcı

// Manuel kontrol — Firebase streaming (SSE) durumu
BearSSL::WiFiClientSecure streamClient;
bool   streamConnected      = false;
unsigned long lastStreamByteMs    = 0;  // son bayt/keep-alive
unsigned long lastStreamAttemptMs = 0;
String streamLine  = "";
String streamEvent = "";

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
} tele;

unsigned long lastTelemetryPushMs = 0;

// Arduino'dan gelen satır buffer'ı (genişletilmiş telemetri ~117 karakter olabilir)
char  arBuf[140];
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
// Transient HTTPS bir handshake için ~16KB ister. Kalıcı manuel stream açıkken
// bellek düşükse bu çağrıları atla → stream çökmesin (manuel kontrol önceliklidir).
bool heapOkForHttps() { return ESP.getFreeHeap() > 20000; }

// GET → response body döner, hata varsa boş String
String httpGet(const String& url) {
  if (!heapOkForHttps()) return "";
  BearSSL::WiFiClientSecure client;
  client.setInsecure();
  HTTPClient https;
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
  client.setInsecure();
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
  client.setInsecure();
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

  float kp = jsonFloat(body, "kp", 35.0);
  float ki = jsonFloat(body, "ki", 0.0);
  float kd = jsonFloat(body, "kd", 20.0);
  int base = constrain(jsonInt(body, "baseSpeed", 180), 0, 255);
  int maxS = constrain(jsonInt(body, "maxSpeed", 255), 0, 255);

  sendPidToArduino(kp, ki, kd, base, maxS);

  Serial1.print(F("[PID] kp=")); Serial1.print(kp);
  Serial1.print(F(" kd="));      Serial1.print(kd);
  Serial1.print(F(" base="));    Serial1.println(base);
}

// ==================== MANUAL CONTROL — FIREBASE STREAMING (SSE) ====================
// Kalıcı bağlantı: Firebase manualControls değişince ANINDA push eder.
// Polling'in ~1sn handshake gecikmesi YOK → manuel gecikme ~200-400ms.

String firebaseHost() {
  String h = String(FIREBASE_URL);
  h.replace("https://", "");
  h.replace("http://", "");
  int slash = h.indexOf('/');
  if (slash >= 0) h = h.substring(0, slash);
  return h;
}

void connectManualStream() {
  streamClient.stop();
  streamConnected = false;
  streamClient.setInsecure();
  // Küçük TLS buffer (MFLN) — kalıcı bağlantı belleği düşük kalsın, transient
  // çağrılara (telemetri/komut/pid) yer kalsın.
  streamClient.setBufferSizes(1024, 512);
  streamClient.setTimeout(4000);

  String host = firebaseHost();
  String path = String("/manualControls/" CAR_ID ".json");

  // Firebase streaming 307/301/302 ile bölgesel sunucuya yönlendirebilir → 1 kez izle
  for (int attempt = 0; attempt < 2; attempt++) {
    if (!streamClient.connect(host.c_str(), 443)) {
      Serial1.println(F("[STREAM] connect FAIL"));
      return;
    }

    streamClient.print(String("GET ") + path + " HTTP/1.1\r\n");
    streamClient.print(String("Host: ") + host + "\r\n");
    streamClient.print(F("Accept: text/event-stream\r\n"));
    streamClient.print(F("Connection: keep-alive\r\n\r\n"));

    String status = streamClient.readStringUntil('\n');
    String location = "";
    unsigned long t = millis();
    while (streamClient.connected() && millis() - t < 4000) {
      String line = streamClient.readStringUntil('\n');
      if (line.length() <= 1) break;             // boş satır = başlık sonu
      String low = line; low.toLowerCase();
      if (low.startsWith("location:")) { location = line.substring(9); location.trim(); }
    }

    if (status.indexOf("200") >= 0) {
      streamConnected  = true;
      lastStreamByteMs = millis();
      lastManualOkMs   = millis();
      streamLine = ""; streamEvent = "";
      Serial1.println(F("[STREAM] connected"));
      return;
    }

    bool redirect = (status.indexOf("307") >= 0 || status.indexOf("301") >= 0 ||
                     status.indexOf("302") >= 0);
    if (redirect && location.length() > 0) {
      location.replace("https://", ""); location.replace("http://", "");
      int sl = location.indexOf('/');
      if (sl >= 0) { host = location.substring(0, sl); path = location.substring(sl); }
      else         { host = location; }
      streamClient.stop();
      Serial1.print(F("[STREAM] redirect -> ")); Serial1.println(host);
      continue;
    }

    Serial1.print(F("[STREAM] HTTP: ")); Serial1.println(status);
    streamClient.stop();
    return;
  }
  Serial1.println(F("[STREAM] redirect basarisiz"));
  streamClient.stop();
}

// {"path":"/...","data":{...}} → manualEnabled/manualCmd/manualSpeed uygula.
// writeManualControl her zaman tüm seti update eder; eksik alanlar mevcut değerde kalır.
void applyStreamData(const String& json) {
  bool   en  = jsonBool(json, "enabled", manualEnabled);
  String cmd = jsonStr(json, "command", String(manualCmd));
  int    spd = jsonInt(json, "speed", manualSpeed);

  manualEnabled = en;
  if (!en) {
    strncpy(manualCmd, "STOP", sizeof(manualCmd));
    manualSpeed = 0;
  } else {
    strncpy(manualCmd, cmd.c_str(), sizeof(manualCmd) - 1);
    manualCmd[sizeof(manualCmd) - 1] = '\0';
    manualSpeed = constrain(spd, 0, 255);
  }

  Serial1.print(F("[STREAM] en=")); Serial1.print(en);
  Serial1.print(F(" cmd="));        Serial1.print(manualCmd);
  Serial1.print(F(" spd="));        Serial1.println(manualSpeed);
}

void handleStreamLine(const String& line) {
  if (line.startsWith("event:")) {
    streamEvent = line.substring(6); streamEvent.trim();
  } else if (line.startsWith("data:")) {
    String data = line.substring(5); data.trim();
    if (streamEvent == "put" || streamEvent == "patch") {
      if (data.indexOf("\"data\":null") >= 0) {
        // manualControls silindi → güvenli dur
        manualEnabled = false;
        strncpy(manualCmd, "STOP", sizeof(manualCmd));
        manualSpeed = 0;
      } else {
        applyStreamData(data);
      }
    }
    // keep-alive / auth_revoked / cancel → yoksay
  }
}

void processManualStream() {
  if (!wifiOk) { streamClient.stop(); streamConnected = false; return; }

  if (streamConnected && streamClient.connected()) {
    lastManualOkMs = millis();
    while (streamClient.available()) {
      char c = (char)streamClient.read();
      lastStreamByteMs = millis();
      if (c == '\n') {
        if (streamLine.length() > 0) handleStreamLine(streamLine);
        streamLine = "";
      } else if (c != '\r') {
        if (streamLine.length() < 500) streamLine += c;
      }
    }
    // Uzun süre hiç bayt yok (keep-alive bile) → bağlantı ölmüş, yeniden kur
    if (millis() - lastStreamByteMs > STREAM_STALE_MS) {
      Serial1.println(F("[STREAM] stale -> reconnect"));
      streamClient.stop();
      streamConnected = false;
    }
  } else {
    streamConnected = false;
    if (millis() - lastStreamAttemptMs > STREAM_RECONNECT_MS) {
      lastStreamAttemptMs = millis();
      connectManualStream();
    }
  }
}

// FALLBACK: stream bağlanamazsa (ör. TLS/MFLN sorunu) polling ile manuel kontrol
// çalışmaya devam etsin (~1sn gecikme). Stream bağlıyken bu çağrılmaz.
#define MANUAL_FALLBACK_POLL_MS 500
void readManualControlFallback() {
  if (!wifiOk) return;
  if (millis() - lastManualPollMs < MANUAL_FALLBACK_POLL_MS) return;
  lastManualPollMs = millis();

  String body = httpGet(manualControlUrl());
  if (body.length() == 0) return;            // okuma başarısız → bağlantı sağlığı düşmez
  lastManualOkMs = millis();                 // başarılı okuma → bağlantı sağlam

  String ver = jsonStr(body, "commandVersion", "0");
  if (ver == lastManualVersion) return;      // değişiklik yok
  lastManualVersion = ver;

  bool en = jsonBool(body, "enabled");
  manualEnabled = en;
  if (!en) { strncpy(manualCmd, "STOP", sizeof(manualCmd)); manualSpeed = 0; }
  else {
    String cmd = jsonStr(body, "command", "STOP");
    strncpy(manualCmd, cmd.c_str(), sizeof(manualCmd) - 1);
    manualCmd[sizeof(manualCmd) - 1] = '\0';
    manualSpeed = constrain(jsonInt(body, "speed", 0), 0, 255);
  }
  Serial1.print(F("[MANUAL poll] en=")); Serial1.print(en);
  Serial1.print(F(" cmd=")); Serial1.println(manualCmd);
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
}

// ==================== ARDUINO'YA GÜNCEL MOD/KOMUT GÖNDER ====================
void pushCommandToArduino() {
  // GÜVENLİK: SADECE WiFi/stream bağlantısı KOPARSA araca STOP (kullanıcı isteği).
  // Bu bir "komut vermezsen dur" zamanlayıcısı DEĞİL — bağlantı sağlamken
  // latch'lenmiş komut sonsuza dek sürer (tıkla, bırak, devam eder).
  //   wifiOk        = WiFi.status() == WL_CONNECTED (fiziksel WiFi kopması)
  //   lastManualOkMs= stream bağlantısının sağlıklı olduğu son an
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

  Serial1.println(F("\n=== MEGABUS ESP8266 Bridge v1 ==="));
  Serial1.print(F("CAR_ID: ")); Serial1.println(F(CAR_ID));

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

  // Manuel komutlar STREAM ile gelir (anlık işlenir) — düşük gecikme.
  processManualStream();
  if (!streamConnected) readManualControlFallback();   // stream yoksa polling
  pushCommandToArduino();

  readVehicleCommandIfNeeded();
  readPidIfNeeded();

  processManualStream();       // transient poll'lardan sonra stream'i tekrar oku
  pushCommandToArduino();

  uploadSegmentIfPending();
  pushTelemetryIfNeeded();

  delay(2);
}
