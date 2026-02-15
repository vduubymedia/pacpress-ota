#include <Arduino.h>
#include <WiFi.h>
#include <WebSocketsServer.h>
#include <ModbusMaster.h>
#include <Preferences.h>
#include <ArduinoJson.h>

// =======================
//  PRODUCT NAME
// =======================
static const char* DEVICE_NAME = "Pacpress Controller";

// =======================
//  DEFAULT WIFI
// =======================
static const char* DEFAULT_WIFI_SSID = "IP frequently";
static const char* DEFAULT_WIFI_PASS = "J@bbAth3HUB";

// =======================
//  FALLBACK SETUP AP
// =======================
static const char* AP_SSID = "Pacpress-Setup";
static const char* AP_PASS = "pacpress123";

// =======================
//  WEBSOCKET
// =======================
static const uint16_t WS_PORT = 8080;
WebSocketsServer ws(WS_PORT);

// =======================
//  AP3 MODBUS
// =======================
static const uint32_t PLC_BAUD     = 38400;
static const uint8_t  PLC_SLAVE_ID = 1;
static const int PLC_RX_PIN = 4;
static const int PLC_TX_PIN = 5;

HardwareSerial PLCSerial(1);
ModbusMaster node;

// =======================
//  ROUND-ROBIN MODBUS (more stable)
// =======================
static const uint32_t MODBUS_TICK_MS = 50; // one request per tick
static const uint32_t MODBUS_GAP_MS  = 10; // quiet gap after request

// =======================
//  ADDRESSES
// =======================
static inline uint16_t REG_DS(int dsNumber) { return (uint16_t)(dsNumber - 1); } // DS1->0
static inline uint16_t DIN_X(int xNumber)   { return (uint16_t)(xNumber - 1); } // X001->0
static inline uint16_t COIL_C(int cNumber)  { return (uint16_t)(cNumber - 1); } // C1->0
static inline uint16_t HOLD_OFFS_FROM_4X(uint32_t addr4x) { return (uint16_t)(addr4x - 400001UL); }

// DF3 float start (2 regs)
static uint16_t DF3_REG_START = HOLD_OFFS_FROM_4X(428677UL);

// =======================
//  STATE (AP3)
// =======================
int ds1StartPsi = 0, ds2FinalPsi = 0, ds3Inc = 0, ds4Stroke = 0, ds5FinalT = 0;

bool pressClosed = false;
int pumpCount = 0;
bool lastFlow = false;

float outputPsi = 0.0f;
bool dfWordSwap = false;
bool dfValid = false;

// =======================
//  INFERRED CYCLE
// =======================
bool cycleOn = false;
bool cycleOver = false;
float psiPrev = 0.0f;
uint32_t psiPrevMs = 0;
uint32_t lastRiseMs = 0;
uint32_t holdStartMs = 0;
bool holding = false;

static const float PSI_RISE_EPS   = 0.08f;
static const float PSI_FLAT_EPS   = 0.05f;
static const uint32_t STALE_STOP_MS   = 2500;
static const uint32_t HOLD_CONFIRM_MS = 800;

// =======================
//  HEALTH
// =======================
bool dsOk = false, xOk = false, dfOk = false;
uint8_t errDS = 0, errX = 0, errDF = 0;

// =======================
//  WIFI
// =======================
Preferences prefs;
String staSsid, staPass;
bool staConnected = false;
bool apMode = false;

// =======================
//  WRITE QUEUE (executes inside modbus loop)
// =======================
volatile bool pendingDsWrite = false;
int wDS1=0,wDS2=0,wDS3=0,wDS4=0,wDS5=0;
bool hasDS1=false,hasDS2=false,hasDS3=false,hasDS4=false,hasDS5=false;

volatile bool pendingStart = false;
volatile bool pendingStop  = false;

// ACK info for app
volatile bool pendingAck = false;
String ackJson;

// prevent WS from touching modbus (all modbus in loop only)
volatile bool modbusBusy = false;

// =======================
//  HELPERS
// =======================
static void clearRxNoise() {
  while (PLCSerial.available()) (void)PLCSerial.read();
}

static void startFallbackAP() {
  WiFi.mode(WIFI_AP_STA);
  bool ok = WiFi.softAP(AP_SSID, AP_PASS);
  apMode = ok;
  Serial.println("Fallback AP started.");
  Serial.printf("AP SSID: %s\n", AP_SSID);
  Serial.printf("AP PASS: %s\n", AP_PASS);
  Serial.printf("AP IP:   %s\n", WiFi.softAPIP().toString().c_str());
}

static bool connectSTA(const String& ssid, const String& pass) {
  if (ssid.isEmpty()) return false;

  WiFi.mode(WIFI_AP_STA);
  WiFi.begin(ssid.c_str(), pass.c_str());

  Serial.printf("STA connect to: %s\n", ssid.c_str());
  uint32_t t0 = millis();
  while (millis() - t0 < 12000) {
    if (WiFi.status() == WL_CONNECTED) {
      staConnected = true;
      Serial.println();
      Serial.printf("STA connected. IP: %s\n", WiFi.localIP().toString().c_str());
      return true;
    }
    delay(150);
    Serial.print(".");
  }
  Serial.println();
  staConnected = false;
  Serial.println("STA connect FAILED.");
  return false;
}

static String makeAck(const char* ack, bool ok, int err, const char* stage) {
  StaticJsonDocument<256> doc;
  doc["ack"] = ack;
  doc["ok"] = ok;
  doc["err"] = err;
  doc["stage"] = stage;
  String out;
  serializeJson(doc, out);
  return out;
}

// =======================
//  MODBUS READS
// =======================
bool readDSBlock() {
  clearRxNoise();
  uint8_t r = node.readHoldingRegisters(0, 5);
  if (r != node.ku8MBSuccess) { errDS = r; return false; }

  errDS = 0; // ✅ clear on success

  ds1StartPsi = node.getResponseBuffer(0);
  ds2FinalPsi = node.getResponseBuffer(1);
  ds3Inc      = (int16_t)node.getResponseBuffer(2);
  ds4Stroke   = node.getResponseBuffer(3);
  ds5FinalT   = node.getResponseBuffer(4);
  return true;
}

bool readXBlock() {
  clearRxNoise();
  uint8_t r = node.readDiscreteInputs(DIN_X(1), 3);
  if (r != node.ku8MBSuccess) { errX = r; return false; }

  errX = 0; // ✅ clear on success

  uint16_t bits = node.getResponseBuffer(0);
  pressClosed = ((bits >> 1) & 1) != 0;

  bool flowNow = ((bits >> 2) & 1) != 0;
  if (!lastFlow && flowNow) pumpCount++;
  lastFlow = flowNow;
  return true;
}

static bool parseFloatWords(uint16_t w0, uint16_t w1, bool swap, float &out) {
  uint32_t u = swap ? (((uint32_t)w1 << 16) | (uint32_t)w0)
                    : (((uint32_t)w0 << 16) | (uint32_t)w1);
  float f;
  memcpy(&f, &u, sizeof(f));
  if (!isfinite(f)) return false;
  if (f < -10.0f || f > 5000.0f) return false;
  out = f;
  return true;
}

bool readDF3Float() {
  clearRxNoise();
  uint8_t r = node.readHoldingRegisters(DF3_REG_START, 2);
  if (r != node.ku8MBSuccess) { errDF = r; return false; }

  errDF = 0; // ✅ clear on success

  uint16_t w0 = node.getResponseBuffer(0);
  uint16_t w1 = node.getResponseBuffer(1);

  float f;
  if (parseFloatWords(w0, w1, dfWordSwap, f)) {
    outputPsi = f; dfValid = true; return true;
  }
  if (parseFloatWords(w0, w1, !dfWordSwap, f)) {
    dfWordSwap = !dfWordSwap;
    outputPsi = f; dfValid = true; return true;
  }
  dfValid = false;
  return true;
}

// =======================
//  MODBUS WRITES (queued)
// =======================
bool doQueuedDsWrite(uint8_t &errOut) {
  auto w = [&](bool has, int dsNum, int v)->bool{
    if (!has) return true;
    clearRxNoise();
    uint8_t r = node.writeSingleRegister(REG_DS(dsNum), (uint16_t)v);
    if (r != node.ku8MBSuccess) { errOut = r; return false; }
    delay(2);
    return true;
  };

  errOut = 0;
  if (!w(hasDS1,1,wDS1)) return false;
  if (!w(hasDS2,2,wDS2)) return false;
  if (!w(hasDS3,3,wDS3)) return false;
  if (!w(hasDS4,4,wDS4)) return false;
  if (!w(hasDS5,5,wDS5)) return false;
  return true;
}

// Start/Stop pulses (C1/C2)
bool doQueuedStartStop(uint8_t &errOut) {
  errOut = 0;

  auto writeCoil = [&](int cNum, uint16_t val)->bool{
    clearRxNoise();
    uint8_t r = node.writeSingleCoil(COIL_C(cNum), val);
    if (r != node.ku8MBSuccess) { errOut = r; return false; }
    delay(2);
    return true;
  };

  const uint16_t PULSE_MS = 150;

  if (pendingStart) {
    if (!writeCoil(2, 0x0000)) return false;   // stop off
    if (!writeCoil(1, 0xFF00)) return false;   // start pulse on
    delay(PULSE_MS);
    if (!writeCoil(1, 0x0000)) return false;   // start off
  }

  if (pendingStop) {
    if (!writeCoil(1, 0x0000)) return false;   // start off
    if (!writeCoil(2, 0xFF00)) return false;   // stop pulse on
    delay(PULSE_MS);
    if (!writeCoil(2, 0x0000)) return false;   // stop off
  }

  return true;
}

// =======================
//  INFER CYCLE
// =======================
void updateCycleInference() {
  uint32_t now = millis();
  if (!dfValid) { cycleOn = false; cycleOver = false; holding = false; holdStartMs = 0; return; }

  if (psiPrevMs == 0) { psiPrevMs = now; psiPrev = outputPsi; return; }

  float dpsi = outputPsi - psiPrev;
  psiPrev = outputPsi;

  if (dpsi > PSI_RISE_EPS) {
    lastRiseMs = now;
    if (!cycleOn) {
      cycleOn = true;
      cycleOver = false;
      holding = false;
      holdStartMs = 0;
    }
  }

  if (cycleOn && !holding && (now - lastRiseMs > STALE_STOP_MS)) {
    cycleOn = false;
  }

  float target = (ds2FinalPsi > 0) ? (float)ds2FinalPsi : (float)ds1StartPsi;
  bool nearTarget = (target > 0) && (fabs(outputPsi - target) <= 1.0f);
  bool flat = fabs(dpsi) <= PSI_FLAT_EPS;

  if (cycleOn && nearTarget && flat) {
    if (holdStartMs == 0) holdStartMs = now;
    if (!holding && (now - holdStartMs >= HOLD_CONFIRM_MS)) holding = true;
  } else {
    holdStartMs = 0;
  }

  if (holding && ds5FinalT > 0) {
    uint32_t holdElapsed = now - holdStartMs;
    uint32_t holdGoal = (uint32_t)ds5FinalT * 1000UL;
    if (holdElapsed >= holdGoal) {
      cycleOver = true;
      cycleOn = false;
    }
  }
}

// =======================
//  JSON
// =======================
String buildJson() {
  String s = "{";
  s += "\"deviceName\":\"" + String(DEVICE_NAME) + "\",";
  s += "\"ssid\":\"" + staSsid + "\",";
  s += "\"ip\":\"" + WiFi.localIP().toString() + "\",";
  s += "\"staConnected\":" + String(staConnected ? "true" : "false") + ",";
  s += "\"apMode\":" + String(apMode ? "true" : "false") + ",";

  s += "\"feedPsi\":" + String(outputPsi, 2) + ",";
  s += "\"pumpCount\":" + String(pumpCount) + ",";
  s += "\"pressClosed\":" + String(pressClosed ? "true" : "false") + ",";
  s += "\"cycleOn\":" + String(cycleOn ? "true" : "false") + ",";
  s += "\"cycleOver\":" + String(cycleOver ? "true" : "false") + ",";

  s += "\"startFeedPressure\":" + String(ds1StartPsi) + ",";
  s += "\"finalPressurePsi\":" + String(ds2FinalPsi) + ",";
  s += "\"feedPressureIncrement\":" + String(ds3Inc) + ",";
  s += "\"strokeCounterTimer\":" + String(ds4Stroke) + ",";
  s += "\"finalPressureTimer\":" + String(ds5FinalT) + ",";

  s += "\"dsOk\":" + String(dsOk ? "true" : "false") + ",";
  s += "\"xOk\":" + String(xOk ? "true" : "false") + ",";
  s += "\"dfOk\":" + String(dfOk ? "true" : "false") + ",";
  s += "\"dfValid\":" + String(dfValid ? "true" : "false") + ",";
  s += "\"dfSwap\":" + String(dfWordSwap ? "true" : "false") + ",";

  s += "\"errDS\":" + String((int)errDS) + ",";
  s += "\"errX\":" + String((int)errX) + ",";
  s += "\"errDF\":" + String((int)errDF);

  s += "}";
  return s;
}

// =======================
//  WS EVENT (queue writes only)
// =======================
void onWsEvent(uint8_t client, WStype_t type, uint8_t* payload, size_t length) {
  if (type == WStype_CONNECTED) {
    String snap = buildJson();
    ws.sendTXT(client, snap);
    return;
  }
  if (type != WStype_TEXT) return;

  String msg((char*)payload, length);
  msg.trim();

  StaticJsonDocument<384> doc;
  if (deserializeJson(doc, msg)) return;

  if (doc["cmd"] == "start") {
    pendingStart = true;
    pendingStop = false;
    ackJson = makeAck("start", true, 0, "queued");
    pendingAck = true;
  } else if (doc["cmd"] == "stop") {
    pendingStop = true;
    pendingStart = false;
    ackJson = makeAck("stop", true, 0, "queued");
    pendingAck = true;
  }

  if (doc.containsKey("set") && doc["set"].is<JsonObject>()) {
    JsonObject setObj = doc["set"].as<JsonObject>();

    hasDS1 = setObj.containsKey("DS1"); if (hasDS1) wDS1 = constrain((int)setObj["DS1"], 0, 5000);
    hasDS2 = setObj.containsKey("DS2"); if (hasDS2) wDS2 = constrain((int)setObj["DS2"], 0, 5000);
    hasDS3 = setObj.containsKey("DS3"); if (hasDS3) wDS3 = constrain((int)setObj["DS3"], -5000, 5000);
    hasDS4 = setObj.containsKey("DS4"); if (hasDS4) wDS4 = constrain((int)setObj["DS4"], 0, 5000);
    hasDS5 = setObj.containsKey("DS5"); if (hasDS5) wDS5 = constrain((int)setObj["DS5"], 0, 5000);

    pendingDsWrite = true;
    ackJson = makeAck("ds", true, 0, "queued");
    pendingAck = true;
  }
}

// =======================
//  MONITOR (HOME + MENU)
// =======================
void printHome() {
  Serial.println("──────────── HOME ────────────");
  Serial.printf("Device: %s\n", DEVICE_NAME);
  Serial.printf("WiFi:   %s | IP: %s | AP:%s\n",
                staConnected ? staSsid.c_str() : "(not connected)",
                WiFi.localIP().toString().c_str(),
                apMode ? "ON" : "OFF");
  Serial.printf("Link:   DS:%s  X:%s  DF:%s(%s)\n",
                dsOk ? "OK" : "BAD",
                xOk  ? "OK" : "BAD",
                dfOk ? "OK" : "BAD",
                dfValid ? "valid" : "n/a");
  Serial.printf("PSI:    %0.2f\n", outputPsi);
  Serial.printf("Pump:   %d\n", pumpCount);
  Serial.printf("Inputs: PressClosed=%d\n", pressClosed);
  Serial.printf("Cycle:  On=%d  Over=%d\n", cycleOn, cycleOver);
  Serial.printf("Err:    DS=0x%02X X=0x%02X DF=0x%02X\n", errDS, errX, errDF);
}

void printMenu() {
  Serial.println("──────────── MENU ────────────");
  Serial.printf("DS1 Start Feed PSI:     %d\n", ds1StartPsi);
  Serial.printf("DS3 Feed PSI Increase:  %d\n", ds3Inc);
  Serial.printf("DS4 Stroke Timer:       %d\n", ds4Stroke);
  Serial.printf("DS2 Final PSI:          %d\n", ds2FinalPsi);
  Serial.printf("DS5 Final Timer:        %d\n", ds5FinalT);
}

// =======================
//  SETUP / LOOP
// =======================
void setup() {
  Serial.begin(115200);
  delay(250);

  Serial.println();
  Serial.println("=== PACPRESS AP3 BRIDGE (STABLE + DS WRITES + C1/C2 CMD + HOME/MENU) ===");

  prefs.begin("wifi", true);
  staSsid = prefs.getString("ssid", "");
  staPass = prefs.getString("pass", "");
  prefs.end();

  if (staSsid.isEmpty()) {
    staSsid = DEFAULT_WIFI_SSID;
    staPass = DEFAULT_WIFI_PASS;
  }

  connectSTA(staSsid, staPass);
  startFallbackAP(); // keep AP always during dev

  ws.begin();
  ws.onEvent(onWsEvent);
  Serial.printf("WebSocket listening on %u\n", WS_PORT);

  PLCSerial.begin(PLC_BAUD, SERIAL_8O1, PLC_RX_PIN, PLC_TX_PIN);
  PLCSerial.setTimeout(220);
  node.begin(PLC_SLAVE_ID, PLCSerial);

  Serial.printf("DF3_REG_START offset=%u\n", DF3_REG_START);
  Serial.println("Polling round-robin: DS -> X -> DF3, with queued writes");
}

void loop() {
  ws.loop();

  static uint32_t lastTick = 0;
  static uint8_t phase = 0;
  uint32_t now = millis();
  if (now - lastTick < MODBUS_TICK_MS) return;
  lastTick = now;

  if (modbusBusy) return;
  modbusBusy = true;

  // ---- Execute queued writes first
  if (pendingDsWrite) {
    uint8_t e = 0;
    bool ok = doQueuedDsWrite(e);
    pendingDsWrite = false;
    ackJson = makeAck("ds", ok, ok ? 0 : e, ok ? "ok" : "err");
    pendingAck = true;
  }

  if (pendingStart || pendingStop) {
    uint8_t e = 0;
    bool ok = doQueuedStartStop(e);
    pendingStart = false;
    pendingStop = false;
    ackJson = makeAck("cmd", ok, ok ? 0 : e, ok ? "ok" : "err");
    pendingAck = true;
  }

  // ---- One read per tick
  if (phase == 0) dsOk = readDSBlock();
  if (phase == 1) xOk  = readXBlock();
  if (phase == 2) dfOk = readDF3Float();
  phase = (phase + 1) % 3;

  updateCycleInference();

  modbusBusy = false;
  delay(MODBUS_GAP_MS);

  // ---- Broadcast at ~10Hz
  static uint32_t lastWs = 0;
  if (now - lastWs >= 100) {
    lastWs = now;
    String out = buildJson();
    ws.broadcastTXT(out);

    if (pendingAck) {
      pendingAck = false;
      String a = ackJson;
      ws.broadcastTXT(a);
    }
  }

  // ---- Print at ~2Hz
  static uint32_t lastPrint = 0;
  if (now - lastPrint >= 500) {
    lastPrint = now;
    Serial.println();
    printHome();
    printMenu();
    Serial.println("──────────────────────────────");
  }
}
