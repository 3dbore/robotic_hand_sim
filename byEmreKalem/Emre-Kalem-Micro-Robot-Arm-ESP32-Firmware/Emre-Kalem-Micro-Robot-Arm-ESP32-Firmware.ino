/*
  Emre Kalem Robot Studio - ESP32-C6 + PCA9685 firmware 21.10.2026 (V1.0)
  Connection: Wi-Fi WebSocket (port 81) OR USB serial port (115200).

  Web -> ESP32 packet : {"j1":0,...,"servo":[[channel,angle],...]}   | command: {"cmd":"status"}
  Plain text commands : wifi, scan, status, ap, sta, estop, resume, cfg, help
  Emergency stop      : {"cmd":"estop"} releases all servos (no pulse) and ignores servo packets until {"cmd":"resume"}
  Settings (flash)    : {"cmd":"getcfg"} -> {"type":"config","ssid":"..","ap_ssid":"..","min_us":500,"max_us":2500}
                        {"cmd":"setcfg","ssid":"..","pass":"..","ap_ssid":"..","ap_pass":"..","min_us":500,"max_us":2500}
                        (any subset of fields; reply: {"type":"cfg_result","ok":true,"msg":".."}; passwords are never sent back)
  ESP32 -> Web status : {"type":"status","mode":"STA|AP","wifi":true,"ssid":"..","ip":"..","rssi":-55,"rx":12,"clients":1,"estop":false}

  E-Stop (software), Wi-Fi / AP / servo pulse range editable from the web UI and kept in flash (Preferences),
      larger serial RX buffer (the web UI now streams without throttling).
      NOTE: the E-Stop is a software feature, not a safety circuit - cut the servo supply for real emergencies.
  
  Wi-Fi: first STA (12 sec) -> if it cannot connect, only its own network: SSID "EK Micro Robot Arm", password "EK102026", IP 192.168.4.1.

  Arduino IDE (ESP32-C6): Board "ESP32C6 Dev Module". If using the device's USB-C "USB" (native) port,
  select Tools > "USB CDC On Boot: Enabled"; not needed if using the "UART" port.
  Libraries: Adafruit PWM Servo Driver, WebSockets (Markus Sattler), ArduinoJson 6.x
  Provide servo power from PCA9685 V+ terminal with EXTERNAL 5-6V supply, connect GNDs together.
*/
#include <WiFi.h>
#include <ESPmDNS.h>
#include <Wire.h>
#include <esp_system.h>
#include <Preferences.h>
#include <Adafruit_PWMServoDriver.h>
#include <WebSocketsServer.h>
#include <ArduinoJson.h>

const char* WIFI_SSID = "WIFI-SSID"; 
const char* WIFI_PASS = "WIFI-PASSWORD";
const char* AP_SSID = "EK Micro Robot Arm";
const char* AP_PASS = "EK102026";
#if defined(CONFIG_IDF_TARGET_ESP32C6)
const int SDA_PIN = 6, SCL_PIN = 7;      // ESP32-C6 default I2C pins - change according to your wiring
#else
const int SDA_PIN = 21, SCL_PIN = 22;
#endif
const int SERVO_MIN_US = 500, SERVO_MAX_US = 2500;   // adjust according to your servo

Adafruit_PWMServoDriver pwm(0x40);
WebSocketsServer ws(81);
String line;
bool mdnsStarted = false, pcaOk = false, scanning = false;
uint8_t phase = 0;                       // 0: Trying STA, 1: STA mode, 2: AP mode
uint32_t phaseStart = 0, lastByteMs = 0, lastStatus = 0, rxCount = 0;
Preferences prefs;
String cfgSsid, cfgPass, cfgApSsid, cfgApPass;   // loaded from flash in setup(); the constants above are only first-boot defaults
int cfgMinUs = SERVO_MIN_US, cfgMaxUs = SERVO_MAX_US;
bool estop = false;                      // true: servos released, servo packets ignored

bool staUp() { return phase != 2 && WiFi.status() == WL_CONNECTED; }

void startSta() {
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.begin(cfgSsid.c_str(), cfgPass.length() ? cfgPass.c_str() : nullptr);
  phase = 0; phaseStart = millis(); mdnsStarted = false;
}

void startAp() {
  WiFi.mode(WIFI_AP);
  WiFi.softAP(cfgApSsid.c_str(), cfgApPass.c_str());
  phase = 2; mdnsStarted = false;
  ws.begin();
}

void setServo(int ch, float deg) {
  if (!pcaOk || estop || ch < 0 || ch > 15) return;
  deg = constrain(deg, 0.0f, 180.0f);
  pwm.writeMicroseconds(ch, (uint16_t)(cfgMinUs + (cfgMaxUs - cfgMinUs) * deg / 180.0f));
}

void reply(const String& s) {
  Serial.println(s);
  String copy = s;            // broadcastTXT(String&) does not accept const
  ws.broadcastTXT(copy);
}

const char* wlText(wl_status_t st) {
  switch (st) {
    case WL_CONNECTED: return "Connected";
    case WL_NO_SSID_AVAIL: return "SSID not found (is the name correct? is the network 2.4 GHz?)";
    case WL_CONNECT_FAILED: return "Connection failed (password might be wrong)";
    case WL_CONNECTION_LOST: return "Connection lost";
    case WL_DISCONNECTED: return "Disconnected / trying";
    case WL_IDLE_STATUS: return "Idle / trying";
    default: return "Unknown";
  }
}

const char* resetText(esp_reset_reason_t r) {
  switch (r) {
    case ESP_RST_POWERON: return "power on";
    case ESP_RST_SW: return "software reset";
    case ESP_RST_PANIC: return "PANIC (crash)";
    case ESP_RST_INT_WDT: return "interrupt watchdog";
    case ESP_RST_TASK_WDT: return "task watchdog";
    case ESP_RST_WDT: return "watchdog";
    case ESP_RST_BROWNOUT: return "BROWNOUT (power drop)";
    default: return "other";
  }
}

void sendStatus() {
  StaticJsonDocument<384> d;
  bool ap = phase == 2, sta = staUp();
  d["type"] = "status";
  d["mode"] = ap ? "AP" : "STA";
  d["wifi"] = ap || sta;
  d["ssid"] = ap ? cfgApSsid : (sta ? WiFi.SSID() : cfgSsid);
  d["ip"] = ap ? WiFi.softAPIP().toString() : (sta ? WiFi.localIP().toString() : String(""));
  d["rssi"] = sta ? WiFi.RSSI() : 0;
  d["rx"] = rxCount;
  d["clients"] = ws.connectedClients();
  d["estop"] = estop;
  String out;
  serializeJson(d, out);
  Serial.println(out);
  ws.broadcastTXT(out);
}

void printWifiInfo() {
  if (phase == 2) {
    reply("Mode: AP (own network)");
    reply("Network name: " + cfgApSsid + "  password: " + cfgApPass);
    reply("IP: " + WiFi.softAPIP().toString());
    reply(String("Connected device count: ") + WiFi.softAPgetStationNum());
  } else {
    reply(String("Mode: STA  status: ") + wlText(WiFi.status()) + "  target SSID: " + cfgSsid);
    if (staUp()) {
      reply("IP: " + WiFi.localIP().toString());
      reply("Gateway: " + WiFi.gatewayIP().toString());
      reply(String("Signal: ") + WiFi.RSSI() + " dBm  Channel: " + WiFi.channel());
      reply(String("mDNS: ") + (mdnsStarted ? "robotkol.local" : "none"));
    }
  }
  reply("MAC: " + WiFi.macAddress());
  reply("WebSocket: port 81, client count " + String(ws.connectedClients()));
  reply(String("PCA9685: ") + (pcaOk ? "found" : "NOT FOUND") + "  Received robot packets: " + rxCount);
}

void scanWifi() {
  reply("Network scan started (takes a few sec)...");
  WiFi.scanDelete();
  WiFi.scanNetworks(true);               // asynchronous: does not block loop()
  scanning = true;
}

void pollScan() {
  if (!scanning) return;
  int n = WiFi.scanComplete();
  if (n == WIFI_SCAN_RUNNING) return;
  scanning = false;
  if (n < 0) { reply("Scan failed"); return; }
  if (n == 0) reply("No network found");
  for (int i = 0; i < n; i++) {
    reply(String(i + 1) + ") " + WiFi.SSID(i) + "  " + WiFi.RSSI(i) + " dBm  channel " + WiFi.channel(i) +
          (WiFi.SSID(i) == cfgSsid ? "  <-- target" : ""));
  }
  WiFi.scanDelete();
}

// ---------- settings in flash ----------
void loadCfg() {
  prefs.begin("robotkol", false);
  cfgSsid = prefs.getString("ssid", WIFI_SSID);
  cfgPass = prefs.getString("pass", WIFI_PASS);
  cfgApSsid = prefs.getString("ap_ssid", AP_SSID);
  cfgApPass = prefs.getString("ap_pass", AP_PASS);
  cfgMinUs = prefs.getInt("min_us", SERVO_MIN_US);
  cfgMaxUs = prefs.getInt("max_us", SERVO_MAX_US);
  if (cfgMinUs < 400 || cfgMinUs > 1500 || cfgMaxUs < 1500 || cfgMaxUs > 3000 || cfgMinUs >= cfgMaxUs) {   // damaged value: back to defaults
    cfgMinUs = SERVO_MIN_US; cfgMaxUs = SERVO_MAX_US;
  }
}

void sendCfg() {
  StaticJsonDocument<384> d;
  d["type"] = "config";
  d["ssid"] = cfgSsid;
  d["ap_ssid"] = cfgApSsid;
  d["min_us"] = cfgMinUs;
  d["max_us"] = cfgMaxUs;
  String out;
  serializeJson(d, out);
  reply(out);
}

void cfgResult(bool ok, const String& msg) {
  StaticJsonDocument<384> d;
  d["type"] = "cfg_result";
  d["ok"] = ok;
  d["msg"] = msg;
  String out;
  serializeJson(d, out);
  reply(out);
}

// Any subset of: ssid, pass, ap_ssid, ap_pass, min_us, max_us. Nothing is saved unless every given field is valid.
void handleSetCfg(JsonDocument& doc) {
  String nSsid = cfgSsid, nPass = cfgPass, nApSsid = cfgApSsid, nApPass = cfgApPass;
  int nMin = cfgMinUs, nMax = cfgMaxUs;

  if (doc.containsKey("ssid")) {
    nSsid = doc["ssid"].as<String>(); nSsid.trim();
    if (nSsid.length() < 1 || nSsid.length() > 32) { cfgResult(false, "Wi-Fi name must be 1-32 characters"); return; }
  }
  if (doc.containsKey("pass")) {
    nPass = doc["pass"].as<String>();
    if (nPass.length() != 0 && (nPass.length() < 8 || nPass.length() > 63)) { cfgResult(false, "Wi-Fi password must be 8-63 characters (or empty for an open network)"); return; }
  }
  if (doc.containsKey("ap_ssid")) {
    nApSsid = doc["ap_ssid"].as<String>(); nApSsid.trim();
    if (nApSsid.length() < 1 || nApSsid.length() > 32) { cfgResult(false, "AP name must be 1-32 characters"); return; }
  }
  if (doc.containsKey("ap_pass")) {
    nApPass = doc["ap_pass"].as<String>();
    if (nApPass.length() < 8 || nApPass.length() > 63) { cfgResult(false, "AP password must be 8-63 characters"); return; }
  }
  if (doc.containsKey("min_us")) nMin = doc["min_us"].as<int>();
  if (doc.containsKey("max_us")) nMax = doc["max_us"].as<int>();
  if (nMin < 400 || nMin > 1500 || nMax < 1500 || nMax > 3000 || nMin >= nMax) {
    cfgResult(false, "Servo pulse range: min 400-1500 us, max 1500-3000 us");
    return;
  }

  bool staChanged = (nSsid != cfgSsid) || (nPass != cfgPass);
  bool apChanged = (nApSsid != cfgApSsid) || (nApPass != cfgApPass);
  cfgSsid = nSsid; cfgPass = nPass; cfgApSsid = nApSsid; cfgApPass = nApPass; cfgMinUs = nMin; cfgMaxUs = nMax;
  prefs.putString("ssid", cfgSsid);
  prefs.putString("pass", cfgPass);
  prefs.putString("ap_ssid", cfgApSsid);
  prefs.putString("ap_pass", cfgApPass);
  prefs.putInt("min_us", cfgMinUs);
  prefs.putInt("max_us", cfgMaxUs);

  String msg = "Saved.";
  bool reconnect = staChanged && phase != 2;
  if (reconnect) msg += " Reconnecting to the new Wi-Fi (the IP may change).";
  else if (staChanged) msg += " Send 'sta' to try the new home network.";
  if (apChanged) msg += " New AP settings apply the next time AP mode starts ('ap').";
  cfgResult(true, msg);
  sendCfg();
  if (reconnect) { delay(200); startSta(); }
}

// ---------- emergency stop ----------
void servosOff() {
  if (!pcaOk) return;
  for (uint8_t ch = 0; ch < 16; ch++) pwm.setPWM(ch, 0, 4096);   // full OFF: no pulse, servo is released
}

void setEstop(bool on) {
  bool changed = (on != estop);
  estop = on;
  if (on) servosOff();                                           // repeated commands re-assert the stop
  if (changed) {
    reply(on ? "E-STOP: servos released, servo packets ignored" : "E-STOP released");
    sendStatus();
  }
}

void handleJson(const char* txt, size_t len) {
  StaticJsonDocument<768> doc;
  if (deserializeJson(doc, txt, len)) return;
  const char* cmd = doc["cmd"] | "";
  if (cmd[0]) {
    if (!strcmp(cmd, "status")) sendStatus();
    else if (!strcmp(cmd, "estop")) setEstop(true);
    else if (!strcmp(cmd, "resume")) setEstop(false);
    else if (!strcmp(cmd, "getcfg")) sendCfg();
    else if (!strcmp(cmd, "setcfg")) handleSetCfg(doc);
    return;
  }
  JsonArray arr = doc["servo"].as<JsonArray>();
  if (arr.isNull()) return;
  rxCount++;
  if (estop) return;                                             // E-Stop: servo packets are dropped
  for (JsonArray p : arr) setServo(p[0], p[1]);
}

// Incoming line: if it starts with '{' it is JSON, otherwise plain text command
void handleLine(const char* txt, size_t len) {
  String s;
  s.reserve(len);
  for (size_t i = 0; i < len; i++) s += txt[i];
  s.trim();
  if (s.length() == 0) return;
  if (s[0] == '{') { handleJson(s.c_str(), s.length()); return; }
  s.toLowerCase();
  if (s == "wifi") printWifiInfo();
  else if (s == "scan") scanWifi();
  else if (s == "status") sendStatus();
  else if (s == "estop") setEstop(true);
  else if (s == "resume") setEstop(false);
  else if (s == "cfg") sendCfg();
  else if (s == "ap") { reply("Starting own network: " + cfgApSsid + "  IP 192.168.4.1"); delay(200); startAp(); }
  else if (s == "sta") { reply("Trying to connect to home network: " + cfgSsid); delay(200); startSta(); }
  else if (s == "help") reply("Commands: wifi, scan, status, ap, sta, estop, resume, cfg, help");
  else reply("Unknown command: " + s + "  (type help)");
}

void onWsEvent(uint8_t, WStype_t type, uint8_t* payload, size_t len) {
  if (type == WStype_TEXT) handleLine((const char*)payload, len);
}

void setup() {
  Serial.setRxBufferSize(1024);           // must be called before begin(); the web UI streams many packets per second
  Serial.begin(115200);
#if defined(ARDUINO_USB_CDC_ON_BOOT) && ARDUINO_USB_CDC_ON_BOOT
  Serial.setTxTimeoutMs(0);               // Do not block writing when USB is not connected
#endif
  delay(300);
  Serial.println(String("[boot] chip: ") + ESP.getChipModel() + "  last reset: " + resetText(esp_reset_reason()));

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setTimeOut(100);
  Wire.beginTransmission(0x40);
  uint8_t err = Wire.endTransmission();
  pcaOk = (err == 0);
  if (pcaOk) {
    pwm.begin();
    pwm.setOscillatorFrequency(27000000);
    pwm.setPWMFreq(50);
    Serial.println("[boot] PCA9685 found");
  } else {
    Serial.println(String("[boot] PCA9685 NOT FOUND (I2C error ") + err + ") SDA=" + SDA_PIN + " SCL=" + SCL_PIN + " - check wiring");
  }

  loadCfg();
  Serial.println(String("[boot] settings loaded (Wi-Fi: ") + cfgSsid + ")");
  WiFi.persistent(false);
  startSta();
  Serial.println("[boot] Wi-Fi started (trying STA)");
  ws.begin();
  ws.onEvent(onWsEvent);
  Serial.println("[boot] WebSocket server ready (port 81)");
  sendStatus();
}

void loop() {
  ws.loop();

  // Serial port: end of line ('\n' or '\r') or 200 ms of silence ends the line
  while (Serial.available()) {
    char c = Serial.read();
    lastByteMs = millis();
    if (c == '\n' || c == '\r') {
      if (line.length()) { handleLine(line.c_str(), line.length()); line = ""; }
    } else if (line.length() < 700) line += c;
  }
  if (line.length() && millis() - lastByteMs > 200) { handleLine(line.c_str(), line.length()); line = ""; }

  pollScan();
  if (phase == 0) {
    if (WiFi.status() == WL_CONNECTED) { phase = 1; sendStatus(); }
    else if (millis() - phaseStart > 12000) { startAp(); sendStatus(); }
  }
  if (phase == 1 && !mdnsStarted && WiFi.status() == WL_CONNECTED) {
    mdnsStarted = MDNS.begin("robotkol");
    if (mdnsStarted) MDNS.addService("ws", "tcp", 81);
  }
  if (millis() - lastStatus > 2000) { lastStatus = millis(); sendStatus(); }
  delay(1);
}