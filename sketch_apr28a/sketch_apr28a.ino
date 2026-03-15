/*
 * ESP32 WROVER - 4 Pots + Sensors → WiFi Setup + MQTT (HiveMQ)
 * Sensors: 4× soil moisture, 1× light, 1× water level, 1× DHT (temp/humidity), 1× relay
 * Camera OFF at startup. Each publish cycle (e.g. every 1 min):
 *   DISCONNECT WIFI → READ SENSORS → CAPTURE IMAGE (offline) → CONNECT WIFI → PUBLISH TO MQTT
 * Data: plot id + pots[] + optional image (base64) on topic plot/<id>/data.
 *
 * Required libraries (Arduino Library Manager):
 *   - WiFi, WebServer, Preferences (built-in ESP32)
 *   - PubSubClient (by Nick O'Leary)
 *   - ArduinoJson (by Benoit Blanchon)
 *   - DHTesp (by beegee-tokyo) for DHT11 temp/humidity
 *   - Keyes KY-018 analog LDR (no library; single analog pin)
 *   - esp32-camera (built-in with ESP32 board package) for image capture
 */

// Set to 1 to enable camera in the sequence above; 0 = sensor-only.
#define ENABLE_CAMERA  1
// Camera: OV3660 wired to ESP32-WROVER (see pin table in camera section below)

#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <DHTesp.h>
#if ENABLE_CAMERA
#include "esp_camera.h"
#include "libb64/cencode.h"
#include "esp_heap_caps.h"
#endif

// ============== PIN CONFIG (ESP32 WROVER) ==============
#define SOIL_PIN_1          32   
#define SOIL_PIN_2          33   
#define SOIL_PIN_3          14   // 4,0,2,22,1,3 
#define SOIL_PIN_4          12 
#define WATER_LEVEL_PIN     0   
#define RELAY_PIN           2  
#define DHT_PIN             13 // okay
// Keyes KY-018 LDR: connect AO to this pin; VCC 3.3V, GND. (GPIO 4 = ADC2; 34/35/36/39 = ADC1 but used by camera.)
#define LIGHT_SENSOR_PIN    15 


// OV3660 on ESP32-WROVER — custom wiring, no conflicts with sensor pins.
// Avoids sensor GPIOs (4,5,21,22,32,33,34,35,39) and reserved GPIOs (6-11=flash, 16-17=PSRAM).
//
// ┌────────────┬────────┬──────────────────────────────────┐
// │ Camera pin │ GPIO   │ Notes                            │
// ├────────────┼────────┼──────────────────────────────────┤
// │ SIOD (SDA) │  26    │ SCCB data  (bidirectional)       │
// │ SIOC (SCL) │  27    │ SCCB clock (output)              │
// │ XCLK       │  21    │ Master clock to camera (output)  │
// │ PCLK       │  22    │ Pixel clock from camera (input)  │
// │ VSYNC      │  25    │ Vertical sync (input)            │
// │ HREF       │  23    │ Horizontal ref (input)           │
// │ D7  (Y9)   │  35    │ Data bit 7 (pull-up at boot OK)  │
// │ D6  (Y8)   │  34    │ Data bit 6 (RX0, input OK)       │
// │ D5  (Y7)   │  39    │ Data bit 5 (input-only OK)       │
// │ D4  (Y6)   │  36    │ Data bit 4                       │
// │ D3  (Y5)   │  19    │ Data bit 3                       │
// │ D2  (Y4)   │  18    │ Data bit 2                       │
// │ D1  (Y3)   │   5    │ Data bit 1 (LOW at boot = OK)    │
// │ D0  (Y2)   │   4    │ Data bit 0 (keep LOW at boot)    │
// │ PWDN       │  -1    │ Tie to GND or leave unconnected  │
// │ RESET      │  -1    │ Tie to 3.3V or leave unconnected │
// └────────────┴────────┴──────────────────────────────────┘
//

// ============== AP / WIFI / MQTT ==============
#define AP_SSID             "ESP32-Plot-Setup"
#define AP_PASS             "12345678"
#define SETUP_PORT          80
#define MQTT_PUBLISH_INTERVAL_MS   (60 * 1000)   // 1 minute: disconnect -> sensors -> capture -> connect -> publish
#define WIFI_DISCONNECT_DELAY_MS   5000           // delay after disconnect before reading sensors/camera

#define MQTT_IMAGE_MAX_SIZE        (20 * 1024)   // 20KB JPEG
#define MQTT_DATA_MAX_SIZE         (40 * 1024)   // JSON payload
#define LAST_IMAGE_B64_MAX         (28 * 1024)   // base64 cache

#define PREF_NAMESPACE       "plotcfg"
#define PREF_WIFI_SSID       "wifi_ssid"
#define PREF_WIFI_PASS       "wifi_pass"
#define PREF_MQTT_BROKER     "mqtt_br"
#define PREF_MQTT_PORT       "mqtt_port"
#define PREF_MQTT_USER       "mqtt_usr"
#define PREF_MQTT_PASS       "mqtt_pwd"
#define PREF_PLOT_ID         "plot_id"

#define MQTT_TOPIC_PREFIX    "plot/"
#define DEFAULT_MQTT_PORT    1883

// Hardcoded HiveMQ Cloud credentials (not shown on setup page)
#define HIVEMQ_BROKER        "broker.hivemq.com"
#define HIVEMQ_PORT          1883
#define HIVEMQ_USER          ""
#define HIVEMQ_PASS          ""

// ============== GLOBALS ==============
WebServer server(SETUP_PORT);
WiFiClient wifiClient;
PubSubClient mqtt(wifiClient);
Preferences prefs;
DHTesp dht;
char prefWifiSsid[64] = "";
char prefWifiPass[64] = "";
char prefMqttBroker[128] = "";
uint16_t prefMqttPort = DEFAULT_MQTT_PORT;
char prefMqttUser[64] = "";
char prefMqttPass[64] = "";
char prefPlotId[64] = "";   // Plot ID - set once, then read-only (cannot be edited)

bool wifiConfigured = false;
bool wifiConnected = false;
bool relayOn = false;
unsigned long lastPublishMs = 0;
#if ENABLE_CAMERA
bool cameraOk = false;
char cameraError[80] = "";  // last init error message
static char *lastImageB64 = nullptr;  // allocated from PSRAM to save DRAM
static size_t lastImageB64Len = 0;
// Stabilization: first frames can be green/corrupt; delay after init and discard dummy frames
#define CAMERA_STABILIZE_MS    800   // delay after init before first capture
#define CAMERA_DUMMY_FRAMES    2     // discard this many frames before keeping one
#define CAMERA_FRAME_DELAY_MS  150   // delay between frames when discarding
#define CAMERA_MIN_FRAME_SIZE  2000  // reject frame if smaller (likely bad/green)
#endif

// ============== SENSOR HELPERS ==============
// ADC for ESP32: 12-bit 0-4095; scale to 0-100 for moisture/waterLevel
int readAnalogPercent(int pin) {
  int raw = analogRead(pin);
  return map(constrain(raw, 0, 4095), 0, 4095, 0, 100);
}

void setupPins() {
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);
  pinMode(SOIL_PIN_1, INPUT);
  pinMode(SOIL_PIN_2, INPUT);
  pinMode(SOIL_PIN_3, INPUT);
  pinMode(SOIL_PIN_4, INPUT);
  pinMode(WATER_LEVEL_PIN, INPUT);
  pinMode(LIGHT_SENSOR_PIN , INPUT);
  dht.setup(DHT_PIN, DHTesp::DHT11);
  Serial.println("KY-018 light sensor on pin " + String(LIGHT_SENSOR_PIN));
}

float readTemperature() {
  return dht.getTemperature();
}

float readHumidity() {
  return dht.getHumidity();
}

// ============== PREFERENCES ==============
void loadPreferences() {
  prefs.begin(PREF_NAMESPACE, true);
  prefs.getString(PREF_WIFI_SSID, "").toCharArray(prefWifiSsid, sizeof(prefWifiSsid));
  prefs.getString(PREF_WIFI_PASS, "").toCharArray(prefWifiPass, sizeof(prefWifiPass));
  prefs.getString(PREF_PLOT_ID, "").toCharArray(prefPlotId, sizeof(prefPlotId));
  prefs.end();

  strncpy(prefMqttBroker, HIVEMQ_BROKER, sizeof(prefMqttBroker));
  prefMqttPort = HIVEMQ_PORT;
  strncpy(prefMqttUser, HIVEMQ_USER, sizeof(prefMqttUser));
  strncpy(prefMqttPass, HIVEMQ_PASS, sizeof(prefMqttPass));

  wifiConfigured = (strlen(prefWifiSsid) > 0);
}

void saveWifi(const char* ssid, const char* pass) {
  prefs.begin(PREF_NAMESPACE, false);
  prefs.putString(PREF_WIFI_SSID, ssid);
  prefs.putString(PREF_WIFI_PASS, pass);
  prefs.end();
}

void savePlotId(const char* plotId) {
  prefs.begin(PREF_NAMESPACE, false);
  prefs.putString(PREF_PLOT_ID, plotId);
  prefs.end();
}

void resetPreferences() {
  prefs.begin(PREF_NAMESPACE, false);
  prefs.clear();
  prefs.end();
  strcpy(prefPlotId, "");
  strcpy(prefWifiSsid, "");
  strcpy(prefWifiPass, "");
  strcpy(prefMqttBroker, "");
  prefMqttPort = DEFAULT_MQTT_PORT;
  strcpy(prefMqttUser, "");
  strcpy(prefMqttPass, "");
  wifiConfigured = false;
}

// ============== WIFI SCAN (JSON API) ==============
void handleScan() {
  int n = WiFi.scanNetworks();
  StaticJsonDocument<2048> doc;
  JsonArray arr = doc.to<JsonArray>();
  for (int i = 0; i < n; i++) {
    JsonObject obj = arr.add<JsonObject>();
    obj["ssid"] = WiFi.SSID(i);
    obj["rssi"] = WiFi.RSSI(i);
    obj["secure"] = (WiFi.encryptionType(i) != WIFI_AUTH_OPEN);
  }
  String out;
  serializeJson(doc, out);
  server.send(200, "application/json", out);
}

// ============== HTML SETUP PAGE ==============
void sendSetupPage(bool plotIdLocked) {
  String html = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>ESP32 Plot Setup</title>
  <style>
    body { font-family: sans-serif; max-width: 420px; margin: 20px auto; padding: 16px; background: #1a1a2e; color: #eee; }
    h1 { font-size: 1.3rem; color: #a8e6cf; }
    label { display: block; margin-top: 10px; }
    input, button, select { width: 100%; padding: 10px; margin-top: 4px; box-sizing: border-box; }
    input, select { background: #16213e; border: 1px solid #0f3460; color: #eee; border-radius: 6px; }
    button { background: #0f3460; color: #fff; border: none; border-radius: 6px; cursor: pointer; margin-top: 12px; }
    button.secondary { background: #533483; }
    button.scan { background: #0d7377; margin-top: 4px; }
    .note { font-size: 0.85rem; color: #888; margin-top: 4px; }
    .readonly { background: #0d2137; color: #888; }
    .row { display: flex; gap: 8px; align-items: center; }
    .row button { width: auto; flex: 0 0 auto; }
    .row select { flex: 1; }
    .relay-box { background: #16213e; border-radius: 8px; padding: 12px; margin: 16px 0; display: flex; align-items: center; gap: 12px; border: 1px solid #0f3460; }
    .relay-box span { font-size: 1rem; }
    .switch { position: relative; display: inline-block; width: 52px; height: 28px; }
    .switch input { opacity: 0; width: 0; height: 0; }
    .slider { position: absolute; cursor: pointer; top: 0; left: 0; right: 0; bottom: 0; background: #444; border-radius: 28px; transition: 0.3s; }
    .slider:before { position: absolute; content: ""; height: 22px; width: 22px; left: 3px; bottom: 3px; background: #fff; border-radius: 50%; transition: 0.3s; }
    input:checked + .slider { background: #0d7377; }
    input:checked + .slider:before { transform: translateX(24px); }
    .relay-status { font-weight: 600; color: #a8e6cf; min-width: 32px; }
  </style>
</head>
<body>
  <h1>Plot & WiFi Setup</h1>
  <form action="/save" method="POST" id="setupForm">
)rawliteral";

  // Plot ID: read-only if already set (cannot be edited)
  html += F("<label>Plot ID</label>");
  if (plotIdLocked) {
    html += "<input type=\"text\" class=\"readonly\" value=\"" + String(prefPlotId) + "\" readonly>";
    html += F("<p class=\"note\">Plot ID is fixed and cannot be edited. Reset config to change.</p>");
  } else {
    html += "<input type=\"text\" name=\"plot_id\" placeholder=\"e.g. plot123\" maxlength=\"60\" value=\"" + String(prefPlotId) + "\">";
    html += F("<p class=\"note\">Set once; after saving it cannot be changed.</p>");
  }

  html += R"rawliteral(
    <label>WiFi network</label>
    <div class="row">
      <select id="wifiSelect" title="Select a network after scanning">
        <option value="">— Scan to see networks —</option>
      </select>
      <button type="button" class="scan" id="scanBtn">Scan</button>
    </div>
    <label>WiFi SSID</label>
    <input type="text" name="ssid" id="ssidInput" placeholder="Select above or type SSID" maxlength="32" required>
    <p class="note">Tap Scan, then choose a network (SSID fills automatically) or type the name.</p>
    <label>WiFi Password</label>
    <input type="password" name="pass" placeholder="WiFi password" maxlength="64">
    <button type="submit">Save & Connect</button>
  </form>
  <form action="/reset" method="POST" onsubmit="return confirm('Clear all config (including Plot ID)?');">
    <button type="submit" class="secondary">Reset configuration</button>
  </form>
  <div class="relay-box">
    <span>Relay / Pump:</span>
    <label class="switch">
      <input type="checkbox" id="relayToggle">
      <span class="slider"></span>
    </label>
    <span class="relay-status" id="relayStatus">OFF</span>
  </div>
)rawliteral";
  html += F("<p class=\"note\"><a href=\"/dashboard\">Live dashboard</a> – current plot data, updates every 20s.</p>");
#if ENABLE_CAMERA
  html += F("<p class=\"note\"><a href=\"/cam\">Test camera</a> – take a snapshot to verify the camera works.</p>");
#endif
  html += R"rawliteral(
  <script>
    var scanBtn = document.getElementById('scanBtn');
    var wifiSelect = document.getElementById('wifiSelect');
    var ssidInput = document.getElementById('ssidInput');
    scanBtn.onclick = function() {
      scanBtn.textContent = 'Scanning...';
      scanBtn.disabled = true;
      fetch('/scan').then(function(r) { return r.json(); }).then(function(networks) {
        wifiSelect.innerHTML = '<option value="">— Select a network —</option>';
        for (var i = 0; i < networks.length; i++) {
          var n = networks[i];
          var opt = document.createElement('option');
          opt.value = n.ssid;
          var sig = n.rssi > -50 ? 'Strong' : (n.rssi > -70 ? 'Good' : 'Weak');
          opt.textContent = n.ssid + ' (' + sig + ', ' + (n.secure ? 'secured' : 'open') + ')';
          wifiSelect.appendChild(opt);
        }
        scanBtn.textContent = 'Scan';
        scanBtn.disabled = false;
      }).catch(function() {
        scanBtn.textContent = 'Scan';
        scanBtn.disabled = false;
        wifiSelect.innerHTML = '<option value="">Scan failed</option>';
      });
    };
    wifiSelect.onchange = function() {
      if (wifiSelect.value) ssidInput.value = wifiSelect.value;
    };
    var relayToggle = document.getElementById('relayToggle');
    var relayStatus = document.getElementById('relayStatus');
    if (relayToggle) {
      relayToggle.onchange = function() {
        var st = relayToggle.checked ? 'on' : 'off';
        fetch('/api/relay?state=' + st, { method: 'POST' })
          .then(function(r) { return r.json(); })
          .then(function(d) {
            relayStatus.textContent = d.relay === 'on' ? 'ON' : 'OFF';
          })
          .catch(function() { relayToggle.checked = !relayToggle.checked; });
      };
      fetch('/api/relay').then(function(r) { return r.json(); }).then(function(d) {
        relayToggle.checked = d.relay === 'on';
        relayStatus.textContent = d.relay === 'on' ? 'ON' : 'OFF';
      }).catch(function(){});
    }
  </script>
</body>
</html>
)rawliteral";

  server.send(200, "text/html", html);
}

void handleRoot() {
  sendSetupPage(strlen(prefPlotId) > 0);
}

void handleSave() {
  if (server.method() != HTTP_POST) {
    server.send(405, "text/plain", "Method Not Allowed");
    return;
  }
  String ssid = server.arg("ssid");
  String pass = server.arg("pass");
  String plotId = server.arg("plot_id");

  if (ssid.length() == 0) {
    server.send(400, "text/plain", "WiFi SSID is required");
    return;
  }

  saveWifi(ssid.c_str(), pass.c_str());
  if (plotId.length() > 0 && strlen(prefPlotId) == 0) {
    plotId.trim();
    if (plotId.length() > 0) {
      plotId.toCharArray(prefPlotId, sizeof(prefPlotId));
      savePlotId(prefPlotId);
    }
  }
  server.send(200, "text/html",
    "<!DOCTYPE html><html><body><h1>Saved.</h1><p>Connecting to WiFi and restarting...</p><script>setTimeout(function(){ location.href='/'; }, 3000);</script></body></html>");
  delay(500);
  ESP.restart();
}

void handleReset() {
  if (server.method() != HTTP_POST) {
    server.send(405, "text/plain", "Method Not Allowed");
    return;
  }
  resetPreferences();
  server.send(200, "text/html",
    "<!DOCTYPE html><html><body><h1>Config cleared.</h1><p>Restarting in AP mode...</p><script>setTimeout(function(){ location.href='/'; }, 2000);</script></body></html>");
  delay(500);
  ESP.restart();
}

void handleRelayToggle() {
  String state = server.arg("state");
  if (state == "on") {
    relayOn = true;
    digitalWrite(RELAY_PIN, HIGH);
  } else {
    relayOn = false;
    digitalWrite(RELAY_PIN, LOW);
  }
  server.send(200, "application/json", relayOn ? "{\"relay\":\"on\"}" : "{\"relay\":\"off\"}");
}

void handleRelayState() {
  server.send(200, "application/json", relayOn ? "{\"relay\":\"on\"}" : "{\"relay\":\"off\"}");
}

void setupWebServer() {
  server.on("/", handleRoot);
  server.on("/scan", handleScan);
  server.on("/save", handleSave);
  server.on("/reset", handleReset);
  server.on("/dashboard", handleDashboard);
  server.on("/api/plot", handleApiPlot);
  server.on("/api/relay", HTTP_GET, handleRelayState);
  server.on("/api/relay", HTTP_POST, handleRelayToggle);
#if ENABLE_CAMERA
  server.on("/cam", handleTestCam);
  server.on("/cam/retry", handleCamRetry);
  server.on("/snapshot", handleSnapshot);
#endif
  server.begin();
}

// ============== WIFI ==============
bool tryConnectWifi() {
  if (!wifiConfigured) return false;
  WiFi.mode(WIFI_STA);
  WiFi.begin(prefWifiSsid, prefWifiPass);
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 30) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi connected: " + WiFi.localIP().toString());
    return true;
  }
  Serial.println("\nWiFi failed");
  return false;
}

void startAP() {
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);
  Serial.println("AP started: " + String(AP_SSID) + " IP: " + WiFi.softAPIP().toString());
  setupWebServer();
  // Camera stays OFF; /cam and /snapshot will init on demand and deinit after
}

// ============== MQTT ==============
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String relayTopic = String(MQTT_TOPIC_PREFIX) + prefPlotId + "/relay";
  if (relayTopic != topic) return;

  char msg[16];
  size_t n = length < (sizeof(msg) - 1) ? length : (sizeof(msg) - 1);
  memcpy(msg, payload, n);
  msg[n] = '\0';
  String s = String(msg);
  s.trim();
  s.toLowerCase();

  if (s == "on") {
    relayOn = true;
    digitalWrite(RELAY_PIN, HIGH);
    Serial.println("Relay ON (MQTT)");
  } else if (s == "off") {
    relayOn = false;
    digitalWrite(RELAY_PIN, LOW);
    Serial.println("Relay OFF (MQTT)");
  }
}

void mqttReconnect() {
  if (mqtt.connected()) return;
  mqtt.setServer(prefMqttBroker, prefMqttPort);
  if (strlen(prefMqttUser) > 0)
    mqtt.connect("ESP32-Plot", prefMqttUser, prefMqttPass);
  else
    mqtt.connect("ESP32-Plot");
  if (mqtt.connected()) {
    Serial.println("MQTT connected");
    if (strlen(prefPlotId) > 0) {
      String relayTopic = String(MQTT_TOPIC_PREFIX) + prefPlotId + "/relay";
      if (mqtt.subscribe(relayTopic.c_str()))
        Serial.println("Subscribed to " + relayTopic);
    }
  } else
    Serial.println("MQTT failed: " + String(mqtt.state()));
}

// Build current plot JSON into buf; returns length. Shared by MQTT and API.
size_t buildPlotJson(char* buf, size_t bufSize) {
  float temp = readTemperature();
  float hum = readHumidity();
  if (isnan(temp)) temp = 0.0f;
  if (isnan(hum)) hum = 0.0f;
  int waterLevel = readAnalogPercent(WATER_LEVEL_PIN);
  // KY-018 on ADC1: high ADC = dark, low ADC = bright → 0–100% (100 = bright). If yours is reversed, use (0, 4095, 0, 100).
  int lightRaw = analogRead(LIGHT_SENSOR_PIN);
  lightRaw = (analogRead(LIGHT_SENSOR_PIN) + lightRaw) / 2;  // quick average
  int lightIntensity = map(constrain(lightRaw, 0, 4095), 0, 4095, 100, 0);
  int m1 = readAnalogPercent(SOIL_PIN_1);
  int m2 = readAnalogPercent(SOIL_PIN_2);
  int m3 = readAnalogPercent(SOIL_PIN_3);
  int m4 = readAnalogPercent(SOIL_PIN_4);

  StaticJsonDocument<800> doc;
  doc["id"] = prefPlotId;
  doc["ip"] = WiFi.localIP().toString();
  doc["relay"] = relayOn ? "on" : "off";
  JsonArray pots = doc.createNestedArray("pots");
  const char* names[] = { "POT1", "POT2", "POT3", "POT4" };
  int moistures[] = { m1, m2, m3, m4 };
  // int moistures[] = { m3, m4 };
  for (int i = 0; i < 4; i++) {
    JsonObject p = pots.add<JsonObject>();
    p["name"] = names[i];
    p["moisture"] = moistures[i];
    p["temperature"] = round(temp * 10) / 10.0;
    p["humidity"] = round(hum * 10) / 10.0;
    p["waterLevel"] = waterLevel;
    p["lightIntensity"] = lightIntensity;
  }
  return serializeJson(doc, buf, bufSize);
}

#if ENABLE_CAMERA
// Camera-only sequence (no WiFi/MQTT): INIT -> CAPTURE -> SAVE AS LAST PICTURE -> TURN OFF.
// Call while WiFi is disconnected. Saves to lastImageB64 for later publish.
static void runCameraSequenceOnly() {
  cameraOk = initCamera();
  if (!cameraOk) {
    Serial.println("Camera init failed (offline)");
    return;
  }
  delay(CAMERA_STABILIZE_MS);
  for (int i = 0; i < CAMERA_DUMMY_FRAMES; i++) {
    camera_fb_t *dummy = esp_camera_fb_get();
    if (dummy) {
      esp_camera_fb_return(dummy);
      delay(CAMERA_FRAME_DELAY_MS);
    }
  }
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb || fb->len == 0 || fb->len < CAMERA_MIN_FRAME_SIZE || fb->len > 32000) {
    if (fb) esp_camera_fb_return(fb);
    esp_camera_deinit();
    cameraOk = false;
    return;
  }
  size_t b64Len = base64_encode_expected_len(fb->len) + 1;
  if (b64Len <= LAST_IMAGE_B64_MAX) {
    if (!lastImageB64 && !(lastImageB64 = (char *)heap_caps_malloc(LAST_IMAGE_B64_MAX, MALLOC_CAP_SPIRAM)))
      lastImageB64 = (char *)heap_caps_malloc(LAST_IMAGE_B64_MAX, MALLOC_CAP_INTERNAL);
    if (lastImageB64) {
      int outLen = base64_encode_chars((const char *)fb->buf, fb->len, lastImageB64);
      lastImageB64[outLen] = '\0';
      lastImageB64Len = (size_t)outLen;
    }
  }
  esp_camera_fb_return(fb);
  esp_camera_deinit();
  cameraOk = false;
  Serial.println("Camera off (offline capture done)");
}

// Publish buf (sensor JSON) to MQTT, with lastImageB64 if available. Call when WiFi/MQTT connected.
static void doPublishToMqtt(const char* buf, size_t len) {
  String topic = String(MQTT_TOPIC_PREFIX) + prefPlotId + "/data";
  if (lastImageB64 && lastImageB64Len > 0) {
    DynamicJsonDocument doc(MQTT_DATA_MAX_SIZE);
    if (deserializeJson(doc, buf) == DeserializationError::Ok) {
      doc["image"] = lastImageB64;
      char *pubBuf = (char *)heap_caps_malloc(MQTT_DATA_MAX_SIZE, MALLOC_CAP_SPIRAM);
      if (!pubBuf) pubBuf = (char *)heap_caps_malloc(MQTT_DATA_MAX_SIZE, MALLOC_CAP_INTERNAL);
      if (pubBuf) {
        size_t pubLen = serializeJson(doc, pubBuf, MQTT_DATA_MAX_SIZE);
        if (mqtt.publish(topic.c_str(), pubBuf, pubLen))
          Serial.println("Published to " + topic + " (with image)");
        else
          Serial.println("Publish failed");
        heap_caps_free(pubBuf);
      } else {
        mqtt.publish(topic.c_str(), buf, len);
      }
    } else {
      mqtt.publish(topic.c_str(), buf, len);
    }
  } else {
    if (mqtt.publish(topic.c_str(), buf, len))
      Serial.println("Published to " + topic);
    else
      Serial.println("Publish failed");
  }
}
#endif

// Sequence: DISCONNECT WIFI -> READ SENSORS -> CAPTURE IMAGE (offline) -> CONNECT WIFI -> PUBLISH TO MQTT
void publishSensorData() {
  if (strlen(prefPlotId) == 0 || strlen(prefMqttBroker) == 0) return;
#if ENABLE_CAMERA
  // 1. Turn WiFi fully OFF so ADC2 is released (WiFi.disconnect() is not enough – ADC2 stays blocked)
  //    ESP32: ADC2 = GPIO 0,2,4,12,13,14,15,25,26,27; ADC1 = 32,33,34,35,36,39
  WiFi.disconnect(false);
  WiFi.mode(WIFI_OFF);
  delay(WIFI_DISCONNECT_DELAY_MS);
  Serial.println("WiFi OFF – reading ADC2 sensors and capturing offline");
#endif
  // 2. Read sensors (build JSON)
  char buf[800];
  size_t len = buildPlotJson(buf, sizeof(buf));
#if ENABLE_CAMERA
  // 3. Capture image (camera init -> capture -> save to lastImageB64 -> camera off)
  runCameraSequenceOnly();
  // 4. Connect WiFi
  wifiConnected = tryConnectWifi();
  if (!wifiConnected) {
    Serial.println("WiFi reconnect failed – data not published");
    return;
  }
  // 5. Publish to MQTT (sensor + image if captured)
  mqttReconnect();
  doPublishToMqtt(buf, len);
#else
  String topic = String(MQTT_TOPIC_PREFIX) + prefPlotId + "/data";
  if (mqtt.publish(topic.c_str(), buf, len))
    Serial.println("Published to " + topic);
  else
    Serial.println("Publish failed");
#endif
}

// API: GET /api/plot returns current sensor data + last cached image (no capture; cache updated every 1 min)
void handleApiPlot() {
  char buf[800];
  size_t len = buildPlotJson(buf, sizeof(buf));
#if ENABLE_CAMERA
  if (lastImageB64Len > 0 && lastImageB64) {
    DynamicJsonDocument doc(MQTT_DATA_MAX_SIZE);
    if (deserializeJson(doc, buf) == DeserializationError::Ok) {
      doc["image"] = lastImageB64;
      String out;
      serializeJson(doc, out);
      server.send(200, "application/json", out);
      return;
    }
  }
#endif
  server.send(200, "application/json", buf);
}

// Dashboard: live data page, updates every 20s
void handleDashboard() {
  String html = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Plot live data</title>
  <style>
    * { box-sizing: border-box; }
    body { font-family: sans-serif; margin: 0; padding: 16px; background: #0f0e17; color: #eee; min-height: 100vh; }
    h1 { font-size: 1.4rem; color: #ffc859; margin: 0 0 8px 0; }
    .meta { color: #888; font-size: 0.9rem; margin-bottom: 16px; }
    .meta span { margin-right: 16px; }
    .pots { display: grid; grid-template-columns: repeat(auto-fill, minmax(200px, 1fr)); gap: 16px; }
    .card { background: #1a1a2e; border-radius: 12px; padding: 16px; border: 1px solid #2d2d44; }
    .card h2 { font-size: 1rem; color: #a8e6cf; margin: 0 0 12px 0; }
    .row { display: flex; justify-content: space-between; margin: 6px 0; font-size: 0.9rem; }
    .row .v { color: #ffc859; font-weight: 600; }
    .updated { color: #666; font-size: 0.8rem; margin-top: 16px; }
    .err { color: #e94560; }
    a { color: #a8e6cf; }
    .camera { margin-bottom: 16px; }
    .camera img { max-width: 100%; max-height: 280px; border-radius: 8px; border: 1px solid #2d2d44; }
    .camera .noimg { color: #666; font-size: 0.9rem; }
    .relay-box { background: #1a1a2e; border-radius: 12px; padding: 16px; border: 1px solid #2d2d44; margin-bottom: 16px; display: flex; align-items: center; gap: 16px; }
    .relay-box span { font-size: 1rem; }
    .switch { position: relative; display: inline-block; width: 52px; height: 28px; }
    .switch input { opacity: 0; width: 0; height: 0; }
    .slider { position: absolute; cursor: pointer; top: 0; left: 0; right: 0; bottom: 0; background: #444; border-radius: 28px; transition: 0.3s; }
    .slider:before { position: absolute; content: ""; height: 22px; width: 22px; left: 3px; bottom: 3px; background: #fff; border-radius: 50%; transition: 0.3s; }
    input:checked + .slider { background: #0f3460; }
    input:checked + .slider:before { transform: translateX(24px); }
    .relay-status { font-weight: 600; color: #ffc859; }
  </style>
</head>
<body>
  <h1>Live plot data</h1>
  <div class="meta">
    <span>Plot: <strong id="plotId">--</strong></span>
    <span>Device IP: <strong id="deviceIp">--</strong></span>
    <span>Updates every 20s</span>
  </div>
  <div class="camera" id="cameraBox">
    <div class="noimg" id="cameraPlaceholder">No image</div>
    <img id="cameraImg" alt="Plot" style="display:none;">
  </div>
  <div class="relay-box">
    <span>Relay / Pump:</span>
    <label class="switch">
      <input type="checkbox" id="relayToggle">
      <span class="slider"></span>
    </label>
    <span class="relay-status" id="relayStatus">OFF</span>
  </div>
  <div class="pots" id="pots"></div>
  <p class="updated">Last update: <span id="lastUpdate">never</span></p>
  <p><a href="/">Setup</a> &nbsp; <a href="/cam">Test camera</a></p>
  <script>
    var potsEl = document.getElementById('pots');
    var plotIdEl = document.getElementById('plotId');
    var deviceIpEl = document.getElementById('deviceIp');
    var lastUpdateEl = document.getElementById('lastUpdate');
    var cameraImg = document.getElementById('cameraImg');
    var cameraPlaceholder = document.getElementById('cameraPlaceholder');
    var relayToggle = document.getElementById('relayToggle');
    var relayStatus = document.getElementById('relayStatus');
    function fmt(v) { return (v != null && v !== '' && v !== undefined) ? v : 0; }
    relayToggle.onchange = function() {
      var st = relayToggle.checked ? 'on' : 'off';
      fetch('/api/relay?state=' + st, { method: 'POST' })
        .then(function(r) { return r.json(); })
        .then(function(d) {
          relayStatus.textContent = d.relay === 'on' ? 'ON' : 'OFF';
        })
        .catch(function() { relayToggle.checked = !relayToggle.checked; });
    };
    function loadRelayState() {
      fetch('/api/relay').then(function(r) { return r.json(); }).then(function(d) {
        relayToggle.checked = d.relay === 'on';
        relayStatus.textContent = d.relay === 'on' ? 'ON' : 'OFF';
      }).catch(function(){});
    }
    loadRelayState();
    function updateUI(data) {
      plotIdEl.textContent = fmt(data.id);
      deviceIpEl.textContent = fmt(data.ip);
      lastUpdateEl.textContent = new Date().toLocaleTimeString();
      if (data.relay !== undefined) {
        relayToggle.checked = data.relay === 'on';
        relayStatus.textContent = data.relay === 'on' ? 'ON' : 'OFF';
      }
      if (data.image) {
        cameraImg.src = 'data:image/jpeg;base64,' + data.image;
        cameraImg.style.display = 'block';
        cameraPlaceholder.style.display = 'none';
      } else {
        cameraImg.style.display = 'none';
        cameraPlaceholder.style.display = 'block';
        cameraPlaceholder.textContent = 'No image';
      }
      potsEl.innerHTML = '';
      if (!data.pots || !data.pots.length) return;
      data.pots.forEach(function(p) {
        var card = document.createElement('div');
        card.className = 'card';
        card.innerHTML = '<h2>' + fmt(p.name) + '</h2>' +
          '<div class="row">Moisture: <span class="v">' + fmt(p.moisture) + '%</span></div>' +
          '<div class="row">Temperature: <span class="v">' + fmt(p.temperature) + ' °C</span></div>' +
          '<div class="row">Humidity: <span class="v">' + fmt(p.humidity) + '%</span></div>' +
          '<div class="row">Water level: <span class="v">' + fmt(p.waterLevel) + '%</span></div>' +
          '<div class="row">Light: <span class="v">' + fmt(p.lightIntensity) + '%</span></div>';
        potsEl.appendChild(card);
      });
    }
    function fetchData() {
      fetch('/api/plot').then(function(r) { return r.json(); }).then(updateUI).catch(function() {
        lastUpdateEl.textContent = 'Error fetching data';
        lastUpdateEl.className = 'updated err';
      });
    }
    fetchData();
    setInterval(fetchData, 20000);
  </script>
</body>
</html>
)rawliteral";
  server.send(200, "text/html", html);
}

// ============== CAMERA (hourly image to MQTT) ==============
#if ENABLE_CAMERA

static camera_config_t buildCameraConfig(int xclk_mhz) {
  camera_config_t cfg = {};
  cfg.pin_pwdn       = -1;
  cfg.pin_reset      = -1;
  cfg.pin_xclk       = 21;
  cfg.pin_sscb_sda   = 26;
  cfg.pin_sscb_scl   = 27;
  cfg.pin_d7         = 35;    // Y9
  cfg.pin_d6         = 34;    // Y8
  cfg.pin_d5         = 39;   // Y7
  cfg.pin_d4         = 36;   // Y6
  cfg.pin_d3         = 19;   // Y5
  cfg.pin_d2         = 18;   // Y4
  cfg.pin_d1         = 5;    // Y3
  cfg.pin_d0         = 4;   // Y2
  cfg.pin_vsync      = 25;
  cfg.pin_href       = 23;
  cfg.pin_pclk       = 22;
  cfg.xclk_freq_hz   = xclk_mhz * 1000000;
  cfg.ledc_timer     = LEDC_TIMER_0;
  cfg.ledc_channel   = LEDC_CHANNEL_0;
  cfg.pixel_format   = PIXFORMAT_JPEG;
  cfg.frame_size     = FRAMESIZE_VGA;
  cfg.jpeg_quality   = 20;
  cfg.fb_count       = 2;
  cfg.fb_location    = CAMERA_FB_IN_PSRAM;
  cfg.grab_mode      = CAMERA_GRAB_LATEST;
  return cfg;
}

bool initCamera() {
  cameraError[0] = '\0';
  if (!psramFound()) {
    Serial.println("Camera: PSRAM not found! Enable PSRAM in Tools menu and re-upload.");
    strncpy(cameraError, "PSRAM not found", sizeof(cameraError) - 1);
    return false;
  }

  // Try 10 MHz first (more reliable for OV3660), fall back to 8 MHz, then 20 MHz
  const int freqs[] = { 20 };
  esp_err_t err = ESP_FAIL;
  for (int i = 0; i < 3; i++) {
    Serial.printf("Camera init attempt: XCLK = %d MHz ...\n", freqs[i]);
    camera_config_t cfg = buildCameraConfig(freqs[i]);
    err = esp_camera_init(&cfg);
    if (err == ESP_OK) {
      Serial.printf("Camera init OK at %d MHz\n", freqs[i]);
      break;
    }
    Serial.printf("  failed: %s (0x%x)\n", esp_err_to_name(err), err);
    esp_camera_deinit();
    delay(300);
  }

  if (err != ESP_OK) {
    const char *msg = esp_err_to_name(err);
    strncpy(cameraError, msg ? msg : "unknown", sizeof(cameraError) - 1);
    cameraError[sizeof(cameraError) - 1] = '\0';
    if (err == ESP_ERR_NOT_SUPPORTED)
      Serial.println("SCCB could not detect a camera sensor. Check:\n"
                     "  1. SIOD wired to GPIO 26, SIOC wired to GPIO 27\n"
                     "  2. XCLK wired to GPIO 13\n"
                     "  3. Camera module has 3.3 V on VCC and GND connected\n"
                     "  4. PWDN pin on camera tied to GND (not floating)\n"
                     "  5. 4.7k pull-up resistors on SIOD & SIOC to 3.3 V");
    else if (err == ESP_FAIL)
      Serial.println("ESP_FAIL: enable PSRAM in Tools -> PSRAM -> OPI PSRAM.");
    return false;
  }

  sensor_t *s = esp_camera_sensor_get();
  if (s) {
    Serial.printf("Sensor PID: 0x%02X  VER: 0x%02X  MIDH: 0x%02X  MIDL: 0x%02X\n",
                  s->id.PID, s->id.VER, s->id.MIDH, s->id.MIDL);
    if (s->id.PID == OV3660_PID) {
      s->set_vflip(s, 1);
      s->set_brightness(s, 1);
      s->set_saturation(s, -1);
      Serial.println("OV3660 tuning applied");
    }
  } else {
    Serial.println("Camera OK (sensor struct unavailable)");
  }
  return true;
}

// Test camera: on "Take snapshot" click -> INIT -> CAPTURE -> send image -> OFF (no camera left on).
void handleSnapshot() {
  // 1. Init camera (it is off until we need it)
  if (!cameraOk)
    cameraOk = initCamera();
  if (!cameraOk) {
    server.send(503, "text/plain", "Camera not available");
    return;
  }
  // Let sensor stabilize (reduces green/corrupt first frame)
  delay(CAMERA_STABILIZE_MS);
  // Discard dummy frames; first frames are often green or bad
  for (int i = 0; i < CAMERA_DUMMY_FRAMES; i++) {
    camera_fb_t *dummy = esp_camera_fb_get();
    if (dummy) {
      esp_camera_fb_return(dummy);
      delay(CAMERA_FRAME_DELAY_MS);
    }
  }
  // 2. Capture (keep this frame)
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb || fb->len == 0 || fb->len < CAMERA_MIN_FRAME_SIZE) {
    if (fb) esp_camera_fb_return(fb);
    esp_camera_deinit();
    cameraOk = false;
    server.send(503, "text/plain", "Capture failed or bad frame");
    return;
  }
  server.setContentLength(fb->len);
  server.send(200, "image/jpeg", "");
  server.sendContent((const char *)fb->buf, fb->len);
  esp_camera_fb_return(fb);
  // 3. Turn off camera
  esp_camera_deinit();
  cameraOk = false;
}

void handleTestCam() {
  // Only show error page (no snapshot button) when init actually failed before
  if (!cameraOk && cameraError[0]) {
    String msg = String("<!DOCTYPE html><html><body><h1>Test camera</h1><p><strong>Error:</strong> " + String(cameraError) + "</p>");
    if (strcmp(cameraError, "PSRAM not found") == 0 || strcmp(cameraError, "ESP_FAIL") == 0)
      msg += "<p class=\"note\"><strong>Fix:</strong> In Arduino IDE set <b>Tools &rarr; PSRAM &rarr; OPI PSRAM</b> (or Enabled), then re-upload.</p>";
    else if (strcmp(cameraError, "ESP_ERR_NOT_SUPPORTED") == 0)
      msg += "<p class=\"note\"><strong>Fix (check all):</strong><br>"
             "1. SIOD &rarr; GPIO 26, SIOC &rarr; GPIO 27<br>"
             "2. XCLK &rarr; GPIO 13<br>"
             "3. Camera VCC &rarr; 3.3&thinsp;V, GND &rarr; GND<br>"
             "4. PWDN pin tied to <b>GND</b> (not floating!)<br>"
             "5. 4.7k&Omega; pull-ups on SIOD &amp; SIOC to 3.3&thinsp;V<br>"
             "Open <b>Serial Monitor</b> (115200) for SCCB scan results.</p>";
    else
      msg += "<p class=\"note\">Check Serial Monitor (115200) for details.</p>";
    msg += "<p><a href=\"/cam/retry\">Retry camera init</a> &nbsp; <a href=\"/\">Back to setup</a></p></body></html>";
    server.send(200, "text/html", msg);
    return;
  }
  // Show snapshot page: click = init -> capture -> off (camera may be off when page loads)
  String html = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Test camera</title>
  <style>
    body { font-family: sans-serif; max-width: 640px; margin: 20px auto; padding: 16px; background: #1a1a2e; color: #eee; }
    h1 { color: #a8e6cf; }
    img { max-width: 100%; border: 2px solid #0f3460; border-radius: 8px; background: #000; }
    button { background: #0f3460; color: #fff; border: none; padding: 12px 24px; border-radius: 6px; cursor: pointer; font-size: 1rem; margin: 8px 8px 8px 0; }
    button:disabled { opacity: 0.6; cursor: not-allowed; }
    a { color: #a8e6cf; }
    .note { color: #888; font-size: 0.9rem; margin-top: 8px; }
  </style>
</head>
<body>
  <h1>Test camera</h1>
  <p class="note">On click: init camera &rarr; capture &rarr; camera off.</p>
  <p><button type="button" id="btn">Take snapshot</button> <a href="/">Back to setup</a></p>
  <p id="status" class="note"></p>
  <img id="preview" alt="Preview" style="display:none;">
  <script>
    var btn = document.getElementById('btn');
    var preview = document.getElementById('preview');
    var status = document.getElementById('status');
    btn.onclick = function() {
      btn.disabled = true;
      status.textContent = 'Capturing...';
      preview.style.display = 'none';
      fetch('/snapshot?t=' + Date.now()).then(function(r) {
        if (!r.ok) throw new Error('Capture failed');
        return r.blob();
      }).then(function(blob) {
        preview.src = URL.createObjectURL(blob);
        preview.style.display = 'block';
        status.textContent = 'OK – ' + (blob.size / 1024).toFixed(1) + ' KB';
        btn.disabled = false;
      }).catch(function(e) {
        status.textContent = 'Error: ' + e.message;
        btn.disabled = false;
      });
    };
  </script>
</body>
</html>
)rawliteral";
  server.send(200, "text/html", html);
}

void handleCamRetry() {
  esp_camera_deinit();
  cameraOk = initCamera();
  server.sendHeader("Location", "/cam", true);
  server.send(302, "text/plain", "");
}
#endif

// ============== SETUP & LOOP ==============
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\nESP32 Plot - 4 Pots + MQTT");

  setupPins();
  loadPreferences();

  wifiConnected = tryConnectWifi();
  if (!wifiConnected) {
    startAP();
    lastPublishMs = 0;
    return;
  }

#if ENABLE_CAMERA
  // Camera stays OFF at startup; init only in sequence: read sensor → init → capture → publish → deinit
  mqtt.setBufferSize(MQTT_DATA_MAX_SIZE);  // large buffer for /data payload with base64 image
#else
  mqtt.setBufferSize(1024);
#endif
  mqtt.setServer(prefMqttBroker, prefMqttPort);
  mqtt.setCallback(mqttCallback);
  lastPublishMs = millis();

  // Start web server on WiFi IP so you can open http://<ESP32_IP>/ and /cam
  setupWebServer();
  Serial.println("Web: http://" + WiFi.localIP().toString());
}

void loop() {
  if (wifiConnected) {
    if (WiFi.status() != WL_CONNECTED) {
      wifiConnected = false;
      Serial.println("WiFi lost");
      return;
    }
    server.handleClient();  // serve setup page and /cam at http://<ESP32_IP>/
    mqtt.loop();
    mqttReconnect();
    if (mqtt.connected() && strlen(prefPlotId) > 0) {
      unsigned long now = millis();
      // Single interval: disconnect WiFi -> read sensors -> capture (offline) -> connect -> publish MQTT
      if (now - lastPublishMs >= MQTT_PUBLISH_INTERVAL_MS) {
        lastPublishMs = now;
        publishSensorData();
      }
    }
  } else {
    server.handleClient();
  }
  delay(500);
}
  