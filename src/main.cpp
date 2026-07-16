#include <Arduino.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <HX711.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include <ESP32Servo.h>

// ================= WiFi =================
// Credentials live in include/secrets.h (git-ignored so they never reach GitHub).
// First-time setup: copy include/secrets.h.example to include/secrets.h and fill it in.
#include "secrets.h"
const char* HOSTNAME      = "thruststand";          // reachable at http://thruststand.local

// ================= Load cell =================
const int DT_PIN  = 14;
const int SCK_PIN = 13;

// Run the calibration sketch and paste the resulting factor here.
const float CALIBRATION_FACTOR = 360;

HX711 scale;

// Averaged reading in grams. Once mounted on the thrust stand this same
// number *is* thrust; convert to Newtons here (grams / 1000 * 9.81) if
// you'd rather the page display SI units.
float getReading() {
  return scale.get_units(5);
}

// ================= ESC / Motor =================
// SAFETY:
// - Motor/ESC power comes from its own supply; the ESP32 stays on USB power.
// - Tie the ESC's signal ground to the ESP32's ground (share a common GND)
//   or the PWM signal has no reference and the ESC will not respond.
// - Secure the prop/stand before powering the motor supply - the motor
//   WILL spin as soon as it's armed and given nonzero throttle.

const int ESC_PIN    = 25;    // signal wire to the ESC
const int ESC_MIN_US = 1000;  // pulse width for zero throttle / arming
const int ESC_MAX_US = 2000;  // pulse width for full throttle
const unsigned long ESC_ARM_DELAY_MS = 3000;  // hold min throttle this long before accepting commands

Servo esc;
int currentThrottlePercent = 0;
unsigned long escAttachTime = 0;
bool escArmed = false;

void setThrottlePercent(int percent) {
  percent = constrain(percent, 0, 100);
  currentThrottlePercent = percent;
  int pulse = map(percent, 0, 100, ESC_MIN_US, ESC_MAX_US);
  esc.writeMicroseconds(pulse);
}

// ================= Web server =================
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

volatile float g_weight = 0;
volatile float g_volts  = 0;
volatile float g_amps   = 0;

unsigned long lastSample = 0;
const unsigned long SAMPLE_INTERVAL_MS = 150;

// Rolling average: take a reading every SAMPLE_INTERVAL_MS, and once
// SAMPLES_PER_AVERAGE of them have been collected, average and broadcast
// (so the page updates every SAMPLE_INTERVAL_MS * SAMPLES_PER_AVERAGE).
const int SAMPLES_PER_AVERAGE = 3;
float sampleBuffer[SAMPLES_PER_AVERAGE];
int sampleIndex = 0;

// Single-file UI: edit the labels/CSS/JS below directly. "Weight (g)" is the
// one line to change to "Thrust (g)" (or similar) when you move to the stand.
const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Thrust Stand</title>
<style>
  :root {
    color-scheme: dark;
    --bg: #0f1216;
    --panel: #171b21;
    --border: #262c35;
    --text: #e6e9ef;
    --muted: #8b93a1;
    --accent: #4da3ff;
  }
  * { box-sizing: border-box; }
  body {
    margin: 0;
    min-height: 100vh;
    background: var(--bg);
    color: var(--text);
    font-family: -apple-system, Segoe UI, Roboto, sans-serif;
    display: flex;
    justify-content: center;
    padding: 32px 16px;
  }
  .wrap { width: 100%; max-width: 480px; display: flex; flex-direction: column; gap: 16px; }
  h1 { font-size: 18px; font-weight: 600; color: var(--muted); margin: 0 0 4px; text-transform: uppercase; letter-spacing: 0.06em; }
  .status { font-size: 12px; color: var(--muted); display: flex; align-items: center; gap: 6px; }
  .dot { width: 8px; height: 8px; border-radius: 50%; background: #e5484d; }
  .dot.connected { background: #3dd68c; }
  .panel { background: var(--panel); border: 1px solid var(--border); border-radius: 12px; padding: 20px; }
  .reading-label { font-size: 13px; color: var(--muted); margin-bottom: 4px; }
  .reading-value { font-size: 56px; font-weight: 700; font-variant-numeric: tabular-nums; line-height: 1.1; }
  .reading-unit { font-size: 20px; color: var(--muted); margin-left: 6px; }
  button {
    background: var(--accent); color: #06121f; border: none; border-radius: 8px;
    padding: 12px 20px; font-size: 15px; font-weight: 600; cursor: pointer; width: 100%;
  }
  button:active { opacity: 0.85; }
  button:disabled { opacity: 0.4; cursor: not-allowed; }
  .grid { display: grid; grid-template-columns: 1fr 1fr; gap: 12px; }
  label { font-size: 12px; color: var(--muted); display: block; margin-bottom: 6px; }
  input {
    width: 100%; background: #0f1216; border: 1px solid var(--border); border-radius: 8px;
    padding: 10px; color: var(--text); font-size: 15px; font-variant-numeric: tabular-nums;
  }
  .stats { display: grid; grid-template-columns: 1fr 1fr; gap: 12px; margin-top: 4px; }
  .stat-label { font-size: 12px; color: var(--muted); }
  .stat-value { font-size: 22px; font-weight: 600; font-variant-numeric: tabular-nums; }
</style>
</head>
<body>
<div class="wrap">
  <div>
    <h1>Thrust Stand</h1>
    <div class="status"><span class="dot" id="dot"></span><span id="statusText">connecting...</span></div>
  </div>

  <div class="panel">
    <div class="reading-label">Force</div>
    <div><span class="reading-value" id="weight">--</span><span class="reading-unit">g</span></div>
  </div>

  <div class="panel">
    <div class="reading-label">Thrust</div>
    <div><span class="reading-value" id="throttleValue">--</span><span class="reading-unit">%</span></div>
  </div>

  <div class="panel">
    <div class="reading-label" style="margin-bottom: 12px;">Motor throttle</div>
    <div style="font-size: 13px; color: var(--muted); margin-bottom: 12px;">
      ESC arms automatically ~3s after the ESP32 boots. Power on the motor
      supply during/after that window.
    </div>
    <label for="throttle">Set throttle (%): <span id="throttleSliderLabel">0</span>%</label>
    <input type="range" id="throttle" min="0" max="100" step="1" value="0"
           oninput="document.getElementById('throttleSliderLabel').textContent = this.value">
    <div class="grid" style="margin-top: 12px;">
      <button id="setThrottleBtn">Apply</button>
      <button id="stopBtn" style="background: #e5484d; color: #fff;">STOP</button>
    </div>
  </div>

  <div class="panel">
    <div class="reading-label" style="margin-bottom: 12px;">Motor power input</div>
    <div class="grid">
      <div>
        <label for="volts">Volts (V)</label>
        <input type="number" id="volts" step="0.1" placeholder="0.0">
      </div>
      <div>
        <label for="amps">Amps (A)</label>
        <input type="number" id="amps" step="0.1" placeholder="0.0">
      </div>
    </div>
    <button id="applyBtn" style="margin-top: 12px;">Apply</button>

    <div class="stats">
      <div>
        <div class="stat-label">Power</div>
        <div class="stat-value" id="power">-- W</div>
      </div>
      <div>
        <div class="stat-label">Efficiency</div>
        <div class="stat-value" id="efficiency">-- g/W</div>
      </div>
    </div>
  </div>

  <div class="panel">
    <div class="reading-label" style="margin-bottom: 12px;">Data logging</div>
    <div style="font-size: 13px; color: var(--muted); margin-bottom: 12px;" id="logStatus">Stopped &mdash; 0 samples</div>
    <div class="grid">
      <button id="recordBtn">Record</button>
      <button id="clearBtn" style="background: #262c35; color: var(--text);">Clear</button>
    </div>
    <button id="downloadBtn" style="margin-top: 12px;" disabled>Download CSV</button>
  </div>

  <div class="panel">
    <div class="reading-label" style="margin-bottom: 12px;">Calibrate</div>
    <div style="font-size: 13px; color: var(--muted); margin-bottom: 12px;">
      1. Remove all weight, click Tare below.<br>
      2. Place a known weight, enter its mass below, click Calculate.
    </div>
    <button id="tareBtn">Tare</button>
    <label for="knownWeight" style="margin-top: 12px;">Known weight (g)</label>
    <input type="number" id="knownWeight" step="0.01" placeholder="e.g. 11.20">
    <button id="calibrateBtn" style="margin-top: 12px;">Calculate factor</button>
    <div class="stats" style="grid-template-columns: 1fr; margin-top: 12px;">
      <div>
        <div class="stat-label">Calibration factor &mdash; paste into CALIBRATION_FACTOR</div>
        <div class="stat-value" id="calFactor">--</div>
      </div>
    </div>
  </div>
</div>

<script>
  const dot = document.getElementById('dot');
  const statusText = document.getElementById('statusText');
  const weightEl = document.getElementById('weight');
  const powerEl = document.getElementById('power');
  const efficiencyEl = document.getElementById('efficiency');
  const throttleValueEl = document.getElementById('throttleValue');
  const recordBtn = document.getElementById('recordBtn');
  const clearBtn = document.getElementById('clearBtn');
  const downloadBtn = document.getElementById('downloadBtn');
  const logStatusEl = document.getElementById('logStatus');

  let recording = false;
  let recordStartTime = 0;
  let log = [];

  function updateLogStatus() {
    logStatusEl.textContent = (recording ? 'Recording — ' : 'Stopped — ') + log.length + ' samples';
    downloadBtn.disabled = log.length === 0;
  }

  recordBtn.onclick = () => {
    recording = !recording;
    recordStartTime = Date.now();
    recordBtn.textContent = recording ? 'Stop recording' : 'Record';
    recordBtn.style.background = recording ? '#e5484d' : '';
    updateLogStatus();
  };

  clearBtn.onclick = () => {
    log = [];
    updateLogStatus();
  };

  downloadBtn.onclick = () => {
    let csv = 'time_s,force_g,throttle_pct,volts,amps,efficiency_g_per_w\n';
    for (const row of log) {
      csv += `${row.t.toFixed(2)},${row.weight.toFixed(2)},${row.throttle},${row.volts.toFixed(2)},${row.amps.toFixed(2)},${row.efficiency.toFixed(2)}\n`;
    }
    const blob = new Blob([csv], { type: 'text/csv' });
    const url = URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.href = url;
    a.download = `thrust-log-${Date.now()}.csv`;
    a.click();
    URL.revokeObjectURL(url);
  };

  function connect() {
    const ws = new WebSocket(`ws://${location.host}/ws`);

    ws.onopen = () => { dot.classList.add('connected'); statusText.textContent = 'live'; };
    ws.onclose = () => { dot.classList.remove('connected'); statusText.textContent = 'disconnected — retrying...'; setTimeout(connect, 1000); };
    ws.onerror = () => ws.close();

    ws.onmessage = (evt) => {
      const d = JSON.parse(evt.data);
      weightEl.textContent = d.weight.toFixed(2);
      powerEl.textContent = d.power.toFixed(2) + ' W';
      efficiencyEl.textContent = d.efficiency.toFixed(2) + ' g/W';
      throttleValueEl.textContent = d.throttle;

      if (recording) {
        log.push({
          t: (Date.now() - recordStartTime) / 1000,
          weight: d.weight, throttle: d.throttle,
          volts: d.volts, amps: d.amps, efficiency: d.efficiency
        });
        updateLogStatus();
      }
    };
  }
  connect();

  document.getElementById('tareBtn').onclick = () => fetch('/tare');

  document.getElementById('calibrateBtn').onclick = () => {
    const w = document.getElementById('knownWeight').value;
    if (!w || w <= 0) { alert('Enter a known weight first'); return; }
    fetch(`/calibrate?weight=${w}`)
      .then(r => r.json())
      .then(d => { document.getElementById('calFactor').textContent = d.factor.toFixed(4); });
  };

  document.getElementById('applyBtn').onclick = () => {
    const v = document.getElementById('volts').value || 0;
    const a = document.getElementById('amps').value || 0;
    fetch(`/setpower?volts=${v}&amps=${a}`);
  };

  document.getElementById('setThrottleBtn').onclick = () => {
    fetch(`/throttle?value=${document.getElementById('throttle').value}`);
  };

  document.getElementById('stopBtn').onclick = () => {
    document.getElementById('throttle').value = 0;
    document.getElementById('throttleSliderLabel').textContent = '0';
    fetch('/throttle?value=0');
  };
</script>
</body>
</html>
)rawliteral";

void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type,
               void *arg, uint8_t *data, size_t len) {
  // One-way: server pushes readings, so no client messages need handling.
  // Safety failsafe: if the controlling page disconnects (tab closed, WiFi
  // drop, etc.), cut the throttle instead of leaving the motor spinning
  // with no one watching it.
  if (type == WS_EVT_DISCONNECT) {
    setThrottlePercent(0);
  }
}

void handleTare(AsyncWebServerRequest *request) {
  scale.tare(10);
  request->send(200, "text/plain", "OK");
}

void handleCalibrate(AsyncWebServerRequest *request) {
  if (!request->hasParam("weight")) {
    request->send(400, "text/plain", "missing weight param");
    return;
  }
  float knownWeight = request->getParam("weight")->value().toFloat();
  if (knownWeight <= 0) {
    request->send(400, "text/plain", "weight must be > 0");
    return;
  }

  // get_value() returns raw counts relative to the last tare, ignoring
  // whatever scale factor is currently set, so this works regardless of
  // what CALIBRATION_FACTOR was flashed with.
  long rawValue = scale.get_value(20);
  float factor = rawValue / knownWeight;
  scale.set_scale(factor);  // applied live so you can sanity-check readings immediately

  char json[64];
  snprintf(json, sizeof(json), "{\"factor\":%.4f}", factor);
  request->send(200, "application/json", json);
}

void handleSetThrottle(AsyncWebServerRequest *request) {
  if (!escArmed) {
    request->send(503, "text/plain", "ESC still arming, try again in a moment");
    return;
  }
  if (!request->hasParam("value")) {
    request->send(400, "text/plain", "missing value param");
    return;
  }
  setThrottlePercent(request->getParam("value")->value().toInt());
  request->send(200, "text/plain", "OK");
}

void handleSetPower(AsyncWebServerRequest *request) {
  if (request->hasParam("volts")) g_volts = request->getParam("volts")->value().toFloat();
  if (request->hasParam("amps"))  g_amps  = request->getParam("amps")->value().toFloat();
  request->send(200, "text/plain", "OK");
}

void setup() {
  Serial.begin(115200);
  delay(500);

  // Attach and hold minimum throttle as early as possible: this is a
  // hardware PWM output, so it keeps running unattended through the rest
  // of setup() (WiFi connect, etc.), giving the ESC its full arm window
  // regardless of when the motor supply gets powered on.
  esc.setPeriodHertz(50);
  esc.attach(ESC_PIN, ESC_MIN_US, ESC_MAX_US);
  setThrottlePercent(0);
  escAttachTime = millis();

  scale.begin(DT_PIN, SCK_PIN);
  scale.set_scale(CALIBRATION_FACTOR);
  scale.tare(20);

  WiFi.mode(WIFI_STA);
  WiFi.setHostname(HOSTNAME);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(300);
    Serial.print(".");
  }
  Serial.println();
  Serial.print("Connected. IP address: ");
  Serial.println(WiFi.localIP());

  if (MDNS.begin(HOSTNAME)) {
    Serial.print("Open http://");
    Serial.print(HOSTNAME);
    Serial.println(".local in your browser");
  } else {
    Serial.println("mDNS failed to start; use the IP address above instead.");
  }

  ws.onEvent(onWsEvent);
  server.addHandler(&ws);

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/html", INDEX_HTML);
  });
  server.on("/tare", HTTP_GET, handleTare);
  server.on("/calibrate", HTTP_GET, handleCalibrate);
  server.on("/setpower", HTTP_GET, handleSetPower);
  server.on("/throttle", HTTP_GET, handleSetThrottle);

  server.begin();
}

void loop() {
  if (!escArmed && millis() - escAttachTime >= ESC_ARM_DELAY_MS) {
    escArmed = true;
  }

  if (millis() - lastSample >= SAMPLE_INTERVAL_MS) {
    lastSample = millis();

    sampleBuffer[sampleIndex] = getReading();
    sampleIndex++;

    if (sampleIndex >= SAMPLES_PER_AVERAGE) {
      float sum = 0;
      for (int i = 0; i < SAMPLES_PER_AVERAGE; i++) sum += sampleBuffer[i];
      g_weight = sum / SAMPLES_PER_AVERAGE;
      sampleIndex = 0;

      float power = g_volts * g_amps;                              // watts
      float efficiency = (power > 0.01f) ? (g_weight / power) : 0;  // grams-force per watt

      char json[192];
      snprintf(json, sizeof(json),
        "{\"weight\":%.2f,\"volts\":%.2f,\"amps\":%.2f,\"power\":%.2f,\"efficiency\":%.2f,\"throttle\":%d}",
        g_weight, g_volts, g_amps, power, efficiency, currentThrottlePercent);

      ws.textAll(json);
      ws.cleanupClients();
    }
  }
}
