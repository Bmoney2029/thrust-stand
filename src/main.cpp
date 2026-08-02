#include <Arduino.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <HX711.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include <INA226_WE.h>

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

// ================= Power monitor (INA226) =================
// WIRING:
//   Cut the BATTERY POSITIVE lead only. Battery side -> "Current +" screw
//   terminal, ESC side -> "Current -". The negative lead runs battery -> ESC
//   unbroken; it does NOT pass through the module.
//   V+  = thin sense wire to the positive rail BETWEEN the module and the ESC.
//   V-  = thin sense wire spliced onto the battery negative.
//   Both sense wires carry no current - never let motor return current find them.
//
//   Use the LARGE SCREW TERMINALS for the current path. On R002 modules the
//   small IN+/IN- through-holes sit behind 4.7 ohm series resistors, which
//   corrupts the shunt voltage. The library author documents this:
//   https://github.com/wollewald/INA226_WE
//
//   VCC -> ESP32 3V3 (NOT 5V - the onboard I2C pull-ups sit at whatever you
//   feed VCC, and 5V on an ESP32 GPIO is out of spec). ALERT stays unconnected.
const int I2C_SDA_PIN = 21;
const int I2C_SCL_PIN = 22;
const uint8_t INA226_I2C_ADDRESS = 0x40;  // A0/A1 unconnected

// Shunt marked R002 on the board = 0.002 ohm. With a 20 A range the library
// picks a 1 mA current LSB, giving calibration register 2560 exactly and a
// full-scale of +/-32.7 A - comfortably past anything this stand will pull.
const float SHUNT_OHMS   = 0.002f;
const float MAX_CURRENT_A = 20.0f;

INA226_WE ina226(INA226_I2C_ADDRESS);
bool inaOk = false;

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
float voltsBuffer[SAMPLES_PER_AVERAGE];
float ampsBuffer[SAMPLES_PER_AVERAGE];
int sampleIndex = 0;

// ================= Automated throttle sweep =================
// Runs entirely on the ESP32 so a browser hiccup can't strand the motor at
// throttle. Each step: command the throttle, wait SETTLE for thrust and current
// to stop moving, then average everything arriving during AVERAGE and emit one
// point. Note the practical cadence: getReading() calls get_units(5) and the
// HX711 runs at 10 SPS, so one averaged sample lands roughly every 1.5 s. A
// 3000 ms averaging window therefore captures about two samples - lengthen it
// if you want smoother points.
enum SweepPhase { SW_IDLE, SW_SETTLE, SW_AVG, SW_DONE };
SweepPhase sweepPhase = SW_IDLE;

int sweepFrom = 0, sweepTo = 100, sweepStepPct = 10;
unsigned long sweepSettleMs = 3000, sweepAvgMs = 3000;
int sweepTarget = 0;
int sweepIndex = 0, sweepTotal = 0;
unsigned long sweepPhaseStart = 0;

double sAccW = 0, sAccW2 = 0, sAccV = 0, sAccA = 0;
int sAccN = 0;

inline void sweepResetAccum() { sAccW = sAccW2 = sAccV = sAccA = 0; sAccN = 0; }

// Accumulated on EVERY sample tick, not once per averaged set. The spread of
// the raw HX711 readings is a free vibration metric: a balanced prop on a rigid
// stand gives a tight cluster, while flutter or a resonance smears it out. If
// this were fed the pre-averaged values instead, the averaging would hide
// roughly a factor of sqrt(SAMPLES_PER_AVERAGE) of the very signal we want.
inline void sweepAccumulate(float w, float v, float a) {
  if (sweepPhase != SW_AVG) return;
  sAccW  += w;
  sAccW2 += (double)w * w;
  sAccV  += v;
  sAccA  += a;
  sAccN++;
}

void sweepBroadcastState(const char *state) {
  char json[96];
  snprintf(json, sizeof(json), "{\"t\":\"sweep\",\"state\":\"%s\"}", state);
  ws.textAll(json);
}

void sweepAbort(const char *reason) {
  if (sweepPhase == SW_IDLE) return;
  // A finished sweep just needs clearing - don't overwrite the "complete"
  // status on the page with an "aborted" message.
  if (sweepPhase == SW_DONE) {
    sweepPhase = SW_IDLE;
    return;
  }
  sweepPhase = SW_IDLE;
  setThrottlePercent(0);
  sweepBroadcastState(reason);
}

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
  .grid3 { display: grid; grid-template-columns: 1fr 1fr 1fr; gap: 10px; }
  canvas { width: 100%; height: 180px; display: block; }
  table { width: 100%; border-collapse: collapse; font-size: 12px; font-variant-numeric: tabular-nums; }
  th { text-align: right; color: var(--muted); font-weight: 500; padding: 6px 4px; border-bottom: 1px solid var(--border); }
  th:first-child, td:first-child { text-align: left; }
  td { text-align: right; padding: 5px 4px; border-bottom: 1px solid #1d2229; }
  .scroll { max-height: 240px; overflow-y: auto; margin-top: 8px; }
  .empty { font-size: 13px; color: var(--muted); padding: 16px 0; text-align: center; }
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
    <div style="font-size: 13px; color: #e5484d; margin-bottom: 12px; display: none;" id="inaWarning">
      INA226 not detected &mdash; check I2C wiring and the 3V3 supply.
    </div>
    <div class="stats">
      <div>
        <div class="stat-label">Voltage</div>
        <div class="stat-value" id="volts">-- V</div>
      </div>
      <div>
        <div class="stat-label">Current</div>
        <div class="stat-value" id="amps">-- A</div>
      </div>
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
    <div class="reading-label" style="margin-bottom: 12px;">Automated sweep</div>
    <div style="font-size: 13px; color: var(--muted); margin-bottom: 12px;">
      Steps the throttle and records one averaged point per step. Runs on the
      board, so it keeps going if the page stutters &mdash; but closing the page
      aborts it and cuts throttle.
    </div>
    <div class="grid3">
      <div>
        <label for="swFrom">From (%)</label>
        <input type="number" id="swFrom" min="0" max="100" step="1" value="0">
      </div>
      <div>
        <label for="swTo">To (%)</label>
        <input type="number" id="swTo" min="0" max="100" step="1" value="100">
      </div>
      <div>
        <label for="swStep">Step (%)</label>
        <input type="number" id="swStep" min="1" max="50" step="1" value="10">
      </div>
    </div>
    <div class="grid" style="margin-top: 10px;">
      <div>
        <label for="swSettle">Settle (s)</label>
        <input type="number" id="swSettle" min="0.5" max="30" step="0.5" value="3">
      </div>
      <div>
        <label for="swAvg">Average (s)</label>
        <input type="number" id="swAvg" min="1" max="30" step="0.5" value="3">
      </div>
    </div>
    <div class="grid" style="margin-top: 12px;">
      <button id="sweepStartBtn">Start sweep</button>
      <button id="sweepStopBtn" style="background: #e5484d; color: #fff;">Abort</button>
    </div>
    <div style="font-size: 13px; color: var(--muted); margin-top: 12px;" id="sweepStatus">Idle</div>
  </div>

  <div class="panel">
    <div class="reading-label" style="margin-bottom: 4px;">Thrust vs throttle</div>
    <canvas id="chartThrust"></canvas>
    <div class="reading-label" style="margin: 16px 0 4px;">Efficiency vs throttle</div>
    <canvas id="chartEff"></canvas>
    <div class="reading-label" style="margin: 16px 0 4px;">Thrust scatter (vibration) vs throttle</div>
    <div style="font-size: 12px; color: var(--muted); margin-bottom: 4px;">
      Standard deviation of raw load cell samples. A rising band means the rig
      started shaking there.
    </div>
    <canvas id="chartSd"></canvas>
  </div>

  <div class="panel">
    <div class="reading-label" style="margin-bottom: 4px;">Sweep results</div>
    <div class="scroll">
      <table id="sweepTable">
        <thead>
          <tr><th>Thr %</th><th>Thrust g</th><th>&plusmn;g</th><th>V</th><th>A</th><th>W</th><th>g/W</th></tr>
        </thead>
        <tbody></tbody>
      </table>
      <div class="empty" id="sweepEmpty">No sweep points yet</div>
    </div>
    <div class="grid" style="margin-top: 12px;">
      <button id="sweepCsvBtn" disabled>Download CSV</button>
      <button id="sweepClearBtn" style="background: #262c35; color: var(--text);">Clear</button>
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
  const voltsEl = document.getElementById('volts');
  const ampsEl = document.getElementById('amps');
  const powerEl = document.getElementById('power');
  const efficiencyEl = document.getElementById('efficiency');
  const inaWarningEl = document.getElementById('inaWarning');
  const throttleValueEl = document.getElementById('throttleValue');
  const recordBtn = document.getElementById('recordBtn');
  const clearBtn = document.getElementById('clearBtn');
  const downloadBtn = document.getElementById('downloadBtn');
  const logStatusEl = document.getElementById('logStatus');
  const sweepStatusEl = document.getElementById('sweepStatus');

  let recording = false;
  let recordStartTime = 0;
  let log = [];
  let sweepPoints = [];

  // ---- charting: no dependencies, so the page works when the only network
  // you're on is the ESP32's own WiFi ----
  const CH_AXIS = '#262c35', CH_GRID = '#1d2229', CH_TEXT = '#8b93a1';

  function drawChart(cv, pts, yKey, yLabel, color) {
    const dpr = window.devicePixelRatio || 1;
    const w = cv.clientWidth || 400, h = 180;
    cv.width = w * dpr; cv.height = h * dpr;
    const g = cv.getContext('2d');
    g.setTransform(dpr, 0, 0, dpr, 0, 0);
    g.clearRect(0, 0, w, h);

    const L = 46, R = 12, T = 10, B = 28;
    const pw = w - L - R, ph = h - T - B;

    g.strokeStyle = CH_AXIS; g.lineWidth = 1;
    g.beginPath(); g.moveTo(L, T); g.lineTo(L, T + ph); g.lineTo(L + pw, T + ph); g.stroke();

    g.fillStyle = CH_TEXT; g.font = '10px sans-serif';

    if (!pts.length) {
      g.textAlign = 'center'; g.textBaseline = 'middle';
      g.fillText('no data', L + pw / 2, T + ph / 2);
      return;
    }

    let x0 = 0, x1 = 100;
    let y1 = Math.max(...pts.map(p => p[yKey]));
    if (!isFinite(y1) || y1 <= 0) y1 = 1;
    y1 *= 1.15;

    const px = v => L + (v - x0) / (x1 - x0) * pw;
    const py = v => T + ph - (v / y1) * ph;

    g.textAlign = 'right'; g.textBaseline = 'middle';
    for (let i = 0; i <= 4; i++) {
      const v = y1 * i / 4, y = py(v);
      g.strokeStyle = CH_GRID; g.beginPath(); g.moveTo(L, y); g.lineTo(L + pw, y); g.stroke();
      g.fillStyle = CH_TEXT; g.fillText(v >= 100 ? v.toFixed(0) : v.toFixed(1), L - 6, y);
    }
    g.textAlign = 'center'; g.textBaseline = 'top';
    for (let i = 0; i <= 5; i++) {
      const v = x0 + (x1 - x0) * i / 5;
      g.fillText(v.toFixed(0), px(v), T + ph + 6);
    }

    const sorted = pts.slice().sort((a, b) => a.throttle - b.throttle);
    g.strokeStyle = color; g.lineWidth = 2; g.beginPath();
    sorted.forEach((p, i) => {
      const x = px(p.throttle), y = py(p[yKey]);
      i ? g.lineTo(x, y) : g.moveTo(x, y);
    });
    g.stroke();
    g.fillStyle = color;
    sorted.forEach(p => { g.beginPath(); g.arc(px(p.throttle), py(p[yKey]), 3, 0, Math.PI * 2); g.fill(); });

    g.fillStyle = CH_TEXT; g.textAlign = 'center'; g.textBaseline = 'bottom';
    g.fillText('throttle %', L + pw / 2, h);
    g.save();
    g.translate(11, T + ph / 2); g.rotate(-Math.PI / 2);
    g.textBaseline = 'top'; g.fillText(yLabel, 0, 0);
    g.restore();
  }

  function redrawCharts() {
    drawChart(document.getElementById('chartThrust'), sweepPoints, 'weight', 'thrust (g)', '#4da3ff');
    drawChart(document.getElementById('chartEff'), sweepPoints, 'efficiency', 'g/W', '#3dd68c');
    drawChart(document.getElementById('chartSd'), sweepPoints, 'sd', 'scatter (g)', '#f0a13c');
  }

  function renderSweep() {
    const tbody = document.querySelector('#sweepTable tbody');
    tbody.innerHTML = '';
    for (const p of sweepPoints) {
      const tr = document.createElement('tr');
      tr.innerHTML = `<td>${p.throttle}</td><td>${p.weight.toFixed(1)}</td>` +
                     `<td>${(p.sd ?? 0).toFixed(2)}</td>` +
                     `<td>${p.volts.toFixed(2)}</td><td>${p.amps.toFixed(2)}</td>` +
                     `<td>${p.power.toFixed(1)}</td><td>${p.efficiency.toFixed(2)}</td>`;
      tbody.appendChild(tr);
    }
    document.getElementById('sweepEmpty').style.display = sweepPoints.length ? 'none' : 'block';
    document.getElementById('sweepCsvBtn').disabled = sweepPoints.length === 0;
    redrawCharts();
  }

  window.addEventListener('resize', redrawCharts);

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
    let csv = 'time_s,force_g,throttle_pct,volts,amps,watts,efficiency_g_per_w\n';
    for (const row of log) {
      csv += `${row.t.toFixed(2)},${row.weight.toFixed(2)},${row.throttle},${row.volts.toFixed(2)},${row.amps.toFixed(2)},${row.power.toFixed(2)},${row.efficiency.toFixed(2)}\n`;
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

      if (d.t === 'pt') {
        sweepPoints.push(d);
        renderSweep();
        return;
      }
      if (d.t === 'sweep') {
        sweepStatusEl.textContent = d.state === 'done' ? 'Sweep complete — throttle returned to 0'
                                                       : 'Sweep aborted — throttle cut';
        return;
      }

      weightEl.textContent = d.weight.toFixed(2);
      voltsEl.textContent = d.volts.toFixed(2) + ' V';
      ampsEl.textContent = d.amps.toFixed(2) + ' A';
      powerEl.textContent = d.power.toFixed(2) + ' W';
      efficiencyEl.textContent = d.efficiency.toFixed(2) + ' g/W';
      throttleValueEl.textContent = d.throttle;
      inaWarningEl.style.display = d.inaOk ? 'none' : 'block';

      if (d.swActive) {
        const phase = d.swPhase === 'settle' ? 'settling' : 'averaging';
        sweepStatusEl.textContent =
          `Running — point ${d.swIndex + 1} of ${d.swTotal} at ${d.throttle}% (${phase})`;
      }

      if (recording) {
        log.push({
          t: (Date.now() - recordStartTime) / 1000,
          weight: d.weight, throttle: d.throttle,
          volts: d.volts, amps: d.amps, power: d.power, efficiency: d.efficiency
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

  document.getElementById('setThrottleBtn').onclick = () => {
    fetch(`/throttle?value=${document.getElementById('throttle').value}`);
  };

  document.getElementById('stopBtn').onclick = () => {
    document.getElementById('throttle').value = 0;
    document.getElementById('throttleSliderLabel').textContent = '0';
    fetch('/throttle?value=0');
  };

  document.getElementById('sweepStartBtn').onclick = () => {
    const from = +document.getElementById('swFrom').value;
    const to = +document.getElementById('swTo').value;
    const step = +document.getElementById('swStep').value;
    const settle = Math.round(+document.getElementById('swSettle').value * 1000);
    const avg = Math.round(+document.getElementById('swAvg').value * 1000);
    if (to <= from) { alert('"To" must be greater than "From"'); return; }
    if (sweepPoints.length && !confirm('Clear the existing sweep and start a new one?')) return;
    sweepPoints = [];
    renderSweep();
    sweepStatusEl.textContent = 'Starting...';
    fetch(`/sweep/start?from=${from}&to=${to}&step=${step}&settle=${settle}&avg=${avg}`)
      .then(r => r.ok ? r.json() : r.text().then(t => { throw new Error(t); }))
      .then(d => { sweepStatusEl.textContent = `Running — 0 of ${d.points}`; })
      .catch(e => { sweepStatusEl.textContent = 'Could not start: ' + e.message; });
  };

  document.getElementById('sweepStopBtn').onclick = () => {
    document.getElementById('throttle').value = 0;
    document.getElementById('throttleSliderLabel').textContent = '0';
    fetch('/sweep/stop');
  };

  document.getElementById('sweepClearBtn').onclick = () => {
    sweepPoints = [];
    renderSweep();
    sweepStatusEl.textContent = 'Idle';
  };

  document.getElementById('sweepCsvBtn').onclick = () => {
    let csv = 'throttle_pct,thrust_g,thrust_sd_g,samples,volts,amps,watts,efficiency_g_per_w\n';
    for (const p of sweepPoints) {
      csv += `${p.throttle},${p.weight.toFixed(2)},${(p.sd ?? 0).toFixed(3)},${p.n ?? 0},` +
             `${p.volts.toFixed(2)},${p.amps.toFixed(2)},${p.power.toFixed(2)},` +
             `${p.efficiency.toFixed(3)}\n`;
    }
    const url = URL.createObjectURL(new Blob([csv], { type: 'text/csv' }));
    const a = document.createElement('a');
    a.href = url;
    a.download = `sweep-${Date.now()}.csv`;
    a.click();
    URL.revokeObjectURL(url);
  };

  renderSweep();
</script>
</body>
</html>
)rawliteral";

// Called once per averaged sample set. Drives the sweep state machine and
// emits one point per throttle step.
void serviceSweep() {
  if (sweepPhase == SW_IDLE || sweepPhase == SW_DONE) return;

  unsigned long now = millis();

  if (sweepPhase == SW_SETTLE) {
    if (now - sweepPhaseStart >= sweepSettleMs) {
      sweepPhase = SW_AVG;
      sweepPhaseStart = now;
      sweepResetAccum();
    }
    return;
  }

  // SW_AVG - sweepAccumulate() fills the accumulators from the sample tick.
  // Hold until the window closes and at least two samples landed, otherwise
  // the spread is meaningless.
  if (now - sweepPhaseStart < sweepAvgMs || sAccN < 2) return;

  float mw = sAccW / sAccN;
  float mv = sAccV / sAccN;
  float ma = sAccA / sAccN;

  // Population variance. Clamp the small negative that float rounding can
  // produce when every sample in the window is identical.
  double var = sAccW2 / sAccN - (double)mw * mw;
  if (var < 0) var = 0;
  float sd = sqrt(var);

  float mp = mv * ma;
  float me = (mp > 0.01f) ? (mw / mp) : 0;

  char json[288];
  snprintf(json, sizeof(json),
    "{\"t\":\"pt\",\"throttle\":%d,\"weight\":%.2f,\"sd\":%.2f,\"n\":%d,"
    "\"volts\":%.2f,\"amps\":%.2f,\"power\":%.2f,\"efficiency\":%.2f}",
    sweepTarget, mw, sd, sAccN, mv, ma, mp, me);
  ws.textAll(json);

  sweepIndex++;

  if (sweepTarget >= sweepTo) {
    setThrottlePercent(0);
    sweepPhase = SW_DONE;
    sweepBroadcastState("done");
    return;
  }

  sweepTarget = min(sweepTarget + sweepStepPct, sweepTo);
  setThrottlePercent(sweepTarget);
  sweepPhase = SW_SETTLE;
  sweepPhaseStart = now;
}

void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type,
               void *arg, uint8_t *data, size_t len) {
  // One-way: server pushes readings, so no client messages need handling.
  // Safety failsafe: if the controlling page disconnects (tab closed, WiFi
  // drop, etc.), abort any running sweep and cut the throttle instead of
  // leaving the motor spinning with no one watching it.
  if (type == WS_EVT_DISCONNECT) {
    sweepPhase = SW_IDLE;
    setThrottlePercent(0);
  }
}

void handleSweepStart(AsyncWebServerRequest *request) {
  if (!escArmed) {
    request->send(503, "text/plain", "ESC still arming, try again in a moment");
    return;
  }
  if (sweepPhase == SW_SETTLE || sweepPhase == SW_AVG) {
    request->send(409, "text/plain", "sweep already running");
    return;
  }

  int from = request->hasParam("from") ? request->getParam("from")->value().toInt() : 0;
  int to   = request->hasParam("to")   ? request->getParam("to")->value().toInt()   : 100;
  int step = request->hasParam("step") ? request->getParam("step")->value().toInt() : 10;
  long settle = request->hasParam("settle") ? request->getParam("settle")->value().toInt() : 3000;
  long avg    = request->hasParam("avg")    ? request->getParam("avg")->value().toInt()    : 3000;

  from = constrain(from, 0, 100);
  to   = constrain(to, 0, 100);
  if (step < 1)  step = 1;
  if (step > 50) step = 50;
  if (to <= from) {
    request->send(400, "text/plain", "'to' must be greater than 'from'");
    return;
  }
  // Floors keep a runaway request from parking the motor at throttle for
  // minutes, and keep the averaging window wide enough to hold a sample.
  sweepSettleMs = constrain(settle, 500L, 30000L);
  sweepAvgMs    = constrain(avg, 1000L, 30000L);

  sweepFrom = from;
  sweepTo = to;
  sweepStepPct = step;
  sweepTotal = (to - from) / step + 1;
  if ((to - from) % step != 0) sweepTotal++;
  sweepIndex = 0;

  sweepTarget = sweepFrom;
  setThrottlePercent(sweepTarget);
  sweepPhase = SW_SETTLE;
  sweepPhaseStart = millis();

  char json[96];
  snprintf(json, sizeof(json), "{\"points\":%d}", sweepTotal);
  request->send(200, "application/json", json);
}

void handleSweepStop(AsyncWebServerRequest *request) {
  sweepAbort("aborted");
  setThrottlePercent(0);
  request->send(200, "text/plain", "OK");
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
  // Manual throttle wins over an automated sweep - this is also what the STOP
  // button hits, so it must not fight the sweep state machine.
  sweepAbort("aborted");
  setThrottlePercent(request->getParam("value")->value().toInt());
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

  // Power monitor. 64 hardware samples at 1.1 ms per channel = ~141 ms per
  // averaged result, which lands just under the 150 ms sample tick: every read
  // gets a fresh value, and the averaging window is long enough to swallow the
  // ESC's switching noise without smearing across throttle steps.
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  inaOk = ina226.init();
  if (inaOk) {
    ina226.setResistorRange(SHUNT_OHMS, MAX_CURRENT_A);
    ina226.setAverage(INA226_AVERAGE_64);
    ina226.setConversionTime(INA226_CONV_TIME_1100);
    ina226.setMeasureMode(INA226_CONTINUOUS);
    Serial.println("INA226 ready");
  } else {
    Serial.println("INA226 NOT FOUND - check I2C wiring (SDA 21, SCL 22) and 3V3 supply");
  }

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
  server.on("/throttle", HTTP_GET, handleSetThrottle);
  server.on("/sweep/start", HTTP_GET, handleSweepStart);
  server.on("/sweep/stop", HTTP_GET, handleSweepStop);

  server.begin();
}

void loop() {
  if (!escArmed && millis() - escAttachTime >= ESC_ARM_DELAY_MS) {
    escArmed = true;
  }

  if (millis() - lastSample >= SAMPLE_INTERVAL_MS) {
    lastSample = millis();

    sampleBuffer[sampleIndex] = getReading();

    if (inaOk) {
      voltsBuffer[sampleIndex] = ina226.getBusVoltage_V();
      ampsBuffer[sampleIndex]  = ina226.getCurrent_A();
    } else {
      voltsBuffer[sampleIndex] = 0;
      ampsBuffer[sampleIndex]  = 0;
    }

    // Feed the sweep the raw per-tick reading so its spread figure sees the
    // real scatter rather than the smoothed value the UI gets.
    sweepAccumulate(sampleBuffer[sampleIndex], voltsBuffer[sampleIndex], ampsBuffer[sampleIndex]);

    sampleIndex++;

    if (sampleIndex >= SAMPLES_PER_AVERAGE) {
      float sum = 0, voltSum = 0, ampSum = 0;
      for (int i = 0; i < SAMPLES_PER_AVERAGE; i++) {
        sum     += sampleBuffer[i];
        voltSum += voltsBuffer[i];
        ampSum  += ampsBuffer[i];
      }
      g_weight = sum / SAMPLES_PER_AVERAGE;
      g_volts  = voltSum / SAMPLES_PER_AVERAGE;
      g_amps   = ampSum / SAMPLES_PER_AVERAGE;
      sampleIndex = 0;

      // The INA226 is bidirectional, so at zero throttle the current reads a
      // few milliamps either side of zero. Clamp the tiny negative excursion so
      // the UI doesn't flicker between -0.01 and 0.01 A while idle.
      if (g_amps > -0.05f && g_amps < 0.05f) g_amps = 0;

      float power = g_volts * g_amps;                              // watts
      float efficiency = (power > 0.01f) ? (g_weight / power) : 0;  // grams-force per watt

      bool sweeping = (sweepPhase == SW_SETTLE || sweepPhase == SW_AVG);

      char json[352];
      snprintf(json, sizeof(json),
        "{\"t\":\"live\",\"weight\":%.2f,\"volts\":%.2f,\"amps\":%.2f,\"power\":%.2f,"
        "\"efficiency\":%.2f,\"throttle\":%d,\"inaOk\":%s,"
        "\"swActive\":%s,\"swIndex\":%d,\"swTotal\":%d,\"swPhase\":\"%s\"}",
        g_weight, g_volts, g_amps, power, efficiency, currentThrottlePercent,
        inaOk ? "true" : "false",
        sweeping ? "true" : "false", sweepIndex, sweepTotal,
        sweepPhase == SW_SETTLE ? "settle" : (sweepPhase == SW_AVG ? "average" : "idle"));

      ws.textAll(json);
      ws.cleanupClients();

      // Phase transitions only - the accumulators are filled per tick above.
      serviceSweep();
    }
  }
}
