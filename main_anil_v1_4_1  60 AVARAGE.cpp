/*
  ============================================================
  ESP32 PUMP / CONTACTOR CONTROLLER
  Hardware : WT32-ETH01 (LAN8720 PHY)
  Firebase : FirebaseESP32 by Mobizt — Service-Account / JWT
  OTA      : HTTP OTA over Ethernet — triggered by X11 command
  WiFi     : PERMANENTLY DISABLED at boot
  ============================================================

  GPIO MAP
  --------
  RELAY1 (PUMP)   → PIN 4   (Active-LOW)
  RELAY2          → PIN 14  (Active-LOW)
  RELAY3          → PIN 15  (Active-LOW)
  RELAY4          → PIN 12  (Active-LOW)
  FEEDBACK1       → PIN 2   (contactor feedback)
  FEEDBACK2       → PIN 35  (contactor feedback)
  PRESSURE (ADC)  → PIN 36

  ETHERNET (LAN8720)
  ------------------
  MDC  → 23 | MDIO → 18 | PHY_EN → 16 | CLK → GPIO17

  RELAY LOGIC : Active-LOW
    LOW  = ON  (energised)
    HIGH = OFF (de-energised)  ← safe / default state

  FIREBASE PATH MAP
  -----------------
  /device/commands/cmd        ← command string (X1–X16)
  /device/commands/timer      ← push timer for X9/X10 (seconds)
  /device/commands/data/      ← settings payload for X1
  /device/commands/ota_url    ← firmware .bin URL for X11 OTA
  /device/commands/status     ← ESP writes command result
  /device/status/             ← live: pressure, relays, contactor, mode
  /device/status/ota/         ← OTA progress and result
  /device/settings/           ← persisted settings
  /device/log/latest          ← per-second log line
  /device/alerts/             ← critical alerts

  COMMAND MAP
  -----------
  X1   Save settings; X2 round-trip; X3 → Mode 2; X4 restart; X17 → Mode 1;
  X5–X8   Relay 1–4 ON     |   X13–X16  Relay 1–4 OFF
  X9/X10  Pulse Relay 2/3 for N sec  (N = /commands/timer)
  X11  OTA from /commands/ota_url   |   X12  Update calibration table

  WEB LOG VIEWER
  --------------
  Open  http://<esp-ip>/  in any browser on the same LAN to see the
  same logs you'd see over USB serial, auto-refreshing every second.
  No USB cable required after first flash.

  OTA COMMAND (X11)
  -----------------
  1. Write firmware .bin URL to /device/commands/ota_url
  2. Write "X11" to /device/commands/cmd
  3. ESP downloads .bin over Ethernet and flashes it
  4. Progress reported to /device/status/ota/progress (0-100)
  5. On success → /device/status/ota/result = "OK" then restarts
  6. On failure → /device/status/ota/result = "FAILED: <reason>"

  PRESSURE CALIBRATION TABLE
  ----------------------------
  Edit the `calibTable[]` box near the top of the file before flashing.
  On first boot, those flashed values are saved into EEPROM (same as
  MINP/MAXP/etc.) and become the source of truth from then on — every
  later boot loads from EEPROM, not from the code. To change it later
  without reflashing, use the X12 command (below).

  CALIBRATION TABLE COMMAND (X12)
  --------------------------------
  Write an array of {"v":<volts>,"p":<value>} points to
  /device/commands/data, then write "X12" to /device/commands/cmd.
  The new table is saved to EEPROM (survives reboot/OTA) and mirrored
  to /device/settings/calib_table for confirmation.

  BOOT-TIME SETTINGS SYNC (automatic, no command needed)
  ----------------------------------------------------------
  Right after boot status is first uploaded (A7/A5), the ESP pauses
  everything for up to 25 seconds watching the server for an X1
  command. If one arrives, it downloads /device/commands/data and
  compares it field-by-field against the values already loaded from
  EEPROM: identical → no change; different → EEPROM/RAM updated to the
  latest immediately, before the device starts running its mode logic.
  If nothing arrives in 25s, boot continues with the EEPROM values
  already loaded.
  ============================================================
*/

// ============================================================
// INCLUDES
// ============================================================
#include <Arduino.h>
#include <EEPROM.h>
#include <ETH.h>
#include <FirebaseESP32.h>
#include <HTTPClient.h>
#include <Update.h>
#include <WebServer.h>      // built-in web log viewer on ESP's IP
#include <esp_wifi.h>       // for esp_wifi_stop() / esp_wifi_deinit()
#include <time.h>

// ============================================================
// FIRMWARE VERSION — shown in log and Firebase status
// ============================================================
#define FW_VERSION "1.4.1"

// ============================================================
// DEVICE IDENTITY — change ONE line per ESP before flashing.
//   ESP for Anil   -> "esp-anil"
//   ESP for Manish -> "esp-manish"
// All Firebase paths are derived from this; nothing else to change.
// ============================================================
#define DEVICE_ID  "esp-anil"
#define DEV_BASE   "/devices/" DEVICE_ID

// ============================================================
// FIREBASE CREDENTIALS — SERVICE ACCOUNT
// ============================================================
#define FIREBASE_HOST         "monk-miner-moniter-default-rtdb.firebaseio.com"
#define FIREBASE_PROJECT_ID   "monk-miner-moniter"
#define FIREBASE_CLIENT_EMAIL "firebase-adminsdk-fbsvc@monk-miner-moniter.iam.gserviceaccount.com"

const char FIREBASE_PRIVATE_KEY[] PROGMEM = R"EOF(-----BEGIN PRIVATE KEY-----
MIIEvgIBADANBgkqhkiG9w0BAQEFAASCBKgwggSkAgEAAoIBAQCurXXZcc0slO7M
sFO/WtVA94RexSQwUIHq3i5Er1Dw0wFUeCJKjHtpTxj/lpDLFxxdUKe6RykQHhDh
sIjnTzGvV+9eukANmkLp9R36WI1teXsQNLMXHwuMp8F13cJ7Z1HsvE9hkmodH0/0
ohSqf32VhbU/7zLgBUrUFlOTlK7yoOf6Z0Ox5HtYjCdiugcXSzIkuV0t2lca0h2b
V6uWb7Wtqx+lGc5iijAtjBTiBWKZjxEqiqUkoquW2jhqWqNlBuzMq2Fg1IgR9BJZ
4IcMVlMipW+sAzGS42KWI6/2nc95SHcdog4ZbLY6m+k9SMyS0c9DVcxtqCg5CQcY
1E8uPVPfAgMBAAECggEAEXHPUCz2UZp6EsUVSAn5PCi79n73sz+HCf6wXjmn+rUH
7/ijbBCFjqnQlE3kHfQr6AHTDl5m8m4eP3vS4E7fZmSAylQNa45ZWmYZXVQUo7c7
HA43SHrYO27RekTwVESoQN05jEQEBu6GnKbHK5PZ5eHZjQ4FZawBohasuBNkOWTA
Dotg+UHcSsh/x++3UZOnX+8ndjUBduHDwSOVOoGOzVgSpzUq2gmNWVJytk9Wa83W
q/V4COHwItiH6SZOhAhoeuP3BNItb+JtrdifI7hDHKIlMxKuCSD2qX9xO2FsA0T/
JToI9k2nEAAM0GZR0wiRv6sSZWWZWtXNp276/EsF4QKBgQDi30riUmfHyZGsaVhS
oO8iSmjZ8AKZBxEjsbRPc1ZKR8T1/fhVGmOU/QsdvJ0wKvw+zclVrCav2OLdvZtE
cQeVDbdJKqHSWMS/H/pNy+eJZw93Pmw+8UwsMtZOtYk1FS/klhaEBDMxkfooptTh
mMhs0JBGRNhUGz8zAOCBTHkrkQKBgQDFGqnfuZKfbcdqLWI0FE1laS03bMLUVlsP
T8pNi06QKzu0avEMa4h0Mqm4fse5Nm9dsZZnHuoqkdvd4ARrftC89fggeR057PoR
syPyISBBCLVOoF02Ji/DuiuElLgpZzmHc5cIDij/c1q6lfBojrFjEJpXAkjvSJKe
p6G6HmRwbwKBgQC36XlkBJdarv1nPbK6sPmJ27YpzdXdRYxQWjMoIQB3kLyLCYmx
O4Y0dfj37zmhnYcERoAK8lYeQPyP8q+WiOYzn33QUz4BLbK4mOyo+j3E9gXkjXbk
g2lTfxaZkbIblQRREpZICLuTWJnpMGzsQJVhGKWMQSz46WmPKAvW5S5o8QKBgQCF
p4b5hkS+hxpqDUxCNGInGiLnKoESq1wkDd8IpiYn7KkXtkyuN1zYLmKYEKAuH5N0
3S83zebL5wxIb9ePbPbuq3wNyRLgbKlFx0vgrEXK07TeFDAgv9QzzNIlnRMkQDLq
fp8zfyad0gL78yCKRfhTTLbbuVjrqTKDEvi/1EwXUQKBgGpPpXQ65tSD5PRlPSTy
p50u3EDtDkgqYPQhQFlBeINhnff3wqIe616nO/OO4P6V0KIaA8mmLD7LXoMzIEv7
vMLXw/aav//GipqUgNm7OUEqOGs/ldi18rgz+7+UQh0sMVG+wlKVcbVIM4fJOQpF
B8zUg4Z8ddHH6REOB3/4PVZm
-----END PRIVATE KEY-----)EOF";

// ============================================================
// ETHERNET CONFIG (WT32-ETH01 / LAN8720) — ESP32 core v3.x
// New ETH.begin() signature:
//   ETH.begin(type, phy_addr, mdc, mdio, power, clk_mode)
// ============================================================
#define ETH_PHY_TYPE   ETH_PHY_LAN8720          // eth_phy_type_t
#define ETH_PHY_ADDR   ((int32_t)1)              // cast to int32_t
#define ETH_PHY_MDC    23
#define ETH_PHY_MDIO   18
#define ETH_PHY_POWER  16
#define ETH_CLK_MODE   ETH_CLOCK_GPIO17_OUT      // eth_clock_mode_t

// ============================================================
// GPIO
// ============================================================
#define RELAY1_PIN    4
#define RELAY2_PIN    14
#define RELAY3_PIN    15
#define RELAY4_PIN    12
#define FEEDBACK1_PIN 39
#define FEEDBACK2_PIN 35
#define PRESSURE_PIN  36

// ============================================================
// PRESSURE TRANSDUCER + VOLTAGE DIVIDER
// Transducer: 0–5.5V output
// Divider   : R1 (top, transducer → ADC pin) + R2 (bottom, ADC pin → GND)
//             Vadc = Vtransducer * R2 / (R1 + R2)
// CONFIRMED WIRING: R1 = 20k (top), R2 = 30k (bottom)
//   → ratio = 30/(20+30) = 0.6
//   → 5.5V transducer max  → 3.3V at ADC pin (safe, <3.3V ESP32 limit)
//   → 5.0V transducer      → 3.0V at ADC pin
// If you ever re-wire or swap resistors, measure the ACTUAL resistors
// with a multimeter (or measure Vadc directly at a known transducer
// voltage) and update the values below to match.
// ============================================================
#define DIVIDER_R1_OHMS   20000.0f   // top resistor (transducer signal side)
#define DIVIDER_R2_OHMS   30000.0f   // bottom resistor (GND side)
#define DIVIDER_RATIO     (DIVIDER_R2_OHMS / (DIVIDER_R1_OHMS + DIVIDER_R2_OHMS))
#define TRANSDUCER_VMAX   5.5f       // transducer full-scale output voltage

// ============================================================
// PRESSURE CALIBRATION TABLE — VOLTAGE → PRESSURE LOOKUP
// ------------------------------------------------------------
// EDIT THIS TABLE to match your transducer's datasheet/calibration,
// then flash to the ESP32. Each row is {transducer_voltage, pressure}.
//   - transducer_voltage = volts AT THE TRANSDUCER (0.0–5.5), not at
//     the ADC pin — the divider is undone in code automatically.
//   - pressure            = whatever unit you use (bar, psi, %, etc.)
//   - Must be sorted by ascending voltage.
//   - Values between points are linearly interpolated.
//   - Up to CALIB_TABLE_MAX_POINTS rows; set calibTableCount to the
//     number of rows you actually fill in below.
// This same table can ALSO be overwritten at runtime via the X12
// command (no reflash needed) — see header comment + X12 handler.
// ============================================================
#define CALIB_TABLE_MAX_POINTS 20

struct CalibPoint {
  float voltage;   // transducer output volts, 0.0–5.5
  float pressure;  // corresponding pressure value
};

// ---- EDIT THIS TABLE TO CALIBRATE YOUR TRANSDUCER ----
CalibPoint calibTable[CALIB_TABLE_MAX_POINTS] = {
  {0.00f,    0.0f},
  {0.55f,   10.0f},
  {1.10f,   20.0f},
  {1.65f,   30.0f},
  {2.20f,   40.0f},
  {2.75f,   50.0f},
  {3.30f,   60.0f},
  {3.85f,   70.0f},
  {4.40f,   80.0f},
  {4.95f,   90.0f},
  {5.50f,  100.0f},
};
uint8_t calibTableCount = 11;   // number of valid rows above (≤ CALIB_TABLE_MAX_POINTS)

// ============================================================
// EEPROM
// ============================================================
#define ADDR_MODE         0
#define ADDR_SETTINGS_OK  1                            // 1 byte: 0xA5 = MINP/MAXP/RT/OP/OFP/ORT saved from runtime
#define ADDR_MINP         2
#define ADDR_MAXP         6
#define ADDR_RT           10
#define ADDR_OP           14
#define ADDR_OFP          18
#define ADDR_ORT          22
#define ADDR_CALIB_VALID  24                          // 1 byte: 0xA5 = table saved from runtime
#define ADDR_CALIB_COUNT  25                           // 1 byte: number of points saved
#define ADDR_CALIB_TABLE  26                           // CALIB_TABLE_MAX_POINTS * sizeof(CalibPoint)
#define EEPROM_SIZE       (ADDR_CALIB_TABLE + (CALIB_TABLE_MAX_POINTS * 8) + 4)

// ============================================================
// FIREBASE OBJECTS
// ============================================================
FirebaseData   fbdo;
FirebaseData   fbdoCmd;
FirebaseAuth   auth;
FirebaseConfig config;

// ============================================================
// GLOBAL STATE
// ============================================================
bool ethConnected  = false;
bool fbConnected   = false;

uint8_t  currentMode = 1;
uint16_t MINP  = 35;
uint16_t MAXP  = 60;
uint16_t RT    = 5000;   // not used by current control logic — reserved for future OTA update
uint16_t OP    = 3;
uint16_t OFP   = 3;
uint16_t ORT   = 3;      // not used by current control logic — reserved for future OTA update

float pressureVal    = 0.0;
float transducerVoltage = 0.0;   // recovered transducer-side voltage (after undoing the divider)
bool  relay1State    = false;
bool  relay2State    = false;
bool  relay3State    = false;
bool  relay4State    = false;
bool  contactorState = false;

unsigned long lastLogMillis = 0;

// ============================================================
// WEB LOG VIEWER — view ESP logs in a browser on the ESP's IP
// (no USB serial needed). Lives at http://<esp-ip>/  on port 80.
// /        → small HTML page that auto-refreshes /logs.txt
// /logs.txt → plain text dump of the last LOG_BUF_LINES lines
// ============================================================
#define LOG_BUF_LINES 240            // ring-buffer depth (≈ 4 minutes at 1 line/sec)
#define LOG_BUF_LINE_LEN 160         // max chars per line

static char     g_logBuf[LOG_BUF_LINES][LOG_BUF_LINE_LEN];
static uint16_t g_logHead = 0;       // next write slot (oldest-overwrite)
static uint16_t g_logCount = 0;      // total lines ever written (so the page can show order)
WebServer       webLog(80);

// Push a line into Serial AND into the ring buffer. Use this instead of
// Serial.println()/printf() for anything you want visible in the web log.
void logLine(const char* line) {
  Serial.println(line);
  strncpy(g_logBuf[g_logHead], line, LOG_BUF_LINE_LEN - 1);
  g_logBuf[g_logHead][LOG_BUF_LINE_LEN - 1] = '\0';
  g_logHead = (g_logHead + 1) % LOG_BUF_LINES;
  g_logCount++;
}

// printf-style variant
void logf(const char* fmt, ...) {
  char buf[LOG_BUF_LINE_LEN];
  va_list ap; va_start(ap, fmt);
  vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  logLine(buf);
}

void webLogHandleRoot() {
  // tiny self-refreshing page; the textarea fetches /logs.txt every 1s
  static const char html[] PROGMEM = R"HTML(<!doctype html><html><head>
<meta charset="utf-8"><title>ESP Log Viewer</title>
<style>
body{margin:0;background:#0b0e14;color:#cfd8e3;font:13px/1.45 'JetBrains Mono',ui-monospace,monospace}
header{padding:10px 14px;border-bottom:1px solid #1a2433;display:flex;justify-content:space-between;align-items:center}
header b{color:#ffd070;letter-spacing:2px}
header span{color:#5a6172;font-size:11px}
#log{padding:10px 14px;white-space:pre-wrap;height:calc(100vh - 56px);overflow:auto}
.row-fault{color:#ff6464}
.row-cmd{color:#ffe28a}
.row-ok{color:#7be7a1}
</style></head><body>
<header><b>ESP-ANIL · LIVE LOG</b><span id="meta">connecting…</span></header>
<div id="log">loading…</div>
<script>
let lastCount = 0, stick = true;
const log = document.getElementById('log');
log.addEventListener('scroll', () => {
  stick = (log.scrollTop + log.clientHeight) >= (log.scrollHeight - 20);
});
function fmt(line){
  if(/FAULT|ERROR|FAILED|CRITICAL|ALERT/i.test(line)) return '<span class="row-fault">'+line+'</span>';
  if(/\[A8\]|CMD:|X\d+/i.test(line)) return '<span class="row-cmd">'+line+'</span>';
  if(/\[A6\] Firebase OK|\[ETH\] IP:|Settings saved|UPDATE successful/i.test(line)) return '<span class="row-ok">'+line+'</span>';
  return line;
}
async function poll(){
  try{
    const r = await fetch('/logs.txt?since=' + lastCount, {cache:'no-store'});
    const t = await r.text();
    const hdr = r.headers.get('X-Log-Count');
    if(hdr) lastCount = parseInt(hdr, 10) || 0;
    document.getElementById('meta').textContent = 'lines: ' + lastCount + ' · auto-refresh 1s';
    log.innerHTML = t.split('\n').map(fmt).join('\n');
    if(stick) log.scrollTop = log.scrollHeight;
  }catch(e){
    document.getElementById('meta').textContent = 'reconnecting…';
  }
}
poll(); setInterval(poll, 1000);
</script></body></html>)HTML";
  webLog.send_P(200, "text/html", html);
}

void webLogHandleLogs() {
  // Dump entire ring buffer (oldest → newest) as plain text.
  // Add X-Log-Count header so the page knows current line count.
  String out;
  out.reserve(LOG_BUF_LINES * 40);
  uint16_t start = (g_logCount >= LOG_BUF_LINES) ? g_logHead : 0;
  uint16_t n     = (g_logCount >= LOG_BUF_LINES) ? LOG_BUF_LINES : g_logCount;
  for (uint16_t i = 0; i < n; i++) {
    uint16_t idx = (start + i) % LOG_BUF_LINES;
    out += g_logBuf[idx];
    out += '\n';
  }
  webLog.sendHeader("X-Log-Count", String(g_logCount));
  webLog.sendHeader("Cache-Control", "no-store");
  webLog.send(200, "text/plain; charset=utf-8", out);
}

void startWebLogServer() {
  webLog.on("/",         webLogHandleRoot);
  webLog.on("/logs.txt", webLogHandleLogs);
  webLog.onNotFound([](){
    webLog.send(404, "text/plain", "Not Found — try / or /logs.txt");
  });
  webLog.begin();
  logf("[WEB] Log viewer listening on http://%s/", ETH.localIP().toString().c_str());
}

// ============================================================
// WIFI — PERMANENTLY DISABLED
// Called once at the very start of setup() before anything else.
// Stops the WiFi radio at the hardware level so it never turns
// on again this session. ETH.begin() works independently of WiFi.
// ============================================================
void disableWiFiForever() {
  // Make sure the WiFi driver is initialised before we shut it down
  WiFi.mode(WIFI_OFF);          // software disable
  esp_wifi_stop();              // stop the radio
  esp_wifi_deinit();            // free resources — cannot be re-enabled
  btStop();                     // also kill Bluetooth to save power
  Serial.println("[WIFI] WiFi and Bluetooth permanently disabled");
}

// ============================================================
// RELAY HELPERS (Active-LOW)
// ============================================================
void relayOn(uint8_t pin)  { digitalWrite(pin, LOW);  }
void relayOff(uint8_t pin) { digitalWrite(pin, HIGH); }
void allRelaysOff() {
  relayOff(RELAY1_PIN);  relayOff(RELAY2_PIN);
  relayOff(RELAY3_PIN);  relayOff(RELAY4_PIN);
  relay1State = relay2State = relay3State = relay4State = false;
  // Push the new (false) state so dashboard stays in sync.
  // pushAllRelayStatus is safe even if !fbConnected.
  if (fbConnected) {
    Firebase.setBool(fbdo, DEV_BASE "/status/relay1", false);
    Firebase.setBool(fbdo, DEV_BASE "/status/relay2", false);
    Firebase.setBool(fbdo, DEV_BASE "/status/relay3", false);
    Firebase.setBool(fbdo, DEV_BASE "/status/relay4", false);
  }
}

// Forward decl — defined later (depends on fbConnected/fbdo).
void pushRelayStatus(uint8_t num);

// Single source of truth for changing a relay state — updates the pin,
// the in-memory bool, AND immediately publishes the new state to
// Firebase at /devices/<id>/status/relayN. The immediate push matters
// for perceived speed: without it, the dashboard toggle only learns
// about the change on the next uploadStatusBatch() tick (up to ~1s
// later), which feels noticeably laggy even though the physical relay
// already switched instantly. uploadStatusBatch() still republishes all
// 4 relays every second as a safety net (so the dashboard can never
// permanently drift out of sync even if this push is ever missed), but
// it's no longer the primary path for "the toggle flips visually".
void setRelay(uint8_t num, bool on) {
  switch (num) {
    case 1: if (on) relayOn(RELAY1_PIN); else relayOff(RELAY1_PIN); relay1State = on; break;
    case 2: if (on) relayOn(RELAY2_PIN); else relayOff(RELAY2_PIN); relay2State = on; break;
    case 3: if (on) relayOn(RELAY3_PIN); else relayOff(RELAY3_PIN); relay3State = on; break;
    case 4: if (on) relayOn(RELAY4_PIN); else relayOff(RELAY4_PIN); relay4State = on; break;
    default: return;
  }
  pushRelayStatus(num);
}

// ============================================================
// PRESSURE — RAW ADC, AVERAGED, LINEAR SCALE
// (Same proven-stable logic as the test sketch — calibration
// table and voltage-divider math removed; X12/table EEPROM code
// below is now unused dead code, left in place but harmless.)
// ============================================================
#define MAX_SENSOR_PRESSURE 100.0f   // match your sensor's max rating
#define RAW_ADC_CLIP_MAX    3723     // same 0V-3V physical clip used in the stable sketch

// Number of quick ADC samples averaged per reading, and the gap
// between them. 10 samples * 1ms = ~10ms added per call — negligible,
// well under the 0.7s ceiling — but smooths out ADC noise nicely.
#define PRESSURE_AVG_SAMPLES 80
#define PRESSURE_AVG_DELAY_MS 1

float readPressure() {
  // Take several quick raw ADC samples and average them (matches the
  // stable test sketch's analogRead() approach, just averaged).
  long rawSum = 0;
  for (int i = 0; i < PRESSURE_AVG_SAMPLES; i++) {
    rawSum += analogRead(PRESSURE_PIN);
    delay(PRESSURE_AVG_DELAY_MS);
  }
  int rawADC = rawSum / PRESSURE_AVG_SAMPLES;

  if (rawADC < 0) rawADC = 0;
  if (rawADC > RAW_ADC_CLIP_MAX) rawADC = RAW_ADC_CLIP_MAX;

  // Diagnostic-only: raw ADC pin voltage (not used in the pressure calc)
  transducerVoltage = rawADC * 3.3f / 4095.0f;

  return (rawADC / (float)RAW_ADC_CLIP_MAX) * MAX_SENSOR_PRESSURE;
}

// ============================================================
// CALIBRATION TABLE — EEPROM SAVE / LOAD
// If no table was ever saved at runtime, the hardcoded calibTable[]
// above (set at flash time) is used as-is.
// ============================================================
void saveCalibTableToEEPROM() {
  EEPROM.write(ADDR_CALIB_VALID, 0xA5);
  EEPROM.write(ADDR_CALIB_COUNT, calibTableCount);
  int addr = ADDR_CALIB_TABLE;
  for (uint8_t i = 0; i < calibTableCount; i++) {
    EEPROM.put(addr, calibTable[i]);
    addr += sizeof(CalibPoint);
  }
  EEPROM.commit();
  Serial.printf("[CALIB] Table saved to EEPROM (%d points)\n", calibTableCount);
}

void loadCalibTableFromEEPROM() {
  if (EEPROM.read(ADDR_CALIB_VALID) != 0xA5) {
    // First boot (or fresh chip) — no table saved yet.
    // Take whatever is hand-written in the calibTable[] box above
    // (set at flash time) and persist it to EEPROM right now, so
    // EEPROM becomes the source of truth from this boot onward —
    // exactly like MINP/MAXP/RT/OP/OFP/ORT already work.
    Serial.println("[CALIB] No saved table in EEPROM — saving flashed box values now");
    saveCalibTableToEEPROM();
    return;
  }
  uint8_t count = EEPROM.read(ADDR_CALIB_COUNT);
  if (count < 2 || count > CALIB_TABLE_MAX_POINTS) {
    Serial.println("[CALIB] Invalid count in EEPROM — using flashed defaults");
    return;
  }
  int addr = ADDR_CALIB_TABLE;
  for (uint8_t i = 0; i < count; i++) {
    EEPROM.get(addr, calibTable[i]);
    addr += sizeof(CalibPoint);
  }
  calibTableCount = count;
  Serial.printf("[CALIB] Table loaded from EEPROM (%d points)\n", calibTableCount);
}

// ============================================================
// CALIBRATION TABLE — UPLOAD ACTIVE TABLE TO FIREBASE (for confirmation)
// ============================================================
void uploadCalibTableToFirebase() {
  if (!fbConnected) return;
  FirebaseJsonArray arr;
  for (uint8_t i = 0; i < calibTableCount; i++) {
    FirebaseJson obj;
    obj.set("v", calibTable[i].voltage);
    obj.set("p", calibTable[i].pressure);
    arr.add(obj);
  }
  Firebase.setArray(fbdo, DEV_BASE "/settings/calib_table", arr);
  Serial.println("[CALIB] Table uploaded to Firebase");
}

// ============================================================
// FEEDBACK PINS
// ============================================================
bool feedbackHigh() {
  return (digitalRead(FEEDBACK1_PIN) == HIGH) &&
         (digitalRead(FEEDBACK2_PIN) == HIGH);
}
bool feedbackLow() {
  return (digitalRead(FEEDBACK1_PIN) == LOW) &&
         (digitalRead(FEEDBACK2_PIN) == LOW);
}

// ============================================================
// EEPROM HELPERS
// ============================================================
void eepromWriteUint16(int addr, uint16_t val) {
  EEPROM.write(addr,     (val >> 8) & 0xFF);
  EEPROM.write(addr + 1,  val       & 0xFF);
  EEPROM.commit();
}
uint16_t eepromReadUint16(int addr) {
  return ((uint16_t)EEPROM.read(addr) << 8) | EEPROM.read(addr + 1);
}

// ============================================================
// FIREBASE HELPERS
// ============================================================
void fbAlert(const String& msg) {
  if (!fbConnected) return;
  FirebaseJson j;
  j.set("message", msg);
  j.set("ts/.sv", "timestamp");
  Firebase.setJSON(fbdo, DEV_BASE "/alerts", j);
  Serial.println("[ALERT] " + msg);
}

void fbCmdStatus(const String& msg) {
  if (!fbConnected) return;
  Firebase.setString(fbdo, DEV_BASE "/commands/status", msg);
}

// ============================================================
// Per-relay live status push to Firebase.
// Called automatically by setRelay() on every state change (this is
// the fast path that makes the dashboard toggle feel instant), and
// also once at boot via pushAllRelayStatus() for the initial snapshot.
// uploadStatusBatch() additionally republishes all 4 relays every
// second as a safety net so the dashboard can never permanently drift
// out of sync even if an individual push above is ever missed.
// ============================================================
void pushRelayStatus(uint8_t num) {
  if (!fbConnected) return;
  String path = String(DEV_BASE) + "/status/relay" + String(num);
  bool state = false;
  switch (num) {
    case 1: state = relay1State; break;
    case 2: state = relay2State; break;
    case 3: state = relay3State; break;
    case 4: state = relay4State; break;
    default: return;
  }
  Firebase.setBool(fbdo, path.c_str(), state);
}
void pushAllRelayStatus() {
  for (uint8_t i = 1; i <= 4; i++) pushRelayStatus(i);
}

// ============================================================
// PER-SECOND LOG LINE (Serial + web buffer + Firebase)
// ============================================================
void printLogLine() {
  // Build once, use everywhere
  char line[LOG_BUF_LINE_LEN];
  snprintf(line, sizeof(line),
    "[LOG] FW:%s ETH:%s FB:%s P:%.1f Vt:%.3fV R1:%s R2:%s R3:%s R4:%s CTR:%s Mode:%d",
    FW_VERSION,
    ethConnected  ? "UP"  : "DOWN",
    fbConnected   ? "UP"  : "DOWN",
    pressureVal,
    transducerVoltage,
    relay1State   ? "ON"  : "OFF",
    relay2State   ? "ON"  : "OFF",
    relay3State   ? "ON"  : "OFF",
    relay4State   ? "ON"  : "OFF",
    contactorState? "ON"  : "OFF",
    currentMode
  );
  logLine(line);    // Serial + web log buffer

  if (!fbConnected) return;
  FirebaseJson j;
  j.set("fw",        FW_VERSION);
  j.set("eth",       ethConnected  ? "UP"  : "DOWN");
  j.set("firebase",  fbConnected   ? "UP"  : "DOWN");
  j.set("pressure",  pressureVal);
  j.set("relay1",    relay1State   ? "ON"  : "OFF");
  j.set("relay2",    relay2State   ? "ON"  : "OFF");
  j.set("relay3",    relay3State   ? "ON"  : "OFF");
  j.set("relay4",    relay4State   ? "ON"  : "OFF");
  j.set("contactor", contactorState? "ON"  : "OFF");
  j.set("mode",      (int)currentMode);
  j.set("ts/.sv",    "timestamp");
  Firebase.setJSON(fbdo, DEV_BASE "/log/latest", j);
}

// ============================================================
// FAST SERIAL-ONLY PRESSURE DEBUG LOG (~3x/sec)
// Independent of the 1-second Firebase/safety cadence above —
// this does its own quick averaged read and prints to Serial
// only (no Firebase writes, no effect on H1 safety monitoring).
// ============================================================
void printFastPressureDebug() {
  float p = readPressure(); // includes its own averaging
  Serial.printf("[DEBUG] Time: %.2fs | Pressure: %.2f | Vt: %.3fV\n",
                millis() / 1000.0f, p, transducerVoltage);
}

// ============================================================
// F1 — RESTART
// ============================================================
void F1_restart(const String& reason = "") {
  Serial.println("[F1] RESTART — " + reason);
  if (fbConnected && reason.length()) fbAlert("RESTART: " + reason);
  delay(500);
  ESP.restart();
}

// ============================================================
// S1 — SAVE SETTINGS FROM JSON TO EEPROM
// ============================================================
void S1_saveSettings(FirebaseJson& json) {
  FirebaseJsonData d;
  if (json.get(d, "mode")) { currentMode = d.intValue; EEPROM.write(ADDR_MODE, currentMode); }
  if (json.get(d, "minp")) { MINP = d.intValue; eepromWriteUint16(ADDR_MINP, MINP); }
  if (json.get(d, "maxp")) { MAXP = d.intValue; eepromWriteUint16(ADDR_MAXP, MAXP); }
  if (json.get(d, "rt"))   { RT   = d.intValue; eepromWriteUint16(ADDR_RT,   RT);   }
  if (json.get(d, "op"))   { OP   = d.intValue; eepromWriteUint16(ADDR_OP,   OP);   }
  if (json.get(d, "ofp"))  { OFP  = d.intValue; eepromWriteUint16(ADDR_OFP,  OFP);  }
  if (json.get(d, "ort"))  { ORT  = d.intValue; eepromWriteUint16(ADDR_ORT,  ORT);  }
  EEPROM.commit();
  Serial.println("[S1] Settings saved");
}

// ============================================================
// T1 — UPLOAD CURRENT SETTINGS TO FIREBASE
// ============================================================
void T1_sendSavedData() {
  if (!fbConnected) return;
  FirebaseJson j;
  j.set("mode", (int)currentMode);
  j.set("minp", (int)MINP);
  j.set("maxp", (int)MAXP);
  j.set("rt",   (int)RT);
  j.set("op",   (int)OP);
  j.set("ofp",  (int)OFP);
  j.set("ort",  (int)ORT);
  Firebase.setJSON(fbdo, DEV_BASE "/settings", j);
  Serial.println("[T1] Settings uploaded");
}

// ============================================================
// S2 — FETCH SETTINGS FROM FIREBASE → SAVE TO EEPROM
// ============================================================
void S2_fetchSettings() {
  if (!fbConnected) return;
  if (Firebase.getJSON(fbdo, DEV_BASE "/settings")) {
    FirebaseJson& j = fbdo.jsonObject();
    S1_saveSettings(j);
    Serial.println("[S2] Settings fetched from Firebase");
  } else {
    Serial.println("[S2] Failed: " + fbdo.errorReason());
  }
}

void T3_sendDataToServer() { T1_sendSavedData(); Serial.println("[T3] Done"); }

void T4_switchToMode2() {
  currentMode = 2;
  EEPROM.write(ADDR_MODE, 2); EEPROM.commit();
  if (fbConnected) {
    Firebase.setInt   (fbdo, DEV_BASE "/status/mode", 2);
    Firebase.setString(fbdo, DEV_BASE "/status/fw",   FW_VERSION);
  }
  Serial.println("[T4] Mode 2");
}

// ============================================================
// T5 — SWITCH TO MODE 1 (atomic counterpart of T4)
// Saves mode=1 to EEPROM, commits, then restarts so Mode 1
// (runMode1) starts cleanly from setup(). No race possible.
// ============================================================
void T5_switchToMode1() {
  currentMode = 1;
  EEPROM.write(ADDR_MODE, 1); EEPROM.commit();
  if (fbConnected) Firebase.setInt(fbdo, DEV_BASE "/status/mode", 1);
  fbCmdStatus("X17: Switched to Mode 1 — restarting");
  Serial.println("[T5] Switched to Mode 1 — restarting");
  delay(500);          // give Firebase writes time to flush
  F1_restart("CMD X17 — switch to Mode 1");
}

void S3_fetchSavedData() { S2_fetchSettings(); Serial.println("[S3] Done"); }

// ============================================================
// A5 — UPLOAD PRESSURE
// ============================================================
void A5_uploadPressure() {
  pressureVal = readPressure();
  if (fbConnected) {
    Firebase.setFloat(fbdo, DEV_BASE "/status/pressure", pressureVal);
    Firebase.setFloat(fbdo, DEV_BASE "/status/transducer_v", transducerVoltage);
  }
}

// ============================================================
// A7 — UPLOAD CONTACTOR STATUS
// Publishes TWO independent booleans:
//   /status/contactor1  (HIGH on FEEDBACK1_PIN / GPIO 39 = "ONLINE")
//   /status/contactor2  (HIGH on FEEDBACK2_PIN / GPIO 35 = "ONLINE")
// Legacy /status/contactor (both HIGH) is kept for backward compat.
// ============================================================
void A7_uploadContactorStatus() {
  bool c1 = (digitalRead(FEEDBACK1_PIN) == HIGH);
  bool c2 = (digitalRead(FEEDBACK2_PIN) == HIGH);
  contactorState = c1 && c2;
  if (fbConnected) {
    Firebase.setBool(fbdo, DEV_BASE "/status/contactor1", c1);
    Firebase.setBool(fbdo, DEV_BASE "/status/contactor2", c2);
    Firebase.setBool(fbdo, DEV_BASE "/status/contactor",  contactorState);
  }
}

// ============================================================
// A5+A7+RELAYS — BATCHED PER-SECOND STATUS UPLOAD (v1.4.0)
// Combines pressure, transducer voltage, both contactor feedback
// pins, the legacy combined contactor flag, and all 4 relay
// states into ONE Firebase.updateNode() PATCH call instead of
// the 8 separate blocking Firebase.set...() calls this used to
// take (A5_uploadPressure x2 + A7_uploadContactorStatus x3 +
// pushAllRelayStatus x4). Each blocking call costs a full
// request/response round trip over Ethernet; doing 8 of them
// back-to-back every "1 second" tick was actually stretching
// the real tick time out to ~8-10 seconds, which is why the
// dashboard only seemed to refresh every ~10 sec even though
// the loop timer said 1000ms. This collapses it to 1 call.
// ============================================================
void uploadStatusBatch() {
  pressureVal = readPressure();

  bool c1 = (digitalRead(FEEDBACK1_PIN) == HIGH);
  bool c2 = (digitalRead(FEEDBACK2_PIN) == HIGH);
  contactorState = c1 && c2;

  if (!fbConnected) return;

  FirebaseJson j;
  j.set("pressure",     pressureVal);
  j.set("transducer_v", transducerVoltage);
  j.set("contactor1",   c1);
  j.set("contactor2",   c2);
  j.set("contactor",    contactorState);
  j.set("relay1",       relay1State);
  j.set("relay2",       relay2State);
  j.set("relay3",       relay3State);
  j.set("relay4",       relay4State);

  if (!Firebase.updateNode(fbdo, DEV_BASE "/status", j)) {
    // Single retry on failure — still 1 extra call at worst, not 8.
    Firebase.updateNode(fbdo, DEV_BASE "/status", j);
  }
}

// ============================================================
// X11 OTA — DOWNLOAD AND FLASH FIRMWARE OVER ETHERNET
// URL is read from /device/commands/ota_url
// Progress reported to /device/status/ota/
// ============================================================
void X11_performOTA() {
  Serial.println("[OTA] Starting OTA update...");
  fbCmdStatus("OTA: Starting...");

  // Read URL from Firebase
  String url = "";
  if (Firebase.getString(fbdoCmd, DEV_BASE "/commands/ota_url")) {
    url = fbdoCmd.stringData();
  }
  if (url.length() == 0) {
    String err = "OTA: No URL in /device/commands/ota_url";
    Serial.println("[OTA] " + err);
    fbCmdStatus(err);
    Firebase.setString(fbdo, DEV_BASE "/status/ota/result", "FAILED: no URL");
    return;
  }

  Serial.println("[OTA] URL: " + url);
  Firebase.setString(fbdo, DEV_BASE "/status/ota/result",   "IN_PROGRESS");
  Firebase.setInt(fbdo,    DEV_BASE "/status/ota/progress", 0);
  Firebase.setString(fbdo, DEV_BASE "/status/ota/url",       url);

  // Pause relays to safe state before OTA
  allRelaysOff();
  fbAlert("OTA started — all relays set to OFF for safety");

  HTTPClient http;
  http.begin(url);
  http.setTimeout(30000); // 30 sec timeout per chunk

  int httpCode = http.GET();
  if (httpCode != HTTP_CODE_OK) {
    String err = "OTA: HTTP error " + String(httpCode);
    Serial.println("[OTA] " + err);
    fbCmdStatus(err);
    Firebase.setString(fbdo, DEV_BASE "/status/ota/result", "FAILED: " + err);
    http.end();
    return;
  }

  int totalSize = http.getSize();
  Serial.printf("[OTA] Firmware size: %d bytes\n", totalSize);

  if (!Update.begin(totalSize > 0 ? totalSize : UPDATE_SIZE_UNKNOWN)) {
    String err = "OTA: Update.begin failed — " + String(Update.errorString());
    Serial.println("[OTA] " + err);
    Firebase.setString(fbdo, DEV_BASE "/status/ota/result", "FAILED: " + err);
    http.end();
    return;
  }

  WiFiClient* stream = http.getStreamPtr();
  uint8_t  buf[1024];
  int      written   = 0;
  int      lastPct   = -1;

  while (http.connected() && (totalSize == -1 || written < totalSize)) {
    int available = stream->available();
    if (available > 0) {
      int toRead = min(available, (int)sizeof(buf));
      int read   = stream->readBytes(buf, toRead);
      if (Update.write(buf, read) != (size_t)read) {
        String err = "OTA: Write error — " + String(Update.errorString());
        Serial.println("[OTA] " + err);
        Firebase.setString(fbdo, DEV_BASE "/status/ota/result", "FAILED: " + err);
        http.end();
        return;
      }
      written += read;

      // Report progress every 5%
      if (totalSize > 0) {
        int pct = (written * 100) / totalSize;
        if (pct != lastPct && pct % 5 == 0) {
          lastPct = pct;
          Serial.printf("[OTA] Progress: %d%%\n", pct);
          Firebase.setInt(fbdo, DEV_BASE "/status/ota/progress", pct);
        }
      }
    } else {
      delay(1);
    }
  }

  http.end();

  if (Update.end(true)) {
    Serial.println("[OTA] Update successful! Rebooting...");
    Firebase.setInt(fbdo,    DEV_BASE "/status/ota/progress", 100);
    Firebase.setString(fbdo, DEV_BASE "/status/ota/result",   "OK");
    fbAlert("OTA successful — rebooting to FW " + url);
    delay(1000);
    ESP.restart();
  } else {
    String err = String(Update.errorString());
    Serial.println("[OTA] Update.end failed: " + err);
    Firebase.setString(fbdo, DEV_BASE "/status/ota/result", "FAILED: " + err);
    fbCmdStatus("OTA FAILED: " + err);
  }
}

// ============================================================
// X12 — UPDATE CALIBRATION TABLE FROM FIREBASE (no reflash needed)
// Expects /device/commands/data = [ {"v":0.0,"p":0.0}, {"v":0.55,"p":10.0}, ... ]
// sorted by ascending "v". Saves to EEPROM so it survives reboot/OTA.
// ============================================================
void X12_updateCalibTable() {
  if (!Firebase.getArray(fbdoCmd, DEV_BASE "/commands/data")) {
    fbCmdStatus("X12: Failed to read table — " + fbdoCmd.errorReason());
    Serial.println("[X12] " + fbdoCmd.errorReason());
    return;
  }
  FirebaseJsonArray& arr = fbdoCmd.jsonArray();
  int n = arr.size();
  if (n < 2) { fbCmdStatus("X12: Need at least 2 points"); return; }
  if (n > CALIB_TABLE_MAX_POINTS) n = CALIB_TABLE_MAX_POINTS;

  CalibPoint newTable[CALIB_TABLE_MAX_POINTS];
  FirebaseJsonData elemData, vd, pd;
  for (int i = 0; i < n; i++) {
    arr.get(elemData, i);
    FirebaseJson obj;
    if (!elemData.getJSON(obj)) { fbCmdStatus("X12: Bad point at index " + String(i)); return; }
    obj.get(vd, "v");
    obj.get(pd, "p");
    newTable[i].voltage  = vd.floatValue;
    newTable[i].pressure = pd.floatValue;
  }

  // Validate ascending voltage order before committing
  for (int i = 1; i < n; i++) {
    if (newTable[i].voltage < newTable[i - 1].voltage) {
      fbCmdStatus("X12: Rejected — voltages must be ascending");
      return;
    }
  }

  for (int i = 0; i < n; i++) calibTable[i] = newTable[i];
  calibTableCount = n;
  saveCalibTableToEEPROM();
  uploadCalibTableToFirebase();
  fbCmdStatus("X12: Calibration table updated (" + String(n) + " points)");
  Serial.printf("[X12] Calibration table updated (%d points)\n", n);
}

// ============================================================
// A9 — BOOT-TIME SETTINGS SYNC CHECK (blocking, up to 10 seconds)
// Called once in setup(), right after the A7/A5 status upload and
// BEFORE the device proceeds to load its mode / enter Mode 1's loop
// or loop(). Everything else is paused while this runs.
//
// Polls /device/commands/cmd every 500ms, same as A8, but only acts
// on "X1". If X1 arrives within the 10s window:
//   - downloads /device/commands/data (the A1 settings payload)
//   - compares every field against the values already loaded from
//     EEPROM in A1_loadEEPROM()
//   - if identical            → does nothing, logs "already up to date"
//   - if anything differs     → saves the new values (S1_saveSettings,
//     same as a normal X1) so EEPROM + RAM end up on the latest data
// If no X1 shows up within 10s, boot continues using whatever was
// already loaded from EEPROM.
// ============================================================
bool A9_settingsDiffer(FirebaseJson& json) {
  FirebaseJsonData d;
  if (json.get(d, "mode") && (int)currentMode != d.intValue) return true;
  if (json.get(d, "minp") && (int)MINP        != d.intValue) return true;
  if (json.get(d, "maxp") && (int)MAXP        != d.intValue) return true;
  if (json.get(d, "rt")   && (int)RT          != d.intValue) return true;
  if (json.get(d, "op")   && (int)OP          != d.intValue) return true;
  if (json.get(d, "ofp")  && (int)OFP         != d.intValue) return true;
  if (json.get(d, "ort")  && (int)ORT         != d.intValue) return true;
  return false;
}

void A9_bootSyncCheck() {
  if (!fbConnected) {
    logLine("[A9] Firebase not connected — skipping boot settings sync");
    return;
  }
  logLine("[A9] Boot sync: watching for X1 (up to 10s)...");
  fbCmdStatus("Boot sync: waiting up to 10s for X1");

  unsigned long startT = millis();
  unsigned long lastTick = 0;
  while (millis() - startT < 10000) {
    // Heartbeat every second so user sees activity on web/serial log
    if (millis() - lastTick >= 1000) {
      lastTick = millis();
      logf("[A9] Waiting for X1… %lus elapsed", (millis() - startT) / 1000);
    }
    if (ethConnected) webLog.handleClient();
    if (Firebase.getString(fbdoCmd, DEV_BASE "/commands/cmd")) {
      String cmd = fbdoCmd.stringData();
      if (cmd == "X1") {
        logLine("[A9] X1 found on server");
        Firebase.setString(fbdoCmd, DEV_BASE "/commands/cmd", "none"); // consume — A8 won't re-run it

        if (Firebase.getJSON(fbdoCmd, DEV_BASE "/commands/data")) {
          FirebaseJson& j = fbdoCmd.jsonObject();
          if (A9_settingsDiffer(j)) {
            logLine("[A9] Settings differ from EEPROM — updating to latest");
            S1_saveSettings(j);
            T1_sendSavedData();
            fbCmdStatus("Boot sync: X1 applied — settings updated to latest");
          } else {
            logLine("[A9] Settings match EEPROM — no update needed");
            fbCmdStatus("Boot sync: X1 received — already up to date, no change");
          }
        } else {
          logLine("[A9] X1 seen but no /commands/data payload");
          fbCmdStatus("Boot sync: X1 received but no data payload");
        }
        return; // handled — stop waiting, continue boot
      }
    }
    delay(200);
  }
  logLine("[A9] No X1 within 10s — continuing with EEPROM values");
  fbCmdStatus("Boot sync: no X1 in 10s — using saved settings");
}

// ============================================================
// A8 — COMMAND HANDLER (polled every 500 ms)
// ============================================================
void A8_handleCommand() {
  if (!fbConnected) return;
  if (!Firebase.getString(fbdoCmd, DEV_BASE "/commands/cmd")) return;
  String cmd = fbdoCmd.stringData();
  if (cmd.length() == 0 || cmd == "none") return;

  Serial.println("[A8] CMD: " + cmd);
  // Clear immediately to prevent re-execution
  Firebase.setString(fbdoCmd, DEV_BASE "/commands/cmd", "none");

  if (cmd == "X1") {
    if (Firebase.getJSON(fbdoCmd, DEV_BASE "/commands/data")) {
      FirebaseJson& j = fbdoCmd.jsonObject();
      S1_saveSettings(j);
      T1_sendSavedData();
      fbCmdStatus("X1: Settings saved & confirmed");
    }
  }
  else if (cmd == "X2")  { S2_fetchSettings(); T3_sendDataToServer(); fbCmdStatus("X2: Done"); }
  else if (cmd == "X3")  { S3_fetchSavedData(); T4_switchToMode2();   fbCmdStatus("X3: Done"); }
  else if (cmd == "X17") { T5_switchToMode1(); /* never returns — ESP restarts */ }
  else if (cmd == "X4")  { fbCmdStatus("X4: Restarting..."); delay(300); F1_restart("CMD X4"); }
  else if (cmd == "X5")  { setRelay(1, true);  }
  else if (cmd == "X6")  { setRelay(2, true);  }
  else if (cmd == "X7")  { setRelay(3, true);  }
  else if (cmd == "X8")  { setRelay(4, true);  }
  // X13-X16: relay OFF (manual). Counterparts to X5-X8.
  else if (cmd == "X13") { setRelay(1, false); }
  else if (cmd == "X14") { setRelay(2, false); }
  else if (cmd == "X15") { setRelay(3, false); }
  else if (cmd == "X16") { setRelay(4, false); }
  else if (cmd == "X9") {
    int t = 2;
    if (Firebase.getInt(fbdoCmd, DEV_BASE "/commands/timer")) t = fbdoCmd.intData();
    setRelay(2, true);
    fbCmdStatus("X9: Relay2 ON for " + String(t) + "s");
    delay(t * 1000UL);
    setRelay(2, false);
    fbCmdStatus("X9: Relay2 pulse done");
  }
  else if (cmd == "X10") {
    int t = 2;
    if (Firebase.getInt(fbdoCmd, DEV_BASE "/commands/timer")) t = fbdoCmd.intData();
    setRelay(3, true);
    fbCmdStatus("X10: Relay3 ON for " + String(t) + "s");
    delay(t * 1000UL);
    setRelay(3, false);
    fbCmdStatus("X10: Relay3 pulse done");
  }
  else if (cmd == "X11") {
    // OTA UPDATE
    X11_performOTA();
  }
  else if (cmd == "X12") {
    // UPDATE PRESSURE CALIBRATION TABLE (no reflash needed)
    X12_updateCalibTable();
  }
  else {
    fbCmdStatus("Unknown cmd: " + cmd);
    Serial.println("[A8] Unknown: " + cmd);
  }
}

// ============================================================
// ETH EVENT HANDLER
// ============================================================
void WiFiEvent(arduino_event_id_t event) {
  switch (event) {
    case ARDUINO_EVENT_ETH_START:
      Serial.println("[ETH] Started");
      ETH.setHostname("esp32-controller");
      break;
    case ARDUINO_EVENT_ETH_CONNECTED:
      Serial.println("[ETH] Cable OK");
      break;
    case ARDUINO_EVENT_ETH_GOT_IP:
      Serial.printf("[ETH] IP: %s  Speed: %dMbps  %s-duplex\n",
        ETH.localIP().toString().c_str(),
        ETH.linkSpeed(),
        ETH.fullDuplex() ? "Full" : "Half");
      ethConnected = true;
      break;
    case ARDUINO_EVENT_ETH_DISCONNECTED:
      Serial.println("[ETH] Disconnected");
      ethConnected = false; fbConnected = false;
      break;
    case ARDUINO_EVENT_ETH_STOP:
      Serial.println("[ETH] Stopped");
      ethConnected = false; fbConnected = false;
      break;
    default: break;
  }
}

// ============================================================
// A6 — CONNECT TO FIREBASE
// ============================================================
// ============================================================
// A6 — START ETHERNET + CONNECT TO FIREBASE
// Nothing network/Ethernet-related happens before this step runs.
// A6 owns the full network bring-up: register ETH event handler,
// start the PHY, wait for link, then authenticate to Firebase.
// ============================================================
bool A6_connectToServer() {
  // ── Bring up Ethernet (only happens here, on first A6 call) ──
  static bool ethStarted = false;
  if (!ethStarted) {
    ethStarted = true;
    Network.onEvent(WiFiEvent); // must register before ETH.begin()
    ETH.begin(ETH_PHY_TYPE, ETH_PHY_ADDR, ETH_PHY_MDC,
              ETH_PHY_MDIO, ETH_PHY_POWER, ETH_CLK_MODE);

    Serial.print("[A6] Waiting for Ethernet link");
    unsigned long ethWait = millis();
    while (!ethConnected && millis() - ethWait < 10000) { delay(500); Serial.print("."); }
    Serial.println(ethConnected ? " OK" : " TIMEOUT");
  }
  if (!ethConnected) { Serial.println("[A6] No Ethernet link — cannot connect Firebase"); return false; }

  Serial.println("[A6] Connecting Firebase...");

  config.database_url  = FIREBASE_HOST;
  config.service_account.data.client_email = FIREBASE_CLIENT_EMAIL;
  config.service_account.data.project_id   = FIREBASE_PROJECT_ID;
  config.service_account.data.private_key  = FIREBASE_PRIVATE_KEY;

  // NTP required for JWT signing
  configTime(0, 0, "pool.ntp.org", "time.nist.gov");
  Serial.print("[A6] NTP sync");
  time_t now = 0; int ntpTry = 0;
  while (now < 1000000000UL && ntpTry++ < 30) { delay(500); time(&now); Serial.print("."); }
  Serial.println();

  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(false); // WiFi is dead — ETH only

  Serial.print("[A6] Auth token");
  int tkTry = 0;
  while (!Firebase.ready() && tkTry++ < 40) { delay(500); Serial.print("."); Firebase.ready(); }
  Serial.println();

  if (!Firebase.ready()) { Serial.println("[A6] Failed"); return false; }

  fbConnected = true;
  Serial.println("[A6] Firebase OK");

  // Announce online with firmware version
  FirebaseJson boot;
  boot.set("online",  true);
  boot.set("mode",    (int)currentMode);
  boot.set("fw",      FW_VERSION);
  boot.set("ts/.sv",  "timestamp");
  Firebase.setJSON(fbdo, DEV_BASE "/status", boot);
  return true;
}

// ============================================================
// C3 — RETRY CONNECTION (50 × 5 sec)
// ============================================================
void C3_retryConnection() {
  Serial.println("[C3] Retry 50×5s...");
  for (int i = 1; i <= 50; i++) {
    Serial.printf("[C3] Attempt %d/50\n", i);
    if (ethConnected && A6_connectToServer()) { fbConnected = true; return; }
    for (int s = 0; s < 5; s++) { delay(1000); printLogLine(); }
  }
  F1_restart("C3: 50 attempts failed");
}

// ============================================================
// A1 — LOAD EEPROM
// ============================================================
void A1_loadEEPROM() {
  EEPROM.begin(EEPROM_SIZE);
  currentMode = EEPROM.read(ADDR_MODE);
  if (currentMode != 1 && currentMode != 2) currentMode = 1;

  if (EEPROM.read(ADDR_SETTINGS_OK) != 0xA5) {
    // First boot on this chip (or EEPROM was never written) — EEPROM is
    // blank (0xFF). Take the hand-written code defaults above and save
    // them into EEPROM right now, so EEPROM becomes the source of truth
    // from this boot onward — exactly like the calibration table does.
    Serial.println("[A1] First flash detected — saving code defaults to EEPROM");
    EEPROM.write(ADDR_MODE, currentMode);
    eepromWriteUint16(ADDR_MINP, MINP);
    eepromWriteUint16(ADDR_MAXP, MAXP);
    eepromWriteUint16(ADDR_RT,   RT);
    eepromWriteUint16(ADDR_OP,   OP);
    eepromWriteUint16(ADDR_OFP,  OFP);
    eepromWriteUint16(ADDR_ORT,  ORT);
    EEPROM.write(ADDR_SETTINGS_OK, 0xA5);
    EEPROM.commit();
  } else {
    // Not first boot — EEPROM already holds the latest saved values
    // (either still the original defaults, or whatever a Firebase X1
    // command last wrote). Load them as-is.
    MINP = eepromReadUint16(ADDR_MINP);
    MAXP = eepromReadUint16(ADDR_MAXP);
    RT   = eepromReadUint16(ADDR_RT);
    OP   = eepromReadUint16(ADDR_OP);
    OFP  = eepromReadUint16(ADDR_OFP);
    ORT  = eepromReadUint16(ADDR_ORT);
  }

  Serial.printf("[A1] Mode=%d MINP=%d MAXP=%d RT=%d OP=%d OFP=%d ORT=%d\n",
                currentMode, MINP, MAXP, RT, OP, OFP, ORT);
}

// ============================================================
// A2 — MODE 3 INIT CHECK
// ============================================================
bool A2_modeThreeCheck() {
  // Pre-delay: Relay1 OFF for 3s before starting the check
  Serial.println("[A2] Relay1 OFF for 3s (pre-delay)...");
  setRelay(1, false);
  delay(3000);

  Serial.println("[A2] Relay1 ON, Relay2/3 OFF, checking feedback 3s...");
  setRelay(1, true);
  setRelay(2, false);
  setRelay(3, false);
  unsigned long t = millis();
  while (millis() - t < 3000) {
    if (feedbackHigh()) { Serial.println("[A2] B1: HIGH"); return true; }
    delay(100);
  }
  Serial.println("[A2] B2: NOT high");
  return false;
}

// ============================================================
// C1 — C-OFF MODE (one attempt)
// ============================================================
bool C1_cOffMode(int attempt) {
  Serial.printf("[C1] C-off attempt %d (OFP=%ds)\n", attempt, OFP);
  setRelay(2, true);
  setRelay(3, true);
  delay(OFP * 1000UL);
  setRelay(2, false);
  setRelay(3, false);
  unsigned long t = millis();
  while (millis() - t < 3000) {
    if (feedbackLow()) { Serial.println("[C1] D2: LOW — OFF confirmed"); return true; }
    delay(100);
  }
  Serial.println("[C1] D0: Still HIGH");
  return false;
}

// ============================================================
// D1 — REPEAT C-OFF UP TO 10 TIMES
// ============================================================
bool D1_repeatCOff() {
  for (int i = 1; i <= 10; i++) {
    Serial.printf("[D1] Attempt %d/10\n", i);
    if (fbConnected) Firebase.setString(fbdo, DEV_BASE "/status/coff_attempt",
                                        "Attempt " + String(i) + "/10");
    if (C1_cOffMode(i)) { Serial.println("[D1] E2: Success"); return true; }
  }
  Serial.println("[D1] E0: All 10 failed");
  return false;
}

// ============================================================
// E3 — WAIT FOR PRESSURE IN RANGE (10 sec)
// ============================================================
bool E3_waitForPressureInRange() {
  Serial.println("[E3] Waiting pressure in range...");
  unsigned long t = millis();
  while (millis() - t < 10000) {
    pressureVal = readPressure();
    Serial.printf("[E3] P=%.1f range[%d-%d]\n", pressureVal, MINP, MAXP);
    if (pressureVal >= MINP && pressureVal <= MAXP) {
      Serial.println("[E3] F2: In range"); return true;
    }
    delay(1000);
  }
  Serial.println("[E3] F3: Timeout"); return false;
}

// ============================================================
// G1 — PULSE RELAY 2+3 (OP seconds), CHECK FEEDBACK
// ============================================================
bool G1_startContactor() {
  Serial.printf("[G1] ON pulse relay2+3 for %ds\n", OP);
  setRelay(2, true);
  setRelay(3, true);
  delay(OP * 1000UL);
  setRelay(2, false);
  setRelay(3, false);
  unsigned long t = millis();
  while (millis() - t < 3000) {
    if (feedbackHigh()) {
      Serial.println("[G1] G0: Contactor ON");
      contactorState = true; return true;
    }
    delay(100);
  }
  Serial.println("[G1] G01: Not HIGH"); return false;
}

// ============================================================
// H1 — MONITOR PRESSURE (Mode 1 steady state, 1 sec interval)
// Returns true (I1=in range), false (I2=out of range)
// ============================================================
bool H1_monitorPressure() {
  pressureVal = readPressure();
  bool ok = (pressureVal >= MINP && pressureVal <= MAXP);
  if (fbConnected) {
    Firebase.setFloat(fbdo, DEV_BASE "/status/pressure", pressureVal);
    Firebase.setBool(fbdo,  DEV_BASE "/status/inRange",  ok);
  }
  if (!ok) {
    Serial.printf("[H1] I2: OUT OF RANGE %.1f\n", pressureVal);
    fbAlert("Pressure out of range: " + String(pressureVal, 1));
  }
  return ok;
}

// ============================================================
// MODE 1 — AUTO SEQUENCE (E3→G1→H1 loop)
// ============================================================
void runMode1() {
  Serial.println("[MODE1] Starting");

  if (!E3_waitForPressureInRange()) {
    F1_restart("Mode1: Pressure not in range (F3)"); return;
  }

  // G1 up to 5 attempts
  bool started = false;
  for (int i = 1; i <= 5; i++) {
    Serial.printf("[MODE1] G1 attempt %d/5\n", i);
    if (fbConnected) Firebase.setString(fbdo, DEV_BASE "/status/g1_attempt",
                                        "Attempt " + String(i) + "/5");
    if (G1_startContactor()) { started = true; break; }
  }
  if (!started) {
    String msg = "Failed to start contactor after 5 attempts";
    fbAlert(msg); fbCmdStatus(msg);
    F1_restart("Mode1: " + msg); return;
  }

  // H1 monitoring loop (J1 = keep going)
  Serial.println("[MODE1] H1 monitoring loop");
  while (true) {
    unsigned long now = millis();

    // Web log server: handle pending HTTP requests on every tick
    if (ethConnected) webLog.handleClient();

    // Per-second tasks
    if (now - lastLogMillis >= 1000) {
      lastLogMillis = now;
      uploadStatusBatch();      // pressure + contactor + relays in 1 call
      printLogLine();
    }

    // A8 command poll — 250ms (was 500ms) to roughly halve average
    // command latency. Each poll is one Firebase.getString() round-trip
    // whether or not a command is waiting, so this doubles idle polling
    // traffic; kept at 250ms rather than tighter to avoid overloading
    // the same one-second window as uploadStatusBatch/printLogLine/
    // printFastPressureDebug.
    static unsigned long lastCmd = 0;
    if (now - lastCmd >= 250) { lastCmd = now; A8_handleCommand(); }

    // Fast Serial-only debug print, ~3x/sec (independent of Firebase/safety timing)
    static unsigned long lastFastDebug = 0;
    if (now - lastFastDebug >= 333) { lastFastDebug = now; printFastPressureDebug(); }

    // H1 pressure check every 1 sec
    static unsigned long lastH1 = 0;
    if (now - lastH1 >= 1000) {
      lastH1 = now;
      if (!H1_monitorPressure()) {
        F1_restart("Mode1: Pressure out of range (I2)"); return; // I2
      }
      // I1 → J1: keep looping
    }

    delay(10);
  }
}

// ============================================================
// SETUP
// Boot flow (per spec):
//   Step 1: A1 + A2 run back-to-back — neither depends on the network,
//           so nothing Ethernet/Firebase related happens before A6.
//   Step 2: Check A2 result → B1 or B2
//   Step 3: B1 → C1 | B2 → A6
//   Step 4: Check C1 result → D0 or D2
//   Step 5: D0 → D1 | D2 → A6
//   Step 6: D1 result → E0 → E1+F1 (restart) | E2 → A6
//   Step 7: A6 — starts Ethernet, waits for link, connects Firebase
//   Step 7.1: A9 — wait up to 10s for X1 from Firebase; if received,
//             download A1 settings payload, compare with saved
//             values, update EEPROM+RAM if different
//   Step 8: B3 → C3 (retry) | B4 → A7 + A5 (status upload)
//   Step 9: After A7+A5 → C2 (load mode) + A8 (command polling)
//           starts here and runs continuously forever in both modes
//   Step 10: C2 loads Mode 1 or Mode 2
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n======== ESP32 CONTROLLER v" FW_VERSION " ========");

  // ── FIRST: Kill WiFi forever ──────────────────────────────
  disableWiFiForever();

  // ── GPIO init (all relays OFF = HIGH = safe) ──────────────
  pinMode(RELAY1_PIN,    OUTPUT); relayOff(RELAY1_PIN);
  pinMode(RELAY2_PIN,    OUTPUT); relayOff(RELAY2_PIN);
  pinMode(RELAY3_PIN,    OUTPUT); relayOff(RELAY3_PIN);
  pinMode(RELAY4_PIN,    OUTPUT); relayOff(RELAY4_PIN);
  pinMode(FEEDBACK1_PIN, INPUT);
  pinMode(FEEDBACK2_PIN, INPUT);
  // PIN 36 ADC — no pinMode needed, but set attenuation for full 0-3.3V range
  analogSetPinAttenuation(PRESSURE_PIN, ADC_11db);

  // ── STEP 1: A1 + A2 — no network dependency, run back-to-back ──
  A1_loadEEPROM();
  loadCalibTableFromEEPROM();
  bool b1 = A2_modeThreeCheck();

  // ── STEP 2/3: B1 → C1 | B2 → A6 ──────────────────────────
  if (b1) {
    // B1 → C1
    bool d2 = C1_cOffMode(0);
    // ── STEP 4/5: D0 → D1 | D2 → A6 ────────────────────────
    if (!d2) {
      bool e2 = D1_repeatCOff();
      // ── STEP 6: E0 → E1+F1 | E2 → A6 ─────────────────────
      if (!e2) {
        Serial.println("[E1] CRITICAL: Cannot turn off contactor!");
        A6_connectToServer(); // E1 — connect just to report the critical alert
        fbAlert("CRITICAL: Cannot turn off contactor (E1)");
        F1_restart("E1: Critical contactor fail");
        return;
      }
      // E2 → A6
    }
    // D2 → A6
  } else {
    // B2 → A6
    Serial.println("[BOOT] B2: Contactors already off");
  }

  // ── STEP 7: A6 — Ethernet + Firebase connect ─────────────
  bool connected = A6_connectToServer();

  // ── STEP 8: B3 → C3 | B4 → A7 + A5 ───────────────────────
  if (!connected) {
    C3_retryConnection();
  } else {
    Serial.println("[BOOT] B4: Connected!");
  }

  // ── STEP 8 (cont.): A7 + A5 ───────────────────────────────
  A7_uploadContactorStatus();
  A5_uploadPressure();
  pushAllRelayStatus();              // initial relay state snapshot
  uploadCalibTableToFirebase();

  // ── Start web log viewer (runs in parallel inside loop()) ─
  // Browse http://<esp-ip>/  in any browser on the same LAN —
  // no more USB to read serial logs.
  if (ethConnected) startWebLogServer();

  // ── STEP 7.1: A9 — boot-time settings sync (up to 10s) ───
  // While waiting we tick once per second so the user sees a
  // steady log heartbeat on Serial + the web viewer instead of
  // a silent stall. printLogLine() pushes a full status line.
  A9_bootSyncCheck();
  printLogLine();

  // ── STEP 9/10: C2 (load mode) + A8 starts here ───────────
  // A8 command polling begins now and runs continuously forever
  // from this point on, in both Mode 1 (inside runMode1()'s loop)
  // and Mode 2 (inside loop()) — see those functions below.
  logf("[C2] Mode: %d", currentMode);
  if (fbConnected) Firebase.setInt(fbdo, DEV_BASE "/status/mode", currentMode);

  // ── Mode 1 enters its own loop (A8 runs inside it) ───────
  if (currentMode == 1) {
    runMode1(); // never returns normally
  }
  // Mode 2 falls into loop() (A8 runs inside it too)
}

// ============================================================
// LOOP — Mode 2 + background tasks
// ============================================================
void loop() {
  unsigned long now = millis();

  // ── Web log server: handle pending HTTP requests on every tick ───
  if (ethConnected) webLog.handleClient();

  // ── Per-second: A5 + A7 + log + live relay status ────────
  if (now - lastLogMillis >= 1000) {
    lastLogMillis = now;
    uploadStatusBatch();        // pressure + contactor + relays in 1 call
    printLogLine();
  }

  // ── A8: command poll every 250ms (was 500ms) — halves average
  // command latency at the cost of doubling idle polling traffic.
  // Kept at 250ms rather than tighter for the same reason as Mode 1.
  static unsigned long lastCmd = 0;
  if (now - lastCmd >= 250) { lastCmd = now; A8_handleCommand(); }

  // ── Fast Serial-only debug print, ~3x/sec ────────────────
  // (independent of Firebase/safety timing above)
  static unsigned long lastFastDebug = 0;
  if (now - lastFastDebug >= 333) { lastFastDebug = now; printFastPressureDebug(); }

  // ── Firebase keep-alive ───────────────────────────────────
  if (fbConnected && !Firebase.ready()) {
    fbConnected = false;
    Serial.println("[LOOP] FB lost — reconnecting...");
    if (ethConnected) fbConnected = A6_connectToServer();
  }

  delay(10);
}
