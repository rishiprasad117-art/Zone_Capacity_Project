// RD-03D 24GHz mmWave Radar — Dual sensor via 74HC4051 MUX
//
// int16 encoding: sign-magnitude, little-endian
//   raw = buf[0] | (buf[1] << 8)
//   value = (raw & 0x8000) ? +(raw & 0x7FFF) : -(raw & 0x7FFF)
//
// Frame (30 bytes): AA FF 03 00 | [3 x 8-byte targets] | 55 CC
// Target block: X(2) Y(2) Speed(2) DistRes(2)
// Units: X/Y in mm, Speed in cm/s
//
// ─── Wiring ───────────────────────────────────────────────────────────────
//
//  Both 74HC4051 modules:
//    VEE → GND
//    VCC → 3.3V
//    GND → GND
//    S0  → GPIO 4   (toggles between sensor 1 and 2)
//    S1  → GND
//    S2  → GND
//    E   → GPIO 5   (active LOW, driven LOW at startup)
//
//  Module A (RX path):
//    Z   → GPIO 16  (ESP32 UART RX)
//    Y0  → Sensor 1 TX
//    Y1  → Sensor 2 TX
//
//  Sensor power:
//    Both RD-03D VCC → 5V rail
//    Both RD-03D GND → GND
// ─────────────────────────────────────────────────────────────────────────

#include <WiFi.h>
#include <HTTPClient.h>

#define RADAR_RX           16
#define RADAR_TX           17
#define RADAR_BAUD         256000
#define LED_PIN            2
#define MUX_S0_PIN         4
#define MUX_EN_PIN         5
#define FRAME_LEN          30
#define PRINT_INTERVAL_MS  500     // print every 500ms for readability
#define POST_INTERVAL_MS   2000
#define SWITCH_INTERVAL_MS 10000

const char* WIFI_SSID   = "UCLA_WEB";
const char* WEBHOOK_URL = "https://webhook.site/YOUR_UNIQUE_URL_HERE";

static const uint8_t HEADER[4] = {0xAA, 0xFF, 0x03, 0x00};
static const uint8_t FOOTER[2] = {0x55, 0xCC};

uint8_t  buf[FRAME_LEN];
uint8_t  bufIdx = 0;

bool          anyTarget    = false;
unsigned long lastPrintMs  = 0;
unsigned long lastPostMs   = 0;
unsigned long lastSwitchMs = 0;
uint8_t       activeSensor = 0;

struct Target { int16_t x, y, speed, distRes; };
Target lastTargets[3];
bool   lastValid[3];

// ─── MUX control ─────────────────────────────────────────────────────────

void selectSensor(uint8_t sensor) {
  activeSensor = sensor & 1;

  digitalWrite(MUX_EN_PIN, HIGH);  // disable during switch
  delayMicroseconds(10);
  digitalWrite(MUX_S0_PIN, activeSensor ? HIGH : LOW);
  delayMicroseconds(10);
  digitalWrite(MUX_EN_PIN, LOW);   // re-enable

  delay(5);
  while (Serial2.available()) Serial2.read();
  bufIdx = 0;

  Serial.printf(">>> Switched to sensor_%02d (S0=%d)\n",
                activeSensor + 1, activeSensor);
}

// ─── Radar helpers ────────────────────────────────────────────────────────

static int16_t decodeSM(uint8_t lo, uint8_t hi) {
  uint16_t raw = (uint16_t)lo | ((uint16_t)hi << 8);
  int16_t  mag = (int16_t)(raw & 0x7FFF);
  return (raw & 0x8000) ? mag : -mag;
}

static bool blockNonZero(int offset) {
  for (int i = 0; i < 8; i++)
    if (buf[offset + i]) return true;
  return false;
}

void processFrame() {
  bool detected = false;
  const int offsets[3] = {4, 12, 20};

  for (int i = 0; i < 3; i++) {
    int o = offsets[i];
    lastValid[i] = blockNonZero(o);
    if (lastValid[i]) {
      lastTargets[i] = {
        decodeSM(buf[o+0], buf[o+1]),
        decodeSM(buf[o+2], buf[o+3]),
        decodeSM(buf[o+4], buf[o+5]),
        decodeSM(buf[o+6], buf[o+7])
      };
      detected = true;
    }
  }

  anyTarget = detected;
  digitalWrite(LED_PIN, detected ? HIGH : LOW);
}

// ─── Print summary ────────────────────────────────────────────────────────

void printSummary() {
  Serial.println("+-----------------------------------------+");
  Serial.printf( "|  Sensor %d                               |\n", activeSensor + 1);
  Serial.println("+=========================================+");

  for (int i = 0; i < 3; i++) {
    if (lastValid[i]) {
      Serial.printf("|  T%d  [DETECTED]                         |\n", i + 1);
      Serial.printf("|    x      = %+7d mm                  |\n", lastTargets[i].x);
      Serial.printf("|    y      = %+7d mm                  |\n", lastTargets[i].y);
      Serial.printf("|    speed  = %+7d cm/s                |\n", lastTargets[i].speed);
      Serial.printf("|    res    = %7d mm                  |\n", lastTargets[i].distRes);
    } else {
      Serial.printf("|  T%d  [NOT IN FRAME]                     |\n", i + 1);
    }

    if (i < 2) Serial.println("|  - - - - - - - - - - - - - - - - - - - |");
  }

  Serial.println("+-----------------------------------------+");
  Serial.println();
}

// ─── WiFi ─────────────────────────────────────────────────────────────────

void wifiBegin() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID);
  Serial.print("WiFi connecting");
}

// ─── HTTP POST ────────────────────────────────────────────────────────────

void sendPost() {
  if (WiFi.status() != WL_CONNECTED) return;

  int count = 0;
  for (int i = 0; i < 3; i++) if (lastValid[i]) count++;

  for (int i = 0; i < 3; i++) {
    if (!lastValid[i]) continue;

    HTTPClient http;
    http.begin(WEBHOOK_URL);
    http.addHeader("Content-Type", "application/json");

    char body[128];
    snprintf(body, sizeof(body),
      "{\"sensor_id\":\"sensor_%02d\","
      "\"target\":%d,"
      "\"x\":%d,"
      "\"y\":%d,"
      "\"speed\":%d,"
      "\"count\":%d}",
      activeSensor + 1,
      i,
      lastTargets[i].x,
      lastTargets[i].y,
      lastTargets[i].speed,
      count);

    int code = http.POST(body);
    if (code > 0)
      Serial.printf("POST sent (S%d): %d\n", activeSensor + 1, code);
    else
      Serial.printf("POST failed (S%d): %s\n",
                    activeSensor + 1,
                    http.errorToString(code).c_str());
    http.end();
  }
}

// ─── setup / loop ─────────────────────────────────────────────────────────

void setup() {
  Serial.begin(115200);
  Serial2.begin(RADAR_BAUD, SERIAL_8N1, RADAR_RX, RADAR_TX);

  pinMode(LED_PIN,    OUTPUT);
  pinMode(MUX_S0_PIN, OUTPUT);
  pinMode(MUX_EN_PIN, OUTPUT);

  digitalWrite(LED_PIN,    LOW);
  digitalWrite(MUX_EN_PIN, LOW);
  digitalWrite(MUX_S0_PIN, LOW);

  memset(lastValid,   0, sizeof(lastValid));
  memset(lastTargets, 0, sizeof(lastTargets));

  selectSensor(0);
  Serial.println("RD-03D dual-sensor (74HC4051) ready");

  wifiBegin();
  lastPrintMs  = millis();
  lastPostMs   = millis();
  lastSwitchMs = millis();
}

void loop() {
  // ── Non-blocking WiFi status ──
  static wl_status_t lastWifiStatus = WL_IDLE_STATUS;
  wl_status_t wifiStatus = WiFi.status();
  if (wifiStatus != lastWifiStatus) {
    if (wifiStatus == WL_CONNECTED)
      Serial.printf("\nWiFi connected! IP: %s\n", WiFi.localIP().toString().c_str());
    else if (lastWifiStatus == WL_CONNECTED) {
      Serial.println("WiFi disconnected, reconnecting...");
      wifiBegin();
    } else {
      Serial.print(".");
    }
    lastWifiStatus = wifiStatus;
  }

  // ── Switch sensor every 10s ──
  if (millis() - lastSwitchMs >= SWITCH_INTERVAL_MS) {
    lastSwitchMs = millis();
    selectSensor(activeSensor ^ 1);
    anyTarget = false;
    memset(lastValid,   0, sizeof(lastValid));
    memset(lastTargets, 0, sizeof(lastTargets));
  }

  // ── Frame ingestion ──
  while (Serial2.available()) {
    uint8_t b = Serial2.read();

    if (bufIdx < 4) {
      if (b == HEADER[bufIdx]) {
        buf[bufIdx++] = b;
      } else {
        bufIdx = (b == HEADER[0]) ? 1 : 0;
        if (bufIdx) buf[0] = b;
      }
      continue;
    }

    buf[bufIdx++] = b;

    if (bufIdx == FRAME_LEN) {
      bufIdx = 0;
      if (buf[28] == FOOTER[0] && buf[29] == FOOTER[1])
        processFrame();
    }
  }

  // ── Print every 500ms ──
  if (millis() - lastPrintMs >= PRINT_INTERVAL_MS) {
    lastPrintMs = millis();
    printSummary();
  }

  // ── POST every 2s when targets present ──
  if (anyTarget && millis() - lastPostMs >= POST_INTERVAL_MS) {
    lastPostMs = millis();
    sendPost();
  }
}