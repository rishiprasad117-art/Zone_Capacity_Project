// RD-03D 24GHz mmWave Radar
//
// int16 encoding: sign-magnitude, little-endian
//   raw = buf[0] | (buf[1] << 8)
//   value = (raw & 0x8000) ? +(raw & 0x7FFF) : -(raw & 0x7FFF)
//
// Frame (30 bytes): AA FF 03 00 | [3 × 8-byte targets] | 55 CC
// Target block: X(2) Y(2) Speed(2) DistRes(2)
// Units: X/Y in mm, Speed in cm/s

#include <WiFi.h>
#include <HTTPClient.h>

#define RADAR_RX          16
#define RADAR_TX          17
#define RADAR_BAUD        256000
#define LED_PIN           2
#define FRAME_LEN         30
#define PRINT_INTERVAL_MS 100
#define POST_INTERVAL_MS  2000

const char* WIFI_SSID   = "UCLA_WEB";
// Go to webhook.site, copy your unique URL and paste it here
const char* WEBHOOK_URL = "https://webhook.site/YOUR_UNIQUE_URL_HERE";

static const uint8_t HEADER[4] = {0xAA, 0xFF, 0x03, 0x00};
static const uint8_t FOOTER[2] = {0x55, 0xCC};

uint8_t  buf[FRAME_LEN];
uint8_t  bufIdx = 0;

bool          anyTarget   = false;
unsigned long lastPrintMs = 0;
unsigned long lastPostMs  = 0;

struct Target { int16_t x, y, speed, distRes; };
Target lastTargets[3];
bool   lastValid[3];

// ─── Radar helpers ────────────────────────────────────────────────────────

static int16_t decodeSM(uint8_t lo, uint8_t hi) {
  uint16_t raw = (uint16_t)lo | ((uint16_t)hi << 8);
  int16_t  mag = (int16_t)(raw & 0x7FFF);
  return (raw & 0x8000) ? mag : -mag;
}

static bool blockValid(int offset) {
  // Library uses distanceRes == 0 to mean "empty slot"
  uint16_t distRes = (uint16_t)buf[offset + 6] | ((uint16_t)buf[offset + 7] << 8);
  return distRes != 0;
}

void processFrame() {
  bool detected = false;
  const int offsets[3] = {4, 12, 20};

  for (int i = 0; i < 3; i++) {
    int o = offsets[i];
    lastValid[i] = blockValid(o);
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

void printSummary() {
  for (int i = 0; i < 3; i++) {
    int16_t x = lastValid[i] ? lastTargets[i].x     : 0;
    int16_t y = lastValid[i] ? lastTargets[i].y     : 0;
    int16_t s = lastValid[i] ? lastTargets[i].speed : 0;
    Serial.printf("T%d x=%+d y=%+d speed=%+d valid=%d\n",
                  i, x, y, s, lastValid[i] ? 1 : 0);
  }
}


// ─── WiFi (non-blocking init) ─────────────────────────────────────────────

void wifiBegin() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID);  // open network — no password
  Serial.print("WiFi connecting");
}

// ─── HTTP POST ────────────────────────────────────────────────────────────

void sendPost() {
  if (WiFi.status() != WL_CONNECTED) return;

  int count = 0;
  for (int i = 0; i < 3; i++) if (lastValid[i]) count++;

  // Send one POST per valid target
  for (int i = 0; i < 3; i++) {
    if (!lastValid[i]) continue;

    HTTPClient http;
    http.begin(WEBHOOK_URL);
    http.addHeader("Content-Type", "application/json");

    char body[128];
    snprintf(body, sizeof(body),
      "{\"sensor_id\":\"sensor_01\","
      "\"target\":%d,"
      "\"x\":%d,"
      "\"y\":%d,"
      "\"speed\":%d,"
      "\"count\":%d}",
      i,
      lastTargets[i].x,
      lastTargets[i].y,
      lastTargets[i].speed,
      count);

    int code = http.POST(body);
    if (code > 0) {
      Serial.printf("POST sent: %d\n", code);
    } else {
      Serial.printf("POST failed: %s\n", http.errorToString(code).c_str());
    }
    http.end();
  }
}

// ─── setup / loop ─────────────────────────────────────────────────────────

void setup() {
  Serial.begin(115200);
  Serial2.begin(RADAR_BAUD, SERIAL_8N1, RADAR_RX, RADAR_TX);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // Flush any startup bytes from the sensor
  delay(100);
  while (Serial2.available()) Serial2.read();

  // Enable multi-target mode (exact command from javier-fg/arduino_rd-03d library)
  static const uint8_t CMD_MULTI[12] = {
    0xFD, 0xFC, 0xFB, 0xFA, 0x02, 0x00,
    0x90, 0x00,
    0x04, 0x03, 0x02, 0x01
  };
  Serial2.write(CMD_MULTI, sizeof(CMD_MULTI));

  // Wait for ACK (ends with 04 03 02 01), up to 500ms
  unsigned long t = millis();
  while (millis() - t < 500) {
    if (Serial2.available()) Serial2.read();
  }

  Serial.println("RD-03D ready (multi-target)");
  wifiBegin();
  lastPrintMs = millis();
  lastPostMs  = millis();
}

void loop() {
  // Non-blocking WiFi status reporting
  static wl_status_t lastWifiStatus = WL_IDLE_STATUS;
  wl_status_t wifiStatus = WiFi.status();
  if (wifiStatus != lastWifiStatus) {
    if (wifiStatus == WL_CONNECTED) {
      Serial.printf("\nWiFi connected! IP: %s\n", WiFi.localIP().toString().c_str());
    } else if (lastWifiStatus == WL_CONNECTED) {
      Serial.println("WiFi disconnected, reconnecting...");
      wifiBegin();
    } else {
      Serial.print(".");
    }
    lastWifiStatus = wifiStatus;
  }

  // Radar frame ingestion
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

  // Print at 100ms
  if (millis() - lastPrintMs >= PRINT_INTERVAL_MS) {
    lastPrintMs = millis();
    printSummary();
  }

  // POST at 2s when targets present
  if (anyTarget && millis() - lastPostMs >= POST_INTERVAL_MS) {
    lastPostMs = millis();
    sendPost();
  }
}
