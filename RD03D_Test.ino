// RD-03D 24GHz mmWave Radar
//
// int16 encoding: sign-magnitude, little-endian
//   raw = buf[0] | (buf[1] << 8)
//   value = (raw & 0x8000) ? +(raw & 0x7FFF) : -(raw & 0x7FFF)
//
// Frame (30 bytes): AA FF 03 00 | [3 × 8-byte targets] | 55 CC
// Target block: X(2) Y(2) Speed(2) DistRes(2)
// Units: X/Y in mm, Speed in cm/s

#define RADAR_RX          16
#define RADAR_TX          17
#define RADAR_BAUD        256000
#define LED_PIN           2
#define FRAME_LEN         30
#define PRINT_INTERVAL_MS 100

static const uint8_t HEADER[4] = {0xAA, 0xFF, 0x03, 0x00};
static const uint8_t FOOTER[2] = {0x55, 0xCC};

uint8_t  buf[FRAME_LEN];
uint8_t  bufIdx = 0;

bool          anyTarget   = false;
unsigned long lastPrintMs = 0;

struct Target { int16_t x, y, speed, distRes; };
Target lastTargets[3];
bool   lastValid[3];

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

void printSummary() {
  bool printed = false;
  for (int i = 0; i < 3; i++) {
    if (!lastValid[i]) continue;
    Serial.printf("T%d x=%+d y=%+d speed=%+d\n",
                  i,
                  lastTargets[i].x,
                  lastTargets[i].y,
                  lastTargets[i].speed);
    printed = true;
  }
  if (!printed) Serial.println("no targets");
}

void setup() {
  Serial.begin(115200);
  Serial2.begin(RADAR_BAUD, SERIAL_8N1, RADAR_RX, RADAR_TX);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  Serial.println("RD-03D ready");
  lastPrintMs = millis();
}

void loop() {
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

  if (millis() - lastPrintMs >= PRINT_INTERVAL_MS) {
    lastPrintMs = millis();
    printSummary();
  }
}
