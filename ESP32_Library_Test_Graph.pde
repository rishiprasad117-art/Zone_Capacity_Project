import processing.serial.*;

// ── Config ────────────────────────────────────────────────────────────────────
final int   MAX_DIST_MM    = 8000;
final int   SIGNAL_TIMEOUT = 5000;   // ms before "NO SIGNAL"
final int   NUM_TARGETS    = 3;
final int   TRAIL_LEN      = 40;
final float LERP_FACTOR    = 0.35;

// ── State ─────────────────────────────────────────────────────────────────────
Serial    port;
float     scale;
int       originX, originY;

// Smoothed display positions
float[]   tX     = new float[NUM_TARGETS];
float[]   tY     = new float[NUM_TARGETS];
float[]   tSpeed = new float[NUM_TARGETS];
boolean[] tValid = new boolean[NUM_TARGETS];

// Raw incoming positions (lerp target)
float[]   tTargetX = new float[NUM_TARGETS];
float[]   tTargetY = new float[NUM_TARGETS];

// Trail circular buffers
float[][] trailX     = new float[NUM_TARGETS][TRAIL_LEN];
float[][] trailY     = new float[NUM_TARGETS][TRAIL_LEN];
int[]     trailHead  = new int[NUM_TARGETS];   // next write index
int[]     trailCount = new int[NUM_TARGETS];   // valid entries so far

long    lastDataMs   = 0;
boolean everReceived = false;

// ── Setup ─────────────────────────────────────────────────────────────────────
void setup() {
  size(800, 800);
  smooth(4);
  frameRate(60);

  originX = width / 2;
  originY = height - 50;
  scale   = (height - 100) / (float) MAX_DIST_MM;  // px per mm

  String[] ports = Serial.list();
  String   pick  = null;

  for (String p : ports) {
    String pl = p.toLowerCase();
    if (pl.contains("usbmodem") || pl.contains("usbserial") ||
        pl.contains("acm")      || pl.contains("usb")) {
      pick = p;
      break;
    }
  }
  if (pick == null && ports.length > 0) pick = ports[0];

  if (pick != null) {
    try {
      port = new Serial(this, pick, 115200);
      port.bufferUntil('\n');
      println("Serial: " + pick);
    } catch (Exception e) {
      println("Could not open port: " + pick + "  -  " + e.getMessage());
    }
  } else {
    println("No serial ports found.");
  }

  textFont(createFont("Monospaced", 13, true));
}

// ── Draw ──────────────────────────────────────────────────────────────────────
void draw() {
  background(0);

  updateSmoothing();
  drawGrid();
  drawTargets();
  drawInfo();

  if (everReceived && (millis() - lastDataMs) > SIGNAL_TIMEOUT) {
    fill(255, 0, 0);
    textAlign(CENTER, CENTER);
    textSize(40);
    text("NO SIGNAL", width / 2, height / 2);
  }
}

// ── Smoothing + trail record (called every frame) ─────────────────────────────
void updateSmoothing() {
  for (int i = 0; i < NUM_TARGETS; i++) {
    if (!tValid[i]) continue;

    tX[i] = lerp(tX[i], tTargetX[i], LERP_FACTOR);
    tY[i] = lerp(tY[i], tTargetY[i], LERP_FACTOR);

    // Push smoothed position into circular trail buffer
    trailX[i][trailHead[i]] = tX[i];
    trailY[i][trailHead[i]] = tY[i];
    trailHead[i] = (trailHead[i] + 1) % TRAIL_LEN;
    if (trailCount[i] < TRAIL_LEN) trailCount[i]++;
  }
}

// ── Grid ──────────────────────────────────────────────────────────────────────
void drawGrid() {
  strokeWeight(1);

  // Concentric range half-circles
  for (int d = 1000; d <= MAX_DIST_MM; d += 1000) {
    float r = d * scale;
    stroke(0, 130, 0);
    noFill();
    arc(originX, originY, r * 2, r * 2, -PI, 0);

    fill(0, 200, 0);
    noStroke();
    textAlign(CENTER, BOTTOM);
    textSize(11);
    text(d / 1000 + "m", originX, originY - r - 3);
  }

  // Angle spokes
  int[] angles = { -60, -30, 0, 30, 60 };
  float maxR   = MAX_DIST_MM * scale;

  for (int a : angles) {
    float rad = radians(a);
    float ex  = originX + sin(rad) * maxR;
    float ey  = originY - cos(rad) * maxR;

    stroke(0, 130, 0);
    strokeWeight(1);
    line(originX, originY, ex, ey);

    fill(0, 200, 0);
    noStroke();
    textAlign(CENTER, CENTER);
    textSize(11);
    text(a + "°", originX + sin(rad) * (maxR + 18),
                  originY - cos(rad) * (maxR + 18));
  }

  // Horizontal baseline
  stroke(0, 130, 0);
  strokeWeight(1);
  line(originX - maxR, originY, originX + maxR, originY);

  // Origin pip
  fill(0, 255, 0);
  noStroke();
  ellipse(originX, originY, 7, 7);
}

// ── Targets ───────────────────────────────────────────────────────────────────
void drawTargets() {
  for (int i = 0; i < NUM_TARGETS; i++) {
    if (!tValid[i]) continue;

    drawTrail(i);

    float sx = originX + tX[i] * scale;
    float sy = originY - tY[i] * scale;

    // Glow layers
    noStroke();
    for (int g = 5; g >= 1; g--) {
      fill(255, 0, 0, 28 * g);
      ellipse(sx, sy, 20 + g * 9, 20 + g * 9);
    }

    // Solid core
    fill(255, 60, 60);
    ellipse(sx, sy, 20, 20);

    // Label
    fill(255);
    textAlign(LEFT, BOTTOM);
    textSize(13);
    text("T" + i, sx + 13, sy - 3);
  }
}

// ── Trail ─────────────────────────────────────────────────────────────────────
void drawTrail(int i) {
  int count = trailCount[i];
  noStroke();
  for (int j = 0; j < count; j++) {
    // j=0 is oldest, j=count-1 is newest
    int bufIdx = (trailHead[i] - count + j + TRAIL_LEN) % TRAIL_LEN;
    float sx = originX + trailX[i][bufIdx] * scale;
    float sy = originY - trailY[i][bufIdx] * scale;

    float t     = (j + 1.0) / count;   // 0→1: oldest→newest
    float alpha = t * 110;
    float diam  = 4 + t * 7;

    fill(255, 80, 80, alpha);
    ellipse(sx, sy, diam, diam);
  }
}

// ── HUD ───────────────────────────────────────────────────────────────────────
void drawInfo() {
  int rowY = 12;

  for (int i = 0; i < NUM_TARGETS; i++) {
    if (!tValid[i]) continue;
    fill(255);
    textAlign(LEFT, TOP);
    textSize(13);
    text(String.format("T%d  x=%+5.0fmm  y=%+5.0fmm  speed=%+4.0fcm/s",
         i, tX[i], tY[i], tSpeed[i]), 10, rowY);
    rowY += 18;
  }

  if (!everReceived) {
    fill(200, 200, 0);
    textAlign(LEFT, TOP);
    textSize(13);
    text("Waiting for data...", 10, 12);
  }
}

// ── Serial ────────────────────────────────────────────────────────────────────
void serialEvent(Serial p) {
  String line = p.readStringUntil('\n');
  if (line == null) return;
  println("RAW: " + line);
  line = trim(line);
  if (line.length() == 0) return;

  // "no targets" → clear all slots
  if (line.equals("no targets")) {
    for (int i = 0; i < NUM_TARGETS; i++) tValid[i] = false;
    lastDataMs   = millis();
    everReceived = true;
    return;
  }

  // Expected: "T0 x=+299 y=+172 speed=+16"
  int idx = -1;
  if      (line.startsWith("T0")) idx = 0;
  else if (line.startsWith("T1")) idx = 1;
  else if (line.startsWith("T2")) idx = 2;
  else return;

  try {
    float newX = 0, newY = 0, newSpeed = 0;
    String[] tokens = split(line, ' ');
    for (String tok : tokens) {
      if (tok.startsWith("x="))
        newX = Float.parseFloat(tok.substring(2));
      else if (tok.startsWith("y="))
        newY = Float.parseFloat(tok.substring(2));
      else if (tok.startsWith("speed="))
        newSpeed = Float.parseFloat(tok.substring(6));
    }
    tTargetX[idx] = newX;
    tTargetY[idx] = newY;
    tSpeed[idx]   = newSpeed;
    tValid[idx]   = true;
    lastDataMs    = millis();
    everReceived  = true;

  } catch (Exception e) {
    println("Parse error: " + line);
  }
}
