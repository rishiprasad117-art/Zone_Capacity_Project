import processing.serial.*;

// ── Config ────────────────────────────────────────────────────────────────────
final int   MAX_DIST_MM    = 8000;
final int   SIGNAL_TIMEOUT = 5000;
final int   NUM_TARGETS    = 3;
final int   TRAIL_LEN      = 15;
final float LERP_FACTOR    = 0.35;

// Per-target colors: T0=red, T1=cyan, T2=yellow
final color[] TARGET_COLORS = { color(255, 60, 60), color(0, 220, 255), color(255, 220, 0) };

// ── State ─────────────────────────────────────────────────────────────────────
Serial    port;
float     scale;
int       originX, originY;

float[]   tX     = new float[NUM_TARGETS];
float[]   tY     = new float[NUM_TARGETS];
float[]   tSpeed = new float[NUM_TARGETS];
boolean[] tValid = new boolean[NUM_TARGETS];

float[]   tTargetX = new float[NUM_TARGETS];
float[]   tTargetY = new float[NUM_TARGETS];

float[][] trailX     = new float[NUM_TARGETS][TRAIL_LEN];
float[][] trailY     = new float[NUM_TARGETS][TRAIL_LEN];
int[]     trailHead  = new int[NUM_TARGETS];
int[]     trailCount = new int[NUM_TARGETS];

long    lastDataMs   = 0;
boolean everReceived = false;

// How long (ms) to keep showing a target after it goes invalid
final int TARGET_PERSIST_MS = 500;
long[]    tLastValidMs = new long[NUM_TARGETS];

int printEvery = 0;  // throttle println

// ── Setup ─────────────────────────────────────────────────────────────────────
void setup() {
  size(800, 800);
  noSmooth();
  frameRate(30);

  originX = width / 2;
  originY = height - 50;
  scale   = (height - 100) / (float) MAX_DIST_MM;

  String[] ports = Serial.list();
  String   pick  = null;
  for (String p : ports) {
    String pl = p.toLowerCase();
    if (pl.contains("usbmodem") || pl.contains("usbserial") ||
        pl.contains("acm")      || pl.contains("usb")) {
      pick = p; break;
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

// ── Smoothing + trail ─────────────────────────────────────────────────────────
boolean targetVisible(int i) {
  return tValid[i] || (millis() - tLastValidMs[i]) < TARGET_PERSIST_MS;
}

void updateSmoothing() {
  for (int i = 0; i < NUM_TARGETS; i++) {
    if (!targetVisible(i)) continue;
    tX[i] = lerp(tX[i], tTargetX[i], LERP_FACTOR);
    tY[i] = lerp(tY[i], tTargetY[i], LERP_FACTOR);
    trailX[i][trailHead[i]] = tX[i];
    trailY[i][trailHead[i]] = tY[i];
    trailHead[i] = (trailHead[i] + 1) % TRAIL_LEN;
    if (trailCount[i] < TRAIL_LEN) trailCount[i]++;
  }
}

// ── Grid ──────────────────────────────────────────────────────────────────────
void drawGrid() {
  strokeWeight(1);
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

  stroke(0, 130, 0);
  strokeWeight(1);
  line(originX - maxR, originY, originX + maxR, originY);
  fill(0, 255, 0);
  noStroke();
  ellipse(originX, originY, 7, 7);
}

// ── Targets ───────────────────────────────────────────────────────────────────
void drawTargets() {
  for (int i = 0; i < NUM_TARGETS; i++) {
    if (!targetVisible(i)) continue;
    drawTrail(i);

    float sx = originX + tX[i] * scale;
    float sy = originY - tY[i] * scale;
    color c  = TARGET_COLORS[i];

    noStroke();
    fill(red(c), green(c), blue(c), 60);
    ellipse(sx, sy, 44, 44);
    fill(c);
    ellipse(sx, sy, 20, 20);
    fill(255);
    textAlign(LEFT, BOTTOM);
    textSize(13);
    text("T" + i, sx + 13, sy - 3);
  }
}

// ── Trail ─────────────────────────────────────────────────────────────────────
void drawTrail(int i) {
  int   count = trailCount[i];
  color c     = TARGET_COLORS[i];
  noStroke();
  for (int j = 0; j < count; j++) {
    int bufIdx = (trailHead[i] - count + j + TRAIL_LEN) % TRAIL_LEN;
    float sx = originX + trailX[i][bufIdx] * scale;
    float sy = originY - trailY[i][bufIdx] * scale;
    float t     = (j + 1.0) / count;
    float alpha = t * 110;
    float diam  = 4 + t * 7;
    fill(red(c), green(c), blue(c), alpha);
    ellipse(sx, sy, diam, diam);
  }
}

// ── HUD ───────────────────────────────────────────────────────────────────────
void drawInfo() {
  int rowY = 12;
  for (int i = 0; i < NUM_TARGETS; i++) {
    if (!targetVisible(i)) continue;
    fill(TARGET_COLORS[i]);
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
  line = trim(line);
  if (line.length() == 0) return;

  int idx = -1;
  if      (line.startsWith("T0")) idx = 0;
  else if (line.startsWith("T1")) idx = 1;
  else if (line.startsWith("T2")) idx = 2;
  else return;

  try {
    float newX = 0, newY = 0, newSpeed = 0;
    boolean newValid = false;
    String[] tokens = split(line, ' ');
    for (String tok : tokens) {
      tok = trim(tok);
      if (tok.startsWith("x="))
        newX = Float.parseFloat(tok.substring(2));
      else if (tok.startsWith("y="))
        newY = Float.parseFloat(tok.substring(2));
      else if (tok.startsWith("speed="))
        newSpeed = Float.parseFloat(tok.substring(6));
      else if (tok.startsWith("valid="))
        newValid = tok.substring(6).trim().equals("1");
    }

    if (!newValid && tValid[idx]) {
      trailCount[idx] = 0;
      trailHead[idx]  = 0;
    }

    tTargetX[idx] = newX;
    tTargetY[idx] = newY;
    tSpeed[idx]   = newSpeed;
    tValid[idx]   = newValid;
    if (newValid) tLastValidMs[idx] = millis();
    lastDataMs    = millis();
    everReceived  = true;

  } catch (Exception e) {
    println("Parse error: " + line);
  }
}
