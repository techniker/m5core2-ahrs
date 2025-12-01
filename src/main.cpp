/*
 * M5Stack Core2 – AHRS / EFIS-Style Attitude Indicator
 * Full-screen horizon with bottom compass/heading tape.
 * Bjoern Heller <tec att sixtopia.net>
 *
 * Features:)
 * - Simple Kalman smoothing for pitch/roll/heading
 * - Artificial horizon
 * - Pitch ladder
 * - Bank scale at top
 * - Slip/skid ball with simple mass-spring-damper physics
 * - Flight director crossbars
 * - Flight Path Vector bird using attitude + accelerometer data
 * - Heading tape at bottom + pitch/roll + battery Voltage
 *
 * Runs on a M5Stack Core2
 */

#include <M5Core2.h>
#include <math.h>

// ───────────────────────────────────────────────────────────
// Screen & Instrument Geometry
// ───────────────────────────────────────────────────────────

TFT_eSprite g_canvas(&M5.Lcd);

const int TFT_W = 320;
const int TFT_H = 240;

// Instrument area for horizon etc.
const int INST_X = 0;
const int INST_Y = 0;
const int INST_W = TFT_W;
const int INST_H = TFT_H - 50;   // leave 50 px bottom for heading tape
const int BOTTOM_BAND_H = 50;

// Center of instrument
const int CX = INST_X + INST_W / 2;
const int CY = INST_Y + INST_H / 2;

// ───────────────────────────────────────────────────────────
// Colors
// ───────────────────────────────────────────────────────────

uint16_t COLOR_PANEL   = 0x2104;      // dark grey (panel)
uint16_t COLOR_SKY     = 0x5A9D;      // light blue-ish
uint16_t COLOR_GND     = 0xC380;      // brownish
uint16_t COLOR_TEXT    = TFT_WHITE;
uint16_t COLOR_SCALE   = TFT_YELLOW;
uint16_t COLOR_FD      = 0xF81F;      // magenta (FD)
uint16_t COLOR_FPV     = TFT_GREEN;   // FPV / bird

// Pitch ladder parameters
const float PITCH_RANGE_DEG    = 30.0f;
const float PITCH_PIX_PER_DEG  = 4.0f;

// Sideslip parameters
const int   SLIP_BAR_WIDTH   = 80;
const int   SLIP_BAR_HEIGHT  = 8;
const int   SLIP_BALL_RADIUS = 6;

// Heading tape
const float HEADING_PX_PER_DEG = 3.0f;

// ───────────────────────────────────────────────────────────
// Globals – attitude, offsets & physics
// ───────────────────────────────────────────────────────────

float g_pitch_raw = 0.0f;
float g_roll_raw  = 0.0f;
float g_yaw_raw   = 0.0f;

float g_pitch = 0.0f;  // filtered
float g_roll  = 0.0f;
float g_yaw   = 0.0f;

// Zero offsets
float g_pitch_zero = 0.0f;
float g_roll_zero  = 0.0f;
float g_yaw_zero   = 0.0f;

// Slip ball physics
float slipPos = 0.0f;  // px
float slipVel = 0.0f;  // px/s

// Timing
unsigned long lastDrawMs   = 0;
unsigned long lastUpdateMs = 0;
const unsigned long DRAW_INTERVAL_MS = 30;

// Flight director command (demo: 5° nose up, 0° bank)
float fd_pitch_cmd = 5.0f;
float fd_roll_cmd  = 0.0f;

// ───────────────────────────────────────────────────────────
// Simple 1D Kalman Filter for smoothing
// ───────────────────────────────────────────────────────────

struct SimpleKalman {
  float x;
  float P;
  float Q;
  float R;

  SimpleKalman() : x(0), P(1), Q(0.02f), R(2.0f) {}

  float update(float z) {
    P += Q;
    float K = P / (P + R);
    x = x + K * (z - x);
    P = (1 - K) * P;
    return x;
  }
};

SimpleKalman kPitch, kRoll, kYaw;
bool kalmanInit = false;

// ───────────────────────────────────────────────────────────
// Utility
// ───────────────────────────────────────────────────────────

float deg2rad(float d) { return d * PI / 180.0f; }

float wrap360(float a) {
  while (a < 0) a += 360.0f;
  while (a >= 360.0f) a -= 360.0f;
  return a;
}

float angleDiff(float a, float b) {
  float d = wrap360(a) - wrap360(b);
  if (d > 180.0f) d -= 360.0f;
  if (d < -180.0f) d += 360.0f;
  return d;
}

void rotateAroundCenter(float x, float y, float rollRad, int &xo, int &yo) {
  float dx = x - CX;
  float dy = y - CY;
  float ca = cos(rollRad);
  float sa = sin(rollRad);
  float xr = dx * ca - dy * sa;
  float yr = dx * sa + dy * ca;
  xo = (int)(CX + xr);
  yo = (int)(CY + yr);
}

// ───────────────────────────────────────────────────────────
// Horizon + Pitch Ladder (full instrument area)
// ───────────────────────────────────────────────────────────

void drawHorizon(float pitchDeg, float rollDeg) {
  // FIX: invert roll sign so horizon turns opposite to aircraft
  float rollRad = deg2rad(-rollDeg);

  // Direction along horizon line
  float dX = cos(rollRad);
  float dY = sin(rollRad);

  // Normal towards sky
  float nX = sin(rollRad);
  float nY = -cos(rollRad);

  float pitchOffset = pitchDeg * PITCH_PIX_PER_DEG;
  float cX = CX;
  float cY = CY + pitchOffset;

  const float L = 400.0f;
  float p1x = cX - dX * L;
  float p1y = cY - dY * L;
  float p2x = cX + dX * L;
  float p2y = cY + dY * L;

  const float Ln = 400.0f;
  float sky1x = p1x + nX * Ln;
  float sky1y = p1y + nY * Ln;
  float sky2x = p2x + nX * Ln;
  float sky2y = p2y + nY * Ln;
  float grd1x = p1x - nX * Ln;
  float grd1y = p1y - nY * Ln;
  float grd2x = p2x - nX * Ln;
  float grd2y = p2y - nY * Ln;

  // Fill sky & ground over whole instrument area
  g_canvas.fillTriangle((int)p1x, (int)p1y, (int)p2x, (int)p2y, (int)sky1x, (int)sky1y, COLOR_SKY);
  g_canvas.fillTriangle((int)p2x, (int)p2y, (int)sky2x, (int)sky2y, (int)sky1x, (int)sky1y, COLOR_SKY);

  g_canvas.fillTriangle((int)p1x, (int)p1y, (int)p2x, (int)p2y, (int)grd1x, (int)grd1y, COLOR_GND);
  g_canvas.fillTriangle((int)p2x, (int)p2y, (int)grd2x, (int)grd2y, (int)grd1x, (int)grd1y, COLOR_GND);

  // Helper for ladder
  auto rot = [&](float x, float y) -> std::pair<int,int> {
    int xo, yo;
    rotateAroundCenter(x, y, rollRad, xo, yo);
    return { xo, yo };
  };

  // Zero pitch line
  {
    float yZero = CY + (pitchDeg - 0.0f) * PITCH_PIX_PER_DEG;
    auto zL = rot(CX - 80, yZero);
    auto zR = rot(CX + 80, yZero);
    g_canvas.drawLine(zL.first, zL.second, zR.first, zR.second, TFT_WHITE);
    g_canvas.drawLine(zL.first, zL.second + 1, zR.first, zR.second + 1, TFT_WHITE);
  }

  // Pitch ladder
  for (int mark = -(int)PITCH_RANGE_DEG; mark <= (int)PITCH_RANGE_DEG; mark += 5) {
    if (mark == 0) continue;
    float deltaPitch = pitchDeg - (float)mark;
    float y = CY + deltaPitch * PITCH_PIX_PER_DEG;
    if (y < INST_Y - 40 || y > INST_Y + INST_H + 40) continue;

    auto pL = rot(CX - 40, y);
    auto pR = rot(CX + 40, y);
    g_canvas.drawLine(pL.first, pL.second, pR.first, pR.second, TFT_WHITE);

    if (mark % 10 == 0) {
      auto pLL = rot(CX - 55, y);
      auto pRR = rot(CX + 55, y);
      g_canvas.drawLine(pLL.first, pLL.second, pRR.first, pRR.second, TFT_WHITE);

      char buf[8];
      snprintf(buf, sizeof(buf), "%+d", -mark);
      int tx = pRR.first + 4;
      int ty = pRR.second - 4;
      g_canvas.setTextColor(COLOR_TEXT, COLOR_SKY);
      g_canvas.setTextSize(1);
      g_canvas.setCursor(tx, ty);
      g_canvas.print(buf);
    }
  }

  // Fixed aircraft symbol
  g_canvas.drawLine(CX - 30, CY, CX - 10, CY, TFT_ORANGE);
  g_canvas.drawLine(CX + 10, CY, CX + 30, CY, TFT_ORANGE);
  g_canvas.drawLine(CX - 10, CY, CX,      CY + 8, TFT_ORANGE);
  g_canvas.drawLine(CX + 10, CY, CX,      CY + 8, TFT_ORANGE);
  g_canvas.fillCircle(CX, CY + 8, 3, TFT_ORANGE);
}

// ───────────────────────────────────────────────────────────
// Bank Scale (semi-circle at top)
// ───────────────────────────────────────────────────────────

void drawBankScale(float rollDeg) {
  const int radiusOuter = 70;
  const int radiusInner = 60;
  const int centerX = CX;
  const int centerY = INST_Y + 5;

  int angles[] = { -45, -30, -20, -10, 0, 10, 20, 30, 45 };
  for (int i = 0; i < (int)(sizeof(angles)/sizeof(angles[0])); i++) {
    float aDeg = (float)angles[i];
    float aRad = deg2rad(aDeg - 90);
    int xInner = centerX + (int)(radiusInner * cos(aRad));
    int yInner = centerY + (int)(radiusInner * sin(aRad));
    int len = (angles[i] == 0 || abs(angles[i]) == 30 || abs(angles[i]) == 45) ? 10 : 6;
    int x2 = centerX + (int)((radiusInner - len) * cos(aRad));
    int y2 = centerY + (int)((radiusInner - len) * sin(aRad));
    g_canvas.drawLine(xInner, yInner, x2, y2, COLOR_SCALE);
  }

  // Fixed pointer at top
  int pX1 = centerX;
  int pY1 = centerY - radiusOuter - 2;
  g_canvas.fillTriangle(pX1 - 6, pY1, pX1 + 6, pY1, pX1, pY1 - 8, TFT_ORANGE);

  // Moving bank index
  float bankRad = deg2rad(rollDeg - 90);
  int indX = centerX + (int)(radiusInner * cos(bankRad));
  int indY = centerY + (int)(radiusInner * sin(bankRad));
  g_canvas.fillCircle(indX, indY, 4, TFT_ORANGE);
}

// ───────────────────────────────────────────────────────────
// Slip / Skid Ball – physics
// ───────────────────────────────────────────────────────────

void updateAndDrawSlip(float ay, float az, float dt) {
  int barCenterX = CX;
  int barCenterY = INST_Y + INST_H - 25;

  g_canvas.fillRect(barCenterX - SLIP_BAR_WIDTH/2 - 10,
                    barCenterY - 15,
                    SLIP_BAR_WIDTH + 20,
                    30,
                    COLOR_GND);  // choose something that blends well

  g_canvas.drawRect(barCenterX - SLIP_BAR_WIDTH/2,
                    barCenterY - SLIP_BAR_HEIGHT/2,
                    SLIP_BAR_WIDTH,
                    SLIP_BAR_HEIGHT,
                    TFT_WHITE);

  float angle = atan2f(ay, az);
  if (angle > 0.5f) angle = 0.5f;
  if (angle < -0.5f) angle = -0.5f;

  float targetPos = angle / 0.5f * (SLIP_BAR_WIDTH/2 - SLIP_BALL_RADIUS);

  float k = 20.0f;
  float d = 6.0f;

  float acc = k * (targetPos - slipPos) - d * slipVel;
  slipVel += acc * dt;
  slipPos += slipVel * dt;

  float maxPos = SLIP_BAR_WIDTH/2 - SLIP_BALL_RADIUS;
  if (slipPos >  maxPos) slipPos =  maxPos;
  if (slipPos < -maxPos) slipPos = -maxPos;

  int ballX = barCenterX + (int)slipPos;
  int ballY = barCenterY;
  g_canvas.fillCircle(ballX, ballY, SLIP_BALL_RADIUS, TFT_WHITE);
}

// ───────────────────────────────────────────────────────────
// Flight Director Crossbars
// ───────────────────────────────────────────────────────────

void drawFlightDirector(float dispPitch, float dispRoll) {
  float pitchErr = fd_pitch_cmd - dispPitch;
  float rollErr  = fd_roll_cmd  - dispRoll;

  float pixPitch = pitchErr * PITCH_PIX_PER_DEG;
  float pixRoll  = rollErr  * (INST_W / 90.0f);

  const float maxPix = 40.0f;
  if (pixPitch >  maxPix) pixPitch =  maxPix;
  if (pixPitch < -maxPix) pixPitch = -maxPix;
  if (pixRoll  >  maxPix) pixRoll  =  maxPix;
  if (pixRoll  < -maxPix) pixRoll  = -maxPix;

  int cx = CX + (int)pixRoll;
  int cy = CY + (int)pixPitch;

  g_canvas.drawLine(cx - 25, cy,      cx + 25, cy,      COLOR_FD);
  g_canvas.drawLine(cx,      cy - 18, cx,      cy + 18, COLOR_FD);
}

// ───────────────────────────────────────────────────────────
// Flight Path Vector (“bird”)
// ───────────────────────────────────────────────────────────

void drawFlightPathVector(float dispPitch, float dispRoll, float ax, float ay) {
  float fpvPitch = dispPitch - ax * 5.0f;
  float fpvRoll  = dispRoll  + ay * 5.0f;

  // FIX: use inverted roll for rotation here too
  float rollRad = deg2rad(-dispRoll);

  float yUnrolled = CY + fpvPitch * PITCH_PIX_PER_DEG;

  int fx, fy;
  rotateAroundCenter(CX, yUnrolled, rollRad, fx, fy);

  g_canvas.fillCircle(fx, fy, 4, COLOR_FPV);
  g_canvas.drawLine(fx - 14, fy,      fx - 3, fy,      COLOR_FPV);
  g_canvas.drawLine(fx + 3,  fy,      fx + 14, fy,      COLOR_FPV);
  g_canvas.drawLine(fx - 3,  fy + 4,  fx + 3,  fy + 4,  COLOR_FPV);
}

// ───────────────────────────────────────────────────────────
// Heading Tape + Bottom Info
// ───────────────────────────────────────────────────────────

void drawHeadingTapeAndInfo(float yawDeg, float dispPitch, float dispRoll) {
  int bandY = TFT_H - BOTTOM_BAND_H;

  g_canvas.fillRect(0, bandY, TFT_W, BOTTOM_BAND_H, COLOR_PANEL);
  g_canvas.drawRect(0, bandY, TFT_W, BOTTOM_BAND_H, TFT_DARKGREY);

  float hdg = wrap360(yawDeg);
  float spanDeg = TFT_W / (2.0f * HEADING_PX_PER_DEG);

  for (int h = 0; h < 360; h += 5) {
    float d = angleDiff((float)h, hdg);
    if (d < -spanDeg || d > spanDeg) continue;

    int x = (int)(CX + d * HEADING_PX_PER_DEG);
    int yTop = bandY + 5;
    bool major = (h % 10 == 0);
    int len = major ? 12 : 7;

    g_canvas.drawLine(x, yTop, x, yTop + len, TFT_WHITE);

    if (major) {
      char buf[8];
      snprintf(buf, sizeof(buf), "%02d", h / 10);
      g_canvas.setTextColor(COLOR_TEXT, COLOR_PANEL);
      g_canvas.setTextSize(1);
      g_canvas.setCursor(x - 6, yTop + len + 2);
      g_canvas.print(buf);
    }
  }

  // Center marker
  int pY = bandY;
  g_canvas.fillTriangle(CX - 6, pY + 2, CX + 6, pY + 2, CX, pY + 10, TFT_ORANGE);

  // Digital heading
  char buff[8];
  snprintf(buff, sizeof(buff), "%03d", (int)roundf(hdg));
  g_canvas.setTextColor(COLOR_TEXT, COLOR_PANEL);
  g_canvas.setTextSize(2);
  g_canvas.setCursor(CX - 3 * 6, bandY + 25);
  g_canvas.print(buff);

  // Right side: roll / pitch
  g_canvas.setTextSize(1);
  g_canvas.setCursor(TFT_W - 80, bandY + 10);
  snprintf(buff, sizeof(buff), "R %+4.0f", dispRoll);
  g_canvas.print(buff);
  g_canvas.setCursor(TFT_W - 80, bandY + 24);
  snprintf(buff, sizeof(buff), "P %+4.0f", dispPitch);
  g_canvas.print(buff);

  // Left side: battery
  float batt = M5.Axp.GetBatVoltage();
  char bbuf[16];
  snprintf(bbuf, sizeof(bbuf), "%.2fV", batt);
  g_canvas.setCursor(10, bandY + 18);
  g_canvas.print(bbuf);
}

// ───────────────────────────────────────────────────────────
// Setup & Loop
// ───────────────────────────────────────────────────────────

void setup() {
  M5.begin(true, true, true, true);
  M5.IMU.Init();

  M5.Lcd.setRotation(1);
  M5.Axp.ScreenBreath(15);

  g_canvas.setColorDepth(16);
  g_canvas.createSprite(TFT_W, TFT_H);
  g_canvas.fillScreen(COLOR_SKY);

  lastDrawMs   = millis();
  lastUpdateMs = lastDrawMs;
}

void loop() {
  M5.update();
  unsigned long now = millis();
  float dt = (now - lastUpdateMs) / 1000.0f;
  if (dt <= 0.0f) dt = 0.001f;
  lastUpdateMs = now;

  // Raw attitude
  float pitchRaw, rollRaw, yawRaw;
  M5.IMU.getAhrsData(&pitchRaw, &rollRaw, &yawRaw);

  // On your Core2 pitch/roll were swapped:
  g_pitch_raw = rollRaw;
  g_roll_raw  = pitchRaw;
  g_yaw_raw   = yawRaw;

  if (!kalmanInit) {
    kPitch.x = g_pitch_raw;
    kRoll.x  = g_roll_raw;
    kYaw.x   = g_yaw_raw;
    kalmanInit = true;
  }

  g_pitch = kPitch.update(g_pitch_raw);
  g_roll  = kRoll.update(g_roll_raw);
  g_yaw   = kYaw.update(g_yaw_raw);

  float ax, ay, az;
  M5.IMU.getAccelData(&ax, &ay, &az);

  if (now - lastDrawMs >= DRAW_INTERVAL_MS) {
    lastDrawMs = now;

    if (M5.BtnB.wasPressed()) {
      g_pitch_zero = g_pitch;
      g_roll_zero  = g_roll;
      g_yaw_zero   = g_yaw;
    }

    float dispPitch = g_pitch - g_pitch_zero;
    float dispRoll  = g_roll  - g_roll_zero;
    float dispYaw   = wrap360(g_yaw - g_yaw_zero);

    // Full-screen horizon first
    drawHorizon(dispPitch, dispRoll);

    // Bank scale on top
    drawBankScale(dispRoll);

    // Flight director
    drawFlightDirector(dispPitch, dispRoll);

    // Flight path vector (bird)
    drawFlightPathVector(dispPitch, dispRoll, ax, ay);

    // Slip/skid ball
    updateAndDrawSlip(ay, az, dt);

    // Heading tape + info band
    drawHeadingTapeAndInfo(dispYaw, dispPitch, dispRoll);

    // Small crosshair in center
    g_canvas.drawLine(CX - 5, CY, CX + 5, CY, TFT_GREEN);
    g_canvas.drawLine(CX, CY - 5, CX, CY + 5, TFT_GREEN);

    g_canvas.pushSprite(0, 0);
  }
}
