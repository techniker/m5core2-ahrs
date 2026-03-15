/*
 * pfd.cpp – Primary Flight Display Renderer
 * Full aviation-grade PFD: horizon, pitch ladder, bank arc, speed tape
 * with V-speed color bands, altitude tape, VSI, heading tape, slip/skid,
 * flight path vector, G-meter, and status annunciations.
 */

#include "pfd.h"
#include "config.h"
#include <cmath>
#include <cstdio>

static TFT_eSprite canvas(&M5.Lcd);

// ─────────────────────────────────────────────────────────
// Utility
// ─────────────────────────────────────────────────────────

static inline float deg2rad(float d) { return d * (float)M_PI / 180.0f; }

static float wrapAngle360(float a) {
    while (a < 0.0f)    a += 360.0f;
    while (a >= 360.0f) a -= 360.0f;
    return a;
}

static float angleDiff(float a, float b) {
    float d = wrapAngle360(a) - wrapAngle360(b);
    if (d >  180.0f) d -= 360.0f;
    if (d < -180.0f) d += 360.0f;
    return d;
}

static void rotatePoint(float x, float y, float cx, float cy,
                         float cosR, float sinR, int &ox, int &oy) {
    float dx = x - cx, dy = y - cy;
    ox = (int)(cx + dx * cosR - dy * sinR);
    oy = (int)(cy + dx * sinR + dy * cosR);
}

// Clamp helper
static inline float clampf(float v, float lo, float hi) {
    return (v < lo) ? lo : (v > hi) ? hi : v;
}

// ─────────────────────────────────────────────────────────
// Slip/Skid Ball State
// ─────────────────────────────────────────────────────────

static float slipBallPos = 0.0f;
static float slipBallVel = 0.0f;

// ─────────────────────────────────────────────────────────
// Horizon + Pitch Ladder
// ─────────────────────────────────────────────────────────

static void drawHorizon(float pitch, float roll) {
    float rollRad = deg2rad(-roll);  // invert for correct horizon tilt
    float cosR = cosf(rollRad);
    float sinR = sinf(rollRad);

    // Direction vectors for horizon line
    float dX = cosR, dY = sinR;
    float nX = sinR, nY = -cosR;  // normal toward sky

    float pitchPx = pitch * PITCH_PX_PER_DEG;
    float hcx = (float)CX;
    float hcy = (float)CY + pitchPx;

    // Extend line and fill regions
    const float L = 450.0f;
    float p1x = hcx - dX * L, p1y = hcy - dY * L;
    float p2x = hcx + dX * L, p2y = hcy + dY * L;

    const float N = 450.0f;
    float s1x = p1x + nX * N, s1y = p1y + nY * N;
    float s2x = p2x + nX * N, s2y = p2y + nY * N;
    float g1x = p1x - nX * N, g1y = p1y - nY * N;
    float g2x = p2x - nX * N, g2y = p2y - nY * N;

    // Sky
    canvas.fillTriangle((int)p1x, (int)p1y, (int)p2x, (int)p2y, (int)s1x, (int)s1y, COL_SKY);
    canvas.fillTriangle((int)p2x, (int)p2y, (int)s2x, (int)s2y, (int)s1x, (int)s1y, COL_SKY);
    // Ground
    canvas.fillTriangle((int)p1x, (int)p1y, (int)p2x, (int)p2y, (int)g1x, (int)g1y, COL_GND);
    canvas.fillTriangle((int)p2x, (int)p2y, (int)g2x, (int)g2y, (int)g1x, (int)g1y, COL_GND);

    // Horizon line (thick)
    canvas.drawLine((int)p1x, (int)p1y, (int)p2x, (int)p2y, COL_HORIZON);
    canvas.drawLine((int)p1x, (int)p1y + 1, (int)p2x, (int)p2y + 1, COL_HORIZON);

    // ── Pitch Ladder ──
    for (int deg = -(int)PITCH_RANGE_DEG; deg <= (int)PITCH_RANGE_DEG; deg += 5) {
        if (deg == 0) continue;
        float deltaPx = (pitch - (float)deg) * PITCH_PX_PER_DEG;
        float ly = (float)CY + deltaPx;

        // Skip if way off screen
        if (ly < -60.0f || ly > INST_H + 60.0f) continue;

        bool isMajor = (deg % 10 == 0);
        float halfW  = isMajor ? 45.0f : 20.0f;

        int x1, y1, x2, y2;
        rotatePoint(CX - halfW, ly, CX, CY, cosR, sinR, x1, y1);
        rotatePoint(CX + halfW, ly, CX, CY, cosR, sinR, x2, y2);

        canvas.drawLine(x1, y1, x2, y2, COL_WHITE);

        // Chevron tips on major lines (point toward horizon)
        if (isMajor) {
            float tipDir = (deg > 0) ? 6.0f : -6.0f;  // point toward horizon
            int tx1, ty1, tx2, ty2;
            rotatePoint(CX - halfW, ly + tipDir, CX, CY, cosR, sinR, tx1, ty1);
            rotatePoint(CX + halfW, ly + tipDir, CX, CY, cosR, sinR, tx2, ty2);
            canvas.drawLine(x1, y1, tx1, ty1, COL_WHITE);
            canvas.drawLine(x2, y2, tx2, ty2, COL_WHITE);

            // Pitch value labels
            char buf[6];
            snprintf(buf, sizeof(buf), "%d", abs(deg));
            int lx, ly2;
            rotatePoint(CX - halfW - 18, ly, CX, CY, cosR, sinR, lx, ly2);
            canvas.setTextColor(COL_WHITE);
            canvas.setTextSize(1);
            canvas.setCursor(lx, ly2 - 3);
            canvas.print(buf);

            rotatePoint(CX + halfW + 4, ly, CX, CY, cosR, sinR, lx, ly2);
            canvas.setCursor(lx, ly2 - 3);
            canvas.print(buf);
        }
    }
}

// ─────────────────────────────────────────────────────────
// Aircraft Reference Symbol (fixed W-shape)
// ─────────────────────────────────────────────────────────

static void drawAircraftSymbol() {
    // W-shape aircraft reference
    int x = CX, y = CY;

    // Left wing
    canvas.drawLine(x - 35, y, x - 12, y, COL_YELLOW);
    canvas.drawLine(x - 35, y + 1, x - 12, y + 1, COL_YELLOW);
    // Left dip
    canvas.drawLine(x - 12, y, x - 6, y + 6, COL_YELLOW);
    canvas.drawLine(x - 12, y + 1, x - 6, y + 7, COL_YELLOW);
    // Center rise
    canvas.drawLine(x - 6, y + 6, x, y + 2, COL_YELLOW);
    canvas.drawLine(x - 6, y + 7, x, y + 3, COL_YELLOW);
    canvas.drawLine(x, y + 2, x + 6, y + 6, COL_YELLOW);
    canvas.drawLine(x, y + 3, x + 6, y + 7, COL_YELLOW);
    // Right dip
    canvas.drawLine(x + 6, y + 6, x + 12, y, COL_YELLOW);
    canvas.drawLine(x + 6, y + 7, x + 12, y + 1, COL_YELLOW);
    // Right wing
    canvas.drawLine(x + 12, y, x + 35, y, COL_YELLOW);
    canvas.drawLine(x + 12, y + 1, x + 35, y + 1, COL_YELLOW);

    // Small dot at center
    canvas.fillCircle(x, y, 2, COL_YELLOW);
}

// ─────────────────────────────────────────────────────────
// Bank Angle Arc
// ─────────────────────────────────────────────────────────

static void drawBankArc(float roll) {
    int cx = CX, cy = 8;

    // Arc tick marks
    const int marks[] = { -60, -45, -30, -20, -10, 0, 10, 20, 30, 45, 60 };
    for (int i = 0; i < 11; i++) {
        float aDeg = (float)marks[i];
        float aRad = deg2rad(aDeg - 90.0f);
        float ca = cosf(aRad), sa = sinf(aRad);

        bool isLong = (marks[i] == 0 || abs(marks[i]) == 30 ||
                       abs(marks[i]) == 60);
        bool isTri  = (marks[i] == 0);
        int len = isLong ? 10 : 6;

        int x1 = cx + (int)(BANK_ARC_R_INNER * ca);
        int y1 = cy + (int)(BANK_ARC_R_INNER * sa);
        int x2 = cx + (int)((BANK_ARC_R_INNER - len) * ca);
        int y2 = cy + (int)((BANK_ARC_R_INNER - len) * sa);

        canvas.drawLine(x1, y1, x2, y2, COL_WHITE);

        if (isTri) {
            // Triangle pointer at 0° (top)
            int tx = cx;
            int ty = cy - BANK_ARC_R_OUTER;
            canvas.fillTriangle(tx - 5, ty, tx + 5, ty, tx, ty + 7, COL_WHITE);
        }
    }

    // Moving bank pointer (triangle pointing inward)
    float bankRad = deg2rad(roll - 90.0f);
    float ca = cosf(bankRad), sa = sinf(bankRad);
    int px = cx + (int)((BANK_ARC_R_INNER + 2) * ca);
    int py = cy + (int)((BANK_ARC_R_INNER + 2) * sa);
    // Small triangle
    float perpX = -sa, perpY = ca;
    int t1x = px + (int)(5 * perpX), t1y = py + (int)(5 * perpY);
    int t2x = px - (int)(5 * perpX), t2y = py - (int)(5 * perpY);
    int t3x = cx + (int)((BANK_ARC_R_INNER - 4) * ca);
    int t3y = cy + (int)((BANK_ARC_R_INNER - 4) * sa);
    canvas.fillTriangle(t1x, t1y, t2x, t2y, t3x, t3y, COL_ORANGE);
}

// ─────────────────────────────────────────────────────────
// Slip/Skid Ball
// ─────────────────────────────────────────────────────────

static void drawSlipBall(float slipAngle, float dt) {
    int bcx = CX;
    int bcy = CY + INST_H / 2 - 18;

    // Tube outline
    canvas.drawRect(bcx - SLIP_BAR_W / 2, bcy - SLIP_BAR_H / 2,
                    SLIP_BAR_W, SLIP_BAR_H, COL_WHITE);
    // Center marks
    canvas.drawLine(bcx - SLIP_BALL_R - 1, bcy - SLIP_BAR_H / 2 - 2,
                    bcx - SLIP_BALL_R - 1, bcy + SLIP_BAR_H / 2 + 2, COL_WHITE);
    canvas.drawLine(bcx + SLIP_BALL_R + 1, bcy - SLIP_BAR_H / 2 - 2,
                    bcx + SLIP_BALL_R + 1, bcy + SLIP_BAR_H / 2 + 2, COL_WHITE);

    // Spring-damper physics
    float clampedAngle = clampf(slipAngle, -0.5f, 0.5f);
    float maxPx = (float)(SLIP_BAR_W / 2 - SLIP_BALL_R);
    float targetPos = (clampedAngle / 0.5f) * maxPx;

    float k = 25.0f, d = 8.0f;
    float acc = k * (targetPos - slipBallPos) - d * slipBallVel;
    slipBallVel += acc * dt;
    slipBallPos += slipBallVel * dt;
    slipBallPos = clampf(slipBallPos, -maxPx, maxPx);

    canvas.fillCircle(bcx + (int)slipBallPos, bcy, SLIP_BALL_R, COL_WHITE);
}

// ─────────────────────────────────────────────────────────
// Speed Tape (left side)
// ─────────────────────────────────────────────────────────

static void drawSpeedTape(float airspeed) {
    int tx = 0, ty = 0, tw = SPD_TAPE_W, th = INST_H;

    // Background
    canvas.fillRect(tx, ty, tw, th, COL_TAPE_BG);

    // V-speed color band (3px wide strip on right edge of tape)
    int bandX = tx + tw - 5;
    int bandW = 3;
    float spanKt = (float)th / SPD_PX_PER_KT / 2.0f;
    float topSpd = airspeed + spanKt;
    float botSpd = airspeed - spanKt;

    // Draw color arcs as vertical strips
    auto drawBand = [&](float vLow, float vHigh, uint16_t col) {
        float cLow  = clampf(vLow,  botSpd, topSpd);
        float cHigh = clampf(vHigh, botSpd, topSpd);
        if (cHigh <= cLow) return;
        int yTop = CY - (int)((cHigh - airspeed) * SPD_PX_PER_KT);
        int yBot = CY - (int)((cLow  - airspeed) * SPD_PX_PER_KT);
        if (yTop < ty) yTop = ty;
        if (yBot > ty + th) yBot = ty + th;
        if (yBot > yTop)
            canvas.fillRect(bandX, yTop, bandW, yBot - yTop, col);
    };

    drawBand(V_S0, V_FE, COL_SPD_WHITE);    // white arc (flap range)
    drawBand(V_S1, V_NO, COL_SPD_GREEN);    // green arc (normal)
    drawBand(V_NO, V_NE, COL_SPD_YELLOW);   // yellow arc (caution)
    // Red line at Vne
    {
        float vneY = CY - (V_NE - airspeed) * SPD_PX_PER_KT;
        if (vneY >= ty && vneY <= ty + th)
            canvas.drawLine(bandX - 2, (int)vneY, bandX + bandW + 1, (int)vneY, COL_RED);
    }

    // Tick marks and labels
    int stepMinor = 5, stepMajor = 10;
    int vStart = (int)(airspeed - spanKt) - stepMinor;
    int vEnd   = (int)(airspeed + spanKt) + stepMinor;

    for (int v = vStart; v <= vEnd; v += stepMinor) {
        if (v < 0) continue;
        int y = CY - (int)((v - airspeed) * SPD_PX_PER_KT);
        if (y < ty + 2 || y > ty + th - 2) continue;

        bool major = (v % stepMajor == 0);
        int len = major ? 10 : 5;
        canvas.drawLine(tx + tw - len - 6, y, tx + tw - 6, y, COL_WHITE);

        if (major) {
            char buf[6];
            snprintf(buf, sizeof(buf), "%3d", v);
            canvas.setTextColor(COL_WHITE, COL_TAPE_BG);
            canvas.setTextSize(1);
            canvas.setCursor(tx + 3, y - 3);
            canvas.print(buf);
        }
    }

    // Tape border
    canvas.drawLine(tx + tw - 1, ty, tx + tw - 1, ty + th, COL_DKGREY);

    // ── Current value window ──
    int winH = 22, winW = tw - 4;
    int winY = CY - winH / 2;
    int winX = tx + 2;

    // Pointer arrow on right edge
    canvas.fillTriangle(tx + tw - 1, CY - 6, tx + tw - 1, CY + 6,
                        tx + tw + 5, CY, COL_BLACK);

    canvas.fillRect(winX, winY, winW, winH, COL_BLACK);
    canvas.drawRect(winX, winY, winW, winH, COL_WHITE);

    char buf[8];
    snprintf(buf, sizeof(buf), "%3.0f", airspeed);
    canvas.setTextColor(COL_WHITE, COL_BLACK);
    canvas.setTextSize(2);
    canvas.setCursor(winX + 4, winY + 3);
    canvas.print(buf);

    // Unit label
    canvas.setTextSize(1);
    canvas.setTextColor(COL_WHITE, COL_TAPE_BG);
    canvas.setCursor(tx + 4, ty + 3);
    canvas.print("KTS");
}

// ─────────────────────────────────────────────────────────
// Altitude Tape (right side)
// ─────────────────────────────────────────────────────────

static void drawAltTape(float altitude) {
    int tx = TFT_W - ALT_TAPE_W - VSI_W;
    int ty = 0, tw = ALT_TAPE_W, th = INST_H;

    canvas.fillRect(tx, ty, tw, th, COL_TAPE_BG);

    float spanFt = (float)th / ALT_PX_PER_FT / 2.0f;

    // Tick marks: minor every 20ft, major every 100ft
    int vStart = ((int)(altitude - spanFt) / 20) * 20 - 20;
    int vEnd   = ((int)(altitude + spanFt) / 20) * 20 + 20;

    for (int h = vStart; h <= vEnd; h += 20) {
        int y = CY - (int)((h - altitude) * ALT_PX_PER_FT);
        if (y < ty + 2 || y > ty + th - 2) continue;

        bool major = (h % 100 == 0);
        int len = major ? 10 : 5;
        canvas.drawLine(tx + 4, y, tx + 4 + len, y, COL_WHITE);

        if (major) {
            char buf[8];
            if (abs(h) >= 10000)
                snprintf(buf, sizeof(buf), "%d", h / 1000);
            else
                snprintf(buf, sizeof(buf), "%d", h);
            canvas.setTextColor(COL_WHITE, COL_TAPE_BG);
            canvas.setTextSize(1);
            canvas.setCursor(tx + 16, y - 3);
            canvas.print(buf);
        }
    }

    // Tape border
    canvas.drawLine(tx, ty, tx, ty + th, COL_DKGREY);

    // ── Current altitude value window (centered on tape) ──
    int winH = 22, winW = tw - 2;
    int winY = CY - winH / 2;
    int winX = tx + 1;

    // Pointer arrow on left edge
    canvas.fillTriangle(tx, CY - 6, tx, CY + 6, tx - 5, CY, COL_BLACK);

    canvas.fillRect(winX, winY, winW, winH, COL_BLACK);
    canvas.drawRect(winX, winY, winW, winH, COL_WHITE);

    // Format altitude to fit: use size 1 font (6px/char) for large values
    char buf[10];
    int altInt = (int)roundf(altitude);
    snprintf(buf, sizeof(buf), "%d", altInt);
    int len = strlen(buf);

    if (len <= 4) {
        // Fits in size 2 (12px/char)
        canvas.setTextColor(COL_WHITE, COL_BLACK);
        canvas.setTextSize(2);
        int textW = len * 12;
        canvas.setCursor(winX + (winW - textW) / 2, winY + 3);
    } else {
        // Use size 1 for 5+ digits
        canvas.setTextColor(COL_WHITE, COL_BLACK);
        canvas.setTextSize(1);
        int textW = len * 6;
        canvas.setCursor(winX + (winW - textW) / 2, winY + 7);
    }
    canvas.print(buf);

    // Unit label
    canvas.setTextSize(1);
    canvas.setTextColor(COL_WHITE, COL_TAPE_BG);
    canvas.setCursor(tx + 10, ty + 3);
    canvas.print("FT");
}

// ─────────────────────────────────────────────────────────
// Vertical Speed Indicator (right edge, pointer-arc style)
// ─────────────────────────────────────────────────────────

static void drawVSI(float vspeed) {
    int vx = TFT_W - VSI_W;
    int vy = 0, vw = VSI_W, vh = INST_H;

    canvas.fillRect(vx, vy, vw, vh, COL_TAPE_BG);
    canvas.drawLine(vx, vy, vx, vy + vh, COL_DKGREY);

    int centerY = CY;

    // Scale marks at 0, +/-500, +/-1000, +/-1500, +/-2000
    const int marks[] = { -2000, -1500, -1000, -500, 0, 500, 1000, 1500, 2000 };
    for (int i = 0; i < 9; i++) {
        float fpm = (float)marks[i];
        // Non-linear scale: compress extremes
        float norm = fpm / VS_MAX_FPM;
        float py = centerY - norm * (float)(vh / 2 - 8);
        if ((int)py < vy + 2 || (int)py > vy + vh - 2) continue;

        bool major = (marks[i] % 1000 == 0);
        int len = major ? 8 : 4;
        canvas.drawLine(vx + 2, (int)py, vx + 2 + len, (int)py, COL_WHITE);

        if (major && marks[i] != 0) {
            char buf[4];
            snprintf(buf, sizeof(buf), "%d", abs(marks[i]) / 1000);
            canvas.setTextColor(COL_WHITE, COL_TAPE_BG);
            canvas.setTextSize(1);
            canvas.setCursor(vx + 11, (int)py - 3);
            canvas.print(buf);
        }
    }

    // Zero line
    canvas.drawLine(vx + 2, centerY, vx + vw - 2, centerY, COL_WHITE);

    // VS pointer (horizontal bar)
    float clampedVS = clampf(vspeed, -VS_MAX_FPM, VS_MAX_FPM);
    float norm = clampedVS / VS_MAX_FPM;
    int ptrY = centerY - (int)(norm * (float)(vh / 2 - 8));
    ptrY = (int)clampf((float)ptrY, (float)(vy + 3), (float)(vy + vh - 3));

    uint16_t vsCol = (fabsf(vspeed) > 1500.0f) ? COL_AMBER :
                     (fabsf(vspeed) > 500.0f)  ? COL_WHITE : COL_GREEN;
    canvas.fillRect(vx + 2, ptrY - 2, vw - 4, 4, vsCol);
}

// ─────────────────────────────────────────────────────────
// Heading Tape (bottom strip)
// ─────────────────────────────────────────────────────────

static void drawHeadingTape(float heading) {
    int bandY = TFT_H - HDG_TAPE_H;

    canvas.fillRect(0, bandY, TFT_W, HDG_TAPE_H, COL_PANEL);
    canvas.drawLine(0, bandY, TFT_W, bandY, COL_DKGREY);

    float hdg = wrapAngle360(heading);
    float spanDeg = TFT_W / (2.0f * HDG_PX_PER_DEG);

    // Cardinal direction labels
    static const char* cardinals[] = {
        "N", nullptr, nullptr, "E", nullptr, nullptr,
        "S", nullptr, nullptr, "W", nullptr, nullptr
    };

    int tickY = bandY + 5;
    for (int h = 0; h < 360; h += 5) {
        float d = angleDiff((float)h, hdg);
        if (d < -spanDeg || d > spanDeg) continue;

        int x = (int)(TFT_W / 2 + d * HDG_PX_PER_DEG);

        bool major  = (h % 10 == 0);
        bool label  = (h % 30 == 0);
        int len = major ? 10 : 5;

        canvas.drawLine(x, tickY, x, tickY + len, COL_WHITE);

        if (label) {
            int idx = h / 30;
            const char* card = cardinals[idx];
            char buf[6];
            if (card) {
                snprintf(buf, sizeof(buf), "%s", card);
            } else {
                snprintf(buf, sizeof(buf), "%02d", h / 10);
            }
            canvas.setTextColor(COL_WHITE, COL_PANEL);
            canvas.setTextSize(1);
            int tw = strlen(buf) * 6;
            canvas.setCursor(x - tw / 2, tickY + len + 2);
            canvas.print(buf);
        }
    }

    // Center lubber line (triangle pointer)
    int cx = TFT_W / 2;
    canvas.fillTriangle(cx - 5, bandY + 2, cx + 5, bandY + 2,
                        cx, bandY + 8, COL_ORANGE);

    // Digital heading readout
    char hbuf[6];
    snprintf(hbuf, sizeof(hbuf), "%03d", (int)roundf(hdg) % 360);
    canvas.setTextColor(COL_GREEN, COL_PANEL);
    canvas.setTextSize(2);
    int tw = 3 * 12;
    canvas.setCursor(cx - tw / 2, bandY + HDG_TAPE_H - 18);
    canvas.print(hbuf);

    // Degree symbol
    canvas.setTextSize(1);
    canvas.setCursor(cx + tw / 2 + 1, bandY + HDG_TAPE_H - 18);
    canvas.print("o");
}

// ─────────────────────────────────────────────────────────
// Status Annunciations (G-load, turn rate, battery, etc.)
// ─────────────────────────────────────────────────────────

static void drawAnnunciations(const SensorData &data) {
    // G-load (bottom-left of attitude area)
    {
        char buf[10];
        snprintf(buf, sizeof(buf), "%+4.1fG", data.gLoad);
        uint16_t col = (fabsf(data.gLoad - 1.0f) > 0.5f) ? COL_AMBER : COL_GREEN;
        canvas.setTextColor(col, COL_SKY);
        canvas.setTextSize(1);
        canvas.setCursor(SPD_TAPE_W + 4, INST_H - 12);
        canvas.print(buf);
    }

    // Turn rate indicator (small marks below bank arc)
    {
        float stdRate = data.turnRate / 3.0f;  // standard rate = 3 deg/s
        float barPx = clampf(stdRate, -1.5f, 1.5f) * 30.0f;
        int ty = 2;
        int barY = CY - BANK_ARC_R_INNER + 14;
        // Background bar
        canvas.drawLine(CX - 30, barY, CX + 30, barY, COL_DKGREY);
        // Center mark
        canvas.drawLine(CX, barY - 2, CX, barY + 2, COL_WHITE);
        // Rate marks at +/- standard rate
        canvas.drawLine(CX - 30, barY - 2, CX - 30, barY + 2, COL_WHITE);
        canvas.drawLine(CX + 30, barY - 2, CX + 30, barY + 2, COL_WHITE);
        // Pointer bar (works both directions from center)
        int ptrX = CX + (int)barPx;
        uint16_t rateCol = (fabsf(stdRate) > 1.0f) ? COL_AMBER : COL_MAGENTA;
        int x0 = (ptrX < CX) ? ptrX : CX;
        int w  = abs(ptrX - CX);
        if (w > 0)
            canvas.fillRect(x0, barY - 1, w, 3, rateCol);
    }

    // Battery voltage (bottom-left heading area)
    {
        char buf[10];
        snprintf(buf, sizeof(buf), "%.1fV", data.battVoltage);
        uint16_t col = (data.battVoltage < 3.4f) ? COL_RED :
                       (data.battVoltage < 3.6f) ? COL_AMBER : COL_GREEN;
        canvas.setTextColor(col, COL_PANEL);
        canvas.setTextSize(1);
        canvas.setCursor(4, TFT_H - 12);
        canvas.print(buf);
    }

    // Pitch / Roll numeric (top of altitude area)
    {
        char buf[12];
        canvas.setTextSize(1);
        canvas.setTextColor(COL_WHITE, COL_TAPE_BG);

        snprintf(buf, sizeof(buf), "P%+5.1f", data.pitch);
        canvas.setCursor(TFT_W - ALT_TAPE_W - VSI_W + 4, INST_H - 22);
        canvas.print(buf);

        snprintf(buf, sizeof(buf), "R%+5.1f", data.roll);
        canvas.setCursor(TFT_W - ALT_TAPE_W - VSI_W + 4, INST_H - 12);
        canvas.print(buf);
    }

    // Vertical speed numeric
    {
        char buf[10];
        snprintf(buf, sizeof(buf), "%+.0f", data.vspeed);
        canvas.setTextColor(COL_WHITE, COL_TAPE_BG);
        canvas.setTextSize(1);
        canvas.setCursor(TFT_W - VSI_W + 2, 3);
        canvas.print(buf);
    }
}

// ─────────────────────────────────────────────────────────
// Flight Path Vector (FPV / "Bird")
// ─────────────────────────────────────────────────────────

static void drawFPV(float pitch, float roll, float ax, float ay) {
    // Estimate FPV offset from accelerometer
    float fpvPitchOff = ax * 4.0f;  // approximate degrees
    float fpvYawOff   = -ay * 4.0f;

    float rollRad = deg2rad(-roll);
    float cosR = cosf(rollRad), sinR = sinf(rollRad);

    float fy = (float)CY + (pitch - fpvPitchOff) * PITCH_PX_PER_DEG;
    float fx = (float)CX + fpvYawOff * PITCH_PX_PER_DEG;

    // Clamp to attitude area
    fx = clampf(fx, SPD_TAPE_W + 15.0f, TFT_W - ALT_TAPE_W - VSI_W - 15.0f);
    fy = clampf(fy, 15.0f, INST_H - 15.0f);

    int ix = (int)fx, iy = (int)fy;

    // Circle
    canvas.drawCircle(ix, iy, 5, COL_GREEN);
    // Wings
    canvas.drawLine(ix - 14, iy, ix - 6, iy, COL_GREEN);
    canvas.drawLine(ix + 6,  iy, ix + 14, iy, COL_GREEN);
    // Tail
    canvas.drawLine(ix, iy - 8, ix, iy - 5, COL_GREEN);
}

// ─────────────────────────────────────────────────────────
// Public API
// ─────────────────────────────────────────────────────────

void PFD::init() {
    M5.Lcd.setRotation(1);
    M5.Axp.ScreenBreath(100);

    canvas.setColorDepth(16);
    canvas.createSprite(TFT_W, TFT_H);
    canvas.fillScreen(COL_BLACK);
    canvas.pushSprite(0, 0);
}

void PFD::drawSplash(const char* msg, int progress) {
    canvas.fillScreen(COL_BLACK);

    // Title
    canvas.setTextColor(COL_CYAN);
    canvas.setTextSize(2);
    canvas.setCursor(60, 60);
    canvas.print("AHRS / PFD");

    // Subtitle
    canvas.setTextSize(1);
    canvas.setTextColor(COL_WHITE);
    canvas.setCursor(80, 90);
    canvas.print("M5Stack Core2");

    // Status message
    canvas.setTextColor(COL_GREEN);
    canvas.setCursor(40, 130);
    canvas.print(msg);

    // Progress bar
    if (progress >= 0) {
        int barW = 200, barH = 8;
        int barX = (TFT_W - barW) / 2, barY = 160;
        canvas.drawRect(barX, barY, barW, barH, COL_DKGREY);
        int fillW = (int)(barW * clampf((float)progress / 100.0f, 0.0f, 1.0f));
        if (fillW > 0)
            canvas.fillRect(barX + 1, barY + 1, fillW - 2, barH - 2, COL_GREEN);
    }

    canvas.pushSprite(0, 0);
}

void PFD::draw(const SensorData &data) {
    // 1. Horizon + pitch ladder (fills attitude background)
    drawHorizon(data.pitch, data.roll);

    // 2. Bank angle arc
    drawBankArc(data.roll);

    // 3. Slip/skid ball
    drawSlipBall(data.slipAngle, data.dt);

    // 4. Aircraft reference symbol (fixed at center)
    drawAircraftSymbol();

    // 5. Flight path vector
    drawFPV(data.pitch, data.roll, data.accel[0], data.accel[1]);

    // 6. Speed tape (left, draws over horizon)
    drawSpeedTape(data.airspeed);

    // 7. Altitude tape + VSI (right, draws over horizon)
    drawAltTape(data.altitude);
    drawVSI(data.vspeed);

    // 8. Heading tape (bottom strip)
    drawHeadingTape(data.yaw);

    // 9. Annunciations (G-load, battery, pitch/roll digits, etc.)
    drawAnnunciations(data);

    // Push to display
    canvas.pushSprite(0, 0);
}
