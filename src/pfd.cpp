/*
 * pfd.cpp – Primary Flight Display Renderer
 *
 * Bugfix highlights vs previous revision:
 *   - FPV axis-swap: now uses longitudinal accel for pitch, lateral for yaw.
 *   - Horizon overdraw is bounded to the attitude window so the heading tape
 *     and menu bar are not painted over every frame.
 *   - Pitch ladder labels are clipped to the attitude window so they stop
 *     bleeding under the bank arc / outside the speed/alt tapes.
 *   - Bank pointer triangle uses a canonical CCW winding; previous code
 *     produced degenerate triangles at extreme bank angles.
 */

#include "pfd.h"
#include "config.h"
#include <cmath>
#include <cstdio>

namespace {
TFT_eSprite g_canvas(&M5.Lcd);

// Slip ball spring-damper state (carried frame-to-frame).
float g_slipPos = 0.0f;
float g_slipVel = 0.0f;

constexpr float kPI = 3.14159265358979f;

inline float deg2rad(float d) { return d * kPI / 180.0f; }

inline float wrapAngle360(float a) {
    while (a < 0.0f)    a += 360.0f;
    while (a >= 360.0f) a -= 360.0f;
    return a;
}

inline float angleDiff(float a, float b) {
    float d = wrapAngle360(a) - wrapAngle360(b);
    if (d >  180.0f) d -= 360.0f;
    if (d < -180.0f) d += 360.0f;
    return d;
}

inline void rotatePoint(float x, float y, float cx, float cy,
                        float cosR, float sinR, int &ox, int &oy) {
    float dx = x - cx, dy = y - cy;
    ox = (int)(cx + dx * cosR - dy * sinR);
    oy = (int)(cy + dx * sinR + dy * cosR);
}

inline float clampf(float v, float lo, float hi) {
    return (v < lo) ? lo : (v > hi) ? hi : v;
}

// ─────────────────────────────────────────────────────────
// Horizon + Pitch Ladder
//
// Drawing strategy: we still overdraw two big triangles (sky/ground), but
// the long axis is sized to just cover the attitude window plus a small
// margin so we don't waste fillRect on the heading tape / menu bar below.
// ─────────────────────────────────────────────────────────

void drawHorizon(float pitch, float roll) {
    float rollRad = deg2rad(-roll);
    float cosR = cosf(rollRad);
    float sinR = sinf(rollRad);

    float dX = cosR, dY = sinR;        // along the horizon
    float nX = sinR, nY = -cosR;       // toward sky (perpendicular)

    float pitchPx = pitch * PITCH_PX_PER_DEG;
    float hcx = (float)CX;
    float hcy = (float)CY + pitchPx;

    // Sized so the rotated rectangle fully covers the *attitude window*
    // (INST_H tall, full screen wide). Slight margin avoids seam pixels.
    const float L = (float)TFT_W + 40.0f;
    const float N = (float)INST_H + 40.0f;

    float p1x = hcx - dX * L, p1y = hcy - dY * L;
    float p2x = hcx + dX * L, p2y = hcy + dY * L;

    float s1x = p1x + nX * N, s1y = p1y + nY * N;
    float s2x = p2x + nX * N, s2y = p2y + nY * N;
    float g1x = p1x - nX * N, g1y = p1y - nY * N;
    float g2x = p2x - nX * N, g2y = p2y - nY * N;

    // Sky
    g_canvas.fillTriangle((int)p1x, (int)p1y, (int)p2x, (int)p2y,
                          (int)s1x, (int)s1y, COL_SKY);
    g_canvas.fillTriangle((int)p2x, (int)p2y, (int)s2x, (int)s2y,
                          (int)s1x, (int)s1y, COL_SKY);
    // Ground
    g_canvas.fillTriangle((int)p1x, (int)p1y, (int)p2x, (int)p2y,
                          (int)g1x, (int)g1y, COL_GND);
    g_canvas.fillTriangle((int)p2x, (int)p2y, (int)g2x, (int)g2y,
                          (int)g1x, (int)g1y, COL_GND);

    // Horizon line (thick)
    g_canvas.drawLine((int)p1x, (int)p1y,     (int)p2x, (int)p2y,     COL_HORIZON);
    g_canvas.drawLine((int)p1x, (int)p1y + 1, (int)p2x, (int)p2y + 1, COL_HORIZON);

    // ── Pitch Ladder ──
    for (int deg = -(int)PITCH_RANGE_DEG; deg <= (int)PITCH_RANGE_DEG; deg += 5) {
        if (deg == 0) continue;
        float deltaPx = (pitch - (float)deg) * PITCH_PX_PER_DEG;
        float ly = (float)CY + deltaPx;

        if (ly < -40.0f || ly > (float)INST_H + 40.0f) continue;

        bool isMajor = (deg % 10 == 0);
        float halfW  = isMajor ? 45.0f : 20.0f;

        int x1, y1, x2, y2;
        rotatePoint(CX - halfW, ly, CX, CY, cosR, sinR, x1, y1);
        rotatePoint(CX + halfW, ly, CX, CY, cosR, sinR, x2, y2);

        // Skip the segment if both endpoints fall inside the tape
        // gutters or outside the attitude window — keeps labels from
        // bleeding under the speed/alt tapes.
        const int xLo = SPD_TAPE_W + 2;
        const int xHi = TFT_W - ALT_TAPE_W - VSI_W - 2;
        if ((x1 < xLo && x2 < xLo) || (x1 > xHi && x2 > xHi)) continue;
        if (y1 < 0 && y2 < 0) continue;
        if (y1 > INST_H && y2 > INST_H) continue;

        g_canvas.drawLine(x1, y1, x2, y2, COL_WHITE);

        if (isMajor) {
            float tipDir = (deg > 0) ? 6.0f : -6.0f;  // chevron toward horizon
            int tx1, ty1, tx2, ty2;
            rotatePoint(CX - halfW, ly + tipDir, CX, CY, cosR, sinR, tx1, ty1);
            rotatePoint(CX + halfW, ly + tipDir, CX, CY, cosR, sinR, tx2, ty2);
            g_canvas.drawLine(x1, y1, tx1, ty1, COL_WHITE);
            g_canvas.drawLine(x2, y2, tx2, ty2, COL_WHITE);

            char buf[6];
            snprintf(buf, sizeof(buf), "%d", abs(deg));
            int lx, ly2;
            rotatePoint(CX - halfW - 18, ly, CX, CY, cosR, sinR, lx, ly2);
            if (lx >= xLo && lx <= xHi - 8 && ly2 >= 0 && ly2 <= INST_H - 8) {
                g_canvas.setTextColor(COL_WHITE);
                g_canvas.setTextSize(1);
                g_canvas.setCursor(lx, ly2 - 3);
                g_canvas.print(buf);
            }
            rotatePoint(CX + halfW + 4, ly, CX, CY, cosR, sinR, lx, ly2);
            if (lx >= xLo && lx <= xHi - 8 && ly2 >= 0 && ly2 <= INST_H - 8) {
                g_canvas.setCursor(lx, ly2 - 3);
                g_canvas.print(buf);
            }
        }
    }
}

// ─────────────────────────────────────────────────────────
// Aircraft Reference Symbol (fixed W-shape)
// ─────────────────────────────────────────────────────────

void drawAircraftSymbol() {
    int x = CX, y = CY;
    g_canvas.drawLine(x - 35, y,     x - 12, y,     COL_YELLOW);
    g_canvas.drawLine(x - 35, y + 1, x - 12, y + 1, COL_YELLOW);
    g_canvas.drawLine(x - 12, y,     x - 6,  y + 6, COL_YELLOW);
    g_canvas.drawLine(x - 12, y + 1, x - 6,  y + 7, COL_YELLOW);
    g_canvas.drawLine(x - 6,  y + 6, x,      y + 2, COL_YELLOW);
    g_canvas.drawLine(x - 6,  y + 7, x,      y + 3, COL_YELLOW);
    g_canvas.drawLine(x,      y + 2, x + 6,  y + 6, COL_YELLOW);
    g_canvas.drawLine(x,      y + 3, x + 6,  y + 7, COL_YELLOW);
    g_canvas.drawLine(x + 6,  y + 6, x + 12, y,     COL_YELLOW);
    g_canvas.drawLine(x + 6,  y + 7, x + 12, y + 1, COL_YELLOW);
    g_canvas.drawLine(x + 12, y,     x + 35, y,     COL_YELLOW);
    g_canvas.drawLine(x + 12, y + 1, x + 35, y + 1, COL_YELLOW);
    g_canvas.fillCircle(x, y, 2, COL_YELLOW);
}

// ─────────────────────────────────────────────────────────
// Bank Angle Arc
// ─────────────────────────────────────────────────────────

void drawBankArc(float roll) {
    int cx = CX, cy = 8;

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

        g_canvas.drawLine(x1, y1, x2, y2, COL_WHITE);

        if (isTri) {
            int tx = cx;
            int ty = cy - BANK_ARC_R_OUTER;
            g_canvas.fillTriangle(tx - 5, ty, tx + 5, ty, tx, ty + 7, COL_WHITE);
        }
    }

    // Moving pointer (arrowhead pointing inward toward attitude centre).
    // Compute three vertices on the inside of the inner arc with a
    // canonical CCW winding so fillTriangle never produces a degenerate.
    float bankRad = deg2rad(roll - 90.0f);
    float ca = cosf(bankRad), sa = sinf(bankRad);
    float perpX = -sa, perpY = ca;

    float tipR  = (float)(BANK_ARC_R_INNER - 4);
    float baseR = (float)(BANK_ARC_R_INNER + 2);

    int tipX  = cx + (int)(tipR  * ca);
    int tipY  = cy + (int)(tipR  * sa);
    int baseAx = cx + (int)(baseR * ca + 5 * perpX);
    int baseAy = cy + (int)(baseR * sa + 5 * perpY);
    int baseBx = cx + (int)(baseR * ca - 5 * perpX);
    int baseBy = cy + (int)(baseR * sa - 5 * perpY);

    g_canvas.fillTriangle(tipX, tipY, baseAx, baseAy, baseBx, baseBy, COL_ORANGE);
}

// ─────────────────────────────────────────────────────────
// Slip/Skid Ball
// ─────────────────────────────────────────────────────────

void drawSlipBall(float slipAngle, float dt) {
    int bcx = CX;
    int bcy = INST_H - 16;

    g_canvas.drawRect(bcx - SLIP_BAR_W / 2, bcy - SLIP_BAR_H / 2,
                      SLIP_BAR_W, SLIP_BAR_H, COL_WHITE);
    g_canvas.drawLine(bcx - SLIP_BALL_R - 1, bcy - SLIP_BAR_H / 2 - 2,
                      bcx - SLIP_BALL_R - 1, bcy + SLIP_BAR_H / 2 + 2, COL_WHITE);
    g_canvas.drawLine(bcx + SLIP_BALL_R + 1, bcy - SLIP_BAR_H / 2 - 2,
                      bcx + SLIP_BALL_R + 1, bcy + SLIP_BAR_H / 2 + 2, COL_WHITE);

    float clampedAngle = clampf(slipAngle, -0.5f, 0.5f);
    float maxPx = (float)(SLIP_BAR_W / 2 - SLIP_BALL_R);
    float targetPos = (clampedAngle / 0.5f) * maxPx;

    float k = 25.0f, d = 8.0f;
    float acc = k * (targetPos - g_slipPos) - d * g_slipVel;
    g_slipVel += acc * dt;
    g_slipPos += g_slipVel * dt;
    g_slipPos = clampf(g_slipPos, -maxPx, maxPx);

    g_canvas.fillCircle(bcx + (int)g_slipPos, bcy, SLIP_BALL_R, COL_WHITE);
}

// ─────────────────────────────────────────────────────────
// Speed Tape (left side)
// ─────────────────────────────────────────────────────────

void drawSpeedTape(float airspeed) {
    int tx = 0, ty = 0, tw = SPD_TAPE_W, th = INST_H;

    g_canvas.fillRect(tx, ty, tw, th, COL_TAPE_BG);

    int bandX = tx + tw - 5;
    int bandW = 3;
    float spanKt = (float)th / SPD_PX_PER_KT / 2.0f;
    float topSpd = airspeed + spanKt;
    float botSpd = airspeed - spanKt;

    auto drawBand = [&](float vLow, float vHigh, uint16_t col) {
        float cLow  = clampf(vLow,  botSpd, topSpd);
        float cHigh = clampf(vHigh, botSpd, topSpd);
        if (cHigh <= cLow) return;
        int yTop = CY - (int)((cHigh - airspeed) * SPD_PX_PER_KT);
        int yBot = CY - (int)((cLow  - airspeed) * SPD_PX_PER_KT);
        if (yTop < ty) yTop = ty;
        if (yBot > ty + th) yBot = ty + th;
        if (yBot > yTop)
            g_canvas.fillRect(bandX, yTop, bandW, yBot - yTop, col);
    };

    drawBand(V_S0, V_FE, COL_SPD_WHITE);
    drawBand(V_S1, V_NO, COL_SPD_GREEN);
    drawBand(V_NO, V_NE, COL_SPD_YELLOW);
    {
        float vneY = CY - (V_NE - airspeed) * SPD_PX_PER_KT;
        if (vneY >= ty && vneY <= ty + th)
            g_canvas.drawLine(bandX - 2, (int)vneY, bandX + bandW + 1, (int)vneY, COL_RED);
    }

    int stepMinor = 5, stepMajor = 10;
    int vStart = (int)(airspeed - spanKt) - stepMinor;
    int vEnd   = (int)(airspeed + spanKt) + stepMinor;

    for (int v = vStart; v <= vEnd; v += stepMinor) {
        if (v < 0) continue;
        int y = CY - (int)((v - airspeed) * SPD_PX_PER_KT);
        if (y < ty + 2 || y > ty + th - 2) continue;

        bool major = (v % stepMajor == 0);
        int len = major ? 10 : 5;
        g_canvas.drawLine(tx + tw - len - 6, y, tx + tw - 6, y, COL_WHITE);

        if (major) {
            char buf[6];
            snprintf(buf, sizeof(buf), "%3d", v);
            g_canvas.setTextColor(COL_WHITE, COL_TAPE_BG);
            g_canvas.setTextSize(1);
            g_canvas.setCursor(tx + 3, y - 3);
            g_canvas.print(buf);
        }
    }

    g_canvas.drawLine(tx + tw - 1, ty, tx + tw - 1, ty + th, COL_DKGREY);

    int winH = 22, winW = tw - 4;
    int winY = CY - winH / 2;
    int winX = tx + 2;

    g_canvas.fillTriangle(tx + tw - 1, CY - 6, tx + tw - 1, CY + 6,
                          tx + tw + 5, CY, COL_BLACK);

    g_canvas.fillRect(winX, winY, winW, winH, COL_BLACK);
    g_canvas.drawRect(winX, winY, winW, winH, COL_WHITE);

    char buf[8];
    snprintf(buf, sizeof(buf), "%3.0f", airspeed);
    g_canvas.setTextColor(COL_WHITE, COL_BLACK);
    g_canvas.setTextSize(2);
    g_canvas.setCursor(winX + 4, winY + 3);
    g_canvas.print(buf);

    g_canvas.setTextSize(1);
    g_canvas.setTextColor(COL_WHITE, COL_TAPE_BG);
    g_canvas.setCursor(tx + 4, ty + 3);
    g_canvas.print("KTS");
}

// ─────────────────────────────────────────────────────────
// Altitude Tape (right side)
// ─────────────────────────────────────────────────────────

void drawAltTape(float altitude) {
    int tx = TFT_W - ALT_TAPE_W - VSI_W;
    int ty = 0, tw = ALT_TAPE_W, th = INST_H;

    g_canvas.fillRect(tx, ty, tw, th, COL_TAPE_BG);

    float spanFt = (float)th / ALT_PX_PER_FT / 2.0f;

    int vStart = ((int)(altitude - spanFt) / 20) * 20 - 20;
    int vEnd   = ((int)(altitude + spanFt) / 20) * 20 + 20;

    for (int h = vStart; h <= vEnd; h += 20) {
        int y = CY - (int)((h - altitude) * ALT_PX_PER_FT);
        if (y < ty + 2 || y > ty + th - 2) continue;

        bool major = (h % 100 == 0);
        int len = major ? 10 : 5;
        g_canvas.drawLine(tx + 4, y, tx + 4 + len, y, COL_WHITE);

        if (major) {
            char buf[8];
            snprintf(buf, sizeof(buf), "%d", h);
            g_canvas.setTextColor(COL_WHITE, COL_TAPE_BG);
            g_canvas.setTextSize(1);
            g_canvas.setCursor(tx + 16, y - 3);
            g_canvas.print(buf);
        }
    }

    g_canvas.drawLine(tx, ty, tx, ty + th, COL_DKGREY);

    int winH = 22, winW = tw - 2;
    int winY = CY - winH / 2;
    int winX = tx + 1;

    g_canvas.fillTriangle(tx, CY - 6, tx, CY + 6, tx - 5, CY, COL_BLACK);

    g_canvas.fillRect(winX, winY, winW, winH, COL_BLACK);
    g_canvas.drawRect(winX, winY, winW, winH, COL_WHITE);

    char buf[10];
    int altInt = (int)roundf(altitude);
    snprintf(buf, sizeof(buf), "%d", altInt);
    int len = (int)strlen(buf);

    if (len <= 4) {
        g_canvas.setTextColor(COL_WHITE, COL_BLACK);
        g_canvas.setTextSize(2);
        int textW = len * 12;
        g_canvas.setCursor(winX + (winW - textW) / 2, winY + 3);
    } else {
        g_canvas.setTextColor(COL_WHITE, COL_BLACK);
        g_canvas.setTextSize(1);
        int textW = len * 6;
        g_canvas.setCursor(winX + (winW - textW) / 2, winY + 7);
    }
    g_canvas.print(buf);

    g_canvas.setTextSize(1);
    g_canvas.setTextColor(COL_WHITE, COL_TAPE_BG);
    g_canvas.setCursor(tx + 10, ty + 3);
    g_canvas.print("FT");
}

// ─────────────────────────────────────────────────────────
// Vertical Speed Indicator
// ─────────────────────────────────────────────────────────

void drawVSI(float vspeed) {
    int vx = TFT_W - VSI_W;
    int vy = 0, vw = VSI_W, vh = INST_H;

    g_canvas.fillRect(vx, vy, vw, vh, COL_TAPE_BG);
    g_canvas.drawLine(vx, vy, vx, vy + vh, COL_DKGREY);

    int centerY = CY;

    const int marks[] = { -2000, -1500, -1000, -500, 0, 500, 1000, 1500, 2000 };
    for (int i = 0; i < 9; i++) {
        float fpm = (float)marks[i];
        float norm = fpm / VS_MAX_FPM;
        float py = centerY - norm * (float)(vh / 2 - 8);
        if ((int)py < vy + 2 || (int)py > vy + vh - 2) continue;

        bool major = (marks[i] % 1000 == 0);
        int len = major ? 8 : 4;
        g_canvas.drawLine(vx + 2, (int)py, vx + 2 + len, (int)py, COL_WHITE);

        if (major && marks[i] != 0) {
            char buf[4];
            snprintf(buf, sizeof(buf), "%d", abs(marks[i]) / 1000);
            g_canvas.setTextColor(COL_WHITE, COL_TAPE_BG);
            g_canvas.setTextSize(1);
            g_canvas.setCursor(vx + 11, (int)py - 3);
            g_canvas.print(buf);
        }
    }

    g_canvas.drawLine(vx + 2, centerY, vx + vw - 2, centerY, COL_WHITE);

    float clampedVS = clampf(vspeed, -VS_MAX_FPM, VS_MAX_FPM);
    float norm = clampedVS / VS_MAX_FPM;
    int ptrY = centerY - (int)(norm * (float)(vh / 2 - 8));
    ptrY = (int)clampf((float)ptrY, (float)(vy + 3), (float)(vy + vh - 3));

    uint16_t vsCol = (fabsf(vspeed) > 1500.0f) ? COL_AMBER :
                     (fabsf(vspeed) > 500.0f)  ? COL_WHITE : COL_GREEN;
    g_canvas.fillRect(vx + 2, ptrY - 2, vw - 4, 4, vsCol);
}

// ─────────────────────────────────────────────────────────
// Heading Tape
// ─────────────────────────────────────────────────────────

void drawHeadingTape(float heading) {
    int bandY = HDG_TAPE_Y;

    g_canvas.fillRect(0, bandY, TFT_W, HDG_TAPE_H, COL_PANEL);
    g_canvas.drawLine(0, bandY, TFT_W, bandY, COL_DKGREY);

    float hdg = wrapAngle360(heading);
    float spanDeg = TFT_W / (2.0f * HDG_PX_PER_DEG);

    static const char* cardinals[] = {
        "N", nullptr, nullptr, "E", nullptr, nullptr,
        "S", nullptr, nullptr, "W", nullptr, nullptr
    };

    int tickY = bandY + 3;
    for (int h = 0; h < 360; h += 5) {
        float d = angleDiff((float)h, hdg);
        if (d < -spanDeg || d > spanDeg) continue;

        int x = (int)(TFT_W / 2 + d * HDG_PX_PER_DEG);

        bool major = (h % 10 == 0);
        bool label = (h % 30 == 0);
        int len = major ? 8 : 4;

        g_canvas.drawLine(x, tickY, x, tickY + len, COL_WHITE);

        if (label) {
            int idx = h / 30;
            const char* card = cardinals[idx];
            char buf[6];
            if (card)  snprintf(buf, sizeof(buf), "%s",  card);
            else       snprintf(buf, sizeof(buf), "%02d", h / 10);
            g_canvas.setTextColor(COL_WHITE, COL_PANEL);
            g_canvas.setTextSize(1);
            int tw = (int)strlen(buf) * 6;
            g_canvas.setCursor(x - tw / 2, tickY + len + 1);
            g_canvas.print(buf);
        }
    }

    int cx = TFT_W / 2;
    g_canvas.fillTriangle(cx - 5, bandY + 1, cx + 5, bandY + 1,
                          cx, bandY + 7, COL_ORANGE);

    char hbuf[6];
    snprintf(hbuf, sizeof(hbuf), "%03d", (int)roundf(hdg) % 360);
    g_canvas.setTextColor(COL_GREEN, COL_PANEL);
    g_canvas.setTextSize(2);
    int tw = 3 * 12;
    g_canvas.setCursor(cx - tw / 2, bandY + HDG_TAPE_H - 14);
    g_canvas.print(hbuf);
}

// ─────────────────────────────────────────────────────────
// Annunciations
// ─────────────────────────────────────────────────────────

void drawAnnunciations(const SensorData &data) {
    char buf[16];

    // G-load
    snprintf(buf, sizeof(buf), "%+4.1fG", data.gLoad);
    uint16_t gCol = (fabsf(data.gLoad - 1.0f) > 0.5f) ? COL_AMBER : COL_GREEN;
    g_canvas.setTextColor(gCol, COL_SKY);
    g_canvas.setTextSize(1);
    g_canvas.setCursor(SPD_TAPE_W + 4, INST_H - 10);
    g_canvas.print(buf);

    // Turn-rate bar (below bank arc)
    float stdRate = data.turnRate / 3.0f;
    float barPx = clampf(stdRate, -1.5f, 1.5f) * 30.0f;
    int barY = CY - BANK_ARC_R_INNER + 14;
    g_canvas.drawLine(CX - 30, barY, CX + 30, barY, COL_DKGREY);
    g_canvas.drawLine(CX, barY - 2, CX, barY + 2, COL_WHITE);
    g_canvas.drawLine(CX - 30, barY - 2, CX - 30, barY + 2, COL_WHITE);
    g_canvas.drawLine(CX + 30, barY - 2, CX + 30, barY + 2, COL_WHITE);
    int ptrX = CX + (int)barPx;
    uint16_t rateCol = (fabsf(stdRate) > 1.0f) ? COL_AMBER : COL_MAGENTA;
    int x0 = (ptrX < CX) ? ptrX : CX;
    int w  = abs(ptrX - CX);
    if (w > 0) g_canvas.fillRect(x0, barY - 1, w, 3, rateCol);

    // Battery (heading tape left side)
    snprintf(buf, sizeof(buf), "%.1fV", data.battVoltage);
    uint16_t bCol = (data.battVoltage < 3.4f) ? COL_RED :
                    (data.battVoltage < 3.6f) ? COL_AMBER : COL_GREEN;
    g_canvas.setTextColor(bCol, COL_PANEL);
    g_canvas.setCursor(4, HDG_TAPE_Y + HDG_TAPE_H - 10);
    g_canvas.print(buf);

    // Pitch / Roll digits (near alt tape inner edge)
    g_canvas.setTextColor(COL_WHITE, COL_TAPE_BG);
    snprintf(buf, sizeof(buf), "P%+5.1f", data.pitch);
    g_canvas.setCursor(TFT_W - ALT_TAPE_W - VSI_W + 4, INST_H - 20);
    g_canvas.print(buf);
    snprintf(buf, sizeof(buf), "R%+5.1f", data.roll);
    g_canvas.setCursor(TFT_W - ALT_TAPE_W - VSI_W + 4, INST_H - 10);
    g_canvas.print(buf);

    // VSI numeric
    snprintf(buf, sizeof(buf), "%+.0f", data.vspeed);
    g_canvas.setTextColor(COL_WHITE, COL_TAPE_BG);
    g_canvas.setCursor(TFT_W - VSI_W + 2, 3);
    g_canvas.print(buf);
}

// ─────────────────────────────────────────────────────────
// Flight Path Vector
// ─────────────────────────────────────────────────────────

void drawFPV(float pitch, float /*roll*/, float lateral, float longitudinal) {
    // Bug fix: previously these were swapped. Pitch offset comes from
    // *longitudinal* accel (forward/back), yaw offset from *lateral*.
    float fpvPitchOff = longitudinal * 4.0f;
    float fpvYawOff   = -lateral     * 4.0f;

    float fy = (float)CY + (pitch - fpvPitchOff) * PITCH_PX_PER_DEG;
    float fx = (float)CX + fpvYawOff * PITCH_PX_PER_DEG;

    fx = clampf(fx, (float)SPD_TAPE_W + 15.0f,
                    (float)(TFT_W - ALT_TAPE_W - VSI_W - 15));
    fy = clampf(fy, 15.0f, (float)INST_H - 15.0f);

    int ix = (int)fx, iy = (int)fy;
    g_canvas.drawCircle(ix, iy, 5, COL_GREEN);
    g_canvas.drawLine(ix - 14, iy, ix - 6,  iy, COL_GREEN);
    g_canvas.drawLine(ix + 6,  iy, ix + 14, iy, COL_GREEN);
    g_canvas.drawLine(ix, iy - 8, ix, iy - 5, COL_GREEN);
}

}  // namespace

// ─────────────────────────────────────────────────────────
// Public API
// ─────────────────────────────────────────────────────────

void PFD::init() {
    M5.Lcd.setRotation(1);
    M5.Axp.ScreenBreath(100);
    g_canvas.setColorDepth(16);
    g_canvas.createSprite(TFT_W, TFT_H);
    g_canvas.fillScreen(COL_BLACK);
    g_canvas.pushSprite(0, 0);
}

TFT_eSprite& PFD::canvas()  { return g_canvas; }
void         PFD::present() { g_canvas.pushSprite(0, 0); }

void PFD::drawSplash(const char* msg, int progress) {
    g_canvas.fillScreen(COL_BLACK);

    g_canvas.setTextColor(COL_CYAN);
    g_canvas.setTextSize(2);
    g_canvas.setCursor(60, 60);
    g_canvas.print("AHRS / PFD");

    g_canvas.setTextSize(1);
    g_canvas.setTextColor(COL_WHITE);
    g_canvas.setCursor(80, 90);
    g_canvas.print("M5Stack Core2");

    g_canvas.setTextColor(COL_GREEN);
    g_canvas.setCursor(40, 130);
    g_canvas.print(msg);

    if (progress >= 0) {
        int barW = 200, barH = 8;
        int barX = (TFT_W - barW) / 2, barY = 160;
        g_canvas.drawRect(barX, barY, barW, barH, COL_DKGREY);
        int fillW = (int)((float)barW * clampf((float)progress / 100.0f, 0.0f, 1.0f));
        if (fillW > 0)
            g_canvas.fillRect(barX + 1, barY + 1, fillW - 2, barH - 2, COL_GREEN);
    }

    g_canvas.pushSprite(0, 0);
}

void PFD::draw(const SensorData &data) {
    drawHorizon(data.pitch, data.roll);
    drawBankArc(data.roll);
    drawSlipBall(data.slipAngle, data.dt);
    drawAircraftSymbol();
    // FPV: pass display-frame lateral (accel[0]) and longitudinal (accel[1])
    drawFPV(data.pitch, data.roll, data.accel[0], data.accel[1]);
    drawSpeedTape(data.airspeed);
    drawAltTape(data.altitude);
    drawVSI(data.vspeed);
    drawHeadingTape(data.yaw);
    drawAnnunciations(data);
    // Menu bar / overlay is painted by main.cpp after this call, before present().
}
