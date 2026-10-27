/*
 * menu.cpp – Service menu state machine + calibration wizards + bottom bar.
 */

#include <M5Core2.h>
#include <Preferences.h>
#include <cstdio>
#include <cmath>
#include "menu.h"
#include "config.h"
#include "sensors.h"
#include "compass.h"
#include "logger.h"

namespace {

Menu::State g_state = Menu::STATE_PFD;
bool        g_dirty = true;             // request a full overlay redraw

// ── Main-menu list ──
struct MainItem {
    const char* label;
    Menu::State target;
};

constexpr MainItem MAIN_ITEMS[] = {
    { "Zero Attitude",      Menu::STATE_LEVEL_ZERO },
    { "Calibrate Gyro",     Menu::STATE_CAL_GYRO   },
    { "Calibrate Accel",    Menu::STATE_CAL_ACCEL  },
    { "Calibrate Compass",  Menu::STATE_CAL_COMPASS},
    { "Set Heading Ref",    Menu::STATE_HDG_SET    },
    { "Brightness",         Menu::STATE_BRIGHTNESS },
    { "System Info",        Menu::STATE_INFO       },
    { "Exit to PFD",        Menu::STATE_PFD        },
};
constexpr int MAIN_COUNT = (int)(sizeof(MAIN_ITEMS) / sizeof(MAIN_ITEMS[0]));

int g_mainIdx = 0;

// ── Accelerometer wizard state ──
int g_accelFace = 0;          // current face being asked

// ── Gyro cal trigger (set when user presses SELECT in CAL_GYRO) ──
bool g_runGyroCal = false;

// ── Accel cal result flash (shown briefly after finalize) ──
//   0 = none, 1 = success, 2 = failure
int  g_accelResult = 0;

// ── Brightness ──
int g_brightness = 100;

// ── Heading set ──
int g_hdgTarget = 0;          // target heading value in degrees

// ── Helpers ──
constexpr uint16_t COL_BG_OVERLAY = 0x10A2;

void drawCenteredText(TFT_eSprite &c, int y, const char* s,
                      uint16_t fg, uint16_t bg, int size) {
    c.setTextSize(size);
    c.setTextColor(fg, bg);
    int tw = (int)strlen(s) * 6 * size;
    c.setCursor((TFT_W - tw) / 2, y);
    c.print(s);
}

void drawHeader(TFT_eSprite &c, const char* title) {
    c.fillRect(0, 0, TFT_W, 22, COL_PANEL);
    c.drawLine(0, 22, TFT_W, 22, COL_DKGREY);
    drawCenteredText(c, 6, title, COL_CYAN, COL_PANEL, 1);
}

const char* labelA(Menu::State s) {
    switch (s) {
        case Menu::STATE_PFD:         return "MENU";
        case Menu::STATE_MAIN_MENU:   return "UP";
        case Menu::STATE_INFO:        return "BACK";
        case Menu::STATE_CAL_GYRO:    return "BACK";
        case Menu::STATE_CAL_ACCEL:   return "CANCEL";
        case Menu::STATE_CAL_COMPASS: return "CANCEL";
        case Menu::STATE_HDG_SET:     return "-";
        case Menu::STATE_BRIGHTNESS:  return "-";
        case Menu::STATE_LEVEL_ZERO:  return "CANCEL";
    }
    return "";
}
const char* labelB(Menu::State s) {
    switch (s) {
        case Menu::STATE_PFD:         return "ZERO";
        case Menu::STATE_MAIN_MENU:   return "SELECT";
        case Menu::STATE_INFO:        return "OK";
        case Menu::STATE_CAL_GYRO:    return "START";
        case Menu::STATE_CAL_ACCEL:   return "CAPTURE";
        case Menu::STATE_CAL_COMPASS: return "FINISH";
        case Menu::STATE_HDG_SET:     return "SET";
        case Menu::STATE_BRIGHTNESS:  return "SAVE";
        case Menu::STATE_LEVEL_ZERO:  return "OK";
    }
    return "";
}
const char* labelC(Menu::State s) {
    switch (s) {
        case Menu::STATE_PFD:         return "INFO";
        case Menu::STATE_MAIN_MENU:   return "DOWN";
        case Menu::STATE_INFO:        return "BACK";
        case Menu::STATE_CAL_GYRO:    return "BACK";
        case Menu::STATE_CAL_ACCEL:   return "SKIP";
        case Menu::STATE_CAL_COMPASS: return "BACK";
        case Menu::STATE_HDG_SET:     return "+";
        case Menu::STATE_BRIGHTNESS:  return "+";
        case Menu::STATE_LEVEL_ZERO:  return "BACK";
    }
    return "";
}

const char* faceLabel(int f) {
    switch (f) {
        case 0: return "SCREEN UP   (flat, face up)";
        case 1: return "SCREEN DOWN (flat, face down)";
        case 2: return "RIGHT EDGE UP";
        case 3: return "LEFT EDGE UP";
        case 4: return "TOP EDGE UP (nose up)";
        case 5: return "BOTTOM EDGE UP (nose down)";
    }
    return "?";
}

void enter(Menu::State s) {
    g_state = s;
    g_dirty = true;
    if (s == Menu::STATE_CAL_COMPASS) {
        Compass::beginCal();
    } else if (s == Menu::STATE_CAL_ACCEL) {
        Sensors::beginAccelCal();
        g_accelFace = 0;
        g_accelResult = 0;
    } else if (s == Menu::STATE_HDG_SET) {
        g_hdgTarget = 0;
    } else if (s == Menu::STATE_BRIGHTNESS) {
        Preferences p;
        if (p.begin(NVS_NAMESPACE, true)) {
            g_brightness = p.getInt(NVS_KEY_BRT, 100);
            p.end();
        }
        if (g_brightness < 20) g_brightness = 20;
        if (g_brightness > 100) g_brightness = 100;
        M5.Axp.ScreenBreath(g_brightness);
    }
}

void doGyroCal(TFT_eSprite &c) {
    drawHeader(c, "GYRO CALIBRATION");
    drawCenteredText(c, 60,  "Keep the device perfectly", COL_WHITE, COL_BG_OVERLAY, 1);
    drawCenteredText(c, 76,  "STILL on a level surface", COL_WHITE, COL_BG_OVERLAY, 1);
    drawCenteredText(c, 110, "Calibrating...", COL_GREEN, COL_BG_OVERLAY, 2);
    c.pushSprite(0, 0);

    Sensors::calibrateGyro();

    c.fillRect(0, 100, TFT_W, 40, COL_BG_OVERLAY);
    drawCenteredText(c, 110, "DONE", COL_GREEN, COL_BG_OVERLAY, 2);
    c.pushSprite(0, 0);
    delay(700);
}

}  // namespace

// ─────────────────────────────────────────────────────────
// Public API
// ─────────────────────────────────────────────────────────

void Menu::init() {
    g_state = STATE_PFD;
    g_dirty = true;
    Preferences p;
    if (p.begin(NVS_NAMESPACE, true)) {
        g_brightness = p.getInt(NVS_KEY_BRT, 100);
        p.end();
    }
    M5.Axp.ScreenBreath(g_brightness);
}

Menu::State Menu::state() { return g_state; }
bool Menu::active()       { return g_state != STATE_PFD; }

bool Menu::update() {
    // NOTE: M5.update() is called once per frame by main.cpp before this.
    bool changed = false;

    switch (g_state) {
        // ── PFD live ──
        case STATE_PFD: {
            if (M5.BtnA.wasPressed()) {
                enter(STATE_MAIN_MENU);
                return true;
            }
            if (M5.BtnB.wasPressed()) {
                Sensors::setZeroAttitude();
                changed = true;
            }
            if (M5.BtnC.wasPressed()) {
                enter(STATE_INFO);
                return true;
            }
        } break;

        // ── Main service menu ──
        case STATE_MAIN_MENU: {
            if (M5.BtnA.wasPressed()) {
                g_mainIdx = (g_mainIdx - 1 + MAIN_COUNT) % MAIN_COUNT;
                g_dirty = true; changed = true;
            }
            if (M5.BtnC.wasPressed()) {
                g_mainIdx = (g_mainIdx + 1) % MAIN_COUNT;
                g_dirty = true; changed = true;
            }
            if (M5.BtnB.wasPressed()) {
                enter(MAIN_ITEMS[g_mainIdx].target);
                return true;
            }
        } break;

        // ── System info ──
        case STATE_INFO: {
            if (M5.BtnB.wasPressed() ||
                M5.BtnA.wasPressed() ||
                M5.BtnC.wasPressed()) {
                enter(STATE_PFD);
                return true;
            }
            g_dirty = true; changed = true;     // info auto-refreshes
        } break;

        // ── Gyro cal ──
        case STATE_CAL_GYRO: {
            if (M5.BtnB.wasPressed()) {
                g_runGyroCal = true;
                g_dirty = true;
                return true;     // force draw → which runs the cal
            }
            if (M5.BtnA.wasPressed() || M5.BtnC.wasPressed()) {
                enter(STATE_MAIN_MENU);
                return true;
            }
        } break;

        // ── Accel cal ──
        case STATE_CAL_ACCEL: {
            // If a result is being shown, any button returns to main menu.
            if (g_accelResult != 0) {
                if (M5.BtnA.wasPressed() ||
                    M5.BtnB.wasPressed() ||
                    M5.BtnC.wasPressed()) {
                    enter(STATE_MAIN_MENU);
                    return true;
                }
                break;
            }
            if (M5.BtnA.wasPressed()) {
                enter(STATE_MAIN_MENU);
                return true;
            }
            if (M5.BtnC.wasPressed()) {
                // Skip to next face
                g_accelFace++;
                if (g_accelFace >= (int)Sensors::FACE_COUNT) g_accelFace = 0;
                g_dirty = true; changed = true;
            }
            if (M5.BtnB.wasPressed()) {
                Sensors::captureAccelFace((Sensors::AccelFace)g_accelFace);
                g_accelFace++;
                if (g_accelFace >= (int)Sensors::FACE_COUNT) {
                    g_accelResult = Sensors::finalizeAccelCal() ? 1 : 2;
                    // Stay on STATE_CAL_ACCEL — overlay shows the result
                    // for a moment, then auto-returns on next button press
                    // or via timed transition handled in drawOverlay.
                    g_accelFace = (int)Sensors::FACE_COUNT;
                    g_dirty = true;
                    return true;
                }
                g_dirty = true; changed = true;
            }
        } break;

        // ── Compass cal ──
        case STATE_CAL_COMPASS: {
            if (M5.BtnA.wasPressed()) {
                enter(STATE_MAIN_MENU);
                return true;
            }
            if (M5.BtnB.wasPressed()) {
                if (Compass::present()) {
                    // Refuse to commit if the sphere isn't reasonably covered.
                    if (Compass::calCoveragePct() >= 70.0f &&
                        Compass::finalizeCal()) {
                        enter(STATE_MAIN_MENU);
                        return true;
                    }
                    // else: stay on screen, keep collecting samples
                } else {
                    // No mag: treat current yaw as 0° (magnetic north).
                    Sensors::setHeadingOffset(Sensors::getData().yaw +
                                              Sensors::getHeadingOffset());
                    enter(STATE_MAIN_MENU);
                    return true;
                }
            }
            if (Compass::present()) {
                Compass::updateCal();
            }
            g_dirty = true; changed = true;
        } break;

        // ── Heading set ──
        case STATE_HDG_SET: {
            if (M5.BtnA.wasPressed()) {
                g_hdgTarget = (g_hdgTarget + 360 - 5) % 360;
                g_dirty = true; changed = true;
            }
            if (M5.BtnC.wasPressed()) {
                g_hdgTarget = (g_hdgTarget + 5) % 360;
                g_dirty = true; changed = true;
            }
            if (M5.BtnB.wasPressed()) {
                // Make current yaw report g_hdgTarget
                float curRaw = Sensors::getData().yaw +
                               Sensors::getHeadingOffset();
                float off = curRaw - (float)g_hdgTarget;
                Sensors::setHeadingOffset(off);
                enter(STATE_MAIN_MENU);
                return true;
            }
        } break;

        // ── Brightness ──
        case STATE_BRIGHTNESS: {
            if (M5.BtnA.wasPressed()) {
                g_brightness = (g_brightness > 25) ? g_brightness - 5 : 20;
                M5.Axp.ScreenBreath(g_brightness);
                g_dirty = true; changed = true;
            }
            if (M5.BtnC.wasPressed()) {
                g_brightness = (g_brightness < 100) ? g_brightness + 5 : 100;
                M5.Axp.ScreenBreath(g_brightness);
                g_dirty = true; changed = true;
            }
            if (M5.BtnB.wasPressed()) {
                Preferences p;
                if (p.begin(NVS_NAMESPACE, false)) {
                    p.putInt(NVS_KEY_BRT, g_brightness);
                    p.end();
                }
                enter(STATE_MAIN_MENU);
                return true;
            }
        } break;

        // ── Confirm zero ──
        case STATE_LEVEL_ZERO: {
            if (M5.BtnB.wasPressed()) {
                Sensors::setZeroAttitude();
                enter(STATE_MAIN_MENU);
                return true;
            }
            if (M5.BtnA.wasPressed() || M5.BtnC.wasPressed()) {
                enter(STATE_MAIN_MENU);
                return true;
            }
        } break;
    }

    return changed;
}

// ─────────────────────────────────────────────────────────
// Bottom button-label bar (always painted over PFD)
// ─────────────────────────────────────────────────────────

void Menu::drawMenuBar(TFT_eSprite &c) {
    c.fillRect(0, MENU_BAR_Y, TFT_W, MENU_BAR_H, COL_MENU_BG);
    c.drawLine(0, MENU_BAR_Y, TFT_W, MENU_BAR_Y, COL_DKGREY);

    // Vertical dividers between the three button zones (~10,120,200,310)
    c.drawLine(120, MENU_BAR_Y + 2, 120, MENU_BAR_Y + MENU_BAR_H - 2, COL_DKGREY);
    c.drawLine(200, MENU_BAR_Y + 2, 200, MENU_BAR_Y + MENU_BAR_H - 2, COL_DKGREY);

    auto cell = [&](int xCenter, const char* s) {
        if (!s || !*s) return;
        c.setTextSize(1);
        c.setTextColor(COL_WHITE, COL_MENU_BG);
        int tw = (int)strlen(s) * 6;
        int y = MENU_BAR_Y + (MENU_BAR_H - 8) / 2;
        c.setCursor(xCenter - tw / 2, y);
        c.print(s);
    };

    cell(60,  labelA(g_state));
    cell(160, labelB(g_state));
    cell(265, labelC(g_state));
}

// ─────────────────────────────────────────────────────────
// Full-screen overlay (only when state != PFD)
// ─────────────────────────────────────────────────────────

void Menu::drawOverlay(TFT_eSprite &c) {
    if (!active()) return;

    // Wipe to overlay background
    c.fillRect(0, 0, TFT_W, MENU_BAR_Y, COL_BG_OVERLAY);

    switch (g_state) {
        case STATE_MAIN_MENU: {
            drawHeader(c, "SERVICE MENU");
            int yStart = 30, rowH = 22;
            for (int i = 0; i < MAIN_COUNT; i++) {
                int y = yStart + i * rowH;
                bool sel = (i == g_mainIdx);
                if (sel) {
                    c.fillRect(8, y - 2, TFT_W - 16, rowH - 2, COL_MENU_HI);
                }
                c.setTextSize(1);
                c.setTextColor(sel ? COL_WHITE : COL_LTGREY,
                               sel ? COL_MENU_HI : COL_BG_OVERLAY);
                c.setCursor(20, y + 4);
                c.print(MAIN_ITEMS[i].label);
                if (sel) {
                    c.setCursor(TFT_W - 24, y + 4);
                    c.print(">");
                }
            }
        } break;

        case STATE_INFO: {
            drawHeader(c, "SYSTEM INFO");
            const SensorData &d = Sensors::getData();
            char buf[40];
            c.setTextSize(1);
            c.setTextColor(COL_WHITE, COL_BG_OVERLAY);

            int y = 34;
            snprintf(buf, sizeof(buf), "Firmware:  %s %s", FW_NAME, FW_VERSION);
            c.setCursor(12, y); c.print(buf); y += 14;

            snprintf(buf, sizeof(buf), "Battery:   %.2f V", d.battVoltage);
            c.setCursor(12, y); c.print(buf); y += 14;

            snprintf(buf, sizeof(buf), "IMU temp:  %.1f C", d.imuTemp);
            c.setCursor(12, y); c.print(buf); y += 14;

            snprintf(buf, sizeof(buf), "Pitch %+5.1f  Roll %+5.1f", d.pitch, d.roll);
            c.setCursor(12, y); c.print(buf); y += 14;

            snprintf(buf, sizeof(buf), "Yaw:       %5.1f deg", d.yaw);
            c.setCursor(12, y); c.print(buf); y += 14;

            snprintf(buf, sizeof(buf), "Hdg ref:   %5.1f deg",
                     Sensors::getHeadingOffset());
            c.setCursor(12, y); c.print(buf); y += 14;

            snprintf(buf, sizeof(buf), "Compass:   %s%s",
                     Compass::driverName(),
                     Compass::isCalibrated() ? " (cal)" : "");
            c.setCursor(12, y); c.print(buf); y += 14;

            const auto &ac = Sensors::getAccelCal();
            snprintf(buf, sizeof(buf), "Accel cal: %s",
                     ac.valid ? "yes" : "no (defaults)");
            c.setCursor(12, y); c.print(buf); y += 14;

            snprintf(buf, sizeof(buf), "SD log:    %s",
                     Logger::isActive() ? Logger::getFilename() : "off");
            c.setCursor(12, y); c.print(buf);
        } break;

        case STATE_CAL_GYRO: {
            if (g_runGyroCal) {
                g_runGyroCal = false;
                doGyroCal(c);
                enter(STATE_MAIN_MENU);
                drawOverlay(c);
                return;
            }
            drawHeader(c, "GYRO CALIBRATION");
            drawCenteredText(c, 60, "Press SELECT (middle)",
                             COL_WHITE, COL_BG_OVERLAY, 1);
            drawCenteredText(c, 76, "to start. Keep device STILL.",
                             COL_WHITE, COL_BG_OVERLAY, 1);
        } break;

        case STATE_CAL_ACCEL: {
            drawHeader(c, "ACCEL CALIBRATION");

            if (g_accelResult != 0) {
                // Show result screen
                const auto &ac = Sensors::getAccelCal();
                if (g_accelResult == 1) {
                    drawCenteredText(c, 50, "CALIBRATION OK",
                                     COL_GREEN, COL_BG_OVERLAY, 2);
                    char buf[40];
                    snprintf(buf, sizeof(buf),
                             "bias  X%+.3f Y%+.3f Z%+.3f",
                             ac.bias[0], ac.bias[1], ac.bias[2]);
                    drawCenteredText(c, 90, buf, COL_LTGREY, COL_BG_OVERLAY, 1);
                    snprintf(buf, sizeof(buf),
                             "scale X%.3f Y%.3f Z%.3f",
                             ac.scale[0], ac.scale[1], ac.scale[2]);
                    drawCenteredText(c, 106, buf, COL_LTGREY, COL_BG_OVERLAY, 1);
                } else {
                    drawCenteredText(c, 50, "CAL FAILED",
                                     COL_RED, COL_BG_OVERLAY, 2);
                    drawCenteredText(c, 90,
                                     "Bias or scale out of range.",
                                     COL_WHITE, COL_BG_OVERLAY, 1);
                    drawCenteredText(c, 106,
                                     "Re-run with cleaner orientations.",
                                     COL_LTGREY, COL_BG_OVERLAY, 1);
                }
                drawCenteredText(c, 150,
                                 "Press any button to continue.",
                                 COL_LTGREY, COL_BG_OVERLAY, 1);
                break;
            }

            char buf[48];
            snprintf(buf, sizeof(buf), "Face %d of %d",
                     g_accelFace + 1, (int)Sensors::FACE_COUNT);
            drawCenteredText(c, 32, buf, COL_CYAN, COL_BG_OVERLAY, 1);

            drawCenteredText(c, 56, "Orient device:",
                             COL_LTGREY, COL_BG_OVERLAY, 1);
            drawCenteredText(c, 72, faceLabel(g_accelFace),
                             COL_WHITE, COL_BG_OVERLAY, 1);
            drawCenteredText(c, 96, "Hold steady, then CAPTURE",
                             COL_LTGREY, COL_BG_OVERLAY, 1);

            // Progress dots
            int yDots = 130;
            for (int i = 0; i < (int)Sensors::FACE_COUNT; i++) {
                int x = TFT_W / 2 - (int)Sensors::FACE_COUNT * 7 + i * 14;
                bool done = Sensors::isAccelFaceCaptured((Sensors::AccelFace)i);
                bool cur  = (i == g_accelFace);
                uint16_t col = done ? COL_GREEN : (cur ? COL_AMBER : COL_DKGREY);
                c.fillCircle(x, yDots, 4, col);
            }

            // Live accel readout (helps user verify orientation)
            const SensorData &d = Sensors::getData();
            snprintf(buf, sizeof(buf), "X%+.2f  Y%+.2f  Z%+.2f",
                     d.accel[0], d.accel[1], d.accel[2]);
            drawCenteredText(c, 152, buf, COL_LTGREY, COL_BG_OVERLAY, 1);
        } break;

        case STATE_CAL_COMPASS: {
            drawHeader(c, "COMPASS CALIBRATION");
            if (!Compass::present()) {
                drawCenteredText(c, 50, "No magnetometer detected.",
                                 COL_AMBER, COL_BG_OVERLAY, 1);
                drawCenteredText(c, 70, "FINISH = set current yaw",
                                 COL_WHITE, COL_BG_OVERLAY, 1);
                drawCenteredText(c, 86, "       as MAGNETIC NORTH (0)",
                                 COL_WHITE, COL_BG_OVERLAY, 1);
                drawCenteredText(c, 120, "Point device NORTH,",
                                 COL_LTGREY, COL_BG_OVERLAY, 1);
                drawCenteredText(c, 136, "then press FINISH.",
                                 COL_LTGREY, COL_BG_OVERLAY, 1);
            } else {
                drawCenteredText(c, 36, "Rotate the device slowly",
                                 COL_WHITE, COL_BG_OVERLAY, 1);
                drawCenteredText(c, 52, "through ALL orientations.",
                                 COL_WHITE, COL_BG_OVERLAY, 1);
                drawCenteredText(c, 68, "Cover the full sphere.",
                                 COL_LTGREY, COL_BG_OVERLAY, 1);

                int pct = (int)Compass::calCoveragePct();
                char buf[24];
                snprintf(buf, sizeof(buf), "Coverage: %d%%", pct);
                drawCenteredText(c, 110, buf, COL_GREEN, COL_BG_OVERLAY, 2);

                int barW = 220, barH = 10;
                int bx = (TFT_W - barW) / 2, by = 140;
                c.drawRect(bx, by, barW, barH, COL_LTGREY);
                int fillW = (barW - 2) * pct / 100;
                if (fillW > 0)
                    c.fillRect(bx + 1, by + 1, fillW, barH - 2,
                               pct >= 70 ? COL_GREEN : COL_AMBER);

                drawCenteredText(c, 160,
                                 (pct >= 70) ? "OK to FINISH" :
                                               "keep rotating...",
                                 (pct >= 70) ? COL_GREEN : COL_LTGREY,
                                 COL_BG_OVERLAY, 1);
            }
        } break;

        case STATE_HDG_SET: {
            drawHeader(c, "SET HEADING");
            drawCenteredText(c, 36,
                             "Point device at known heading,",
                             COL_WHITE, COL_BG_OVERLAY, 1);
            drawCenteredText(c, 52,
                             "use -/+ to dial, then SET.",
                             COL_LTGREY, COL_BG_OVERLAY, 1);

            char buf[16];
            snprintf(buf, sizeof(buf), "%03d deg", g_hdgTarget);
            c.setTextSize(4);
            int tw = (int)strlen(buf) * 6 * 4;
            c.setTextColor(COL_GREEN, COL_BG_OVERLAY);
            c.setCursor((TFT_W - tw) / 2, 90);
            c.print(buf);

            snprintf(buf, sizeof(buf), "current: %5.1f",
                     Sensors::getData().yaw);
            drawCenteredText(c, 150, buf, COL_LTGREY, COL_BG_OVERLAY, 1);
        } break;

        case STATE_BRIGHTNESS: {
            drawHeader(c, "BRIGHTNESS");
            char buf[16];
            snprintf(buf, sizeof(buf), "%d %%", g_brightness);
            c.setTextSize(4);
            int tw = (int)strlen(buf) * 6 * 4;
            c.setTextColor(COL_CYAN, COL_BG_OVERLAY);
            c.setCursor((TFT_W - tw) / 2, 70);
            c.print(buf);

            int barW = 240, barH = 14;
            int bx = (TFT_W - barW) / 2, by = 130;
            c.drawRect(bx, by, barW, barH, COL_LTGREY);
            int fillW = (barW - 2) * g_brightness / 100;
            if (fillW > 0)
                c.fillRect(bx + 1, by + 1, fillW, barH - 2, COL_CYAN);
        } break;

        case STATE_LEVEL_ZERO: {
            drawHeader(c, "ZERO ATTITUDE");
            drawCenteredText(c, 50, "Place device level, then OK.",
                             COL_WHITE, COL_BG_OVERLAY, 1);
            const SensorData &d = Sensors::getData();
            char buf[24];
            snprintf(buf, sizeof(buf), "Pitch %+5.1f  Roll %+5.1f",
                     d.pitch, d.roll);
            drawCenteredText(c, 96, buf, COL_GREEN, COL_BG_OVERLAY, 2);
        } break;

        default: break;
    }

    g_dirty = false;
}
