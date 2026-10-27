/*
 * M5Stack Core2 – Aviation-Grade AHRS / Primary Flight Display
 * Bjoern Heller <tec att sixtopia.net>
 *
 * Loop layout:
 *   - 200 Hz: sensor + AHRS update
 *   - 30 Hz : display refresh (PFD + menu overlay)
 *   - 10 Hz : SD CSV logging
 *
 * Buttons (touch zones below the LCD, three white dots):
 *   - BtnA (left)   → MENU on PFD, UP/CANCEL inside menus
 *   - BtnB (middle) → ZERO on PFD, SELECT inside menus
 *   - BtnC (right)  → INFO on PFD, DOWN inside menus
 */

#include <M5Core2.h>
#include "config.h"
#include "ahrs.h"
#include "sensors.h"
#include "pfd.h"
#include "logger.h"
#include "menu.h"
#include "compass.h"

namespace {
MadgwickAHRS  g_ahrs;
unsigned long g_lastDisplayMs = 0;
unsigned long g_lastAhrsUs    = 0;
unsigned long g_lastLogMs     = 0;
}

void setup() {
    M5.begin(true, true, true, true);

    PFD::init();
    PFD::drawSplash("Initializing sensors...", 0);

    Sensors::init();
    Compass::begin();
    Menu::init();

    PFD::drawSplash("Gyro calibration - keep still", 10);
    Sensors::calibrateGyro();

    g_ahrs.begin(AHRS_BETA_DEFAULT);
    float calAx, calAy, calAz;
    Sensors::getCalAccel(calAx, calAy, calAz);
    g_ahrs.initFromAccel(calAx, calAy, calAz);

    PFD::drawSplash("Stabilizing AHRS...", 60);

    unsigned long startMs = millis();
    while (millis() - startMs < 300) {
        Sensors::readAndUpdate(g_ahrs);
        delay(5);
    }

    Logger::init();
    if (Logger::isActive()) {
        PFD::drawSplash("SD logging active", 90);
        delay(300);
    }

    PFD::drawSplash("Ready", 100);
    delay(200);

    Sensors::setZeroAttitude();

    g_lastDisplayMs = millis();
    g_lastAhrsUs    = micros();
    g_lastLogMs     = millis();
}

void loop() {
    // ── High-rate sensor + AHRS ──
    unsigned long nowUs = micros();
    if (nowUs - g_lastAhrsUs >= AHRS_INTERVAL_US) {
        g_lastAhrsUs = nowUs;
        Sensors::readAndUpdate(g_ahrs);
    }

    // ── Display refresh ──
    unsigned long nowMs = millis();
    if (nowMs - g_lastDisplayMs >= DISPLAY_INTERVAL_MS) {
        g_lastDisplayMs = nowMs;

        M5.update();            // single source of button state for this tick
        Menu::update();         // route buttons → menu state machine

        TFT_eSprite &c = PFD::canvas();

        if (Menu::active()) {
            // Full-screen menu/calibration overlay — only menu bar of the
            // PFD remains visible at the bottom.
            Menu::drawOverlay(c);
        } else {
            PFD::draw(Sensors::getData());
        }
        Menu::drawMenuBar(c);
        PFD::present();
    }

    // ── SD logging ──
    if (nowMs - g_lastLogMs >= LOG_INTERVAL_MS_CFG) {
        g_lastLogMs = nowMs;
        Logger::log(Sensors::getData());
    }
}
