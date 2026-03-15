/*
 * M5Stack Core2 – Aviation-Grade AHRS / Primary Flight Display
 * Bjoern Heller <tec att sixtopia.net>
 *
 * Architecture:
 *   - Madgwick AHRS filter (quaternion-based, adaptive beta)
 *   - Automatic gyro bias calibration at startup
 *   - 200 Hz sensor/AHRS update, 30 Hz display refresh
 *   - Modular: config.h, ahrs.h/cpp, sensors.h/cpp, pfd.h/cpp
 *
 * Sensors used:
 *   - MPU6886 6-axis IMU (accelerometer + gyroscope)
 *   - AXP192 power management (battery voltage)
 *
 * Controls:
 *   - Button B (middle): zero attitude reference
 *   - Button A (left):   (reserved)
 *   - Button C (right):  (reserved)
 *
 * Data inputs (airspeed, altitude, vspeed) default to 0 and can
 * be driven by external sensors / serial / GPS as needed.
 */

#include <M5Core2.h>
#include "config.h"
#include "ahrs.h"
#include "sensors.h"
#include "pfd.h"
#include "logger.h"

static MadgwickAHRS ahrs;
static unsigned long lastDisplayMs = 0;
static unsigned long lastAhrsUs    = 0;
static unsigned long lastLogMs     = 0;

// Log at 10 Hz (every 100ms) — good balance of detail vs file size
static constexpr unsigned long LOG_INTERVAL_MS = 100;

void setup() {
    M5.begin(true, true, true, true);

    PFD::init();
    PFD::drawSplash("Initializing sensors...", 0);

    Sensors::init();

    // Gyro bias + accel average calibration (keep device still)
    PFD::drawSplash("Gyro calibration - keep still", 10);
    Sensors::calibrateGyro();

    // Seed the AHRS quaternion directly from the averaged gravity vector.
    // This gives correct pitch/roll instantly — no convergence drift.
    ahrs.begin(AHRS_BETA_DEFAULT);
    float calAx, calAy, calAz;
    Sensors::getCalAccel(calAx, calAy, calAz);
    ahrs.initFromAccel(calAx, calAy, calAz);

    PFD::drawSplash("Stabilizing AHRS...", 60);

    // Run the filter briefly to let gyro integration settle
    unsigned long startMs = millis();
    while (millis() - startMs < 300) {
        Sensors::readAndUpdate(ahrs);
        delay(5);
    }

    // Initialize SD card logger (silently skipped if no card)
    Logger::init();

    if (Logger::isActive()) {
        PFD::drawSplash("SD logging active", 90);
        delay(300);
    }

    PFD::drawSplash("Ready", 100);
    delay(200);

    // Set current (already stable) attitude as zero reference
    Sensors::setZeroAttitude();

    lastDisplayMs = millis();
    lastAhrsUs    = micros();
    lastLogMs     = millis();
}

void loop() {
    // ── High-rate: sensor read + AHRS update (~200 Hz) ──
    unsigned long nowUs = micros();
    if (nowUs - lastAhrsUs >= AHRS_INTERVAL_US) {
        lastAhrsUs = nowUs;
        Sensors::readAndUpdate(ahrs);
    }

    // ── Display refresh (~30 Hz) ──
    unsigned long nowMs = millis();
    if (nowMs - lastDisplayMs >= DISPLAY_INTERVAL_MS) {
        lastDisplayMs = nowMs;

        M5.update();

        // Button B: re-zero attitude
        if (M5.BtnB.wasPressed()) {
            Sensors::setZeroAttitude();
        }

        PFD::draw(Sensors::getData());
    }

    // ── SD card logging (~10 Hz) ──
    if (nowMs - lastLogMs >= LOG_INTERVAL_MS) {
        lastLogMs = nowMs;
        Logger::log(Sensors::getData());
    }
}
