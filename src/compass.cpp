/*
 * compass.cpp – Magnetometer driver + tilt-compensated heading
 */

#include <M5Core2.h>
#include <Wire.h>
#include <Preferences.h>
#include <cmath>
#include "compass.h"
#include "config.h"
#include "sensors.h"

namespace {

constexpr uint8_t BMM150_ADDR  = 0x10;
constexpr uint8_t QMC5883_ADDR = 0x0D;
constexpr uint8_t HMC5883_ADDR = 0x1E;

Compass::Driver  g_drv = Compass::DRV_NONE;
MagCalibration   g_mCal = {};

// Tumble cal running min/max
float g_calMin[3] = { 99999.0f,  99999.0f,  99999.0f};
float g_calMax[3] = {-99999.0f, -99999.0f, -99999.0f};
unsigned long g_calStartMs = 0;

inline bool probe(uint8_t addr) {
    Wire.beginTransmission(addr);
    return (Wire.endTransmission() == 0);
}

// ── BMM150 ──
bool initBmm150() {
    // Power on
    Wire.beginTransmission(BMM150_ADDR);
    Wire.write(0x4B); Wire.write(0x01);
    Wire.endTransmission();
    delay(10);
    // Set op mode normal, ODR 30 Hz
    Wire.beginTransmission(BMM150_ADDR);
    Wire.write(0x4C); Wire.write(0x38);
    Wire.endTransmission();
    return true;
}
bool readBmm150(float &mx, float &my, float &mz) {
    Wire.beginTransmission(BMM150_ADDR);
    Wire.write(0x42);
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom((int)BMM150_ADDR, 8) != 8) return false;
    // Read each byte into its own variable — under gnu++11 the order of
    // sub-expression reads in `(Wire.read()<<8)|Wire.read()` is unspecified.
    uint8_t xl = Wire.read(), xh = Wire.read();
    uint8_t yl = Wire.read(), yh = Wire.read();
    uint8_t zl = Wire.read(), zh = Wire.read();
    Wire.read(); Wire.read();          // RHALL low/high (unused here)
    int16_t rx = (int16_t)((xh << 8) | xl);
    int16_t ry = (int16_t)((yh << 8) | yl);
    int16_t rz = (int16_t)((zh << 8) | zl);
    rx >>= 3; ry >>= 3; rz >>= 1;     // strip status bits (sign-preserved)
    mx = (float)rx; my = (float)ry; mz = (float)rz;
    return true;
}

// ── QMC5883L ──
bool initQmc5883() {
    Wire.beginTransmission(QMC5883_ADDR);
    Wire.write(0x0B); Wire.write(0x01);  // SET/RESET period
    Wire.endTransmission();
    Wire.beginTransmission(QMC5883_ADDR);
    Wire.write(0x09); Wire.write(0x1D);  // continuous, 200 Hz, 8 G, OSR=512
    Wire.endTransmission();
    return true;
}
bool readQmc5883(float &mx, float &my, float &mz) {
    Wire.beginTransmission(QMC5883_ADDR);
    Wire.write(0x00);
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom((int)QMC5883_ADDR, 6) != 6) return false;
    uint8_t xl = Wire.read(), xh = Wire.read();
    uint8_t yl = Wire.read(), yh = Wire.read();
    uint8_t zl = Wire.read(), zh = Wire.read();
    int16_t rx = (int16_t)((xh << 8) | xl);
    int16_t ry = (int16_t)((yh << 8) | yl);
    int16_t rz = (int16_t)((zh << 8) | zl);
    mx = (float)rx; my = (float)ry; mz = (float)rz;
    return true;
}

// ── HMC5883L ──
bool initHmc5883() {
    Wire.beginTransmission(HMC5883_ADDR);
    Wire.write(0x00); Wire.write(0x70);  // 8 avg, 15 Hz
    Wire.endTransmission();
    Wire.beginTransmission(HMC5883_ADDR);
    Wire.write(0x01); Wire.write(0xA0);  // gain 5
    Wire.endTransmission();
    Wire.beginTransmission(HMC5883_ADDR);
    Wire.write(0x02); Wire.write(0x00);  // continuous
    Wire.endTransmission();
    return true;
}
bool readHmc5883(float &mx, float &my, float &mz) {
    Wire.beginTransmission(HMC5883_ADDR);
    Wire.write(0x03);
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom((int)HMC5883_ADDR, 6) != 6) return false;
    // HMC5883L data order on the wire is X-MSB, X-LSB, Z-MSB, Z-LSB, Y-MSB, Y-LSB.
    uint8_t xh = Wire.read(), xl = Wire.read();
    uint8_t zh = Wire.read(), zl = Wire.read();
    uint8_t yh = Wire.read(), yl = Wire.read();
    int16_t rx = (int16_t)((xh << 8) | xl);
    int16_t ry = (int16_t)((yh << 8) | yl);
    int16_t rz = (int16_t)((zh << 8) | zl);
    mx = (float)rx; my = (float)ry; mz = (float)rz;
    return true;
}

}  // namespace

bool Compass::begin() {
    g_drv = DRV_NONE;
    // External I²C (port A) already initialised by M5.begin() on pins 32/33.

    if (probe(BMM150_ADDR)) {
        initBmm150();
        g_drv = DRV_BMM150;
    } else if (probe(QMC5883_ADDR)) {
        initQmc5883();
        g_drv = DRV_QMC5883L;
    } else if (probe(HMC5883_ADDR)) {
        initHmc5883();
        g_drv = DRV_HMC5883L;
    }

    g_mCal = {};
    g_mCal.scale[0] = g_mCal.scale[1] = g_mCal.scale[2] = 1.0f;
    loadCal();

    return g_drv != DRV_NONE;
}

bool Compass::present()   { return g_drv != DRV_NONE; }
Compass::Driver Compass::driver() { return g_drv; }

const char* Compass::driverName() {
    switch (g_drv) {
        case DRV_BMM150:   return "BMM150";
        case DRV_QMC5883L: return "QMC5883L";
        case DRV_HMC5883L: return "HMC5883L";
        default:           return "none";
    }
}

bool Compass::readRaw(float &mx, float &my, float &mz) {
    switch (g_drv) {
        case DRV_BMM150:   return readBmm150(mx, my, mz);
        case DRV_QMC5883L: return readQmc5883(mx, my, mz);
        case DRV_HMC5883L: return readHmc5883(mx, my, mz);
        default: return false;
    }
}

bool Compass::readCalibrated(float &mx, float &my, float &mz) {
    if (!readRaw(mx, my, mz)) return false;
    if (g_mCal.valid) {
        mx = (mx - g_mCal.bias[0]) * g_mCal.scale[0];
        my = (my - g_mCal.bias[1]) * g_mCal.scale[1];
        mz = (mz - g_mCal.bias[2]) * g_mCal.scale[2];
    }
    return true;
}

float Compass::heading(float pitchDeg, float rollDeg) {
    if (!present()) return NAN;
    float mx, my, mz;
    if (!readCalibrated(mx, my, mz)) return NAN;

    // Tilt compensation: rotate mag into the horizontal plane.
    float p = pitchDeg * (float)M_PI / 180.0f;
    float r = rollDeg  * (float)M_PI / 180.0f;
    float cp = cosf(p), sp = sinf(p);
    float cr = cosf(r), sr = sinf(r);

    float xh =  mx * cp + my * sp * sr + mz * sp * cr;
    float yh =  my * cr - mz * sr;

    float h = atan2f(yh, xh) * 180.0f / (float)M_PI;
    while (h < 0.0f)    h += 360.0f;
    while (h >= 360.0f) h -= 360.0f;
    return h;
}

void Compass::beginCal() {
    for (int i = 0; i < 3; i++) {
        g_calMin[i] =  99999.0f;
        g_calMax[i] = -99999.0f;
    }
    g_calStartMs = millis();
}

bool Compass::updateCal() {
    float mx, my, mz;
    if (!readRaw(mx, my, mz)) return false;
    float v[3] = { mx, my, mz };
    for (int i = 0; i < 3; i++) {
        if (v[i] < g_calMin[i]) g_calMin[i] = v[i];
        if (v[i] > g_calMax[i]) g_calMax[i] = v[i];
    }
    return true;
}

float Compass::calCoveragePct() {
    // Heuristic: minimum span across axes (in raw units) — once each axis
    // sees a healthy swing, we consider coverage good.
    float minSpan = 1e9f;
    for (int i = 0; i < 3; i++) {
        float span = g_calMax[i] - g_calMin[i];
        if (span < minSpan) minSpan = span;
    }
    // Anything <50 raw units is "no data"; >800 is fully covered.
    float pct = (minSpan - 50.0f) / 7.5f;
    if (pct < 0.0f) pct = 0.0f;
    if (pct > 100.0f) pct = 100.0f;
    return pct;
}

bool Compass::finalizeCal() {
    // Need a meaningful span on every axis
    for (int i = 0; i < 3; i++) {
        if (g_calMax[i] - g_calMin[i] < 50.0f) return false;
    }
    float bias[3], halfSpan[3];
    for (int i = 0; i < 3; i++) {
        bias[i]     = 0.5f * (g_calMax[i] + g_calMin[i]);
        halfSpan[i] = 0.5f * (g_calMax[i] - g_calMin[i]);
    }
    float avg = (halfSpan[0] + halfSpan[1] + halfSpan[2]) / 3.0f;
    for (int i = 0; i < 3; i++) {
        g_mCal.bias[i]  = bias[i];
        g_mCal.scale[i] = (halfSpan[i] > 0.1f) ? (avg / halfSpan[i]) : 1.0f;
    }
    g_mCal.valid = true;

    Preferences prefs;
    if (prefs.begin(NVS_NAMESPACE, false)) {
        prefs.putBytes(NVS_KEY_MCAL, &g_mCal, sizeof(g_mCal));
        prefs.end();
    }
    return true;
}

void Compass::resetCal() {
    g_mCal = {};
    g_mCal.scale[0] = g_mCal.scale[1] = g_mCal.scale[2] = 1.0f;
    Preferences prefs;
    if (prefs.begin(NVS_NAMESPACE, false)) {
        prefs.remove(NVS_KEY_MCAL);
        prefs.end();
    }
}

bool Compass::isCalibrated() { return g_mCal.valid; }

bool Compass::loadCal() {
    Preferences prefs;
    if (!prefs.begin(NVS_NAMESPACE, true)) return false;
    MagCalibration tmp = {};
    size_t got = prefs.getBytes(NVS_KEY_MCAL, &tmp, sizeof(tmp));
    prefs.end();
    if (got != sizeof(tmp) || !tmp.valid) return false;
    g_mCal = tmp;
    return true;
}
