/*
 * sensors.cpp – Sensor Management Implementation
 * MPU6886 reading, gyro + accel calibration, derived data, NVS persistence.
 */

#include <M5Core2.h>
#include <Preferences.h>
#include <cmath>
#include "sensors.h"
#include "config.h"

namespace {

struct InternalCal {
    // Live gyro/accel bias from boot-time still calibration
    float gyroBias[3];
    float accelAvg[3];
    bool  calibrated;
    int   sampleCount;
    float gyroSum[3];
    float accelSum[3];

    // 6-point accelerometer calibration state
    bool  facesCaptured[Sensors::FACE_COUNT];
    float faceReading[Sensors::FACE_COUNT][3];
};

SensorData       g_data = {};
InternalCal      g_cal  = {};
AccelCalibration g_aCal = {};   // persisted accel cal (bias + scale)
float            g_pitchZero  = 0.0f;
float            g_rollZero   = 0.0f;
float            g_hdgOffset  = 0.0f;
unsigned long    g_lastUs     = 0;

constexpr float LPF_ALPHA = 0.15f;

inline float lpf(float prev, float cur, float alpha) {
    return prev + alpha * (cur - prev);
}

// Apply persistent accel calibration to a raw chip-frame reading.
inline void applyAccelCal(float &ax, float &ay, float &az) {
    if (!g_aCal.valid) return;
    ax = (ax - g_aCal.bias[0]) * g_aCal.scale[0];
    ay = (ay - g_aCal.bias[1]) * g_aCal.scale[1];
    az = (az - g_aCal.bias[2]) * g_aCal.scale[2];
}

}  // namespace

void Sensors::init() {
    M5.IMU.Init();
    g_data = {};
    g_cal  = {};
    g_aCal = {};
    g_aCal.scale[0] = g_aCal.scale[1] = g_aCal.scale[2] = 1.0f;
    g_data.mag[0] = g_data.mag[1] = g_data.mag[2] = NAN;
    g_data.magHeading = NAN;
    g_lastUs = micros();

    loadAccelCal();
    loadHeadingOffset();
}

// ─────────────────────────────────────────────────────────
// Gyro bias + accel average (still, ~1s)
// ─────────────────────────────────────────────────────────

void Sensors::calibrateGyro() {
    g_cal.calibrated  = false;
    g_cal.sampleCount = 0;
    for (int i = 0; i < 3; i++) {
        g_cal.gyroSum[i]  = 0.0f;
        g_cal.accelSum[i] = 0.0f;
    }

    for (int i = 0; i < GYRO_CAL_SAMPLES; i++) {
        float gx, gy, gz, ax, ay, az;
        M5.IMU.getGyroData(&gx, &gy, &gz);
        M5.IMU.getAccelData(&ax, &ay, &az);
        applyAccelCal(ax, ay, az);
        g_cal.gyroSum[0]  += gx;
        g_cal.gyroSum[1]  += gy;
        g_cal.gyroSum[2]  += gz;
        g_cal.accelSum[0] += ax;
        g_cal.accelSum[1] += ay;
        g_cal.accelSum[2] += az;
        g_cal.sampleCount++;
        delayMicroseconds(2000);
    }

    float n = (float)g_cal.sampleCount;
    for (int i = 0; i < 3; i++) {
        g_cal.gyroBias[i] = g_cal.gyroSum[i]  / n;
        g_cal.accelAvg[i] = g_cal.accelSum[i] / n;
    }
    g_cal.calibrated = true;
}

bool Sensors::isCalibrated() { return g_cal.calibrated; }

void Sensors::getCalAccel(float &ax, float &ay, float &az) {
    ax = g_cal.accelAvg[0];
    ay = g_cal.accelAvg[1];
    az = g_cal.accelAvg[2];
}

// ─────────────────────────────────────────────────────────
// 6-point accelerometer calibration
// ─────────────────────────────────────────────────────────

void Sensors::beginAccelCal() {
    for (int f = 0; f < FACE_COUNT; f++) {
        g_cal.facesCaptured[f] = false;
        g_cal.faceReading[f][0] = 0.0f;
        g_cal.faceReading[f][1] = 0.0f;
        g_cal.faceReading[f][2] = 0.0f;
    }
}

bool Sensors::captureAccelFace(AccelFace face) {
    if (face < 0 || face >= FACE_COUNT) return false;

    // Average ACCEL_CAL_SAMPLES readings while still.
    double sum[3] = {0.0, 0.0, 0.0};
    for (int i = 0; i < ACCEL_CAL_SAMPLES; i++) {
        float ax, ay, az;
        M5.IMU.getAccelData(&ax, &ay, &az);
        sum[0] += ax; sum[1] += ay; sum[2] += az;
        delayMicroseconds(2000);
    }
    float n = (float)ACCEL_CAL_SAMPLES;
    g_cal.faceReading[face][0] = (float)(sum[0] / n);
    g_cal.faceReading[face][1] = (float)(sum[1] / n);
    g_cal.faceReading[face][2] = (float)(sum[2] / n);
    g_cal.facesCaptured[face] = true;
    return true;
}

bool Sensors::isAccelFaceCaptured(AccelFace face) {
    if (face < 0 || face >= FACE_COUNT) return false;
    return g_cal.facesCaptured[face];
}

bool Sensors::finalizeAccelCal() {
    for (int f = 0; f < FACE_COUNT; f++) {
        if (!g_cal.facesCaptured[f]) return false;
    }

    // Each axis sees +1g on one face and -1g on the opposite face.
    // Bias_i  = (face_pos_i + face_neg_i) / 2
    // Scale_i = 2 / (face_pos_i - face_neg_i)
    auto computeAxis = [&](int axis, AccelFace pos, AccelFace neg,
                           float &bias, float &scale) {
        float vPos = g_cal.faceReading[pos][axis];
        float vNeg = g_cal.faceReading[neg][axis];
        bias  = 0.5f * (vPos + vNeg);
        float span = vPos - vNeg;
        scale = (fabsf(span) > 0.1f) ? (2.0f / span) : 1.0f;
    };

    computeAxis(2, FACE_Z_UP, FACE_Z_DN, g_aCal.bias[2], g_aCal.scale[2]);
    computeAxis(0, FACE_X_UP, FACE_X_DN, g_aCal.bias[0], g_aCal.scale[0]);
    computeAxis(1, FACE_Y_UP, FACE_Y_DN, g_aCal.bias[1], g_aCal.scale[1]);

    // Sanity: scale should be within ±20% of 1.0; otherwise the user
    // probably mis-oriented one of the faces.
    for (int i = 0; i < 3; i++) {
        if (g_aCal.scale[i] < 0.8f || g_aCal.scale[i] > 1.2f) {
            g_aCal.valid = false;
            return false;
        }
        if (fabsf(g_aCal.bias[i]) > 0.5f) {
            g_aCal.valid = false;
            return false;
        }
    }
    g_aCal.valid = true;

    // Persist to NVS
    Preferences prefs;
    if (prefs.begin(NVS_NAMESPACE, false)) {
        prefs.putBytes(NVS_KEY_ACAL, &g_aCal, sizeof(g_aCal));
        prefs.end();
    }
    return true;
}

const AccelCalibration& Sensors::getAccelCal() { return g_aCal; }

bool Sensors::loadAccelCal() {
    Preferences prefs;
    if (!prefs.begin(NVS_NAMESPACE, true)) return false;
    AccelCalibration tmp = {};
    size_t got = prefs.getBytes(NVS_KEY_ACAL, &tmp, sizeof(tmp));
    prefs.end();
    if (got != sizeof(tmp) || !tmp.valid) return false;
    g_aCal = tmp;
    return true;
}

void Sensors::resetAccelCal() {
    g_aCal = {};
    g_aCal.scale[0] = g_aCal.scale[1] = g_aCal.scale[2] = 1.0f;
    Preferences prefs;
    if (prefs.begin(NVS_NAMESPACE, false)) {
        prefs.remove(NVS_KEY_ACAL);
        prefs.end();
    }
}

// ─────────────────────────────────────────────────────────
// Heading offset (magnetic north reference)
// ─────────────────────────────────────────────────────────

void Sensors::setHeadingOffset(float deg) {
    while (deg < 0.0f)     deg += 360.0f;
    while (deg >= 360.0f)  deg -= 360.0f;
    g_hdgOffset = deg;
    Preferences prefs;
    if (prefs.begin(NVS_NAMESPACE, false)) {
        prefs.putFloat(NVS_KEY_HDG0, g_hdgOffset);
        prefs.end();
    }
}

float Sensors::getHeadingOffset() { return g_hdgOffset; }

void Sensors::loadHeadingOffset() {
    Preferences prefs;
    if (!prefs.begin(NVS_NAMESPACE, true)) return;
    g_hdgOffset = prefs.getFloat(NVS_KEY_HDG0, 0.0f);
    prefs.end();
}

// ─────────────────────────────────────────────────────────
// Per-tick update
// ─────────────────────────────────────────────────────────

void Sensors::readAndUpdate(MadgwickAHRS &ahrs) {
    unsigned long now = micros();
    float dt = (now - g_lastUs) * 1e-6f;
    if (dt <= 0.0f || dt > 0.5f) dt = 0.005f;  // sanity clamp
    g_lastUs = now;
    g_data.dt = dt;

    // Read raw IMU
    float gx, gy, gz, ax, ay, az;
    M5.IMU.getGyroData(&gx, &gy, &gz);
    M5.IMU.getAccelData(&ax, &ay, &az);

    // Apply persisted accel bias/scale first
    applyAccelCal(ax, ay, az);

    // Remove gyro bias
    if (g_cal.calibrated) {
        gx -= g_cal.gyroBias[0];
        gy -= g_cal.gyroBias[1];
        gz -= g_cal.gyroBias[2];
    }

    // Adaptive beta: trust accel less during dynamic maneuvers
    float accelMag = sqrtf(ax * ax + ay * ay + az * az);
    float accelErr = fabsf(accelMag - 1.0f);
    float adaptiveBeta = AHRS_BETA_DEFAULT;
    if (accelErr > 0.1f) {
        float scale = 1.0f - (accelErr - 0.1f) * 3.0f;
        if (scale < 0.0f) scale = 0.0f;
        adaptiveBeta = AHRS_BETA_MIN + (AHRS_BETA_DEFAULT - AHRS_BETA_MIN) * scale;
    }
    ahrs.setBeta(adaptiveBeta);

    // Madgwick update with raw chip-frame data
    ahrs.update(gx, gy, gz, ax, ay, az, dt);

    // Extract Euler in chip frame
    float chipPitch = ahrs.getPitch();
    float chipRoll  = ahrs.getRoll();
    float chipYaw   = ahrs.getYaw();

    // Landscape (rotation 1): display pitch = chip roll, display roll = chip pitch
    float rawPitch = chipRoll;
    float rawRoll  = chipPitch;
    float rawYaw   = chipYaw;

    g_data.pitch = rawPitch - g_pitchZero;
    g_data.roll  = rawRoll  - g_rollZero;

    // Yaw to 0-360, apply user heading offset (magnetic north reference)
    float yaw = rawYaw - g_hdgOffset;
    while (yaw < 0.0f)    yaw += 360.0f;
    while (yaw >= 360.0f) yaw -= 360.0f;
    g_data.yaw = yaw;

    // Store accel/gyro in display frame for PFD
    g_data.gyro[0]  = gy;
    g_data.gyro[1]  = gx;
    g_data.gyro[2]  = gz;
    g_data.accel[0] = ay;   // display-frame lateral
    g_data.accel[1] = ax;   // display-frame longitudinal
    g_data.accel[2] = az;   // vertical

    // Derived
    g_data.gLoad     = lpf(g_data.gLoad,     accelMag,         LPF_ALPHA);
    g_data.turnRate  = lpf(g_data.turnRate,  gz,               LPF_ALPHA);

    if (fabsf(az) > 0.1f) {
        float lateralCorr = ay - g_cal.accelAvg[1];
        g_data.slipAngle = lpf(g_data.slipAngle,
                               atan2f(-lateralCorr, az), LPF_ALPHA);
    }
    g_data.vertAccel = lpf(g_data.vertAccel, accelMag - 1.0f, 0.05f);

    // Temperature + battery
    float t = 0.0f;
    M5.IMU.getTempData(&t);
    g_data.imuTemp = t;
    g_data.battVoltage = M5.Axp.GetBatVoltage();
}

const SensorData& Sensors::getData() { return g_data; }

void Sensors::setZeroAttitude() {
    g_pitchZero += g_data.pitch;
    g_rollZero  += g_data.roll;
}
