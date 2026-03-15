/*
 * sensors.cpp – Sensor Management Implementation
 * MPU6886 reading, gyro bias calibration, derived data computation.
 */

#include <M5Core2.h>
#include "sensors.h"
#include "config.h"

static SensorData     s_data = {};
static CalibrationData s_cal = {};
static float s_pitchZero = 0.0f;
static float s_rollZero  = 0.0f;
static unsigned long s_lastUs = 0;

// Low-pass filter coefficient for derived quantities
static constexpr float LPF_ALPHA = 0.15f;

static float lpf(float prev, float cur, float alpha) {
    return prev + alpha * (cur - prev);
}

void Sensors::init() {
    M5.IMU.Init();
    s_data = {};
    s_cal  = {};
    s_lastUs = micros();
}

void Sensors::calibrateGyro() {
    s_cal.calibrated  = false;
    s_cal.sampleCount = 0;
    s_cal.gyroSum[0]  = 0.0f;
    s_cal.gyroSum[1]  = 0.0f;
    s_cal.gyroSum[2]  = 0.0f;
    s_cal.accelSum[0] = 0.0f;
    s_cal.accelSum[1] = 0.0f;
    s_cal.accelSum[2] = 0.0f;

    for (int i = 0; i < GYRO_CAL_SAMPLES; i++) {
        float gx, gy, gz, ax, ay, az;
        M5.IMU.getGyroData(&gx, &gy, &gz);
        M5.IMU.getAccelData(&ax, &ay, &az);
        s_cal.gyroSum[0] += gx;
        s_cal.gyroSum[1] += gy;
        s_cal.gyroSum[2] += gz;
        s_cal.accelSum[0] += ax;
        s_cal.accelSum[1] += ay;
        s_cal.accelSum[2] += az;
        s_cal.sampleCount++;
        delayMicroseconds(2000);
    }

    float n = (float)s_cal.sampleCount;
    s_cal.gyroBias[0] = s_cal.gyroSum[0] / n;
    s_cal.gyroBias[1] = s_cal.gyroSum[1] / n;
    s_cal.gyroBias[2] = s_cal.gyroSum[2] / n;
    s_cal.accelAvg[0] = s_cal.accelSum[0] / n;
    s_cal.accelAvg[1] = s_cal.accelSum[1] / n;
    s_cal.accelAvg[2] = s_cal.accelSum[2] / n;
    s_cal.calibrated  = true;
}

bool Sensors::isCalibrated() {
    return s_cal.calibrated;
}

void Sensors::getCalAccel(float &ax, float &ay, float &az) {
    ax = s_cal.accelAvg[0];
    ay = s_cal.accelAvg[1];
    az = s_cal.accelAvg[2];
}

void Sensors::readAndUpdate(MadgwickAHRS &ahrs) {
    unsigned long now = micros();
    float dt = (now - s_lastUs) * 1e-6f;
    if (dt <= 0.0f || dt > 0.5f) dt = 0.005f;  // sanity clamp
    s_lastUs = now;
    s_data.dt = dt;

    // Read raw IMU
    float gx, gy, gz, ax, ay, az;
    M5.IMU.getGyroData(&gx, &gy, &gz);
    M5.IMU.getAccelData(&ax, &ay, &az);

    // Remove gyro bias
    if (s_cal.calibrated) {
        gx -= s_cal.gyroBias[0];
        gy -= s_cal.gyroBias[1];
        gz -= s_cal.gyroBias[2];
    }

    // Feed raw chip-frame data directly to Madgwick.
    // DO NOT remap axes — gyro and accel must be in the same frame
    // for the gradient descent to work correctly.
    // Landscape pitch/roll swap is done AFTER Euler extraction.

    // Adaptive beta: reduce accel trust during dynamic maneuvers
    float accelMag = sqrtf(ax * ax + ay * ay + az * az);
    float accelErr = fabsf(accelMag - 1.0f);
    float adaptiveBeta = AHRS_BETA_DEFAULT;
    if (accelErr > 0.1f) {
        float scale = 1.0f - (accelErr - 0.1f) * 3.0f;
        if (scale < 0.0f) scale = 0.0f;
        adaptiveBeta = AHRS_BETA_MIN + (AHRS_BETA_DEFAULT - AHRS_BETA_MIN) * scale;
    }
    ahrs.setBeta(adaptiveBeta);

    // Update AHRS with raw chip-frame data
    ahrs.update(gx, gy, gz, ax, ay, az, dt);

    // Extract Euler angles in chip frame
    float chipPitch = ahrs.getPitch();
    float chipRoll  = ahrs.getRoll();
    float chipYaw   = ahrs.getYaw();

    // M5Core2 landscape (rotation 1): swap pitch/roll from chip frame.
    // This matches the original code: display_pitch = chip_roll,
    //                                  display_roll  = chip_pitch.
    float rawPitch =  chipRoll;
    float rawRoll  =  chipPitch;
    float rawYaw   =  chipYaw;

    s_data.pitch = rawPitch - s_pitchZero;
    s_data.roll  = rawRoll  - s_rollZero;

    // Yaw: wrap to 0-360 (absolute heading, not zeroed)
    float yaw = rawYaw;
    while (yaw < 0.0f)    yaw += 360.0f;
    while (yaw >= 360.0f) yaw -= 360.0f;
    s_data.yaw = yaw;

    // Store accel/gyro in display frame (swapped for landscape) for PFD use
    s_data.gyro[0]  = gy;   // display-frame roll rate
    s_data.gyro[1]  = gx;   // display-frame pitch rate
    s_data.gyro[2]  = gz;   // yaw rate
    s_data.accel[0] = ay;   // display-frame lateral
    s_data.accel[1] = ax;   // display-frame longitudinal
    s_data.accel[2] = az;   // vertical

    // Derived: G-load
    s_data.gLoad = lpf(s_data.gLoad, accelMag, LPF_ALPHA);

    // Derived: turn rate (yaw gyro, deg/s)
    s_data.turnRate = lpf(s_data.turnRate, gz, LPF_ALPHA);

    // Derived: slip angle (lateral accel / vertical accel)
    // Chip az ≈ +1g when screen-up. Chip ay = lateral in landscape.
    // Subtract calibrated accel bias so ball is centered when level.
    if (fabsf(az) > 0.1f) {
        float lateralCorr = ay - s_cal.accelAvg[1];  // remove static bias
        s_data.slipAngle = lpf(s_data.slipAngle, atan2f(-lateralCorr, az), LPF_ALPHA);
    }

    // Derived: vertical acceleration (subtract gravity)
    s_data.vertAccel = lpf(s_data.vertAccel, accelMag - 1.0f, 0.05f);

    // Battery voltage
    s_data.battVoltage = M5.Axp.GetBatVoltage();
}

const SensorData& Sensors::getData() {
    return s_data;
}

void Sensors::setZeroAttitude() {
    // Current AHRS output becomes the new zero reference for pitch/roll only.
    // Yaw (heading) is kept absolute — not zeroed.
    s_pitchZero += s_data.pitch;
    s_rollZero  += s_data.roll;
}
