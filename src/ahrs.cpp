/*
 * ahrs.cpp – Madgwick AHRS Filter Implementation
 * IMU-only (6DOF) gradient descent orientation filter.
 */

#include "ahrs.h"

void MadgwickAHRS::begin(float b) {
    beta  = b;
    q[0] = 1.0f; q[1] = 0.0f; q[2] = 0.0f; q[3] = 0.0f;
}

void MadgwickAHRS::update(float gx, float gy, float gz,
                           float ax, float ay, float az, float dt) {
    float q0 = q[0], q1 = q[1], q2 = q[2], q3 = q[3];

    // Convert gyro to rad/s
    float gxr = gx * (M_PI / 180.0f);
    float gyr = gy * (M_PI / 180.0f);
    float gzr = gz * (M_PI / 180.0f);

    // Rate of change of quaternion from gyroscope
    float qDot0 = 0.5f * (-q1 * gxr - q2 * gyr - q3 * gzr);
    float qDot1 = 0.5f * ( q0 * gxr + q2 * gzr - q3 * gyr);
    float qDot2 = 0.5f * ( q0 * gyr - q1 * gzr + q3 * gxr);
    float qDot3 = 0.5f * ( q0 * gzr + q1 * gyr - q2 * gxr);

    // Compute feedback only if accelerometer data valid
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
        // Normalise accelerometer
        float recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // Auxiliary variables to avoid repeated arithmetic
        float _2q0 = 2.0f * q0, _2q1 = 2.0f * q1;
        float _2q2 = 2.0f * q2, _2q3 = 2.0f * q3;
        float _4q0 = 4.0f * q0, _4q1 = 4.0f * q1;
        float _4q2 = 4.0f * q2;
        float _8q1 = 8.0f * q1, _8q2 = 8.0f * q2;
        float q0q0 = q0 * q0, q1q1 = q1 * q1;
        float q2q2 = q2 * q2, q3q3 = q3 * q3;

        // Gradient descent corrective step
        float s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
        float s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1
                   - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
        float s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3
                   - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
        float s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;

        recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
        s0 *= recipNorm;
        s1 *= recipNorm;
        s2 *= recipNorm;
        s3 *= recipNorm;

        // Apply feedback
        qDot0 -= beta * s0;
        qDot1 -= beta * s1;
        qDot2 -= beta * s2;
        qDot3 -= beta * s3;
    }

    // Integrate rate of change
    q0 += qDot0 * dt;
    q1 += qDot1 * dt;
    q2 += qDot2 * dt;
    q3 += qDot3 * dt;

    // Normalise quaternion
    float recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q[0] = q0 * recipNorm;
    q[1] = q1 * recipNorm;
    q[2] = q2 * recipNorm;
    q[3] = q3 * recipNorm;
}

float MadgwickAHRS::getPitch() const {
    // Pitch (theta) = asin(2*(q0*q2 - q3*q1))
    float sinp = 2.0f * (q[0] * q[2] - q[3] * q[1]);
    if (sinp >  1.0f) sinp =  1.0f;
    if (sinp < -1.0f) sinp = -1.0f;
    return asinf(sinp) * (180.0f / M_PI);
}

float MadgwickAHRS::getRoll() const {
    // Roll (phi) = atan2(2*(q0*q1 + q2*q3), 1 - 2*(q1^2 + q2^2))
    float sinr = 2.0f * (q[0] * q[1] + q[2] * q[3]);
    float cosr = 1.0f - 2.0f * (q[1] * q[1] + q[2] * q[2]);
    return atan2f(sinr, cosr) * (180.0f / M_PI);
}

float MadgwickAHRS::getYaw() const {
    // Yaw (psi) = atan2(2*(q0*q3 + q1*q2), 1 - 2*(q2^2 + q3^2))
    float siny = 2.0f * (q[0] * q[3] + q[1] * q[2]);
    float cosy = 1.0f - 2.0f * (q[2] * q[2] + q[3] * q[3]);
    return atan2f(siny, cosy) * (180.0f / M_PI);
}

void MadgwickAHRS::initFromAccel(float ax, float ay, float az) {
    // Compute the quaternion that rotates [0,0,1] (gravity in earth frame)
    // to the measured accelerometer vector. This gives correct pitch/roll
    // immediately — only yaw remains unknown (set to 0).
    float norm = sqrtf(ax * ax + ay * ay + az * az);
    if (norm < 0.01f) return;  // no valid reading
    ax /= norm; ay /= norm; az /= norm;

    // Euler angles from gravity vector
    float pitch = asinf(-ax);                   // nose-up positive
    float roll  = atan2f(ay, az);               // right-wing-down positive

    // Convert Euler (ZYX) to quaternion with yaw=0
    float cp = cosf(pitch * 0.5f), sp = sinf(pitch * 0.5f);
    float cr = cosf(roll  * 0.5f), sr = sinf(roll  * 0.5f);
    // yaw = 0 → cy=1, sy=0
    q[0] =  cr * cp;         // w
    q[1] =  sr * cp;         // x
    q[2] =  cr * sp;         // y
    q[3] = -sr * sp;         // z

    // Normalise
    float rn = invSqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    q[0] *= rn; q[1] *= rn; q[2] *= rn; q[3] *= rn;
}

void MadgwickAHRS::getQuaternion(float &qw, float &qx, float &qy, float &qz) const {
    qw = q[0]; qx = q[1]; qy = q[2]; qz = q[3];
}

float MadgwickAHRS::invSqrt(float x) {
    // Use standard math for accuracy on ESP32 (FPU available)
    return 1.0f / sqrtf(x);
}
