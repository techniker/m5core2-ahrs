/*
 * logger.cpp – SD Card CSV Logger
 * Creates sequentially numbered log files on the SD card.
 * Each write is flushed immediately so data survives power loss.
 */

#include <M5Core2.h>
#include <SD.h>
#include "logger.h"
#include "config.h"

static bool     s_active = false;
static File     s_file;
static char     s_filename[32] = {};
static uint32_t s_startMs = 0;

// Find next available log number and open file
static bool openNextLogFile() {
    for (int i = 0; i < 10000; i++) {
        snprintf(s_filename, sizeof(s_filename), "/log_%04d.csv", i);
        if (!SD.exists(s_filename)) {
            s_file = SD.open(s_filename, FILE_WRITE);
            return (bool)s_file;
        }
    }
    return false;
}

void Logger::init() {
    s_active = false;

    // SD is already initialized by M5.begin()
    // Check if card is present by trying to open root
    File root = SD.open("/");
    if (!root) return;
    root.close();

    if (!openNextLogFile()) return;

    // Write CSV header
    s_file.println(
        "time_ms,"
        "pitch,roll,yaw,"
        "gyro_x,gyro_y,gyro_z,"
        "accel_x,accel_y,accel_z,"
        "g_load,turn_rate,slip_angle,vert_accel,"
        "airspeed,altitude,vspeed,"
        "batt_v,imu_temp,dt"
    );
    s_file.flush();

    s_startMs = millis();
    s_active = true;
}

bool Logger::isActive() {
    return s_active;
}

void Logger::log(const SensorData &data) {
    if (!s_active) return;

    uint32_t t = millis() - s_startMs;

    // Build CSV line
    char buf[256];
    snprintf(buf, sizeof(buf),
        "%lu,"
        "%.2f,%.2f,%.2f,"
        "%.2f,%.2f,%.2f,"
        "%.4f,%.4f,%.4f,"
        "%.3f,%.2f,%.4f,%.4f,"
        "%.1f,%.1f,%.1f,"
        "%.3f,%.1f,%.6f",
        (unsigned long)t,
        data.pitch, data.roll, data.yaw,
        data.gyro[0], data.gyro[1], data.gyro[2],
        data.accel[0], data.accel[1], data.accel[2],
        data.gLoad, data.turnRate, data.slipAngle, data.vertAccel,
        data.airspeed, data.altitude, data.vspeed,
        data.battVoltage, data.imuTemp, data.dt
    );

    s_file.println(buf);
    s_file.flush();   // force write to card — survives power loss
}

const char* Logger::getFilename() {
    return s_active ? s_filename : nullptr;
}
