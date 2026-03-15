# M5Stack Core2 – Aviation-Grade AHRS / Primary Flight Display

This project turns an **M5Stack Core2** into a **Primary Flight Display (PFD)** using the built-in **MPU6886 6-axis IMU** and a **Madgwick AHRS filter** for accurate, low-drift attitude estimation.

<p align="center">
  <img src="https://github.com/techniker/m5core2-ahrs/blob/main/img/m5stackCore2_ahrs.jpeg" width="350" />
</p>

---

## Features

### AHRS (Attitude & Heading Reference System)

- **Madgwick gradient-descent orientation filter** — quaternion-based, same algorithm used in commercial drone flight controllers
- **Adaptive filter gain** — reduces accelerometer trust during dynamic maneuvers to prevent attitude corruption
- **Accelerometer-seeded initialization** — quaternion computed directly from gravity vector at startup, no convergence drift
- **Automatic gyro bias calibration** — 512-sample average at boot, subtracted from all readings
- **200 Hz sensor/filter update rate**, decoupled from 30 Hz display refresh

### PFD Display

- **Artificial horizon** with sky/ground and thick horizon line
- **Pitch ladder** with chevron tips on major lines (±5°/10° markings) and numeric labels
- **Bank angle arc** (±60°) with tick marks and moving triangle pointer
- **W-shape aircraft reference symbol** (yellow, fixed at center)
- **Speed tape** (left) with V-speed color bands:
  - White arc (Vs0–Vfe), Green arc (Vs1–Vno), Yellow arc (Vno–Vne), Red line (Vne)
  - Current speed window with pointer arrow
- **Altitude tape** (right) with scrolling scale and centered digital readout
- **Vertical speed indicator** (far right) with color-coded pointer
- **Heading tape** (bottom) with cardinal direction labels (N/E/S/W) and digital heading
- **Slip/skid ball** with spring-damper physics simulation
- **Flight path vector** (green bird symbol)
- **G-load indicator** with color-coded warning (amber > ±0.5g from 1g)
- **Turn rate indicator** (standard rate marks)
- **Battery voltage** with color-coded warning (red < 3.4V, amber < 3.6V)

### Controls

- **Button B** (middle): Re-zero attitude reference
- **Button A** (left): Reserved
- **Button C** (right): Reserved

---

## Architecture

```
include/
  config.h        Layout geometry, colors, V-speeds, AHRS tuning, timing
  ahrs.h          Madgwick AHRS filter class
  sensors.h       Sensor data structures and calibration
  pfd.h           PFD renderer interface
src/
  ahrs.cpp        Madgwick quaternion filter with accel-seeded init
  sensors.cpp     MPU6886 reading, gyro bias cal, derived data
  pfd.cpp         Full PFD rendering (horizon, tapes, annunciations)
  main.cpp        Setup/loop orchestrator (~80 lines)
```

Sensor update and display refresh are decoupled:
- **Sensor + AHRS**: 200 Hz (every 5 ms)
- **Display**: 30 Hz (every 33 ms)

---

## Hardware

- **M5Stack Core2** (ESP32 + MPU6886 IMU + AXP192 PMIC + 320×240 IPS)

### Sensors Used

| Sensor | Data | Purpose |
|--------|------|---------|
| MPU6886 accelerometer | 3-axis, ±8g | Gravity reference (pitch/roll), G-load, slip |
| MPU6886 gyroscope | 3-axis, ±2000°/s | Angular rate for AHRS integration |
| AXP192 PMIC | Battery voltage | Low-battery warning |

### External Data Inputs

Airspeed, altitude, and vertical speed are exposed as fields in `SensorData` and default to 0. Connect to real sources (GPS, barometric sensor, pitot tube) via serial or I²C as needed — the display is ready.

---

## Building & Flashing

### PlatformIO (recommended)

```ini
[env:m5stack-core2]
platform = espressif32
board = m5stack-core2
framework = arduino
lib_deps = m5stack/M5Core2@^0.2.0
```

```bash
pio run            # build
pio run -t upload  # flash
```

### Arduino IDE

1. Install **ESP32 board support** and **M5Core2** library via Library Manager.
2. Select board: `M5Stack-Core2`.
3. Copy all files from `src/` and `include/` into your sketch folder.
4. Compile & upload.

---

## Configuration

All tunable parameters are in [`include/config.h`](include/config.h):

| Parameter | Default | Description |
|-----------|---------|-------------|
| `AHRS_BETA_DEFAULT` | 0.06 | Madgwick filter gain (higher = more accel trust) |
| `PITCH_PX_PER_DEG` | 3.4 | Pitch ladder scale |
| `SPD_PX_PER_KT` | 1.8 | Speed tape scale |
| `ALT_PX_PER_FT` | 0.12 | Altitude tape scale |
| `V_S0..V_NE` | 45–160 kt | V-speed color band thresholds |
| `GYRO_CAL_SAMPLES` | 512 | Samples for gyro bias estimation |

---

## License

Have Fun!
