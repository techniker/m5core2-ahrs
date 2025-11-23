# M5Stack Core2 – EFIS-Style AHRS Indicator

This project turns an **M5Stack Core2** into a cheap **Primary Flight Display (PFD)** / **EFIS-style attitude indicator** using only the built-in **MPU6886 IMU**(not verry accurate!!).

<p align="center">
  <img src="https://raw.githubusercontent.com/techniker/m5core2-ahrs/main/img/m5stackCore2_ahrs.jpeg" width="350" />
</p>


It shows:

- Artificial horizon (sky/ground) with pitch ladder  
- Bank scale
- Slip/skid ball with simple physics model  
- Flight Director crossbars (FD)  
- Flight Path Vector / bird symbol (FPV)  
- Compass **heading tape** with digital heading display
- pitch/roll values + battery voltage  

---

## Features


### Flight Director (FD)

- Magenta **crossbars** in the center

### Flight Path Vector (FPV / „Bird“)

- Green **FPV symbol** showing a rough estimated„flight path“ indication.
- Combines display pitch/roll with linear accelerations `ax`, `ay`:
- This is of course not a true FMS calibrated FPV (we have no airspeed / GPS)!

### Heading Tape & Bottom Band

- **Heading tape** across the bottom:
  - Moving scale centered on current heading.
  - Central orange triangle pointer.
- **Digital heading** (000–359°)
- Right: numeric **Roll** and **Pitch**.
- Left: **Battery voltage** from AXP192.

---

## Hardware & Software Requirements

### Hardware

- **M5Stack Core2** (with MPU6886 IMU and AXP192 PMIC)

### Libraries used

- [M5Core2](https://github.com/m5stack/M5Core2)  
  (which internally uses TFT_eSPI / TFT_eSprite)
- Arduino core for ESP32 (standard M5Stack setup)

---

## Building & Flashing

You can build either with the **Arduino IDE** or **PlatformIO**.

### 1. Arduino IDE

1. Install the **ESP32 board support** and the **M5Core2** library via Library Manager.
2. Select board:
   - `M5Stack-Core2` (or corresponding M5Stack Core2 entry)
3. Create a new sketch and paste the entire `main.cpp` code from this project.
4. Compile & upload.

### 2. PlatformIO

Example `platformio.ini`:

```ini
[env:m5stack-core2]
platform = espressif32
board = m5stack-core2
framework = arduino

lib_deps =
    m5stack/M5Core2

Have Fun!