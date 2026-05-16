# Wiring

DOT's electronics in one place. Diagram still TODO.

---

## Bus

ESP32-S3-Matrix ↔ PCA9685 over I2C.

| Signal | ESP32-S3-Matrix pin | PCA9685 pin |
|--------|---------------------|-------------|
| SDA    | GPIO 34             | SDA         |
| SCL    | GPIO 33             | SCL         |
| VCC    | 3.3 V               | VCC         |
| GND    | GND                 | GND         |

The I2C pins are set in code at `firmware/dot_ik_test/dot_ik_test.ino` via `SDA_PIN` / `SCL_PIN`. If you change them in hardware, change them there too.

PCA9685 I2C address: `0x40` (default, all address-select pads open).

---

## Power

| Source | Goes to |
|--------|---------|
| 2×18650 battery shield, 5 V 3 A rail | PCA9685 V+ screw terminal (servo rail) |
| 2×18650 battery shield, 3 V 1 A rail | ESP32-S3-Matrix 5V/USB pin (via the shield's regulated output — *not* shared with servos) |
| Common ground | battery shield GND ↔ PCA9685 GND ↔ ESP32 GND |

**Do not** power the servos off the ESP32's 5 V pin. Twelve MG90S under load will brown out the board, the IMU will glitch, and the I2C bus will go down with it.

---

## Servo channel map

PCA9685 channels 0–11. Channel numbers match `firmware/dot_ik_test/dot_ik_test.ino` `FRONT_LEFT` / `FRONT_RIGHT` / `BACK_LEFT` / `BACK_RIGHT` arrays.

| Leg | Hip (side) | Upper leg | Knee/foot |
|------|------------|-----------|-----------|
| Front left  | 0 | 4 | 8  |
| Front right | 1 | 5 | 9  |
| Back left   | 2 | 6 | 10 |
| Back right  | 3 | 7 | 11 |

Reversed (mounted mirrored) servos are listed in the firmware constant `REVERSED_SERVOS`. Current value: `{0, 2, 4, 6, 9, 11}`. Update there if you remount.

---

## TODO

- [ ] Wiring diagram (image)
- [ ] Photo of the assembled wire loom
- [ ] Connector / pinout reference for the battery shield
