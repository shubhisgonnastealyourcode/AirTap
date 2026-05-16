# Hardware

## Components Used (v1.0 – v1.2)

* Seeed Studio XIAO ESP32-C3
* MPU6050 (GY-521) IMU module
* SSD1306 OLED Display (128×32, I2C)
* Coin vibration motor module
* Li-ion/LiPo battery (optional portable power)
* Jumper wires / prototype PCB

---

# Connections

## I2C Bus

| Device | ESP32-C3 Pin |
| ------ | ------------ |
| SDA    | GPIO 6       |
| SCL    | GPIO 7       |

Both MPU6050 and OLED share the same I2C bus.

---

## MPU6050

| MPU6050 Pin | ESP32-C3 |
| ----------- | -------- |
| VCC         | 3.3V     |
| GND         | GND      |
| SDA         | GPIO 6   |
| SCL         | GPIO 7   |

I2C Address: `0x68`

---

## SSD1306 OLED

| OLED Pin | ESP32-C3 |
| -------- | -------- |
| VCC      | 3.3V     |
| GND      | GND      |
| SDA      | GPIO 6   |
| SCL      | GPIO 7   |

I2C Address: `0x3C`

---

## Vibration Motor Module

| Module Pin | ESP32-C3  |
| ---------- | --------- |
| VCC        | 3.3V / 5V |
| GND        | GND       |
| SIG        | GPIO 21   |

The motor is active HIGH.
