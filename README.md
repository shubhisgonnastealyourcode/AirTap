<img width="1026" height="585" alt="image" src="https://github.com/user-attachments/assets/d00b1245-c648-43d2-a6eb-460047d48da5" /># AirTap
A smart wearable bracelet that tracks wrist orientation and detects pinch gestures in real time, enabling intuitive hands-free control of PCs and other devices using BLE communication, motion sensing, haptic feedback, and natural gesture-based interaction.


Here’s a clean GitHub README-style markdown for your v1.2 🚀

# WristGesture v1.2

BLE-enabled smart wrist gesture interface using the Seeed Studio XIAO ESP32-C3 + MPU6050.

## Features

* Real-time wrist tilt detection
* Single pinch detection
* Double pinch detection
* OLED UI feedback
* Haptic vibration feedback
* BLE telemetry streaming
* BLE gesture event notifications
* Non-blocking FSM architecture
* Data-driven jerk detection algorithm

---

# Hardware

* Seeed Studio XIAO ESP32-C3
* MPU6050 (GY-521)
* SSD1306 OLED Display (128x32)
* Coin vibration motor

---

# Gesture Detection

## Tilt Detection

Uses filtered accelerometer roll angle:

* LEFT
* CENTER
* RIGHT

with hysteresis to avoid chattering.

## Pinch Detection

Uses:

* dynamic acceleration magnitude
* jerk peak detection
* 4-stage finite state machine

States:

```text
IDLE → PEAK_1 → WAIT_SECOND → PEAK_2 → LOCKOUT
```

Supports:

* Single pinch
* Double pinch

---

# BLE Features (v1.2)

## Device Name

```text
WristGesture
```

## BLE Service UUID

```text
4fafc201-1fb5-459e-8fcc-c5c9c331914b
```

## BLE Characteristic UUID

```text
beb5483e-36e1-4688-b7f5-ea07361b26a8
```

---

# BLE Packet Types

## Live Telemetry

```json
{
  "t":"d",
  "o":"LEFT",
  "ax":0.01,
  "ay":0.34,
  "az":0.92,
  "gx":1.24,
  "gy":-0.22,
  "gz":0.18,
  "j":12.44
}
```

## Gesture Event

### Single Pinch

```json
{
  "t":"p",
  "n":1
}
```

### Double Pinch

```json
{
  "t":"p",
  "n":2
}
```

---

# Python BLE Monitor

Uses Python + Bleak to:

* scan for device
* connect over BLE
* receive telemetry
* receive gesture events

Install:

```bash
pip install bleak
```

Run:

```bash
python wrist_monitor.py
```

---

# Libraries Used

* NimBLE-Arduino
* Adafruit SSD1306
* Adafruit GFX
* Wire

---

# Version 1.2 Changes

## BLE Added

* BLE advertising
* Live telemetry notifications
* Gesture notifications
* JSON packet system
* Python desktop monitor

## Stability Improvements

* Fixed characteristic notification handling
* Fixed UTF-8 packet issues
* Added BLE reconnect support
* Added connection callbacks

---

# Future Plans

* Touch gesture navigation
* Media control
* BLE HID support
* Mobile app
* Custom PCB
* TinyML gesture classification
* Battery-powered wearable version

---

# Notes

This project is experimental and designed for wearable gesture interaction research and prototyping.

Built with caffeine, debugging pain, and random 2 AM breakthroughs.
