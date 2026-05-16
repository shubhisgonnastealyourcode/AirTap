# Software

## Overview

The WristGesture software stack processes real-time IMU motion data to detect wrist orientation and pinch gestures using lightweight signal-processing algorithms running directly on the ESP32-C3.

The gesture system was inspired by the pinch interaction style used in the Samsung Galaxy Watch, adapted into a custom wearable interface using accelerometer-based motion analysis.

---

# Gesture Dataset & Calibration

To improve reliability, a real-world gesture dataset was collected containing:

* ~300 real pinch gestures
* Multiple wrist orientations
* Different motion speeds
* Rest/noise samples
* Raw CSV telemetry logs

The collected data was used to tune thresholds, timing windows, filtering strength, and false-positive rejection behavior.

---

# Signal Processing Pipeline

## 1. IMU Sampling

The MPU6050 runs at:

* 100 Hz sample rate
* ±2g accelerometer range
* ±250°/s gyro range

Raw accelerometer and gyroscope values are continuously read over I2C.

---

## 2. EMA Filtering

Two Exponential Moving Average (EMA) filters are used:

### Slow Filter

Used for stable wrist orientation tracking.

```text id="lc0d07"
alpha = 0.20
```

### Fast Filter

Used for responsive pinch detection.

```text id="gjb5ul"
alpha = 0.75
```

This dual-filter approach allows stable tilt tracking while preserving fast motion spikes required for pinch detection.

---

# Tilt Detection

Wrist direction is computed using roll angle:

```text id="dbw6u3"
roll = atan2(Ay, Az)
```

Detected states:

* LEFT
* CENTER
* RIGHT

Hysteresis thresholds are used to prevent rapid flickering between states.

---

# Pinch Detection

## Dynamic Magnitude

Gravity is removed by calculating motion magnitude relative to 1g.

```text id="0r3vc0"
dynMag = | |A| - 1g |
```

---

## Jerk Calculation

Pinch gestures create sharp acceleration spikes.

The software calculates jerk:

```text id="j13e4w"
jerk = Δacceleration / Δtime
```

Measured in:

```text id="8i3g1t"
g/s
```

This captures the impulsive nature of pinch movements.

---

# Finite State Machine (FSM)

Pinch detection uses a multi-stage FSM:

```text id="f7r9xe"
IDLE → PEAK_1 → WAIT_SECOND → PEAK_2 → LOCKOUT
```

Supports:

* Single pinch
* Double pinch
* Debounce handling
* Noise rejection
* False trigger suppression

---

# BLE Telemetry (v1.2)

The ESP32-C3 streams:

* live accelerometer data
* gyroscope data
* jerk values
* orientation state
* gesture events

using BLE notifications and JSON packets.

---

# Desktop Monitoring

A Python monitoring tool using Bleak was built for:

* BLE debugging
* live telemetry visualization
* gesture validation
* threshold tuning
* development testing

---

# Design Goals

* Low latency
* Real-time wearable interaction
* Minimal CPU usage
* Fully embedded processing
* Expandable gesture architecture
