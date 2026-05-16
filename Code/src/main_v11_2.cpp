/**
 * ╔══════════════════════════════════════════════════════════════════╗
 * ║  WristGesture v1.1                                              ║
 * ║  Hardware : Seeed Studio XIAO ESP32-C3                          ║
 * ║             MPU6050 GY-521  (I2C)                               ║
 * ║             SSD1306 OLED    (I2C, shared bus)                   ║
 * ║             Vibration motor (GPIO, active-high)                 ║
 * ║                                                                 ║
 * ║  v1.1 fixes (data-driven, 150-window / 30 000-sample dataset): ║
 * ║   • REMOVED double-EMA bug: computeDynMag no longer applies a   ║
 * ║     second α=0.20 smooth on top of the already-filtered         ║
 * ║     accelFast signal — this was crushing jerk peaks by 5–6×.   ║
 * ║   • JERK_THRESH raised   1.2 → 8.0 g/s  (data-calibrated)      ║
 * ║   • PINCH_DUR_MIN_MS lowered 40 → 10 ms (real spike width)      ║
 * ║   • accelFast warm-up now runs at boot alongside accelFilt       ║
 * ║                                                                 ║
 * ║  Algorithms:                                                    ║
 * ║   • Dual EMA: accelFilt (α=0.20) for tilt,                      ║
 * ║               accelFast (α=0.75) for pinch                      ║
 * ║   • Gravity removal via magnitude normalisation                  ║
 * ║   • Jerk (dA/dt) peak detector for pinch                        ║
 * ║   • roll = atan2(Ay, Az) for left/right tilt                    ║
 * ║   • 4-phase FSM: IDLE → PEAK_1 → WAIT → PEAK_2 → LOCKOUT       ║
 * ║   • Hysteresis band on tilt transitions                         ║
 * ║   • Non-blocking haptic & flash animation                       ║
 * ╚══════════════════════════════════════════════════════════════════╝
 */

 //working
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <math.h>
#include <NimBLEDevice.h>

// ════════════════════════════════════════════════════════════════════
//  ██████╗ ██████╗ ███╗   ██╗███████╗██╗ ██████╗
//  ██╔════╝██╔═══██╗████╗  ██║██╔════╝██║██╔════╝
//  ██║     ██║   ██║██╔██╗ ██║█████╗  ██║██║  ███╗
//  ██║     ██║   ██║██║╚██╗██║██╔══╝  ██║██║   ██║
//  ╚██████╗╚██████╔╝██║ ╚████║██║     ██║╚██████╔╝
//   ╚═════╝ ╚═════╝ ╚═╝  ╚═══╝╚═╝     ╚═╝ ╚═════╝
//  Tune these to match your hardware & wrist placement
// ════════════════════════════════════════════════════════════════════


// =====================================================
// BLE
// =====================================================

#define BLE_DEVICE_NAME     "WristGesture"
#define BLE_SERVICE_UUID    "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define BLE_DATA_CHAR_UUID  "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define BLE_DATA_INTERVAL   100
//BLE Global below


// ── I2C pins (XIAO ESP32-C3 default) ─────────────────────────────
#define PIN_SDA             6
#define PIN_SCL             7

// ── Haptic / vibration motor (active HIGH) ────────────────────────
#define PIN_HAPTIC          21     // change to your GPIO

///
float   peakJerkSeen = 0.0f;
uint32_t jerkPrintMs = 0;

// ── OLED ─────────────────────────────────────────────────────────
#define OLED_I2C_ADDR       0x3C  // change to 0x3D if your module needs it
#define OLED_WIDTH          128   // pixels — set to your display width
#define OLED_HEIGHT         32    // pixels — set to your display height
#define OLED_RESET_PIN      -1    // -1 = share Arduino reset pin

// ── MPU6050 ──────────────────────────────────────────────────────
#define MPU_I2C_ADDR        0x68  // AD0 low; use 0x69 if AD0 pulled high

// ── Sample rate ──────────────────────────────────────────────────
#define SAMPLE_HZ           100
#define SAMPLE_DT_MS        (1000 / SAMPLE_HZ)  // 10 ms
#define SAMPLE_DT_S         (1.0f / SAMPLE_HZ)   // 0.01 s

// ── Tilt detection ───────────────────────────────────────────────
//  Roll angle (degrees) computed from atan2(Ay, Az)
//  Positive roll = right tilt, Negative roll = left tilt
#define TILT_ENTER_DEG      18.0f   // angle to enter LEFT/RIGHT state
#define TILT_EXIT_DEG       10.0f   // angle to return to CENTER (hysteresis)

// ── Pinch detection thresholds ───────────────────────────────────
//  Tuned from 150-window real pinch dataset (25 windows × 6 orientations).
//  accelFast (α=0.75) path, no second EMA on magnitude:
//    Pinch jerk_peak: mean=26 g/s, p5=8.2 g/s
//    Rest jerk_peak:  mean=1.0 g/s, p99=7.7 g/s
//  FSM sim @ thresh=8.0, dur_min=10ms → 96% detection, 1% false-positive
#define JERK_THRESH         8.0f    // g/s  (data-driven; was 1.2 / 2.5)
#define PINCH_DUR_MIN_MS    10      // ms   (jerk spike width p10=10ms; was 40/60)
#define PINCH_DUR_MAX_MS    300     // ms   (was 320–450)
#define DBL_GAP_MIN_MS      100     // ms   (unchanged)
#define DBL_GAP_MAX_MS      650     // ms   (unchanged)
#define LOCKOUT_MS          350     // ms   (unchanged)

// ── EMA filter coefficient ───────────────────────────────────────
//  Lower α = smoother but slower response (trade-off for gesture)
#define EMA_ALPHA           0.20f   // was 0.35 — slower, for tilt only
#define EMA_ALPHA_FAST      0.75f   // new — for pinch path

// ── Haptic pulse durations ───────────────────────────────────────
#define HAPTIC_SHORT_MS     200    // single pinch feedback
#define HAPTIC_LONG_MS      400   // double pinch feedback

// ── Flash animation ──────────────────────────────────────────────
#define FLASH_CYCLES        3     // bright/dark cycles on pinch
#define FLASH_HALF_MS       90    // ms per half-cycle (bright OR dark)

// ════════════════════════════════════════════════════════════════════
//  MPU6050 REGISTER MAP
//  Direct I2C register access — no external library dependency
// ════════════════════════════════════════════════════════════════════
#define MPU_REG_SMPLRT_DIV   0x19
#define MPU_REG_CONFIG       0x1A  // DLPF config
#define MPU_REG_GYRO_CONFIG  0x1B
#define MPU_REG_ACCEL_CONFIG 0x1C
#define MPU_REG_ACCEL_OUT    0x3B  // burst-read base: Ax,Ay,Az,Temp,Gx,Gy,Gz (14 bytes)
#define MPU_REG_WHO_AM_I     0x75  // should return 0x68
#define MPU_REG_PWR_MGMT_1   0x6B

#define ACCEL_LSB_PER_G      16384.0f   // AFS_SEL=0 → ±2g
#define GYRO_LSB_PER_DPS     131.0f     // FS_SEL=0  → ±250°/s

// ════════════════════════════════════════════════════════════════════
//  DATA TYPES
// ════════════════════════════════════════════════════════════════════

struct Vec3f { float x, y, z; };

// Tilt direction with hysteresis
enum class TiltDir  { CENTER, LEFT, RIGHT };

// Pinch FSM phases
enum class PinchPhase {
    IDLE,         // no motion above threshold
    PEAK_1,       // first peak in progress (above threshold)
    WAIT_SECOND,  // first peak done, waiting to decide single vs double
    PEAK_2,       // second peak in progress
    LOCKOUT       // post-event debounce
};

enum class Gesture { NONE, PINCH_SINGLE, PINCH_DOUBLE };



// =====================================================
// TELEMETRY SNAPSHOT
// =====================================================

float telAx = 0;
float telAy = 0;
float telAz = 0;

float telGx = 0;
float telGy = 0;
float telGz = 0;

float telJerk = 0;







// ════════════════════════════════════════════════════════════════════
//  GLOBALS
// ════════════════════════════════════════════════════════════════════


// =====================================================
// BLE GLOBALS
// =====================================================

static NimBLEServer*         pBleServer   = nullptr;
static NimBLECharacteristic* pDataChar    = nullptr;
static bool                  bleConnected = false;

uint32_t lastBleDataMs = 0;


Adafruit_SSD1306 oled(OLED_WIDTH, OLED_HEIGHT, &Wire, OLED_RESET_PIN);

// ── Signal processing state ─────────────────────────────────────
Vec3f accelFilt    = {0.0f, 0.0f, 1.0f};  // slow EMA (α=0.20) for tilt only
Vec3f accelFast    = {0.0f, 0.0f, 1.0f};  // fast EMA (α=0.75) for pinch only
// NOTE: dynMagFilt removed — the second EMA on magnitude was crushing the
//       jerk signal. accelFast already smooths adequately; use raw mag directly.
float prevDynMag   = 0.0f;               // previous dynMag for jerk computation

// ── Tilt FSM ────────────────────────────────────────────────────
TiltDir tiltCurrent  = TiltDir::CENTER;
TiltDir tiltPrev     = TiltDir::CENTER;

// ── Pinch FSM ───────────────────────────────────────────────────
PinchPhase pinchPhase = PinchPhase::IDLE;
uint32_t   phaseEntryMs = 0;
uint32_t   firstPeakEndMs = 0;

// ── Haptic (non-blocking) ───────────────────────────────────────
bool     hapticOn    = false;
uint32_t hapticEndMs = 0;

// ── OLED flash animation (non-blocking) ─────────────────────────
bool     flashActive  = false;
uint8_t  flashCycle   = 0;         // completed cycles
bool     flashIsWhite = true;
uint32_t flashNextMs  = 0;

// ── Loop timing ─────────────────────────────────────────────────
uint32_t nextSampleMs = 0;

// ── UI redraw flag ──────────────────────────────────────────────
bool uiDirty = true;

// ════════════════════════════════════════════════════════════════════
//  MPU6050 DRIVER
// ════════════════════════════════════════════════════════════════════

static void mpuWriteReg(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(MPU_I2C_ADDR);
    Wire.write(reg);
    Wire.write(val);
    Wire.endTransmission();
}

static bool mpuInit() {
    // Wake from sleep, use PLL with X gyro clock reference (more stable)
    mpuWriteReg(MPU_REG_PWR_MGMT_1, 0x01);
    delay(100);

    // Sample Rate = Gyro_Output_Rate / (1 + SMPLRT_DIV)
    // Gyro output = 1 kHz when DLPF enabled → div=9 → 100 Hz
    mpuWriteReg(MPU_REG_SMPLRT_DIV, 9);

    // DLPF_CFG = 3 → accel BW 44 Hz, gyro BW 42 Hz (good anti-alias for gestures)
    mpuWriteReg(MPU_REG_CONFIG, 0x03);

    // Gyro  full-scale = ±250 °/s  (FS_SEL  = 0b00)
    mpuWriteReg(MPU_REG_GYRO_CONFIG, 0x00);

    // Accel full-scale = ±2 g       (AFS_SEL = 0b00)
    mpuWriteReg(MPU_REG_ACCEL_CONFIG, 0x00);

    // Verify WHO_AM_I
    Wire.beginTransmission(MPU_I2C_ADDR);
    Wire.write(MPU_REG_WHO_AM_I);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_I2C_ADDR, (uint8_t)1);
    if (!Wire.available()) return false;
    return (Wire.read() == 0x68);
}

// Burst-reads all 6 axes (14 bytes). Returns false on I2C error.
static bool mpuRead(Vec3f &accel, Vec3f &gyro) {
    Wire.beginTransmission(MPU_I2C_ADDR);
    Wire.write(MPU_REG_ACCEL_OUT);
    if (Wire.endTransmission(false) != 0) return false;

    Wire.requestFrom(MPU_I2C_ADDR, (uint8_t)14);
    if (Wire.available() < 14) return false;

    auto readWord = []() -> int16_t {
        return (int16_t)((uint8_t)Wire.read() << 8 | (uint8_t)Wire.read());
    };

    int16_t ax = readWord(), ay = readWord(), az = readWord();
    readWord();  // temperature — discard
    int16_t gx = readWord(), gy = readWord(), gz = readWord();

    accel.x = (float)ax / ACCEL_LSB_PER_G;
    accel.y = (float)ay / ACCEL_LSB_PER_G;
    accel.z = (float)az / ACCEL_LSB_PER_G;
    gyro.x  = (float)gx / GYRO_LSB_PER_DPS;
    gyro.y  = (float)gy / GYRO_LSB_PER_DPS;
    gyro.z  = (float)gz / GYRO_LSB_PER_DPS;
    return true;
}

// ════════════════════════════════════════════════════════════════════
//  SIGNAL PROCESSING
// ════════════════════════════════════════════════════════════════════

// EMA per-axis: filtered = α·raw + (1-α)·filtered
static inline void emaUpdate(Vec3f &f, const Vec3f &raw, float alpha) {
    float beta = 1.0f - alpha;
    f.x = alpha * raw.x + beta * f.x;
    f.y = alpha * raw.y + beta * f.y;
    f.z = alpha * raw.z + beta * f.z;
}

/**
 * Roll angle (degrees) about the forearm axis.
 * Device flat on top of wrist, USB toward fingers:
 *   roll > 0 → right wrist tilt
 *   roll < 0 → left wrist tilt
 */
static float computeRoll(const Vec3f &a) {
    return atan2f(a.y, a.z) * (180.0f / (float)M_PI);
}

/**
 * Dynamic acceleration magnitude (gravity removed).
 *
 * FIX v1.1: Removed the second EMA pass on magnitude.
 * The input `a` is already accelFast (α=0.75 filtered). Applying
 * another EMA_ALPHA=0.20 smooth on top was double-smoothing the signal
 * and killing the jerk (pinch) peak by ~5–6×.  Now we just compute
 * |A| from the already-filtered axes and subtract 1g.
 */
static float computeDynMag(const Vec3f &a) {
    float mag = sqrtf(a.x*a.x + a.y*a.y + a.z*a.z);
    return fabsf(mag - 1.0f);   // always ≥ 0; no second smooth
}

/**
 * Jerk = |ΔA_dynamic / Δt|  (g/s)
 * First-order finite difference — captures the impulsive onset of a pinch.
 */
static float computeJerk(float dynMag) {
    float jerk = fabsf(dynMag - prevDynMag) / SAMPLE_DT_S;
    prevDynMag = dynMag;
    return jerk;
}

// ════════════════════════════════════════════════════════════════════
//  TILT STATE MACHINE  (hysteresis prevents chattering)
// ════════════════════════════════════════════════════════════════════

static TiltDir updateTilt(float roll) {
    switch (tiltCurrent) {
        case TiltDir::CENTER:
            if (roll >  TILT_ENTER_DEG) return TiltDir::RIGHT;
            if (roll < -TILT_ENTER_DEG) return TiltDir::LEFT;
            break;
        case TiltDir::RIGHT:
            if (roll <  TILT_EXIT_DEG)  return TiltDir::CENTER;
            break;
        case TiltDir::LEFT:
            if (roll > -TILT_EXIT_DEG)  return TiltDir::CENTER;
            break;
    }
    return tiltCurrent;
}

// ════════════════════════════════════════════════════════════════════
//  PINCH STATE MACHINE
//  4-phase FSM with per-phase timing:
//
//  IDLE ─[jerk>J]──────────────────► PEAK_1
//  PEAK_1 ─[jerk<J, valid dur]──────► WAIT_SECOND ─[gap>max]──► fire SINGLE → LOCKOUT
//  PEAK_1 ─[jerk<J, bad dur OR long]► IDLE
//  WAIT_SECOND ─[jerk>J, gap valid]─► PEAK_2
//  PEAK_2 ─[jerk<J, valid dur]──────► fire DOUBLE → LOCKOUT
//  PEAK_2 ─[bad dur]────────────────► fire SINGLE → LOCKOUT (first was real)
//  LOCKOUT ─[timeout]───────────────► IDLE
// ════════════════════════════════════════════════════════════════════

static Gesture updatePinch(float jerk) {
    uint32_t now    = millis();
    Gesture  result = Gesture::NONE;

    switch (pinchPhase) {

        // ── Waiting for any motion above threshold ─────────────────
        case PinchPhase::IDLE:
            if (jerk > JERK_THRESH) {
                pinchPhase   = PinchPhase::PEAK_1;
                phaseEntryMs = now;
                Serial.printf("[PINCH-FSM] IDLE → PEAK_1  jerk=%.2f\n", jerk);
            }
            break;

        // ── First peak in progress ─────────────────────────────────
        case PinchPhase::PEAK_1: {
            uint32_t dur = now - phaseEntryMs;

            if (jerk < JERK_THRESH) {
                // Peak ended — validate duration
                if (dur >= PINCH_DUR_MIN_MS && dur <= PINCH_DUR_MAX_MS) {
                    pinchPhase    = PinchPhase::WAIT_SECOND;
                    firstPeakEndMs = now;
                    Serial.printf("[PINCH-FSM] PEAK_1 → WAIT  dur=%ums\n", dur);
                } else {
                    // Too short or too long → noise or held motion
                    pinchPhase = PinchPhase::IDLE;
                    Serial.printf("[PINCH-FSM] PEAK_1 invalid dur=%ums → IDLE\n", dur);
                }
            } else if (dur > PINCH_DUR_MAX_MS) {
                // Sustained press — not a pinch
                pinchPhase = PinchPhase::IDLE;
                Serial.println("[PINCH-FSM] PEAK_1 timeout → IDLE");
            }
            break;
        }

        // ── Waiting to decide: single or double ────────────────────
        case PinchPhase::WAIT_SECOND: {
            uint32_t gap = now - firstPeakEndMs;

            if (jerk > JERK_THRESH) {
                if (gap >= DBL_GAP_MIN_MS) {
                    // Valid gap → second peak started
                    pinchPhase   = PinchPhase::PEAK_2;
                    phaseEntryMs = now;
                    Serial.printf("[PINCH-FSM] WAIT → PEAK_2  gap=%ums\n", gap);
                } else {
                    // Too soon → likely rebound noise; dismiss
                    pinchPhase = PinchPhase::IDLE;
                    Serial.println("[PINCH-FSM] Second peak too soon → IDLE");
                }
            } else if (gap > DBL_GAP_MAX_MS) {
                // Window closed — emit single pinch
                result        = Gesture::PINCH_SINGLE;
                pinchPhase    = PinchPhase::LOCKOUT;
                phaseEntryMs  = now;
                Serial.println("[PINCH-FSM] WAIT timeout → SINGLE");
            }
            break;
        }

        // ── Second peak in progress ────────────────────────────────
        case PinchPhase::PEAK_2: {
            uint32_t dur = now - phaseEntryMs;

            if (jerk < JERK_THRESH) {
                if (dur >= PINCH_DUR_MIN_MS && dur <= PINCH_DUR_MAX_MS) {
                    result = Gesture::PINCH_DOUBLE;
                    Serial.printf("[PINCH-FSM] PEAK_2 → DOUBLE  dur=%ums\n", dur);
                } else {
                    // Second peak was invalid — first was still real
                    result = Gesture::PINCH_SINGLE;
                    Serial.printf("[PINCH-FSM] PEAK_2 invalid dur=%ums → SINGLE\n", dur);
                }
                pinchPhase   = PinchPhase::LOCKOUT;
                phaseEntryMs = now;
            } else if (dur > PINCH_DUR_MAX_MS) {
                result        = Gesture::PINCH_SINGLE;
                pinchPhase    = PinchPhase::LOCKOUT;
                phaseEntryMs  = now;
                Serial.println("[PINCH-FSM] PEAK_2 timeout → SINGLE");
            }
            break;
        }

        // ── Post-event debounce ────────────────────────────────────
        case PinchPhase::LOCKOUT:
            if (now - phaseEntryMs >= LOCKOUT_MS) {
                pinchPhase = PinchPhase::IDLE;
                Serial.println("[PINCH-FSM] LOCKOUT → IDLE");
            }
            break;
    }

    return result;
}

// ════════════════════════════════════════════════════════════════════
//  HAPTIC DRIVER  (non-blocking)
// ════════════════════════════════════════════════════════════════════

static void hapticFire(uint32_t durationMs) {
    digitalWrite(PIN_HAPTIC, HIGH);
    hapticOn    = true;
    hapticEndMs = millis() + durationMs;
}

static void hapticTick() {
    if (hapticOn && millis() >= hapticEndMs) {
        digitalWrite(PIN_HAPTIC, LOW);
        hapticOn = false;
    }
}

// ════════════════════════════════════════════════════════════════════
//  OLED UI RENDERER
// ════════════════════════════════════════════════════════════════════

// Centre a string horizontally at a given Y, given textSize
static void oledCentreText(const char *str, uint8_t y, uint8_t textSize = 1) {
    oled.setTextSize(textSize);
    int16_t x1, y1;
    uint16_t w, h;
    oled.getTextBounds(str, 0, 0, &x1, &y1, &w, &h);
    oled.setCursor((OLED_WIDTH - w) / 2, y);
    oled.print(str);
}

// Small left-pointing solid or hollow arrow at (x,y)
static void drawLeftArrow(uint8_t cx, uint8_t cy, bool filled) {
    uint8_t hw = 10, hh = 8;
    if (filled)
        oled.fillTriangle(cx - hw, cy, cx + hw, cy - hh, cx + hw, cy + hh, SSD1306_WHITE);
    else
        oled.drawTriangle(cx - hw, cy, cx + hw, cy - hh, cx + hw, cy + hh, SSD1306_WHITE);
}

// Small right-pointing solid or hollow arrow at (x,y)
static void drawRightArrow(uint8_t cx, uint8_t cy, bool filled) {
    uint8_t hw = 10, hh = 8;
    if (filled)
        oled.fillTriangle(cx + hw, cy, cx - hw, cy - hh, cx - hw, cy + hh, SSD1306_WHITE);
    else
        oled.drawTriangle(cx + hw, cy, cx - hw, cy - hh, cx - hw, cy + hh, SSD1306_WHITE);
}

/**
 * Full screen redraw.
 * Layout (128×64):
 *   y=0..9   : thin status bar with "WristGesture"
 *   y=10     : separator line
 *   y=18..46 : big tilt label (LEFT / CENTER / RIGHT)
 *   y=28     : arrows (left / right, filled when active)
 *   y=55..63 : bottom hint
 */

static void drawUI(TiltDir tilt) {

    oled.clearDisplay();

    oled.setTextColor(SSD1306_WHITE);

    // top bar
    oled.setTextSize(1);

    oled.setCursor(2, 0);
    oled.print("WristGesture");

    oled.setCursor(92, 0);

    if (bleConnected)
        oled.print("BLE");
    else
        oled.print("---");

    oled.drawLine(0, 10, OLED_WIDTH - 1, 10, SSD1306_WHITE);

    // orientation
    const char *label = "CENTER";

    if (tilt == TiltDir::LEFT)
        label = "LEFT";

    else if (tilt == TiltDir::RIGHT)
        label = "RIGHT";

    oledCentreText(label, 16, 2);

    oled.display();
}

// ════════════════════════════════════════════════════════════════════
//  FLASH ANIMATION  (non-blocking)
//  Alternates between a full-white and full-black screen N times,
//  showing "PINCH!" text in inverse color each half-cycle.
// ════════════════════════════════════════════════════════════════════

static void flashStart() {
    flashActive  = true;
    flashCycle   = 0;
    flashIsWhite = true;
    flashNextMs  = millis();  // fire immediately
}

static void flashTick() {
    if (!flashActive) return;
    if (millis() < flashNextMs) return;

    oled.clearDisplay();

    if (flashIsWhite) {
        // Bright frame — invert screen, black text on white bg
        oled.fillRect(0, 0, OLED_WIDTH, OLED_HEIGHT, SSD1306_WHITE);
        oled.setTextColor(SSD1306_BLACK);
        oledCentreText("PINCH!", 22, 2);

        // Show which gesture type
        oled.setTextColor(SSD1306_BLACK);
        oled.setTextSize(1);
        oledCentreText("detected!", 42, 1);
    } else {
        // Dark frame
        oled.setTextColor(SSD1306_WHITE);
        oledCentreText("PINCH!", 22, 2);
        oled.setTextSize(1);
        oledCentreText("detected!", 42, 1);
        flashCycle++;
    }

    oled.display();

    flashIsWhite = !flashIsWhite;
    flashNextMs  = millis() + FLASH_HALF_MS;

    if (flashCycle >= FLASH_CYCLES) {
        flashActive = false;
        // Force redraw of normal UI after flash
        uiDirty = true;
    }
}

// =====================================================
// BLE CALLBACKS
// =====================================================

class WristBLECallbacks : public NimBLEServerCallbacks {

    void onConnect(NimBLEServer*) override {
        bleConnected = true;
        Serial.println("[BLE] Client connected");
    }

    void onDisconnect(NimBLEServer*) override {
        bleConnected = false;

        Serial.println("[BLE] Client disconnected");

        NimBLEDevice::startAdvertising();
    }
};

// =====================================================
// BLE INIT
// =====================================================


static void bleInit() {

    NimBLEDevice::init(BLE_DEVICE_NAME);

    NimBLEDevice::setMTU(185);

    NimBLEDevice::setPower(ESP_PWR_LVL_P9);

    pBleServer = NimBLEDevice::createServer();

    pBleServer->setCallbacks(new WristBLECallbacks());

    NimBLEService* pService =
        pBleServer->createService(BLE_SERVICE_UUID);

    pDataChar = pService->createCharacteristic(
        BLE_DATA_CHAR_UUID,
        NIMBLE_PROPERTY::READ |
        NIMBLE_PROPERTY::NOTIFY
    );

    pDataChar->setValue("boot");

    pService->start();

    NimBLEAdvertising* pAdvertising =
        NimBLEDevice::getAdvertising();

    pAdvertising->addServiceUUID(BLE_SERVICE_UUID);

    pAdvertising->setScanResponse(true);

    NimBLEDevice::startAdvertising();

    Serial.println("[BLE] Advertising started");
}

// =====================================================
// SEND LIVE DATA
// =====================================================

static void bleSendData(const char *orientation) {

    if (!bleConnected || !pDataChar) return;

    char buf[185];

    snprintf(
        buf,
        sizeof(buf),

        "{\"t\":\"d\","
        "\"o\":\"%s\","
        "\"ax\":%.3f,"
        "\"ay\":%.3f,"
        "\"az\":%.3f,"
        "\"gx\":%.2f,"
        "\"gy\":%.2f,"
        "\"gz\":%.2f,"
        "\"j\":%.2f}",

        orientation,
        telAx,
        telAy,
        telAz,
        telGx,
        telGy,
        telGz,
        telJerk
    );

    pDataChar->setValue((uint8_t*)buf, strlen(buf));

    pDataChar->notify();
}


// =====================================================
// SEND GESTURE EVENT
// =====================================================

static void bleSendGesture(int gestureNum) {

    if (!bleConnected || !pDataChar) return;

    char buf[32];

    snprintf(
        buf,
        sizeof(buf),
        "{\"t\":\"p\",\"n\":%d}",
        gestureNum
    );

    pDataChar->setValue((uint8_t*)buf, strlen(buf));

    pDataChar->notify();
}
// ════════════════════════════════════════════════════════════════════
//  SETUP
// ════════════════════════════════════════════════════════════════════

void setup() {
    Serial.begin(115200);
    delay(300);
    Serial.println();
    Serial.println("╔══════════════════════════════════╗");
    Serial.println("║  WristGesture v1.1 — boot        ║");
    Serial.println("╚══════════════════════════════════╝");

    bleInit();
    
    // ── GPIO ───────────────────────────────────────────────────
    pinMode(PIN_HAPTIC, OUTPUT);
    digitalWrite(PIN_HAPTIC, LOW);

    // ── I2C @ 400 kHz ──────────────────────────────────────────
    Wire.begin(PIN_SDA, PIN_SCL);
    Wire.setClock(400000);

    // ── OLED init ───────────────────────────────────────────────
    if (!oled.begin(SSD1306_SWITCHCAPVCC, OLED_I2C_ADDR)) {
        Serial.println("[FATAL] OLED SSD1306 not found!");
        // Blink LED forever so we can debug without display
        while (true) {
            digitalWrite(PIN_HAPTIC, HIGH); delay(200);
            digitalWrite(PIN_HAPTIC, LOW);  delay(200);
        }
    }
    oled.clearDisplay();
    oled.setTextColor(SSD1306_WHITE);
    oled.setTextSize(1);
    oledCentreText("WristGesture", 10, 1);
    oledCentreText("Initialising...", 18, 1);
    oled.display();

    // ── MPU6050 init ────────────────────────────────────────────
    if (!mpuInit()) {
        Serial.println("[FATAL] MPU6050 not found! Check wiring & I2C address.");
        oled.clearDisplay();
        oled.setTextColor(SSD1306_WHITE);
        oledCentreText("MPU6050 ERROR", 4, 1);
        oledCentreText("Check wiring!", 18, 1);
        oled.display();
        while (true) delay(1000);
    }
    Serial.println("[OK] MPU6050 detected");
    Serial.printf("     Accel: ±2g  (1 LSB = %.4f g)\n",  1.0f / ACCEL_LSB_PER_G);
    Serial.printf("     Gyro:  ±250 dps  (1 LSB = %.4f °/s)\n", 1.0f / GYRO_LSB_PER_DPS);
    Serial.printf("     Sample rate: %d Hz\n", SAMPLE_HZ);

    // ── EMA warm-up (let filter settle before we start detecting) ─
    Serial.print("[INIT] EMA warm-up");
    Vec3f wa, wg;
    for (int i = 0; i < 80; i++) {
        if (mpuRead(wa, wg)) {
            emaUpdate(accelFilt, wa, EMA_ALPHA);
            emaUpdate(accelFast, wa, EMA_ALPHA_FAST);   // warm up fast path too
        }
        delay(SAMPLE_DT_MS);
        if (i % 20 == 0) Serial.print(".");
    }
    // Seed prevDynMag from the settled fast buffer
    {
        float mag0 = sqrtf(accelFast.x*accelFast.x +
                           accelFast.y*accelFast.y +
                           accelFast.z*accelFast.z);
        prevDynMag = fabsf(mag0 - 1.0f);
    }
    Serial.println(" done");

    oled.clearDisplay();
    oledCentreText("Ready!", 10, 2);
    oled.display();
    delay(600);

    // Initial UI draw
    drawUI(tiltCurrent);

    nextSampleMs = millis();
    Serial.println("[RUN] entering main loop");
    Serial.println("── Serial output ──────────────────────────────────────");
    Serial.println("  roll(deg)  jerk(g/s)  dyn(g)  gesture");
    Serial.println("───────────────────────────────────────────────────────");
}

// ════════════════════════════════════════════════════════════════════
//  MAIN LOOP
// ════════════════════════════════════════════════════════════════════

void loop() {
    uint32_t now = millis();

    // ── Non-blocking peripheral ticks ──────────────────────────
    hapticTick();
    flashTick();

    // ── Fixed-rate IMU sample ───────────────────────────────────
    if (now < nextSampleMs) return;
    nextSampleMs = now + SAMPLE_DT_MS;

    Vec3f rawAccel, rawGyro;
    if (!mpuRead(rawAccel, rawGyro)) {
        // I2C glitch — skip this sample, don't disrupt FSMs
        Serial.println("[WARN] IMU read failed — skipping sample");
        return;
    }

    // ── Stage 1: EMA filter ────────────────────────────────────
    emaUpdate(accelFilt, rawAccel, EMA_ALPHA);
    emaUpdate(accelFast, rawAccel, EMA_ALPHA_FAST);

    // ── Stage 2: Roll angle → tilt direction ──────────────────
    float roll    = computeRoll(accelFilt);
    TiltDir newTilt = updateTilt(roll);

    if (newTilt != tiltCurrent) {
        tiltCurrent = newTilt;
        uiDirty = true;
        const char *names[] = {"CENTER", "LEFT", "RIGHT"};
        Serial.printf("[TILT] → %s  (roll=%.1f°)\n",
                      names[(int)tiltCurrent], roll);
    }
        // cache telemetry
    telAx = rawAccel.x;
    telAy = rawAccel.y;
    telAz = rawAccel.z;

    telGx = rawGyro.x;
    telGy = rawGyro.y;
    telGz = rawGyro.z;

    // ── Stage 3: Dynamic magnitude + jerk ─────────────────────
    float dynMag = computeDynMag(accelFast);   // fast path — not double-smoothed
    float jerk   = computeJerk(dynMag);

    if (jerk > peakJerkSeen) peakJerkSeen = jerk;
    if (millis() - jerkPrintMs > 2000) {
        Serial.printf("[JERK-PEAK] last 2s peak = %.3f g/s  (thresh=%.1f)\n",
                      peakJerkSeen, JERK_THRESH);
        peakJerkSeen = 0.0f;
        jerkPrintMs  = millis();
    }

    telJerk = jerk;

    // ── Stage 4: Pinch FSM ─────────────────────────────────────
    Gesture gesture = updatePinch(jerk);

    // ── Stage 5: React to gesture ─────────────────────────────
    if (gesture == Gesture::PINCH_SINGLE) {
        bleSendGesture(1);

        Serial.println("[GESTURE] ★ SINGLE PINCH ★");
        flashStart();
        hapticFire(HAPTIC_SHORT_MS);
        uiDirty = true;
    } else if (gesture == Gesture::PINCH_DOUBLE) {
        bleSendGesture(2);
        Serial.println("[GESTURE] ★★ DOUBLE PINCH ★★");
        flashStart();
        hapticFire(HAPTIC_LONG_MS);
        uiDirty = true;
    }

    // ── Stage 6: Redraw UI (only when something changed) ──────
    if (uiDirty && !flashActive) {
        drawUI(tiltCurrent);
        uiDirty = false;
    }

    if (millis() - lastBleDataMs >= BLE_DATA_INTERVAL) {
    lastBleDataMs = millis();
    const char *oriStr =
        (tiltCurrent == TiltDir::LEFT)  ? "LEFT" :
        (tiltCurrent == TiltDir::RIGHT) ? "RIGHT" :
                                          "CENTER";

    bleSendData(oriStr);
}

    // ── Debug telemetry (uncomment for Serial Plotter tuning) ──
    // Paste output into PlatformIO Serial Plotter.
    // jerk should spike to 8–50+ on pinch, sit near 0 at rest.
    //
    // Serial.printf("%.2f,%.3f,%.4f\n", roll, jerk, dynMag);
}