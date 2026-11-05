#pragma once

#include <Arduino.h>

// ---------------------- DISPLAY SELECTION --------------------
// Select one of the display options:
// - DISPLAY_MODE_AUTO : Probes I2C bus at startup to auto-detect OLED or LCD
// - DISPLAY_MODE_LCD  : Forces 16x2 I2C LCD (PCF8574 backpack)
// - DISPLAY_MODE_OLED : Forces 0.91" I2C OLED (SSD1306 128x32)
#define DISPLAY_MODE_AUTO 0
#define DISPLAY_MODE_LCD  1
#define DISPLAY_MODE_OLED 2

#ifndef DEFAULT_DISPLAY_MODE
#define DEFAULT_DISPLAY_MODE DISPLAY_MODE_AUTO
#endif

// ---------------------- PIN DEFINITIONS ----------------------
#define I2C_SDA_PIN          3     // Hardware I2C SDA (GPIO 3)
#define I2C_SCL_PIN          4     // Hardware I2C SCL (GPIO 4)
#define HOLD_BUTTON_PIN      1     // Dedicated Hold Button (GPIO 1, active LOW)
#define TARE_BUTTON_PIN      0     // Dedicated Tare Button (GPIO 0, active LOW / pull to GND)
#define UNIT_BUTTON_PIN      6     // Dedicated Unit Change Button (GPIO 6, active LOW / pull to GND)
#define PCS_BUTTON_PIN       5     // Dedicated PCS Counting Button (GPIO 5, active LOW / pull to GND)
#define POWER_BANK_PULSE_PIN 7     // Pulse pin to keep IP5306 power bank alive (pull to GND every 10s)

// ---------------------- 4-BUTTON ARROW NAVIGATION MAPPING ----
// Physical layout Left-to-Right: GPIO 1 -> 0 -> 6 -> 5
// Momentarily re-bound during modal screens (e.g. PCS Sample Wizard):
#define NAV_BTN_CANCEL       HOLD_BUTTON_PIN  // [ESC / Cancel / Back] (Leftmost - GPIO 1)
#define NAV_BTN_DEC          TARE_BUTTON_PIN  // [- / Previous]        (Middle-Left - GPIO 0)
#define NAV_BTN_INC          UNIT_BUTTON_PIN  // [+ / Next]            (Middle-Right - GPIO 6)
#define NAV_BTN_CONFIRM      PCS_BUTTON_PIN   // [OK / Set / Confirm]  (Rightmost - GPIO 5)

// Power On/Off Long-Press Assignment:
// Assignable to any button (HOLD_BUTTON_PIN, TARE_BUTTON_PIN, UNIT_BUTTON_PIN, or PCS_BUTTON_PIN).
// Long-pressing (>= 1.5s) this button enters sleep/power off; wakes up on button press.
#define POWER_OFF_BUTTON_PIN HOLD_BUTTON_PIN

// Backward compatibility aliases
#define BOOT_BUTTON_PIN      HOLD_BUTTON_PIN
#define TARE_PIN_D2          TARE_BUTTON_PIN

// ---------------------- I2C ADDRESSES ------------------------
#define NAU7802_I2C_ADDR     0x2A  // Fixed hardware address for NAU7802 ADC
#define LCD_DEFAULT_ADDR     0x27  // Default address for PCF8574 LCD (scans 0x27, 0x3F)
#define OLED_DEFAULT_ADDR    0x3C  // Default address for 0.91" SSD1306 OLED (scans 0x3C, 0x3D)

// ---------------------- OLED DISPLAY CONFIG ------------------
#define OLED_SCREEN_WIDTH    128   // OLED display width in pixels
#define OLED_SCREEN_HEIGHT   32    // 0.91" OLED display height in pixels
#define OLED_RESET_PIN       -1    // Reset pin # (-1 if sharing Arduino reset)
#define DISPLAY_ORIENTATION  180   // Display orientation: 0 (normal), 180 (landscape flipped)
#define OLED_ORIENTATION     DISPLAY_ORIENTATION

// ---------------------- SCALE FILTER & TIMING ----------------
#define ADC_SATURATION_LIMIT         8388600 // 24-bit ADC saturation boundary (full scale 8,388,607)
#define FILTER_SIZE                  16      // 16-sample rolling buffer (~400ms window at 40 SPS)
#define DISPLAY_INTERVAL_MS          100     // 10 Hz display update rate
#define BUTTON_DEBOUNCE_MS           30      // Mechanical switch debounce integration time (30 ms)
#define POWER_OFF_HOLD_MS            1500    // 1.5 seconds hold for power off / deep sleep
#define BOOT_SLEEP_HOLD_MS           POWER_OFF_HOLD_MS
#define AUTO_SHUTDOWN_TIMEOUT_MS     -1      // inactivity timeout in ms (set <= 0 to disable)
#define POWER_BANK_PULSE_INTERVAL_MS 10000   // IP5306 keep-alive pulse interval (10 seconds)
#define POWER_BANK_PULSE_DURATION_MS 100     // IP5306 keep-alive pull-to-GND pulse duration (100 ms)

// ---------------------- RAW ADC METROLOGY & STABILITY --------
// All filtering, stability, and zero-tracking operate directly in native ADC
// raw counts, completely decoupled from calibrated scale factor / sensitivity.
#define FILTER_SLEW_COUNTS           280     // Slew delta threshold in ADC counts (~3g @ 93 cnt/g)
#define FILTER_SETTLE_COUNTS         140     // Settle delta threshold in ADC counts (~1.5g @ 93 cnt/g)
#define STABILITY_WINDOW_SIZE        12      // 12-sample rolling window (~300ms at 40 SPS)
#define STABILITY_SPREAD_COUNTS      85      // Max peak-to-peak variation in raw counts for stability
#define STABILITY_CONFIRM_FRAMES     10      // Frames within limit required to declare stable (~250ms)
#define ZERO_DEADBAND_COUNTS         55      // Center-of-Zero breakout threshold (~0.6g, detects 1g+ weights)
#define ZERO_RELEASE_COUNTS          30      // Center-of-Zero return threshold (~0.3g, 25 count hysteresis)
#define AUTO_ZERO_BAND_COUNTS        35      // Zero-tracking window (~0.37g, will not swallow small items)
#define AUTO_ZERO_MAX_STEP_COUNTS    1       // Max zero-tracking step per conversion frame (counts)
#define DISPLAY_LOCK_HYSTERESIS_G    0.75f   // Grams hysteresis required to break integer display lock

// ---------------------- DEFAULT CALIBRATION ------------------
#define DEFAULT_SCALE_FACTOR     93.44f  // Default counts per gram
#define DEFAULT_TARE_OFFSET      81628   // Default tare offset
#define PREFS_NAMESPACE          "scale_cfg"

// ---------------------- PCS COUNTING CONFIGURATION -----------
#define PCS_SETUP_HOLD_MS            1000    // Long press on PCS button to enter wizard (1.0 second)
#define SAMPLE_SETUP_TIMEOUT_MS      -1      // Wizard inactivity timeout in ms (set <= 0 to disable)
#define DEFAULT_SAMPLE_COUNT         10      // Default sampling count
#define MIN_SAMPLE_TOTAL_WEIGHT_G    0.20f   // Minimum total sample weight required to teach APW
#define MIN_PIECE_WEIGHT_G           0.005f  // Minimum piece weight allowed (~5mg resolution)

