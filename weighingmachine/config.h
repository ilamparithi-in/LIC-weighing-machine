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
#define I2C_SDA_PIN          3     // Hardware I2C SDA (GPIO 6)
#define I2C_SCL_PIN          4     // Hardware I2C SCL (GPIO 7)
#define BOOT_BUTTON_PIN      1     // On-board BOOT Button (GPIO 9, active LOW: Click=HOLD, Hold 1.5s=SLEEP/WAKE)
#define TARE_PIN_D2          0     // Dedicated External Tare Button on D2 (GPIO 2, active LOW / pull to GND)
#define UNIT_BUTTON_PIN      6     // External Unit Change Button on D4 (GPIO 4, active LOW / pull to GND)
#define POWER_BANK_PULSE_PIN 7     // Pulse pin to keep IP5306 power bank alive (pull to GND every 10s)

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
#define FILTER_SIZE                  8       // 8-sample rolling buffer (~200ms window at 40 SPS)
#define DISPLAY_INTERVAL_MS          100     // 10 Hz display update rate
#define BOOT_SLEEP_HOLD_MS           1500    // 1.5 seconds hold for power off / deep sleep
#define AUTO_SHUTDOWN_TIMEOUT_MS     120000  // 2 minutes inactivity timeout
#define POWER_BANK_PULSE_INTERVAL_MS 10000   // IP5306 keep-alive pulse interval (10 seconds)
#define POWER_BANK_PULSE_DURATION_MS 100     // IP5306 keep-alive pull-to-GND pulse duration (100 ms)

// ---------------------- DEFAULT CALIBRATION ------------------
#define DEFAULT_SCALE_FACTOR     93.44f  // Default counts per gram
#define DEFAULT_TARE_OFFSET      81628   // Default tare offset
#define PREFS_NAMESPACE          "scale_cfg"
