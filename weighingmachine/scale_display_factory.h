#pragma once

#include <Arduino.h>
#include <Wire.h>
#include "config.h"
#include "scale_display.h"
#include "lcd_display.h"
#include "oled_display.h"

/**
 * Factory function to instantiate the chosen or auto-detected display driver.
 */
inline ScaleDisplay* createScaleDisplay(int mode = DEFAULT_DISPLAY_MODE) {
  if (mode == DISPLAY_MODE_LCD) {
    Serial.println("[DISPLAY] Selected: I2C 16x2 LCD");
    return new LcdDisplay();
  } else if (mode == DISPLAY_MODE_OLED) {
    Serial.println("[DISPLAY] Selected: 0.91\" I2C OLED (SSD1306)");
    return new OledDisplay();
  } else {
    // Auto-detect mode: Probe I2C bus for known display addresses
    Serial.println("[DISPLAY] Auto-detecting display on I2C bus...");

    // Check for 0.91" OLED (SSD1306)
    uint8_t oledAddrs[] = {0x3C, 0x3D};
    for (uint8_t addr : oledAddrs) {
      Wire.beginTransmission(addr);
      if (Wire.endTransmission() == 0) {
        Serial.printf("[DISPLAY] Auto-detected OLED display at 0x%02X\n", addr);
        return new OledDisplay(addr);
      }
    }

    // Check for 16x2 LCD (PCF8574 backpack)
    uint8_t lcdAddrs[] = {0x27, 0x3F};
    for (uint8_t addr : lcdAddrs) {
      Wire.beginTransmission(addr);
      if (Wire.endTransmission() == 0) {
        Serial.printf("[DISPLAY] Auto-detected 16x2 LCD at 0x%02X\n", addr);
        return new LcdDisplay(addr);
      }
    }

    Serial.println("[DISPLAY] No display responded on I2C bus, defaulting to OLED driver.");
    return new OledDisplay();
  }
}
