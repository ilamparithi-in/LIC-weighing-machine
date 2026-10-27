#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "scale_display.h"
#include "config.h"

/**
 * Concrete ScaleDisplay implementation for 0.91" I2C OLED (SSD1306 128x32).
 */
class OledDisplay : public ScaleDisplay {
private:
  Adafruit_SSD1306 _display;
  uint8_t _address;

public:
  OledDisplay(uint8_t defaultAddr = OLED_DEFAULT_ADDR)
    : _display(OLED_SCREEN_WIDTH, OLED_SCREEN_HEIGHT, &Wire, OLED_RESET_PIN),
      _address(defaultAddr) {}

  virtual ~OledDisplay() {}

  virtual const char* getDisplayName() const override {
    return "0.91\" I2C OLED (SSD1306 128x32)";
  }

  virtual bool begin() override {
    Serial.println("[INFO] Scanning I2C bus for 0.91\" OLED...");
    bool oledFound = false;
    uint8_t candidates[] = {0x3C, 0x3D};

    for (uint8_t addr : candidates) {
      Wire.beginTransmission(addr);
      if (Wire.endTransmission() == 0) {
        _address = addr;
        oledFound = true;
        Serial.printf("[INFO] OLED detected at address: 0x%02X\n", _address);
        break;
      }
    }

    if (!oledFound) {
      Serial.printf("[WARN] OLED not detected at 0x3C or 0x3D, defaulting to 0x%02X\n", _address);
    }

    if (!_display.begin(SSD1306_SWITCHCAPVCC, _address, false, false)) {
      return false;
    }

    // Ensure Wire frequency and pins are preserved
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN, 400000);

    _display.clearDisplay();
    _display.setTextColor(SSD1306_WHITE);
    _display.display();
    return true;
  }

  virtual void showWeight(float weight, WeightUnit unit, bool isStable, bool isHeld,
                          int32_t netCounts, int32_t rawADC, bool saturated) override {
    (void)isStable;
    (void)netCounts;
    _display.clearDisplay();
    _display.setTextColor(SSD1306_WHITE);

    if (saturated || rawADC <= -8388600 || rawADC >= 8388600) {
      _display.setTextSize(2);
      _display.setCursor(4, 8);
      _display.print("! OVERLOAD !");
      _display.display();
      return;
    }

    char numStr[16];
    formatWeightNumber(weight, unit, numStr, sizeof(numStr));
    const char* unitStr = getUnitSymbol(unit);

    int lenNum = strlen(numStr);
    int lenUnit = strlen(unitStr);

    // Unit is fixed on the right side of the display (flush right with 2px margin)
    int unitW = (lenUnit * 12) - 2;
    int unitX = 126 - unitW;
    int unitY = 12; // Size 2 baseline (y=12 to y=28)

    // Digits right-aligned with a 6px gap before the fixed unit
    int digitsRightX = unitX - 6;

    // Try Size 3 for digits (18px per char, height 24px)
    int wNum3 = (lenNum * 18) - 3;
    if (wNum3 <= digitsRightX) {
      int numX = digitsRightX - wNum3;
      _display.setTextSize(3);
      _display.setCursor(numX, 4); // y=4 centers 24px height
      _display.print(numStr);
    } else {
      // If number is too wide for Size 3, fall back to Size 2
      int wNum2 = (lenNum * 12) - 2;
      int numX = digitsRightX - wNum2;
      if (numX < 0) numX = 0;
      _display.setTextSize(2);
      _display.setCursor(numX, 8); // y=8 centers 16px height
      _display.print(numStr);
    }

    // Draw HOLD badge in top-left corner if active
    if (isHeld) {
      _display.fillRect(0, 0, 26, 9, SSD1306_WHITE);
      _display.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
      _display.setTextSize(1);
      _display.setCursor(1, 1);
      _display.print("HOLD");
      _display.setTextColor(SSD1306_WHITE);
    }

    // Draw fixed unit on right
    _display.setTextSize(2);
    _display.setCursor(unitX, unitY);
    _display.print(unitStr);

    _display.display();
  }

  virtual void showTareProgress(float animWeight, WeightUnit unit) override {
    showWeight(animWeight, unit, false, false, 0, 0, false);
  }

  virtual void showStatus(const char* line1, const char* line2 = nullptr, uint16_t delayMs = 0) override {
    _display.clearDisplay();
    _display.setTextColor(SSD1306_WHITE);

    if (line2 == nullptr) {
      _display.setTextSize(2);
      _display.setCursor(0, 8);
      _display.print(line1);
    } else {
      _display.setTextSize(1);
      _display.setCursor(0, 4);
      _display.print(line1);
      _display.drawFastHLine(0, 16, 128, SSD1306_WHITE);
      _display.setCursor(0, 20);
      _display.print(line2);
    }

    _display.display();
    if (delayMs > 0) {
      delay(delayMs);
    }
  }

  virtual void showShutdownCountdown(uint16_t holdMs, uint16_t maxHoldMs) override {
    if (holdMs >= 700 && holdMs < maxHoldMs) {
      int width = map(holdMs, 700, maxHoldMs, 20, 128);
      _display.fillRect(0, 0, width, 4, SSD1306_WHITE);
      _display.display();
    }
  }

  virtual void showPowerOff() override {
    showStatus("POWER OFF", "Goodbye!", 800);
  }

  virtual void powerDown() override {
    _display.clearDisplay();
    _display.display();
    _display.ssd1306_command(SSD1306_DISPLAYOFF);
  }
};
