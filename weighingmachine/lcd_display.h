#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include "scale_display.h"
#include "config.h"

/**
 * Concrete ScaleDisplay implementation for standard I2C 16x2 LCD (PCF8574 backpack).
 */
class LcdDisplay : public ScaleDisplay {
private:
  LiquidCrystal_I2C* _lcd;
  uint8_t _address;
  bool _tick;

public:
  LcdDisplay(uint8_t defaultAddr = LCD_DEFAULT_ADDR)
    : _lcd(nullptr), _address(defaultAddr), _tick(false) {}

  virtual ~LcdDisplay() {
    if (_lcd) {
      delete _lcd;
      _lcd = nullptr;
    }
  }

  virtual const char* getDisplayName() const override {
    return "I2C 16x2 LCD (PCF8574)";
  }

  virtual bool begin() override {
    Serial.println("[INFO] Scanning I2C bus for 16x2 LCD...");
    bool lcdFound = false;
    uint8_t candidates[] = {0x27, 0x3F};

    for (uint8_t addr : candidates) {
      Wire.beginTransmission(addr);
      if (Wire.endTransmission() == 0) {
        _address = addr;
        lcdFound = true;
        Serial.printf("[INFO] I2C LCD detected at address: 0x%02X\n", _address);
        break;
      }
    }

    if (!lcdFound) {
      Serial.printf("[WARN] No LCD detected at 0x27 or 0x3F, defaulting to 0x%02X\n", _address);
    }

    if (_lcd) delete _lcd;
    _lcd = new LiquidCrystal_I2C(_address, 16, 2);
    _lcd->init();
    _lcd->backlight();
    _lcd->clear();
    return true;
  }

  virtual void showWeight(float weight, WeightUnit unit, bool isStable, bool isHeld,
                          int32_t netCounts, int32_t rawADC, bool saturated) override {
    if (!_lcd) return;
    _tick = !_tick;

    // Line 0: Weight value + Unit + Activity indicator
    char weightBuf[16];
    formatWeightString(weight, unit, weightBuf, sizeof(weightBuf));
    char lineBuf[24];
    const char* label = (unit == UNIT_PCS) ? "Qty" : "Wt";
    snprintf(lineBuf, sizeof(lineBuf), "%s:%-10s %c", label, weightBuf, isHeld ? 'H' : (_tick ? '*' : ' '));
    lineBuf[16] = '\0'; // Guarantee 16 chars for 16x2 LCD
    _lcd->setCursor(0, 0);
    _lcd->print(lineBuf);

    // Line 1: Status & Net ADC counts or saturation warning
    _lcd->setCursor(0, 1);
    if (saturated || rawADC <= -ADC_SATURATION_LIMIT || rawADC >= ADC_SATURATION_LIMIT) {
      _lcd->print("ADC SATURATED!  ");
    } else {
      if (isHeld) {
        snprintf(lineBuf, sizeof(lineBuf), "HOLD    %7ld", (long)netCounts);
      } else if (isStable) {
        snprintf(lineBuf, sizeof(lineBuf), "STABLE  %7ld", (long)netCounts);
      } else {
        snprintf(lineBuf, sizeof(lineBuf), "Net:    %7ld", (long)netCounts);
      }
      lineBuf[16] = '\0';
      _lcd->print(lineBuf);
    }
  }

  virtual void showTareProgress(float animWeight, WeightUnit unit) override {
    if (!_lcd) return;
    char weightBuf[16];
    formatWeightString(animWeight, unit, weightBuf, sizeof(weightBuf));
    char lineBuf[24];
    snprintf(lineBuf, sizeof(lineBuf), "Wt:%-10s   ", weightBuf);
    lineBuf[16] = '\0';
    _lcd->setCursor(0, 0);
    _lcd->print(lineBuf);
  }

  virtual void showShutdownCountdown(uint16_t holdMs, uint16_t maxHoldMs) override {
    if (!_lcd) return;
    if (holdMs >= 700 && holdMs < maxHoldMs) {
      uint16_t remainMs = (maxHoldMs > holdMs) ? (maxHoldMs - holdMs) : 0;
      char lineBuf[24];
      snprintf(lineBuf, sizeof(lineBuf), "OFF IN: %1.1fs   ", (float)remainMs / 1000.0f);
      lineBuf[16] = '\0';
      _lcd->setCursor(0, 1);
      _lcd->print(lineBuf);
    }
  }

  virtual void showSampleWizard(uint16_t sampleCount, float currentWeight, const char* promptMsg = nullptr) override {
    if (!_lcd) return;
    char line0[24];
    char line1[24];
    if (promptMsg != nullptr && strlen(promptMsg) > 0) {
      snprintf(line0, sizeof(line0), "%-16.16s", promptMsg);
      snprintf(line1, sizeof(line1), "ESC  -   +   OK ");
    } else {
      snprintf(line0, sizeof(line0), "SAMPLE: %3u pcs ", sampleCount);
      snprintf(line1, sizeof(line1), "ESC  -   +   OK ");
    }
    line0[16] = '\0';
    line1[16] = '\0';
    _lcd->setCursor(0, 0);
    _lcd->print(line0);
    _lcd->setCursor(0, 1);
    _lcd->print(line1);
  }

  virtual void showStatus(const char* line1, const char* line2 = nullptr, uint16_t delayMs = 0) override {
    if (!_lcd) return;
    _lcd->clear();
    _lcd->setCursor(0, 0);
    _lcd->print(line1);

    if (line2 != nullptr) {
      _lcd->setCursor(0, 1);
      _lcd->print(line2);
    }

    if (delayMs > 0) {
      delay(delayMs);
      _lcd->clear();
    }
  }

  virtual void showPowerOff() override {
    showStatus("POWER OFF", "Goodbye!", 800);
  }

  virtual void powerDown() override {
    if (_lcd) {
      _lcd->noBacklight();
      _lcd->noDisplay();
    }
  }
};
