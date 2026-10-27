#pragma once

#include <Arduino.h>
#include "scale_types.h"

/**
 * Pluggable Abstract Display Interface for Weighing Scale.
 * Enables seamless swapping between I2C 16x2 LCD, 0.91" I2C OLED,
 * or custom display modules.
 */
class ScaleDisplay {
public:
  virtual ~ScaleDisplay() {}

  // Lifecycle
  virtual bool begin() = 0;
  virtual const char* getDisplayName() const = 0;

  // Primary measurement display
  virtual void showWeight(float weight, WeightUnit unit, bool isStable, bool isHeld,
                          int32_t netCounts, int32_t rawADC, bool saturated) = 0;

  // Animated tare progress display
  virtual void showTareProgress(float animWeight, WeightUnit unit) = 0;

  // Notification / diagnostic / splash screen
  virtual void showStatus(const char* line1, const char* line2 = nullptr, uint16_t delayMs = 0) = 0;

  // Visual countdown indicator during sleep hold button press
  virtual void showShutdownCountdown(uint16_t holdMs, uint16_t maxHoldMs) {
    (void)holdMs;
    (void)maxHoldMs;
  }

  // Graceful shutdown screen & hardware sleep
  virtual void showPowerOff() = 0;
  virtual void powerDown() = 0;
};
