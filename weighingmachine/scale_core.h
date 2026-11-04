#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <7Semi_NAU7802.h>
#include <Preferences.h>
#include "esp_bt.h"
#include "esp_sleep.h"

#include "config.h"
#include "scale_types.h"
#include "scale_display.h"

/**
 * ScaleCore: Encapsulates all load cell signal processing, digital filtering,
 * calibration, tare, button debounce, non-volatile storage, power management,
 * and serial command parsing.
 */
class ScaleCore {
private:
  ScaleDisplay* _display;
  NAU7802_7Semi _scale;
  Preferences _prefs;

  // Calibration and Settings
  float _scaleFactor;
  int32_t _tareOffset;
  bool _isCalibrated;
  WeightUnit _currentUnit;

  // Digital Filtering & Rolling Buffer
  int32_t _adcBuffer[FILTER_SIZE];
  int _bufferIndex;
  bool _bufferFull;
  float _filteredWeight;
  float _lockedWeight;
  bool _isStable;
  int _stableFrames;
  int32_t _latestFilteredADC;

  // Hold State
  bool _isHeld;
  float _heldWeight;

  // Buttons & Inactivity Timers
  unsigned long _lastActivityTime;
  float _lastActivityWeight;
  unsigned long _lastDisplayUpdate;

  // BOOT button (GPIO 9) tracking
  unsigned long _bootPressStartTime;
  bool _bootShutdownTriggered;

  // Tare (D2) and Unit (D4) button debouncing
  unsigned long _lastTareTriggerTime;
  bool _tareButtonActive;
  unsigned long _lastUnitTriggerTime;
  bool _unitButtonActive;

public:
  ScaleCore(ScaleDisplay* display)
    : _display(display),
      _scale(Wire),
      _scaleFactor(DEFAULT_SCALE_FACTOR),
      _tareOffset(DEFAULT_TARE_OFFSET),
      _isCalibrated(true),
      _currentUnit(UNIT_G),
      _bufferIndex(0),
      _bufferFull(false),
      _filteredWeight(0.0f),
      _lockedWeight(0.0f),
      _isStable(false),
      _stableFrames(0),
      _latestFilteredADC(0),
      _isHeld(false),
      _heldWeight(0.0f),
      _lastActivityTime(0),
      _lastActivityWeight(0.0f),
      _lastDisplayUpdate(0),
      _bootPressStartTime(0),
      _bootShutdownTriggered(false),
      _lastTareTriggerTime(0),
      _tareButtonActive(false),
      _lastUnitTriggerTime(0),
      _unitButtonActive(false) {
    for (int i = 0; i < FILTER_SIZE; i++) {
      _adcBuffer[i] = 0;
    }
  }

  void begin() {
    // 1. Disable Bluetooth controller to reduce 2.4 GHz RF emissions & power ripple
    btStop();
    esp_bt_controller_disable();
    esp_bt_controller_deinit();
    esp_bt_controller_mem_release(ESP_BT_MODE_BLE);

    // 2. Reduce CPU clock from 160MHz to 80MHz for minimal thermal drift & rail ripple
    setCpuFrequencyMhz(80);

    // 3. Configure hardware buttons with internal pullups
    pinMode(BOOT_BUTTON_PIN, INPUT_PULLUP);
    pinMode(TARE_PIN_D2, INPUT_PULLUP);
    pinMode(UNIT_BUTTON_PIN, INPUT_PULLUP);

    // If woken from sleep while BOOT button is held, wait for release
    unsigned long releaseStart = millis();
    while (digitalRead(BOOT_BUTTON_PIN) == LOW && (millis() - releaseStart < 1500)) {
      delay(10);
    }
    delay(50);

    // 4. Initialize Hardware I2C Bus on GPIO 6 & 7 at 400kHz
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN, 400000);
    delay(50);

    // 5. Initialize Display
    if (_display) {
      if (!_display->begin()) {
        Serial.println("[ERROR] Display initialization failed!");
      } else {
        Serial.printf("[INFO] Display initialized: %s\n", _display->getDisplayName());
      }
    }

    // 6. Initialize 7Semi NAU7802 24-bit ADC
    Serial.println("[INFO] Initializing NAU7802 ADC...");
    if (!_scale.begin(I2C_SDA_PIN, I2C_SCL_PIN, 400000)) {
      Serial.println("[ERROR] NAU7802 not found! Check wiring (3.3V, GND, SDA, SCL).");
      if (_display) {
        _display->showStatus("NAU7802 ERROR!", "Check Wiring");
      }
      while (1) delay(1000);
    }

    // Configure 3.0V internal LDO and 128x PGA
    _scale.setConfig(LDO_3_0V, PGA_128);

    // Set sample rate to 40 SPS (fast 25ms conversion time)
    _scale.setSampleRate(CRS_40);

    // Apply Nuvoton recommended setting: Set bits 4 & 5 in ADC_CTRL1 to eliminate chopper ripple
    Wire.beginTransmission(NAU7802_I2C_ADDR);
    Wire.write(0x15); // REG_ADC_CTRL1
    Wire.endTransmission(false);
    if (Wire.requestFrom((uint8_t)NAU7802_I2C_ADDR, (uint8_t)1) == 1) {
      uint8_t ctrl1 = Wire.read();
      ctrl1 |= 0x30;
      Wire.beginTransmission(NAU7802_I2C_ADDR);
      Wire.write(0x15);
      Wire.write(ctrl1);
      Wire.endTransmission();
      Serial.printf("[INFO] Chopper ripple filter set. ADC_CTRL1 = 0x%02X\n", ctrl1);
    }

    // Enable internal LDO
    enableInternalLDO();

    // Run internal Analog Front-End (AFE) offset calibration
    calibrateAFE();

    // Load calibration factors from flash
    loadSettings();

    // Zero the scale on startup
    performTare();

    resetActivityTimer();
    _lastActivityWeight = 0.0f;

    printHelp();
  }

  void update() {
    // 1. Process physical button inputs
    checkBootButton();
    checkTareButton();
    checkUnitButton();

    // 2. Auto-Shutdown on 2 minutes inactivity
    if (!_isHeld && (millis() - _lastActivityTime >= AUTO_SHUTDOWN_TIMEOUT_MS)) {
      Serial.println("[AUTO-OFF] 2 minutes of inactivity reached. Entering sleep mode...");
      enterSleepMode();
    }

    // 3. Process Serial Commands
    processSerial();

    // 4. Continuously acquire ADC samples into rolling ring buffer
    if (_scale.available()) {
      int32_t val = 0;
      if (_scale.readRaw(val, 5)) {
        _adcBuffer[_bufferIndex] = val;
        _bufferIndex = (_bufferIndex + 1) % FILTER_SIZE;
        if (_bufferIndex == 0) _bufferFull = true;

        // Trimmed-mean average from buffer
        _latestFilteredADC = getTrimmedMeanADC();
        int32_t netCounts = _latestFilteredADC - _tareOffset;
        float targetWeight = (_scaleFactor != 0) ? ((float)netCounts / _scaleFactor) : 0.0f;

        // Multi-tier fast-settling filter:
        // - Delta > 3.0g: Fast slew tracking (70% new sample per frame)
        // - Delta <= 3.0g: Smooth landing (20% new sample per frame)
        float delta = abs(targetWeight - _filteredWeight);
        if (delta > 3.0f) {
          _filteredWeight = (0.70f * targetWeight) + (0.30f * _filteredWeight);
          _isStable = false;
          _stableFrames = 0;
        } else {
          _filteredWeight = (0.20f * targetWeight) + (0.80f * _filteredWeight);
          if (_stableFrames < 20) _stableFrames++;
          if (_stableFrames >= 4) {
            _isStable = true;
            // Lock to stable integer 1g weight with 0.8g hysteresis
            if (abs(_filteredWeight - _lockedWeight) >= 0.8f) {
              _lockedWeight = (float)lround(_filteredWeight);
            }
          }
        }

        // Zero-tracking deadband: snap to 0g within +/-1.0g
        if (abs(_filteredWeight) < 1.0f) {
          _filteredWeight = 0.0f;
          _lockedWeight = 0.0f;
        }

        // Reset inactivity timer on significant weight change (>= 2.0g)
        if (abs(_filteredWeight - _lastActivityWeight) >= 2.0f) {
          _lastActivityWeight = _filteredWeight;
          resetActivityTimer();
        }
      }
    }

    // 5. Refresh display and serial output at 100ms intervals (10 Hz)
    if (millis() - _lastDisplayUpdate >= DISPLAY_INTERVAL_MS) {
      _lastDisplayUpdate = millis();

      int32_t netCounts = _latestFilteredADC - _tareOffset;
      bool saturated = (_latestFilteredADC <= -8388600 || _latestFilteredADC >= 8388600);
      float liveValue = _isStable ? _lockedWeight : _filteredWeight;
      float displayValue = _isHeld ? _heldWeight : liveValue;

      if (_display) {
        _display->showWeight(displayValue, _currentUnit, _isStable, _isHeld, netCounts, _latestFilteredADC, saturated);
      }

      // Stream to serial monitor if buffer space available
      if (Serial.availableForWrite() >= 64) {
        char serWeightBuf[20];
        formatWeightString(displayValue, _currentUnit, serWeightBuf, sizeof(serWeightBuf));
        Serial.printf("Weight: %s | Raw: %ld | Net: %ld | %s%s\n",
                      serWeightBuf, (long)_latestFilteredADC, (long)netCounts,
                      _isHeld ? "[HOLD] " : "",
                      _isStable ? "[STABLE]" : "[MOVING]");
      }
    }
  }

  void performTare() {
    Serial.println("[ACTION] Performing Tare (Zeroing)...");
    float startWeight = _filteredWeight;

    // Collect 14 samples and animate smooth fall down to 0
    const int TARE_SAMPLES = 14;
    int32_t samples[TARE_SAMPLES];
    int validSamples = 0;
    unsigned long start = millis();

    while (validSamples < TARE_SAMPLES && (millis() - start < 1500)) {
      int32_t val;
      if (_scale.readRaw(val, 80)) {
        samples[validSamples++] = val;

        float progress = (float)validSamples / (float)TARE_SAMPLES;
        float remain = 1.0f - progress;
        float animWeight = startWeight * remain * remain;

        if (_display) {
          _display->showTareProgress(animWeight, _currentUnit);
        }
      }
      delay(5);
    }

    if (validSamples >= 8) {
      // Sort samples
      for (int i = 1; i < validSamples; i++) {
        int32_t key = samples[i];
        int j = i - 1;
        while (j >= 0 && samples[j] > key) {
          samples[j + 1] = samples[j];
          j--;
        }
        samples[j + 1] = key;
      }

      // Discard top 20% and bottom 20% outliers
      int drop = validSamples / 5;
      long sum = 0;
      int used = 0;
      for (int i = drop; i < validSamples - drop; i++) {
        sum += samples[i];
        used++;
      }
      _tareOffset = (used > 0) ? (sum / used) : samples[validSamples / 2];

      _scale.setTareOffset(_tareOffset);

      // Flush filter buffer with new offset
      for (int i = 0; i < FILTER_SIZE; i++) {
        _adcBuffer[i] = _tareOffset;
      }
      _bufferIndex = 0;
      _bufferFull = true;
      _latestFilteredADC = _tareOffset;
      _filteredWeight = 0.0f;
      _lockedWeight = 0.0f;
      _isStable = true;
      _stableFrames = 15;

      saveSettings();
      Serial.printf("[SUCCESS] Tare locked. Offset = %ld (samples: %d)\n", (long)_tareOffset, validSamples);

      if (_display) {
        _display->showTareProgress(0.0f, _currentUnit);
      }
    } else {
      Serial.println("[ERROR] Tare failed! ADC timeout.");
    }
  }

  void calibrateScaleWithWeight(float knownWeight) {
    Serial.printf("[ACTION] Calibrating with reference weight: %.2f g\n", knownWeight);

    char refBuf[22];
    snprintf(refBuf, sizeof(refBuf), "Ref: %.1fg", knownWeight);
    if (_display) {
      _display->showStatus("CALIBRATING...", refBuf);
    }

    const int CAL_SAMPLES = 20;
    int32_t samples[CAL_SAMPLES];
    int validSamples = 0;
    unsigned long start = millis();
    while (validSamples < CAL_SAMPLES && (millis() - start < 3000)) {
      int32_t val;
      if (_scale.readRaw(val, 120)) {
        samples[validSamples++] = val;
      }
      delay(10);
    }

    if (validSamples >= 10) {
      // Sort samples
      for (int i = 1; i < validSamples; i++) {
        int32_t key = samples[i];
        int j = i - 1;
        while (j >= 0 && samples[j] > key) {
          samples[j + 1] = samples[j];
          j--;
        }
        samples[j + 1] = key;
      }

      int drop = validSamples / 5;
      long sum = 0;
      int used = 0;
      for (int i = drop; i < validSamples - drop; i++) {
        sum += samples[i];
        used++;
      }
      int32_t avgVal = (used > 0) ? (sum / used) : samples[validSamples / 2];

      int32_t net = avgVal - _tareOffset;
      _scaleFactor = (float)net / knownWeight;
      _isCalibrated = true;
      _scale.setScaleFactor(_scaleFactor);

      // Flush buffer with new calibrated reference
      for (int i = 0; i < FILTER_SIZE; i++) {
        _adcBuffer[i] = avgVal;
      }
      _bufferIndex = 0;
      _bufferFull = true;
      _latestFilteredADC = avgVal;
      _filteredWeight = knownWeight;
      _lockedWeight = knownWeight;
      _isStable = true;
      _stableFrames = 15;

      saveSettings();
      Serial.printf("[SUCCESS] Calibration complete! Factor: %.4f counts/g (Net: %ld)\n", _scaleFactor, (long)net);

      char factorBuf[22];
      snprintf(factorBuf, sizeof(factorBuf), "F: %.1f c/g", _scaleFactor);
      if (_display) {
        _display->showStatus("CAL SUCCESS!", factorBuf, 1200);
      }
    } else {
      Serial.println("[ERROR] Calibration failed. Make sure weight is steady.");
      if (_display) {
        _display->showStatus("CAL FAILED!", "Check weight", 1200);
      }
    }
  }

  void previewScaleFactor(float knownWeight) {
    Serial.printf("\n[PREVIEW] Calculating calibration preview for reference weight: %.2f g...\n", knownWeight);

    char refBuf[22];
    snprintf(refBuf, sizeof(refBuf), "Ref: %.1fg", knownWeight);
    if (_display) {
      _display->showStatus("CAL PREVIEW...", refBuf);
    }

    const int CAL_SAMPLES = 20;
    int32_t samples[CAL_SAMPLES];
    int validSamples = 0;
    unsigned long start = millis();
    while (validSamples < CAL_SAMPLES && (millis() - start < 3000)) {
      int32_t val;
      if (_scale.readRaw(val, 120)) {
        samples[validSamples++] = val;
      }
      delay(10);
    }

    if (validSamples >= 10) {
      // Sort samples
      for (int i = 1; i < validSamples; i++) {
        int32_t key = samples[i];
        int j = i - 1;
        while (j >= 0 && samples[j] > key) {
          samples[j + 1] = samples[j];
          j--;
        }
        samples[j + 1] = key;
      }

      int drop = validSamples / 5;
      long sum = 0;
      int used = 0;
      for (int i = drop; i < validSamples - drop; i++) {
        sum += samples[i];
        used++;
      }
      int32_t avgVal = (used > 0) ? (sum / used) : samples[validSamples / 2];

      int32_t net = avgVal - _tareOffset;
      float previewFactor = (float)net / knownWeight;

      Serial.println("--------------------------------------------------");
      Serial.printf("  [PREVIEW] Samples Count:      %d\n", validSamples);
      Serial.printf("  [PREVIEW] Trimmed Avg ADC:    %ld\n", (long)avgVal);
      Serial.printf("  [PREVIEW] Active Tare Offset: %ld\n", (long)_tareOffset);
      Serial.printf("  [PREVIEW] Net Counts:         %ld\n", (long)net);
      Serial.printf("  [PREVIEW] Reference Weight:   %.2f g\n", knownWeight);
      Serial.printf("  [PREVIEW] Preview Factor:     %.4f counts/g\n", previewFactor);
      Serial.printf("  [PREVIEW] Current Active:     %.4f counts/g\n", _scaleFactor);

      if (_scaleFactor > 0.0001f) {
        float diff = previewFactor - _scaleFactor;
        float pct = (diff / _scaleFactor) * 100.0f;
        Serial.printf("  [PREVIEW] Difference:         %+0.4f (%+0.2f%%)\n", diff, pct);
      }
      Serial.println("  [PREVIEW] (Notice: Settings unchanged. Use 'c <weight>' to commit.)");
      Serial.println("--------------------------------------------------");

      char factorBuf[22];
      snprintf(factorBuf, sizeof(factorBuf), "P: %.1f c/g", previewFactor);
      if (_display) {
        _display->showStatus("PREVIEW DONE", factorBuf, 1500);
      }
    } else {
      Serial.println("[ERROR] Calibration preview failed. ADC timeout or unstable readings.");
      if (_display) {
        _display->showStatus("PREVIEW FAILED!", "Check weight", 1200);
      }
    }
  }

  void cycleUnit() {
    _currentUnit = (WeightUnit)((_currentUnit + 1) % 4);
    const char* uSym = getUnitSymbol(_currentUnit);
    const char* uName = getUnitName(_currentUnit);

    Serial.printf("[UNIT] Unit changed to: %s (%s)\n", uSym, uName);
    saveSettings();

    // Instant redraw with new unit
    if (_display) {
      float val = _isHeld ? _heldWeight : (_isStable ? _lockedWeight : _filteredWeight);
      int32_t netCounts = _latestFilteredADC - _tareOffset;
      _display->showWeight(val, _currentUnit, _isStable, _isHeld, netCounts, _latestFilteredADC, false);
    }
  }

  void enterSleepMode() {
    Serial.println("[POWER] Entering low power sleep mode...");
    if (_display) {
      _display->showPowerOff();
      _display->powerDown();
    }

    // Power down NAU7802 internal LDO & ADC
    Wire.beginTransmission(NAU7802_I2C_ADDR);
    Wire.write(0x00); // REG_PU_CTRL
    Wire.write(0x00); // Shut down digital core, LDO, and PGA
    Wire.endTransmission();

    // Configure BOOT button as wake-up source
    pinMode(BOOT_BUTTON_PIN, INPUT_PULLUP);
    gpio_wakeup_enable((gpio_num_t)BOOT_BUTTON_PIN, GPIO_INTR_LOW_LEVEL);
    esp_sleep_enable_gpio_wakeup();

    Serial.flush();
    delay(100);

    // Enter light sleep (~0.13mA)
    esp_light_sleep_start();

    // On wake: wait for button release
    unsigned long wakeWaitStart = millis();
    while (digitalRead(BOOT_BUTTON_PIN) == LOW && (millis() - wakeWaitStart < 1500)) {
      delay(10);
    }
    delay(50);

    // Restart cleanly
    esp_restart();
  }

  void resetActivityTimer() {
    _lastActivityTime = millis();
  }

  void loadSettings() {
    _prefs.begin(PREFS_NAMESPACE, true);
    _scaleFactor = _prefs.getFloat("scale_factor", DEFAULT_SCALE_FACTOR);
    _tareOffset = _prefs.getInt("tare_offset", DEFAULT_TARE_OFFSET);
    _isCalibrated = _prefs.getBool("calibrated", true);
    _currentUnit = (WeightUnit)_prefs.getUChar("unit", (uint8_t)UNIT_G);
    _prefs.end();

    if (_scaleFactor <= 0.1f) _scaleFactor = DEFAULT_SCALE_FACTOR;
    if (_currentUnit > UNIT_LB) _currentUnit = UNIT_G;

    _scale.setScaleFactor(_scaleFactor);
    if (_tareOffset != 0) {
      _scale.setTareOffset(_tareOffset);
    }

    Serial.printf("[CONFIG] Loaded Settings -> Factor: %.4f | Offset: %ld | Unit: %s | Calibrated: %s\n",
                  _scaleFactor, (long)_tareOffset, getUnitSymbol(_currentUnit), _isCalibrated ? "YES" : "NO");
  }

  void saveSettings() {
    _prefs.begin(PREFS_NAMESPACE, false);
    _prefs.putFloat("scale_factor", _scaleFactor);
    _prefs.putInt("tare_offset", _tareOffset);
    _prefs.putBool("calibrated", _isCalibrated);
    _prefs.putUChar("unit", (uint8_t)_currentUnit);
    _prefs.end();
    Serial.println("[CONFIG] Saved settings to Non-Volatile Flash.");
  }

  int32_t getTrimmedMeanADC() {
    int count = _bufferFull ? FILTER_SIZE : _bufferIndex;
    if (count <= 0) return 0;
    if (count < 4) {
      long sum = 0;
      for (int i = 0; i < count; i++) sum += _adcBuffer[i];
      return sum / count;
    }

    int32_t sorted[FILTER_SIZE];
    for (int i = 0; i < count; i++) sorted[i] = _adcBuffer[i];

    // Insertion sort
    for (int i = 1; i < count; i++) {
      int32_t key = sorted[i];
      int j = i - 1;
      while (j >= 0 && sorted[j] > key) {
        sorted[j + 1] = sorted[j];
        j--;
      }
      sorted[j + 1] = key;
    }

    // Drop 1 lowest and 1 highest (averages middle 6 of 8)
    int drop = (count >= 6) ? 1 : 0;
    long sum = 0;
    int used = 0;
    for (int i = drop; i < count - drop; i++) {
      sum += sorted[i];
      used++;
    }
    return (used > 0) ? (sum / used) : sorted[count / 2];
  }

  void printHelp() {
    Serial.println();
    Serial.println("--- SERIAL COMMANDS ---");
    Serial.println("  t              : Tare (zero) scale");
    Serial.println("  u              : Cycle unit (g -> kg -> oz -> lb)");
    Serial.println("  hold           : Toggle weight HOLD freeze");
    Serial.println("  off / sleep    : Enter ultra-low power sleep");
    Serial.println("  c <weight>     : Calibrate with known weight (e.g., 'c 608')");
    Serial.println("  cp <weight>    : Calibration preview (calculate scale factor without saving)");
    Serial.println("  offset <val>   : Manually set tare offset (e.g., 'offset 75491')");
    Serial.println("  factor <val>   : Manually set scale factor (e.g., 'factor 93.44')");
    Serial.println("  r              : Read raw ADC and scale diagnostics");
    Serial.println("  h              : Print this help menu");
    Serial.println("-----------------------");
  }

private:
  void enableInternalLDO() {
    Wire.beginTransmission(NAU7802_I2C_ADDR);
    Wire.write(0x00); // REG_PU_CTRL
    if (Wire.endTransmission(false) == 0 && Wire.requestFrom((uint8_t)NAU7802_I2C_ADDR, (uint8_t)1) == 1) {
      uint8_t pu = Wire.read();
      pu |= (1 << 7); // AVDDS = 1 (Enable internal LDO)
      pu |= (1 << 4); // CS = 1 (Cycle start)
      Wire.beginTransmission(NAU7802_I2C_ADDR);
      Wire.write(0x00);
      Wire.write(pu);
      Wire.endTransmission();
      Serial.printf("[INFO] Internal LDO enabled. PU_CTRL = 0x%02X\n", pu);
    }
  }

  void calibrateAFE() {
    Wire.beginTransmission(NAU7802_I2C_ADDR);
    Wire.write(0x02); // REG_CTRL2
    if (Wire.endTransmission(false) == 0 && Wire.requestFrom((uint8_t)NAU7802_I2C_ADDR, (uint8_t)1) == 1) {
      uint8_t c2 = Wire.read();
      c2 &= 0xFC; // CALMOD = 00 (Offset calibration)
      c2 |= (1 << 2); // CALS = 1 (Start calibration)
      Wire.beginTransmission(NAU7802_I2C_ADDR);
      Wire.write(0x02);
      Wire.write(c2);
      Wire.endTransmission();

      unsigned long start = millis();
      while (millis() - start < 400) {
        Wire.beginTransmission(NAU7802_I2C_ADDR);
        Wire.write(0x02);
        if (Wire.endTransmission(false) == 0 && Wire.requestFrom((uint8_t)NAU7802_I2C_ADDR, (uint8_t)1) == 1) {
          if ((Wire.read() & (1 << 2)) == 0) break; // Finished
        }
        delay(10);
      }
      Serial.println("[INFO] AFE internal offset calibration complete.");
    }
  }

  void checkBootButton() {
    if (digitalRead(BOOT_BUTTON_PIN) == LOW) {
      resetActivityTimer();
      if (_bootPressStartTime == 0) {
        _bootPressStartTime = millis();
        _bootShutdownTriggered = false;
      } else {
        unsigned long holdMs = millis() - _bootPressStartTime;
        if (_display && holdMs >= 700 && holdMs < BOOT_SLEEP_HOLD_MS) {
          _display->showShutdownCountdown((uint16_t)holdMs, (uint16_t)BOOT_SLEEP_HOLD_MS);
        } else if (holdMs >= BOOT_SLEEP_HOLD_MS && !_bootShutdownTriggered) {
          _bootShutdownTriggered = true;
          enterSleepMode();
        }
      }
    } else {
      if (_bootPressStartTime > 0) {
        unsigned long pressDuration = millis() - _bootPressStartTime;
        _bootPressStartTime = 0;
        if (!_bootShutdownTriggered && pressDuration >= 80 && pressDuration < BOOT_SLEEP_HOLD_MS) {
          _isHeld = !_isHeld;
          if (_isHeld) {
            _heldWeight = _isStable ? _lockedWeight : _filteredWeight;
            Serial.printf("[HOLD] Weight held at: %.1f g\n", _heldWeight);
          } else {
            Serial.println("[HOLD] Weight unheld (live stream resumed).");
          }
        }
        _bootShutdownTriggered = false;
      }
    }
  }

  void checkTareButton() {
    if (digitalRead(TARE_PIN_D2) == LOW) {
      if (!_tareButtonActive && (millis() - _lastTareTriggerTime > 300)) {
        _tareButtonActive = true;
        _lastTareTriggerTime = millis();
        resetActivityTimer();
        if (_isHeld) _isHeld = false;
        Serial.println("[BUTTON] Tare requested via D2 pin.");
        performTare();
      }
    } else {
      _tareButtonActive = false;
    }
  }

  void checkUnitButton() {
    if (digitalRead(UNIT_BUTTON_PIN) == LOW) {
      if (!_unitButtonActive && (millis() - _lastUnitTriggerTime > 250)) {
        _unitButtonActive = true;
        _lastUnitTriggerTime = millis();
        resetActivityTimer();
        cycleUnit();
      }
    } else {
      _unitButtonActive = false;
    }
  }

  void processSerial() {
    if (Serial.available()) {
      String cmd = Serial.readStringUntil('\n');
      cmd.trim();
      resetActivityTimer();

      if (cmd.equalsIgnoreCase("t") || cmd.equalsIgnoreCase("tare")) {
        performTare();
      } else if (cmd.equalsIgnoreCase("u") || cmd.equalsIgnoreCase("unit")) {
        cycleUnit();
      } else if (cmd.equalsIgnoreCase("hold")) {
        _isHeld = !_isHeld;
        if (_isHeld) {
          _heldWeight = _isStable ? _lockedWeight : _filteredWeight;
          Serial.printf("[HOLD] Weight held at: %.1f g\n", _heldWeight);
        } else {
          Serial.println("[HOLD] Weight released.");
        }
      } else if (cmd.equalsIgnoreCase("off") || cmd.equalsIgnoreCase("sleep")) {
        enterSleepMode();
      } else if (cmd.startsWith("cp ") || cmd.startsWith("preview ") || cmd.startsWith("calpreview ")) {
        int spaceIdx = cmd.indexOf(' ');
        float weight = cmd.substring(spaceIdx + 1).toFloat();
        if (weight > 0) {
          previewScaleFactor(weight);
        } else {
          Serial.println("[ERROR] Enter valid weight, e.g.: cp 608");
        }
      } else if (cmd.startsWith("c ") || cmd.startsWith("cal ")) {
        int spaceIdx = cmd.indexOf(' ');
        float weight = cmd.substring(spaceIdx + 1).toFloat();
        if (weight > 0) {
          calibrateScaleWithWeight(weight);
        } else {
          Serial.println("[ERROR] Enter valid weight, e.g.: c 608");
        }
      } else if (cmd.startsWith("offset ")) {
        int32_t off = cmd.substring(7).toInt();
        _tareOffset = off;
        saveSettings();
        Serial.printf("[INFO] Manual TareOffset set to: %ld\n", (long)_tareOffset);
      } else if (cmd.startsWith("factor ")) {
        float f = cmd.substring(7).toFloat();
        if (f > 0) {
          _scaleFactor = f;
          saveSettings();
          Serial.printf("[INFO] Manual ScaleFactor set to: %.4f\n", _scaleFactor);
        }
      } else if (cmd.equalsIgnoreCase("r") || cmd.equalsIgnoreCase("raw")) {
        int32_t rawVal = getTrimmedMeanADC();
        Serial.printf("[RAW] FilteredADC: %ld | TareOffset: %ld | Net: %ld | ScaleFactor: %.4f\n",
                      (long)rawVal, (long)_tareOffset, (long)(rawVal - _tareOffset), _scaleFactor);
      } else if (cmd.equalsIgnoreCase("h") || cmd.equalsIgnoreCase("help")) {
        printHelp();
      }
    }
  }
};
