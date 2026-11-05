#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <7Semi_NAU7802.h>
#include <Preferences.h>
#include <cmath>
#include "esp_bt.h"
#include "esp_sleep.h"
#include "esp_wifi.h"

#include "config.h"
#include "scale_types.h"
#include "scale_display.h"

/**
 * ButtonDebouncer: Finite-state mechanical contact debouncer.
 * Rejects contact chatter/bounce on press and release transitions,
 * tracking clean debounced edges and continuous hold durations.
 */
struct ButtonDebouncer {
  uint8_t pin;
  bool isPressed;                    // Debounced steady state: true = pressed (LOW)
  bool lastRawState;                 // Last sampled raw reading (LOW = true)
  unsigned long lastStateChangeTime; // Timestamp when raw reading changed
  unsigned long pressStartTime;      // Timestamp when debounced press began
  bool longPressHandled;             // True if a long-press action fired during this press

  void init(uint8_t p) {
    pin = p;
    isPressed = false;
    lastRawState = false;
    lastStateChangeTime = 0;
    pressStartTime = 0;
    longPressHandled = false;
  }

  // Returns:
  //   1: Debounced Press Event (transitioned from released to pressed)
  //  -1: Debounced Release Event (transitioned from pressed to released)
  //   0: Steady state (no transition)
  int update(unsigned long now) {
    bool raw = (digitalRead(pin) == LOW);
    if (raw != lastRawState) {
      lastRawState = raw;
      lastStateChangeTime = now;
    }

    if ((now - lastStateChangeTime) >= BUTTON_DEBOUNCE_MS) {
      if (isPressed != raw) {
        isPressed = raw;
        if (isPressed) {
          pressStartTime = now;
          longPressHandled = false;
          return 1;
        } else {
          return -1;
        }
      }
    }
    return 0;
  }

  unsigned long holdDuration(unsigned long now) const {
    if (!isPressed || pressStartTime == 0) return 0;
    return (now >= pressStartTime) ? (now - pressStartTime) : 0;
  }
};

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
  uint16_t _tareAnimDurationMs;

  // Operating Mode & PCS (Piece Counting) State
  ScaleOperatingMode _operatingMode;
  float _pieceWeight;              // Average Piece Weight (APW) in grams
  uint16_t _sampleCount;           // Current sample count (continuously variable 1..9999)
  bool _inSampleWizard;            // True when 4-button arrow interface is modal
  unsigned long _wizardStartTime;  // Inactivity timer for sample wizard
  unsigned long _incLastRepeatTime;// Last auto-repeat step timestamp for increment
  unsigned long _decLastRepeatTime;// Last auto-repeat step timestamp for decrement

  // Digital Filtering & Rolling Buffer (operating directly in native ADC counts)
  int32_t _adcBuffer[FILTER_SIZE];
  int _bufferIndex;
  bool _bufferFull;
  int32_t _rawADC;              // Literal untouched 24-bit raw ADC reading direct from hardware
  int32_t _latestFilteredADC;   // 16-sample trimmed-mean ADC from rolling buffer
  float _filteredADC;           // EMA-filtered ADC count (floating point for smooth tracking)
  float _lockedWeight;          // Stable locked display weight (in grams)
  bool _isStable;
  int _stableFrames;

  // Metrology & Stability State (Native ADC Counts)
  float _stabilityBuffer[STABILITY_WINDOW_SIZE];
  int _stabilityIndex;
  bool _stabilityFilled;
  float _autoZeroCorrection;    // In native ADC counts

  // Hold State
  bool _isHeld;
  float _heldWeight;

  // Timers
  unsigned long _lastActivityTime;
  float _lastActivityWeight;
  unsigned long _lastDisplayUpdate;

  // Button Debouncers and Management
  ButtonDebouncer _btnHold;
  ButtonDebouncer _btnTare;
  ButtonDebouncer _btnUnit;
  ButtonDebouncer _btnPcs;
  unsigned long _lastCountdownUpdate;
  bool _isShutdownCountdownActive;

  // Power Bank Keep-Alive (IP5306)
  unsigned long _lastPowerPulseTime;
  bool _isPowerPulsing;

  // Non-blocking serial command buffer
  char _serialBuf[48];
  uint8_t _serialIndex;

public:
  ScaleCore(ScaleDisplay* display)
    : _display(display),
      _scale(Wire),
      _scaleFactor(DEFAULT_SCALE_FACTOR),
      _tareOffset(DEFAULT_TARE_OFFSET),
      _isCalibrated(true),
      _currentUnit(UNIT_G),
      _tareAnimDurationMs(DEFAULT_TARE_ANIM_MS),
      _operatingMode(MODE_WEIGHING),
      _pieceWeight(0.0f),
      _sampleCount(DEFAULT_SAMPLE_COUNT),
      _inSampleWizard(false),
      _wizardStartTime(0),
      _incLastRepeatTime(0),
      _decLastRepeatTime(0),
      _bufferIndex(0),
      _bufferFull(false),
      _rawADC(0),
      _latestFilteredADC(0),
      _filteredADC(0.0f),
      _lockedWeight(0.0f),
      _isStable(false),
      _stableFrames(0),
      _stabilityIndex(0),
      _stabilityFilled(false),
      _autoZeroCorrection(0.0f),
      _isHeld(false),
      _heldWeight(0.0f),
      _lastActivityTime(0),
      _lastActivityWeight(0.0f),
      _lastDisplayUpdate(0),
      _lastCountdownUpdate(0),
      _isShutdownCountdownActive(false),
      _lastPowerPulseTime(0),
      _isPowerPulsing(false),
      _serialIndex(0) {
    for (int i = 0; i < FILTER_SIZE; i++) {
      _adcBuffer[i] = 0;
    }
    for (int i = 0; i < STABILITY_WINDOW_SIZE; i++) {
      _stabilityBuffer[i] = 0.0f;
    }
    _serialBuf[0] = '\0';
  }

  void begin() {
    // 1. Disable Radios (Wi-Fi and Bluetooth) to eliminate 2.4 GHz RF emissions & power ripple
    esp_wifi_stop();
    btStop();
    esp_bt_controller_disable();
    esp_bt_controller_deinit();
    esp_bt_controller_mem_release(ESP_BT_MODE_BLE);

    // 2. Reduce CPU clock from 160MHz to 80MHz for minimal thermal drift & rail ripple
    setCpuFrequencyMhz(80);

    // 3. Configure hardware buttons with internal pullups
    pinMode(HOLD_BUTTON_PIN, INPUT_PULLUP);
    pinMode(TARE_BUTTON_PIN, INPUT_PULLUP);
    pinMode(UNIT_BUTTON_PIN, INPUT_PULLUP);
    pinMode(PCS_BUTTON_PIN, INPUT_PULLUP);

    _btnHold.init(HOLD_BUTTON_PIN);
    _btnTare.init(TARE_BUTTON_PIN);
    _btnUnit.init(UNIT_BUTTON_PIN);
    _btnPcs.init(PCS_BUTTON_PIN);

#ifdef POWER_BANK_PULSE_PIN
    // 3b. Configure Power Bank keep-alive pulse pin (default high-Z input)
    pinMode(POWER_BANK_PULSE_PIN, INPUT);
    _lastPowerPulseTime = millis();
    _isPowerPulsing = false;
    Serial.printf("[INFO] Power Bank keep-alive active on GPIO %d (pulsing every %lu ms)\n",
                  POWER_BANK_PULSE_PIN, (unsigned long)POWER_BANK_PULSE_INTERVAL_MS);
#endif

    // If woken from sleep while any button is held, wait for release
    unsigned long releaseStart = millis();
    while ((digitalRead(HOLD_BUTTON_PIN) == LOW || digitalRead(TARE_BUTTON_PIN) == LOW ||
            digitalRead(UNIT_BUTTON_PIN) == LOW || digitalRead(PCS_BUTTON_PIN) == LOW) && (millis() - releaseStart < 1500)) {
      delay(10);
    }
    delay(50);

    // Re-initialize button debouncers clean after wake release
    _btnHold.init(HOLD_BUTTON_PIN);
    _btnTare.init(TARE_BUTTON_PIN);
    _btnUnit.init(UNIT_BUTTON_PIN);
    _btnPcs.init(PCS_BUTTON_PIN);

    // 4. Initialize Hardware I2C Bus at 400kHz
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

    // 6. Initialize 7Semi NAU7802 24-bit ADC with retry
    Serial.println("[INFO] Initializing NAU7802 ADC...");
    int retries = 0;
    while (!_scale.begin(I2C_SDA_PIN, I2C_SCL_PIN, 400000)) {
      retries++;
      Serial.printf("[ERROR] NAU7802 not found! Attempt %d/3. Check wiring (3.3V, GND, SDA, SCL).\n", retries);
      if (_display) {
        _display->showStatus("NAU7802 RETRY", "Check Wiring", 1000);
      }
      if (retries >= 3) {
        Serial.println("[FATAL] NAU7802 communication failed. Rebooting in 3 seconds...");
        if (_display) {
          _display->showStatus("NAU7802 ERROR!", "Rebooting in 3s", 3000);
        }
        esp_restart();
      }
      delay(500);
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

    // Enable internal LDO and allow rail to settle
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
    checkButtons();

    // 1b. Power bank keep-alive pulse (IP5306)
    updatePowerBankPulse();

    // 2. Auto-Shutdown on inactivity (disabled if AUTO_SHUTDOWN_TIMEOUT_MS <= 0)
    if (AUTO_SHUTDOWN_TIMEOUT_MS > 0 && !_isHeld && (millis() - _lastActivityTime >= (unsigned long)AUTO_SHUTDOWN_TIMEOUT_MS)) {
      Serial.println("[AUTO-OFF] Inactivity timeout reached. Entering sleep mode...");
      enterSleepMode();
    }

    // 3. Process Serial Commands (non-blocking)
    processSerial();

    // 4. Continuously acquire ADC samples into rolling ring buffer
    if (_scale.available()) {
      int32_t val = 0;
      if (_scale.readRaw(val, 5)) {
        _rawADC = val; // Untouched 24-bit raw hardware reading
        _adcBuffer[_bufferIndex] = val;
        _bufferIndex = (_bufferIndex + 1) % FILTER_SIZE;
        if (_bufferIndex == 0) _bufferFull = true;

        // 1. Trimmed-mean average from buffer (removes impulse glitches/spikes)
        _latestFilteredADC = getTrimmedMeanADC();

        // 2. Multi-tier adaptive EMA filter operating in native ADC counts:
        // - Delta > FILTER_SLEW_COUNTS (280 cnts): Fast slew tracking (60% new sample per frame)
        // - FILTER_SETTLE_COUNTS < Delta <= FILTER_SLEW_COUNTS (140-280 cnts): Fine settling (20% new sample per frame)
        // - Delta <= FILTER_SETTLE_COUNTS (<= 140 cnts): Quiescent noise suppression (8% new sample per frame)
        float deltaADC = fabsf((float)_latestFilteredADC - _filteredADC);
        if (deltaADC > (float)FILTER_SLEW_COUNTS) {
          _filteredADC = (0.60f * (float)_latestFilteredADC) + (0.40f * _filteredADC);
          _isStable = false;
          _stableFrames = 0;
        } else if (deltaADC > (float)FILTER_SETTLE_COUNTS) {
          _filteredADC = (0.20f * (float)_latestFilteredADC) + (0.80f * _filteredADC);
        } else {
          _filteredADC = (0.08f * (float)_latestFilteredADC) + (0.92f * _filteredADC);
        }

        // 3. Rolling Window Peak-to-Peak Stability Evaluation in native ADC counts
        updateStability(_filteredADC);

        // 4. Rate-Limited Auto-Zero Tracking in native ADC counts
        updateAutoZero();

        // 5. Net counts and Zero Deadband with Hysteresis in native ADC counts
        float netCounts = _filteredADC - (float)_tareOffset - _autoZeroCorrection;
        float absNet = fabsf(netCounts);

        bool atZero = false;
        if (_lockedWeight == 0.0f) {
          // While currently at zero, require >= ZERO_DEADBAND_COUNTS (100 cnts) to break out
          if (absNet < (float)ZERO_DEADBAND_COUNTS) {
            atZero = true;
          }
        } else {
          // While non-zero, require < ZERO_RELEASE_COUNTS (65 cnts) to return to zero
          if (absNet < (float)ZERO_RELEASE_COUNTS) {
            atZero = true;
          }
        }

        // Converted weight in grams
        float rawWeight = (_scaleFactor != 0) ? (netCounts / _scaleFactor) : 0.0f;

        if (atZero) {
          _lockedWeight = 0.0f;
        } else {
          if (_isStable) {
            // When stable, lock firmly to integer weight to eliminate display jitter.
            // Only update locked weight if the settled weight genuinely drifts >= DISPLAY_LOCK_HYSTERESIS_G
            if (_lockedWeight == 0.0f || fabsf(rawWeight - _lockedWeight) >= DISPLAY_LOCK_HYSTERESIS_G) {
              _lockedWeight = (float)lround(rawWeight);
            }
          } else {
            // While moving / slewing, track live rounded weight smoothly
            _lockedWeight = (float)lround(rawWeight);
          }
        }

        // Reset inactivity timer on significant weight change (>= 2.0g)
        if (fabsf(rawWeight - _lastActivityWeight) >= 2.0f) {
          _lastActivityWeight = rawWeight;
          resetActivityTimer();
        }
      }
    }

    // 5. Refresh display and serial output at 100ms intervals (10 Hz)
    // If modal sample wizard is active, render wizard UI
    if (_inSampleWizard) {
      if (millis() - _lastDisplayUpdate >= DISPLAY_INTERVAL_MS) {
        _lastDisplayUpdate = millis();
        if (SAMPLE_SETUP_TIMEOUT_MS > 0 && (millis() - _wizardStartTime >= (unsigned long)SAMPLE_SETUP_TIMEOUT_MS)) {
          Serial.println("[WIZARD] Inactivity timeout. Exiting wizard...");
          exitSampleWizard(false);
          return;
        }
        int32_t netCounts = (int32_t)(_filteredADC - (float)_tareOffset - _autoZeroCorrection);
        float netGrams = (_scaleFactor != 0) ? ((float)netCounts / _scaleFactor) : 0.0f;
        if (_display) {
          _display->showSampleWizard(_sampleCount, netGrams, nullptr);
        }
      }
      return;
    }

    // Suppress regular weight update if shutdown countdown is actively rendering
    if (!_isShutdownCountdownActive && (millis() - _lastDisplayUpdate >= DISPLAY_INTERVAL_MS)) {
      _lastDisplayUpdate = millis();

      int32_t netCounts = (int32_t)(_filteredADC - (float)_tareOffset - _autoZeroCorrection);
      bool saturated = (_latestFilteredADC <= -ADC_SATURATION_LIMIT || _latestFilteredADC >= ADC_SATURATION_LIMIT);

      WeightUnit activeUnit;
      float displayValue;

      if (_operatingMode == MODE_PCS) {
        activeUnit = UNIT_PCS;
        float netGrams = (_scaleFactor != 0) ? ((float)netCounts / _scaleFactor) : 0.0f;
        float count = (_pieceWeight > 0.0001f) ? (netGrams / _pieceWeight) : 0.0f;
        float livePcs = (_lockedWeight == 0.0f) ? 0.0f : (float)lround(count);
        displayValue = _isHeld ? _heldWeight : livePcs;
      } else {
        activeUnit = _currentUnit;
        float liveValue = (_currentUnit == UNIT_G) ? _lockedWeight : (_isStable ? _lockedWeight : ((_scaleFactor != 0) ? ((float)netCounts / _scaleFactor) : 0.0f));
        displayValue = _isHeld ? _heldWeight : liveValue;
      }

      if (_display) {
        _display->showWeight(displayValue, activeUnit, _isStable, _isHeld, netCounts, _latestFilteredADC, saturated);
      }

      // Stream to serial monitor if buffer space available
      if (Serial.availableForWrite() >= 64) {
        char serWeightBuf[20];
        formatWeightString(displayValue, activeUnit, serWeightBuf, sizeof(serWeightBuf));
        if (_operatingMode == MODE_PCS) {
          float netGrams = (_scaleFactor != 0) ? ((float)netCounts / _scaleFactor) : 0.0f;
          Serial.printf("PCS: %s | Wt: %.2fg | APW: %.4fg | %s%s\n",
                        serWeightBuf, netGrams, _pieceWeight,
                        _isHeld ? "[HOLD] " : "",
                        _isStable ? "[STABLE]" : "[MOVING]");
        } else {
          Serial.printf("Weight: %s | Raw: %ld | Net: %ld | %s%s\n",
                        serWeightBuf, (long)_rawADC, (long)netCounts,
                        _isHeld ? "[HOLD] " : "",
                        _isStable ? "[STABLE]" : "[MOVING]");
        }
      }
    }
  }

  void toggleHold() {
    _isHeld = !_isHeld;
    int32_t netCounts = (int32_t)(_filteredADC - (float)_tareOffset - _autoZeroCorrection);
    WeightUnit activeUnit;
    float currentVal;

    if (_operatingMode == MODE_PCS) {
      activeUnit = UNIT_PCS;
      float netGrams = (_scaleFactor != 0) ? ((float)netCounts / _scaleFactor) : 0.0f;
      float count = (_pieceWeight > 0.0001f) ? (netGrams / _pieceWeight) : 0.0f;
      currentVal = (_lockedWeight == 0.0f) ? 0.0f : (float)lround(count);
    } else {
      activeUnit = _currentUnit;
      currentVal = (_currentUnit == UNIT_G) ? _lockedWeight : (_isStable ? _lockedWeight : ((_scaleFactor != 0) ? ((float)netCounts / _scaleFactor) : 0.0f));
    }

    if (_isHeld) {
      _heldWeight = currentVal;
      if (_operatingMode == MODE_PCS) {
        Serial.printf("[HOLD] Piece count held at: %ld pcs\n", lround(_heldWeight));
      } else {
        Serial.printf("[HOLD] Weight held at: %.3f %s\n", _heldWeight, getUnitSymbol(activeUnit));
      }
    } else {
      Serial.println("[HOLD] Weight released (live stream resumed).");
    }
    if (_display) {
      float val = _isHeld ? _heldWeight : currentVal;
      _display->showWeight(val, activeUnit, _isStable, _isHeld, netCounts, _latestFilteredADC, false);
    }
  }

  uint16_t getTareAnimDurationMs() const { return _tareAnimDurationMs; }
  void setTareAnimDurationMs(uint16_t durationMs) {
    if (durationMs > MAX_TARE_ANIM_MS) durationMs = MAX_TARE_ANIM_MS;
    _tareAnimDurationMs = durationMs;
    saveSettings();
    Serial.printf("[CONFIG] Tare animation duration set to %u ms (%s)\n",
                  _tareAnimDurationMs,
                  (_tareAnimDurationMs == 0) ? "Instant" :
                  (_tareAnimDurationMs <= 60) ? "Ultra Quick" :
                  (_tareAnimDurationMs <= 120) ? "Super Quick" :
                  (_tareAnimDurationMs <= 250) ? "Normal" : "Slow");
  }

  void performTare() {
    Serial.println("[ACTION] Performing Tare (Zeroing)...");
    if (_isHeld) _isHeld = false;

    // Start animating smoothly from current displayed weight down to 0
    float startWeight = _lockedWeight;
    if (fabsf(startWeight) < 0.001f) {
      float netCounts = _filteredADC - (float)_tareOffset - _autoZeroCorrection;
      startWeight = (_scaleFactor != 0) ? (netCounts / _scaleFactor) : 0.0f;
    }

    // High-precision ADC sampling buffer (metrology accuracy is completely decoupled from UI animation)
    int32_t samples[TARE_ACCURACY_SAMPLES];
    int validSamples = 0;
    unsigned long start = millis();
    unsigned long animStart = start;
    unsigned long lastFrameTime = 0;
    const unsigned long FRAME_INTERVAL_MS = 18; // ~55 FPS smooth slide
    bool animCompleted = (_tareAnimDurationMs == 0 || fabsf(startWeight) < 0.001f);

    if (animCompleted && _display) {
      _display->showTareProgress(0.0f, _currentUnit);
    }

    while (validSamples < TARE_ACCURACY_SAMPLES && (millis() - start < 1500)) {
      int32_t val;
      // Non-blocking 5ms raw read so animation timer runs smoothly
      if (_scale.readRaw(val, 5)) {
        samples[validSamples++] = val;
      }

      // Update slide animation based on elapsed wall-clock time (independent of ADC sample rate)
      if (!animCompleted && _display) {
        unsigned long now = millis();
        if (now - lastFrameTime >= FRAME_INTERVAL_MS) {
          lastFrameTime = now;
          unsigned long elapsed = now - animStart;
          if (elapsed >= _tareAnimDurationMs) {
            animCompleted = true;
            _display->showTareProgress(0.0f, _currentUnit);
          } else {
            float progress = (float)elapsed / (float)_tareAnimDurationMs;
            float remain = 1.0f - progress;
            float animWeight = startWeight * remain * remain;
            _display->showTareProgress(animWeight, _currentUnit);
          }
        }
      }
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

      // Discard top 20% and bottom 20% outliers for maximum metrological accuracy
      int drop = validSamples / 5;
      long sum = 0;
      int used = 0;
      for (int i = drop; i < validSamples - drop; i++) {
        sum += samples[i];
        used++;
      }
      _tareOffset = (used > 0) ? (sum / used) : samples[validSamples / 2];

      _scale.setTareOffset(_tareOffset);

      // Reset auto-zero tracking correction
      _autoZeroCorrection = 0.0f;

      // Flush filter buffer with new offset
      for (int i = 0; i < FILTER_SIZE; i++) {
        _adcBuffer[i] = _tareOffset;
      }
      _bufferIndex = 0;
      _bufferFull = true;
      _latestFilteredADC = _tareOffset;
      _filteredADC = (float)_tareOffset;
      _lockedWeight = 0.0f;
      _isStable = true;
      _stableFrames = STABILITY_CONFIRM_FRAMES;

      for (int i = 0; i < STABILITY_WINDOW_SIZE; i++) {
        _stabilityBuffer[i] = _filteredADC;
      }
      _stabilityIndex = 0;
      _stabilityFilled = true;

      // Notice: Operational tare is kept in RAM to protect flash sectors from wear
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

      // Reset auto-zero tracking correction
      _autoZeroCorrection = 0.0f;

      // Flush buffer with new calibrated reference
      for (int i = 0; i < FILTER_SIZE; i++) {
        _adcBuffer[i] = avgVal;
      }
      _bufferIndex = 0;
      _bufferFull = true;
      _latestFilteredADC = avgVal;
      _filteredADC = (float)avgVal;
      _lockedWeight = (float)lround(knownWeight);
      _isStable = true;
      _stableFrames = STABILITY_CONFIRM_FRAMES;

      for (int i = 0; i < STABILITY_WINDOW_SIZE; i++) {
        _stabilityBuffer[i] = _filteredADC;
      }
      _stabilityIndex = 0;
      _stabilityFilled = true;

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
    if (_inSampleWizard) return;
    if (_operatingMode == MODE_PCS) {
      _operatingMode = MODE_WEIGHING;
      Serial.printf("[UNIT] Exited PCS mode. Returned to weighing (%s)\n", getUnitSymbol(_currentUnit));
      saveSettings();
    } else {
      _currentUnit = (WeightUnit)((_currentUnit + 1) % 4);
      const char* uSym = getUnitSymbol(_currentUnit);
      const char* uName = getUnitName(_currentUnit);
      Serial.printf("[UNIT] Unit changed to: %s (%s)\n", uSym, uName);
      saveSettings();
    }

    // Instant redraw with new unit
    if (_display) {
      int32_t netCounts = (int32_t)(_filteredADC - (float)_tareOffset - _autoZeroCorrection);
      float currentVal = (_currentUnit == UNIT_G) ? _lockedWeight : (_isStable ? _lockedWeight : ((_scaleFactor != 0) ? ((float)netCounts / _scaleFactor) : 0.0f));
      float val = _isHeld ? _heldWeight : currentVal;
      _display->showWeight(val, _currentUnit, _isStable, _isHeld, netCounts, _latestFilteredADC, false);
    }
  }

  void togglePcsMode() {
    if (_inSampleWizard) return;
    if (_operatingMode == MODE_WEIGHING) {
      if (_pieceWeight <= 0.0001f) {
        Serial.println("[PCS] No APW calibrated yet. Entering Sample Wizard...");
        startSampleWizard();
      } else {
        _operatingMode = MODE_PCS;
        Serial.printf("[PCS] Activated Piece Counting. APW = %.4fg | SampleRef = %u pcs\n", _pieceWeight, _sampleCount);
        saveSettings();
        if (_display) {
          char apwBuf[22];
          snprintf(apwBuf, sizeof(apwBuf), "APW: %.3fg", _pieceWeight);
          _display->showStatus("PCS MODE", apwBuf, 800);
        }
      }
    } else {
      _operatingMode = MODE_WEIGHING;
      Serial.printf("[PCS] Returned to standard weighing mode (%s).\n", getUnitSymbol(_currentUnit));
      saveSettings();
      if (_display) {
        char unitBuf[22];
        snprintf(unitBuf, sizeof(unitBuf), "Unit: %s", getUnitSymbol(_currentUnit));
        _display->showStatus("WEIGH MODE", unitBuf, 800);
      }
    }
  }

  void updateWizardDisplay() {
    if (_display) {
      int32_t netCounts = (int32_t)(_filteredADC - (float)_tareOffset - _autoZeroCorrection);
      float netGrams = (_scaleFactor != 0) ? ((float)netCounts / _scaleFactor) : 0.0f;
      _display->showSampleWizard(_sampleCount, netGrams, nullptr);
    }
  }

  void startSampleWizard() {
    _inSampleWizard = true;
    _wizardStartTime = millis();
    if (_sampleCount == 0) _sampleCount = DEFAULT_SAMPLE_COUNT;

    // Suppress release actions for any buttons currently held down during transition
    if (_btnHold.isPressed) _btnHold.longPressHandled = true;
    if (_btnTare.isPressed) _btnTare.longPressHandled = true;
    if (_btnUnit.isPressed) _btnUnit.longPressHandled = true;
    if (_btnPcs.isPressed)  _btnPcs.longPressHandled  = true;

    Serial.printf("[WIZARD] Started Sample Wizard. Target: %u pcs (continuously variable 1..9999).\n", _sampleCount);
    Serial.printf("[WIZARD] 4-Button Controls: [ESC: GPIO %d] [-: GPIO %d] [+: GPIO %d] [OK: GPIO %d]\n",
                  NAV_BTN_CANCEL, NAV_BTN_DEC, NAV_BTN_INC, NAV_BTN_CONFIRM);

    updateWizardDisplay();
  }

  void exitSampleWizard(bool success) {
    _inSampleWizard = false;
    if (!success && _operatingMode == MODE_PCS && _pieceWeight <= 0.0001f) {
      _operatingMode = MODE_WEIGHING;
    }

    // Suppress release actions for any buttons currently held down during transition
    if (_btnHold.isPressed) _btnHold.longPressHandled = true;
    if (_btnTare.isPressed) _btnTare.longPressHandled = true;
    if (_btnUnit.isPressed) _btnUnit.longPressHandled = true;
    if (_btnPcs.isPressed)  _btnPcs.longPressHandled  = true;
  }

  bool calibratePcsWithSample(uint16_t count) {
    if (count == 0) count = DEFAULT_SAMPLE_COUNT;
    Serial.printf("[ACTION] Calibrating APW with %u reference pieces...\n", count);
    if (_display) {
      char refBuf[22];
      snprintf(refBuf, sizeof(refBuf), "Ref: %u pcs", count);
      _display->showStatus("SAMPLING...", refBuf);
    }

    const int CAL_SAMPLES = 20;
    int32_t samples[CAL_SAMPLES];
    int validSamples = 0;
    unsigned long start = millis();
    while (validSamples < CAL_SAMPLES && (millis() - start < 2000)) {
      int32_t val;
      if (_scale.readRaw(val, 80)) {
        samples[validSamples++] = val;
      }
      delay(10);
    }

    if (validSamples >= 10) {
      // Sort and calculate trimmed mean
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
      int32_t netCounts = avgVal - _tareOffset;
      float netGrams = (_scaleFactor != 0) ? ((float)netCounts / _scaleFactor) : 0.0f;

      if (netGrams < MIN_SAMPLE_TOTAL_WEIGHT_G) {
        Serial.printf("[ERROR] Sample total weight too small (%.2fg < %.2fg required)!\n",
                      netGrams, MIN_SAMPLE_TOTAL_WEIGHT_G);
        if (_display) {
          _display->showStatus("TOO LIGHT!", "Need > 0.20g", 1500);
        }
        return false;
      }

      float calculatedApw = netGrams / (float)count;
      if (calculatedApw < MIN_PIECE_WEIGHT_G) {
        Serial.printf("[ERROR] Piece weight too small (%.4fg < %.4fg required)!\n",
                      calculatedApw, MIN_PIECE_WEIGHT_G);
        if (_display) {
          _display->showStatus("APW TOO SMALL!", "Check weight", 1500);
        }
        return false;
      }

      _pieceWeight = calculatedApw;
      _sampleCount = count;
      _operatingMode = MODE_PCS;
      _inSampleWizard = false;
      saveSettings();

      Serial.printf("[SUCCESS] APW set to %.4f g/pc (Total weight: %.2fg for %u pcs)\n",
                    _pieceWeight, netGrams, _sampleCount);
      char apwBuf[22];
      snprintf(apwBuf, sizeof(apwBuf), "APW: %.3fg", _pieceWeight);
      if (_display) {
        _display->showStatus("APW SAVED!", apwBuf, 1200);
      }
      return true;
    } else {
      Serial.println("[ERROR] Sampling failed. ADC timeout or unstable readings.");
      if (_display) {
        _display->showStatus("SAMPLE FAILED!", "Check weight", 1200);
      }
      return false;
    }
  }

  void setManualAPW(float apw) {
    if (apw < MIN_PIECE_WEIGHT_G) {
      Serial.printf("[ERROR] APW must be >= %.4fg\n", MIN_PIECE_WEIGHT_G);
      return;
    }
    _pieceWeight = apw;
    _operatingMode = MODE_PCS;
    _inSampleWizard = false;
    saveSettings();
    Serial.printf("[SUCCESS] Manual APW set to: %.4f g/pc. PCS mode active.\n", _pieceWeight);
    char apwBuf[22];
    snprintf(apwBuf, sizeof(apwBuf), "APW: %.3fg", _pieceWeight);
    if (_display) {
      _display->showStatus("APW SET!", apwBuf, 1000);
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

    // Configure RTC pull-ups on RTC-capable GPIOs
    gpio_pullup_en((gpio_num_t)HOLD_BUTTON_PIN);
    gpio_pulldown_dis((gpio_num_t)HOLD_BUTTON_PIN);
    gpio_pullup_en((gpio_num_t)TARE_BUTTON_PIN);
    gpio_pulldown_dis((gpio_num_t)TARE_BUTTON_PIN);
    gpio_pullup_en((gpio_num_t)UNIT_BUTTON_PIN);
    gpio_pulldown_dis((gpio_num_t)UNIT_BUTTON_PIN);
    gpio_pullup_en((gpio_num_t)PCS_BUTTON_PIN);
    gpio_pulldown_dis((gpio_num_t)PCS_BUTTON_PIN);

    // Configure wake-up source on active buttons
    gpio_wakeup_enable((gpio_num_t)HOLD_BUTTON_PIN, GPIO_INTR_LOW_LEVEL);
    gpio_wakeup_enable((gpio_num_t)TARE_BUTTON_PIN, GPIO_INTR_LOW_LEVEL);
    gpio_wakeup_enable((gpio_num_t)UNIT_BUTTON_PIN, GPIO_INTR_LOW_LEVEL);
    gpio_wakeup_enable((gpio_num_t)PCS_BUTTON_PIN, GPIO_INTR_LOW_LEVEL);
    esp_sleep_enable_gpio_wakeup();

    Serial.flush();
    delay(100);

    // Enter light sleep (~0.13mA)
    esp_light_sleep_start();

    // On wake: wait for button release
    unsigned long wakeWaitStart = millis();
    while ((digitalRead(HOLD_BUTTON_PIN) == LOW || digitalRead(TARE_BUTTON_PIN) == LOW ||
            digitalRead(UNIT_BUTTON_PIN) == LOW || digitalRead(PCS_BUTTON_PIN) == LOW) && (millis() - wakeWaitStart < 1500)) {
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
    _pieceWeight = _prefs.getFloat("piece_wt", 0.0f);
    _sampleCount = _prefs.getUShort("sample_cnt", DEFAULT_SAMPLE_COUNT);
    uint8_t opMode = _prefs.getUChar("mode", (uint8_t)MODE_WEIGHING);
    _operatingMode = (opMode == (uint8_t)MODE_PCS && _pieceWeight > 0.0001f) ? MODE_PCS : MODE_WEIGHING;
    _tareAnimDurationMs = _prefs.getUShort("tare_anim_ms", DEFAULT_TARE_ANIM_MS);
    _prefs.end();

    if (_tareAnimDurationMs > MAX_TARE_ANIM_MS) _tareAnimDurationMs = MAX_TARE_ANIM_MS;
    if (_scaleFactor <= 0.1f) _scaleFactor = DEFAULT_SCALE_FACTOR;
    if (_currentUnit > UNIT_LB) _currentUnit = UNIT_G;

    _scale.setScaleFactor(_scaleFactor);
    if (_tareOffset != 0) {
      _scale.setTareOffset(_tareOffset);
    }

    Serial.printf("[CONFIG] Loaded Settings -> Factor: %.4f | Offset: %ld | Unit: %s | Mode: %s | APW: %.4fg (Ref: %u pcs) | TareAnim: %u ms | Calibrated: %s\n",
                  _scaleFactor, (long)_tareOffset, getUnitSymbol(_currentUnit),
                  (_operatingMode == MODE_PCS) ? "PCS" : "WEIGH", _pieceWeight, _sampleCount,
                  _tareAnimDurationMs, _isCalibrated ? "YES" : "NO");
  }

  void saveSettings() {
    _prefs.begin(PREFS_NAMESPACE, false);
    _prefs.putFloat("scale_factor", _scaleFactor);
    _prefs.putInt("tare_offset", _tareOffset);
    _prefs.putBool("calibrated", _isCalibrated);
    _prefs.putUChar("unit", (uint8_t)_currentUnit);
    _prefs.putFloat("piece_wt", _pieceWeight);
    _prefs.putUShort("sample_cnt", _sampleCount);
    _prefs.putUChar("mode", (uint8_t)_operatingMode);
    _prefs.putUShort("tare_anim_ms", _tareAnimDurationMs);
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

    // Discard outliers (drop 2 lowest & 2 highest for 16-sample buffer, else 1)
    int drop = (count >= 12) ? 2 : ((count >= 6) ? 1 : 0);
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
    Serial.println("  tarespeed [ms] : Get/set tare animation slide duration (e.g., 'tarespeed 100', 'tarespeed quick')");
    Serial.println("  u              : Cycle unit (g -> kg -> oz -> lb)");
    Serial.println("  pcs            : Toggle / switch to PCS piece counting mode");
    Serial.println("  sample <count> : Calibrate APW with current load as <count> pieces (e.g. 'sample 25')");
    Serial.println("  apw <weight>   : Manually set Average Piece Weight in grams (e.g. 'apw 0.254')");
    Serial.println("  apw            : Query current APW and piece counting status");
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
  void updateStability(float filteredADC) {
    _stabilityBuffer[_stabilityIndex] = filteredADC;
    _stabilityIndex = (_stabilityIndex + 1) % STABILITY_WINDOW_SIZE;
    if (_stabilityIndex == 0) _stabilityFilled = true;

    int count = _stabilityFilled ? STABILITY_WINDOW_SIZE : _stabilityIndex;
    if (count >= 6) {
      float minVal = _stabilityBuffer[0];
      float maxVal = _stabilityBuffer[0];
      for (int i = 1; i < count; i++) {
        if (_stabilityBuffer[i] < minVal) minVal = _stabilityBuffer[i];
        if (_stabilityBuffer[i] > maxVal) maxVal = _stabilityBuffer[i];
      }
      float spread = maxVal - minVal;

      if (spread <= (float)STABILITY_SPREAD_COUNTS) {
        if (_stableFrames < STABILITY_CONFIRM_FRAMES) _stableFrames++;
        if (_stableFrames >= STABILITY_CONFIRM_FRAMES) {
          _isStable = true;
        }
      } else {
        _stableFrames = 0;
        _isStable = false;
      }
    }
  }

  void updateAutoZero() {
    // Rate-limited Auto-Zero Tracking (OIML R76 §4.5.7) operating in native ADC counts
    // Operates only when scale is stable and net counts are within the auto-zero window
    float net = _filteredADC - (float)_tareOffset - _autoZeroCorrection;
    if (_isStable && fabsf(net) <= (float)AUTO_ZERO_BAND_COUNTS) {
      float step = net * 0.10f;
      float maxStep = (float)AUTO_ZERO_MAX_STEP_COUNTS;
      if (step > maxStep) step = maxStep;
      if (step < -maxStep) step = -maxStep;

      _autoZeroCorrection += step;
    }
  }

  void updatePowerBankPulse() {
#ifdef POWER_BANK_PULSE_PIN
    unsigned long now = millis();
    if (!_isPowerPulsing) {
      if (now - _lastPowerPulseTime >= POWER_BANK_PULSE_INTERVAL_MS) {
        _isPowerPulsing = true;
        _lastPowerPulseTime = now;
        // Start pulse: pull to ground
        digitalWrite(POWER_BANK_PULSE_PIN, LOW);
        pinMode(POWER_BANK_PULSE_PIN, OUTPUT);
      }
    } else {
      if (now - _lastPowerPulseTime >= POWER_BANK_PULSE_DURATION_MS) {
        _isPowerPulsing = false;
        // End pulse: release back to high-impedance
        pinMode(POWER_BANK_PULSE_PIN, INPUT);
      }
    }
#endif
  }

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
      // Datasheet Rev 2.6 §2.1: Allow internal LDO voltage and bandgap reference to stabilize
      delay(10);
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
      bool calComplete = false;
      bool calError = false;

      while (millis() - start < 400) {
        Wire.beginTransmission(NAU7802_I2C_ADDR);
        Wire.write(0x02);
        if (Wire.endTransmission(false) == 0 && Wire.requestFrom((uint8_t)NAU7802_I2C_ADDR, (uint8_t)1) == 1) {
          uint8_t ctrl2 = Wire.read();
          if ((ctrl2 & (1 << 2)) == 0) { // CALS == 0: complete
            calComplete = true;
            calError = (ctrl2 & (1 << 3)) != 0; // CAL_ERR bit 3
            break;
          }
        }
        delay(10);
      }

      if (!calComplete) {
        Serial.println("[WARN] AFE internal offset calibration timed out!");
      } else if (calError) {
        Serial.println("[WARN] AFE calibration error detected (CAL_ERR bit set)!");
      } else {
        Serial.println("[INFO] AFE internal offset calibration complete (success).");
      }
    }
  }

  ButtonDebouncer* getPowerOffDebouncer() {
    if (POWER_OFF_BUTTON_PIN == HOLD_BUTTON_PIN) return &_btnHold;
    if (POWER_OFF_BUTTON_PIN == TARE_BUTTON_PIN) return &_btnTare;
    if (POWER_OFF_BUTTON_PIN == UNIT_BUTTON_PIN) return &_btnUnit;
    if (POWER_OFF_BUTTON_PIN == PCS_BUTTON_PIN)  return &_btnPcs;
    return &_btnHold;
  }

  void checkNormalButtons(int evHold, int evTare, int evUnit, int evPcs, unsigned long now) {
    ButtonDebouncer* pwrBtn = getPowerOffDebouncer();

    // 1. Check long-press power off / deep sleep countdown while power button is held
    if (pwrBtn && pwrBtn->isPressed) {
      unsigned long holdMs = pwrBtn->holdDuration(now);
      if (holdMs >= 700 && holdMs < POWER_OFF_HOLD_MS) {
        _isShutdownCountdownActive = true;
        if (now - _lastCountdownUpdate >= 50) {
          _lastCountdownUpdate = now;
          if (_display) {
            _display->showShutdownCountdown((uint16_t)holdMs, (uint16_t)POWER_OFF_HOLD_MS);
          }
        }
      } else if (holdMs >= POWER_OFF_HOLD_MS && !pwrBtn->longPressHandled) {
        pwrBtn->longPressHandled = true;
        _isShutdownCountdownActive = false;
        enterSleepMode();
      }
    }

    // 2. HOLD Button (Pin 1)
    if (evHold == -1) {
      if (pwrBtn == &_btnHold) {
        _isShutdownCountdownActive = false;
      }
      if (!_btnHold.longPressHandled) {
        toggleHold();
      }
      _btnHold.longPressHandled = false;
    }

    // 3. TARE Button (Pin 0)
    if (evTare == -1) {
      if (pwrBtn == &_btnTare) {
        _isShutdownCountdownActive = false;
      }
      if (!_btnTare.longPressHandled) {
        Serial.println("[BUTTON] Tare requested via button click.");
        performTare();
      }
      _btnTare.longPressHandled = false;
    }

    // 4. UNIT Button (Pin 6)
    if (evUnit == -1) {
      if (pwrBtn == &_btnUnit) {
        _isShutdownCountdownActive = false;
      }
      if (!_btnUnit.longPressHandled) {
        Serial.println("[BUTTON] Unit cycle requested via button click.");
        cycleUnit();
      }
      _btnUnit.longPressHandled = false;
    }

    // 5. PCS Button (Pin 5): Long-press check & Click Release
    if (_btnPcs.isPressed) {
      unsigned long holdMs = _btnPcs.holdDuration(now);
      if (holdMs >= PCS_SETUP_HOLD_MS && !_btnPcs.longPressHandled) {
        _btnPcs.longPressHandled = true;
        Serial.println("[BUTTON] PCS long-press detected (>=1.0s). Entering Sample Wizard...");
        startSampleWizard();
      }
    }

    if (evPcs == -1) {
      if (pwrBtn == &_btnPcs) {
        _isShutdownCountdownActive = false;
      }
      if (!_btnPcs.longPressHandled) {
        Serial.println("[BUTTON] PCS button click detected.");
        togglePcsMode();
      }
      _btnPcs.longPressHandled = false;
    }
  }

  void checkWizardButtons(int evHold, int evTare, int evUnit, int evPcs, unsigned long now) {
    // 1. CANCEL / ESC (NAV_BTN_CANCEL, GPIO 1, _btnHold)
    if (evHold == -1) {
      if (!_btnHold.longPressHandled) {
        Serial.println("[WIZARD] Cancel requested (ESC). Exiting wizard.");
        exitSampleWizard(false);
        if (_display) {
          _display->showStatus("CANCELLED", nullptr, 600);
        }
      }
      _btnHold.longPressHandled = false;
    }

    // 2. CONFIRM / OK (NAV_BTN_CONFIRM, GPIO 5, _btnPcs)
    if (evPcs == -1) {
      if (!_btnPcs.longPressHandled) {
        Serial.printf("[WIZARD] Confirming reference sample for %u pcs...\n", _sampleCount);
        calibratePcsWithSample(_sampleCount);
      }
      _btnPcs.longPressHandled = false;
    }

    // 3. INCREMENT [+] (NAV_BTN_INC, GPIO 6, _btnUnit)
    if (evUnit == 1) {
      // Debounced single click: exactly 1 step
      _wizardStartTime = now;
      _incLastRepeatTime = now;
      if (_sampleCount < 9999) _sampleCount++;
      Serial.printf("[WIZARD] Sample Count [+]: %u pcs\n", _sampleCount);
      updateWizardDisplay();
    } else if (_btnUnit.isPressed) {
      // Continuous hold: adaptive auto-repeat acceleration curve
      _wizardStartTime = now;
      unsigned long holdMs = _btnUnit.holdDuration(now);
      const unsigned long INITIAL_DWELL_MS = 400; // 400ms dwell before auto-repeat

      if (holdMs >= INITIAL_DWELL_MS) {
        uint16_t step = 1;
        unsigned long interval = 130;

        if (holdMs >= 5000) {
          step = 50;
          interval = 60;
        } else if (holdMs >= 3000) {
          step = 10;
          interval = 70;
        } else if (holdMs >= 1500) {
          step = 2;
          interval = 90;
        } else {
          step = 1;
          interval = 130;
        }

        if (now - _incLastRepeatTime >= interval) {
          _incLastRepeatTime = now;
          if ((uint32_t)_sampleCount + step > 9999) {
            _sampleCount = 9999;
          } else {
            _sampleCount += step;
          }
          updateWizardDisplay();
        }
      }
    }

    // 4. DECREMENT [-] (NAV_BTN_DEC, GPIO 0, _btnTare)
    if (evTare == 1) {
      // Debounced single click: exactly 1 step
      _wizardStartTime = now;
      _decLastRepeatTime = now;
      if (_sampleCount > 1) _sampleCount--;
      Serial.printf("[WIZARD] Sample Count [-]: %u pcs\n", _sampleCount);
      updateWizardDisplay();
    } else if (_btnTare.isPressed) {
      // Continuous hold: adaptive auto-repeat acceleration curve
      _wizardStartTime = now;
      unsigned long holdMs = _btnTare.holdDuration(now);
      const unsigned long INITIAL_DWELL_MS = 400; // 400ms dwell before auto-repeat

      if (holdMs >= INITIAL_DWELL_MS) {
        uint16_t step = 1;
        unsigned long interval = 130;

        if (holdMs >= 5000) {
          step = 50;
          interval = 60;
        } else if (holdMs >= 3000) {
          step = 10;
          interval = 70;
        } else if (holdMs >= 1500) {
          step = 2;
          interval = 90;
        } else {
          step = 1;
          interval = 130;
        }

        if (now - _decLastRepeatTime >= interval) {
          _decLastRepeatTime = now;
          if (_sampleCount > step) {
            _sampleCount -= step;
          } else {
            _sampleCount = 1;
          }
          updateWizardDisplay();
        }
      }
    }
  }

  void checkButtons() {
    unsigned long now = millis();

    int evHold = _btnHold.update(now);
    int evTare = _btnTare.update(now);
    int evUnit = _btnUnit.update(now);
    int evPcs  = _btnPcs.update(now);

    if (_btnHold.isPressed || _btnTare.isPressed || _btnUnit.isPressed || _btnPcs.isPressed) {
      resetActivityTimer();
    }

    if (_inSampleWizard) {
      checkWizardButtons(evHold, evTare, evUnit, evPcs, now);
    } else {
      checkNormalButtons(evHold, evTare, evUnit, evPcs, now);
    }
  }

  void handleSerialCommand(String cmd) {
    cmd.trim();
    resetActivityTimer();

    if (cmd.equalsIgnoreCase("t") || cmd.equalsIgnoreCase("tare")) {
      performTare();
    } else if (cmd.equalsIgnoreCase("tarespeed") || cmd.equalsIgnoreCase("tareanim")) {
      Serial.println("--------------------------------------------------");
      Serial.printf("  [TARE ANIMATION] Duration: %u ms (%s)\n",
                    _tareAnimDurationMs,
                    (_tareAnimDurationMs == 0) ? "Instant" :
                    (_tareAnimDurationMs <= 60) ? "Ultra Quick" :
                    (_tareAnimDurationMs <= 120) ? "Super Quick (Default)" : "Quick Slide (Max 200ms)");
      Serial.printf("  [TARE METROLOGY] Accuracy: %d samples (Trimmed Mean, Decoupled)\n", TARE_ACCURACY_SAMPLES);
      Serial.println("  Usage: tarespeed <ms | instant | ultra | quick | fast | max>");
      Serial.println("--------------------------------------------------");
    } else if (cmd.startsWith("tarespeed ") || cmd.startsWith("tareanim ") ||
               cmd.startsWith("tare speed ") || cmd.startsWith("tare anim ")) {
      int spaceIdx = cmd.lastIndexOf(' ');
      String arg = cmd.substring(spaceIdx + 1);
      arg.trim();
      arg.toLowerCase();
      if (arg == "instant" || arg == "off" || arg == "none") {
        setTareAnimDurationMs(0);
      } else if (arg == "ultra" || arg == "fastest") {
        setTareAnimDurationMs(50);
      } else if (arg == "quick" || arg == "super") {
        setTareAnimDurationMs(100);
      } else if (arg == "fast") {
        setTareAnimDurationMs(150);
      } else if (arg == "normal" || arg == "max") {
        setTareAnimDurationMs(200);
      } else {
        int val = arg.toInt();
        if (val >= MIN_TARE_ANIM_MS && val <= MAX_TARE_ANIM_MS) {
          setTareAnimDurationMs((uint16_t)val);
        } else {
          Serial.printf("[ERROR] Enter duration between %d and %d ms, or instant/ultra/quick/fast/max (e.g. 'tarespeed 100')\n",
                        MIN_TARE_ANIM_MS, MAX_TARE_ANIM_MS);
        }
      }
    } else if (cmd.equalsIgnoreCase("u") || cmd.equalsIgnoreCase("unit")) {
      cycleUnit();
    } else if (cmd.equalsIgnoreCase("pcs")) {
      togglePcsMode();
    } else if (cmd.startsWith("sample ") || cmd.startsWith("pcs sample ")) {
      int spaceIdx = cmd.lastIndexOf(' ');
      uint16_t cnt = cmd.substring(spaceIdx + 1).toInt();
      if (cnt > 0) {
        calibratePcsWithSample(cnt);
      } else {
        Serial.println("[ERROR] Enter valid sample count, e.g.: sample 25");
      }
    } else if (cmd.startsWith("apw ")) {
      float apw = cmd.substring(4).toFloat();
      if (apw > 0.0f) {
        setManualAPW(apw);
      } else {
        Serial.println("[ERROR] Enter valid piece weight in grams, e.g.: apw 0.245");
      }
    } else if (cmd.equalsIgnoreCase("apw")) {
      int32_t netCounts = (int32_t)(_filteredADC - (float)_tareOffset - _autoZeroCorrection);
      float netGrams = (_scaleFactor != 0) ? ((float)netCounts / _scaleFactor) : 0.0f;
      float count = (_pieceWeight > 0.0001f) ? (netGrams / _pieceWeight) : 0.0f;
      Serial.println("--------------------------------------------------");
      Serial.printf("  [APW] Active Mode:          %s\n", (_operatingMode == MODE_PCS) ? "PCS COUNTING" : "WEIGHING");
      Serial.printf("  [APW] Piece Weight:         %.4f g/pc\n", _pieceWeight);
      Serial.printf("  [APW] Reference Sample:     %u pcs\n", _sampleCount);
      Serial.printf("  [APW] Current Net Weight:   %.2f g\n", netGrams);
      Serial.printf("  [APW] Calculated Count:     %ld pcs (raw: %.2f)\n", lround(count), count);
      Serial.println("--------------------------------------------------");
    } else if (cmd.equalsIgnoreCase("hold")) {
      toggleHold();
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
      int32_t trimmedVal = getTrimmedMeanADC();
      Serial.printf("[RAW] RawADC: %ld | TrimmedADC: %ld | TareOffset: %ld | Net: %ld | ScaleFactor: %.4f\n",
                    (long)_rawADC, (long)trimmedVal, (long)_tareOffset, (long)(trimmedVal - _tareOffset), _scaleFactor);
    } else if (cmd.equalsIgnoreCase("h") || cmd.equalsIgnoreCase("help")) {
      printHelp();
    }
  }

  void processSerial() {
    while (Serial.available()) {
      char c = (char)Serial.read();
      if (c == '\r') continue;
      if (c == '\n') {
        _serialBuf[_serialIndex] = '\0';
        if (_serialIndex > 0) {
          handleSerialCommand(String(_serialBuf));
          _serialIndex = 0;
        }
      } else {
        if (_serialIndex < sizeof(_serialBuf) - 1) {
          _serialBuf[_serialIndex++] = c;
        }
      }
    }
  }
};
