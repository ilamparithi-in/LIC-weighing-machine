/**
 * ESP32-C3 SuperMini + 7Semi NAU7802 24-bit ADC + 0.91" I2C OLED (SSD1306 128x32)
 *
 * Pin Connections:
 * ====================================================================
 * ESP32-C3 SuperMini      7Semi NAU7802          0.91" I2C OLED (SSD1306)
 * --------------------------------------------------------------------
 * 3V3                     VIN                    VCC
 * 3V3 (or internal LDO)   AVDD (Excitation)      -
 * GND                     GND                    GND
 * GPIO 6 (SDA)            SDA                    SDA
 * GPIO 7 (SCL)            SCL                    SCL
 * 
 * Note: The onboard BOOT button (GPIO 9) is used as the TARE button!
 *
 * Load Cell (Standard 4-Wire) to 7Semi NAU7802:
 * --------------------------------------------------------------------
 * E+ (Excitation +) -> RED
 * E- (Excitation -) -> BLACK
 * A+ (Signal +)     -> GREEN
 * A- (Signal -)     -> WHITE
 * ====================================================================
 */

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <7Semi_NAU7802.h>
#include <Preferences.h>
#include <WiFi.h>
#include "esp_wifi.h"
#include "esp_bt.h"

// ---------------------- PIN DEFINITIONS ----------------------
#define I2C_SDA_PIN        6     // I2C SDA (GPIO 6)
#define I2C_SCL_PIN        7     // I2C SCL (GPIO 7)
#define TARE_BUTTON_PIN    9     // On-board BOOT Button on ESP32-C3 SuperMini (active LOW)

// ---------------------- OLED DISPLAY CONFIG ------------------
#define SCREEN_WIDTH     128     // OLED display width in pixels
#define SCREEN_HEIGHT     32     // 0.91" OLED display height in pixels
#define OLED_RESET        -1     // Reset pin # (or -1 if sharing Arduino reset pin)

// ---------------------- OBJECTS & STORAGE --------------------
NAU7802_7Semi scale(Wire);
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
Preferences prefs;

// ---------------------- CONFIG & CALIBRATION -----------------
uint8_t oledAddress = 0x3C;      // Typical 0.91" OLED I2C address (0x3C or 0x3D)
const char* PREFS_NAMESPACE = "scale_cfg";
float scaleFactor = 93.44f;      // Calibrated counts per gram
int32_t tareOffset = 81628;      // Empty-scale baseline tare offset
bool isCalibrated = true;

// Debounced button polling variables (immune to USB UART DTR/RTS boot glitches)
unsigned long buttonPressStartTime = 0;
bool buttonActive = false;

// ---------------------- DIGITAL FILTER CONFIG ----------------
// 12-sample rolling buffer for raw ADC samples
const int FILTER_SIZE = 12;
int32_t adcBuffer[FILTER_SIZE];
int bufferIndex = 0;
bool bufferFull = false;

// Display and stability variables
unsigned long lastDisplayUpdate = 0;
const unsigned long DISPLAY_INTERVAL_MS = 250;  // 4 Hz OLED update rate
float filteredWeight = 0.0f;
float lockedWeight = 0.0f;
bool isStable = false;
int stableFrames = 0;
int32_t latestFilteredADC = 0;

// ---------------------- FORWARD DECLARATIONS -----------------
bool scanAndInitOLED();
void showStatusScreen(const char* line1, const char* line2 = nullptr, uint16_t delayMs = 0);
void drawScaleScreen(float weight, int32_t netCounts, bool saturated, bool tick, bool stable);
void enableInternalLDO();
void calibrateAFE();
void performTare();
void calibrateScaleWithWeight(float knownWeight);
void checkTareButton();
void loadSettings();
void saveSettings();
void printHelp();
int32_t getTrimmedMeanADC();

void setup() {
  // 1. Completely disable Wi-Fi and Bluetooth / BLE radios to eliminate
  // 2.4 GHz RF emissions, switching harmonics, and power rail ripple.
  WiFi.mode(WIFI_OFF);
  esp_wifi_stop();
  esp_wifi_deinit();

  btStop();
  esp_bt_controller_disable();
  esp_bt_controller_deinit();
  esp_bt_controller_mem_release(ESP_BT_MODE_BLE);

  // Reduce CPU clock from 160MHz down to 80MHz for minimal thermal drift & rail ripple.
  setCpuFrequencyMhz(80);

  Serial.begin(115200);
#if ARDUINO_USB_CDC_ON_BOOT
  // Disable TX blocking on ESP32-C3 native USB CDC so the loop
  // never stalls when the serial monitor/terminal is closed!
  Serial.setTxTimeoutMs(0);
#endif
  delay(500);

  Serial.println();
  Serial.println("====================================================");
  Serial.println(" ESP32-C3 SuperMini + NAU7802 + 0.91\" OLED Scale   ");
  Serial.println("====================================================");

  // Initialize BOOT button as Tare with internal pullup
  pinMode(TARE_BUTTON_PIN, INPUT_PULLUP);

  // Initialize I2C Bus on GPIO 6 & 7
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN, 400000);
  delay(50);

  // Auto-detect and initialize 0.91" OLED (0x3C or 0x3D)
  if (!scanAndInitOLED()) {
    Serial.println("[ERROR] OLED Display initialization failed! Check wiring.");
  } else {
    showStatusScreen("ESP32-C3 Scale", "Init NAU7802...", 500);
  }

  // Initialize 7Semi NAU7802
  Serial.println("[INFO] Initializing NAU7802 ADC...");
  if (!scale.begin(I2C_SDA_PIN, I2C_SCL_PIN, 400000)) {
    Serial.println("[ERROR] NAU7802 not found! Check wiring (3.3V, GND, SDA, SCL).");
    showStatusScreen("NAU7802 ERROR!", "Check Wiring");
    while (1) delay(1000);
  }

  // Set internal LDO to 3.0V (provides ~300mV headroom below 3.3V VIN for clean regulation)
  scale.setConfig(LDO_3_0V, PGA_128);

  // Set conversion rate to 10 SPS for built-in 50/60Hz notch filter rejection
  scale.setSampleRate(CRS_10);

  // Nuvoton recommended setting: Set bits 4 & 5 in ADC_CTRL1 to eliminate alternating chopper ripple
  Wire.beginTransmission(0x2A);
  Wire.write(0x15); // REG_ADC_CTRL1
  Wire.endTransmission(false);
  if (Wire.requestFrom((uint8_t)0x2A, (uint8_t)1) == 1) {
    uint8_t ctrl1 = Wire.read();
    ctrl1 |= 0x30; // Set bits 4 & 5 to eliminate alternating chopper modulation ripple
    Wire.beginTransmission(0x2A);
    Wire.write(0x15);
    Wire.write(ctrl1);
    Wire.endTransmission();
    Serial.printf("[INFO] Chopper ripple filter set. ADC_CTRL1 = 0x%02X\n", ctrl1);
  }

  // Force-enable the NAU7802 internal LDO by setting AVDDS (bit 7 of REG_PU_CTRL)
  enableInternalLDO();

  // Run internal Analog Front-End (AFE) offset calibration
  calibrateAFE();

  // Load saved calibration and tare offset from flash memory
  loadSettings();

  // Pre-fill rolling filter buffer with initial samples
  Serial.println("[INFO] Stabilizing filter buffer...");
  showStatusScreen("Stabilizing...", "Please wait", 300);
  int filled = 0;
  unsigned long startFill = millis();
  while (filled < FILTER_SIZE && (millis() - startFill < 1500)) {
    int32_t s = 0;
    if (scale.readRaw(s, 150)) {
      adcBuffer[filled++] = s;
    }
    delay(5);
  }
  bufferIndex = 0;
  bufferFull = (filled == FILTER_SIZE);
  latestFilteredADC = getTrimmedMeanADC();
  filteredWeight = (scaleFactor != 0) ? ((float)(latestFilteredADC - tareOffset) / scaleFactor) : 0.0f;
  lockedWeight = filteredWeight;

  printHelp();
}

void loop() {
  // 1. Debounced Tare button polling (immune to USB UART reset glitches)
  checkTareButton();

  // 2. Check Serial Commands (if serial port has data)
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd.equalsIgnoreCase("t") || cmd.equalsIgnoreCase("tare")) {
      performTare();
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
      tareOffset = off;
      saveSettings();
      Serial.printf("[INFO] Manual TareOffset set to: %ld\n", (long)tareOffset);
    } else if (cmd.startsWith("factor ")) {
      float f = cmd.substring(7).toFloat();
      if (f > 0) {
        scaleFactor = f;
        saveSettings();
        Serial.printf("[INFO] Manual ScaleFactor set to: %.4f\n", scaleFactor);
      }
    } else if (cmd.equalsIgnoreCase("r") || cmd.equalsIgnoreCase("raw")) {
      int32_t rawVal = getTrimmedMeanADC();
      Serial.printf("[RAW] FilteredADC: %ld | TareOffset: %ld | Net: %ld | ScaleFactor: %.4f\n", 
                    (long)rawVal, (long)tareOffset, (long)(rawVal - tareOffset), scaleFactor);
    } else if (cmd.equalsIgnoreCase("h") || cmd.equalsIgnoreCase("help")) {
      printHelp();
    }
  }

  // 3. Continuously acquire ADC samples into rolling ring buffer
  if (scale.available()) {
    int32_t val = 0;
    if (scale.readRaw(val, 5)) {
      adcBuffer[bufferIndex] = val;
      bufferIndex = (bufferIndex + 1) % FILTER_SIZE;
      if (bufferIndex == 0) bufferFull = true;

      // Get trimmed-mean averaged ADC count (eliminates single-sample noise & chopper ripple)
      latestFilteredADC = getTrimmedMeanADC();
      int32_t netCounts = latestFilteredADC - tareOffset;
      float targetWeight = (scaleFactor != 0) ? ((float)netCounts / scaleFactor) : 0.0f;

      // Adaptive Smart Filter (1g integer resolution):
      // - Real load change (> 8.0g): Fast tracking for placing/removing items
      // - Resting load (<= 8.0g): Smooth filtering (6% new sample per frame)
      float delta = abs(targetWeight - filteredWeight);
      if (delta > 8.0f) {
        filteredWeight = (0.60f * targetWeight) + (0.40f * filteredWeight);
        isStable = false;
        stableFrames = 0;
      } else {
        filteredWeight = (0.06f * targetWeight) + (0.94f * filteredWeight);
        if (stableFrames < 30) stableFrames++;
        if (stableFrames >= 6) {
          isStable = true;
          // Lock to stable integer 1g weight with 0.8g hysteresis
          if (abs(filteredWeight - lockedWeight) >= 0.8f) {
            lockedWeight = (float)lround(filteredWeight);
          }
        }
      }

      // Snap to 0g near zero (< 0.6g)
      if (abs(filteredWeight) < 0.6f) {
        filteredWeight = 0.0f;
        lockedWeight = 0.0f;
      }
    }
  }

  // 4. Refresh 0.91" OLED at exact 250ms intervals
  if (millis() - lastDisplayUpdate >= DISPLAY_INTERVAL_MS) {
    lastDisplayUpdate = millis();

    static bool tick = false;
    tick = !tick;

    int32_t netCounts = latestFilteredADC - tareOffset;
    bool isSaturated = (latestFilteredADC <= -8388600 || latestFilteredADC >= 8388600);
    float displayValue = isStable ? lockedWeight : filteredWeight;

    drawScaleScreen(displayValue, netCounts, isSaturated, tick, isStable);

    // Serial stream output (1g resolution)
    if (Serial.availableForWrite() >= 64) {
      Serial.printf("Weight: %ld g | Raw: %ld | Net: %ld | %s\n", 
                    (long)lround(displayValue), (long)latestFilteredADC, (long)netCounts, isStable ? "[STABLE]" : "[MOVING]");
    }
  }
}

// ---------------------- OLED DISPLAY FUNCTIONS ----------------

/**
 * Auto-detects 0.91" OLED address (0x3C or 0x3D) and initializes SSD1306.
 */
bool scanAndInitOLED() {
  Serial.println("[INFO] Scanning I2C bus for 0.91\" OLED...");
  bool oledFound = false;
  uint8_t candidates[] = {0x3C, 0x3D};

  for (uint8_t addr : candidates) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      oledAddress = addr;
      oledFound = true;
      Serial.printf("[INFO] OLED detected at address: 0x%02X\n", oledAddress);
      break;
    }
  }

  if (!oledFound) {
    Serial.println("[WARN] OLED not detected at 0x3C or 0x3D, defaulting to 0x3C");
    oledAddress = 0x3C;
  }

  // Initialize SSD1306 display (reset = false, periphBegin = false)
  if (!display.begin(SSD1306_SWITCHCAPVCC, oledAddress, false, false)) {
    return false;
  }

  // Ensure Wire remains explicitly on GPIO 6 & 7 at 400kHz
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN, 400000);

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.display();
  return true;
}

/**
 * Displays full-screen status message or notification.
 */
void showStatusScreen(const char* line1, const char* line2, uint16_t delayMs) {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  if (line2 == nullptr) {
    // Single line: large font centered
    display.setTextSize(2);
    display.setCursor(0, 8);
    display.print(line1);
  } else {
    // Two lines: line 1 slightly larger, line 2 smaller
    display.setTextSize(1);
    display.setCursor(0, 4);
    display.print(line1);
    display.drawFastHLine(0, 16, 128, SSD1306_WHITE);
    display.setCursor(0, 20);
    display.print(line2);
  }

  display.display();
  if (delayMs > 0) {
    delay(delayMs);
  }
}

/**
 * Renders the main weighing scale interface on the 128x32 OLED display.
 * 
 * Layout:
 * +-----------------------------------------------+
 * |  608.0 g                                  [*] |  <- Size 2 Weight + Heartbeat dot
 * |-----------------------------------------------|  <- Horizontal rule
 * | STABLE  6043c                             [C] |  <- Size 1 Status, Net counts & Calibrated badge
 * +-----------------------------------------------+
 */
void drawScaleScreen(float weight, int32_t netCounts, bool saturated, bool tick, bool stable) {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  // Line 1: Weight in large Size 2 font (12x16 pixels per char)
  display.setTextSize(2);
  display.setCursor(0, 0);

  long displayGrams = lround(weight);
  char weightBuf[16];
  snprintf(weightBuf, sizeof(weightBuf), "%5ld g", displayGrams);
  display.print(weightBuf);

  // Heartbeat indicator icon in top-right corner (128x32 display)
  if (tick) {
    display.fillCircle(123, 6, 2, SSD1306_WHITE);
  } else {
    display.drawCircle(123, 6, 2, SSD1306_WHITE);
  }

  // Divider line
  display.drawFastHLine(0, 18, 128, SSD1306_WHITE);

  // Line 2: Status line in Size 1 font (6x8 pixels per char)
  display.setTextSize(1);
  display.setCursor(0, 22);

  if (saturated) {
    display.print("! ADC SATURATED !");
  } else {
    char subBuf[24];
    if (stable) {
      snprintf(subBuf, sizeof(subBuf), "STABLE %5ldc   [%c]", (long)netCounts, isCalibrated ? 'C' : 'U');
    } else {
      snprintf(subBuf, sizeof(subBuf), "Net:   %5ldc   [%c]", (long)netCounts, isCalibrated ? 'C' : 'U');
    }
    display.print(subBuf);
  }

  display.display();
}

// ---------------------- HELPER FUNCTIONS ---------------------

/**
 * Checks physical BOOT button (GPIO 9) with 120ms debounce.
 */
void checkTareButton() {
  if (digitalRead(TARE_BUTTON_PIN) == LOW) {
    if (buttonPressStartTime == 0) {
      buttonPressStartTime = millis();
    } else if (millis() - buttonPressStartTime > 120 && !buttonActive) {
      buttonActive = true;
      Serial.println("[BUTTON] BOOT button pressed. Performing Tare...");
      performTare();
    }
  } else {
    buttonPressStartTime = 0;
    buttonActive = false;
  }
}

/**
 * Calculates a trimmed-mean ADC value from the rolling buffer:
 * Discards outlier samples and averages the middle samples.
 */
int32_t getTrimmedMeanADC() {
  int count = bufferFull ? FILTER_SIZE : bufferIndex;
  if (count <= 0) return 0;
  if (count < 4) {
    long sum = 0;
    for (int i = 0; i < count; i++) sum += adcBuffer[i];
    return sum / count;
  }

  int32_t sorted[FILTER_SIZE];
  for (int i = 0; i < count; i++) sorted[i] = adcBuffer[i];

  // Simple insertion sort
  for (int i = 1; i < count; i++) {
    int32_t key = sorted[i];
    int j = i - 1;
    while (j >= 0 && sorted[j] > key) {
      sorted[j + 1] = sorted[j];
      j--;
    }
    sorted[j + 1] = key;
  }

  // Drop 2 lowest and 2 highest
  int drop = 2;
  long sum = 0;
  int used = 0;
  for (int i = drop; i < count - drop; i++) {
    sum += sorted[i];
    used++;
  }
  return (used > 0) ? (sum / used) : sorted[count / 2];
}

/**
 * Ensures the NAU7802 internal LDO is powered up and routed to AVDD.
 */
void enableInternalLDO() {
  Wire.beginTransmission(0x2A);
  Wire.write(0x00); // REG_PU_CTRL
  if (Wire.endTransmission(false) == 0 && Wire.requestFrom((uint8_t)0x2A, (uint8_t)1) == 1) {
    uint8_t pu = Wire.read();
    pu |= (1 << 7); // AVDDS = 1 (Enable internal LDO)
    pu |= (1 << 4); // CS = 1 (Cycle start)
    Wire.beginTransmission(0x2A);
    Wire.write(0x00);
    Wire.write(pu);
    Wire.endTransmission();
    Serial.printf("[INFO] Internal LDO enabled. PU_CTRL = 0x%02X\n", pu);
  }
}

/**
 * Runs internal AFE offset calibration to cancel amplifier DC offset.
 */
void calibrateAFE() {
  Wire.beginTransmission(0x2A);
  Wire.write(0x02); // REG_CTRL2
  if (Wire.endTransmission(false) == 0 && Wire.requestFrom((uint8_t)0x2A, (uint8_t)1) == 1) {
    uint8_t c2 = Wire.read();
    c2 &= 0xFC; // CALMOD = 00 (Offset calibration)
    c2 |= (1 << 2); // CALS = 1 (Start calibration)
    Wire.beginTransmission(0x2A);
    Wire.write(0x02);
    Wire.write(c2);
    Wire.endTransmission();

    unsigned long start = millis();
    while (millis() - start < 400) {
      Wire.beginTransmission(0x2A);
      Wire.write(0x02);
      if (Wire.endTransmission(false) == 0 && Wire.requestFrom((uint8_t)0x2A, (uint8_t)1) == 1) {
        if ((Wire.read() & (1 << 2)) == 0) break; // Finished
      }
      delay(10);
    }
    Serial.println("[INFO] AFE internal offset calibration complete.");
  }
}

void performTare() {
  Serial.println("[ACTION] Performing Tare (Zeroing)...");
  showStatusScreen("TARING...", "Keep plate empty");

  // Read 20 samples and use trimmed mean for exact zero
  const int TARE_SAMPLES = 20;
  int32_t samples[TARE_SAMPLES];
  int validSamples = 0;
  unsigned long start = millis();
  while (validSamples < TARE_SAMPLES && (millis() - start < 2500)) {
    int32_t val;
    if (scale.readRaw(val, 120)) {
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

    // Discard top 20% and bottom 20%
    int drop = validSamples / 5;
    long sum = 0;
    int used = 0;
    for (int i = drop; i < validSamples - drop; i++) {
      sum += samples[i];
      used++;
    }
    tareOffset = (used > 0) ? (sum / used) : samples[validSamples / 2];

    scale.setTareOffset(tareOffset);

    // Flush rolling filter buffer so old samples do not contaminate post-tare readings
    for (int i = 0; i < FILTER_SIZE; i++) {
      adcBuffer[i] = tareOffset;
    }
    bufferIndex = 0;
    bufferFull = true;
    latestFilteredADC = tareOffset;
    filteredWeight = 0.0f;
    lockedWeight = 0.0f;
    isStable = true;
    stableFrames = 15;

    saveSettings();
    Serial.printf("[SUCCESS] Tare locked. Offset = %ld (samples: %d)\n", (long)tareOffset, validSamples);
    
    char offsetBuf[22];
    snprintf(offsetBuf, sizeof(offsetBuf), "Offset: %ld", (long)tareOffset);
    showStatusScreen("TARE OK!", offsetBuf, 400);
  } else {
    Serial.println("[ERROR] Tare failed! ADC timeout.");
    showStatusScreen("TARE ERROR!", "ADC Timeout", 800);
  }
}

void calibrateScaleWithWeight(float knownWeight) {
  Serial.printf("[ACTION] Calibrating with reference weight: %.2f g\n", knownWeight);
  
  char refBuf[22];
  snprintf(refBuf, sizeof(refBuf), "Ref: %.1fg", knownWeight);
  showStatusScreen("CALIBRATING...", refBuf);

  // Take trimmed average reading with known weight
  const int CAL_SAMPLES = 20;
  int32_t samples[CAL_SAMPLES];
  int validSamples = 0;
  unsigned long start = millis();
  while (validSamples < CAL_SAMPLES && (millis() - start < 3000)) {
    int32_t val;
    if (scale.readRaw(val, 120)) {
      samples[validSamples++] = val;
    }
    delay(10);
  }

  if (validSamples >= 10) {
    // Sort and trimmed mean
    for (int i = 1; i < validSamples; i++) {
      int32_t key = samples[i];
      int j = i - 1;
      while (j >= 0 && samples[j] > key) {
        samples[j + 1] = samples[j];
        j--;
      }
      samples[j + 1] = key;
    }

    // Discard top 20% and bottom 20%
    int drop = validSamples / 5;
    long sum = 0;
    int used = 0;
    for (int i = drop; i < validSamples - drop; i++) {
      sum += samples[i];
      used++;
    }
    int32_t avgVal = (used > 0) ? (sum / used) : samples[validSamples / 2];

    int32_t net = avgVal - tareOffset;
    scaleFactor = (float)net / knownWeight;
    isCalibrated = true;
    scale.setScaleFactor(scaleFactor);

    // Flush rolling filter buffer with reference weight
    for (int i = 0; i < FILTER_SIZE; i++) {
      adcBuffer[i] = avgVal;
    }
    bufferIndex = 0;
    bufferFull = true;
    latestFilteredADC = avgVal;
    filteredWeight = knownWeight;
    lockedWeight = knownWeight;
    isStable = true;
    stableFrames = 15;

    saveSettings();
    Serial.printf("[SUCCESS] Calibration complete! Factor: %.4f counts/g (Net: %ld)\n", scaleFactor, (long)net);

    char factorBuf[22];
    snprintf(factorBuf, sizeof(factorBuf), "F: %.1f c/g", scaleFactor);
    showStatusScreen("CAL SUCCESS!", factorBuf, 1200);
  } else {
    Serial.println("[ERROR] Calibration failed. Make sure weight is steady.");
    showStatusScreen("CAL FAILED!", "Check weight", 1200);
  }
}

void loadSettings() {
  prefs.begin(PREFS_NAMESPACE, true);
  scaleFactor = prefs.getFloat("scale_factor", 93.44f);
  tareOffset = prefs.getInt("tare_offset", 81628);
  isCalibrated = prefs.getBool("calibrated", true);
  prefs.end();

  if (scaleFactor <= 0.1f) scaleFactor = 93.44f;
  if (tareOffset == 0) tareOffset = 81628;

  scale.setScaleFactor(scaleFactor);
  scale.setTareOffset(tareOffset);

  Serial.printf("[CONFIG] Loaded Settings -> Factor: %.4f | Offset: %ld | Calibrated: %s\n",
                scaleFactor, (long)tareOffset, isCalibrated ? "YES" : "NO");
}

void saveSettings() {
  prefs.begin(PREFS_NAMESPACE, false);
  prefs.putFloat("scale_factor", scaleFactor);
  prefs.putInt("tare_offset", tareOffset);
  prefs.putBool("calibrated", isCalibrated);
  prefs.end();
  Serial.println("[CONFIG] Saved settings to Non-Volatile Flash.");
}

void printHelp() {
  Serial.println();
  Serial.println("--- SERIAL COMMANDS ---");
  Serial.println("  t              : Tare (zero) scale");
  Serial.println("  c <weight>     : Calibrate with known weight (e.g., 'c 608')");
  Serial.println("  offset <val>   : Manually set tare offset (e.g., 'offset 75491')");
  Serial.println("  factor <val>   : Manually set scale factor (e.g., 'factor 9.94')");
  Serial.println("  r              : Read raw ADC and scale diagnostics");
  Serial.println("  h              : Print this help menu");
  Serial.println("-----------------------");
}
