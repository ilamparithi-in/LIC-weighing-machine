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
#define TARE_PIN_D2        2     // External Tare Button on D2 (GPIO 2, active LOW / pull to GND)
#define UNIT_BUTTON_PIN    4     // External Unit Change Button on D4 (GPIO 4, active LOW / pull to GND)

// ---------------------- UNITS OF MEASURE --------------------
enum WeightUnit {
  UNIT_G = 0,                    // Grams (e.g., 607 g)
  UNIT_KG = 1,                   // Kilograms (e.g., 0.607 kg)
  UNIT_OZ = 2,                   // Ounces (e.g., 21.41 oz)
  UNIT_LB = 3                    // Pounds (e.g., 1.338 lb)
};
WeightUnit currentUnit = UNIT_G;

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

// Debounced tare button polling variables (D2 pin & BOOT pin GPIO 9)
unsigned long buttonPressStartTime = 0;
bool buttonActive = false;

// Debounced unit change button polling variables (D4 pin GPIO 4)
unsigned long unitPressStartTime = 0;
bool unitButtonActive = false;

// ---------------------- DIGITAL FILTER CONFIG ----------------
// 8-sample fast rolling buffer for ADC samples (200ms window at 40 SPS)
const int FILTER_SIZE = 8;
int32_t adcBuffer[FILTER_SIZE];
int bufferIndex = 0;
bool bufferFull = false;

// Display and stability variables
unsigned long lastDisplayUpdate = 0;
const unsigned long DISPLAY_INTERVAL_MS = 100;  // 10 Hz crisp OLED update rate
float filteredWeight = 0.0f;
float lockedWeight = 0.0f;
bool isStable = false;
int stableFrames = 0;
int32_t latestFilteredADC = 0;

// ---------------------- FORWARD DECLARATIONS -----------------
bool scanAndInitOLED();
void showStatusScreen(const char* line1, const char* line2 = nullptr, uint16_t delayMs = 0);
void drawScaleScreen(float weight, bool saturated = false);
void enableInternalLDO();
void calibrateAFE();
void performTare();
void calibrateScaleWithWeight(float knownWeight);
void checkTareButton();
void checkUnitButton();
void cycleUnit();
void formatWeightString(float grams, WeightUnit unit, char* outBuf, size_t maxLen);
void formatWeightNumber(float grams, WeightUnit unit, char* outBuf, size_t maxLen);
const char* getUnitSymbol(WeightUnit unit);
const char* getUnitName(WeightUnit unit);
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

  // Initialize BOOT button and D2 as Tare inputs with internal pullups
  pinMode(TARE_BUTTON_PIN, INPUT_PULLUP);
  pinMode(TARE_PIN_D2, INPUT_PULLUP);

  // Initialize D4 as Unit Change button input with internal pullup
  pinMode(UNIT_BUTTON_PIN, INPUT_PULLUP);

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

  // Set conversion rate to 40 SPS for fast 25ms response time
  scale.setSampleRate(CRS_40);

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

  // Load saved calibration scale factor from flash memory
  loadSettings();

  // Always tare on startup to zero the empty scale
  performTare();

  printHelp();
}

void loop() {
  // 1. Debounced Button polling (Tare BOOT/D2 buttons & D4 Unit button)
  checkTareButton();
  checkUnitButton();

  // 2. Check Serial Commands (if serial port has data)
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd.equalsIgnoreCase("t") || cmd.equalsIgnoreCase("tare")) {
      performTare();
    } else if (cmd.equalsIgnoreCase("u") || cmd.equalsIgnoreCase("unit")) {
      cycleUnit();
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

      // Multi-tier fast-settling filter (1g integer resolution):
      // - Step change (> 3.0g): Fast slew tracking (70% new sample per frame)
      // - Final landing (delta <= 3.0g): Smooth landing (20% new sample per frame)
      float delta = abs(targetWeight - filteredWeight);
      if (delta > 3.0f) {
        filteredWeight = (0.70f * targetWeight) + (0.30f * filteredWeight);
        isStable = false;
        stableFrames = 0;
      } else {
        filteredWeight = (0.20f * targetWeight) + (0.80f * filteredWeight);
        if (stableFrames < 20) stableFrames++;
        if (stableFrames >= 4) {
          isStable = true;
          // Lock to stable integer 1g weight with 0.8g hysteresis
          if (abs(filteredWeight - lockedWeight) >= 0.8f) {
            lockedWeight = (float)lround(filteredWeight);
          }
        }
      }

      // Zero-tracking deadband: snap to 0g within +/-1.0g
      if (abs(filteredWeight) < 1.0f) {
        filteredWeight = 0.0f;
        lockedWeight = 0.0f;
      }
    }
  }

  // 4. Refresh 0.91" OLED at exact 250ms intervals
  if (millis() - lastDisplayUpdate >= DISPLAY_INTERVAL_MS) {
    lastDisplayUpdate = millis();

    int32_t netCounts = latestFilteredADC - tareOffset;
    bool isSaturated = (latestFilteredADC <= -8388600 || latestFilteredADC >= 8388600);
    float displayValue = isStable ? lockedWeight : filteredWeight;

    drawScaleScreen(displayValue, isSaturated);

    // Serial stream output formatted with current active unit
    if (Serial.availableForWrite() >= 64) {
      char serWeightBuf[20];
      formatWeightString(displayValue, currentUnit, serWeightBuf, sizeof(serWeightBuf));
      Serial.printf("Weight: %s | Raw: %ld | Net: %ld | %s\n", 
                    serWeightBuf, (long)latestFilteredADC, (long)netCounts, isStable ? "[STABLE]" : "[MOVING]");
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
 * Renders the full-screen weight interface on the 128x32 OLED display.
 * The measurement unit is fixed on the right side of the display,
 * and the digits are right-aligned to maintain a stable, non-jumping readout.
 */
void drawScaleScreen(float weight, bool saturated) {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  if (saturated) {
    display.setTextSize(2);
    display.setCursor(4, 8);
    display.print("! OVERLOAD !");
    display.display();
    return;
  }

  char numStr[16];
  formatWeightNumber(weight, currentUnit, numStr, sizeof(numStr));
  const char* unitStr = getUnitSymbol(currentUnit);

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
    display.setTextSize(3);
    display.setCursor(numX, 4); // y=4 centers 24px height, baseline at y=28
    display.print(numStr);
  } else {
    // If number is too wide for Size 3 (e.g. >= 10.000 kg), use Size 2
    int wNum2 = (lenNum * 12) - 2;
    int numX = digitsRightX - wNum2;
    if (numX < 0) numX = 0;
    display.setTextSize(2);
    display.setCursor(numX, 8); // y=8 centers 16px height
    display.print(numStr);
  }

  // Draw the fixed unit on the right side
  display.setTextSize(2);
  display.setCursor(unitX, unitY);
  display.print(unitStr);

  display.display();
}

// ---------------------- HELPER FUNCTIONS ---------------------

/**
 * Checks Tare buttons (D2 pin & physical BOOT button GPIO 9) with 80ms debounce.
 */
void checkTareButton() {
  bool isTareLow = (digitalRead(TARE_BUTTON_PIN) == LOW) || (digitalRead(TARE_PIN_D2) == LOW);
  if (isTareLow) {
    if (buttonPressStartTime == 0) {
      buttonPressStartTime = millis();
    } else if (millis() - buttonPressStartTime > 80 && !buttonActive) {
      buttonActive = true;
      Serial.println("[BUTTON] Tare requested via D2 / BOOT button. Performing Tare...");
      performTare();
    }
  } else {
    buttonPressStartTime = 0;
    buttonActive = false;
  }
}

/**
 * Checks Unit change button (D4 pin GPIO 4) with 80ms debounce.
 * Cycles through g -> kg -> oz -> lb.
 */
void checkUnitButton() {
  if (digitalRead(UNIT_BUTTON_PIN) == LOW) {
    if (unitPressStartTime == 0) {
      unitPressStartTime = millis();
    } else if (millis() - unitPressStartTime > 80 && !unitButtonActive) {
      unitButtonActive = true;
      Serial.println("[BUTTON] Unit change requested via D4 pin.");
      cycleUnit();
    }
  } else {
    unitPressStartTime = 0;
    unitButtonActive = false;
  }
}

/**
 * Cycles to the next weight unit (g -> kg -> oz -> lb -> g)
 * and saves user preference to non-volatile flash.
 */
void cycleUnit() {
  currentUnit = (WeightUnit)((currentUnit + 1) % 4);
  const char* uSym = getUnitSymbol(currentUnit);
  const char* uName = getUnitName(currentUnit);

  Serial.printf("[UNIT] Unit changed to: %s (%s)\n", uSym, uName);
  saveSettings();

  char unitLine[20];
  snprintf(unitLine, sizeof(unitLine), "UNIT: %s", uSym);
  showStatusScreen(unitLine, uName, 400);
}

const char* getUnitSymbol(WeightUnit unit) {
  switch (unit) {
    case UNIT_KG: return "kg";
    case UNIT_OZ: return "oz";
    case UNIT_LB: return "lb";
    case UNIT_G:
    default:      return "g";
  }
}

const char* getUnitName(WeightUnit unit) {
  switch (unit) {
    case UNIT_KG: return "Kilograms";
    case UNIT_OZ: return "Ounces";
    case UNIT_LB: return "Pounds";
    case UNIT_G:
    default:      return "Grams";
  }
}

/**
 * Formats weight value into string according to current unit:
 * - g:  integer resolution (e.g., "  607 g")
 * - kg: 3 decimals for 1g resolution (e.g., "0.607 kg")
 * - oz: 2 decimals for ~0.035 oz resolution (e.g., "21.41 oz")
 * - lb: 3 decimals for ~0.0022 lb resolution (e.g., "1.338 lb")
 */
void formatWeightString(float grams, WeightUnit unit, char* outBuf, size_t maxLen) {
  switch (unit) {
    case UNIT_KG: {
      float kg = grams / 1000.0f;
      if (abs(kg) < 0.0005f) kg = 0.0f;
      snprintf(outBuf, maxLen, "%.3f kg", kg);
      break;
    }
    case UNIT_OZ: {
      float oz = grams / 28.349523f;
      if (abs(oz) < 0.005f) oz = 0.0f;
      snprintf(outBuf, maxLen, "%.2f oz", oz);
      break;
    }
    case UNIT_LB: {
      float lb = grams / 453.59237f;
      if (abs(lb) < 0.0005f) lb = 0.0f;
      snprintf(outBuf, maxLen, "%.3f lb", lb);
      break;
    }
    case UNIT_G:
    default: {
      long g = lround(grams);
      snprintf(outBuf, maxLen, "%5ld g", g);
      break;
    }
  }
}

/**
 * Formats just the numeric weight value without unit or leading padding.
 */
void formatWeightNumber(float grams, WeightUnit unit, char* outBuf, size_t maxLen) {
  switch (unit) {
    case UNIT_KG: {
      float kg = grams / 1000.0f;
      if (abs(kg) < 0.0005f) kg = 0.0f;
      snprintf(outBuf, maxLen, "%.3f", kg);
      break;
    }
    case UNIT_OZ: {
      float oz = grams / 28.349523f;
      if (abs(oz) < 0.005f) oz = 0.0f;
      snprintf(outBuf, maxLen, "%.2f", oz);
      break;
    }
    case UNIT_LB: {
      float lb = grams / 453.59237f;
      if (abs(lb) < 0.0005f) lb = 0.0f;
      snprintf(outBuf, maxLen, "%.3f", lb);
      break;
    }
    case UNIT_G:
    default: {
      long g = lround(grams);
      snprintf(outBuf, maxLen, "%ld", g);
      break;
    }
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

  // Drop 1 lowest and 1 highest (averages middle 6 samples of 8-sample buffer)
  int drop = (count >= 6) ? 1 : 0;
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
  currentUnit = (WeightUnit)prefs.getUChar("unit", (uint8_t)UNIT_G);
  prefs.end();

  if (scaleFactor <= 0.1f) scaleFactor = 93.44f;
  if (tareOffset == 0) tareOffset = 81628;
  if (currentUnit > UNIT_LB) currentUnit = UNIT_G;

  scale.setScaleFactor(scaleFactor);
  scale.setTareOffset(tareOffset);

  Serial.printf("[CONFIG] Loaded Settings -> Factor: %.4f | Offset: %ld | Unit: %s | Calibrated: %s\n",
                scaleFactor, (long)tareOffset, getUnitSymbol(currentUnit), isCalibrated ? "YES" : "NO");
}

void saveSettings() {
  prefs.begin(PREFS_NAMESPACE, false);
  prefs.putFloat("scale_factor", scaleFactor);
  prefs.putInt("tare_offset", tareOffset);
  prefs.putBool("calibrated", isCalibrated);
  prefs.putUChar("unit", (uint8_t)currentUnit);
  prefs.end();
  Serial.println("[CONFIG] Saved settings to Non-Volatile Flash.");
}

void printHelp() {
  Serial.println();
  Serial.println("--- SERIAL COMMANDS ---");
  Serial.println("  t              : Tare (zero) scale");
  Serial.println("  u              : Cycle unit (g -> kg -> oz -> lb)");
  Serial.println("  c <weight>     : Calibrate with known weight (e.g., 'c 608')");
  Serial.println("  offset <val>   : Manually set tare offset (e.g., 'offset 75491')");
  Serial.println("  factor <val>   : Manually set scale factor (e.g., 'factor 9.94')");
  Serial.println("  r              : Read raw ADC and scale diagnostics");
  Serial.println("  h              : Print this help menu");
  Serial.println("-----------------------");
}
