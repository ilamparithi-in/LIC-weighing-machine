/**
 * ESP32-C3 SuperMini + 7Semi NAU7802 24-bit ADC + I2C LCD 16x2 Weighing Scale
 *
 * Pin Connections:
 * ====================================================================
 * ESP32-C3 SuperMini      7Semi NAU7802          I2C LCD 16x2 (PCF8574)
 * --------------------------------------------------------------------
 * 3V3                     VIN                    -
 * 3V3 (or internal LDO)   AVDD (Excitation)      -
 * 5V / VBUS               -                      VCC (or 3V3 if contrast allows)
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
#include <LiquidCrystal_I2C.h>
#include <7Semi_NAU7802.h>
#include <Preferences.h>

// ---------------------- PIN DEFINITIONS ----------------------
#define I2C_SDA_PIN        6     // I2C SDA (GPIO 6)
#define I2C_SCL_PIN        7     // I2C SCL (GPIO 7)
#define TARE_BUTTON_PIN    9     // On-board BOOT Button on ESP32-C3 SuperMini (active LOW)
#define PIN_D0             0     // D0 Button (GPIO 0, active LOW / pull to GND)
#define PIN_D1             1     // D1 Button (GPIO 1, active LOW / pull to GND)
#define PIN_D2             2     // D2 Button (GPIO 2, active LOW / pull to GND)

// ---------------------- OBJECTS & STORAGE --------------------
NAU7802_7Semi scale(Wire);
LiquidCrystal_I2C *lcd = nullptr;
Preferences prefs;

// ---------------------- CONFIG & CALIBRATION -----------------
uint8_t lcdAddress = 0x27;
const char* PREFS_NAMESPACE = "scale_cfg";
float scaleFactor = 88.0f;       // Calibrated counts per gram
int32_t tareOffset = 0;
bool isCalibrated = true;

// Tare button interrupt flag
volatile bool tareRequested = false;
volatile unsigned long lastTareInterruptTime = 0;

void IRAM_ATTR handleTareInterrupt() {
  unsigned long now = millis();
  if (now - lastTareInterruptTime > 400) {
    tareRequested = true;
    lastTareInterruptTime = now;
  }
}

// ---------------------- AUX BUTTONS (D0, D1, D2) -------------
struct AuxButton {
  uint8_t pin;
  const char* name;
  bool isPressed;
  unsigned long pressStart;
};

AuxButton auxButtons[] = {
  {PIN_D0, "D0", false, 0},
  {PIN_D1, "D1", false, 0},
  {PIN_D2, "D2", false, 0}
};

char activeButtonsStr[20] = "";
unsigned long buttonBannerUntil = 0;

// ---------------------- DIGITAL FILTER CONFIG ----------------
// 8-sample fast rolling buffer for ADC samples (200ms window at 40 SPS)
const int FILTER_SIZE = 8;
int32_t adcBuffer[FILTER_SIZE];
int bufferIndex = 0;
bool bufferFull = false;

// Display and stability variables
unsigned long lastDisplayUpdate = 0;
const unsigned long DISPLAY_INTERVAL_MS = 100;  // 10 Hz LCD update rate
float filteredWeight = 0.0f;
float lockedWeight = 0.0f;
bool isStable = false;
int stableFrames = 0;
int32_t latestFilteredADC = 0;

// ---------------------- FORWARD DECLARATIONS -----------------
void scanAndInitI2C();
void enableInternalLDO();
void calibrateAFE();
void performTare();
void calibrateScaleWithWeight(float knownWeight);
void checkAuxButtons();
void loadSettings();
void saveSettings();
void printHelp();
int32_t getTrimmedMeanADC();

void setup() {
  Serial.begin(115200);
  Serial.setTxTimeoutMs(0);
  delay(500);

  Serial.println();
  Serial.println("===============================================");
  Serial.println(" ESP32-C3 SuperMini + NAU7802 + I2C LCD Scale ");
  Serial.println("===============================================");

  // Initialize BOOT button as Tare with Hardware Interrupt
  pinMode(TARE_BUTTON_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(TARE_BUTTON_PIN), handleTareInterrupt, FALLING);

  // Initialize D0, D1, D2 auxiliary button inputs with internal pullups
  pinMode(PIN_D0, INPUT_PULLUP);
  pinMode(PIN_D1, INPUT_PULLUP);
  pinMode(PIN_D2, INPUT_PULLUP);

  // Initialize I2C Bus on GPIO 6 & 7
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN, 400000);
  delay(50);

  // Auto-detect LCD address (0x27 or 0x3F) and initialize
  scanAndInitI2C();

  lcd->init();
  lcd->backlight();
  lcd->clear();
  lcd->setCursor(0, 0);
  lcd->print("ESP32-C3 Scale");
  lcd->setCursor(0, 1);
  lcd->print("Init NAU7802...");

  // Initialize 7Semi NAU7802
  Serial.println("[INFO] Initializing NAU7802 ADC...");
  if (!scale.begin(I2C_SDA_PIN, I2C_SCL_PIN, 400000)) {
    Serial.println("[ERROR] NAU7802 not found! Check wiring (3.3V, GND, SDA, SCL).");
    lcd->clear();
    lcd->setCursor(0, 0);
    lcd->print("NAU7802 ERROR!");
    lcd->setCursor(0, 1);
    lcd->print("Check Wiring");
    while (1) delay(1000);
  }

  // Set internal LDO to 3.0V (provides ~300mV headroom below 3.3V VIN for clean regulation)
  scale.setConfig(LDO_3_0V, PGA_128);

  // Set conversion rate to 40 SPS for fast 25ms response time
  scale.setSampleRate(CRS_40);

  // Nuvoton recommended setting for optimal low-noise operation (0x30 in ADC_CTRL1)
  Wire.beginTransmission(0x2A);
  Wire.write(0x15); // REG_ADC_CTRL1
  Wire.endTransmission(false);
  if (Wire.requestFrom((uint8_t)0x2A, (uint8_t)1) == 1) {
    uint8_t ctrl1 = Wire.read();
    ctrl1 |= 0x30; // Set bits 4 & 5 as per Nuvoton 9.1 guidelines
    Wire.beginTransmission(0x2A);
    Wire.write(0x15);
    Wire.write(ctrl1);
    Wire.endTransmission();
  }

  // Force-enable the NAU7802 internal LDO by setting AVDDS (bit 7 of REG_PU_CTRL)
  enableInternalLDO();

  // Run internal Analog Front-End (AFE) offset calibration
  calibrateAFE();

  // Load saved calibration scale factor from flash memory
  loadSettings();

  // Always tare on startup to zero the empty scale
  performTare();

  lcd->clear();
  printHelp();
}

void loop() {
  // 1. Check Tare request & Aux buttons (D0, D1, D2)
  if (tareRequested) {
    tareRequested = false;
    performTare();
  }
  checkAuxButtons();

  // 2. Check Serial Commands
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

  // 4. Refresh LCD at exact 250ms intervals
  if (millis() - lastDisplayUpdate >= DISPLAY_INTERVAL_MS) {
    lastDisplayUpdate = millis();

    static bool tick = false;
    tick = !tick;

    int32_t netCounts = latestFilteredADC - tareOffset;
    float displayValue = isStable ? lockedWeight : filteredWeight;

    // Line 0: Weight display (1g integer resolution)
    char lineBuf[17];
    snprintf(lineBuf, sizeof(lineBuf), "Wt: %5ld g   %c", (long)lround(displayValue), tick ? '*' : ' ');
    lcd->setCursor(0, 0);
    lcd->print(lineBuf);

    // Line 1: Status & Net ADC counts or Button Press
    lcd->setCursor(0, 1);
    if (latestFilteredADC <= -8388600 || latestFilteredADC >= 8388600) {
      lcd->print("ADC SATURATED!  ");
    } else if (millis() < buttonBannerUntil) {
      char btnBuf[17];
      snprintf(btnBuf, sizeof(btnBuf), "BTN: %-11s", activeButtonsStr);
      lcd->print(btnBuf);
    } else {
      if (isStable) {
        snprintf(lineBuf, sizeof(lineBuf), "STABLE  %6ld", (long)netCounts);
      } else {
        snprintf(lineBuf, sizeof(lineBuf), "Net:    %6ld", (long)netCounts);
      }
      lcd->print(lineBuf);
    }

    // Serial stream output (1g resolution)
    if (Serial.availableForWrite() >= 64) {
      Serial.printf("Weight: %ld g | Raw: %ld | Net: %ld | %s\n", 
                    (long)lround(displayValue), (long)latestFilteredADC, (long)netCounts, isStable ? "[STABLE]" : "[MOVING]");
    }
  }
}

// ---------------------- HELPER FUNCTIONS ---------------------

/**
 * Monitors D0, D1, D2 buttons (pulled to GND when pressed).
 * Debounces with 30ms window and activates on-screen indicator.
 */
void checkAuxButtons() {
  bool anyHeld = false;
  char currentPressed[20] = "";

  for (int i = 0; i < 3; i++) {
    bool rawLow = (digitalRead(auxButtons[i].pin) == LOW);
    if (rawLow) {
      if (!auxButtons[i].isPressed) {
        if (auxButtons[i].pressStart == 0) {
          auxButtons[i].pressStart = millis();
        } else if (millis() - auxButtons[i].pressStart >= 30) {
          auxButtons[i].isPressed = true;
          buttonBannerUntil = millis() + 1500;
          Serial.printf("[BUTTON] %s Pressed (GND)!\n", auxButtons[i].name);
        }
      }
    } else {
      if (auxButtons[i].isPressed) {
        auxButtons[i].isPressed = false;
        Serial.printf("[BUTTON] %s Released.\n", auxButtons[i].name);
      }
      auxButtons[i].pressStart = 0;
    }

    if (auxButtons[i].isPressed || (rawLow && auxButtons[i].pressStart != 0)) {
      anyHeld = true;
      if (strlen(currentPressed) > 0) strcat(currentPressed, " ");
      strcat(currentPressed, auxButtons[i].name);
    }
  }

  if (anyHeld) {
    strncpy(activeButtonsStr, currentPressed, sizeof(activeButtonsStr) - 1);
    buttonBannerUntil = millis() + 1500;
  }
}

/**
 * Calculates a trimmed-mean ADC value from the rolling buffer:
 * Sorts the window, drops the lowest 2 and highest 2 outlier samples,
 * and averages the middle samples. Completely cancels chopper ripple and spikes.
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

void scanAndInitI2C() {
  Serial.println("[INFO] Scanning I2C bus...");
  bool lcdFound = false;
  uint8_t candidates[] = {0x27, 0x3F};

  for (uint8_t addr : candidates) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      lcdAddress = addr;
      lcdFound = true;
      Serial.printf("[INFO] I2C LCD detected at address: 0x%02X\n", lcdAddress);
      break;
    }
  }

  if (!lcdFound) {
    Serial.println("[WARN] No LCD found at 0x27 or 0x3F, defaulting to 0x27");
    lcdAddress = 0x27;
  }

  lcd = new LiquidCrystal_I2C(lcdAddress, 16, 2);
}

void performTare() {
  Serial.println("[ACTION] Performing Tare (Zeroing)...");
  lcd->setCursor(0, 1);
  lcd->print("TARING...       ");

  // Read 16 samples using trimmed mean
  int32_t samples[16];
  int count = 0;
  unsigned long start = millis();
  while (count < 16 && (millis() - start < 2000)) {
    int32_t val;
    if (scale.readRaw(val, 150)) {
      samples[count++] = val;
    }
    delay(5);
  }

  if (count >= 8) {
    // Sort and average middle samples
    for (int i = 1; i < count; i++) {
      int32_t key = samples[i];
      int j = i - 1;
      while (j >= 0 && samples[j] > key) {
        samples[j + 1] = samples[j];
        j--;
      }
      samples[j + 1] = key;
    }
    long sum = 0;
    int used = 0;
    for (int i = 2; i < count - 2; i++) {
      sum += samples[i];
      used++;
    }
    tareOffset = sum / used;
    scale.setTareOffset(tareOffset);
    filteredWeight = 0.0f;
    lockedWeight = 0.0f;
    isStable = true;
    saveSettings();
    Serial.printf("[SUCCESS] Tare locked. Offset = %ld\n", (long)tareOffset);
    lcd->setCursor(0, 1);
    lcd->print("TARE OK!        ");
  } else {
    Serial.println("[ERROR] Tare failed! ADC timeout.");
    lcd->setCursor(0, 1);
    lcd->print("TARE ERROR!     ");
  }
  delay(300);
}

void calibrateScaleWithWeight(float knownWeight) {
  Serial.printf("[ACTION] Calibrating with reference weight: %.2f g\n", knownWeight);
  lcd->clear();
  lcd->setCursor(0, 0);
  lcd->print("CALIBRATING...");
  lcd->setCursor(0, 1);
  char buf[17];
  snprintf(buf, sizeof(buf), "Ref: %.1fg", knownWeight);
  lcd->print(buf);

  // Take 20 samples with trimmed mean
  int32_t samples[20];
  int count = 0;
  unsigned long start = millis();
  while (count < 20 && (millis() - start < 2500)) {
    int32_t val;
    if (scale.readRaw(val, 150)) {
      samples[count++] = val;
    }
    delay(5);
  }

  if (count >= 10) {
    // Sort and average middle
    for (int i = 1; i < count; i++) {
      int32_t key = samples[i];
      int j = i - 1;
      while (j >= 0 && samples[j] > key) {
        samples[j + 1] = samples[j];
        j--;
      }
      samples[j + 1] = key;
    }
    long sum = 0;
    int used = 0;
    for (int i = 3; i < count - 3; i++) {
      sum += samples[i];
      used++;
    }
    int32_t avgVal = sum / used;
    int32_t net = avgVal - tareOffset;
    scaleFactor = (float)net / knownWeight;
    isCalibrated = true;
    scale.setScaleFactor(scaleFactor);
    filteredWeight = knownWeight;
    lockedWeight = knownWeight;
    isStable = true;
    saveSettings();
    Serial.printf("[SUCCESS] Calibration complete! Factor: %.4f counts/g (Net: %ld)\n", scaleFactor, (long)net);

    lcd->clear();
    lcd->setCursor(0, 0);
    lcd->print("CAL SUCCESS!");
    lcd->setCursor(0, 1);
    snprintf(buf, sizeof(buf), "F:%.2f c/g", scaleFactor);
    lcd->print(buf);
  } else {
    Serial.println("[ERROR] Calibration failed. Make sure weight is steady.");
    lcd->clear();
    lcd->setCursor(0, 0);
    lcd->print("CAL FAILED!");
    lcd->setCursor(0, 1);
    lcd->print("Check weight");
  }
  delay(1200);
  lcd->clear();
}

void loadSettings() {
  prefs.begin(PREFS_NAMESPACE, true);
  scaleFactor = prefs.getFloat("scale_factor", 88.0f);
  tareOffset = prefs.getInt("tare_offset", 0);
  isCalibrated = prefs.getBool("calibrated", true);
  prefs.end();

  if (scaleFactor <= 0.0f) scaleFactor = 88.0f;
  scale.setScaleFactor(scaleFactor);
  if (tareOffset != 0) {
    scale.setTareOffset(tareOffset);
  }

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
  Serial.println("  r              : Read raw ADC and scale diagnostics");
  Serial.println("  h              : Print this help menu");
  Serial.println("-----------------------");
}
