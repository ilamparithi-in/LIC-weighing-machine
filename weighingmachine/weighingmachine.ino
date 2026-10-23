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

// ---------------------- OBJECTS & STORAGE --------------------
NAU7802_7Semi scale(Wire);
LiquidCrystal_I2C *lcd = nullptr;
Preferences prefs;

// ---------------------- CONFIG & CALIBRATION -----------------
uint8_t lcdAddress = 0x27;
const char* PREFS_NAMESPACE = "scale_cfg";
float scaleFactor = 420.0f;      // Default scale factor (counts per gram)
int32_t tareOffset = 0;
bool isCalibrated = true;

// Tare button interrupt flag
volatile bool tareRequested = false;
volatile unsigned long lastTareInterruptTime = 0;

void IRAM_ATTR handleTareInterrupt() {
  unsigned long now = millis();
  if (now - lastTareInterruptTime > 300) {
    tareRequested = true;
    lastTareInterruptTime = now;
  }
}

// Sampling & Refresh rates
unsigned long lastDisplayUpdate = 0;
const unsigned long DISPLAY_INTERVAL_MS = 250;  // Refresh LCD every 250ms (4 Hz)
float filteredWeight = 0.0f;
int32_t latestRawADC = 0;
bool newSampleAvailable = false;

// ---------------------- FORWARD DECLARATIONS -----------------
void scanAndInitI2C();
void enableInternalLDO();
void calibrateAFE();
void performTare();
void calibrateScaleWithWeight(float knownWeight);
void loadSettings();
void saveSettings();
void printHelp();

void setup() {
  Serial.begin(115200);
  // CRITICAL FIX: Disable TX blocking on ESP32-C3 native USB CDC so the loop
  // never stalls when the serial monitor/terminal is closed!
  Serial.setTxTimeoutMs(0);
  delay(500);

  Serial.println();
  Serial.println("===============================================");
  Serial.println(" ESP32-C3 SuperMini + NAU7802 + I2C LCD Scale ");
  Serial.println("===============================================");

  // Initialize BOOT button as Tare with Hardware Interrupt
  pinMode(TARE_BUTTON_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(TARE_BUTTON_PIN), handleTareInterrupt, FALLING);

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

  // Set internal LDO to 3.0V and PGA to 64x (64x gives 2x the headroom of 128x to prevent rail saturation)
  scale.setConfig(LDO_3_0V, PGA_64);
  scale.setSampleRate(CRS_80);

  // Force-enable the NAU7802 internal LDO by setting AVDDS (bit 7 of REG_PU_CTRL)
  enableInternalLDO();

  // Run internal Analog Front-End (AFE) offset calibration
  calibrateAFE();

  // Load saved calibration or set reasonable default
  loadSettings();

  // Initial zero tare
  lcd->clear();
  lcd->setCursor(0, 0);
  lcd->print("Taring scale...");
  lcd->setCursor(0, 1);
  lcd->print("Keep plate empty");
  delay(600);

  performTare();

  lcd->clear();
  printHelp();
}

void loop() {
  // 1. Check Tare request (from physical BOOT button interrupt or Serial)
  if (tareRequested) {
    tareRequested = false;
    performTare();
  }

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
        Serial.println("[ERROR] Enter valid weight, e.g.: c 100");
      }
    } else if (cmd.equalsIgnoreCase("r") || cmd.equalsIgnoreCase("raw")) {
      int32_t rawVal = 0;
      if (scale.readRaw(rawVal, 100)) {
        Serial.printf("[RAW] ADC: %ld | TareOffset: %ld | Net: %ld | ScaleFactor: %.4f\n", 
                      (long)rawVal, (long)tareOffset, (long)(rawVal - tareOffset), scaleFactor);
      }
    } else if (cmd.equalsIgnoreCase("h") || cmd.equalsIgnoreCase("help")) {
      printHelp();
    }
  }

  // 3. Continuously acquire ADC data at 80 SPS
  if (scale.available()) {
    int32_t val = 0;
    if (scale.readRaw(val, 5)) {
      latestRawADC = val;
      int32_t netCounts = latestRawADC - tareOffset;
      float targetWeight = (scaleFactor != 0) ? ((float)netCounts / scaleFactor) : 0.0f;

      // Snappy filter: responsive with light jitter suppression
      float delta = abs(targetWeight - filteredWeight);
      if (delta > 0.15f) {
        filteredWeight = targetWeight; // Instant response
      } else {
        filteredWeight = (0.60f * targetWeight) + (0.40f * filteredWeight);
      }

      // Snap to 0.00g near zero
      if (abs(filteredWeight) < 0.05f) {
        filteredWeight = 0.0f;
      }
      newSampleAvailable = true;
    }
  }

  // 4. Refresh LCD at exact 250ms intervals
  if (millis() - lastDisplayUpdate >= DISPLAY_INTERVAL_MS) {
    lastDisplayUpdate = millis();

    static bool tick = false;
    tick = !tick;

    int32_t netCounts = latestRawADC - tareOffset;
    char lineBuf[17];
    // Line 0: Weight with 2 decimal places and heartbeat indicator in top right
    snprintf(lineBuf, sizeof(lineBuf), "Wt:%7.2fg  %c", filteredWeight, tick ? '*' : ' ');
    lcd->setCursor(0, 0);
    lcd->print(lineBuf);

    lcd->setCursor(0, 1);
    if (latestRawADC == -8388608 || latestRawADC == 8388607) {
      lcd->print("ADC SATURATED!  ");
    } else {
      snprintf(lineBuf, sizeof(lineBuf), "Net:%6ld [%c] ", (long)netCounts, isCalibrated ? 'R' : 'U');
      lcd->print(lineBuf);
    }

    // Only send serial if buffer is ready (never blocks!)
    if (newSampleAvailable) {
      newSampleAvailable = false;
      if (Serial.availableForWrite() >= 64) {
        Serial.printf("Weight: %.2f g | Raw: %ld | Net: %ld\n", filteredWeight, (long)latestRawADC, (long)netCounts);
      }
    }
  }
}

// ---------------------- HELPER FUNCTIONS ---------------------

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

  // Read average of 15 samples
  long sum = 0;
  int validSamples = 0;
  unsigned long start = millis();
  while (validSamples < 15 && (millis() - start < 1500)) {
    int32_t val;
    if (scale.readRaw(val, 100)) {
      sum += val;
      validSamples++;
    }
    delay(10);
  }

  if (validSamples > 0) {
    tareOffset = sum / validSamples;
    scale.setTareOffset(tareOffset);
    filteredWeight = 0.0f;
    saveSettings();
    Serial.printf("[SUCCESS] Tare locked. Offset = %ld (samples: %d)\n", (long)tareOffset, validSamples);
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

  // Take average reading with known weight
  long sum = 0;
  int validSamples = 0;
  unsigned long start = millis();
  while (validSamples < 20 && (millis() - start < 2500)) {
    int32_t val;
    if (scale.readRaw(val, 100)) {
      sum += val;
      validSamples++;
    }
    delay(10);
  }

  if (validSamples > 0) {
    int32_t avgVal = sum / validSamples;
    int32_t net = avgVal - tareOffset;
    scaleFactor = (float)net / knownWeight;
    isCalibrated = true;
    scale.setScaleFactor(scaleFactor);
    saveSettings();
    Serial.printf("[SUCCESS] Calibration complete! Factor: %.4f counts/g\n", scaleFactor);

    lcd->clear();
    lcd->setCursor(0, 0);
    lcd->print("CAL SUCCESS!");
    lcd->setCursor(0, 1);
    snprintf(buf, sizeof(buf), "F:%.1f c/g", scaleFactor);
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
  scaleFactor = prefs.getFloat("scale_factor", 420.0f);
  tareOffset = prefs.getInt("tare_offset", 0);
  isCalibrated = prefs.getBool("calibrated", true);
  prefs.end();

  if (scaleFactor == 0.0f) scaleFactor = 420.0f;
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
  Serial.println("  c <weight>     : Calibrate with known weight (e.g., 'c 100')");
  Serial.println("  r              : Read raw ADC and scale diagnostics");
  Serial.println("  h              : Print this help menu");
  Serial.println("-----------------------");
}
