/**
 * ESP32-C3 SuperMini + 7Semi NAU7802 24-bit ADC + I2C LCD 16x2 Weighing Scale
 *
 * Pin Connections:
 * ====================================================================
 * ESP32-C3 SuperMini      7Semi NAU7802          I2C LCD 16x2 (PCF8574)
 * --------------------------------------------------------------------
 * 3V3                     VIN                    -
 * 5V / VBUS               -                      VCC (or 3V3 if contrast allows)
 * GND                     GND                    GND
 * GPIO 4 (SDA)            SDA                    SDA
 * GPIO 5 (SCL)            SCL                    SCL
 * 
 * Note: The onboard BOOT button (GPIO 9) is used as the TARE button!
 * No external push button wiring is needed.
 *
 * Load Cell (Standard 4-Wire) to 7Semi NAU7802:
 * --------------------------------------------------------------------
 * E+ (Excitation +) -> RED
 * E- (Excitation -) -> BLACK
 * A+ (Signal +)     -> GREEN
 * A- (Signal -)     -> WHITE
 * (If weight reading goes negative when adding weight, swap A+ and A-)
 * ====================================================================
 */

#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <7Semi_NAU7802.h>
#include <Preferences.h>

// ---------------------- PIN DEFINITIONS ----------------------
#define I2C_SDA_PIN        4     // Clean I2C SDA (GPIO 4)
#define I2C_SCL_PIN        5     // Clean I2C SCL (GPIO 5)
#define TARE_BUTTON_PIN    9     // On-board BOOT Button on ESP32-C3 SuperMini (active LOW)

// ---------------------- OBJECTS & STORAGE --------------------
NAU7802_7Semi scale(Wire);
LiquidCrystal_I2C *lcd = nullptr;
Preferences prefs;

// ---------------------- CONFIG & CALIBRATION -----------------
uint8_t lcdAddress = 0x27;       // Auto-detected between 0x27 and 0x3F
const char* PREFS_NAMESPACE = "scale_cfg";
float scaleFactor = 420.0f;      // Default scale factor (counts per gram)
int32_t tareOffset = 0;
bool isCalibrated = false;

// Sampling & Smoothing
unsigned long lastDisplayUpdate = 0;
const unsigned long DISPLAY_INTERVAL_MS = 200; // Refresh LCD every 200ms
float filteredWeight = 0.0f;
const float SMOOTHING_ALPHA = 0.35f;           // EMA filter factor (0.1 = smooth, 0.9 = fast)

// ---------------------- FORWARD DECLARATIONS -----------------
void scanAndInitI2C();
void performTare();
void calibrateScaleWithWeight(float knownWeight);
void loadSettings();
void saveSettings();
void printHelp();

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("===============================================");
  Serial.println(" ESP32-C3 SuperMini + NAU7802 + I2C LCD Scale ");
  Serial.println("===============================================");

  // Initialize Tare Button
  pinMode(TARE_BUTTON_PIN, INPUT_PULLUP);

  // Initialize I2C Bus on specified ESP32-C3 pins
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
    while (1) {
      delay(1000);
    }
  }

  // Configure LDO 3.3V and Gain 128 (optimal for 4-wire strain gauge load cells)
  scale.setConfig(LDO_3_3V, PGA_128);
  scale.setSampleRate(CRS_80); // 80 SPS for smooth & responsive updates
  scale.setSampling(10, 500);

  // Load saved calibration from flash
  loadSettings();

  // Show status on LCD
  lcd->clear();
  lcd->setCursor(0, 0);
  lcd->print("Taring scale...");
  lcd->setCursor(0, 1);
  lcd->print("Keep plate empty");
  delay(1000);

  performTare();

  lcd->clear();
  printHelp();
}

void loop() {
  // 1. Check physical Tare button (active LOW with debouncing)
  static unsigned long lastBtnPress = 0;
  if (digitalRead(TARE_BUTTON_PIN) == LOW && (millis() - lastBtnPress > 400)) {
    lastBtnPress = millis();
    performTare();
  }

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
        Serial.println("[ERROR] Enter valid weight, e.g.: c 100");
      }
    } else if (cmd.equalsIgnoreCase("r") || cmd.equalsIgnoreCase("raw")) {
      int32_t rawVal = 0;
      if (scale.readRaw(rawVal, 100)) {
        Serial.printf("[RAW] ADC: %ld | Tare: %ld | ScaleFactor: %.4f\n", 
                      (long)rawVal, (long)scale.getTareOffset(), scale.getScaleFactor());
      }
    } else if (cmd.equalsIgnoreCase("h") || cmd.equalsIgnoreCase("help")) {
      printHelp();
    }
  }

  // 3. Periodic reading and LCD update
  if (millis() - lastDisplayUpdate >= DISPLAY_INTERVAL_MS) {
    lastDisplayUpdate = millis();

    float currentWeight = 0.0f;
    if (scale.getWeight(currentWeight)) {
      // Exponential Moving Average filter for clean reading display
      filteredWeight = (SMOOTHING_ALPHA * currentWeight) + ((1.0f - SMOOTHING_ALPHA) * filteredWeight);

      // Snap near zero to eliminate slight drift noise
      if (abs(filteredWeight) < 0.05f) {
        filteredWeight = 0.0f;
      }

      // Display on LCD (Line 0: Weight value, Line 1: Status)
      char lineBuf[17];
      snprintf(lineBuf, sizeof(lineBuf), "Wt: %7.1f g", filteredWeight);
      lcd->setCursor(0, 0);
      lcd->print(lineBuf);

      lcd->setCursor(0, 1);
      if (isCalibrated) {
        lcd->print("Status: READY   ");
      } else {
        lcd->print("Status: UNCAL   ");
      }

      // Serial plot output
      Serial.printf("Weight: %.2f g\n", filteredWeight);
    } else {
      lcd->setCursor(0, 0);
      lcd->print("Reading ADC...  ");
    }
  }
}

// ---------------------- HELPER FUNCTIONS ---------------------

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

  int32_t newTare = 0;
  if (scale.tare(newTare, 30, 2000)) {
    tareOffset = newTare;
    filteredWeight = 0.0f;
    saveSettings();
    Serial.printf("[SUCCESS] Tare locked. Offset = %ld\n", (long)tareOffset);
    lcd->setCursor(0, 1);
    lcd->print("TARE OK!        ");
  } else {
    Serial.println("[ERROR] Tare failed! ADC timeout or unstable signal.");
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

  if (scale.calibrateScale(knownWeight, 40, 4000)) {
    scaleFactor = scale.getScaleFactor();
    isCalibrated = true;
    saveSettings();
    Serial.printf("[SUCCESS] Calibration complete! Factor: %.4f counts/g\n", scaleFactor);

    lcd->clear();
    lcd->setCursor(0, 0);
    lcd->print("CAL SUCCESS!");
    lcd->setCursor(0, 1);
    snprintf(buf, sizeof(buf), "Factor: %.2f", scaleFactor);
    lcd->print(buf);
  } else {
    Serial.println("[ERROR] Calibration failed. Make sure weight is steady.");
    lcd->clear();
    lcd->setCursor(0, 0);
    lcd->print("CAL FAILED!");
    lcd->setCursor(0, 1);
    lcd->print("Check weight");
  }
  delay(1500);
  lcd->clear();
}

void loadSettings() {
  prefs.begin(PREFS_NAMESPACE, true); // Read-only mode
  scaleFactor = prefs.getFloat("scale_factor", 420.0f);
  tareOffset = prefs.getInt("tare_offset", 0);
  isCalibrated = prefs.getBool("calibrated", false);
  prefs.end();

  scale.setScaleFactor(scaleFactor);
  if (tareOffset != 0) {
    scale.setTareOffset(tareOffset);
  }

  Serial.printf("[CONFIG] Loaded Settings -> Factor: %.4f | Offset: %ld | Calibrated: %s\n",
                scaleFactor, (long)tareOffset, isCalibrated ? "YES" : "NO (Using default)");
}

void saveSettings() {
  prefs.begin(PREFS_NAMESPACE, false); // Read-write mode
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
  Serial.println("  c <weight>     : Calibrate with known weight (e.g., 'c 100' or 'c 500')");
  Serial.println("  r              : Read raw ADC and scale diagnostics");
  Serial.println("  h              : Print this help menu");
  Serial.println("-----------------------");
}
