/**
 * ESP32-C3 SuperMini + 7Semi NAU7802 24-bit ADC Weighing Scale
 * Pluggable Display Architecture: I2C 16x2 LCD or 0.91" I2C OLED (SSD1306)
 *
 * Pin Connections:
 * ====================================================================
 * ESP32-C3 SuperMini      7Semi NAU7802      I2C LCD / OLED
 * --------------------------------------------------------------------
 * 3V3                     VIN                VCC (3V3 for OLED, 5V/VBUS for LCD)
 * GND                     GND                GND
 * GPIO 6 (SDA)            SDA                SDA
 * GPIO 7 (SCL)            SCL                SCL
 *
 * Button Configuration:
 * - On-board BOOT Button (GPIO 9) : Click = HOLD/UNHOLD, Hold 1.5s = SLEEP/WAKE
 * - External D2 Pin (GPIO 2)     : Dedicated TARE (Zero) button (pull to GND)
 * - External D4 Pin (GPIO 4)     : Dedicated UNIT cycle button (g -> kg -> oz -> lb)
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
#include "config.h"
#include "scale_display.h"
#include "scale_display_factory.h"
#include "scale_core.h"

ScaleDisplay* display = nullptr;
ScaleCore* scaleCore = nullptr;

void setup() {
  Serial.begin(115200);
#if ARDUINO_USB_CDC_ON_BOOT
  // Disable TX blocking on native USB CDC so the loop never stalls when terminal is closed
  Serial.setTxTimeoutMs(0);
#endif
  delay(500);

  Serial.println();
  Serial.println("====================================================");
  Serial.println(" ESP32-C3 SuperMini + NAU7802 Digital Scale        ");
  Serial.println(" Pluggable Display Architecture (LCD / OLED)       ");
  Serial.println("====================================================");

  // Initialize selected or auto-detected display driver
  display = createScaleDisplay(DEFAULT_DISPLAY_MODE);

  // Initialize scale core engine with the plugged display
  scaleCore = new ScaleCore(display);
  scaleCore->begin();
}

void loop() {
  if (scaleCore) {
    scaleCore->update();
  }
}
