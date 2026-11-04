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
 * GPIO 3 (SDA)            SDA                SDA
 * GPIO 4 (SCL)            SCL                SCL
 *
 * Button Configuration (defined in config.h):
 * - Pin 1 (HOLD_BUTTON_PIN) : Dedicated HOLD Button (Click = HOLD/UNHOLD, Default Hold 1.5s = SLEEP/WAKE)
 * - Pin 0 (TARE_BUTTON_PIN) : Dedicated TARE Button (Click = Zero)
 * - Pin 6 (UNIT_BUTTON_PIN) : Dedicated UNIT Button (Click = Cycle g -> kg -> oz -> lb)
 * - Pin 7 (PULSE_PIN)       : IP5306 Power Bank Keep-Alive Pulse Pin
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

  // Initialize hardware I2C bus before display factory probes the bus
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN, 400000);

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
