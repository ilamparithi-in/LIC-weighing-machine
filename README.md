# ESP32-C3 SuperMini + 7Semi NAU7802 + I2C 16x2 LCD Weighing Scale

A digital weighing scale firmware running on an **ESP32-C3 SuperMini**, reading a 4-wire strain gauge load cell via the **7Semi NAU7802 24-bit ADC Breakout**, and displaying continuous filtered weight measurements on an **I2C 16x2 LCD Display**.

---

## 1. Hardware Architecture & Wiring

Both the **7Semi NAU7802** and the **16x2 LCD Display (PCF8574 backpack)** share the hardware I2C bus on the ESP32-C3 SuperMini.

### Connection Table

| ESP32-C3 SuperMini Pin | 7Semi NAU7802 Pin | I2C LCD 16x2 (PCF8574) | Notes |
| :--- | :--- | :--- | :--- |
| **3V3** | **VIN** | - | Power for NAU7802 |
| **5V / VBUS** | - | **VCC** (or 3V3 if contrast allows) | Power for LCD |
| **GND** | **GND** | **GND** | Common Ground |
| **GPIO 4** | **SDA** | **SDA** | I2C Data line |
| **GPIO 5** | **SCL** | **SCL** | I2C Clock line |
| **GPIO 9 (BOOT Button)**| - | - | **Power / HOLD Button**:<br>- 1-Click: HOLD/UNHOLD weight<br>- Hold 1.5s: Power Off (Sleep)<br>- Press: Power On (Wake) |
| **GPIO 2 (D2 Pin)**     | - | - | **Dedicated TARE Button** (Pull to GND) |
| **GPIO 4 (D4 Pin)**     | - | - | **Dedicated UNIT Button** (Cycle g/kg/oz/lb) |

> **Note on I2C Addresses**:
> - **7Semi NAU7802**: `0x2A` (fixed by hardware)
> - **16x2 LCD**: `0x27` or `0x3F` (auto-detected by the firmware at startup)

---

### Load Cell (4-Wire) to 7Semi NAU7802

| Load Cell Wire (Standard) | 7Semi NAU7802 Terminal | Description |
| :--- | :--- | :--- |
| **Red** | `E+` / `AVDD` | Excitation Voltage Positive |
| **Black** | `E-` / `GND` | Excitation Voltage Ground |
| **Green** | `A+` | Signal Output Positive |
| **White** | `A-` | Signal Output Negative |

*(If reading decreases when weight is placed, swap `A+` and `A-`).*

---

## 2. Firmware Features

* **Auto-detect I2C LCD**: Scans and binds to either `0x27` or `0x3F` automatically.
* **Persistent Calibration in Flash**: Calibration scale factor and zero offset are saved using ESP32 `Preferences` non-volatile storage, so calibration persists across power cycles and reboots.
* **Dual Tare Trigger**: Tare via physical push button (GPIO 2) or via Serial command (`t`).
* **Noise Filter**: Exponential Moving Average (EMA) and low-noise snap threshold to prevent LCD flicker and jitter.
* **Interactive Serial Console**: Calibrate, zero, or inspect raw 24-bit ADC counts anytime at 115200 baud.

---

## 3. How to Calibrate

### Quick Calibration Steps:
1. Open the Serial Monitor at **115200 baud** (or press the Tare button on GPIO 2).
2. With the scale platform empty, send:
   ```text
   t
   ```
   *(The LCD will show `TARING...` then `TARE OK!`)*
3. Place a known reference weight on the scale (e.g. 100 grams or 500 grams).
4. Send the calibration command with the weight value in grams:
   ```text
   c 100
   ```
   *(Or `c 500` if using 500g)*
5. The scale calculates the calibration factor, displays `CAL SUCCESS!`, and saves it directly to flash. Subsequent reboots will load this calibration automatically!

### Available Serial Commands:
- `t` or `tare` : Zero (Tare) the scale.
- `c <weight>` : Calibrate using known weight (e.g., `c 200`).
- `r` or `raw`  : Output raw ADC values and calibration diagnostics.
- `h` or `help` : Display the command list.

---

## 4. Compiling & Flashing

Using **Arduino CLI**:

* For the **I2C 16x2 LCD** version:
```bash
arduino-cli compile --fqbn esp32:esp32:nologo_esp32c3_super_mini weighingmachine
arduino-cli upload -p /dev/ttyACM0 --fqbn esp32:esp32:nologo_esp32c3_super_mini weighingmachine
```

* For the **0.91" I2C OLED (SSD1306 128x32)** version:
```bash
arduino-cli compile --fqbn esp32:esp32:nologo_esp32c3_super_mini weighingmachine_oled
arduino-cli upload -p /dev/ttyACM0 --fqbn esp32:esp32:nologo_esp32c3_super_mini weighingmachine_oled
```

Or open [weighingmachine/weighingmachine.ino](file:///home/ilam_common/DevHome/GitHub/weighingmachine/weighingmachine/weighingmachine.ino) or [weighingmachine_oled/weighingmachine_oled.ino](file:///home/ilam_common/DevHome/GitHub/weighingmachine/weighingmachine_oled/weighingmachine_oled.ino) directly in the Arduino IDE, select board **ESP32C3 Dev Module** or **ESP32-C3 SuperMini**, and upload!

