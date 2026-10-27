# ESP32-C3 SuperMini + 7Semi NAU7802 Precision Digital Scale

A high-precision, production-grade digital weighing scale firmware running on the **ESP32-C3 SuperMini**, reading a 4-wire strain gauge load cell via the **7Semi NAU7802 24-bit ADC Breakout**, featuring a **pluggable display architecture** that seamlessly supports both **I2C 16x2 LCD** (PCF8574) and **0.91" I2C OLED** (SSD1306 128x32).

---

## 1. Hardware Architecture & Wiring

Both the **7Semi NAU7802** and the chosen display share the hardware I2C bus on the ESP32-C3 SuperMini (`GPIO 6` for SDA, `GPIO 7` for SCL).

### Connection Table

| ESP32-C3 SuperMini Pin | 7Semi NAU7802 Pin | I2C LCD 16x2 (PCF8574) | 0.91" I2C OLED (SSD1306) | Notes |
| :--- | :--- | :--- | :--- | :--- |
| **3V3** | **VIN** | - | **VCC** | Power for NAU7802 and OLED |
| **5V / VBUS** | - | **VCC** | - | Power for LCD (5V backlight) |
| **GND** | **GND** | **GND** | **GND** | Common Ground |
| **GPIO 6** | **SDA** | **SDA** | **SDA** | Hardware I2C Data line |
| **GPIO 7** | **SCL** | **SCL** | **SCL** | Hardware I2C Clock line |
| **GPIO 9 (BOOT Button)**| - | - | - | **Power / HOLD Button**:<br>- Short Press: Toggle HOLD on/off<br>- Hold 1.5s: Enter Sleep (Power Off)<br>- Press while Off: Wake up |
| **GPIO 2 (D2 Pin)**     | - | - | - | **Dedicated TARE Button** (Pull to GND) |
| **GPIO 4 (D4 Pin)**     | - | - | - | **Dedicated UNIT Button** (Cycle g/kg/oz/lb) |

> **I2C Addresses**:
> - **7Semi NAU7802 ADC**: `0x2A` (fixed by hardware)
> - **0.91" I2C OLED (SSD1306)**: `0x3C` or `0x3D` (auto-detected)
> - **16x2 I2C LCD (PCF8574)**: `0x27` or `0x3F` (auto-detected)

---

### Load Cell (4-Wire) to 7Semi NAU7802

| Load Cell Wire (Standard) | 7Semi NAU7802 Terminal | Description |
| :--- | :--- | :--- |
| **Red** | `E+` / `AVDD` | Excitation Voltage Positive |
| **Black** | `E-` / `GND` | Excitation Voltage Ground |
| **Green** | `A+` | Signal Output Positive |
| **White** | `A-` | Signal Output Negative |

*(If reading decreases when weight is placed on the platform, swap `A+` and `A-`).*

---

## 2. Pluggable Display Architecture

The firmware decouples all scale measurement and signal processing logic from display rendering using an object-oriented C++ interface (`ScaleDisplay`).

### Repository Structure

```text
weighingmachine/
├── config.h                  # Central pinout, timing, addresses, and display mode settings
├── scale_types.h             # Units of measure (g, kg, oz, lb), formatters, and math helpers
├── scale_display.h           # Pure abstract ScaleDisplay interface
├── lcd_display.h             # Concrete 16x2 LCD driver implementation
├── oled_display.h            # Concrete 0.91" SSD1306 OLED driver implementation
├── scale_display_factory.h   # Runtime I2C auto-detection & display factory
├── scale_core.h              # Unified load cell DSP, filtering, tare, calibration, and power engine
└── weighingmachine.ino       # Clean sketch entry point (setup and loop)
```

### Display Modes

In [weighingmachine/config.h](file:///home/ilam_common/DevHome/GitHub/weighingmachine/weighingmachine/config.h), set `DEFAULT_DISPLAY_MODE`:

```cpp
#define DEFAULT_DISPLAY_MODE DISPLAY_MODE_AUTO
```

- **`DISPLAY_MODE_AUTO` (Default)**: Dynamically probes the I2C bus at startup. If an OLED (`0x3C`/`0x3D`) is detected, it loads the OLED graphics engine. If an LCD (`0x27`/`0x3F`) is detected, it loads the LCD driver. **Zero code modifications required when swapping hardware!**
- **`DISPLAY_MODE_LCD`**: Explicitly locks to the I2C 16x2 LCD.
- **`DISPLAY_MODE_OLED`**: Explicitly locks to the 0.91" I2C OLED.

---

## 3. Firmware Features

* **Universal Pluggable Display**: Switch between I2C 16x2 LCD and 0.91" SSD1306 OLED with auto-detection.
* **Low Noise & Low Drift RF Suppression**: Radios (Wi-Fi and Bluetooth controllers) are turned off, and CPU clock frequency is scaled down to 80MHz to eliminate switching supply ripple and 2.4 GHz interference with the 24-bit ADC.
* **Dual-Tier Dynamic Filter**:
  * Step change (> 3.0g): Fast slew tracking (70% new sample per frame) for immediate tactile response.
  * Settling (<= 3.0g): Smooth landing (20% new sample per frame) with trimmed-mean averaging (discards highest and lowest outliers) to cancel chopper ripple and mechanical vibration.
  * Hysteresis locking: 1g integer resolution lock with 0.8g hysteresis band to eliminate digit jitter.
  * Zero-tracking deadband: Automatically snaps to exact 0 within ±1.0g.
* **Instant Unit Cycling**: Dedicated button on D4 (GPIO 4) cycles through `g` -> `kg` -> `oz` -> `lb` instantaneously with zero screen flicker or lag.
* **Quadratic Tare Animation**: Pressing Tare on D2 (GPIO 2) or via Serial animates the live reading gracefully falling to 0 with quadratic deceleration (`remain * remain`).
* **Non-Volatile Calibration**: Calibration scale factor, tare zero offset, and active unit are saved to ESP32 Flash memory via `Preferences` and restored on boot.
* **Ultra-Low Power Sleep**: BOOT button (GPIO 9) long-press (1.5s) enters light sleep (~0.13mA). Auto-shutdown automatically turns off the scale after 2 minutes of inactivity. Pressing BOOT wakes the system back up instantly.

---

## 4. Calibration & Serial Console

### Quick Calibration Steps:
1. Open the Serial Monitor at **115200 baud** (or press the Tare button on GPIO 2).
2. With the platform empty, send:
   ```text
   t
   ```
3. Place a known reference weight on the scale (e.g. 100g, 500g, or 608g).
4. Send the calibration command with the weight value in grams:
   ```text
   c 500
   ```
5. The scale computes the exact scale factor, displays `CAL SUCCESS!`, and saves the value to flash memory.

### Available Serial Commands:
| Command | Arguments | Description |
| :--- | :--- | :--- |
| `t` or `tare` | None | Zero (Tare) the scale |
| `u` or `unit` | None | Cycle active unit (`g` -> `kg` -> `oz` -> `lb`) |
| `hold` | None | Freeze / unfreeze live weight display |
| `off` or `sleep` | None | Enter ultra-low power sleep |
| `c <weight>` | Float (g) | Calibrate scale with reference weight (e.g., `c 500`) |
| `offset <val>`| Integer | Manually set raw tare offset count |
| `factor <val>`| Float | Manually set calibration factor (counts per gram) |
| `r` or `raw` | None | Inspect raw 24-bit ADC, offset, net counts, and diagnostics |
| `h` or `help` | None | Print command summary |

---

## 5. Compiling & Flashing

### Prerequisites

Install the required Arduino libraries:
* **7Semi_NAU7802_24bit_ADC_WeightScale**
* **LiquidCrystal I2C**
* **Adafruit SSD1306**
* **Adafruit GFX Library**

### Using Arduino CLI

To compile and upload the unified firmware:

```bash
# Compile
arduino-cli compile --fqbn esp32:esp32:esp32c3 weighingmachine

# Upload
arduino-cli upload -p /dev/ttyACM0 --fqbn esp32:esp32:esp32c3 weighingmachine
```

*(For ESP32-C3 SuperMini with native USB CDC, you can append `--board-options CDCOnBoot=cdc` if desired).*

### Using Arduino IDE

1. Open [weighingmachine/weighingmachine.ino](file:///home/ilam_common/DevHome/GitHub/weighingmachine/weighingmachine/weighingmachine.ino) in the Arduino IDE.
2. Select Board: **ESP32C3 Dev Module** or **ESP32-C3 SuperMini**.
3. Choose your USB port.
4. Click **Upload**.
