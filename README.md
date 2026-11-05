# ESP32-C3 SuperMini + 7Semi NAU7802 Precision Digital Scale

A high-precision, production-grade digital weighing scale firmware running on the **ESP32-C3 SuperMini**, reading a 4-wire strain gauge load cell via the **7Semi NAU7802 24-bit ADC Breakout**, featuring a **pluggable display architecture** that seamlessly supports both **I2C 16x2 LCD** (PCF8574) and **0.91" I2C OLED** (SSD1306 128x32).

---

## 1. Hardware Architecture & Wiring

Both the **7Semi NAU7802** and the chosen display share the I2C bus on the ESP32-C3 SuperMini (`GPIO 3` for SDA, `GPIO 4` for SCL).

### Connection Table

| ESP32-C3 SuperMini Pin | 7Semi NAU7802 Pin | I2C LCD 16x2 (PCF8574) | 0.91" I2C OLED (SSD1306) | Notes |
| :--- | :--- | :--- | :--- | :--- |
| **3V3** | **VIN** | - | **VCC** | Power for NAU7802 and OLED |
| **5V / VBUS** | - | **VCC** | - | Power for LCD (5V backlight) |
| **GND** | **GND** | **GND** | **GND** | Common Ground |
| **GPIO 3** | **SDA** | **SDA** | **SDA** | I2C Data line |
| **GPIO 4** | **SCL** | **SCL** | **SCL** | I2C Clock line |
| **GPIO 1** | - | - | - | **Dedicated HOLD Button** (Leftmost):<br>- Short Press: Toggle HOLD on/off<br>- Hold 1.5s: Enter Sleep (Power Off, configurable in config.h)<br>- Press while Off: Wake up<br>- In Sample Wizard: **`[ESC]`** Cancel / Exit |
| **GPIO 0** | - | - | - | **Dedicated TARE Button** (Middle-Left):<br>- Short Press: Zero / Tare scale<br>- In Sample Wizard: **`[-]`** Decrement sample count |
| **GPIO 6** | - | - | - | **Dedicated UNIT Button** (Middle-Right):<br>- Short Press: Cycle weight unit (g/kg/oz/lb)<br>- In Sample Wizard: **`[+]`** Increment sample count |
| **GPIO 5** | - | - | - | **Dedicated PCS Button** (Rightmost):<br>- Short Press: Toggle Weighing $\leftrightarrow$ PCS Counting mode<br>- Hold 1.0s: Open Sample Calibration Wizard<br>- In Sample Wizard: **`[OK]`** Confirm reference sample & save APW |
| **GPIO 7** | - | - | - | **Power Bank Keep-Alive** (Pulses IP5306 keypad every 10s) |

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

*(If reading decreases/goes negative when weight is placed on the platform, swap `A+` and `A-`).*

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

- **`DISPLAY_MODE_AUTO` (Default)**: Dynamically probes the I2C bus at startup. If an OLED (`0x3C`/`0x3D`) is detected, it loads the OLED graphics engine. If an LCD (`0x27`/`0x3F`) is detected, it loads the LCD driver. 
- **`DISPLAY_MODE_LCD`**: I2C 16x2 LCD with PCF8574 backpack.
- **`DISPLAY_MODE_OLED`**: I2C 0.91" OLED.

---

## 3. Firmware Features

* **Universal Pluggable Display**: Switch between I2C 16x2 LCD and 0.91" SSD1306 OLED with auto-detection.
* **Low Noise & Low Drift RF Suppression**: Radios (Wi-Fi and Bluetooth controllers) are turned off, and CPU clock frequency is scaled down to 80MHz to eliminate switching supply ripple and 2.4 GHz interference with the 24-bit ADC.
* **Dual-Tier Dynamic Filter**:
  * Step change (> 3.0g): Fast slew tracking (70% new sample per frame) for immediate tactile response.
  * Settling (<= 3.0g): Smooth landing (20% new sample per frame) with trimmed-mean averaging (discards highest and lowest outliers) to cancel chopper ripple and mechanical vibration.
  * Hysteresis locking: 1g integer resolution lock with 0.8g hysteresis band to eliminate digit jitter.
  * Zero-tracking deadband: Automatically snaps to exact 0 within ±1.0g.
* **Instant Unit Cycling**: Dedicated button on GPIO 6 cycles through `g` -> `kg` -> `oz` -> `lb` instantaneously with zero screen flicker or lag.
* **PCS (Piece Counting System)**:
  * Dedicated button on GPIO 5 toggles instantly between standard weighing and piece counting mode.
  * Long-pressing GPIO 5 (>= 1.0s) triggers an interactive **Reference Sample Wizard** with continuously variable sample count (1 to 9999 pcs).
  * Smooth auto-repeat with 4-stage adaptive acceleration curve on long-press for rapid, precise value adjustments.
  * Momentarily rebinds the 4 physical buttons (left-to-right GPIO 1, 0, 6, 5) to an arrow navigation interface (`[ESC]`, `[-]`, `[+]`, `[OK]`).
  * Validates minimum total sample weight (>= 0.20g) and piece resolution (>= 0.005g) to prevent inaccurate piece counting.
  * Optimized display formatting for both 0.91" 128x32 OLED (large Size 3 digits with fixed right `pcs` badge) and 16x2 LCD (`Qty: <count> pcs`).
* **Quadratic Tare Animation**: Pressing Tare on GPIO 0 or via Serial animates the live reading gracefully falling to 0 with quadratic deceleration (`remain * remain`). Routine zeroing is kept in RAM to protect flash memory from sector wear.
* **Non-Volatile Calibration & APW Storage**: Calibration scale factor, zero offset, active unit, average piece weight (APW), reference sample count, and operating mode are persisted in ESP32 Flash memory via `Preferences` and restored on boot.
* **Ultra-Low Power Sleep**: Dedicated power button (GPIO 1 by default, assignable in `config.h`) long-press (1.5s) enters light sleep (~0.13mA). Auto-shutdown automatically turns off the scale after 2 minutes of inactivity (can be disabled in `config.h`). Pressing any button wakes the system back up instantly.

---

## 4. Calibration & Serial Console

### Quick Calibration Steps:
1. Open the Serial Monitor at **115200 baud** (or press the Tare button on GPIO 0).
2. With the platform empty, send:
   ```text
   t
   ```
3. Place a known reference weight on the scale (e.g. 100g, 500g, or 1kg).
4. Send the calibration command with the weight value **in grams**:
   ```text
   c 500
   ```
5. The scale computes the exact scale factor, displays `CAL SUCCESS!`, and saves the value to flash memory.

### Available Serial Commands:
| Command | Arguments | Description |
| :--- | :--- | :--- |
| `t` or `tare` | None | Zero (Tare) the scale |
| `u` or `unit` | None | Cycle active unit (`g` -> `kg` -> `oz` -> `lb`) |
| `pcs` | None | Toggle / enter PCS piece counting mode |
| `sample <count>` | Integer | Calibrate APW with current load as `<count>` pieces (e.g., `sample 25`) |
| `apw <weight>` | Float (g) | Manually set Average Piece Weight in grams (e.g., `apw 0.254`) |
| `apw` | None | Query current APW, reference sample, and piece count diagnostics |
| `hold` | None | Freeze / unfreeze live weight or piece count display |
| `off` or `sleep` | None | Enter ultra-low power sleep |
| `c <weight>` | Float (g) | Calibrate scale with reference weight (e.g., `c 500`) |
| `cp <weight>`| Float (g) | Calibration preview (calculate scale factor without saving) |
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

1. Open weighingmachine/weighingmachine.ino in the Arduino IDE.
2. Select Board: **ESP32C3 Dev Module** or **Nologo ESP32-C3 Super Mini**.
3. Choose your USB port.
4. Click **Upload**.
