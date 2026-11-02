# Yoke Arduino Peripheral Library

[中文](README_CN.md)

Yoke provides drivers for Ryzobee expansion modules on ESP32, including buttons, lighting, motors, encoders, radar, capacitive touch, and LCD displays. Applications create and initialize peripherals separately; initializing RootMaker does not automatically enable them.

See [library.properties](library.properties) for library metadata and installation dependencies. When used with Ryzobee, the board library supplies expansion port GPIO mappings. The LCD can also be wired directly to other ESP32 boards using explicit GPIO configuration.

## Supported peripherals

| Module | Class and header | Interface | Main features |
| --- | --- | --- | --- |
| KEYW | `Yoke_keyw` · [yoke_keyw.hpp](src/yoke_keyw/yoke_keyw.hpp) | RS / GPIO | Key debounce, press/release events, digital and PWM LED output |
| RGBW | `Yoke_rgbw` · [yoke_rgbw.hpp](src/yoke_rgbw/yoke_rgbw.hpp) | RS / RMT | Pixel count, whole-strip and per-pixel color control |
| MOTO | `Yoke_moto` · [yoke_moto.hpp](src/yoke_moto/yoke_moto.hpp) | RS / MCPWM | One motor with direction and speed control |
| MOTO-A | `Yoke_moto_a` · [yoke_moto_a.hpp](src/yoke_moto_a/yoke_moto_a.hpp) | F_BUS / MCPWM | Motor control through F_BUS17 and F_BUS19 |
| EC11 | `Yoke_ec11` · [yoke_ec11.hpp](src/yoke_ec11/yoke_ec11.hpp) | RS / I2C | Encoder count, delta, button, and RGB LEDs |
| RAD60 | `Yoke_rad60` · [yoke_rad60.hpp](src/yoke_rad60/yoke_rad60.hpp) | RS / UART | AT6010 radar distance, angle, velocity, and target ID |
| TOUCH4 | `Yoke_touch4` · [yoke_touch4.hpp](src/yoke_touch4/yoke_touch4.hpp) | RS / UART | SC92F837x four-channel touch state, raw values, and configuration |
| LCD1D54C | `Yoke_lcd1d54c` · [yoke_lcd1d54c.hpp](src/yoke_lcd1d54c/yoke_lcd1d54c.hpp) | Explicit GPIO / SPI + I2C | 240×240 ST7789 display and CST816T touch |

ESP32 is the target platform. The source uses the ESP32 core's RMT, MCPWM, UART, Wire, and FreeRTOS APIs. Non-ESP32 stubs in some files do not imply support for other platforms. Previous example compilation checks used ESP32-S3.

## Installation and dependencies

1. Install the ESP32 board core in Arduino IDE. The current drivers use ESP-IDF 5 APIs and should be used with Arduino-ESP32 3.x.
2. Place this library in the Arduino sketchbook's `libraries/Yoke` directory, with `library.properties` and `src` directly inside it.
3. Install the dependencies declared in `library.properties`:

| Dependency | Minimum version | Purpose |
| --- | --- | --- |
| LovyanGFX | 1.2.19 | LCD graphics, SPI display, and touch interfaces |
| AnimatedGIF | 2.2.0 | Companion GIF playback example |

Wire, FreeRTOS, and ESP32 driver APIs are included in the board core, requiring no separate third-party installation. Examples using `board.maker.useport()` also require Ryzobee and its dependencies; the explicit-GPIO LCD example does not require a Ryzobee board class.

## Quick start: KEYW

Connect KEYW to RootMaker RS_PORT4. The key is active LOW with an internal pull-up. The default debounce interval is 30 ms.

```cpp
#define BOARD_RYZOBEE_ROOTMAKER (1)

#include "Ryzobee.h"
#include <ryzobee_yoke.hpp>
#include <yoke_keyw/yoke_keyw.hpp>

Ryzobee board;
Yoke_keyw keyw(30);
bool ledOn = false;

void setup() {
    Serial.begin(115200);
    board.maker.begin();
    auto* port = board.maker.useport("rs_port4");
    if (port == nullptr || keyw.begin(port) != YOKE_PORT_ERROR_NONE) {
        Serial.println("KEYW initialization failed");
    }
}

void loop() {
    if (!keyw.isInitialized()) {
        delay(100);
        return;
    }
    keyw.update();
    if (keyw.wasPressed()) {
        ledOn = !ledOn;
        keyw.setLED(ledOn);
    }
    delay(1);
}
```

`wasPressed()` and `wasReleased()` consume pending events; `isPressed()` reads the debounced state. Call `update()` frequently: short presses completed between polls may be missed, and unread events of the same kind coalesce. Use `setLEDBrightness(0~255)` for PWM brightness.

## LCD: explicit GPIO configuration

The display supports `begin(const Lcd1d54cPins& pins)` only, without `begin(useport(...))` or fixed F_BUS pin mappings. All fields default to `-1`; configure valid, distinct GPIOs before initialization.

```cpp
#include <ryzobee_yoke.hpp>
#include <yoke_lcd1d54c/yoke_lcd1d54c.hpp>

Yoke_lcd1d54c lcd;

void setup() {
    // RootMaker wiring example; change these GPIOs for other boards.
    Lcd1d54cPins pins;
    pins.sclk = 12;
    pins.mosi = 9;
    pins.dc = 3;
    pins.cs = 5;
    pins.rst = 10;
    pins.bl = 7;
    pins.touch_sda = 41;
    pins.touch_scl = 40;
    pins.touch_rst = 2;
    pins.touch_int = 1;

    if (lcd.begin(pins) != YOKE_PORT_ERROR_NONE) return;
    lcd.setBrightness(255);
    lcd.fillScreen(TFT_BLACK);
    lcd.drawString("Yoke LCD1D54C", 20, 20);
}

void loop() {
    delay(10);
}
```

By default, the driver uses `Wire` and initializes the touch I2C bus. To reuse an initialized bus and mutex, construct `Yoke_lcd1d54c lcd(&board.maker.getWire(), board.maker.getI2CMutex())`. Initialize the board before the LCD and keep shared resources alive longer than the LCD object. Graphics methods are inherited from LovyanGFX; read touch with `getTouch(&x, &y)`. `end()` turns off the backlight and releases SPI while preserving the shared I2C bus.

## Ports and resources

Except for the LCD, the peripherals above use the port structure in [ryzobee_yoke.hpp](src/ryzobee_yoke.hpp). Its vector layout is `[type, used, invalid, pinCount, pin1, pin2, ...]`. RS, S_BUS, and F_BUS identify port types; the peripheral determines whether an RS port carries GPIO, UART, or I2C.

- A typical call is `device.begin(board.maker.useport("rs_port4"))`; MOTO-A uses `"f_bus"`. Check the port pointer, initialization result, and `isInitialized()`. Do not treat a zero `begin()` return alone as proof of success for all drivers: the current base class can also return zero for an invalid port.
- The board owns port data and must outlive the peripheral. Call `end()` only after successful initialization. Port reservation handling is not yet uniform across drivers; applications must coordinate port and GPIO use.
- MOTO-A expects an F_BUS vector starting at F_BUS9 and uses F_BUS17 and F_BUS19. Other boards need the same layout for direct compatibility; identical port names do not guarantee identical wiring.
- MOTO and MOTO-A both run at 20 kHz, using MCPWM unit 0 timers 0 and 1 respectively. One of each class can use separate timers; multiple instances of the same class share fixed resources.
- `setmoto(dir, speed)` accepts speed from 0 to 100. MOTO currently defaults to a mapping maximum of 255 while the underlying MCPWM API takes percentages; call `setSpeedMap(0, 100)` before using MOTO. MOTO-A defaults to 0-100.
- For RGBW, call `setNumLeds()` and check its result before using `setColor()` / `setPixelColor()` and `show()`.
- EC11 defaults to address 0x70 and uses its own `TwoWire(1)`. Reading the delta clears the module DIFF value. Addresses 0x70-0x77 can be configured and take effect after reset. Avoid conflicts with other users of the same I2C controller.

## UART peripherals

### RAD60

The default baud rate is 115200. The first RS pin is host TX and the second is RX; connect them to the module's RX and TX respectively. Select UART in the constructor, for example `Yoke_rad60 radar(UART_NUM_1, 115200)`, and call `update()` in the loop to receive data.

The driver parses 8-byte target records in `TYPE=0x0A` reports and caches only the first target in report order. `getTargetCount()` returns 0 or 1. `getTargetDistance()`, `getTargetAngle()`, and `getTargetVelocity()` return cm, degrees, and cm/s respectively. This does not switch the module algorithm or guarantee the same target ID across frames.

### TOUCH4

UART is fixed at 9600, 8-N-1. `TOUCH4` is a descriptive name for the firmware's four-channel touch functionality, not a confirmed commercial model. `readData()` requests data; `update()` only receives it. `getTouchMask()`, `isTouched(0~3)`, and `getRawData(0~3)` read cached values. Failed reads preserve the cache, which must not be treated as a successful new reading.

The driver defaults to the first port pin as host TX. The verified RootMaker companion example swaps the two port entries before initialization, resulting in TX=GPIO18 and RX=GPIO17 on RS_PORT4. Follow actual wiring rather than applying the RAD60 direction unchanged.

Raw values are reported in `RawData[3, 2, 0, 1]` order; configuration APIs directly index `TKChannelCfg[channel]`. Do not assume the two orders match. Configuration byte meanings are defined by device firmware; the driver does not invent units. Enable raw diagnostics with `setDebugStream(&Serial)` and disable them with `nullptr`, using a console separate from the device UART.

Each RAD60 or TOUCH4 instance needs exclusive use of its selected UART. Do not share it with another instance or a serial console. TOUCH4 configuration queries wait synchronously, up to 1000 ms by default; serialize commands on an instance.

## Companion examples

Examples are maintained in the Ryzobee companion library under `example/ext_periph`. The libraries are normally installed side by side in the Arduino `libraries` directory:

| Directory | Demonstration |
| --- | --- |
| `Yoke_KEYW` | Key events and LED toggling |
| `Yoke_RGBW` | RGBW color and brightness control |
| `Yoke_MOTO` | RS motor control |
| `Yoke_MOTO_A` | F_BUS motor control |
| `Yoke_RAD60` | Single-target output |
| `Yoke_TOUCH4` | Touch press/release events with verified swapped TX/RX |
| `Yoke_LCD1D54C` | Red square and touch interaction |
| `Yoke_LCD1D54C_GIF` | GIF playback with onboard button switching |

EC11 currently has no corresponding companion example. Each example directory includes Chinese and English documentation. Refer to module specifications and board mappings for electrical ratings and complete pin definitions.

## Directory layout

```text
Yoke/
├── library.properties
├── LICENSE
├── README.md
├── README_CN.md
└── src/
    ├── ryzobee_yoke.hpp
    ├── yoke_keyw/
    ├── yoke_rgbw/
    ├── yoke_moto/
    ├── yoke_moto_a/
    ├── yoke_ec11/
    ├── yoke_rad60/
    ├── yoke_touch4/
    └── yoke_lcd1d54c/
```

## License

This project is licensed under the [MIT License](LICENSE).
