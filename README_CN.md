# Yoke Arduino 外设库

[English](README.md)

Yoke 为 ESP32 平台提供 Ryzobee 扩展模块驱动，包括按键、灯光、电机、编码器、雷达、电容触摸和 LCD 屏幕。外设由应用独立创建并初始化，不随 RootMaker 开发板自动启用。

库信息与安装依赖见 [library.properties](library.properties)。配合 Ryzobee 使用时，由开发板库提供扩展端口的 GPIO 映射；LCD 也可直接在其他 ESP32 开发板上按实际 GPIO 接线使用。

## 支持的外设

| 模块 | 类与头文件 | 接口 | 主要功能 |
| --- | --- | --- | --- |
| KEYW | `Yoke_keyw` · [yoke_keyw.hpp](src/yoke_keyw/yoke_keyw.hpp) | RS / GPIO | 按键消抖、按下/释放事件、LED 开关与 PWM |
| RGBW | `Yoke_rgbw` · [yoke_rgbw.hpp](src/yoke_rgbw/yoke_rgbw.hpp) | RS / RMT | RGBW 灯珠数量、整体与单灯颜色控制 |
| MOTO | `Yoke_moto` · [yoke_moto.hpp](src/yoke_moto/yoke_moto.hpp) | RS / MCPWM | 一路电机正反转与调速 |
| MOTO-A | `Yoke_moto_a` · [yoke_moto_a.hpp](src/yoke_moto_a/yoke_moto_a.hpp) | F_BUS / MCPWM | 使用 F_BUS17、F_BUS19 的电机控制 |
| EC11 | `Yoke_ec11` · [yoke_ec11.hpp](src/yoke_ec11/yoke_ec11.hpp) | RS / I2C | 编码器计数、增量、按键及 RGB LED |
| RAD60 | `Yoke_rad60` · [yoke_rad60.hpp](src/yoke_rad60/yoke_rad60.hpp) | RS / UART | AT6010 雷达距离、角度、速度和目标 ID |
| TOUCH4 | `Yoke_touch4` · [yoke_touch4.hpp](src/yoke_touch4/yoke_touch4.hpp) | RS / UART | SC92F837x 四路触摸状态、原始值及配置 |
| LCD1D54C | `Yoke_lcd1d54c` · [yoke_lcd1d54c.hpp](src/yoke_lcd1d54c/yoke_lcd1d54c.hpp) | 显式 GPIO / SPI + I2C | 240×240 ST7789 显示与 CST816T 触摸 |

目标平台为 ESP32，源码使用 ESP32 核心提供的 RMT、MCPWM、UART、Wire 和 FreeRTOS 接口。部分文件包含非 ESP32 占位实现，不代表支持其他平台。此前示例编译验证使用 ESP32-S3。

## 安装与依赖

1. 在 Arduino IDE 中安装对应的 ESP32 开发板核心。当前驱动使用 ESP-IDF 5 系列接口，应使用 Arduino-ESP32 3.x 环境。
2. 将本库放入 Arduino sketchbook 的 `libraries/Yoke` 目录，确保该目录下直接包含 `library.properties` 和 `src`。
3. 安装 `library.properties` 声明的依赖：

| 依赖 | 最低版本 | 用途 |
| --- | --- | --- |
| LovyanGFX | 1.2.19 | LCD 绘图、SPI 屏幕和触摸接口 |
| AnimatedGIF | 2.2.0 | 配套 GIF 播放示例 |

Wire、FreeRTOS 和 ESP32 驱动接口由开发板核心提供，无需作为第三方库另行安装。使用 `board.maker.useport()` 的示例还需安装 Ryzobee 及其依赖；显式 GPIO 的 LCD 示例无需 Ryzobee 开发板类。

## 快速开始：KEYW

将 KEYW 接到 RootMaker 的 RS_PORT4。按键为低电平有效，驱动使用内部上拉；默认消抖时间为 30 ms。

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

`wasPressed()` 和 `wasReleased()` 消费对应的待处理事件，`isPressed()` 读取消抖状态。应频繁调用 `update()`；两次轮询之间完成的短按可能漏掉，未消费的同类事件会合并。PWM 亮度接口为 `setLEDBrightness(0~255)`。

## LCD：显式 GPIO 配置

屏幕仅支持 `begin(const Lcd1d54cPins& pins)`，没有 `begin(useport(...))` 或固定 F_BUS 针脚映射。所有字段默认 `-1`，初始化前必须设置有效且不重复的 GPIO。

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

默认使用 `Wire` 并初始化触摸 I2C 总线。复用已初始化的总线及互斥锁时，可使用 `Yoke_lcd1d54c lcd(&board.maker.getWire(), board.maker.getI2CMutex())`；先初始化开发板，再初始化 LCD，并保证共享资源比 LCD 对象存活更久。绘图接口继承自 LovyanGFX，触摸通过 `getTouch(&x, &y)` 读取。`end()` 关闭背光并释放 SPI，保留共享 I2C 总线。

## 端口与资源

除 LCD 外，上述外设通过 [ryzobee_yoke.hpp](src/ryzobee_yoke.hpp) 的端口结构接入。向量格式为 `[type, used, invalid, pinCount, pin1, pin2, ...]`。RS、S_BUS、F_BUS 是端口类型，RS 端口上的 GPIO、UART 或 I2C 用途由具体外设决定。

- 常见调用为 `device.begin(board.maker.useport("rs_port4"))`；MOTO-A 使用 `"f_bus"`。先检查端口指针，再检查初始化结果与 `isInitialized()`。不要只凭 `begin()` 返回 0 判断所有驱动均已成功，当前基类在端口无效时也可能返回 0。
- 端口数据由开发板对象持有；开发板必须比外设存活更久。仅在成功初始化后调用 `end()`。不同驱动的端口占用处理尚未完全一致，应由应用确保端口和 GPIO 不冲突。
- MOTO-A 的 F_BUS 向量按 F_BUS9 起始，并使用 F_BUS17、F_BUS19。其他设备只有采用相同布局才可直接使用；名称相同不代表接线相同。
- MOTO 和 MOTO-A 均为 20 kHz，分别使用 MCPWM 单元 0 的定时器 0 和 1。不同类可各使用一个；多个同类实例会共用固定资源。
- `setmoto(dir, speed)` 的速度输入为 0～100。当前 MOTO 默认映射上限为 255，而底层 MCPWM 接收百分比；使用 MOTO 前调用 `setSpeedMap(0, 100)`。MOTO-A 默认映射为 0～100。
- RGBW 使用 `setNumLeds()` 配置灯数并检查其返回值，再使用 `setColor()` / `setPixelColor()` 和 `show()`。
- EC11 默认地址为 0x70，使用独立 `TwoWire(1)`。增量读取会清零模块的 DIFF；地址可设置为 0x70～0x77，复位后生效。不要与其他功能争用同一 I2C 控制器。

## UART 外设说明

### RAD60

默认 115200 波特率，RS 第一个引脚作为主机 TX、第二个作为 RX，使用时应连接到模块的对应 RX、TX。构造函数选择 UART，例如 `Yoke_rad60 radar(UART_NUM_1, 115200)`。循环调用 `update()` 接收数据。

驱动解析 `TYPE=0x0A` 的 8 字节目标记录，只缓存上报顺序中的第一个目标，`getTargetCount()` 为 0 或 1。`getTargetDistance()`、`getTargetAngle()`、`getTargetVelocity()` 分别返回 cm、度、cm/s；这不会切换模块算法，也不保证跨帧跟踪同一个 ID。

### TOUCH4

固定 9600、8-N-1。`TOUCH4` 是根据固件功能命名的四通道触摸驱动，不是已确认的商业型号。使用 `readData()` 主动查询，`update()` 只接收数据。`getTouchMask()`、`isTouched(0~3)` 和 `getRawData(0~3)` 读取缓存；读取失败后缓存保留，不能当作本次有效结果。

驱动默认将端口第一个引脚用作主机 TX。已验证的 RootMaker 配套示例在初始化前交换两个端口条目，实际为 TX=GPIO18、RX=GPIO17，模块接在 RS_PORT4。请以实际接线为准，不要套用 RAD60 的收发方向。

原始值上报顺序为 `RawData[3, 2, 0, 1]`；配置接口则直接索引 `TKChannelCfg[channel]`，两者不能默认等同。配置字节语义沿用设备固件，驱动不推测其单位。`setDebugStream(&Serial)` 开启收发诊断，传入 `nullptr` 关闭；调试串口必须与设备 UART 分离。

RAD60 和 TOUCH4 的每个实例需要独占所选 UART。不要与其他实例或串口控制台共用。TOUCH4 的配置查询为同步等待，默认最多 1000 ms；同一实例的命令应串行调用。

## 配套例程

例程维护在配合库 Ryzobee 的 `example/ext_periph` 下，两个库通常并列安装在 Arduino 的 `libraries` 目录中：

| 目录 | 演示 |
| --- | --- |
| `Yoke_KEYW` | 按键事件与 LED 切换 |
| `Yoke_RGBW` | RGBW 颜色和亮度控制 |
| `Yoke_MOTO` | RS 电机控制 |
| `Yoke_MOTO_A` | F_BUS 电机控制 |
| `Yoke_RAD60` | 单目标输出 |
| `Yoke_TOUCH4` | 触摸按下/释放事件，已验证交换 TX/RX |
| `Yoke_LCD1D54C` | 红色方块与触摸交互 |
| `Yoke_LCD1D54C_GIF` | GIF 播放及板载按键切换 |

EC11 驱动目前没有对应的配套例程。各例程目录包含中英文说明。完整硬件电压、电流和引脚定义以模块资料及开发板映射为准。

## 目录结构

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

## 许可证

本项目使用 [MIT License](LICENSE)。
