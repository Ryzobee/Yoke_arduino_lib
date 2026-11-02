#pragma once

#include <LovyanGFX.hpp>
#include <Wire.h>
#include "driver/gpio.h"

#include "../ryzobee_yoke.hpp"
#include "Touch_CST816T.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#define YOKE_LCD_WRITE_FREQ 40000000
#define YOKE_LCD_READ_FREQ  16000000
#define YOKE_LCD_BL_FREQ    44100
#define YOKE_LCD_BL_CHANNEL 7

static constexpr uint16_t YOKE_LCD1D54C_SIZE = 240;

// All fields are actual GPIO numbers; -1 means not configured.
struct Lcd1d54cPins {
    int sclk = -1;
    int mosi = -1;
    int dc = -1;
    int cs = -1;
    int rst = -1;
    int bl = -1;
    int touch_sda = -1;
    int touch_scl = -1;
    int touch_rst = -1;
    int touch_int = -1;
};

class Yoke_lcd1d54c : public lgfx::LGFX_Device {
public:
    Yoke_lcd1d54c(TwoWire* wire = nullptr, SemaphoreHandle_t* mutex = nullptr)
        : _dev_i2c(wire == nullptr ? &Wire : wire), _i2c_mutex(mutex) {
        configure();
    }

    /**
     * @brief Initialize the display and touch controller with actual GPIOs without reserving a board port.
     *
     * @param pins Display and touch GPIO configuration; all fields must be valid and distinct.
     * @return YOKE_PORT_ERROR_NONE on success, otherwise a port error code.
     */
    int begin(const Lcd1d54cPins& pins) {
        if (_initialized) {
            return YOKE_PORT_ERROR_ALREADY_INITIALIZED;
        }
        if (!validPins(pins)) {
            return YOKE_PORT_ERROR_NOT_CONFIGURED;
        }
        _pins = pins;
        configure();
        if (!init()) {
            releaseBus();
            return YOKE_PORT_ERROR_NOT_AVAILABLE;
        }
        return YOKE_PORT_ERROR_NONE;
    }

    /**
     * @brief Turn off the backlight and release SPI, keeping the shared I2C bus active.
     *
     * @return YOKE_PORT_ERROR_NONE on success, or YOKE_PORT_ERROR_NOT_INITIALIZED.
     */
    int end() {
        if (!_initialized) {
            return YOKE_PORT_ERROR_NOT_INITIALIZED;
        }
        setBrightness(0);
        releaseBus();
        _initialized = false;
        return YOKE_PORT_ERROR_NONE;
    }

    bool isInitialized() const { return _initialized; }

    /**
     * @brief Initialize the display with configured pins; initialize the supplied TwoWire first when sharing I2C.
     *
     * @return Whether the display was initialized successfully.
     */
    bool init() {
        if (_initialized) {
            return true;
        }
        if (!validPins(_pins) || (_i2c_mutex != nullptr && *_i2c_mutex == nullptr)) {
            return false;
        }
        if (_i2c_mutex == nullptr && !_dev_i2c->begin(_pins.touch_sda, _pins.touch_scl, TP_FREQ)) {
            return false;
        }
        if (_i2c_mutex != nullptr) {
            xSemaphoreTake(*_i2c_mutex, portMAX_DELAY);
        }
        bool ready = lgfx::LGFX_Device::init();
        if (_i2c_mutex != nullptr) {
            xSemaphoreGive(*_i2c_mutex);
        }
        _initialized = ready;
        return ready;
    }

    std::string getYokeName() { return "Yoke LCD1D54C"; }
    std::string getYokeInfo() { return "240x240 ST7789 LCD with CST816T touch"; }

    /**
     * @brief Clear the screen to black.
     */
    void clear() {
        fillScreen(TFT_BLACK);
    }

    /**
     * @brief Read single-touch coordinates transformed to match the display rotation.
     *
     * @param x Output horizontal coordinate; must not be null.
     * @param y Output vertical coordinate; must not be null.
     * @return Whether a touch was detected.
     */
    bool getTouch(uint16_t* x, uint16_t* y) {
        if (x == nullptr || y == nullptr || !_initialized ||
            (_i2c_mutex != nullptr && *_i2c_mutex == nullptr)) {
            return false;
        }

        bool touched = false;
        if (_i2c_mutex == nullptr || xSemaphoreTake(*_i2c_mutex, portMAX_DELAY) == pdTRUE) {
            lgfx::touch_point_t point;
            int count = lgfx::LGFX_Device::getTouch(&point, 1);
            if (count > 0) {
                *x = point.x;
                *y = point.y;
                touched = true;
            }
            if (_i2c_mutex != nullptr) {
                xSemaphoreGive(*_i2c_mutex);
            }
        }
        return touched;
    }

private:
    /**
     * @brief Check GPIO capabilities and duplicate assignments before accessing hardware.
     *
     * @param pins Actual GPIO configuration to validate.
     * @return True if all output pins and the touch input pin are valid and distinct.
     */
    static bool validPins(const Lcd1d54cPins& pins) {
        const int outputs[] = {pins.sclk, pins.mosi, pins.dc, pins.cs, pins.rst,
                               pins.bl, pins.touch_sda, pins.touch_scl, pins.touch_rst};
        for (int pin : outputs) {
            if (!GPIO_IS_VALID_OUTPUT_GPIO(pin) || pin == pins.touch_int) {
                return false;
            }
        }
        if (!GPIO_IS_VALID_GPIO(pins.touch_int)) {
            return false;
        }
        for (size_t i = 0; i < sizeof(outputs) / sizeof(outputs[0]); ++i) {
            for (size_t j = i + 1; j < sizeof(outputs) / sizeof(outputs[0]); ++j) {
                if (outputs[i] == outputs[j]) {
                    return false;
                }
            }
        }
        return true;
    }

    /**
     * @brief Configure the ST7789 display, backlight PWM, and CST816T touch controller.
     */
    void configure() {
        {
            auto cfg = _bus_instance.config();

            cfg.spi_mode = 0;
            cfg.freq_write = YOKE_LCD_WRITE_FREQ;
            cfg.freq_read = YOKE_LCD_READ_FREQ;
            cfg.spi_3wire = true;
            cfg.use_lock = true;
            cfg.dma_channel = SPI_DMA_CH_AUTO;
            cfg.pin_sclk = _pins.sclk;
            cfg.pin_mosi = _pins.mosi;
            cfg.pin_miso = -1;
            cfg.pin_dc = _pins.dc;

            _bus_instance.config(cfg);
            _panel_instance.setBus(&_bus_instance);
        }

        {
            auto cfg = _panel_instance.config();

            cfg.pin_cs = _pins.cs;
            cfg.pin_rst = _pins.rst;
            cfg.pin_busy = -1;
            cfg.panel_width = YOKE_LCD1D54C_SIZE;
            cfg.panel_height = YOKE_LCD1D54C_SIZE;
            cfg.offset_x = 0;
            cfg.offset_y = 0;
            cfg.offset_rotation = 0;
            cfg.dummy_read_pixel = 8;
            cfg.dummy_read_bits = 1;
            cfg.readable = true;
            cfg.invert = true;
            cfg.rgb_order = false;
            cfg.dlen_16bit = false;
            cfg.bus_shared = true;

            _panel_instance.config(cfg);
        }

        {
            auto cfg = _light_instance.config();

            cfg.pin_bl = _pins.bl;
            cfg.invert = false;
            cfg.freq = YOKE_LCD_BL_FREQ;
            cfg.pwm_channel = YOKE_LCD_BL_CHANNEL;

            _light_instance.config(cfg);
            _panel_instance.setLight(&_light_instance);
        }

        {
            auto cfg = _touch_instance.config();

            cfg.x_min = 0;
            cfg.x_max = YOKE_LCD1D54C_SIZE;
            cfg.y_min = 0;
            cfg.y_max = YOKE_LCD1D54C_SIZE;
            cfg.pin_int = _pins.touch_int;
            cfg.pin_rst = _pins.touch_rst;
            cfg.bus_shared = true;
            cfg.offset_rotation = 0;
            cfg.i2c_port = 0;
            cfg.i2c_addr = TP_ADDR;
            cfg.pin_sda = _pins.touch_sda;
            cfg.pin_scl = _pins.touch_scl;
            cfg.freq = TP_FREQ;

            _touch_instance.config(cfg);
            _touch_instance.setWire(_dev_i2c);
            _panel_instance.setTouch(&_touch_instance);
        }

        setPanel(&_panel_instance);
    }

    lgfx::Panel_ST7789 _panel_instance;
    lgfx::Bus_SPI _bus_instance;
    lgfx::Light_PWM _light_instance;
    lgfx::Touch_CST816T _touch_instance;
    TwoWire* _dev_i2c = nullptr;
    SemaphoreHandle_t* _i2c_mutex = nullptr;
    Lcd1d54cPins _pins;
    bool _initialized = false;
};
