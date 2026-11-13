#pragma once

#include <LovyanGFX.hpp>
#include <Wire.h>
#include "driver/gpio.h"

#include "../ryzobee_yoke.hpp"
#include "Touch_CST816T.hpp"
#include "Panel_ST7789V_BOE154.hpp"
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

class Yoke_lcd1d54c : public lgfx::LGFX_Device, public ryzobee_yoke {
public:
    Yoke_lcd1d54c(TwoWire* wire = nullptr, SemaphoreHandle_t* mutex = nullptr)
        : ryzobee_yoke(kRYZOBEE_YOKE_F_BUS), _dev_i2c(wire == nullptr ? &Wire : wire), _i2c_mutex(mutex) {
        configure();
    }

    /**
     * @brief Initialize the LCD using the standard F_BUS connector wiring.
     *
     * @param iodata F_BUS description ordered by physical pins 9 through 24.
     * @return YOKE_PORT_ERROR_NONE on success, otherwise a port error code.
     */
    int begin(std::vector<uint8_t>* iodata) {
        return ryzobee_yoke::begin(iodata);
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
            setBrightness(0);
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
        if (_iodata != nullptr) {
            _iodata->at(YOKE_PORT_USED) = 0;
            _iodata = nullptr;
        }
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

protected:
    int portBegin(std::vector<uint8_t>* iodata) override {
        Lcd1d54cPins pins;
        pins.sclk = getFBusPin(iodata, LCD_SCK_F_BUS_PIN);
        pins.rst = getFBusPin(iodata, LCD_RST_F_BUS_PIN);
        pins.mosi = getFBusPin(iodata, LCD_MOSI_F_BUS_PIN);
        pins.bl = getFBusPin(iodata, LCD_BL_F_BUS_PIN);
        pins.cs = getFBusPin(iodata, LCD_CS_F_BUS_PIN);
        pins.dc = getFBusPin(iodata, LCD_DC_F_BUS_PIN);
        pins.touch_rst = getFBusPin(iodata, TP_RST_F_BUS_PIN);
        pins.touch_int = getFBusPin(iodata, TP_INT_F_BUS_PIN);
        pins.touch_sda = getFBusPin(iodata, TP_SDA_F_BUS_PIN);
        pins.touch_scl = getFBusPin(iodata, TP_SCL_F_BUS_PIN);
        return begin(pins);
    }

    int portEnd() override {
        setBrightness(0);
        releaseBus();
        _initialized = false;
        return YOKE_PORT_ERROR_NONE;
    }

private:
    // Physical connector positions, independent of the board GPIO mapping.
    enum : uint8_t {
        F_BUS_FIRST_PIN = 9,
        LCD_SCK_F_BUS_PIN = 11,
        LCD_RST_F_BUS_PIN = 13,
        LCD_MOSI_F_BUS_PIN = 14,
        LCD_BL_F_BUS_PIN = 16,
        LCD_CS_F_BUS_PIN = 18,
        LCD_DC_F_BUS_PIN = 20,
        TP_RST_F_BUS_PIN = 21,
        TP_INT_F_BUS_PIN = 22,
        TP_SDA_F_BUS_PIN = 23,
        TP_SCL_F_BUS_PIN = 24,
    };

    static int getFBusPin(const std::vector<uint8_t>* iodata, uint8_t pin) {
        return iodata->at(YOKE_PORT_PIN_START + pin - F_BUS_FIRST_PIN);
    }

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
            cfg.rgb_order = true;
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

    yoke_detail::Panel_ST7789V_BOE154 _panel_instance;
    lgfx::Bus_SPI _bus_instance;
    lgfx::Light_PWM _light_instance;
    lgfx::Touch_CST816T _touch_instance;
    TwoWire* _dev_i2c = nullptr;
    SemaphoreHandle_t* _i2c_mutex = nullptr;
    Lcd1d54cPins _pins;
};
