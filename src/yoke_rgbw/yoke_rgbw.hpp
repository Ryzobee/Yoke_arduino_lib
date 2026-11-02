#pragma once 

#include "../ryzobee_yoke.hpp"

#if defined(ARDUINO_ARCH_ESP32)
#include "driver/rmt_tx.h"
#include <cstdlib>
#include <cstring>

class Yoke_rgbw : public ryzobee_yoke {

public:
    Yoke_rgbw(void) : ryzobee_yoke(kRYZOBEE_YOKE_RS) {
    }

    ~Yoke_rgbw() {
        _freeBuffers();
    }

    // ==== Set number of NeoPixels (call after begin, before setPixelColor) ====
    // Dynamically allocates color and RMT symbol buffers.
    // Returns false on allocation failure, true on success.
    bool setNumLeds(int count) {
        if (count < 1) count = 1;
        _freeBuffers();
        _num_leds = count;
        _led_colors = (uint32_t*)calloc(_num_leds, sizeof(uint32_t));
        _rmt_items  = (rmt_symbol_word_t*)calloc(_num_leds * 32, sizeof(rmt_symbol_word_t));
        if (!_led_colors || !_rmt_items) {
            _freeBuffers();
            _num_leds = 1;
            return false;
        }
        return true;
    }

    int getNumLeds() const { return _num_leds; }

    // ==== All LEDs same color (backward compatible) ====
    void setColor(uint8_t r, uint8_t g, uint8_t b, uint8_t w = 0) {
        if (!_initialized) return;
        for (int i = 0; i < _num_leds; i++) {
            _led_colors[i] = _packGRBW(r, g, b, w);
        }
    }

    void setColor(uint32_t color) {
        if (!_initialized) return;
        for (int i = 0; i < _num_leds; i++) {
            _led_colors[i] = color;
        }
    }

    // ==== Per-pixel color ====
    void setPixelColor(int index, uint8_t r, uint8_t g, uint8_t b, uint8_t w = 0) {
        if (!_initialized || index < 0 || index >= _num_leds) return;
        _led_colors[index] = _packGRBW(r, g, b, w);
    }

    void setPixelColor(int index, uint32_t color) {
        if (!_initialized || index < 0 || index >= _num_leds) return;
        _led_colors[index] = color;
    }

    // ==== Transmit all LED data to the strip ====
    void show() {
        if (!_initialized || !_rmt_items) return;
        _buildAllItems();
        rmt_transmit_config_t tx_config = {};
        tx_config.loop_count = 0;
        rmt_transmit(_tx_chan, _copy_encoder, _rmt_items,
                     _num_leds * 32 * sizeof(rmt_symbol_word_t), &tx_config);
        rmt_tx_wait_all_done(_tx_chan, portMAX_DELAY);
    }

protected:
    int portBegin(std::vector<uint8_t>* iodata) override {
        if (_initialized) {
            YOKE_I_PRINTF("Port is already initialized.\n");
            return YOKE_PORT_ERROR_ALREADY_INITIALIZED;
        }
        gpio_num_t gpio = (gpio_num_t)iodata->at(YOKE_PORT_PIN_START);

        // ==== New RMT TX driver (ESP-IDF v5.x) ====
        rmt_tx_channel_config_t tx_chan_config = {};
        tx_chan_config.gpio_num = gpio;
        tx_chan_config.clk_src = RMT_CLK_SRC_DEFAULT;
        tx_chan_config.resolution_hz = 20 * 1000 * 1000;  // 20MHz → 50ns/tick
        tx_chan_config.mem_block_symbols = 64;
        tx_chan_config.trans_queue_depth = 1;

        esp_err_t err = rmt_new_tx_channel(&tx_chan_config, &_tx_chan);
        if (err != ESP_OK) {
            YOKE_I_PRINTF("rmt_new_tx_channel failed: %d\n", err);
            return YOKE_PORT_ERROR_NOT_AVAILABLE;
        }

        rmt_copy_encoder_config_t copy_config = {};
        err = rmt_new_copy_encoder(&copy_config, &_copy_encoder);
        if (err != ESP_OK) {
            YOKE_I_PRINTF("rmt_new_copy_encoder failed: %d\n", err);
            rmt_del_channel(_tx_chan);
            _tx_chan = NULL;
            return YOKE_PORT_ERROR_NOT_AVAILABLE;
        }

        err = rmt_enable(_tx_chan);
        if (err != ESP_OK) {
            YOKE_I_PRINTF("rmt_enable failed: %d\n", err);
            rmt_del_encoder(_copy_encoder);
            rmt_del_channel(_tx_chan);
            _copy_encoder = NULL;
            _tx_chan = NULL;
            return YOKE_PORT_ERROR_NOT_AVAILABLE;
        }

        // Default: 1 LED
        setNumLeds(1);

        YOKE_I_PRINTF("RGBW RMT TX ready on GPIO%d\n", gpio);
        _initialized = true;
        return YOKE_PORT_ERROR_NONE;
    }

    int portEnd() override {
        _initialized = false;
        _freeBuffers();
        if (_tx_chan) {
            rmt_disable(_tx_chan);
        }
        if (_copy_encoder) {
            rmt_del_encoder(_copy_encoder);
            _copy_encoder = NULL;
        }
        if (_tx_chan) {
            rmt_del_channel(_tx_chan);
            _tx_chan = NULL;
        }
        return YOKE_PORT_ERROR_NONE;
    }

    std::string getYokeName() override { return "Yoke RGBW"; }
    std::string getYokeInfo() override { return "Yoke RGBW using RS PORT"; }

private:
    rmt_channel_handle_t _tx_chan = NULL;
    rmt_encoder_handle_t _copy_encoder = NULL;

    int _num_leds = 1;
    uint32_t* _led_colors = nullptr;       // GRBW packed color per LED
    rmt_symbol_word_t* _rmt_items = nullptr; // RMT symbols for all LEDs

    void _freeBuffers() {
        if (_led_colors) { free(_led_colors); _led_colors = nullptr; }
        if (_rmt_items)  { free(_rmt_items);  _rmt_items  = nullptr; }
    }

    static uint32_t _packGRBW(uint8_t r, uint8_t g, uint8_t b, uint8_t w) {
        return ((uint32_t)g << 24) | ((uint32_t)r << 16) | ((uint32_t)b << 8) | w;
    }

    // Build RMT symbols for ALL LEDs from _led_colors[]
    void _buildAllItems() {
        for (int led = 0; led < _num_leds; led++) {
            uint32_t grbw = _led_colors[led];
            rmt_symbol_word_t* items = &_rmt_items[led * 32];
            for (int i = 0; i < 32; i++) {
                bool bit = grbw & (1 << (31 - i));
                if (bit) {
                    items[i].level0 = 1; items[i].duration0 = 14;
                    items[i].level1 = 0; items[i].duration1 = 10;
                } else {
                    items[i].level0 = 1; items[i].duration0 = 6;
                    items[i].level1 = 0; items[i].duration1 = 18;
                }
            }
        }
    }
};

#else
// Non-ESP32 stub
class Yoke_rgbw : public ryzobee_yoke {
public:
    Yoke_rgbw(void) : ryzobee_yoke(kRYZOBEE_YOKE_RS) {}
    bool setNumLeds(int count) { return false; }
    int getNumLeds() const { return 0; }
    void setColor(uint8_t r, uint8_t g, uint8_t b, uint8_t w = 0) {}
    void setColor(uint32_t color) {}
    void setPixelColor(int index, uint8_t r, uint8_t g, uint8_t b, uint8_t w = 0) {}
    void setPixelColor(int index, uint32_t color) {}
    void show() {}
protected:
    int portBegin(std::vector<uint8_t>* iodata) override { return YOKE_PORT_ERROR_NOT_SUPPORTED; }
    int portEnd() override { return YOKE_PORT_ERROR_NONE; }
    std::string getYokeName() override { return "Yoke RGBW"; }
    std::string getYokeInfo() override { return "Yoke RGBW"; }
};
#endif
