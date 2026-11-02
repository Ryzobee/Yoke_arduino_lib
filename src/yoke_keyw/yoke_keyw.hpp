#pragma once

#include <Arduino.h>
#include "../ryzobee_yoke.hpp"

#if defined(ARDUINO_ARCH_ESP32)
#include "driver/gpio.h"
#endif

class Yoke_keyw : public ryzobee_yoke {
public:
    explicit Yoke_keyw(uint32_t debounce_ms = 30)
        : ryzobee_yoke(kRYZOBEE_YOKE_RS, kRS_BUS_GPIO),
          _debounce_ms(debounce_ms) {
        _iodata = nullptr;
    }

    /**
     * @brief Reserve an RS port and initialize its key and LED pins.
     *
     * @param iodata Board port data; must remain valid until end().
     * @return YOKE_PORT_ERROR_NONE on success, otherwise a port error code.
     */
    int begin(std::vector<uint8_t>* iodata) {
        if (_initialized) return YOKE_PORT_ERROR_ALREADY_INITIALIZED;
        if (iodata == nullptr || iodata->size() < YOKE_PORT_PIN_START + 2) {
            return YOKE_PORT_ERROR_NOT_CONFIGURED;
        }
        if (portInvalid(iodata)) return YOKE_PORT_ERROR_INVALID;
        int result = portBegin(iodata);
        if (result == YOKE_PORT_ERROR_NONE) {
            _iodata = iodata;
            _iodata->at(YOKE_PORT_USED) = 1;
        }
        return result;
    }

    /**
     * @brief Turn off the LED, release the pins, and free the RS port.
     *
     * @return YOKE_PORT_ERROR_NONE on success, otherwise a port error code.
     */
    int end() {
        int result = portEnd();
        if (result == YOKE_PORT_ERROR_NONE) {
            _iodata->at(YOKE_PORT_USED) = 0;
            _iodata = nullptr;
        }
        return result;
    }

    /**
     * @brief Poll and debounce the active-low key; call frequently from loop().
     */
    void update() {
        if (!_initialized) return;
        uint32_t now = millis();
        bool pressed = digitalRead(_key_pin) == LOW;
        if (pressed != _raw_pressed) {
            _raw_pressed = pressed;
            _last_change_ms = now;
        }
        if (_pressed != _raw_pressed && now - _last_change_ms >= _debounce_ms) {
            _pressed = _raw_pressed;
            if (_pressed) {
                _press_event = true;
            } else {
                _release_event = true;
            }
        }
    }

    bool isPressed() const { return _initialized && _pressed; }

    /**
     * @brief Consume the pending debounced press event.
     *
     * @return True once per pending press; repeated presses may coalesce until read.
     */
    bool wasPressed() {
        bool event = _press_event;
        _press_event = false;
        return event;
    }

    /**
     * @brief Consume the pending debounced release event.
     *
     * @return True once per pending release; repeated releases may coalesce until read.
     */
    bool wasReleased() {
        bool event = _release_event;
        _release_event = false;
        return event;
    }

    /**
     * @brief Set the LED using a digital output.
     *
     * @param enabled True to turn the LED on, false to turn it off.
     */
    void setLED(bool enabled) {
        if (!_initialized) return;
#if defined(ARDUINO_ARCH_ESP32)
        if (_pwm_active) {
            ledcDetach(_led_pin);
            _pwm_active = false;
            pinMode(_led_pin, OUTPUT);
        }
#endif
        digitalWrite(_led_pin, enabled ? HIGH : LOW);
    }

    /**
     * @brief Set the LED brightness through analogWrite().
     *
     * @param brightness Brightness from 0 (off) to 255 (full brightness).
     */
    void setLEDBrightness(uint8_t brightness) {
        if (!_initialized) return;
        analogWrite(_led_pin, brightness);
        _pwm_active = true;
    }

    std::string getYokeName() override { return "Yoke KEYW"; }
    std::string getYokeInfo() override { return "KEYW key and LED via GPIO on RS PORT"; }

protected:
    int portBegin(std::vector<uint8_t>* iodata) override {
        if (_initialized) return YOKE_PORT_ERROR_ALREADY_INITIALIZED;
        if (iodata->size() < YOKE_PORT_PIN_START + 2 ||
            iodata->at(YOKE_PORT_PIN_NUM) < 2) {
            return YOKE_PORT_ERROR_NOT_CONFIGURED;
        }
#if defined(ARDUINO_ARCH_ESP32)
        int key_pin = iodata->at(YOKE_PORT_PIN_START);
        int led_pin = iodata->at(YOKE_PORT_PIN_START + 1);
        if (!GPIO_IS_VALID_GPIO(key_pin) || !GPIO_IS_VALID_OUTPUT_GPIO(led_pin) ||
            key_pin == led_pin) {
            return YOKE_PORT_ERROR_NOT_CONFIGURED;
        }
        _key_pin = key_pin;
        _led_pin = led_pin;
        pinMode(_key_pin, INPUT_PULLUP);
        pinMode(_led_pin, OUTPUT);
        digitalWrite(_led_pin, LOW);
        _pressed = false;
        _raw_pressed = digitalRead(_key_pin) == LOW;
        _last_change_ms = millis();
        _press_event = false;
        _release_event = false;
        _pwm_active = false;
        _initialized = true;
        return YOKE_PORT_ERROR_NONE;
#else
        return YOKE_PORT_ERROR_NOT_SUPPORTED;
#endif
    }

    int portEnd() override {
        if (!_initialized) return YOKE_PORT_ERROR_NOT_INITIALIZED;
        setLED(false);
        pinMode(_led_pin, INPUT);
        pinMode(_key_pin, INPUT);
        _pressed = false;
        _raw_pressed = false;
        _press_event = false;
        _release_event = false;
        _initialized = false;
        return YOKE_PORT_ERROR_NONE;
    }

private:
    int _key_pin = -1;
    int _led_pin = -1;
    uint32_t _debounce_ms;
    uint32_t _last_change_ms = 0;
    bool _raw_pressed = false;
    bool _pressed = false;
    bool _press_event = false;
    bool _release_event = false;
    bool _pwm_active = false;
};
