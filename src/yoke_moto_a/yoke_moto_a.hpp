#pragma once

#include "../ryzobee_yoke.hpp"

#if defined(ARDUINO_ARCH_ESP32)
#include "driver/gpio.h"
#include "driver/mcpwm.h"
#endif

class Yoke_moto_a : public ryzobee_yoke {
public:
    static constexpr uint8_t DIR_FORWARD = 0;
    static constexpr uint8_t DIR_REVERSE = 1;

    Yoke_moto_a(void) : ryzobee_yoke(kRYZOBEE_YOKE_F_BUS) {
    }

    // dir: DIR_FORWARD(0) forward, DIR_REVERSE(1) reverse
    // speed: 0 stopped, 100 full speed
    void setmoto(uint8_t dir, uint8_t speed) {
        if (!_initialized) {
            YOKE_I_PRINTF("Port is not initialized.\n");
            return;
        }

        if (dir != DIR_FORWARD && dir != DIR_REVERSE) {
            YOKE_I_PRINTF("Invalid motor direction: %u.\n", dir);
            _setDuty(0, 0);
            return;
        }

        speed = speed > 100 ? 100 : speed;
        float duty = static_cast<float>(
            map(speed, 0, 100, _speed_map_min, _speed_map_max));
        duty = duty < 0 ? 0 : (duty > 100 ? 100 : duty);

        if (dir == DIR_FORWARD) {
            _setDuty(duty, 0);
        } else {
            _setDuty(0, duty);
        }
    }

    void setSpeedMap(int min, int max) {
        _speed_map_min = min;
        _speed_map_max = max;
    }

protected:
    int portBegin(std::vector<uint8_t>* iodata) override {
        if (_initialized) {
            YOKE_I_PRINTF("Port is already initialized.\n");
            return YOKE_PORT_ERROR_ALREADY_INITIALIZED;
        }
        if (iodata->size() <= F_BUS19_VECTOR_INDEX ||
            iodata->at(YOKE_PORT_PIN_NUM) < F_BUS19_PIN_OFFSET + 1) {
            YOKE_I_PRINTF("MOTO-A requires F_BUS17 and F_BUS19.\n");
            return YOKE_PORT_ERROR_NOT_CONFIGURED;
        }

#if defined(ARDUINO_ARCH_ESP32)
        _pin_a = static_cast<gpio_num_t>(iodata->at(F_BUS17_VECTOR_INDEX));
        _pin_b = static_cast<gpio_num_t>(iodata->at(F_BUS19_VECTOR_INDEX));

        esp_err_t err = mcpwm_gpio_init(_unit, MCPWM1A, _pin_a);
        if (err == ESP_OK) {
            err = mcpwm_gpio_init(_unit, MCPWM1B, _pin_b);
        }

        mcpwm_config_t config = {};
        config.frequency = PWM_FREQUENCY_HZ;
        config.cmpr_a = 0;
        config.cmpr_b = 0;
        config.counter_mode = MCPWM_UP_COUNTER;
        config.duty_mode = MCPWM_DUTY_MODE_0;
        if (err == ESP_OK) {
            err = mcpwm_init(_unit, _timer, &config);
        }
        if (err != ESP_OK) {
            YOKE_I_PRINTF("MOTO-A MCPWM initialization failed: %d.\n", err);
            gpio_reset_pin(_pin_a);
            gpio_reset_pin(_pin_b);
            return YOKE_PORT_ERROR_NOT_AVAILABLE;
        }
#endif

        YOKE_I_PRINTF("MOTO-A initialized on F_BUS17/F_BUS19.\n");
        _initialized = true;
        return YOKE_PORT_ERROR_NONE;
    }

    int portEnd() override {
        if (!_initialized) {
            return YOKE_PORT_ERROR_NOT_INITIALIZED;
        }

        setmoto(DIR_FORWARD, 0);
#if defined(ARDUINO_ARCH_ESP32)
        mcpwm_stop(_unit, _timer);
        gpio_reset_pin(_pin_a);
        gpio_reset_pin(_pin_b);
#endif
        _initialized = false;
        return YOKE_PORT_ERROR_NONE;
    }

    std::string getYokeName() override { return "Yoke MOTO-A"; }
    std::string getYokeInfo() override {
        return "Yoke MOTO-A using F_BUS17/F_BUS19";
    }

private:
    // The RootMaker F-BUS vector starts at F_BUS9.
    static constexpr size_t F_BUS17_PIN_OFFSET = 17 - 9;
    static constexpr size_t F_BUS19_PIN_OFFSET = 19 - 9;
    static constexpr size_t F_BUS17_VECTOR_INDEX =
        YOKE_PORT_PIN_START + F_BUS17_PIN_OFFSET;
    static constexpr size_t F_BUS19_VECTOR_INDEX =
        YOKE_PORT_PIN_START + F_BUS19_PIN_OFFSET;
    static constexpr uint32_t PWM_FREQUENCY_HZ = 20000;

#if defined(ARDUINO_ARCH_ESP32)
    // TIMER_1 is separate from TIMER_0 used by the RS-port MOTO, allowing concurrent operation.
    mcpwm_unit_t _unit = MCPWM_UNIT_0;
    mcpwm_timer_t _timer = MCPWM_TIMER_1;
    gpio_num_t _pin_a = GPIO_NUM_NC;
    gpio_num_t _pin_b = GPIO_NUM_NC;
#endif

    int _speed_map_min = 0;
    int _speed_map_max = 100;

    void _setDuty(float duty_a, float duty_b) {
#if defined(ARDUINO_ARCH_ESP32)
        mcpwm_set_duty(_unit, _timer, MCPWM_OPR_A, duty_a);
        mcpwm_set_duty(_unit, _timer, MCPWM_OPR_B, duty_b);
        mcpwm_set_duty_type(_unit, _timer, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);
        mcpwm_set_duty_type(_unit, _timer, MCPWM_OPR_B, MCPWM_DUTY_MODE_0);
#else
        (void)duty_a;
        (void)duty_b;
#endif
    }
};
