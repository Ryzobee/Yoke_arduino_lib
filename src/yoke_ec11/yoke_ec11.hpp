#pragma once

#include "../ryzobee_yoke.hpp"

#if defined(ARDUINO_ARCH_ESP32)
#include <Wire.h>
#endif

class Yoke_ec11 : public ryzobee_yoke {
public:
    static constexpr uint8_t DEFAULT_I2C_ADDRESS = 0x70;
    static constexpr uint16_t DEVICE_ID = 0x4543;

    explicit Yoke_ec11(uint8_t address = DEFAULT_I2C_ADDRESS)
        : ryzobee_yoke(kRYZOBEE_YOKE_RS, kRS_BUS_I2C),
          _address(address) {
    }

    uint16_t getDeviceId() {
        uint16_t value = 0;
        return _readInt16(REG_ID_HIGH, value) ? value : 0;
    }

    uint8_t getKey() {
        uint8_t value = 0;
        readRegister(REG_KEY, value);
        return value;
    }

    bool isPressed() {
        return (getKey() & KEY_PRESSED_MASK) != 0;
    }

    uint8_t getMissedKeyCount() {
        return getKey() & KEY_MISSED_COUNT_MASK;
    }

    // The return value is in units of 0.1 seconds.
    uint8_t getKeyPressedTime() {
        uint8_t value = 0;
        readRegister(REG_KEY_PRESSED_TIME, value);
        return value;
    }

    int16_t getCount() {
        uint16_t value = 0;
        return _readInt16(REG_COUNT_HIGH, value)
            ? static_cast<int16_t>(value)
            : 0;
    }

    // The module automatically clears DIFF after the low byte is read.
    int16_t getDiff() {
        uint16_t value = 0;
        return _readInt16(REG_DIFF_HIGH, value)
            ? static_cast<int16_t>(value)
            : 0;
    }

    bool setLED1(uint8_t r, uint8_t g, uint8_t b) {
        const uint8_t data[] = {r, g, b};
        return writeRegisters(REG_LED1_RED, data, sizeof(data));
    }

    bool setLED2(uint8_t r, uint8_t g, uint8_t b) {
        const uint8_t data[] = {r, g, b};
        return writeRegisters(REG_LED2_RED, data, sizeof(data));
    }

    bool setLED(uint8_t index, uint8_t r, uint8_t g, uint8_t b) {
        if (index == 1) {
            return setLED1(r, g, b);
        }
        if (index == 2) {
            return setLED2(r, g, b);
        }
        return false;
    }

    uint8_t getCtrl1() {
        uint8_t value = 0;
        readRegister(REG_CTRL1, value);
        return value;
    }

    bool setCtrl1(uint8_t value) {
        return writeRegister(REG_CTRL1, value);
    }

    uint8_t getCtrl2() {
        uint8_t value = 0;
        readRegister(REG_CTRL2, value);
        return value;
    }

    // The new address is written to Flash and takes effect after module reset. Valid range: 0x70-0x77.
    bool setI2CAddress(uint8_t address) {
        if (address < 0x70 || address > 0x77) {
            return false;
        }
        const uint8_t address_bits = address & 0x07;
        const uint8_t ctrl2 = 0x80 |
                              ((static_cast<uint8_t>(~address_bits) & 0x07) << 4) |
                              address_bits;
        return writeRegister(REG_CTRL2, ctrl2);
    }

    uint8_t getI2CAddress() const {
        return _address;
    }

    bool readRegister(uint8_t reg, uint8_t& value) {
        return readRegisters(reg, &value, 1);
    }

    bool readRegisters(uint8_t reg, uint8_t* data, size_t length) {
        if (!_initialized || data == nullptr || length == 0 || length > 255) {
            return false;
        }
#if defined(ARDUINO_ARCH_ESP32)
        _wire.beginTransmission(_address);
        _wire.write(reg);
        if (_wire.endTransmission(false) != 0) {
            return false;
        }

        const size_t received = _wire.requestFrom(_address, static_cast<uint8_t>(length));
        if (received != length) {
            while (_wire.available()) {
                _wire.read();
            }
            return false;
        }
        for (size_t i = 0; i < length; ++i) {
            data[i] = static_cast<uint8_t>(_wire.read());
        }
        return true;
#else
        (void)reg;
        return false;
#endif
    }

    bool writeRegister(uint8_t reg, uint8_t value) {
        return writeRegisters(reg, &value, 1);
    }

    bool writeRegisters(uint8_t reg, const uint8_t* data, size_t length) {
        if (!_initialized || data == nullptr || length == 0) {
            return false;
        }
#if defined(ARDUINO_ARCH_ESP32)
        _wire.beginTransmission(_address);
        _wire.write(reg);
        _wire.write(data, length);
        return _wire.endTransmission() == 0;
#else
        (void)reg;
        return false;
#endif
    }

protected:
    int portBegin(std::vector<uint8_t>* iodata) override {
        if (_initialized) {
            YOKE_I_PRINTF("Port is already initialized.\n");
            return YOKE_PORT_ERROR_ALREADY_INITIALIZED;
        }
        if (iodata->size() < YOKE_PORT_PIN_START + 2 ||
            iodata->at(YOKE_PORT_PIN_NUM) < 2) {
            YOKE_I_PRINTF("EC11 requires two I2C port pins.\n");
            return YOKE_PORT_ERROR_NOT_CONFIGURED;
        }
        if (_address < 0x70 || _address > 0x77) {
            YOKE_I_PRINTF("Invalid EC11 I2C address: 0x%02X.\n", _address);
            return YOKE_PORT_ERROR_NOT_CONFIGURED;
        }

#if defined(ARDUINO_ARCH_ESP32)
        const int sda = iodata->at(YOKE_PORT_PIN_START);
        const int scl = iodata->at(YOKE_PORT_PIN_START + 1);
        if (!_wire.begin(sda, scl, I2C_FREQUENCY_HZ)) {
            YOKE_I_PRINTF("EC11 I2C initialization failed.\n");
            return YOKE_PORT_ERROR_NOT_AVAILABLE;
        }

        _wire.beginTransmission(_address);
        if (_wire.endTransmission() != 0) {
            YOKE_I_PRINTF("EC11 not found at address 0x%02X.\n", _address);
            _wire.end();
            return YOKE_PORT_ERROR_NOT_AVAILABLE;
        }
#else
        return YOKE_PORT_ERROR_NOT_SUPPORTED;
#endif

        _initialized = true;
        return YOKE_PORT_ERROR_NONE;
    }

    int portEnd() override {
        if (!_initialized) {
            return YOKE_PORT_ERROR_NOT_INITIALIZED;
        }
#if defined(ARDUINO_ARCH_ESP32)
        _wire.end();
#endif
        _initialized = false;
        return YOKE_PORT_ERROR_NONE;
    }

    std::string getYokeName() override { return "Yoke EC11"; }
    std::string getYokeInfo() override { return "Yoke EC11 using RS PORT I2C"; }

private:
    enum Register : uint8_t {
        REG_ID_HIGH = 0x00,
        REG_KEY = 0x02,
        REG_KEY_PRESSED_TIME = 0x03,
        REG_COUNT_HIGH = 0x04,
        REG_DIFF_HIGH = 0x06,
        REG_LED1_RED = 0x08,
        REG_LED2_RED = 0x0B,
        REG_CTRL1 = 0x0E,
        REG_CTRL2 = 0x0F,
    };

    static constexpr uint32_t I2C_FREQUENCY_HZ = 100000;
    static constexpr uint8_t KEY_PRESSED_MASK = 0x80;
    static constexpr uint8_t KEY_MISSED_COUNT_MASK = 0x0F;

    uint8_t _address;
#if defined(ARDUINO_ARCH_ESP32)
    TwoWire _wire = TwoWire(1);
#endif

    bool _readInt16(uint8_t high_register, uint16_t& value) {
        uint8_t data[2] = {};
        if (!readRegisters(high_register, data, sizeof(data))) {
            return false;
        }
        value = (static_cast<uint16_t>(data[0]) << 8) | data[1];
        return true;
    }
};
