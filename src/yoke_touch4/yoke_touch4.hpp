#pragma once

#include <Arduino.h>
#include "../ryzobee_yoke.hpp"
#include "../yoke_component/yoke_uart_task.hpp"
#include <cstring>

#if defined(ARDUINO_ARCH_ESP32)
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_idf_version.h"
#endif

struct Touch4Config {
    bool swap_tx_rx = true;  // Verified TOUCH4 wiring: second RS pin is host TX.
    uint8_t enabled_channels = 0x0F;
    uint32_t startup_delay_ms = 500;
    uint32_t poll_interval_ms = 100;  // Zero disables automatic polling.
    uint32_t timeout_ms = 1000;
};

// SC92F837x four-channel capacitive touch module; UART firmware from mian.c.
class Yoke_touch4 : public ryzobee_yoke {
public:
    static constexpr uint8_t CHANNEL_COUNT = 4;
    static constexpr uint32_t BAUDRATE = 9600;
    static constexpr uint32_t COMMAND_TIMEOUT_MS = 1000;

#if defined(ARDUINO_ARCH_ESP32)
    explicit Yoke_touch4(uart_port_t uart_num = UART_NUM_1)
#else
    explicit Yoke_touch4(int uart_num = 1)
#endif
        : ryzobee_yoke(kRYZOBEE_YOKE_RS, kRS_BUS_UART), _uart_num(uart_num) {
        _iodata = nullptr;
    }

    ~Yoke_touch4() { if (_initialized) end(); }

    /**
     * @brief Initialize UART, verify firmware communication, and enable channels.
     *
     * @param iodata RS port description; never modified except for its reservation flag.
     * @param use_task Run UART polling in a background task (default true).
     * @param config Wiring, startup, channel, and polling settings.
     * Initialization and explicit configuration commands remain synchronous.
     * Call begin/end from the owning task, without concurrent commands.
     * @return YOKE_PORT_ERROR_NONE on success, otherwise a port error code.
     */
    int begin(std::vector<uint8_t>* iodata, bool use_task = true,
              const Touch4Config& config = Touch4Config()) {
        yoke_detail::UartTask::Lock lock(_task);
        if (!lock) return YOKE_PORT_ERROR_NOT_AVAILABLE;
        if (_initialized) return YOKE_PORT_ERROR_ALREADY_INITIALIZED;
        if (config.enabled_channels > 0x0F || config.timeout_ms == 0) {
            return YOKE_PORT_ERROR_NOT_CONFIGURED;
        }
        _config = config;
        int result = ryzobee_yoke::begin(iodata);
        if (result != YOKE_PORT_ERROR_NONE) return result;
        if (use_task && !_task.start("yoke_touch4", _taskPoll, this)) {
            ryzobee_yoke::end();
            return YOKE_PORT_ERROR_NOT_AVAILABLE;
        }
        return YOKE_PORT_ERROR_NONE;
    }

    /**
     * @brief Manual polling when use_task=false; may block up to timeout_ms.
     * With a background task this is a no-op, so old loop code remains safe.
     */
    void update() {
        if (_task.running()) return;
        yoke_detail::UartTask::Lock lock(_task);
        if (lock) _update();
    }

    bool isConnected() const {
        yoke_detail::CacheLock lock(_cache_mutex);
        return _connected;
    }
    uint16_t getFirmwareVersion() const {
        yoke_detail::CacheLock lock(_cache_mutex);
        return _version;
    }

    /**
     * @brief Consume a latched press event; repeated presses coalesce until consumed.
     *
     * @param channel Logical channel index, 0-3.
     * @return Whether a press event was pending for this channel.
     */
    bool wasPressed(uint8_t channel) { return _consumeEvent(_pressed_events, channel); }

    /**
     * @brief Consume a latched release event; repeated releases coalesce until consumed.
     *
     * @param channel Logical channel index, 0-3.
     * @return Whether a release event was pending for this channel.
     */
    bool wasReleased(uint8_t channel) { return _consumeEvent(_released_events, channel); }

    /**
     * @brief Enable raw UART tracing on a separate console stream.
     *
     * @param stream Debug output, or nullptr to disable; must outlive tracing.
     */
    void setDebugStream(Stream* stream) {
        yoke_detail::UartTask::Lock lock(_task);
        if (lock) _debug = stream;
    }

    /**
     * @brief Request and cache all four touch states and raw values.
     *
     * @param timeout_ms Maximum response wait in milliseconds.
     * @return True on a valid response; cached data is unchanged on failure.
     */
    bool readData(uint32_t timeout_ms = COMMAND_TIMEOUT_MS) {
        yoke_detail::UartTask::Lock lock(_task);
        if (!lock) return false;
        uint8_t unused = 0;
        bool ready = _transact(0x21, &unused, 1, 0x11, 9, timeout_ms);
        if (_initialized) {
            yoke_detail::CacheLock cache_lock(_cache_mutex);
            _connected = ready;
            _last_poll_ms = millis();
        }
        return ready;
    }

    uint8_t getTouchMask() const {
        yoke_detail::CacheLock lock(_cache_mutex);
        return _touch_mask;
    }
    bool hasData() const {
        yoke_detail::CacheLock lock(_cache_mutex);
        return _has_data;
    }

    /**
     * @brief Read a cached channel touch state.
     *
     * @param channel Logical channel index, 0-3.
     * @return True if touched; false for an invalid channel or before the first read.
     */
    bool isTouched(uint8_t channel) const {
        yoke_detail::CacheLock lock(_cache_mutex);
        return channel < CHANNEL_COUNT && (_touch_mask & (1U << channel)) != 0;
    }

    /**
     * @brief Read a cached raw value in firmware report order.
     *
     * @param channel Logical channel index, 0-3.
     * @return Raw 16-bit value, or zero for an invalid channel.
     */
    uint16_t getRawData(uint8_t channel) const {
        yoke_detail::CacheLock lock(_cache_mutex);
        return channel < CHANNEL_COUNT ? _raw[channel] : 0;
    }

    /**
     * @brief Enable channels using the low four bits of a mask.
     *
     * @param mask Channel enable mask, 0x00-0x0F.
     * @return True if the returned mask matches the request.
     */
    bool setEnabledChannels(uint8_t mask) {
        yoke_detail::UartTask::Lock lock(_task);
        if (!lock) return false;
        if (mask > 0x0F) return false;
        return _transact(0x22, &mask, 1, 0x12, 1, _config.timeout_ms) && _response[0] == mask;
    }

    /**
     * @brief Write the threshold byte pairs to TKChannelCfg[0..3][6..7].
     *
     * @param data Eight firmware configuration bytes; must not be null.
     * @return True if firmware acknowledges success. Byte semantics are firmware-defined.
     */
    bool setThresholds(const uint8_t* data) {
        yoke_detail::UartTask::Lock lock(_task);
        if (!lock) return false;
        if (data == nullptr) return false;
        return _transact(0x23, data, 8, 0x13, 1, _config.timeout_ms) && _response[0] == 1;
    }

    /**
     * @brief Reinitialize the firmware touch sensor.
     *
     * @return True if firmware acknowledges success.
     */
    bool reconfigure() {
        yoke_detail::UartTask::Lock lock(_task);
        if (!lock) return false;
        uint8_t unused = 0;
        return _transact(0x24, &unused, 1, 0x14, 1, _config.timeout_ms) && _response[0] == 1;
    }

    /**
     * @brief Read the opaque configuration of one firmware channel.
     *
     * @param channel TKChannelCfg index, 0-3; not the raw report order.
     * @param data Output buffer for eight bytes; must not be null.
     * @return True on success; output is unchanged on failure.
     */
    bool getChannelConfig(uint8_t channel, uint8_t* data) {
        yoke_detail::UartTask::Lock lock(_task);
        if (!lock) return false;
        if (channel >= CHANNEL_COUNT || data == nullptr) return false;
        if (!_transact(0x25, &channel, 1, 0x15, 8, _config.timeout_ms)) return false;
        memcpy(data, _response, 8);
        return true;
    }

    /**
     * @brief Write one firmware channel configuration without reinitializing touch.
     *
     * @param channel TKChannelCfg index, 0-3.
     * @param data Eight opaque configuration bytes; must not be null.
     * @return True on the expected acknowledgement; call reconfigure() to apply changes.
     */
    bool setChannelConfig(uint8_t channel, const uint8_t* data) {
        yoke_detail::UartTask::Lock lock(_task);
        if (!lock) return false;
        if (channel >= CHANNEL_COUNT || data == nullptr) return false;
        uint8_t payload[9];
        payload[0] = channel;
        memcpy(payload + 1, data, 8);
        return _transact(0x26, payload, 9, 0x16, 1, _config.timeout_ms) &&
               _response[0] == (1 | (channel << 4));
    }

    /**
     * @brief Query the firmware version (high byte first on the wire).
     *
     * @param version Output version; must not be null.
     * @return True on success; output is unchanged on failure.
     */
    bool getVersion(uint16_t* version) {
        yoke_detail::UartTask::Lock lock(_task);
        if (!lock) return false;
        if (version == nullptr) return false;
        uint8_t unused = 0;
        if (!_transact(0x2F, &unused, 1, 0x1F, 2, _config.timeout_ms)) return false;
        *version = ((uint16_t)_response[0] << 8) | _response[1];
        return true;
    }

    std::string getYokeName() override { return "Yoke TOUCH4"; }
    std::string getYokeInfo() override { return "SC92F837x four-channel touch via UART"; }

protected:
    int portBegin(std::vector<uint8_t>* iodata) override {
        if (_initialized) return YOKE_PORT_ERROR_ALREADY_INITIALIZED;
        if (iodata->size() < YOKE_PORT_PIN_START + 2 || iodata->at(YOKE_PORT_PIN_NUM) < 2) {
            return YOKE_PORT_ERROR_NOT_CONFIGURED;
        }
#if defined(ARDUINO_ARCH_ESP32)
        int tx = iodata->at(YOKE_PORT_PIN_START + (_config.swap_tx_rx ? 1 : 0));
        int rx = iodata->at(YOKE_PORT_PIN_START + (_config.swap_tx_rx ? 0 : 1));
        if (!GPIO_IS_VALID_OUTPUT_GPIO(tx) || !GPIO_IS_VALID_GPIO(rx) || tx == rx) {
            return YOKE_PORT_ERROR_NOT_CONFIGURED;
        }
        if (_uart_num < UART_NUM_0 || _uart_num >= UART_NUM_MAX ||
            uart_is_driver_installed(_uart_num)) return YOKE_PORT_ERROR_NOT_AVAILABLE;
        uart_config_t config = {};
        config.baud_rate = BAUDRATE;
        config.data_bits = UART_DATA_8_BITS;
        config.parity = UART_PARITY_DISABLE;
        config.stop_bits = UART_STOP_BITS_1;
        config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
#if ESP_IDF_VERSION_MAJOR >= 5
        config.source_clk = UART_SCLK_DEFAULT;
#else
        config.source_clk = UART_SCLK_APB;
#endif
        esp_err_t err = uart_param_config(_uart_num, &config);
        if (err != ESP_OK) return YOKE_PORT_ERROR_NOT_AVAILABLE;
        err = uart_driver_install(_uart_num, 256, 0, 0, nullptr, 0);
        if (err != ESP_OK) return YOKE_PORT_ERROR_NOT_AVAILABLE;
        err = uart_set_pin(_uart_num, tx, rx, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
        if (err != ESP_OK) {
            uart_driver_delete(_uart_num);
            return YOKE_PORT_ERROR_NOT_AVAILABLE;
        }
        _state = WAIT_AA;
        _waiting = false;
        _received = false;
        _resetCache();
        _initialized = true;
        delay(_config.startup_delay_ms);
        uint16_t version = 0;
        if (!getVersion(&version) || !setEnabledChannels(_config.enabled_channels)) {
            portEnd();
            return YOKE_PORT_ERROR_NOT_AVAILABLE;
        }
        {
            yoke_detail::CacheLock lock(_cache_mutex);
            _version = version;
            _connected = true;
        }
        _last_poll_ms = millis();
        return YOKE_PORT_ERROR_NONE;
#else
        return YOKE_PORT_ERROR_NOT_SUPPORTED;
#endif
    }

    int portEnd() override {
        _task.stop();
        yoke_detail::UartTask::Lock lock(_task);
        if (!lock) return YOKE_PORT_ERROR_NOT_AVAILABLE;
        if (!_initialized) return YOKE_PORT_ERROR_NOT_INITIALIZED;
#if defined(ARDUINO_ARCH_ESP32)
        if (uart_driver_delete(_uart_num) != ESP_OK) return YOKE_PORT_ERROR_NOT_AVAILABLE;
#endif
        _initialized = false;
        _resetCache();
        _waiting = false;
        _state = WAIT_AA;
        return YOKE_PORT_ERROR_NONE;
    }

private:
    static void _taskPoll(void* context) {
        static_cast<Yoke_touch4*>(context)->_update();
    }

    void _update() {
        if (!_initialized) return;
        _receive();
        if (!_waiting && _config.poll_interval_ms != 0 &&
            millis() - _last_poll_ms >= _config.poll_interval_ms) {
            readData(_config.timeout_ms);
        }
    }

    void _resetCache() {
        yoke_detail::CacheLock lock(_cache_mutex);
        _connected = false;
        _version = 0;
        _pressed_events = 0;
        _released_events = 0;
        _has_data = false;
        _touch_mask = 0;
        memset(_raw, 0, sizeof(_raw));
    }

    /**
     * @brief Drain UART input without blocking and parse complete CRC-checked frames.
     */
    void _receive() {
#if defined(ARDUINO_ARCH_ESP32)
        if (!_initialized) return;
        if (_state != WAIT_AA && millis() - _last_byte_ms > FRAME_TIMEOUT_MS) {
            _state = WAIT_AA;
        }
        uint8_t byte;
        while (uart_read_bytes(_uart_num, &byte, 1, 0) == 1) {
            if (_debug != nullptr) {
                _debug->print("TOUCH4 RX: ");
                if (byte < 0x10) _debug->print('0');
                _debug->println(byte, HEX);
            }
            _processByte(byte);
            _last_byte_ms = millis();
        }
#endif
    }

    enum ParseState { WAIT_AA, WAIT_55, WAIT_CMD, WAIT_LEN, WAIT_DATA, WAIT_CRC, WAIT_END };
    static constexpr uint32_t FRAME_TIMEOUT_MS = 100;

    /**
     * @brief Update CRC8 with polynomial 0x31, init 0, no reflection or final XOR.
     *
     * @param crc Current CRC accumulator.
     * @param byte Next CMD, LEN, or DATA byte; framing bytes are excluded.
     * @return Updated CRC accumulator.
     */
    static uint8_t _crc8(uint8_t crc, uint8_t byte) {
        crc ^= byte;
        for (uint8_t i = 0; i < 8; i++) {
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x31) : (uint8_t)(crc << 1);
        }
        return crc;
    }

    bool _transact(uint8_t cmd, const uint8_t* data, uint8_t len,
                   uint8_t reply, uint8_t reply_len, uint32_t timeout_ms) {
#if defined(ARDUINO_ARCH_ESP32)
        if (!_initialized || _waiting || data == nullptr || len == 0 || len > 9) return false;
        // Firmware has no sequence number: serialize requests and drain earlier replies.
        _receive();
        _state = WAIT_AA;
        uint8_t frame[15] = {0xAA, 0x55, cmd, len};
        memcpy(frame + 4, data, len);
        uint8_t crc = 0;
        for (uint8_t i = 2; i < len + 4; i++) crc = _crc8(crc, frame[i]);
        frame[len + 4] = crc;
        frame[len + 5] = 0xEE;
        if (_debug != nullptr) {
            _debug->print("TOUCH4 TX:");
            for (uint8_t i = 0; i < len + 6; i++) {
                _debug->print(' ');
                if (frame[i] < 0x10) _debug->print('0');
                _debug->print(frame[i], HEX);
            }
            _debug->println();
        }
        _expected_cmd = reply;
        _expected_len = reply_len;
        _received = false;
        _waiting = true;
        if (uart_write_bytes(_uart_num, frame, len + 6) != len + 6) {
            _waiting = false;
            return false;
        }
        uint32_t start = millis();
        while (!_received && millis() - start < timeout_ms) {
            _receive();
            if (!_received) delay(1);
        }
        _waiting = false;
        return _received;
#else
        return false;
#endif
    }

    void _processByte(uint8_t byte) {
        switch (_state) {
            case WAIT_AA:
                if (byte == 0xAA) _state = WAIT_55;
                break;
            case WAIT_55:
                if (byte == 0x55) _state = WAIT_CMD;
                else if (byte != 0xAA) _state = WAIT_AA;
                break;
            case WAIT_CMD:
                _cmd = byte;
                _crc = _crc8(0, byte);
                _state = WAIT_LEN;
                break;
            case WAIT_LEN:
                _len = byte;
                _pos = 0;
                _crc = _crc8(_crc, byte);
                _state = (_len > 0 && _len <= sizeof(_data)) ? WAIT_DATA : WAIT_AA;
                break;
            case WAIT_DATA:
                _data[_pos++] = byte;
                _crc = _crc8(_crc, byte);
                if (_pos == _len) _state = WAIT_CRC;
                break;
            case WAIT_CRC:
                _state = (byte == _crc) ? WAIT_END : (byte == 0xAA ? WAIT_55 : WAIT_AA);
                break;
            case WAIT_END:
                _state = byte == 0xAA ? WAIT_55 : WAIT_AA;
                if (byte == 0xEE) _processFrame();
                break;
        }
    }

    void _processFrame() {
        if (_debug != nullptr) {
            _debug->print("TOUCH4 valid frame CMD=0x");
            _debug->print(_cmd, HEX);
            _debug->print(" LEN=");
            _debug->println(_len);
        }
        if (_cmd == 0x11 && _len == 9) {
            yoke_detail::CacheLock lock(_cache_mutex);
            uint8_t mask = _data[0] & 0x0F;
            uint8_t changed = mask ^ _touch_mask;
            _pressed_events |= changed & mask;
            _released_events |= changed & _touch_mask;
            _touch_mask = mask;
            for (uint8_t i = 0; i < CHANNEL_COUNT; i++) {
                _raw[i] = ((uint16_t)_data[1 + i * 2] << 8) | _data[2 + i * 2];
            }
            _has_data = true;
        }
        // An extra 0x1F response after 0x26 is valid firmware fall-through.
        if (_waiting && !_received && _cmd == _expected_cmd && _len == _expected_len) {
            memcpy(_response, _data, _len);
            _received = true;
        }
    }

#if defined(ARDUINO_ARCH_ESP32)
    uart_port_t _uart_num;
#else
    int _uart_num;
#endif
    bool _consumeEvent(uint8_t& events, uint8_t channel) {
        yoke_detail::CacheLock lock(_cache_mutex);
        if (channel >= CHANNEL_COUNT) return false;
        uint8_t bit = 1U << channel;
        bool pending = (events & bit) != 0;
        events &= static_cast<uint8_t>(~bit);
        return pending;
    }

    yoke_detail::UartTask _task;
    mutable yoke_detail::CacheMutex _cache_mutex;
    Touch4Config _config;
    uint32_t _last_poll_ms = 0;
    uint16_t _version = 0;
    uint8_t _pressed_events = 0, _released_events = 0;
    bool _connected = false;
    Stream* _debug = nullptr;
    ParseState _state = WAIT_AA;
    uint32_t _last_byte_ms = 0;
    uint8_t _cmd = 0, _len = 0, _pos = 0, _crc = 0;
    uint8_t _data[9] = {};
    uint8_t _response[9] = {};
    uint8_t _expected_cmd = 0, _expected_len = 0;
    uint8_t _touch_mask = 0;
    uint16_t _raw[CHANNEL_COUNT] = {};
    bool _has_data = false;
    bool _waiting = false;
    bool _received = false;
};
