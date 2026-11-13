#pragma once

#include <Arduino.h>
#include "../ryzobee_yoke.hpp"
#include "../yoke_component/yoke_uart_task.hpp"

#if defined(ARDUINO_ARCH_ESP32)
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_idf_version.h"
#include <cstdlib>
#include <cstring>

// =============================================================================
//   Default configuration
// =============================================================================

#define RAD60_DEFAULT_BAUDRATE    115200
#define RAD60_CMD_TIMEOUT_MS      1000
#define RAD60_CMD_RETRY_COUNT     3
#define RAD60_QUERY_INTERVAL_MS   600
#define RAD60_NO_DATA_TIMEOUT_MS  500
#define RAD60_COMM_TIMEOUT_MS     1500

// =============================================================================
//   Command enumerations
// =============================================================================

enum Rad60_BasicCmd_t : uint8_t {
    CMD_REG_WRITE        = 0x00,
    CMD_REG_READ         = 0x01,
    CMD_SET_SENSE_LEVEL  = 0x02,
    CMD_GET_SENSE_LEVEL  = 0x03,
    CMD_SET_SENSE_TIME   = 0x04,
    CMD_GET_SENSE_TIME   = 0x05,
    CMD_SET_LIGHT_THRES  = 0x06,
    CMD_GET_LIGHT_THRES  = 0x07,
    CMD_SAVE_SETTINGS    = 0x08,
    CMD_GET_SAVE_STATUS  = 0x09,
    CMD_SET_OUT_STATE    = 0x0A,
    CMD_SET_PWM_DUTY     = 0x0B,
    CMD_MEM_WRITE        = 0x10,
    CMD_MEM_READ         = 0x11,
    CMD_SYSTEM_RESET     = 0x13,
    CMD_FLASH_WRITE      = 0x14,
    CMD_RESTORE_DEFAULTS = 0x15,
    CMD_SET_BAUD_RATE    = 0x19,
};

enum Rad60_RadarCfgCmd_t : uint8_t {
    CMD_GET_RADAR_INFO   = 0x30,
    CMD_GET_ALGO_TYPE    = 0x31,
    CMD_GET_BOUNDARY     = 0x32,
    CMD_GET_SENSE_CONFIG = 0x33,
    CMD_SET_MOT_NEAR     = 0x34,
    CMD_SET_MOT_SENS     = 0x35,
    CMD_SET_MICRO_FAR    = 0x36,
    CMD_SET_MICRO_NEAR   = 0x37,
    CMD_SET_MICRO_SENS   = 0x38,
    CMD_SET_BREATH_FAR   = 0x39,
    CMD_SET_BREATH_NEAR  = 0x3A,
    CMD_SET_BREATH_SENS  = 0x3B,
};

enum Rad60_RadarCtrlCmd_t : uint8_t {
    CMD_GET_RADAR_STATE  = 0xD0,
    CMD_SET_RADAR_ENABLE = 0xD1,
    CMD_SET_MOT_FAR      = 0xD2,
};

enum Rad60_UlpCmd_t : uint8_t {
    CMD_SET_ULP_TIME     = 0x90,
    CMD_READ_AT5815C     = 0x91,
    CMD_WRITE_AT5815C    = 0x92,
    CMD_ENTER_COMM_MODE  = 0xE0,
};

// =============================================================================
//   Protocol constants
// =============================================================================

#define PACK_SEND_HEAD   0x58
#define PACK_RECV_HEAD   0x59
#define PACK_REPORT_HEAD 0x5A
#define FRAME_MIN_LEN    5

// =============================================================================
//   Data structures
// =============================================================================

typedef struct {
    uint16_t range_val;
    int16_t  angle_val;
    int16_t  velo_val;
    uint8_t  objid;
} Rad60_TargetInfo_t;

typedef struct {
    uint8_t  obj_num;
    Rad60_TargetInfo_t obj[1];  // Cache only the first target in report order.
} Rad60_MultiTargetInfo_t;

// =============================================================================
//   Yoke_rad60 — AT6010 mmWave radar (Yoke library style)
//
//   Communicates via UART on an RS expansion port.
//   SDA = TX, SCL = RX. Baud rate fixed at 115200.
//
//   Usage:
//     Yoke_rad60 radar(UART_NUM_1);
//     radar.begin(board.maker.useport("rs_port4"));
//     // In loop (UART is serviced by a background task by default):
//     if (radar.isDetected()) { ... }
//     // For manual polling, use begin(port, false) and call update().
// =============================================================================

class Yoke_rad60 : public ryzobee_yoke {

public:
    explicit Yoke_rad60(uart_port_t uart_num = UART_NUM_1,
                        uint32_t baudrate = RAD60_DEFAULT_BAUDRATE)
        : ryzobee_yoke(kRYZOBEE_YOKE_RS, kRS_BUS_UART),
          _uart_num(uart_num), _baudrate(baudrate) {
        memset(&_multi_target, 0, sizeof(_multi_target));
        memset(_tmp_buf, 0, sizeof(_tmp_buf));
    }

    /**
     * @brief Enable optional UART diagnostics on a separate console stream.
     *
     * @param stream Console stream, or nullptr to disable; must outlive tracing.
     */
    void setDebugStream(Stream* stream) {
        yoke_detail::UartTask::Lock lock(_task);
        if (lock) _debug = stream;
    }

    ~Yoke_rad60() { if (_initialized) end(); }

    /**
     * @brief Initialize UART, wait for startup, and verify radar communication.
     *
     * @param iodata RS port description; must remain valid until end().
     * @param use_task Run UART receive/query processing in a background task (default true).
     * @param apply_defaults Whether to apply the default detection settings without saving to flash.
     * @param use_ulp Enter/exit communication mode for ULP firmware only (default false).
     * Initialization and explicit configuration commands remain synchronous.
     * Call begin/end from the owning task, without concurrent commands.
     * @return YOKE_PORT_ERROR_NONE on success, otherwise a port error code.
     */
    int begin(std::vector<uint8_t>* iodata, bool use_task = true, bool apply_defaults = true, bool use_ulp = false) {
        yoke_detail::UartTask::Lock lock(_task);
        if (!lock) { _trace("UART mutex unavailable"); return YOKE_PORT_ERROR_NOT_AVAILABLE; }
        if (_initialized) return YOKE_PORT_ERROR_ALREADY_INITIALIZED;
        _apply_defaults = apply_defaults;
        _use_ulp = use_ulp;
        int result = ryzobee_yoke::begin(iodata);
        if (result != YOKE_PORT_ERROR_NONE) return result;
        if (use_task && !_task.start("yoke_rad60", _taskPoll, this)) {
            _trace("Background task creation failed");
            ryzobee_yoke::end();
            return YOKE_PORT_ERROR_NOT_AVAILABLE;
        }
        return YOKE_PORT_ERROR_NONE;
    }

    // ====================================================================
    //   Status queries (non-blocking, read from cache)
    // ====================================================================

    bool isConnected() const {
        yoke_detail::CacheLock lock(_cache_mutex);
        return _is_comm_ok;
    }
    bool isDetected() const {
        yoke_detail::CacheLock lock(_cache_mutex);
        return _is_detected;
    }
    uint8_t getTargetCount() const {
        yoke_detail::CacheLock lock(_cache_mutex);
        return _multi_target.obj_num;
    }

    uint16_t getTargetDistance(uint8_t index = 0) const {
        yoke_detail::CacheLock lock(_cache_mutex);
        if (index >= _multi_target.obj_num) return 0;
        return _multi_target.obj[index].range_val;
    }

    int16_t getTargetAngle(uint8_t index = 0) const {
        yoke_detail::CacheLock lock(_cache_mutex);
        if (index >= _multi_target.obj_num) return 0;
        return _multi_target.obj[index].angle_val;
    }

    int16_t getTargetVelocity(uint8_t index = 0) const {
        yoke_detail::CacheLock lock(_cache_mutex);
        if (index >= _multi_target.obj_num) return 0;
        return _multi_target.obj[index].velo_val;
    }

    uint8_t getTargetID(uint8_t index = 0) const {
        yoke_detail::CacheLock lock(_cache_mutex);
        if (index >= _multi_target.obj_num) return 0;
        return _multi_target.obj[index].objid;
    }

    // ====================================================================
    //   Manual update (call frequently from loop only when use_task=false)
    // ====================================================================

    void update() {
        if (_task.running()) return;
        yoke_detail::UartTask::Lock lock(_task);
        if (lock) _update();
    }

    // ====================================================================
    //   Radar configuration commands
    // ====================================================================

    bool setMotionNear(uint16_t distance_cm) {
        uint8_t tx[2] = { (uint8_t)(distance_cm & 0xFF), (uint8_t)((distance_cm >> 8) & 0xFF) };
        uint8_t rx[8], rx_len = sizeof(rx);
        if (!_sendCommandSync(CMD_SET_MOT_NEAR, tx, 2, rx, &rx_len, RAD60_CMD_TIMEOUT_MS)) return false;
        return (rx_len > 0 && rx[0] == 0x00);
    }

    bool setMotionFar(uint16_t distance_cm) {
        uint8_t tx[2] = { (uint8_t)(distance_cm & 0xFF), (uint8_t)((distance_cm >> 8) & 0xFF) };
        uint8_t rx[8], rx_len = sizeof(rx);
        if (!_sendCommandSync(CMD_SET_MOT_FAR, tx, 2, rx, &rx_len, RAD60_CMD_TIMEOUT_MS)) return false;
        return (rx_len > 0 && rx[0] == 0x00);
    }

    bool setMotionSensitivity(uint8_t sensitivity) {
        uint8_t tx[1] = { sensitivity };
        uint8_t rx[8], rx_len = sizeof(rx);
        if (!_sendCommandSync(CMD_SET_MOT_SENS, tx, 1, rx, &rx_len, RAD60_CMD_TIMEOUT_MS)) return false;
        return (rx_len > 0 && rx[0] == 0x00);
    }

    bool setBreathNear(uint16_t distance_cm) {
        uint8_t tx[2] = { (uint8_t)(distance_cm & 0xFF), (uint8_t)((distance_cm >> 8) & 0xFF) };
        uint8_t rx[8], rx_len = sizeof(rx);
        if (!_sendCommandSync(CMD_SET_BREATH_NEAR, tx, 2, rx, &rx_len, RAD60_CMD_TIMEOUT_MS)) return false;
        return (rx_len > 0 && rx[0] == 0x00);
    }

    bool setBreathFar(uint16_t distance_cm) {
        uint8_t tx[2] = { (uint8_t)(distance_cm & 0xFF), (uint8_t)((distance_cm >> 8) & 0xFF) };
        uint8_t rx[8], rx_len = sizeof(rx);
        if (!_sendCommandSync(CMD_SET_BREATH_FAR, tx, 2, rx, &rx_len, RAD60_CMD_TIMEOUT_MS)) return false;
        return (rx_len > 0 && rx[0] == 0x00);
    }

    bool setBreathSensitivity(uint8_t sensitivity) {
        uint8_t tx[1] = { sensitivity };
        uint8_t rx[8], rx_len = sizeof(rx);
        if (!_sendCommandSync(CMD_SET_BREATH_SENS, tx, 1, rx, &rx_len, RAD60_CMD_TIMEOUT_MS)) return false;
        return (rx_len > 0 && rx[0] == 0x00);
    }

    bool getRadarState(uint8_t* state) {
        if (state == nullptr) return false;
        uint8_t rx[8], rx_len = sizeof(rx);
        if (!_sendCommandSync(CMD_GET_RADAR_STATE, nullptr, 0, rx, &rx_len, RAD60_CMD_TIMEOUT_MS)) return false;
        if (rx_len < 1) return false;
        *state = rx[0];
        return true;
    }

    bool setRadarEnable(uint8_t enable) {
        uint8_t tx[1] = { enable };
        uint8_t rx[8], rx_len = sizeof(rx);
        if (!_sendCommandSync(CMD_SET_RADAR_ENABLE, tx, 1, rx, &rx_len, RAD60_CMD_TIMEOUT_MS)) return false;
        return (rx_len > 0 && rx[0] == 0x00);
    }

    bool saveSettings() {
        uint8_t rx[8], rx_len = sizeof(rx);
        if (!_sendCommandSync(CMD_SAVE_SETTINGS, nullptr, 0, rx, &rx_len, RAD60_CMD_TIMEOUT_MS)) return false;
        return (rx_len > 0 && rx[0] == 0x00);
    }

    bool systemReset() {
        uint8_t rx[8], rx_len = sizeof(rx);
        return _sendCommandSync(CMD_SYSTEM_RESET, nullptr, 0, rx, &rx_len, RAD60_CMD_TIMEOUT_MS);
    }

    bool setSenseLevel(uint8_t level) {
        uint8_t tx[1] = { level };
        uint8_t rx[8], rx_len = sizeof(rx);
        if (!_sendCommandSync(CMD_SET_SENSE_LEVEL, tx, 1, rx, &rx_len, RAD60_CMD_TIMEOUT_MS)) return false;
        return (rx_len > 0 && rx[0] == 0x00);
    }

    bool getSenseLevel(uint8_t* level) {
        if (level == nullptr) return false;
        uint8_t rx[8], rx_len = sizeof(rx);
        if (!_sendCommandSync(CMD_GET_SENSE_LEVEL, nullptr, 0, rx, &rx_len, RAD60_CMD_TIMEOUT_MS)) return false;
        if (rx_len < 1) return false;
        *level = rx[0];
        return true;
    }

    bool enterCommunicationMode(bool enter = true) {
        uint8_t tx[1] = { enter ? (uint8_t)1 : (uint8_t)0 };
        uint8_t rx[8], rx_len = sizeof(rx);
        if (!_sendCommandSync(CMD_ENTER_COMM_MODE, tx, 1, rx, &rx_len, RAD60_CMD_TIMEOUT_MS)) return false;
        return (rx_len == 1 && rx[0] == 0x00);
    }

    bool applyDefaultConfig() {
        yoke_detail::UartTask::Lock lock(_task);
        if (!lock || !_initialized) return false;
        bool ok = true;
        if (!setMotionNear(10))       { ok = false; }
        if (!setMotionFar(1000))      { ok = false; }
        if (!setMotionSensitivity(5)) { ok = false; }
        if (!setBreathNear(10))       { ok = false; }
        if (!setBreathFar(1000))      { ok = false; }
        if (!setBreathSensitivity(5)) { ok = false; }
        uint8_t state = 0;
        if (!getRadarState(&state))   { ok = false; }
        return ok;
    }

    uint32_t getRawBytesReceived() const {
        yoke_detail::CacheLock lock(_cache_mutex);
        return _raw_byte_count;
    }

    /**
     * @brief Print receive counters and the last 32 UART bytes.
     *
     * @param stream Console stream, separate from the radar UART.
     */
    void printDiagnostics(Stream& stream) {
        uint32_t bytes, responses, reports, targets;
        uint8_t type, count, tail[32];
        {
            yoke_detail::UartTask::Lock lock(_task);
            if (!lock) return;
            bytes = _raw_byte_total;
            responses = _response_count;
            reports = _report_count;
            targets = _target_report_count;
            type = _last_report_type;
            count = bytes < sizeof(tail) ? bytes : sizeof(tail);
            for (uint8_t i = 0; i < count; ++i) {
                tail[i] = _raw_tail[(_raw_tail_pos + sizeof(tail) - count + i) % sizeof(tail)];
            }
        }
        stream.printf("[RAD60] RX=%lu replies=%lu reports=%lu target_frames=%lu last_type=0x%02X\n",
                      (unsigned long)bytes, (unsigned long)responses, (unsigned long)reports,
                      (unsigned long)targets, type);
        stream.print("[RAD60] RX tail:");
        for (uint8_t i = 0; i < count; ++i) stream.printf(" %02X", tail[i]);
        stream.println();
    }

    static const char* detStatusToString(uint8_t det_result) {
        switch (det_result) {
            case 0x01: return "approach";
            case 0x02: return "leave";
            case 0x04: return "motion";
            case 0x08: return "micro";
            case 0x10: return "breath";
            default:   return "unknown";
        }
    }

protected:
    // ====================================================================
    //   Yoke base class overrides
    // ====================================================================

    int portBegin(std::vector<uint8_t>* iodata) override {
        if (_initialized) {
            return YOKE_PORT_ERROR_ALREADY_INITIALIZED;
        }

        if (_uart_num < UART_NUM_0 || _uart_num >= UART_NUM_MAX ||
            uart_is_driver_installed(_uart_num)) {
            _trace("UART index invalid or already occupied");
            return YOKE_PORT_ERROR_NOT_AVAILABLE;
        }
        if (!GPIO_IS_VALID_OUTPUT_GPIO(iodata->at(YOKE_PORT_PIN_START)) ||
            !GPIO_IS_VALID_GPIO(iodata->at(YOKE_PORT_PIN_START + 1)) ||
            iodata->at(YOKE_PORT_PIN_START) == iodata->at(YOKE_PORT_PIN_START + 1)) {
            return YOKE_PORT_ERROR_NOT_CONFIGURED;
        }

        _tx_pin = iodata->at(YOKE_PORT_PIN_START);      // SDA → TX
        _rx_pin = iodata->at(YOKE_PORT_PIN_START + 1);  // SCL → RX

        const uart_config_t uart_config = {
            .baud_rate  = (int)_baudrate,
            .data_bits  = UART_DATA_8_BITS,
            .parity     = UART_PARITY_DISABLE,
            .stop_bits  = UART_STOP_BITS_1,
            .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
#if ESP_IDF_VERSION_MAJOR >= 5
            .source_clk = UART_SCLK_DEFAULT,
#else
            .source_clk = UART_SCLK_APB,
#endif
        };

        esp_err_t err = uart_param_config(_uart_num, &uart_config);
        if (err != ESP_OK) {
            if (_debug) _debug->printf("[RAD60] UART config failed: %d\n", err);
            return YOKE_PORT_ERROR_NOT_AVAILABLE;
        }

        err = uart_set_pin(_uart_num, _tx_pin, _rx_pin,
                           UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
        if (err != ESP_OK) {
            if (_debug) _debug->printf("[RAD60] UART pins failed: %d\n", err);
            return YOKE_PORT_ERROR_NOT_AVAILABLE;
        }

        err = uart_driver_install(_uart_num, 2048, 1024, 0, nullptr, 0);
        if (err != ESP_OK) {
            if (_debug) _debug->printf("[RAD60] UART install failed: %d\n", err);
            return YOKE_PORT_ERROR_NOT_AVAILABLE;
        }

        uart_set_rx_timeout(_uart_num, 50);
        if (_baudrate > 460800) {
            uart_set_rx_full_threshold(_uart_num, 50);
        }

        _parse_state   = WAIT_HEADER;
        _resetCache();
        _query_sent    = false;
        _waiting_resp  = false;
        _resp_ok       = false;

        _init_time_ms   = millis();
        _last_data_ms   = _init_time_ms;
        _last_query_ms  = 0;
        _last_target_ms = 0;
        _target_from_query = false;
        _query_cmd = CMD_GET_RADAR_STATE;
        _raw_byte_total = 0;
        _response_count = 0;
        _report_count = 0;
        _target_report_count = 0;
        _last_report_type = 0;
        _raw_tail_pos = 0;
        _initialized    = true;

        // ULP entry is restricted to the module's power-on window.
        // Do not impose the normal firmware startup delay before ULP entry.
        bool entered_ulp = false;
        bool ready = true;
        if (_use_ulp) {
            _trace("Entering ULP communication mode");
            entered_ulp = enterCommunicationMode(true);
            ready = entered_ulp;
            if (!ready) _trace("ULP entry failed; check firmware and module power-on window");
        } else {
            _trace("UART ready; waiting 2000ms for module startup");
            delay(2000);
        }
        uint8_t state = 0;
        if (ready) {
            _trace("Querying radar state");
            ready = false;
            for (uint8_t attempt = 0; attempt < RAD60_CMD_RETRY_COUNT; ++attempt) {
                if (getRadarState(&state)) {
                    ready = true;
                    break;
                }
            }
            if (!ready) _trace("Radar state query failed");
        }
        if (ready && _apply_defaults) {
            _trace("Applying default settings");
            ready = applyDefaultConfig();
            if (!ready) _trace("Default settings failed");
        }
        // Resume ULP detection even when a configuration command failed.
        if (entered_ulp) {
            _trace("Leaving ULP communication mode");
            if (!enterCommunicationMode(false)) {
                _trace("ULP exit failed");
                ready = false;
            }
        }
        if (!ready) {
            portEnd();
            return YOKE_PORT_ERROR_NOT_AVAILABLE;
        }
        {
            yoke_detail::CacheLock lock(_cache_mutex);
            _is_comm_ok = true;
        }
        _last_data_ms = millis();
        return YOKE_PORT_ERROR_NONE;
    }

    int portEnd() override {
        _task.stop();
        yoke_detail::UartTask::Lock lock(_task);
        if (!lock) return YOKE_PORT_ERROR_NOT_AVAILABLE;
        if (!_initialized) return YOKE_PORT_ERROR_NOT_INITIALIZED;
        uart_flush(_uart_num);
        uart_wait_tx_done(_uart_num, pdMS_TO_TICKS(100));
        if (uart_driver_delete(_uart_num) != ESP_OK) return YOKE_PORT_ERROR_NOT_AVAILABLE;
        _initialized  = false;
        _resetCache();
        _waiting_resp = false;
        _resp_buf = nullptr;
        return YOKE_PORT_ERROR_NONE;
    }

    std::string getYokeName() override { return "Yoke RAD60"; }
    std::string getYokeInfo() override { return "AT6010 mmWave radar via UART on RS PORT"; }

private:
    void _trace(const char* message) {
        if (_debug) { _debug->print("[RAD60] "); _debug->println(message); }
    }

    Stream* _debug = nullptr;
    static void _taskPoll(void* context) {
        static_cast<Yoke_rad60*>(context)->_update();
    }

    // Caller holds the UART mutex, including during synchronous response waits.
    void _update() {
        if (!_initialized) return;
        uint32_t byte_count = 0;
        uint8_t rx_byte;
        // Bound each pass so continuous reports cannot starve command timeouts.
        while (byte_count < 256 && uart_read_bytes(_uart_num, &rx_byte, 1, 0) > 0) {
            byte_count++;
            _raw_tail[_raw_tail_pos] = rx_byte;
            _raw_tail_pos = (_raw_tail_pos + 1) % sizeof(_raw_tail);
            _raw_byte_total++;
            _processByte(rx_byte);
        }
        {
            yoke_detail::CacheLock lock(_cache_mutex);
            _raw_byte_count = _raw_byte_total;
        }

        uint32_t now = millis();
        uint32_t target_timeout = _target_from_query
            ? 2 * RAD60_QUERY_INTERVAL_MS + RAD60_COMM_TIMEOUT_MS
            : RAD60_NO_DATA_TIMEOUT_MS;
        if (now - _last_target_ms > target_timeout) {
            yoke_detail::CacheLock lock(_cache_mutex);
            _is_detected = false;
            _multi_target.obj_num = 0;
        }
        if (_query_sent && now - _last_query_ms > RAD60_COMM_TIMEOUT_MS) {
            _query_sent = false;
        }
        if (now - _init_time_ms > 5000 && now - _last_data_ms > RAD60_COMM_TIMEOUT_MS) {
            yoke_detail::CacheLock lock(_cache_mutex);
            _is_comm_ok = false;
        }
        // Poll detection information when the firmware sends no target reports.
        // Alternate with the state query to retain a heartbeat if 0x30 is unsupported.
        if (!_waiting_resp && !_query_sent &&
            now - _last_query_ms > RAD60_QUERY_INTERVAL_MS &&
            now - _last_target_ms > RAD60_NO_DATA_TIMEOUT_MS) {
            _query_cmd = (_query_cmd == CMD_GET_RADAR_STATE)
                ? CMD_GET_RADAR_INFO : CMD_GET_RADAR_STATE;
            _sendCommand(_query_cmd, nullptr, 0);
            _last_query_ms = now;
            _query_sent = true;
        }
    }

    void _resetCache() {
        yoke_detail::CacheLock lock(_cache_mutex);
        _is_comm_ok = false;
        _is_detected = false;
        _raw_byte_count = 0;
        memset(&_multi_target, 0, sizeof(_multi_target));
    }

    yoke_detail::UartTask _task;
    mutable yoke_detail::CacheMutex _cache_mutex;

    // ====================================================================
    //   Hardware
    // ====================================================================

    uart_port_t _uart_num;
    uint8_t     _tx_pin = 0, _rx_pin = 0;
    uint32_t    _baudrate;

    // ====================================================================
    //   Radar data cache
    // ====================================================================

    bool                    _is_comm_ok = false;
    bool                    _apply_defaults = true;
    bool                    _use_ulp = false;
    bool                    _is_detected = false;
    Rad60_MultiTargetInfo_t _multi_target;

    // ====================================================================
    //   Timeout tracking
    // ====================================================================

    uint32_t _last_data_ms   = 0;
    uint32_t _last_query_ms  = 0;
    bool     _query_sent     = false;
    uint8_t  _query_cmd = CMD_GET_RADAR_STATE;
    uint32_t _last_target_ms = 0;
    bool     _target_from_query = false;
    uint32_t _init_time_ms   = 0;
    uint32_t _raw_byte_count = 0;
    uint32_t _raw_byte_total = 0;
    uint32_t _response_count = 0, _report_count = 0, _target_report_count = 0;
    uint8_t _last_report_type = 0;
    uint8_t _raw_tail[32] = {}, _raw_tail_pos = 0;

    // ====================================================================
    //   Protocol: build and send a command frame
    //   Format: 0x58 + CMD(1) + ParaLen(1) + Data[ParaLen] + CheckCode(2, LE)
    // ====================================================================

    void _sendCommand(uint8_t cmd, const uint8_t* data, uint8_t len) {
        if (!_initialized) return;
        uint32_t total_len = 1 + 1 + 1 + len + 2;
        uint8_t* out_buf = (uint8_t*)malloc(total_len);
        if (out_buf == nullptr) return;

        uint16_t check = 0x58 + cmd + len;
        out_buf[0] = 0x58;
        out_buf[1] = cmd;
        out_buf[2] = len;
        for (uint8_t i = 0; i < len; i++) {
            out_buf[3 + i] = data[i];
            check += data[i];
        }
        out_buf[3 + len]     = check & 0xFF;
        out_buf[3 + len + 1] = (check >> 8) & 0xFF;

        if (_debug) {
            _debug->print("[RAD60] TX:");
            for (uint32_t i = 0; i < total_len; ++i) _debug->printf(" %02X", out_buf[i]);
            _debug->println();
        }
        int written = uart_write_bytes(_uart_num, out_buf, total_len);
        if (_debug && written != static_cast<int>(total_len)) _trace("UART write incomplete");
        uart_wait_tx_done(_uart_num, pdMS_TO_TICKS(50));
        free(out_buf);
    }

    // ====================================================================
    //   Protocol: synchronous command with response wait
    // ====================================================================

    bool _sendCommandSync(uint8_t cmd, const uint8_t* tx_data, uint8_t tx_len,
                          uint8_t* rx_buf, uint8_t* rx_len, uint32_t timeout_ms) {
        yoke_detail::UartTask::Lock lock(_task);
        if (!lock || !_initialized) return false;
        _waiting_resp    = true;
        _wait_cmd        = cmd;
        _resp_buf        = rx_buf;
        _resp_buf_size   = (rx_buf && rx_len) ? *rx_len : 0;
        _resp_len        = 0;
        _resp_ok         = false;
        _resp_start_ms   = millis();
        _resp_timeout_ms = timeout_ms;

        uint32_t received_before = _raw_byte_total;
        _sendCommand(cmd, tx_data, tx_len);

        while (!_resp_ok && (millis() - _resp_start_ms < _resp_timeout_ms)) {
            _update();
            delay(1);
        }

        if (_debug) {
            _debug->printf("[RAD60] CMD=0x%02X %s; received %lu bytes\n",
                           cmd, _resp_ok ? "reply matched" : "timeout",
                           static_cast<unsigned long>(_raw_byte_total - received_before));
        }
        _waiting_resp = false;
        _resp_buf = nullptr;
        if (_resp_ok && rx_len) {
            *rx_len = _resp_len;
        }
        return _resp_ok;
    }

    // ====================================================================
    //   Protocol: byte-level state machine
    // ====================================================================

    enum ParseState {
        WAIT_HEADER,
        WAIT_RESP_CMD, WAIT_RESP_LEN, WAIT_RESP_DATA,
        WAIT_RESP_CHECK1, WAIT_RESP_CHECK2,
        WAIT_REPORT_LEN, WAIT_REPORT_TYPE, WAIT_REPORT_DATA,
        WAIT_REPORT_CHECK,
    };

    ParseState _parse_state = WAIT_HEADER;
    uint8_t    _frame_head = 0, _cmd = 0, _data_len = 0, _data_idx = 0;
    uint8_t    _tmp_buf[256];
    uint16_t   _check_sum_calc = 0;

    bool     _waiting_resp = false;
    uint8_t  _wait_cmd = 0;
    uint8_t* _resp_buf = nullptr;
    uint8_t  _resp_buf_size = 0, _resp_len = 0;
    bool     _resp_ok = false;
    uint32_t _resp_start_ms = 0, _resp_timeout_ms = 0;

    void _processByte(uint8_t c) {
        switch (_parse_state) {

            case WAIT_HEADER:
                if (c == PACK_RECV_HEAD) {
                    _frame_head     = PACK_RECV_HEAD;
                    _parse_state    = WAIT_RESP_CMD;
                    _check_sum_calc = PACK_RECV_HEAD;
                } else if (c == PACK_REPORT_HEAD) {
                    _frame_head     = PACK_REPORT_HEAD;
                    _parse_state    = WAIT_REPORT_LEN;
                    _check_sum_calc = PACK_REPORT_HEAD;
                }
                break;

            case WAIT_RESP_CMD:
                _cmd = c;
                _check_sum_calc += c;
                _parse_state = WAIT_RESP_LEN;
                break;

            case WAIT_RESP_LEN:
                _data_len = c;
                _check_sum_calc += c;
                _data_idx = 0;
                if (_data_len > sizeof(_tmp_buf)) {
                    _parse_state = WAIT_HEADER;
                } else if (_data_len == 0) {
                    _parse_state = WAIT_RESP_CHECK1;
                } else {
                    _parse_state = WAIT_RESP_DATA;
                }
                break;

            case WAIT_RESP_DATA:
                _tmp_buf[_data_idx++] = c;
                _check_sum_calc += c;
                if (_data_idx >= _data_len) {
                    _parse_state = WAIT_RESP_CHECK1;
                }
                break;

            case WAIT_RESP_CHECK1:
                _tmp_buf[_data_len] = c;
                _parse_state = WAIT_RESP_CHECK2;
                break;

            case WAIT_RESP_CHECK2: {
                uint16_t check_recv = _tmp_buf[_data_len] | ((uint16_t)c << 8);
                if (_check_sum_calc == check_recv) {
                    _processResponseFrame(_cmd, _data_len, _tmp_buf);
                }
                _parse_state = WAIT_HEADER;
                break;
            }

            case WAIT_REPORT_LEN:
                _data_len = c;
                _check_sum_calc += c;
                _data_idx = 0;
                if (_data_len < 1 || _data_len > sizeof(_tmp_buf) + 1) {
                    _parse_state = WAIT_HEADER;
                } else {
                    _parse_state = WAIT_REPORT_TYPE;
                }
                break;

            case WAIT_REPORT_TYPE:
                _tmp_buf[0] = c;
                _check_sum_calc += c;
                _data_idx = 0;
                _parse_state = (_data_len == 1) ? WAIT_REPORT_CHECK : WAIT_REPORT_DATA;
                break;

            case WAIT_REPORT_DATA:
                _tmp_buf[1 + _data_idx] = c;
                _check_sum_calc += c;
                _data_idx++;
                if (_data_idx >= _data_len - 1) {
                    _parse_state = WAIT_REPORT_CHECK;
                }
                break;

            case WAIT_REPORT_CHECK: {
                if ((_check_sum_calc & 0xFF) == c) {
                    uint8_t type = _tmp_buf[0];
                    _processReportFrame(_data_len, type, &_tmp_buf[1]);
                }
                _parse_state = WAIT_HEADER;
                break;
            }
        }
    }

    void _processResponseFrame(uint8_t cmd, uint8_t len, uint8_t* data) {
        _response_count++;
        if (_waiting_resp && !_resp_ok && _wait_cmd == cmd) {
            if (_resp_buf && data && len > 0) {
                uint8_t copy_len = (len < _resp_buf_size) ? len : _resp_buf_size;
                memcpy(_resp_buf, data, copy_len);
                _resp_len = copy_len;
            }
            _resp_ok = true;
        }
        _last_data_ms = millis();
        {
            yoke_detail::CacheLock lock(_cache_mutex);
            _is_comm_ok = true;
        }
        if (cmd == _query_cmd) _query_sent = false;
        if (cmd == CMD_GET_RADAR_INFO && len == 20) {
            _processSingleTargetInfo(len, data, true);
        }
    }

    void _processReportFrame(uint8_t len, uint8_t type, uint8_t* data) {
        _report_count++;
        _last_report_type = type;
        _last_data_ms = millis();
        {
            yoke_detail::CacheLock lock(_cache_mutex);
            _is_comm_ok = true;
        }
        if (type == 0x0A) {
            _processMultiTargetInfo(len - 1, data);
        } else if (type == 0x03 || type == 0x08) {
            _processSingleTargetInfo(len - 1, data, false);
        }
    }

    /**
     * @brief Cache single-target detection information from replies or reports.
     *
     * @param len Payload length, excluding report TYPE; at least 8 bytes.
     * @param data Detection payload, starting with is_detected; must not be null.
     * @param from_query True for a 0x30 reply, false for a 0x03/0x08 report.
     */
    void _processSingleTargetInfo(uint8_t len, const uint8_t* data, bool from_query) {
        if (!data || len < 8) return;
        _target_report_count++;
        _last_target_ms = millis();
        _target_from_query = from_query;
        uint16_t range_mm = (uint16_t)data[2] | ((uint16_t)data[3] << 8);
        uint16_t angle = (uint16_t)data[4] | ((uint16_t)data[5] << 8);
        yoke_detail::CacheLock lock(_cache_mutex);
        memset(&_multi_target, 0, sizeof(_multi_target));
        _is_detected = data[0] != 0;
        if (!_is_detected) return;
        _multi_target.obj_num = 1;
        _multi_target.obj[0].range_val = range_mm / 10;
        _multi_target.obj[0].angle_val = (int16_t)((angle & 0x8000) ? (int32_t)angle - 0x10000 : angle);
        // The protocol reserves velocity and supplies no target ID in this format.
    }

    // ====================================================================
    //   Single-target cache from multi-target reports (type=0x0A)
    //
    //   Data: count + 3 reserved bytes + count * 8-byte records.
    //   Distance: cm; angle: degrees; velocity: cm/s; little-endian.
    // ====================================================================

    /**
     * @brief Validate a multi-target report and save only the first target in report order.
     *
     * @param len Data length, excluding the TYPE byte.
     * @param data Report data; must not be null. Ignore incomplete frames or counts above the target limit.
     */
    void _processMultiTargetInfo(uint8_t len, uint8_t* data) {
        if (data == nullptr || len < 4) return;
        uint8_t obj_num = data[0];
        if (obj_num > 3 || len < 4 + obj_num * 8) return;

        _target_report_count++;
        _last_target_ms = millis();
        _target_from_query = false;
        yoke_detail::CacheLock lock(_cache_mutex);
        memset(&_multi_target, 0, sizeof(_multi_target));
        _is_detected = (obj_num > 0);
        if (!_is_detected) return;

        const uint8_t* target = data + 4;
        _multi_target.obj[0].range_val = (uint16_t)target[0] | ((uint16_t)target[1] << 8);
        uint16_t angle = (uint16_t)target[2] | ((uint16_t)target[3] << 8);
        uint16_t velocity = (uint16_t)target[4] | ((uint16_t)target[5] << 8);
        _multi_target.obj[0].angle_val = (int16_t)((angle & 0x8000) ? (int32_t)angle - 0x10000 : angle);
        _multi_target.obj[0].velo_val = (int16_t)((velocity & 0x8000) ? (int32_t)velocity - 0x10000 : velocity);
        _multi_target.obj[0].objid = target[6];
        _multi_target.obj_num = 1;
    }

};

#else
// Non-ESP32 stub
class Yoke_rad60 : public ryzobee_yoke {
public:
    Yoke_rad60(int uart_num = 0, uint32_t baudrate = 115200)
        : ryzobee_yoke(kRYZOBEE_YOKE_RS, kRS_BUS_UART) {}
    int begin(std::vector<uint8_t>* iodata, bool use_task = true, bool apply_defaults = true, bool use_ulp = false) {
        return ryzobee_yoke::begin(iodata);
    }
    void update() {}
    bool isConnected() const { return false; }
    bool isDetected() const { return false; }
    uint8_t getTargetCount() const { return 0; }
    uint16_t getTargetDistance(uint8_t i = 0) const { return 0; }
    int16_t getTargetAngle(uint8_t i = 0) const { return 0; }
    int16_t getTargetVelocity(uint8_t i = 0) const { return 0; }
    uint8_t getTargetID(uint8_t i = 0) const { return 0; }
    bool setMotionNear(uint16_t d) { return false; }
    bool setMotionFar(uint16_t d) { return false; }
    bool setMotionSensitivity(uint8_t s) { return false; }
    bool setBreathNear(uint16_t d) { return false; }
    bool setBreathFar(uint16_t d) { return false; }
    bool setBreathSensitivity(uint8_t s) { return false; }
    bool getRadarState(uint8_t* s) { return false; }
    bool setRadarEnable(uint8_t e) { return false; }
    bool saveSettings() { return false; }
    bool systemReset() { return false; }
    bool setSenseLevel(uint8_t l) { return false; }
    bool getSenseLevel(uint8_t* l) { return false; }
    bool enterCommunicationMode(bool e = true) { return false; }
    bool applyDefaultConfig() { return false; }
    uint32_t getRawBytesReceived() const { return 0; }
    static const char* detStatusToString(uint8_t d) { return "unknown"; }
protected:
    int portBegin(std::vector<uint8_t>* iodata) override { return YOKE_PORT_ERROR_NOT_SUPPORTED; }
    int portEnd() override { return YOKE_PORT_ERROR_NONE; }
    std::string getYokeName() override { return "Yoke RAD60"; }
    std::string getYokeInfo() override { return "Yoke RAD60"; }
};
#endif
