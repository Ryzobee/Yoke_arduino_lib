#pragma once

#if defined(ARDUINO_ARCH_ESP32)
#include <atomic>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#endif

namespace yoke_detail {

// Shared by UART drivers. Lifecycle calls (begin/end/destruction) belong to the
// owning application task; they must not race each other or public commands.
class UartTask {
public:
    UartTask() = default;
    UartTask(const UartTask&) = delete;
    UartTask& operator=(const UartTask&) = delete;

    ~UartTask() {
        stop();
#if defined(ARDUINO_ARCH_ESP32)
        if (_done) vSemaphoreDelete(_done);
        if (_mutex) vSemaphoreDelete(_mutex);
#endif
    }

    // Recursive because a polling pass may invoke a public synchronous command.
    class Lock {
    public:
        explicit Lock(UartTask& owner) : _owner(owner) {
#if defined(ARDUINO_ARCH_ESP32)
            _locked = owner._mutex &&
                      xSemaphoreTakeRecursive(owner._mutex, portMAX_DELAY) == pdTRUE;
#endif
        }
        ~Lock() {
#if defined(ARDUINO_ARCH_ESP32)
            if (_locked) xSemaphoreGiveRecursive(_owner._mutex);
#endif
        }
        explicit operator bool() const { return _locked; }
        Lock(const Lock&) = delete;
        Lock& operator=(const Lock&) = delete;
    private:
        UartTask& _owner;
        bool _locked = true;
    };

    bool start(const char* name, void (*poll)(void*), void* context) {
#if defined(ARDUINO_ARCH_ESP32)
        if (_running || !_mutex) return false;
        if (!_done) _done = xSemaphoreCreateBinary();
        if (!_done) return false;
        _poll = poll;
        _context = context;
        _stop = false;
        _running = true;
        // ESP-IDF specifies the stack size in bytes.
        if (xTaskCreate(_entry, name, 4096, this, 1, nullptr) != pdPASS) {
            _running = false;
            return false;
        }
        return true;
#else
        return false;
#endif
    }

    void stop() {
#if defined(ARDUINO_ARCH_ESP32)
        if (!_running) return;
        _stop = true;
        // Never delete a task in the middle of a UART transaction/held mutex.
        xSemaphoreTake(_done, portMAX_DELAY);
        _running = false;
#endif
    }

    bool running() const {
#if defined(ARDUINO_ARCH_ESP32)
        return _running.load();
#else
        return false;
#endif
    }

private:
#if defined(ARDUINO_ARCH_ESP32)
    static void _entry(void* arg) {
        auto* self = static_cast<UartTask*>(arg);
        while (!self->_stop.load()) {
            {
                Lock lock(*self);
                if (!self->_stop.load()) self->_poll(self->_context);
            }
            vTaskDelay(1);
        }
        // No access to self after signalling: its owner may now destroy it.
        xSemaphoreGive(self->_done);
        vTaskDelete(nullptr);
    }

    SemaphoreHandle_t _mutex = xSemaphoreCreateRecursiveMutex();
    SemaphoreHandle_t _done = nullptr;
    std::atomic<bool> _running{false}, _stop{false};
    void (*_poll)(void*) = nullptr;
    void* _context = nullptr;
#endif
};

// Cache reads never wait for a UART reply. Only small memory operations belong
// under this lock: no delay, UART, logging, or acquisition of UartTask::Lock.
class CacheMutex {
public:
    CacheMutex() = default;
    CacheMutex(const CacheMutex&) = delete;
    CacheMutex& operator=(const CacheMutex&) = delete;
    void lock() {
#if defined(ARDUINO_ARCH_ESP32)
        portENTER_CRITICAL(&_mux);
#endif
    }
    void unlock() {
#if defined(ARDUINO_ARCH_ESP32)
        portEXIT_CRITICAL(&_mux);
#endif
    }
private:
#if defined(ARDUINO_ARCH_ESP32)
    portMUX_TYPE _mux = portMUX_INITIALIZER_UNLOCKED;
#endif
};

class CacheLock {
public:
    explicit CacheLock(CacheMutex& mutex) : _mutex(mutex) { _mutex.lock(); }
    ~CacheLock() { _mutex.unlock(); }
    CacheLock(const CacheLock&) = delete;
    CacheLock& operator=(const CacheLock&) = delete;
private:
    CacheMutex& _mutex;
};

} // namespace yoke_detail
