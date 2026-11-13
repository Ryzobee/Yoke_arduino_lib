#pragma once
#include <vector>
#include <stdio.h>
#include <string>

typedef enum{
    kRYZOBEE_YOKE_RS = 0,
    kRYZOBEE_YOKE_S_BUS,
    kRYZOBEE_YOKE_F_BUS,
}yoke_inferface_type_t;

typedef enum{
    kRS_BUS_NONE = -1,
    kRS_BUS_I2C = 0,
    kRS_BUS_UART,
    kRS_BUS_GPIO,
    kRS_BUS_UNKNOWN,
}rs_bus_type_t;

enum yoke_port_field : uint8_t {
    YOKE_PORT_TYPE = 0,
    YOKE_PORT_USED = 1,
    YOKE_PORT_INVALID = 2,
    YOKE_PORT_PIN_NUM = 3,
    YOKE_PORT_PIN_START = 4,
};

enum yoke_port_error : int {
    YOKE_PORT_ERROR_NONE = 0,
    YOKE_PORT_ERROR_INVALID = -1,
    YOKE_PORT_ERROR_ALREADY_INITIALIZED = -2,
    YOKE_PORT_ERROR_NOT_INITIALIZED = -3,
    YOKE_PORT_ERROR_NOT_AVAILABLE = -4,
    YOKE_PORT_ERROR_NOT_SUPPORTED = -5,
    YOKE_PORT_ERROR_NOT_CONFIGURED = -6,
};

#define YOKE_DEBUG

#ifdef YOKE_DEBUG
#define YOKE_D_PRINTF(...) fprintf(stderr, __VA_ARGS__)
#define YOKE_I_PRINTF(...) fprintf(stdout, __VA_ARGS__)
#else
#define YOKE_D_PRINTF(...) ((void)0)
#define YOKE_I_PRINTF(...) ((void)0)
#endif

class ryzobee_yoke {
    public:
    ryzobee_yoke (yoke_inferface_type_t yoke_type, rs_bus_type_t bus_type = kRS_BUS_NONE):
        _yoke_type(yoke_type), _bus_type(bus_type) {};

    /**
     * @brief Initialize and reserve a compatible board port.
     *
     * @param iodata Port description; must remain valid until end().
     * @return YOKE_PORT_ERROR_NONE on success, otherwise a port error code.
     */
    int begin(std::vector<uint8_t>* iodata){
        if (_initialized) return YOKE_PORT_ERROR_ALREADY_INITIALIZED;
        if (portInvalid(iodata)) return YOKE_PORT_ERROR_INVALID;
        int result = portBegin(iodata);
        if (result == YOKE_PORT_ERROR_NONE) {
            _iodata = iodata;
            _iodata->at(YOKE_PORT_USED) = 1;
            _initialized = true;
        }
        return result;
    };

    /**
     * @brief Release hardware before making the reserved port available again.
     *
     * @return YOKE_PORT_ERROR_NONE on success, otherwise a port error code.
     */
    int end(){
        if (!_initialized || _iodata == nullptr) return YOKE_PORT_ERROR_NOT_INITIALIZED;
        int result = portEnd();
        if (result == YOKE_PORT_ERROR_NONE) {
            _iodata->at(YOKE_PORT_USED) = 0;
            _iodata = nullptr;
            _initialized = false;
        }
        return result;
    };

    bool portInvalid(std::vector<uint8_t>* iodata){
        if(iodata == nullptr){  YOKE_I_PRINTF("Port is null.\n"); return true; }
        if(iodata->size() < YOKE_PORT_PIN_START){  YOKE_I_PRINTF("Port header is incomplete.\n"); return true; }
        if(iodata->at(YOKE_PORT_USED) == 1){  YOKE_I_PRINTF("Port is used.\n"); return true; }
        if(iodata->at(YOKE_PORT_INVALID) == 1){  YOKE_I_PRINTF("Port is invalid.\n"); return true; }
        if(iodata->at(YOKE_PORT_TYPE) != _yoke_type){  YOKE_I_PRINTF("Port type is not %d.\n", _yoke_type); return true; }

        if (iodata->at(YOKE_PORT_PIN_NUM) == 0 ||
            iodata->size() < YOKE_PORT_PIN_START + iodata->at(YOKE_PORT_PIN_NUM)) return true;
        if (_yoke_type == kRYZOBEE_YOKE_RS && iodata->at(YOKE_PORT_PIN_NUM) != 2) return true;
        if (_yoke_type == kRYZOBEE_YOKE_F_BUS && iodata->at(YOKE_PORT_PIN_NUM) != 16) return true;

        YOKE_I_PRINTF("Port is available.\n");
        return false;
    };
    bool isInitialized(){ return _initialized; }
    virtual std::string getYokeName() = 0;
    virtual std::string getYokeInfo() = 0;
    

    std::string getYokeType(){ 
        switch(_yoke_type){
            case kRYZOBEE_YOKE_RS: return "RS_PORT";
            case kRYZOBEE_YOKE_F_BUS: return "F_BUS";
            case kRYZOBEE_YOKE_S_BUS: return "S_BUS";
            default: return "UNKNOWN";
        }
    }
    std::string getBusType(){ 
        switch(_bus_type){
            case kRS_BUS_NONE: return "NONE";
            case kRS_BUS_I2C: return "I2C";
            case kRS_BUS_UART: return "UART";
            case kRS_BUS_GPIO: return "GPIO";
            default: return "UNKNOWN";
        }
    }

    #if defined(CONFIG_IDF_TARGET_ESP32S3)

        // ESP32-S3 专用代码
        #define CHIP_IS_S3 1
    #endif
    protected:
        yoke_inferface_type_t _yoke_type;
        rs_bus_type_t _bus_type;
        bool _initialized = false;
        std::vector<uint8_t>* _iodata = nullptr;

        virtual int portBegin(std::vector<uint8_t>* iodata) = 0;
        virtual int portEnd() = 0;

};