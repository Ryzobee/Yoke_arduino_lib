#include "Touch_CST816T.hpp"

#include <LovyanGFX.hpp>
#include <Wire.h>

namespace lgfx
{
 inline namespace v1
 {

  static constexpr uint8_t CST816S_TOUCH_REG  = 0x01;
  static constexpr uint8_t CST816S_CHIPID_REG = 0xA7;

  bool Touch_CST816T::_write_reg(uint8_t reg, uint8_t val)
  {
    if (_wire == nullptr) {
      return false;
    }
    _wire->beginTransmission(_cfg.i2c_addr);
    _wire->write(reg);
    _wire->write(val);
    return (_wire->endTransmission() == 0);
  }

  bool Touch_CST816T::_write_regs(uint8_t* val, size_t length)
  {
    if (_wire == nullptr) {
      return false;
    }
    _wire->beginTransmission(_cfg.i2c_addr);
    _wire->write(val, length);
    return (_wire->endTransmission() == 0);
  }

  bool Touch_CST816T::_read_reg(uint8_t reg, uint8_t *data, size_t length)
  {
    if (_wire == nullptr) {
      return false;
    }
    _wire->beginTransmission(_cfg.i2c_addr);
    _wire->write(reg);
    if (_wire->endTransmission(false) != 0) {
      return false;
    }

    size_t received = _wire->requestFrom((uint8_t)_cfg.i2c_addr, (size_t)length);
    if (received != length) {
      return false;
    }

    for (size_t i = 0; i < length; i++) {
      data[i] = _wire->read();
    }
    return true;
  }

  bool Touch_CST816T::_check_init(void)
  {
    if (_inited) return true;

    uint8_t tmp[3] = { 0 };
    _inited = _read_reg(CST816S_CHIPID_REG, tmp, 3);
    return _inited;
  }

  bool Touch_CST816T::init(void)
  {
    _inited = false;

    if (_cfg.pin_rst >= 0)
    {
      lgfx::pinMode(_cfg.pin_rst, pin_mode_t::output);
      lgfx::gpio_lo(_cfg.pin_rst);
      lgfx::delay(10);
      lgfx::gpio_hi(_cfg.pin_rst);
      lgfx::delay(50);
    }

    if (_cfg.pin_int >= 0)
    {
      lgfx::pinMode(_cfg.pin_int, pin_mode_t::input_pullup);
    }

    _inited = true;
    return true;
  }

  void Touch_CST816T::wakeup(void)
  {
    if (!_inited) return;
    if (_cfg.pin_int < 0) return;
    lgfx::gpio_hi(_cfg.pin_int);
    lgfx::lgfxPinMode(_cfg.pin_int, pin_mode_t::output);
    delay(5);
    lgfx::lgfxPinMode(_cfg.pin_int, pin_mode_t::input);
  }

  void Touch_CST816T::sleep(void)
  {
    if (!_inited) return;
  }

  size_t Touch_CST816T::_read_data(uint8_t* readdata)
  {
    if (_wire == nullptr) {
      return 0;
    }

    size_t res = 0;
    _wire->beginTransmission(_cfg.i2c_addr);
    _wire->write(0x02);
    if (_wire->endTransmission(false) == 0)
    {
      if (_wire->requestFrom((uint8_t)_cfg.i2c_addr, (size_t)1) == 1)
      {
        readdata[0] = _wire->read();
        uint_fast8_t points = std::min<uint_fast8_t>(max_touch_points, readdata[0] & 0x0Fu);
        if (points)
        {
          size_t bytesToRead = points * 6 - 2;
          if (_wire->requestFrom((uint8_t)_cfg.i2c_addr, bytesToRead) == bytesToRead)
          {
            for (size_t i = 0; i < bytesToRead; i++) {
              readdata[1 + i] = _wire->read();
            }
            res = points * 6 - 1;
          }
        }
        else
        {
          res = 1;
        }
      }
    }
    return res;
  }

  uint_fast8_t Touch_CST816T::getTouchRaw(touch_point_t *tp, uint_fast8_t count)
  {
    if (tp == nullptr || count == 0) { return 0; }
    if (count > max_touch_points) { count = max_touch_points; }

    uint8_t readdata[8] = {0};
    if (!_read_reg(CST816S_TOUCH_REG, readdata, 6))
    {
      return 0;
    }

    uint8_t points = readdata[1];
    if (points == 0 || points > max_touch_points)
    {
      return 0;
    }

    uint8_t event_flag = readdata[2] & 0xC0;
    if (event_flag == 0x40)
    {
      return 0;
    }

    int32_t x = ((readdata[2] & 0x0F) << 8) + readdata[3];
    int32_t y = ((readdata[4] & 0x0F) << 8) + readdata[5];

    tp[0].id = 0;
    tp[0].size = 1;
    tp[0].x = x;
    tp[0].y = y;

    return 1;
  }

 }
}