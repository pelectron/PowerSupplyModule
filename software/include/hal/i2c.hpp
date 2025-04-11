#ifndef HAL_I2C_HPP
#define HAL_I2C_HPP

#include "hal/config.hpp"
namespace hal::i2c {
struct Config {};
struct Master {
  void init();
};

ConfigResult<Master> configure(const Config &cfg);
} // namespace hal::i2c

#endif
