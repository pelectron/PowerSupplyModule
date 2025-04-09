/**
 * @file
 * @brief
 */
#ifndef MCP45HVV51_HPP
#define MCP45HVV51_HPP
#include "hal/gpio.hpp"
#include "hal/i2c.hpp"

class MCP45HV51 {
public:
  static constexpr unsigned max_resistance = 5'000;
  enum Error {

  };

  void enable();
  void disable();
  Error set_resistance(float resistance);
  Error set_resistance(uint32_t code);

private:
  hal::i2c::Master i2c;
  hal::gpio::Output wlat;
  hal::gpio::Output shdn;
};

#endif // !MCP45HVV51_HPP
