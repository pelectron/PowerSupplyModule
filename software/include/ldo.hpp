#ifndef LDO_HPP
#define LDO_HPP

#include "ad5293.hpp"
#include "hal/gpio.hpp"
#include "hal/i2c.hpp"
#include "hal/spi.hpp"
#include "mcp45HV51.hpp"
#include "units.hpp"
namespace psm {
class Ldo {
public:
  constexpr Ldo(hal::spi::Master &spi, hal::i2c::Master &i2c,
                hal::gpio::Output &wlat, hal::gpio::Output &shdn);
  enum Error {};

  Error enable();
  Error disable();

  Error set_voltage(Voltage v);
  Error set_current(Current i);

  Voltage get_voltage();
  Current get_current();

private:
  MCP45HV51 i_set_resistor_;
  AD5293 v_set_resistor_;
};

} // namespace psm
#endif
