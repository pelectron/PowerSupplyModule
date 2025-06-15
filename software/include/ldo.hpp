#ifndef LDO_HPP
#define LDO_HPP

#include "drivers/ad5293.hpp"
#include "drivers/mcp45hvx1.hpp"
#include "hal/gpio.hpp"
#include "hal/i2c.hpp"
#include "hal/spi.hpp"
#include "units.hpp"
namespace psm {
class Ldo {
public:
  constexpr Ldo(hal::spi::Device spi, hal::i2c::Device i2c,
                hal::gpio::Output wlat, hal::gpio::Output shdn);
  Error enable();
  Error disable();

  Error set_voltage(Voltage v);
  Error set_current(Current i);

  Voltage get_voltage();
  Current get_current();

private:
  mcp45hvx1::MCP45HVX1 i_set_resistor_;
  ad5293::AD5293 v_set_resistor_;
};

} // namespace psm
#endif
