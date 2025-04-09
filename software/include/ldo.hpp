#ifndef LDO_HPP
#define LDO_HPP

#include "ad5293.hpp"
#include "mcp45HV51.hpp"
#include "units.hpp"
namespace psm {
class LDO {
public:
  enum Error {};

  Error enable();
  Error disable();

  Error set_voltage(Voltage v);
  Error set_current(Current i);

  Voltage get_voltage();
  Current get_current();

private:
  Voltage v_set_;
  Current i_set_;
  MCP45HV51 i_set_resistor_;
  AD5293 v_set_resistor_;
};

} // namespace psm
#endif
