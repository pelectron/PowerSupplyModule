#ifndef PSM_TPS55288_HPP
#define PSM_TPS55288_HPP

#include "converter.hpp"
#include "hal/gpio.hpp"
#include "hal/i2c.hpp"

namespace psm {
inline namespace tps55288 {
struct TPS55288 {

  constexpr TPS55288(hal::i2c::Master &i2c, hal::gpio::Output &enbale,
                     hal::gpio::Output &cdc);
  hal::i2c::Master &i2c;
  hal::gpio::Output enable;
  hal::gpio::Output cdc;
  static constexpr conv::Specs specs{
      .v_min{0.8f}, .v_max{22.0f}, .i_min{0.0f}, .i_max{10.0f}};
};

bool get(conv::enabled, TPS55288 &conv) {}

void set(conv::enabled, TPS55288 &conv, bool en) {}

Voltage get(conv::voltage, TPS55288 &conv) {}

void set(conv::voltage, TPS55288 &conv, Voltage v) {}

Current get(conv::current, TPS55288 &conv) {}

void set(conv::current, TPS55288 &conv, Current i) {}

Celcius get(conv::temperature, TPS55288 &conv) {}
const conv::Specs &get(conv::specs, TPS55288 &conv) {}
} // namespace tps55288
} // namespace psm

#endif
