#include "mock/hal/gpio.hpp"
#include "hal/config.hpp"

namespace hal::gpio {
ConfigResult<Pin> configure(const Config &cfg) noexcept {
  return Pin(hal::gpio::MockHandle{cfg.port, 0}, cfg.pins);
}

bool pin_exists(Id id) noexcept { return true; }
} // namespace hal::gpio
