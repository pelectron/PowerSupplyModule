#include "mock/hal/gpio.hpp"
#include "hal/config.hpp"

#include <map>
#include <vector>

namespace hal::gpio {
static std::map<hal::gpio::Port, MockHandle> handles;

ConfigResult<Pin> configure(const Config &cfg) noexcept {
  return Pin(handles[cfg.port], cfg.pins);
}

bool pin_exists(Id id) noexcept {
  return handles.contains(hal::gpio::port(id));
}
} // namespace hal::gpio
