#include "mock/hal/i2c.hpp"
#include "hal/config.hpp"
#include "hal/i2c.hpp"

namespace hal::i2c {
ConfigResult<HandleOwner> configure(const Config &cfg) noexcept {
  return HandleOwner{MockHandle{cfg}};
}

} // namespace hal::i2c
