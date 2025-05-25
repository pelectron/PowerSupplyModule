#include "mock/hal/spi.hpp"

namespace hal::spi {

ConfigResult<HandleOwner> configure(const Config &cfg) noexcept {
  return HandleOwner(MockHandle(cfg));
}

} // namespace hal::spi
