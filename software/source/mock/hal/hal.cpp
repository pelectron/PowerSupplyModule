#include "mock/hal/hal.hpp"

namespace hal {

Hal get_hal() noexcept {
  static MockHal handle{};
  return {handle};
}
} // namespace hal
