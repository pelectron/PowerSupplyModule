#include "mock/hal/hal.hpp"

namespace hal {

Hal get_hal() noexcept {
  static MockHandle handle{};
  return {handle};
}
} // namespace hal
