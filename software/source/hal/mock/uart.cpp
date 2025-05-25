#include "mock/hal/uart.hpp"
#include "hal/uart.hpp"

namespace hal::uart {

ConfigResult<HandleOwner> configure(const Config &cfg) noexcept {
  return HandleOwner(TerminalHandle{});
}
} // namespace hal::uart
