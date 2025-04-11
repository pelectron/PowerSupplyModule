#ifndef HAL_UART_HPP
#define HAL_UART_HPP

#include "cortex/function.hpp"
#include "hal/config.hpp"
#include <concepts>
#include <cstdint>

namespace hal::uart {

enum class Delimiter { none, r, n, rn };

enum class Bits { seven, eight, nine };

enum class StopBits { one, one_and_a_half, two };

enum class Parity { none, odd, even };

struct Config {
  Delimiter delimiter;
  std::uint32_t baudrate;
};

class Uart {
public:
  enum Error {

  };

  Error put(const char *s, std::size_t size);

  template <std::invocable<const char *, std::size_t> OnMessage>
  void set_on_message(OnMessage &&on_msg) {
    on_msg_ = std::forward<OnMessage>(on_msg);
  }

private:
  cm::MoveOnlyFunction<void(char), 32, 4> on_msg_;
};

ConfigResult<Uart> configure(const Config &cfg);
} // namespace hal::uart

#endif
