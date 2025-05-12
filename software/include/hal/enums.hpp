/**
 * @file hal/enums.hpp
 * @brief
 */

#ifndef HAL_ENUMS_HPP
#define HAL_ENUMS_HPP
#include <cstdint>

namespace hal {

enum class Error {
  none,
  invalid_handle,
  config,
  callback_already_registered,
  invalid_param
};

enum class ConfigError : std::uint32_t {
  success,
  already_locked,
  invalid_config,
  invalid_port,
  invalid_pin_nr,
  invalid_function,
  invalid_mode,
  invalid_speed,
  invalid_pull,
  invalid_state,
  invalid_alternate,
  invalid_enumerator,
  invalid_phase,
  invalid_polarity,
  invalid_format,
  invalid_baudrate,
  invalid_id,
  invalid_sclk,
  invalid_mosi,
  invalid_miso,
  invalid_cs,
};

namespace gpio {

enum class Port : uint8_t {
  none,
  A,
  B,
  C,
  D,
  E,
  F,
  G,
  H,
  I,
  J,
  K,
  L,
  M,
  N,
  O,
  P,
  Q,
  R,
  S,
  T,
  U,
  V,
  W,
  X,
  Y,
  Z
};

enum class Id : uint16_t { invalid = 0 };

enum class State { reset, set, x };

enum class Function : std::uint8_t { input, output, analog, alternate };

enum class Mode : std::uint8_t { none, push_pull, open_drain };

enum class Speed : std::uint8_t { none, slow, medium, fast, very_fast };

enum class Pull : std::uint8_t { none, up, down };

enum class AlternateFunction {
  none,
  rx,
  tx,
  sclk,
  mosi,
  miso,
  nss,
};

constexpr Id operator|(Port p, uint8_t pin_num) noexcept {
  return static_cast<Id>(static_cast<uint16_t>(p) << 8 |
                         static_cast<uint16_t>(pin_num));
}

constexpr Port port(Id pin) noexcept {
  return static_cast<Port>(
      static_cast<uint8_t>(static_cast<uint16_t>(pin) >> 8));
}

constexpr uint8_t pin_nr(Id pin) noexcept { return static_cast<uint8_t>(pin); }

constexpr State operator!(State s) {
  return s == State::set ? State::reset : State::set;
}
} // namespace gpio

namespace spi {

enum class Id {
  invalid = 0,
  A,
  B,
  C,
  D,
  E,
  F,
  G,
  H,
  I,
  J,
  K,
  L,
  M,
  N,
  O,
  P,
  Q,
  R,
  S,
  T,
  U,
  V,
  W,
  X,
  Y,
  Z
};

enum class Phase { low, high };

enum class Polarity { low, high };

enum class Format { lsb_first, msb_first };

enum class Crc { none, eight_bit, sixteen_bit };

enum class Error {
  success = 0,
  busy = 1 << 1,
  rx_overrun = 1 << 1,
  mode_fault = 1 << 2,
  crc_error = 1 << 3,
  frame_error = 1 << 4,
};

constexpr Error operator|(Error a, Error b) {
  return static_cast<Error>(static_cast<unsigned>(a) |
                            static_cast<unsigned>(b));
}

} // namespace spi

namespace i2c {

enum class Id : std::uint8_t {
  invalid = 0,
  A,
  B,
  C,
  D,
  E,
  F,
  G,
  H,
  I,
  J,
  K,
  L,
  M,
  N,
  O,
  P,
  Q,
  R,
  S,
  T,
  U,
  V,
  W,
  X,
  Y,
  Z
};

}

namespace uart {

enum class Id : std::uint8_t {
  invalid = 0,
  A,
  B,
  C,
  D,
  E,
  F,
  G,
  H,
  I,
  J,
  K,
  L,
  M,
  N,
  O,
  P,
  Q,
  R,
  S,
  T,
  U,
  V,
  W,
  X,
  Y,
  Z
};

enum class Bits { seven, eight, nine };

enum class StopBits { one, one_and_a_half, two };

enum class Parity { none, odd, even };

} // namespace uart
} // namespace hal
#endif
