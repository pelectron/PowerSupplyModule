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
  invalid_param,
  not_implemented,
  buffer_overflow,
  bus_error,
  crc_error,
  frame_error,
  parity_error,
  already_in_use,
  clock_error
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
  invalid_data_size,
  invalid_parity,
  invalid_stop_bits
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

enum Pins : std::uint16_t {
  Pin0,
  Pin1,
  Pin2,
  Pin3,
  Pin4,
  Pin5,
  Pin6,
  Pin7,
  Pin8,
  Pin9,
  Pin10,
  Pin11,
  Pin12,
  Pin13,
  Pin14,
  Pin15
};

enum class Id : std::uint32_t { invalid = 0 };

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

constexpr Id operator|(Port p, std::uint16_t pin_num) noexcept {
  return static_cast<Id>(static_cast<std::uint16_t>(p) << 16u |
                         static_cast<std::uint16_t>(pin_num));
}

constexpr Port port(Id pin) noexcept {
  return static_cast<Port>(static_cast<std::uint16_t>(pin) >> 16u);
}

constexpr std::uint16_t pins(Id pin) noexcept {
  return static_cast<std::uint16_t>(pin);
}

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

enum class Feature {
  msb_first,
  data_inversion,
  tx_inversion,
  rx_inversion,
  tx_rx_swap,
  auto_baudrate,
  wakeup
};

constexpr Feature operator|(Feature f1, Feature f2) {
  return static_cast<Feature>(static_cast<std::uint32_t>(f1) |
                              static_cast<std::uint32_t>(f2));
}

constexpr Feature operator&(Feature f1, Feature f2) {
  return static_cast<Feature>(static_cast<std::uint32_t>(f1) &
                              static_cast<std::uint32_t>(f2));
}

} // namespace uart

namespace adc {

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

enum ChannelId : std::uint16_t {
  invalid = 0,
  Channel1 = 1 << 0,
  Channel2 = 1 << 1,
  Channel3 = 1 << 2,
  Channel4 = 1 << 3,
  Channel5 = 1 << 4,
  Channel6 = 1 << 5,
  Channel7 = 1 << 6,
  Channel8 = 1 << 7,
  Channel9 = 1 << 8,
  Channel10 = 1 << 9,
  Channel11 = 1 << 10,
  Channel12 = 1 << 11,
  Channel13 = 1 << 12,
  Channel14 = 1 << 13,
  Channel15 = 1 << 14,
  Channel16 = 1 << 15,
  AllChannels = 0xFFFFu,
};

enum class Options : std::uint8_t {
  discontinous =
      0, // the adc performs one conversion cycle when started. One conversion
         // cycle means one sample operation in single_conversion or
         // sampling all channels in scan_conversion
  continous = 1 << 0, // the adc starts sampling when started and does not stop
                      // until stop is called.
  single_conversion = 0, // only a single channel is sampled at a time
  scan_conversion =
      1 << 1, // all enabled channels are sampled one after another
};

constexpr Options operator|(Options m1, Options m2) {
  return static_cast<Options>(static_cast<std::uint8_t>(m1) |
                              static_cast<std::uint8_t>(m2));
}

constexpr Options operator&(Options m1, Options m2) {
  return static_cast<Options>(static_cast<std::uint8_t>(m1) &
                              static_cast<std::uint8_t>(m2));
}
} // namespace adc

namespace clock {

enum class Source {
  high_speed_internal,
  low_speed_internal,
  high_speed_external,
  low_speed_external,
  peripheral_external
};

}

enum class Peripheral {
  flash,
  pwr,
  crc,
  dma,
  dmamux,
  nvic,
  exti,
  adc_a,
  adc_b,
  adc_c,
  adc_d,
  adc_e,
  adc_f,
  adc_g,
  adc_h,
  adc_i,
  adc_j,
  adc_k,
  adc_l,
  adc_m,
  adc_n,
  adc_o,
  adc_p,
  adc_q,
  adc_r,
  adc_s,
  adc_t,
  adc_u,
  adc_v,
  adc_w,
  adc_x,
  adc_y,
  adc_z,
  adc_all,
  gpio_a,
  gpio_b,
  gpio_c,
  gpio_d,
  gpio_e,
  gpio_f,
  gpio_g,
  gpio_h,
  gpio_i,
  gpio_j,
  gpio_k,
  gpio_l,
  gpio_m,
  gpio_n,
  gpio_o,
  gpio_p,
  gpio_q,
  gpio_r,
  gpio_s,
  gpio_t,
  gpio_u,
  gpio_v,
  gpio_w,
  gpio_x,
  gpio_y,
  gpio_z,
  gpio_all,
  i2c_a,
  i2c_b,
  i2c_c,
  i2c_d,
  i2c_e,
  i2c_f,
  i2c_g,
  i2c_h,
  i2c_i,
  i2c_j,
  i2c_k,
  i2c_l,
  i2c_m,
  i2c_n,
  i2c_o,
  i2c_p,
  i2c_q,
  i2c_r,
  i2c_s,
  i2c_t,
  i2c_u,
  i2c_v,
  i2c_w,
  i2c_x,
  i2c_y,
  i2c_z,
  i2c_all,
  spi_a,
  spi_b,
  spi_c,
  spi_d,
  spi_e,
  spi_f,
  spi_g,
  spi_h,
  spi_i,
  spi_j,
  spi_k,
  spi_l,
  spi_m,
  spi_n,
  spi_o,
  spi_p,
  spi_q,
  spi_r,
  spi_s,
  spi_t,
  spi_u,
  spi_v,
  spi_w,
  spi_x,
  spi_y,
  spi_z,
  spi_all,
  tim_a,
  tim_b,
  tim_c,
  tim_d,
  tim_e,
  tim_f,
  tim_g,
  tim_h,
  tim_i,
  tim_j,
  tim_k,
  tim_l,
  tim_m,
  tim_n,
  tim_o,
  tim_p,
  tim_q,
  tim_r,
  tim_s,
  tim_t,
  tim_u,
  tim_v,
  tim_w,
  tim_x,
  tim_y,
  tim_z,
  tim_all,
  uart_a,
  uart_b,
  uart_c,
  uart_d,
  uart_e,
  uart_f,
  uart_g,
  uart_h,
  uart_i,
  uart_j,
  uart_k,
  uart_l,
  uart_m,
  uart_n,
  uart_o,
  uart_p,
  uart_q,
  uart_r,
  uart_s,
  uart_t,
  uart_u,
  uart_v,
  uart_w,
  uart_x,
  uart_y,
  uart_z,
  uart_all,
  iwdg,
  wwdg,
  rtc,
  dbg
};

} // namespace hal
#endif
