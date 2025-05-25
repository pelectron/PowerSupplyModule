#ifndef HAL_CONFIG_HPP
#define HAL_CONFIG_HPP

#include "hal/enums.hpp"
#include "units.hpp"

#include <array>
#include <utility>

namespace hal {

template <typename Peripheral> struct ConfigResult {
  ConfigResult() = delete;
  constexpr ConfigResult(const ConfigResult &) = default;
  constexpr ConfigResult(ConfigResult &&) = default;
  constexpr ConfigResult(ConfigError error) : error(error), peripheral() {}
  constexpr ConfigResult(Peripheral &&p)
      : error(ConfigError::success), peripheral(std::move(p)) {}
  constexpr ConfigResult &operator=(const ConfigResult &) = default;
  constexpr ConfigResult &operator=(ConfigResult &&) = default;
  constexpr operator bool() const noexcept {
    return error == ConfigError::success;
  }

  ConfigError error;
  Peripheral peripheral;
};

namespace adc {

struct ChannelConfig {
  ChannelId channel = ChannelId::invalid;
  gpio::Id inp = gpio::Id::invalid;
  gpio::Id inn = gpio::Id::invalid;
  Voltage offset{};
  signed_fixed<16, 16> gain{1_sf};
};

struct Config {
  Id id{};
  ChannelId channels = ChannelId::invalid;
  Options options{};
  uint8_t num_bits{};
  uint32_t clock_rate{};
  uint32_t oversampling{};
  std::array<ChannelConfig, 16> channel_configs{};
};
} // namespace adc

namespace clock {
struct Config {
  Source source;
  Frequency frequency;
};
} // namespace clock

namespace gpio {

/// general gpio configuration structure
struct Config {
  Id pins;           // specifies the pins to be configured.
  Function function; //< specifies input, output, or alternate
  Mode mode;         //< specifies driver mode when output
  Speed speed;       //< specifies drive strength
  Pull pull;         //< specifies pull up, pull down or no pull up
  State state;       //< initial state of the pin
  unsigned alternate{0};
  constexpr auto operator<=>(const Config &) const = default;
};

/// gpio input configuration structure. Defaults to slow input without pull up
/// or pull down. Converts implicitly to gpio::Config.
struct InputConfig {
  Id pins;               // specifies the pins to be configured.
  Pull pull{Pull::none}; //< specifies pull up, pull down or no pull up
  unsigned alternate{0};

  constexpr operator Config() const {
    return {pins, Function::input, Mode::none, Speed::none,
            pull, State::x,        alternate};
  }
};

/// gpio output configuration structure. Defaults to a slow, push-pull output
/// without pull up/down resistor and reset initial state (0V). Converts
/// implicitly to gpio::Config.
struct OutputConfig {
  Id pins;                    // specifies the pins to be configured.
  Mode mode{Mode::push_pull}; //< specifies driver mode
  Speed speed{Speed::slow};   //< specifies drive strength
  Pull pull{Pull::none};      //< specifies pull up/down or no pull up
  State state{State::reset};  //< initial state of the pin
  unsigned alternate{0};

  constexpr operator Config() const {
    return {pins, Function::output, mode, speed, pull, state, alternate};
  }
};

} // namespace gpio

namespace spi {

struct Config {
  Id id;
  hal::gpio::Id sclk;
  hal::gpio::Id mosi;
  hal::gpio::Id miso;
  hal::gpio::Id cs;
  Phase phase = Phase::low;
  Polarity polarity = Polarity::low;
  Format format = Format::msb_first;
  Frequency baudrate = au::hertz(1'000'000u);
  uint8_t data_size = 8;
  bool use_hw_cs = false;
  Crc crc = Crc::none;
  std::uint32_t crc_polynomial = 0;
  bool three_wire = false;
  bool cs_pulse;
  constexpr auto operator<=>(const Config &) const = default;
};

} // namespace spi

namespace i2c {

struct Config {
  Id id;
  uint8_t address;
  hal::gpio::Id scl;
  hal::gpio::Id sda;
  constexpr auto operator<=>(const Config &) const = default;
};

} // namespace i2c

namespace uart {

struct Config {
  Id id;
  std::uint32_t baudrate;
  Bits bits;
  StopBits stop_bits;
  Parity parity;
  Feature features;

  constexpr bool has_feature(Feature f) const noexcept {
    return (features & f) == f;
  }

  constexpr auto operator<=>(const Config &) const = default;
};

} // namespace uart

} // namespace hal
#endif
