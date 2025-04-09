#ifndef HAL_PIN_HPP
#define HAL_PIN_HPP

#include "config.hpp"

#include <cstdint>
#include <utility>
namespace hal::gpio {

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

constexpr Id operator|(Port p, uint8_t pin_num) noexcept {
  return static_cast<Id>(static_cast<uint16_t>(p) << 8 |
                         static_cast<uint16_t>(pin_num));
}

constexpr Port port(Id pin) noexcept {
  return static_cast<Port>(
      static_cast<uint8_t>(static_cast<uint16_t>(pin) >> 8));
}

constexpr uint8_t pin_nr(Id pin) noexcept { return static_cast<uint8_t>(pin); }

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

constexpr State operator!(State s) {
  return s == State::set ? State::reset : State::set;
}

/// general gpio configuration structure
struct Config {
  Id id;             //< pin to configure
  Function function; //< specifies input, output, or alternate
  Mode mode;         //< specifies driver mode when output
  Speed speed;       //< specifies drive strength or acquisition speed
  Pull pull;         //< specifies pull up, pull down or no pull up
  State state;       //< initial state of the pin
  unsigned alternate{0};
};

/// gpio input configuration structure. Defaults to slow input without pull up
/// or pull down. Converts implicitly to gpio::Config.
struct InputConfig {
  Id id;                 //< pin to configure
  Pull pull{Pull::none}; //< specifies pull up, pull down or no pull up
  unsigned alternate{0};

  constexpr operator Config() const {
    return {id, Function::input, Mode::none, Speed::none, pull, State::x, 0};
  }
};

/// gpio output configuration structure. Defaults to a slow, push-pull output
/// without pull up/down resistor and reset initial state (0V). Converts
/// implicitly to gpio::Config.
struct OutputConfig {
  Id id;                      //< pin to configure
  Mode mode{Mode::push_pull}; //< specifies driver mode
  Speed speed{Speed::slow};   //< specifies drive strength
  Pull pull{Pull::none};      //< specifies pull up/down or no pull up
  State state{State::reset};  //< initial state of the pin
  unsigned alternate{0};

  constexpr operator Config() const {
    return {id, Function::output, mode, speed, pull, state, 0};
  }
};

struct AlternateFunctionConfig {
  Mode mode{Mode::push_pull}; //< specifies driver mode
  Speed speed{Speed::slow};   //< specifies drive strength
  Pull pull{Pull::none};      //< specifies pull up/down or no pull up
  State state{State::reset};  //< initial state of the pin
  unsigned alternate{0};
};

struct port_type;

class Pin {
public:
  constexpr Pin() noexcept = default;
  constexpr Pin(const Pin &) = delete;
  constexpr Pin(Pin &&other) noexcept
      : port_(other.port_), pin_(other.pin_), id_(other.id_) {
    other.port_ = nullptr;
    other.pin_ = 0;
    other.id_ = Id::invalid;
  }
  constexpr Pin &operator=(const Pin &) = delete;
  constexpr Pin &operator=(Pin &&other) noexcept {
    std::exchange(port_, other.port_);
    std::exchange(pin_, other.pin_);
    std::exchange(id_, other.id_);
    return *this;
  }

  /**
   * returns true if the pin is valid. Using the other methods with an invalid
   * pin invokes undefined behaviour.
   */
  constexpr bool is_valid() const noexcept { return port_ != nullptr; }

  /**
   * returns the pins id
   */
  constexpr Id id() const noexcept { return id_; }

  /**
   * sets the state of a pin configured as output.
   * @param state State::set or State::reset
   */
  void set(State state);

  /**
   * toggle the state of a pin configured as output.
   */
  void toggle();

  /**
   * get the state of a pin.
   */
  State get();

private:
  friend struct port_type;
  friend class Output;
  friend class Input;
  friend ConfigResult<Pin> configure(const Config &cfg) noexcept;

  port_type *port_ = nullptr;
  uint32_t pin_ = 0;
  Id id_ = Id::invalid;
};

class Output : Pin {
public:
  constexpr Output() noexcept = default;
  constexpr Output(const Output &) = delete;
  constexpr Output(Output &&) noexcept = default;
  constexpr Output &operator=(const Output &) = delete;
  constexpr Output &operator=(Output &&) noexcept = default;

  using Pin::id;
  using Pin::is_valid;
  using Pin::set;
  using Pin::toggle;

private:
  constexpr Output(Pin &&pin) noexcept : Pin(std::move(pin)) {}
  friend ConfigResult<Output> configure(const OutputConfig &cfg) noexcept;
};

class Input : Pin {
public:
  constexpr Input() noexcept = default;
  constexpr Input(const Input &) = delete;
  constexpr Input(Input &&) noexcept = default;
  constexpr Input &operator=(const Input &) = delete;
  constexpr Input &operator=(Input &&) noexcept = default;

  using Pin::get;
  using Pin::id;
  using Pin::is_valid;

private:
  constexpr Input(Pin &&pin) noexcept : Pin(std::move(pin)) {}

  friend ConfigResult<Input> configure(const InputConfig &cfg) noexcept;
};

ConfigResult<Pin> configure(const Config &cfg) noexcept;
bool pin_exists(Id id) noexcept;
inline ConfigResult<Output> configure(const OutputConfig &cfg) noexcept {
  ConfigResult<Pin> res = configure(static_cast<Config>(cfg));
  if (res.error != ConfigError::success) {
    return res.error;
  }
  return Output(std::move(res.peripheral));
}

inline ConfigResult<Input> configure(const InputConfig &cfg) noexcept {
  ConfigResult<Pin> res = configure(static_cast<Config>(cfg));
  if (res.error != ConfigError::success) {
    return res.error;
  }
  return Input(std::move(res.peripheral));
}
} // namespace hal::gpio
#endif
