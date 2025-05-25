#ifndef HAL_PIN_HPP
#define HAL_PIN_HPP

#include "hal/config.hpp"
#include "hal/enums.hpp"
#include "hal/operations.hpp"
#include "poly.hpp"
#include "poly/fwd.hpp"
#include "poly/storage/ref_storage.hpp"

#include <cstdint>
#include <utility>
namespace hal::gpio {

POLY_METHOD(get_state)
POLY_METHOD(set_state)
POLY_METHOD(toggle_state)

struct NullHandle {
  std::uint32_t get_state(unsigned) { return 0; }

  void set_state(unsigned, State) {}

  void toggle_state(unsigned) {}
};

using HandleRef =
    poly::Struct<poly::ref_storage, poly::type_list<>,
                 poly::type_list<std::uint32_t(get_state, unsigned pins),
                                 void(set_state, unsigned pins, State state),
                                 void(toggle_state, unsigned pins)>>;

using InputHandle =
    poly::Struct<poly::ref_storage, poly::type_list<>,
                 poly::type_list<State(get_state, unsigned pin_mask)>>;

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
  constexpr Pin(HandleRef handle, uint32_t pin_mask)
      : handle_(handle), pin_mask_(pin_mask) {}
  constexpr Pin(const Pin &) = default;
  constexpr Pin(Pin &&other) = default;
  constexpr Pin &operator=(const Pin &) = default;
  constexpr Pin &operator=(Pin &&other) = default;
  /**
   * returns true if the pin is valid. Using the other methods with an invalid
   * pin invokes undefined behaviour.
   */
  constexpr bool is_valid() const noexcept { return handle_; }

  /**
   * sets the state of a pin configured as output.
   * @param state State::set or State::reset
   */
  constexpr void set(State state) { handle_.set_state(pin_mask_, state); }

  /**
   * toggle the state of a pin configured as output.
   */
  constexpr void toggle() { handle_.toggle_state(pin_mask_); }

  /**
   * get the state of a pin.
   */
  constexpr State get() {
    return handle_.get_state(pin_mask_) == 0 ? State::reset : State::set;
  }

private:
  friend struct port_type;
  friend class Output;
  friend class Input;
  friend ConfigResult<Pin> configure(const Config &cfg) noexcept;

  HandleRef handle_{};
  uint32_t pin_mask_ = 0;
};

class Output : Pin {
public:
  using Pin::Pin;
  constexpr Output() noexcept = default;
  constexpr Output(Pin &&pin) noexcept : Pin(std::move(pin)) {}
  constexpr Output(const Output &) = delete;
  constexpr Output(Output &&) noexcept = default;
  constexpr Output &operator=(const Output &) = delete;
  constexpr Output &operator=(Output &&) noexcept = default;

  using Pin::get;
  using Pin::is_valid;
  using Pin::set;
  using Pin::toggle;

private:
  friend ConfigResult<Output> configure(const OutputConfig &cfg) noexcept;
};

class Input {
public:
  constexpr Input() noexcept = default;
  constexpr Input(InputHandle &&handle, uint32_t pin_mask)
      : handle_(std::move(handle)), pin_mask_(pin_mask) {}
  // constexpr Input(Pin &&pin) noexcept
  //     : handle_(std::move(pin.handle_)), pin_mask_(pin.pin_mask_) {}
  constexpr Input(const Input &) = delete;
  constexpr Input(Input &&) noexcept = default;
  constexpr Input &operator=(const Input &) = delete;
  constexpr Input &operator=(Input &&) noexcept = default;

  /**
   * returns true if the pin is valid. Using the other methods with an invalid
   * pin invokes undefined behaviour.
   */
  constexpr bool is_valid() const noexcept { return handle_; }

  /**
   * get the state of a pin.
   */
  State get() { return handle_.get_state(pin_mask_); }

private:
  InputHandle handle_{};
  std::uint32_t pin_mask_ = 0;

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
  return hal::ConfigError::invalid_config;
  // return Input(std::move(res.peripheral));
}

static constinit NullHandle null_handle{};
static constexpr Pin nullpin(null_handle, 0);
} // namespace hal::gpio
#endif
