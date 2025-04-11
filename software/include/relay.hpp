#ifndef RELAY_HPP
#define RELAY_HPP
#include "hal/gpio.hpp"
#include <cstdint>

namespace psm {

class Relay {
public:
  enum class Id : std::uint8_t {
    out_en_p = 0,
    out_en_n = 1,
    out_select = 2,
    out_series = 3,
    invalid = 0xFFu
  };

  enum class State { open = 0, closed = 1 };

  /**
   * Specifies the pin, the pin state to keep the relay open,
   * and the default status of the relay.
   */
  struct Settings {
    hal::gpio::Id pin;
    hal::gpio::State open_state;
    State default_state;
  };

  constexpr Relay() = default;

  /**
   * initialize the relay.
   *
   * @param settings relay settings
   * @return ConfigError::success, if initialization succeeded, else the error
   * source.
   */
  hal::ConfigError init(const Settings &settings) {
    using namespace hal;
    using namespace hal::gpio;
    auto res = hal::gpio::configure(hal::gpio::OutputConfig{
        .id = settings.pin,
        .mode = Mode::push_pull,
        .speed = Speed::slow,
        .pull = settings.open_state == hal::gpio::State::set ? Pull::up
                                                             : Pull::down,
        .state = settings.open_state});

    if (res.error != ConfigError::success)
      return res.error;

    pin_ = std::move(res.peripheral);
    settings_ = settings;

    switch (settings.default_state) {
    case State::open:
      open();
      break;
    case State::closed:
      close();
      break;
    }

    return ConfigError::success;
  }

  /**
   * open the relay
   */
  void open() {
    pin_.set(settings_.open_state);
    state_ = State::open;
  }

  /**
   * close the relay
   */
  void close() {
    pin_.set(not settings_.open_state);
    state_ = State::open;
  }

  /**
   * returns the state of the relay
   */
  constexpr State state() const noexcept { return state_; }

  /**
   * returns the relay settings
   */
  constexpr const Settings &settings() const noexcept { return settings_; }

private:
  hal::gpio::Output pin_{};
  Settings settings_{};
  State state_;
};

} // namespace psm
#endif
