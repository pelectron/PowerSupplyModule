#ifndef MOCK_HAL_GPIO_HPP
#define MOCK_HAL_GPIO_HPP

#include "cli/ctti.hpp"
#include "hal/gpio.hpp"
#include <cstdint>
#include <iostream>

namespace hal::gpio {

struct MockHandle {
  State get_state(unsigned pin_mask) {
    auto state = ((pin_mask & value) == 0) ? State::reset : State::set;
    std::cout << "gpio get  [" << cli::ctti::enum_name(port)
              << "] pin_mask: " << pin_mask
              << " state: " << cli::ctti::enum_name(state) << std::endl;
    return ((pin_mask & value) == 0) ? State::reset : State::set;
  }

  void set_state(unsigned pin_mask, State s) {
    if (s == State::set) {
      value |= pin_mask;
      std::cout << "gpio set  [P" << cli::ctti::enum_name(port)
                << "] pin_mask: " << pin_mask << std::endl;
    } else {
      value &= ~pin_mask;
      std::cout << "gpio reset  [P" << cli::ctti::enum_name(port)
                << "] pin_mask: " << pin_mask << std::endl;
    }
  }

  void toggle_state(unsigned pin_mask) {
    value ^= pin_mask;
    std::cout << "gpio toggle  [P" << cli::ctti::enum_name(port)
              << "] pin_mask: " << pin_mask << std::endl;
  }

  Port port;
  std::uint16_t value;
};

} // namespace hal::gpio

#endif
