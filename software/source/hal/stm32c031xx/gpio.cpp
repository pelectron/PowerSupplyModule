#include "hal/gpio.hpp"
#include "clocks.hpp"
#include "hal/config.hpp"
#include "hal/enums.hpp"
#include <cstdint>

#define GPIOA 0x50000000UL
#define GPIOB 0x50000400UL
#define GPIOC 0x50000800UL
#define GPIOD 0x50000C00UL
#define GPIOF 0x50001400UL

using namespace hal;
using namespace hal::gpio;

struct Gpio {
  volatile std::uint32_t
      MODER; /*!< GPIO port mode register, Address offset: 0x00 */
  volatile std::uint32_t
      OTYPER; /*!< GPIO port output type register, Address offset: 0x04*/
  volatile std::uint32_t
      OSPEEDR; /*!< GPIO port output speed register, Address offset: 0x08*/
  volatile std::uint32_t
      PUPDR; /*!< GPIO port pull-up/pull-down register, Address offset: 0x0C */
  volatile std::uint32_t
      IDR; /*!< GPIO port input data register, Address offset: 0x10 */
  volatile std::uint32_t
      ODR; /*!< GPIO port output data register, Address offset: 0x14 */
  volatile std::uint32_t
      BSRR; /*!< GPIO port bit set/reset  register, Address offset: 0x18 */
  volatile std::uint32_t
      LCKR; /*!< GPIO port configuration lock register, Address offset: 0x1C */
  volatile std::uint32_t AFR[2]; /*!< GPIO alternate function registers, Address
                               offset: 0x20-0x24 */
  volatile std::uint32_t
      BRR; /*!< GPIO Bit Reset register,               Address offset: 0x28 */

  std::uint32_t get_state(unsigned pins) {
    return ((volatile Gpio *)this)->IDR & pins;
  }

  void set_state(unsigned pins, State state) {
    if (state == State::set) {
      ((volatile Gpio *)this)->BSRR = pins;
    } else {
      ((volatile Gpio *)this)->BSRR = pins << 16u;
    }
  }

  void toggle_state(unsigned pins) {
    if ((((volatile Gpio *)this)->IDR & pins) == 0u)
      ((volatile Gpio *)this)->BSRR = pins;
    else
      ((volatile Gpio *)this)->BSRR = pins << 16u;
  }
};

Gpio *port_io(Port port) {
  switch (port) {
  case Port::A:
    return reinterpret_cast<Gpio *>(GPIOA);
  case Port::B:
    return reinterpret_cast<Gpio *>(GPIOB);
  case Port::C:
    return reinterpret_cast<Gpio *>(GPIOC);
  case Port::F:
    return reinterpret_cast<Gpio *>(GPIOF);
  default:
    break;
  }
  return nullptr;
}

ConfigResult<Pin> hal::gpio::configure(const Config &cfg) noexcept {
  Gpio *port = nullptr;
  switch (hal::gpio::port(cfg.pins)) {
  case Port::A:
    stm32c031xx::clock_tree.enable(hal::Peripheral::gpio_a);
    port = reinterpret_cast<Gpio *>(GPIOA);
    break;
  case Port::B:
    stm32c031xx::clock_tree.enable(hal::Peripheral::gpio_b);
    port = reinterpret_cast<Gpio *>(GPIOB);
    break;
  case Port::C:
    stm32c031xx::clock_tree.enable(hal::Peripheral::gpio_c);
    port = reinterpret_cast<Gpio *>(GPIOC);
    break;
  case Port::D:
    break;
  case Port::F:
    stm32c031xx::clock_tree.enable(hal::Peripheral::gpio_f);
    port = reinterpret_cast<Gpio *>(GPIOF);
    break;
  default:
    return ConfigError::invalid_port;
  }

  if (port == nullptr)
    return ConfigError::invalid_port;

  if ((port->LCKR & (1u << 16)) == 1u << 16)
    return ConfigError::already_locked;

  // mask for single bit fields
  const std::uint32_t mask = hal::gpio::pins(cfg.pins);

  // mask for 2 bit wide fields
  std::uint32_t mask2 = 0;

  // pins contains the actual pin numbers
  std::uint32_t pins[16] = {};
  std::uint32_t num_pins = 0;
  for (std::uint32_t i = 0; i < 16; ++i) {
    if ((1u << i) & mask) {
      pins[num_pins++] = i;
      mask2 |= 0b11u << (2u * i);
    }
  }

  if (num_pins == 0)
    return ConfigError::invalid_pin_nr;

  std::uint32_t moder = 0;
  std::uint32_t otyper = 0;
  std::uint32_t ospeedr = 0;
  std::uint32_t pupdr = 0;
  std::uint32_t afrl = 0;
  std::uint32_t afrh = 0;
  std::uint32_t afrl_mask = 0;
  std::uint32_t afrh_mask = 0;

  // set the output mode
  switch (cfg.mode) {
  case Mode::none:
    [[fallthrough]];
  case Mode::push_pull:
    break;
  case Mode::open_drain:
    otyper = mask;
    break;
  default:
    return ConfigError::invalid_mode;
  }

  for (std::uint32_t i = 0; i < num_pins; ++i) {
    // set the general function
    switch (cfg.function) {
    case Function::input:
      break;
    case Function::output:
      moder |= 0b01u << pins[i] * 2;
      break;
    case Function::alternate:
      moder |= 0b10u << pins[i] * 2;
      break;
    case Function::analog:
      moder |= 0b11u << pins[i] * 2;
      break;
    default:
      return ConfigError::invalid_function;
    }

    // set the speed
    switch (cfg.speed) {
    case Speed::none:
      [[fallthrough]];
    case Speed::slow:
      break;
    case Speed::medium:
      ospeedr |= 0b01u << pins[i] * 2;
      break;
    case Speed::fast:
      ospeedr |= 0b10u << pins[i] * 2;
      break;
    case Speed::very_fast:
      ospeedr |= 0b11u << pins[i] * 2;
      break;
    default:
      return ConfigError::invalid_speed;
    }

    // set pull up / pull down resistors
    // TODO: check that on the same pin, this pull up/down must not be
    // activated when a pull down/up is set through the PWR_PDCRx/PWR_PUCRx
    // registers.
    switch (cfg.pull) {
    case Pull::none:
      break;
    case Pull::up:
      pupdr |= 0b01u << pins[i] * 2;
      break;
    case Pull::down:
      pupdr |= 0b10u << pins[i] * 2;
      break;
    default:
      return ConfigError::invalid_pull;
    }

    if (cfg.function == Function::output and cfg.state == State::x)
      return ConfigError::invalid_state;

    if (cfg.function == Function::alternate) {
      if (cfg.alternate > 15u)
        return ConfigError::invalid_alternate;

      if (pins[i] < 8) {
        const uint32_t pos = 4 * pins[i];
        afrl |= cfg.alternate << pos;
        afrl_mask |= 0x0000000Fu << pos;
      } else {
        const uint32_t pos = 4 * (pins[i] - 8);
        afrh |= cfg.alternate << pos;
        afrh_mask |= 0x0000000Fu << pos;
      }
    }
  }

  // actually apply configuration
  port->MODER = (port->MODER & ~mask2) | moder;
  port->OTYPER = (port->OTYPER & ~mask2) | otyper;
  port->OSPEEDR = (port->OSPEEDR & ~mask2) | ospeedr;
  port->PUPDR = (port->PUPDR & ~mask2) | pupdr;
  port->AFR[0] = (port->AFR[0] & ~afrl_mask) | afrl;
  port->AFR[1] = (port->AFR[1] & ~afrh_mask) | afrh;

  return Pin{*port, mask};
}
