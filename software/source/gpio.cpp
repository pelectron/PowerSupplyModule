#include "hal/gpio.hpp"
#include "hal/config.hpp"
#include <cstdint>

#define GPIOA 0x50000000UL
#define GPIOB 0x50000400UL
#define GPIOC 0x50000800UL
#define GPIOD 0x50000C00UL
#define GPIOF 0x50001400UL

namespace hal::gpio {

struct port_type {
  volatile uint32_t
      MODER; /*!< GPIO port mode register,               Address offset: 0x00 */
  volatile uint32_t OTYPER;  /*!< GPIO port output type register,        Address
                                offset: 0x04      */
  volatile uint32_t OSPEEDR; /*!< GPIO port output speed register,       Address
                                offset: 0x08      */
  volatile uint32_t
      PUPDR; /*!< GPIO port pull-up/pull-down register,  Address offset: 0x0C */
  volatile uint32_t
      IDR; /*!< GPIO port input data register,         Address offset: 0x10 */
  volatile uint32_t
      ODR; /*!< GPIO port output data register,        Address offset: 0x14 */
  volatile uint32_t
      BSRR; /*!< GPIO port bit set/reset  register,     Address offset: 0x18 */
  volatile uint32_t
      LCKR; /*!< GPIO port configuration lock register, Address offset: 0x1C */
  volatile uint32_t AFR[2]; /*!< GPIO alternate function registers,     Address
                               offset: 0x20-0x24 */
  volatile uint32_t
      BRR; /*!< GPIO Bit Reset register,               Address offset: 0x28 */
};

port_type *port_io(Id id) {
  switch (port(id)) {
  case Port::A:
    return reinterpret_cast<port_type *>(GPIOA);
  case Port::B:
    return reinterpret_cast<port_type *>(GPIOB);
  case Port::C:
    return reinterpret_cast<port_type *>(GPIOC);
  case Port::D:
    break;
  case Port::F:
    return reinterpret_cast<port_type *>(GPIOF);
  default:
    break;
  }
  return nullptr;
}

bool pin_exists(Id id) noexcept {
  const auto nr = pin_nr(id);
  switch (port(id)) {
  case Port::A:
    return nr < 16;
  case Port::B:
    return nr < 10;
  case Port::C:
    return nr == 6 or nr == 14 or nr == 15;
  case Port::D:
    return false;
  case Port::F:
    return pin_nr(id) < 16;
  default:
    break;
  }
  return false;
}

ConfigResult<Pin> configure(const Config &cfg) noexcept {
  if (not pin_exists(cfg.id))
    return ConfigError::invalid_id;

  port_type *const port = port_io(cfg.id);
  // mask and position for single bit fields
  const std::uint32_t nr = pin_nr(cfg.id);
  const std::uint32_t mask = 1 << nr;
  // position and mask for 2 bit wide fields
  const std::uint32_t pos2 = 2 * nr;
  const std::uint32_t mask2 = 0b11 << pos2;

  if (port == nullptr)
    return ConfigError::invalid_port;

  if (nr > 15)
    return ConfigError::invalid_pin_nr;

  if ((port->LCKR & (1 << 16)) == 1 << 16)
    return ConfigError::already_locked;

  // set the general function
  std::uint32_t moder = 0;
  switch (cfg.function) {
  case Function::input:
    break;
  case Function::output:
    moder |= 0b01 << pos2;
    break;
  case Function::alternate:
    moder |= 0b10 << pos2;
    break;
  case Function::analog:
    moder |= 0b11 << pos2;
    break;
  default:
    return ConfigError::invalid_function;
  }

  std::uint32_t otyper = 0;
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

  // set the speed
  std::uint32_t ospeedr = 0;
  switch (cfg.speed) {
  case Speed::none:
    [[fallthrough]];
  case Speed::slow:
    break;
  case Speed::medium:
    ospeedr |= 0b01 << pos2;
    break;
  case Speed::fast:
    ospeedr |= 0b10 << pos2;
    break;
  case Speed::very_fast:
    ospeedr |= 0b11 << pos2;
    break;
  default:
    return ConfigError::invalid_speed;
  }

  // set pull up / pull down resistors
  std::uint32_t pupdr = 0;
  // TODO: check that on the same pin, this pull up/down must not be activated
  // when a pull down/up is set through the PWR_PDCRx/PWR_PUCRx registers.
  switch (cfg.pull) {
  case Pull::none:
    break;
  case Pull::up:
    pupdr |= 0b01 << pos2;
    break;
  case Pull::down:
    pupdr |= 0b10 << pos2;
    break;
  default:
    return ConfigError::invalid_pull;
  }

  if ((cfg.function == Function::output or
       cfg.function == Function::alternate) and
      cfg.state == State::x)
    return ConfigError::invalid_state;

  std::uint32_t afrl = 0;
  std::uint32_t afrh = 0;
  std::uint32_t afrl_mask = 0;
  std::uint32_t afrh_mask = 0;
  if (cfg.function == Function::alternate) {
    if ((cfg.alternate & 0xF) != cfg.alternate)
      return ConfigError::invalid_alternate;

    if (nr < 8) {
      const uint32_t pos = 4 * nr;
      afrl |= cfg.alternate << pos;
      afrl_mask |= 0xF << pos;
    } else {
      const uint32_t pos = 4 * (nr - 8);
      afrh |= cfg.alternate << pos;
      afrh_mask |= 0xF << pos;
    }
  }

  // actually apply configuration
  port->MODER = (port->MODER & ~mask2) | moder;
  port->OTYPER = (port->OTYPER & ~mask2) | otyper;
  port->OSPEEDR = (port->OSPEEDR & ~mask2) | ospeedr;
  port->PUPDR = (port->PUPDR & ~mask2) | pupdr;
  port->AFR[0] = (port->AFR[0] & ~afrl_mask) | afrl;
  port->AFR[1] = (port->AFR[1] & ~afrh_mask) | afrh;

  Pin pin;
  pin.port_ = port;
  pin.pin_ = mask;
  pin.id_ = cfg.id;

  if (cfg.function == Function::output or cfg.function == Function::alternate) {
    pin.set(cfg.state);
  }

  return pin;
}

void Pin::set(State state) {
  switch (state) {
  case State::reset:
    port_->BSRR |= pin_ << 16;
    break;
  case State::set:
    port_->BSRR |= pin_;
    break;
  default:
    break;
  }
}

void Pin::toggle() { port_->ODR = port_->ODR ^ pin_; }

State Pin::get() {
  if (port_->IDR & pin_)
    return State::set;
  return State::reset;
}

} // namespace hal::gpio
