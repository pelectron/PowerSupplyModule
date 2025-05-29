#include "hal/i2c.hpp"
#include "clocks.hpp"
#include "hal/config.hpp"
#include "hal/enums.hpp"
#include "hal/gpio.hpp"

#define I2C1 reinterpret_cast<stm32c031xx::I2c *>(0x40005400u)
#define I2C2 reinterpret_cast<stm32c031xx::I2c *>(0x40005800u)

namespace stm32c031xx {
enum I2C_CR1 : std::uint32_t {
  I2C_CR1_PECEN = 1u << 23u,
  I2C_CR1_ALERTEN = 1u << 22u,
  I2C_CR1_SMBDEN = 1u << 21u,
  I2C_CR1_SMBHEN = 1u << 20u,
  I2C_CR1_I2C_CR1_GCEN = 1u << 19u,
  I2C_CR1_WUPEN = 1u << 18u,
  I2C_CR1_NOSTRETCH = 1u << 17u,
  I2C_CR1_SBC = 1u << 16u,
  I2C_CR1_RXDMAEN = 1u << 15u,
  I2C_CR1_TXDMAEN = 1u << 14u,
  I2C_CR1_ANFOFF = 1u << 13u,
  I2C_CR1_DNF = 0xFu << 8u,
  I2C_CR1_DNF_POS = 8u,
  I2C_CR1_ERRIE = 1u << 7u,
  I2C_CR1_TCIE = 1u << 6u,
  I2C_CR1_STOPIE = 1u << 5u,
  I2C_CR1_NACKIE = 1u << 4u,
  I2C_CR1_ADDRIE = 1u << 3u,
  I2C_CR1_RXIE = 1u << 2u,
  I2C_CR1_TXIE = 1u << 1u,
  I2C_CR1_PE = 1u << 0u,
};

enum I2C_CR2 : std::uint32_t {
  I2C_CR2_PECBYTE = 1u << 26u,
  I2C_CR2_AUTOEND = 1u << 25u,
  I2C_CR2_RELOAD = 1u << 24u,
  I2C_CR2_NBYTES = 0xFFu << 16u,
  I2C_CR2_NBYTES_POS = 16u,
  I2C_CR2_NACK = 1u << 15u,
  I2C_CR2_STOP = 1u << 14u,
  I2C_CR2_START = 1u << 13u,
  I2C_CR2_HEAD10R = 1u << 12u,
  I2C_CR2_ADD10 = 1u << 11u,
  I2C_CR2_RD_WRN = 1u << 10u,
  I2C_CR2_SADD = 0x1FFu,
  I2C_CR2_SADD_POS = 0
};

enum I2C_OAR1 : std::uint32_t {
  I2C_OAR1_OA1EN = 1u << 15u,
  I2C_OAR1_OA1MODE = 1u << 10u,
  I2C_OAR1_OA1 = 0x1FFu,
  I2C_OAR1_OA1_POS = 0
};

enum I2C_OAR2 : std::uint32_t {
  I2C_OAR2_OA2EN = 1u << 15u,
  I2C_OAR2_OA2MASK = 0b111u << 8u,
  I2C_OAR2_OA2MASK_POS = 8,
  I2C_OAR2_OA2 = 0x7Fu << 1u,
  I2C_OAR2_OA2_POS = 1
};

enum I2C_TIMINGR : std::uint32_t {
  I2C_TIMINGR_PRESC = 0xFu << 28u,
  I2C_TIMINGR_PRESC_POS = 28u,
  I2C_TIMINGR_SCLDEL = 0xFu << 20u,
  I2C_TIMINGR_SCLDEL_POS = 20u,
  I2C_TIMINGR_SDADEL = 0xFu << 16u,
  I2C_TIMINGR_SDADEL_POS = 16u,
  I2C_TIMINGR_SCLH = 0xFFu << 8,
  I2C_TIMINGR_SCLH_POS = 8,
  I2C_TIMINGR_SCLL = 0xFFu,
  I2C_TIMINGR_SCLL_POS = 0
};

enum I2C_TIMEOUTR : std::uint32_t {
  I2C_TIMEOUTR_TEXTEN = 1u << 31u,
  I2C_TIMEOUTR_TIMEOUTB = 0xFFFu << 16u,
  I2C_TIMEOUTR_TIMEOUTB_POS = 16u,
  I2C_TIMEOUTR_TIMEOUTEN = 1u << 15u,
  I2C_TIMEOUTR_TIDLE = 1u << 12u,
  I2C_TIMEOUTR_TIMEOUTA = 0xFFFu,
  I2C_TIMEOUTR_TIMEOUTA_POS = 0u,
};

enum I2C_ISR : std::uint32_t {
  I2C_ISR_ADDCODE = 0b1111111u << 17u,
  I2C_ISR_ADDCODE_POS = 17u,
  I2C_ISR_DIR = 1u << 16u,
  I2C_ISR_BUSY = 1u << 15u,
  I2C_ISR_ALERT = 1u << 13u,
  I2C_ISR_TIMEOUT = 1u << 12u,
  I2C_ISR_PECERR = 1u << 11u,
  I2C_ISR_OVR = 1u << 10u,
  I2C_ISR_ARLO = 1u << 9u,
  I2C_ISR_BERR = 1u << 8u,
  I2C_ISR_TCR = 1u << 7u,
  I2C_ISR_TC = 1u << 6u,
  I2C_ISR_STOPF = 1u << 5u,
  I2C_ISR_NACKF = 1u << 4u,
  I2C_ISR_ADDR = 1u << 3u,
  I2C_ISR_RXNE = 1u << 2u,
  I2C_ISR_TXIS = 1u << 1u,
  I2C_ISR_TXE = 1u << 0u
};

enum I2C_ICR : std::uint32_t {
  I2C_ICR_ALERTCF = 1u << 13u,
  I2C_ICR_TIMOUTCF = 1u << 12u,
  I2C_ICR_PECCF = 1u << 11u,
  I2C_ICR_OVRCF = 1u << 10u,
  I2C_ICR_ARLOCF = 1u << 9u,
  I2C_ICR_BERRCF = 1u << 8u,
  I2C_ICR_STOPCF = 1u << 5u,
  I2C_ICR_NACKCF = 1u << 4u,
  I2C_ICR_ADDRCF = 1u << 3u
};

enum I2C_PECR : std::uint32_t { I2C_PECR_PEC = 0xFFu, I2C_PECR_PEC_POS = 0u };

enum I2C_RXDR : std::uint32_t {
  I2C_RXDR_RXDATA = 0xFFu,
  I2C_RXDR_RXDATA_POS = 0u
};

enum I2C_TXDR : std::uint32_t {
  I2C_TXDR_TXDATA = 0xFFu,
  I2C_TXDR_TXDATA_POS = 0u
};

struct I2c {
  volatile std::uint32_t CR1;
  volatile std::uint32_t CR2;
  volatile std::uint32_t OAR1;
  volatile std::uint32_t OAR2;
  volatile std::uint32_t TIMINGR;
  volatile std::uint32_t TIMEOUTR;
  volatile std::uint32_t ISR;
  volatile std::uint32_t ICR;
  volatile std::uint32_t PECR;
  volatile std::uint32_t RXDR;
  volatile std::uint32_t TXDR;
  static hal::Error to_error(std::uint32_t e) {
    if (e == 0)
      return hal::Error::none;
    if (e & I2C_ISR_BERR)
      return hal::Error::bus_error;
    if (e & I2C_ISR_NACKF)
      return hal::Error::protocol_error;
    return {};
  }
  bool enabled() const { return (CR1 & I2C_CR1_PE) != 0; }
  bool is_busy() const { return hal::mmio::get(ISR, I2C_ISR_BUSY); }
  bool arbitration_lost() const { return hal::mmio::get(ISR, I2C_ISR_ARLO); }
  bool bus_error() const { return hal::mmio::get(ISR, I2C_ISR_BERR); }
  bool transfer_complete_reload() const {
    return hal::mmio::get(ISR, I2C_ISR_TCR);
  }

  void enable() { hal::mmio::set_bits(CR1, I2C_CR1_PE); }
  void disable() { hal::mmio::reset_bits(CR1, I2C_CR1_PE); }
  hal::Error write(uint8_t addr, std::span<const uint8_t> buffer) {
    if (enabled() or is_busy())
      return hal::Error::already_in_use;

    enable();
    if (buffer.size() > 255) {
      CR2 = (uint32_t{addr} << 1u) | I2C_CR2_RELOAD |
            0xFFu << I2C_CR2_NBYTES_POS | I2C_CR2_START;
      while (buffer.size() > 255) {
        auto b = buffer.subspan(0, 255);
        for (const auto &data : b) {
          while (1) {
            auto isr = ISR;
            if (isr & I2C_ISR_TXIS)
              break;
            if (isr & I2C_ISR_NACKF) {
              CR2 |= I2C_CR2_STOP;
              disable();
              return hal::Error::protocol_error;
            }
          }
          TXDR = data;
        }
        if (ISR & I2C_ISR_TC) {
          // ...
        }

        if (ISR & I2C_ISR_TCR) {
          buffer = buffer.subspan(255);
          if (buffer.size() == 0) {
            CR2 = I2C_CR2_STOP;
            disable();
            return hal::Error::none;
          }
          CR2 |= I2C_CR2_RELOAD | 0xFFu << I2C_CR2_NBYTES_POS;
        }
      }
      CR2 = (uint32_t{addr} << 1u) | I2C_CR2_AUTOEND |
            buffer.size() << I2C_CR2_NBYTES_POS | I2C_CR2_START;
      for (const auto &data : buffer) {
        while ((ISR & I2C_ISR_TXIS) == 0) {
          if (auto e = ISR & (I2C_ISR_NACKF | I2C_ISR_BERR); e != 0) {
            CR2 = I2C_CR2_STOP;
            disable();
            return to_error(e);
          }
        }
        TXDR = data;
      }
      disable();
      return hal::Error::none;
    }

    CR2 = (uint32_t{addr} << 1u) | I2C_CR2_AUTOEND |
          buffer.size() << I2C_CR2_NBYTES_POS | I2C_CR2_START;
    for (const auto &data : buffer) {
      while ((ISR & I2C_ISR_TXIS) == 0) {
        if (auto e = ISR & (I2C_ISR_NACKF | I2C_ISR_BERR); e != 0) {
          CR2 = I2C_CR2_STOP;
          disable();
          return to_error(e);
        }
      }
      TXDR = data;
    }
    disable();
    return hal::Error::none;
  }

  hal::Error read(uint8_t addr, std::span<uint8_t> buffer) {
    if (enabled() or is_busy())
      return hal::Error::already_in_use;

    enable();

    if (buffer.size() > 255) {
      CR2 = (uint32_t{addr} << 1u) | I2C_CR2_RELOAD |
            0xFFu << I2C_CR2_NBYTES_POS | I2C_CR2_START;
      while (buffer.size() > 255) {
        auto b = buffer.subspan(0, 255);
        for (auto &data : b) {
          while ((ISR & I2C_ISR_RXNE) == 0) {
          }
          data = static_cast<std::uint8_t>(RXDR);
        }

        if (ISR & I2C_ISR_TCR) {
          buffer = buffer.subspan(255);
          if (buffer.size() == 0) {
            CR2 = I2C_CR2_STOP;
            disable();
            return hal::Error::none;
          }
          CR2 |= I2C_CR2_RELOAD | 0xFFu << I2C_CR2_NBYTES_POS;
        } else {
          CR2 = I2C_CR2_STOP;
          disable();
          return hal::Error::none;
        }
      }
      CR2 = (uint32_t{addr} << 1u) | I2C_CR2_AUTOEND |
            buffer.size() << I2C_CR2_NBYTES_POS | I2C_CR2_START;
      for (auto &data : buffer) {
        while ((ISR & I2C_ISR_RXNE) == 0) {
        }
        data = static_cast<std::uint8_t>(RXDR);
      }
      disable();
      return hal::Error::none;
    }
    CR2 = (uint32_t{addr} << 1u) | I2C_CR2_AUTOEND |
          buffer.size() << I2C_CR2_NBYTES_POS | I2C_CR2_START;
    for (auto &data : buffer) {
      while ((ISR & I2C_ISR_RXNE) == 0) {
      }
      TXDR = data;
    }
    disable();
    return hal::Error::none;
  }
};

} // namespace stm32c031xx

namespace hal::i2c {
ConfigResult<HandleRef> configure(const Config &cfg) noexcept {
  using namespace hal::gpio;
  if (cfg.id != Id::A)
    return ConfigError::invalid_id;
  if (cfg.scl == gpio::Id::invalid)
    return ConfigError::invalid_scl;
  if (cfg.sda == gpio::Id::invalid)
    return ConfigError::invalid_sda;
  if (cfg.scl == cfg.sda)
    return ConfigError::invalid_pin;

  constexpr std::pair<gpio::Id, unsigned> scl_pins[] = {{Port::A | Pin9, 6},
                                                        {Port::B | Pin6, 6},
                                                        {Port::B | Pin8, 6},
                                                        {Port::B | Pin7, 14}};
  constexpr std::pair<gpio::Id, unsigned> sda_pins[] = {{Port::A | Pin10, 6},
                                                        {Port::B | Pin7, 6},
                                                        {Port::B | Pin9, 6},
                                                        {Port::C | Pin14, 14}};
  // TODO: set gpio speed according to i2c speed
  hal::gpio::Config scl_cfg{.pins = cfg.scl,
                            .function = gpio::Function::alternate,
                            .mode = gpio::Mode::open_drain,
                            .speed = gpio::Speed::slow,
                            .pull = gpio::Pull::none,
                            .state = hal::gpio::State::set,
                            .alternate = 0};
  hal::gpio::Config sda_cfg{.pins = cfg.sda,
                            .function = gpio::Function::alternate,
                            .mode = gpio::Mode::open_drain,
                            .speed = gpio::Speed::slow,
                            .pull = gpio::Pull::none,
                            .state = hal::gpio::State::set,
                            .alternate = 0};
  bool found = false;
  for (const auto &[pin, alternate] : scl_pins) {
    if (pin == scl_cfg.pins) {
      scl_cfg.alternate = alternate;
      found = true;
      break;
    }
  }
  if (not found)
    return ConfigError::invalid_scl;

  found = false;
  for (const auto &[pin, alternate] : sda_pins) {
    if (pin == sda_cfg.pins) {
      sda_cfg.alternate = alternate;
      found = true;
      break;
    }
  }
  if (not found)
    return ConfigError::invalid_sda;

  using namespace stm32c031xx;
  // section 25.4.5 of the TRM
  clock_tree.reset(hal::Peripheral::i2c_a);
  clock_tree.enable(hal::Peripheral::i2c_a);
  clock_tree.set_clock(hal::Peripheral::i2c_a, Clock::SYSCLK);
  const auto f = stm32c031xx::clock_tree.sysclk;

  if (f < 4 * cfg.frequency)
    return ConfigError::invalid_baudrate;

  std::uint32_t CR1{};
  std::uint32_t CR2{};
  std::uint32_t OAR1{};
  std::uint32_t OAR2{};
  std::uint32_t TIMINGR{};
  std::uint32_t TIMEOUTR{};

  // section 25.4.9
  // f ~= [(SCLH+1) + (SCLL+1)](PRESC+1)cfg.frequency
  constexpr unsigned max_presc = 0x10u;
  constexpr unsigned max_period = 0x200u;
  constexpr unsigned max_div = max_presc * max_period;

  const unsigned div = f / cfg.frequency;
  if (div > max_div)
    return ConfigError::invalid_baudrate;

  auto presc = div / max_period;
  if (presc > max_presc)
    return ConfigError::invalid_baudrate;

  if (presc == 0)
    presc = 1;

  const auto period = div / presc;
  // TODO: SCLDEL and SDADEL
  const auto sda_del = 0;
  const auto scl_del = 10;
  CR1 = I2C_CR1_ERRIE | I2C_CR1_TCIE | I2C_CR1_STOPIE | I2C_CR1_RXIE |
        I2C_CR1_TXIE;
  CR2 = I2C_CR2_AUTOEND;

  --presc;
  const auto scll = period / 2 - 1;
  auto sclh = period / 2;
  if (period % 2 == 0) {
    --sclh;
  }
  TIMINGR = (presc << I2C_TIMINGR_PRESC_POS) |
            (scl_del << I2C_TIMINGR_SCLDEL_POS) |
            (sda_del << I2C_TIMINGR_SDADEL_POS) |
            (sclh << I2C_TIMINGR_SCLH_POS) | (scll << I2C_TIMINGR_SCLL_POS);

  auto pin_res = gpio::configure(scl_cfg);
  if (not pin_res)
    return pin_res.error;
  pin_res.peripheral.set(gpio::State::set);
  pin_res = gpio::configure(sda_cfg);
  if (not pin_res)
    return pin_res.error;
  pin_res.peripheral.set(gpio::State::set);

  I2C1->TIMINGR = 0x10805D88u;
  I2C1->enable();
  I2C1->CR1 = CR1;
  I2C1->CR2 = CR2;
  I2C1->OAR1 = OAR1;
  I2C1->OAR2 = OAR2;
  // I2C1->TIMINGR = TIMINGR;
  I2C1->TIMEOUTR = TIMEOUTR;
  return HandleRef(*I2C1);
}
} // namespace hal::i2c

extern "C" void I2C1_IRQHandler(void) {}
