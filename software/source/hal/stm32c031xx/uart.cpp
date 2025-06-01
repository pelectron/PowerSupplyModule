#include "hal/uart.hpp"
#include "clocks.hpp"
#include "hal/enums.hpp"
#include "hal/gpio.hpp"
#include <cstdint>

#define USART1 reinterpret_cast<stm32c031xx::Usart *>(0x40013800u)
#define USART2 reinterpret_cast<stm32c031xx::Usart *>(0x40004400u)
#define USART3 reinterpret_cast<stm32c031xx::Usart *>(0x40004800u)
#define USART4 reinterpret_cast<stm32c031xx::Usart *>(0x40004C00u)

namespace stm32c031xx {

enum : std::uint32_t {
  BASE = 0,
  USART_CR1 = BASE + 0x00,
  USART_CR2 = BASE + 0x04,
  USART_CR3 = BASE + 0x08,
  USART_BRR = BASE + 0x0C,
  USART_GTPR = BASE + 0x10,
  USART_RTOR = BASE + 0x14,
  USART_RQR = BASE + 0x18,
  UASRT_ISR = BASE + 0x1C,
  USART_ICR = BASE + 0x20,
  USART_RDR = BASE + 0x24,
  USART_TDR = BASE + 0x28,
  USART_PRESC = BASE + 0x2C,
};

enum USART_CR1 : std::uint32_t {
  USART_CR1_RXFFIE = 1u << 31u,
  USART_CR1_TXFEIE = 1 << 30,
  USART_CR1_FIFOEN = 1 << 29,
  USART_CR1_M1 = 1 << 28,
  USART_CR1_EOBIE = 1 << 27,
  USART_CR1_RTOIE = 1 << 26,
  USART_CR1_DEAT = 0x1F << 21,
  USART_CR1_DEAT_POS = 21,
  USART_CR1_DEDT = 0x1F << 16,
  USART_CR1_DEDT_POS = 16,
  USART_CR1_OVER8 = 1 << 15,
  USART_CR1_CMIE = 1 << 14,
  USART_CR1_MME = 1 << 13,
  USART_CR1_M0 = 1 << 12,
  USART_CR1_WAKE = 1 << 11,
  USART_CR1_PCE = 1 << 10,
  USART_CR1_PS = 1 << 9,
  USART_CR1_PEIE = 1 << 8,
  USART_CR1_TXFNFIE = 1 << 7,
  USART_CR1_TCIE = 1 << 6,
  USART_CR1_RXFNEIE = 1 << 5,
  USART_CR1_IDLEIE = 1 << 4,
  USART_CR1_TE = 1 << 3,
  USART_CR1_RE = 1 << 2,
  USART_CR1_UESM = 1 << 1,
  USART_CR1_UE = 1 << 0
};

enum USART_CR2 : std::uint32_t {
  USART_CR2_ADD = 0xFFu << 24,
  USART_CR2_ADD_POS = 24,
  USART_CR2_RTOEN = 1 << 23,
  USART_CR2_ABRMOD = 0b11 << 21,
  USART_CR2_ABRMOD_POS = 21,
  USART_CR2_ABREN = 1 << 20,
  USART_CR2_MSBFIRST = 1 << 19,
  USART_CR2_DATAINV = 1 << 18,
  USART_CR2_TXINV = 1 << 17,
  USART_CR2_RXINV = 1 << 16,
  USART_CR2_SWAP = 1 << 15,
  USART_CR2_LINEN = 1 << 14,
  USART_CR2_STOP = 0b11 << 12,
  USART_CR2_STOP_POS = 12,
  USART_CR2_CLKEN = 1 << 11,
  USART_CR2_CPOL = 1 << 10,
  USART_CR2_SPHA = 1 << 9,
  USART_CR2_LBCL = 1 << 8,
  USART_CR2_LBDIE = 1 << 6,
  USART_CR2_LBDL = 1 << 5,
  USART_CR2_ADDM7 = 1 << 4,
  USART_CR2_DIS_NSS = 1 << 3,
  USART_CR2_SLVEN = 1 << 0
};

enum USART_CR3 : std::uint32_t {
  USART_CR3_TXFTCFG = 0b111u << 29,
  USART_CR3_TXFTCFG_POS = 29,
  USART_CR3_RXFTIE = 1 << 28,
  USART_CR3_RXFTCFG = 0b111u << 25,
  USART_CR3_RXFTCFG_POS = 25,
  USART_CR3_TCBGTIE = 1 << 24,
  USART_CR3_TXFTIE = 1 << 23,
  USART_CR3_WUFIE = 1 << 22,
  USART_CR3_WUS = 0b11 << 20,
  USART_CR3_WUS_POS = 20,
  USART_CR3_SCARCNT = 0b111 << 17,
  USART_CR3_SCARCNT_POS = 17,
  USART_CR3_DEP = 1 << 15,
  USART_CR3_DEM = 1 << 14,
  USART_CR3_DDRE = 1 << 13,
  USART_CR3_OVRDIS = 1 << 12,
  USART_CR3_ONEBIT = 1 << 11,
  USART_CR3_CTSIE = 1 << 10,
  USART_CR3_CTSE = 1 << 9,
  USART_CR3_RTSE = 1 << 8,
  USART_CR3_DMAT = 1 << 7,
  USART_CR3_DMAR = 1 << 6,
  USART_CR3_SCEN = 1 << 5,
  USART_CR3_NACK = 1 << 4,
  USART_CR3_HDSEL = 1 << 3,
  USART_CR3_IRLP = 1 << 2,
  USART_CR3_IREN = 1 << 1,
  USART_CR3_EIE = 1 << 0
};

enum USART_BRR : std::uint32_t { USART_BRR_BRR = 0xFF, USART_BRR_BRR_POS = 0 };

enum USART_GTPR : std::uint32_t {
  USART_GTPR_GT = 0xFF << 8,
  USART_GTPR_GT_POS = 8,
  USART_GTPR_PSC = 0xFF,
  USART_GTPR_PSC_POS = 0
};

enum USART_RTOR : std::uint32_t {
  USART_RTOR_BLEN = 0xFFu << 24,
  USART_RTOR_BLEN_POS = 24,
  USART_RTOR_RTO = 0xFFFFFF,
  USART_RTOR_RTO_POS = 0,
};

enum USART_RQR : std::uint32_t {
  USART_RQR_TXFRQ = 1 << 4,
  USART_RQR_RXFRQ = 1 << 3,
  USART_RQR_MMRQ = 1 << 2,
  USART_RQR_SBKRQ = 1 << 1,
  USART_RQR_ABRQ = 1 << 0
};

enum USART_ISR : std::uint32_t {
  USART_ISR_TXFT = 1 << 27,
  USART_ISR_RXFT = 1 << 26,
  USART_ISR_TCBGT = 1 << 25,
  USART_ISR_RXFF = 1 << 24,
  USART_ISR_TXFE = 1 << 23,
  USART_ISR_REACK = 1 << 22,
  USART_ISR_TEACK = 1 << 21,
  USART_ISR_WUF = 1 << 20,
  USART_ISR_RWU = 1 << 19,
  USART_ISR_SBKF = 1 << 18,
  USART_ISR_CMF = 1 << 17,
  USART_ISR_BUSY = 1 << 16,
  USART_ISR_ABRF = 1 << 15,
  USART_ISR_ABRE = 1 << 14,
  USART_ISR_UDR = 1 << 13,
  USART_ISR_EOBF = 1 << 12,
  USART_ISR_RTOF = 1 << 11,
  USART_ISR_CTS = 1 << 10,
  USART_ISR_CTSIF = 1 << 9,
  USART_ISR_LBDF = 1 << 8,
  USART_ISR_TXFNF = 1 << 7,
  USART_ISR_TC = 1 << 6,
  USART_ISR_RXFNE = 1 << 5,
  USART_ISR_IDLE = 1 << 4,
  USART_ISR_ORE = 1 << 3,
  USART_ISR_NE = 1 << 2,
  USART_ISR_FE = 1 << 1,
  USART_ISR_PE = 1 << 0
};

enum USART_ICR : std::uint32_t {
  USART_ICR_WUCF = 1 << 20,
  USART_ICR_CMCF = 1 << 17,
  USART_ICR_UDRCF = 1 << 13,
  USART_ICR_EOBCF = 1 << 12,
  USART_ICR_RTOCF = 1 << 11,
  USART_ICR_CTSCF = 1 << 10,
  USART_ICR_LBDCF = 1 << 8,
  USART_ICR_TCBGTCF = 1 << 7,
  USART_ICR_TCCF = 1 << 6,
  USART_ICR_TXFECF = 1 << 5,
  USART_ICR_IDLECF = 1 << 4,
  USART_ICR_ORECF = 1 << 3,
  USART_ICR_NECF = 1 << 2,
  USART_ICR_FECF = 1 << 1,
  USART_ICR_PECF = 1 << 0
};

enum USART_RDR : std::uint32_t {
  USART_RDR_RDR = 0x1FF,
};

enum USART_TDR : std::uint32_t {
  USART_TDR_TDR = 0x1FF,
};

enum USART_PRESC : std::uint32_t {
  USART_PRESC_PRESCALER = 0xF,
};

struct Usart {
  volatile std::uint32_t CR1;
  volatile std::uint32_t CR2;
  volatile std::uint32_t CR3;
  volatile std::uint32_t BRR;
  volatile std::uint32_t GPTR;
  volatile std::uint32_t RTOR;
  volatile std::uint32_t RQR;
  volatile std::uint32_t ISR;
  volatile std::uint32_t ICR;
  volatile std::uint32_t RDR;
  volatile std::uint32_t TDR;
  volatile std::uint32_t PRESC;

  void enable() { CR1 |= USART_CR1_UE; }

  void disable() { CR1 &= ~USART_CR1_UE; }

  void tx_enable() { CR1 |= USART_CR1_TE; }
  bool tx_enabled() { return (CR1 & USART_CR1_TE) != 0; }
  void tx_disable() { CR1 &= ~USART_CR1_TE; }

  void rx_enable() { CR1 |= USART_CR1_RE; }
  bool rx_enabled() { return (CR1 & USART_CR1_RE) != 0; }
  void rx_disable() { CR1 &= ~USART_CR1_RE; }

  bool is_busy() { return (ISR & USART_ISR_BUSY) != 0; }

  static hal::Error to_error(std::uint32_t errs) {
    if (errs == 0)
      return hal::Error::none;
    if (errs & USART_ISR_ORE)
      return hal::Error::buffer_overflow;
    if (errs & USART_ISR_NE)
      return hal::Error::noisy;
    if (errs & USART_ISR_FE)
      return hal::Error::frame_error;
    if (errs & USART_ISR_PE)
      return hal::Error::parity_error;
    return hal::Error::unknown;
  }

  hal::Error write(std::span<const std::uint8_t> buf) {
    for (std::size_t i = 0; i < buf.size(); ++i) {
      while ((ISR & USART_ISR_TXFNF) == 0) {
        // wait while tx fifo is full
      }
      TDR = static_cast<std::uint16_t>(buf[i]) & 0x1FFu;
    }
    while ((ISR & USART_ISR_TC) == 0) {
      // wait for transmission complete
    }
    return hal::Error::none;
  }

  void clear_errors() {
    ICR |= USART_ICR_ORECF | USART_ICR_NECF | USART_ICR_FECF | USART_ICR_PECF;
  }

  hal::Error read(std::span<std::uint8_t> buf) {
    constexpr auto errors =
        USART_ISR_ORE | USART_ISR_NE | USART_ISR_FE | USART_ISR_PE;
    for (std::size_t i = 0; i < buf.size(); ++i) {
      while ((ISR & USART_ISR_RXFNE) == 0) {
        // wait while rx fifo is empty
      }
      if (auto errs = ISR & errors; errs != 0) {
        rx_disable();
        clear_errors();
        return to_error(errs);
      }
      buf[i] = RDR;
    }
    if (auto errs = ISR & errors; errs != 0) {
      rx_disable();
      clear_errors();
      return to_error(errs);
    }
    return hal::Error::none;
  }

  hal::Error transceive(std::span<const std::uint8_t> write_buf,
                        std::span<std::uint8_t> read_buf) {
    return hal::Error::not_implemented;
  }
};

} // namespace stm32c031xx

hal::ConfigResult<hal::uart::Device>
hal::uart::configure(const hal::uart::Config &cfg) noexcept {
  using namespace stm32c031xx;
  using namespace hal::uart;
  std::uint32_t CR1{};
  std::uint32_t CR2{};
  std::uint32_t CR3{};
  std::uint32_t BRR{};
  std::uint32_t RTOR{};
  std::uint32_t PRESC{};

  // TODO: usart 2, 3 and 4
  Usart *usart = nullptr;
  hal::Peripheral p;
  switch (cfg.id) {
  case Id::A:
    usart = USART1;
    p = hal::Peripheral::uart_a;
    break;
  default:
    return ConfigError::invalid_id;
  }

  // check gpio
  constexpr std::pair<gpio::Id, unsigned> usart_1_rx[]{
      {gpio::Port::A | gpio::Pin1, 4},
      {gpio::Port::A | gpio::Pin10, 1},
      {gpio::Port::A | gpio::Pin8, 14},
      {gpio::Port::B | gpio::Pin2, 0},
      {gpio::Port::B | gpio::Pin7, 0}};
  constexpr std::pair<gpio::Id, unsigned> usart_1_tx[]{
      {gpio::Port::A | gpio::Pin0, 4},
      {gpio::Port::A | gpio::Pin9, 1},
      {gpio::Port::B | gpio::Pin6, 0},
      {gpio::Port::C | gpio::Pin14, 0}};

  gpio::Config rx_cfg{cfg.rx,
                      gpio::Function::alternate,
                      gpio::Mode::push_pull,
                      gpio::Speed::slow,
                      gpio::Pull::none,
                      gpio::State::x,
                      0};
  gpio::Config tx_cfg{cfg.tx,
                      gpio::Function::alternate,
                      gpio::Mode::push_pull,
                      gpio::Speed::slow,
                      gpio::Pull::none,
                      gpio::State::set,
                      0};
  bool found = false;
  for (const auto [id, alternate] : usart_1_rx) {
    if (id == cfg.rx) {
      found = true;
      rx_cfg.alternate = alternate;
      break;
    }
  }
  if (not found)
    return ConfigError::invalid_rx;

  found = false;
  for (const auto [id, alternate] : usart_1_tx) {
    if (id == cfg.tx) {
      found = true;
      tx_cfg.alternate = alternate;
      break;
    }
  }
  if (not found)
    return ConfigError::invalid_tx;

  CR1 = USART_CR1_TE | USART_CR1_RE;

  switch (cfg.bits) {
  case Bits::seven:
    CR1 |= USART_CR1_M1;
    break;
  case Bits::eight:
    break;
  case Bits::nine:
    CR1 |= USART_CR1_M0;
    break;
  default:
    return ConfigError::invalid_data_size;
  }

  switch (cfg.parity) {
  case Parity::none:
    break;
  case Parity::even:
    CR1 |= USART_CR1_PCE;
    break;
  case Parity::odd:
    CR1 |= USART_CR1_PCE | USART_CR1_PS;
    break;
  default:
    return ConfigError::invalid_parity;
  }

  switch (cfg.stop_bits) {
  case StopBits::one:
    break;
  case StopBits::one_and_a_half:
    CR2 |= 0b11u << USART_CR2_STOP_POS;
    break;
  case StopBits::two:
    CR2 |= 0b10u << USART_CR2_STOP_POS;
    break;
  default:
    return ConfigError::invalid_stop_bits;
  }

  if (cfg.has_feature(Feature::msb_first))
    CR2 |= USART_CR2_MSBFIRST;

  if (cfg.has_feature(Feature::auto_baudrate))
    CR2 |= USART_CR2_ABREN;

  if (cfg.has_feature(Feature::data_inversion))
    CR2 |= USART_CR2_DATAINV;

  if (cfg.has_feature(Feature::tx_inversion))
    CR2 |= USART_CR2_TXINV;

  if (cfg.has_feature(Feature::rx_inversion))
    CR2 |= USART_CR2_RXINV;

  if (cfg.has_feature(Feature::tx_rx_swap))
    CR2 |= USART_CR2_SWAP;

  if (cfg.has_feature(Feature::wakeup))
    CR1 |= USART_CR1_UESM;

  if (cfg.has_feature(Feature::receiver_timeout)) {
    CR2 |= USART_CR2_RTOEN;
    if (cfg.receiver_timeout < ((1u << 23) - 1u)) {
      RTOR = cfg.receiver_timeout;
    } else {
      return ConfigError::invalid_timeout;
    }
  }

  // DMA Disable on reception error, Error interrupt enable, Wake-up from
  // low-power mode interrupt enable
  // CR3 |= USART_CR3_DDRE | USART_CR3_EIE | USART_CR3_WUFIE;

  // configure baud rate
  // USART_DIV*PRESC*baud= usart_clk
  // When oversampling by 16, the baud rate ranges from usart_ker_ck_pres/65535
  // and usart_ker_ck_pres/16. usart_ker_ck_pres = usart_clk/PRESC
  constexpr unsigned div_min = 16u;
  constexpr unsigned div_max = (1u << 16u) - 1u;
  const auto usart_clk =
      cfg.id == uart::Id::A ? clock_tree().usart1 : clock_tree().pclk;

  if (usart_clk == 0)
    return ConfigError::invalid_clock_frequency;

  if (cfg.baudrate * div_min > usart_clk) // baudrate too high
    return ConfigError::invalid_baudrate;

  constexpr unsigned prescs[]{1, 2, 4, 6, 8, 10, 12, 16, 32, 64, 128, 256};
  constexpr unsigned num_prescs = 12;
  std::size_t p_idx = 0;
  for (const auto &presc : prescs) {
    if (cfg.baudrate * div_max * presc > usart_clk) {
      break;
    }
    ++p_idx;
  }

  if (p_idx == num_prescs) // baudrate too low
    return ConfigError::invalid_baudrate;

  BRR = usart_clk / (cfg.baudrate * prescs[p_idx]);
  if (BRR < 16)
    return ConfigError::invalid_baudrate;

  PRESC = p_idx;

  // enable clock
  hal::mmio::set_bits(RCC->APBENR2, RCC_APBENR2_USART1EN);
  auto tmp = hal::mmio::get(RCC->APBENR2, RCC_APBENR2_USART1EN);
  (void)tmp;
  // apply the configuration
  // configure rx
  auto pin_res = gpio::configure(rx_cfg);
  if (not pin_res)
    return pin_res.error;

  // configure tx
  pin_res = gpio::configure(tx_cfg);
  if (not pin_res)
    return pin_res.error;
  usart->CR1 &= ~USART_CR1_UE;
  usart->CR1 = CR1;
  usart->CR2 = CR2;
  usart->CR3 = CR3;
  usart->BRR = BRR;
  usart->RTOR = RTOR;
  usart->PRESC = PRESC;
  usart->CR1 |= USART_CR1_UE;
  while (hal::mmio::get(usart->ISR, USART_ISR_TEACK | USART_ISR_REACK) == 0) {
  }

  return Device(*usart, 0);
}

extern "C" void USART1_IRQHandler(void) {}

extern "C" void USART2_IRQHandler(void) {}
