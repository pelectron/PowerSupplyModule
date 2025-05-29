#include "hal/spi.hpp"
#include "clocks.hpp"
#include "hal/config.hpp"
#include "hal/enums.hpp"
#include "hal/gpio.hpp"
#include "hal/mmio.hpp"
#include "units.hpp"
#include <cstdint>

#define SPI1 reinterpret_cast<Spi *>(SPI1_ADDR)

using namespace hal;
using namespace hal::spi;

enum : std::uint32_t { SPI1_ADDR = 0x40012C00UL };

enum CR1 : std::uint32_t {
  BIDIMODE = 1 << 15,
  BIDIOE = 1 << 14,
  CRCEN = 1 << 13,
  CRCNEXT = 1 << 12,
  CRCL = 1 << 11,
  RXONLY = 1 << 10,
  SSM = 1 << 9,
  SSI = 1 << 8,
  LSBFIRST = 1 << 7,
  SPE = 1 << 6,
  BR = 0b111 << 3,
  BR_POS = 3,
  MSTR = 1 << 2,
  CPOL = 1 << 1,
  CPHA = 1 << 0
};
enum CR2 : std::uint32_t {
  LDMA_TX = 1 << 14,
  LDMA_RX = 1 << 13,
  FRXTH = 1 << 12,
  DS = 0xF << 8,
  DS_POS = 8,
  TXEIE = 1 << 7,
  RXNEIE = 1 << 6,
  ERRIE = 1 << 5,
  FRF = 1 << 4,
  NSSP = 1 << 3,
  SSOE = 1 << 2,
  TXDMAEN = 1 << 1,
  RXDMAEN = 1 << 0
};
enum SR : std::uint32_t {
  FTLVL = 0b11 << 11,
  FTLVL_POS = 11,
  FRLVL = 0b11 << 9,
  FRLVL_POS = 9,
  FRE = 1 << 8,
  BSY = 1 << 7,
  OVR = 1 << 6,
  MODF = 1 << 5,
  CRCERR = 1 << 4,
  UDR = 1 << 3,
  CHSIDE = 1 << 2,
  TXE = 1 << 1,
  RXNE = 1 << 0,
};
enum : std::uint32_t { DR_MASK = 0xFF };
enum IRQs : std::uint32_t {
  IRQ_TX_EMPTY = TXEIE,
  IRQ_RX_NOT_EMPTY = RXNEIE,
  IRQ_ERROR = ERRIE
};

enum FifoLevel : std::uint32_t {
  fifo_empty = 0,
  fifo_one_fourth = 1,
  fifo_one_half = 2,
  fifo_full = 3
};

constexpr IRQs operator|(IRQs a, IRQs b) {
  return static_cast<IRQs>(static_cast<unsigned>(a) | static_cast<unsigned>(b));
}

struct Spi {
  volatile uint32_t CR1; /*!< SPI Control register 1 (not used in I2S mode),
                            Address offset: 0x00 */
  volatile uint32_t CR2; /*!< SPI Control register 2,    Address offset: 0x04 */
  volatile uint32_t SR;  /*!< SPI Status register,     Address offset: 0x08 */
  volatile uint32_t DR;  /*!< SPI data register,     Address offset: 0x0C */
  volatile uint32_t CRCPR;   /*!< SPI CRC polynomial register (not used in I2S
                                mode),  Address offset: 0x10 */
  volatile uint32_t RXCRCR;  /*!< SPI Rx CRC register (not used in I2S mode),
                                Address offset: 0x14 */
  volatile uint32_t TXCRCR;  /*!< SPI Tx CRC register (not used in I2S mode),
                                Address offset: 0x18 */
  volatile uint32_t I2SCFGR; /*!< SPI_I2S configuration register, Address
                                offset: 0x1C */
  volatile uint32_t I2SPR; /*!< SPI_I2S prescaler register, Address offset: 0x20
                            */
  void enable() { mmio::set_bits(CR1, SPE); }

  void disable() {

    if (mmio::get(CR1, RXONLY)) {
      // 1. Interrupt the receive flow by disabling SPI (SPE=0) in the specific
      // time window while the last data frame is ongoing.
      mmio::reset_bits(CR1, SPE);
      // 2. Wait until BSY=0 (the last data frame is processed).
      while (mmio::get(CR1, BSY) != 0) {
      }
      // 3. Read data until FRLVL[1:0] = 00 (read all the received data).
      while (auto frlvl = mmio::get(CR1, FRLVL)) {
        if (frlvl == 1 and mmio::get(CR2, DS, DS_POS) <= 8) {
          // TODO: If packing mode is used and an odd number of data frames with
          // a format less than or equal to 8 bits (fitting into one byte) has
          // to be received, FRXTH must be set when FRLVL[1:0] = 01, in order to
          // generate the RXNE event to read the last odd data frame and to keep
          // good FIFO pointer alignment.
        }
        mmio::get(DR, DR_MASK);
      }

    } else {
      // 1. Wait until FTLVL[1:0] = 00 (no more data to transmit).
      while (mmio::get(CR1, FTLVL) != 0) {
      }
      // 2. Wait until BSY=0 (the last data frame is processed).
      while (mmio::get(CR1, BSY) != 0) {
      }
      // 3. Disable the SPI (SPE=0).
      mmio::reset_bits(CR1, SPE);
      // 4. Read data until FRLVL[1:0] = 00 (read all the received data)
      while (auto frlvl = mmio::get(CR1, FRLVL)) {
        if (frlvl == 1 and mmio::get(CR2, DS, DS_POS) <= 8) {
          // TODO: If packing mode is used and an odd number of data frames with
          // a format less than or equal to 8 bits (fitting into one byte) has
          // to be received, FRXTH must be set when FRLVL[1:0] = 01, in order to
          // generate the RXNE event to read the last odd data frame and to keep
          // good FIFO pointer alignment.
        }
        mmio::get(DR, DR_MASK);
      }
    }
  }

  bool is_enabled() { return mmio::get(CR1, SPE) != 0; }

  void enable_irqs(IRQs irqs) { mmio::set_bits(CR2, irqs); }

  void disable_irqs(IRQs irqs) { mmio::reset_bits(CR2, irqs); }
  bool rx_fifo_not_empty() { return hal::mmio::get(SR, RXNE) != 0; }
  bool tx_fifo_empty() { return hal::mmio::get(SR, TXE) != 0; }
  std::uint32_t errors() {
    return hal::mmio::get(SR, OVR | MODF | CRCERR | FRE);
  }

  FifoLevel rx_fifo_level() {
    return static_cast<FifoLevel>(mmio::get(SR, FRLVL, FRLVL_POS));
  }

  FifoLevel tx_fifo_level() {
    return static_cast<FifoLevel>(mmio::get(SR, FTLVL, FTLVL_POS));
  }

  bool is_busy() {
    return hal::mmio::get(SR, RXNE | TXE | BSY | FRLVL | FTLVL) != 0;
  }

  void start(Operation *op) {
    using enum Operation::Type;
    switch (op->type) {
    case Read:
      break;
    case Write:
      enable_irqs(IRQ_ERROR | IRQ_TX_EMPTY);
      enable();
      op->write_idx = 1;
      this->putc(op->write_buf[0]);
      break;
    case ReadWrite:
      enable_irqs(IRQ_ERROR | IRQ_TX_EMPTY | IRQ_RX_NOT_EMPTY);
      enable();
      op->write_idx = 1;
      this->putc(op->write_buf[0]);
      break;
    default:
      return;
    }
  }

  void putc(uint8_t c) { *reinterpret_cast<volatile uint8_t *>(&DR) = c; }
  void putc(uint16_t c) { *reinterpret_cast<volatile uint16_t *>(&DR) = c; }

  template <typename T> T getc() {
    return *reinterpret_cast<volatile T *>(&DR);
  }

  constexpr static hal::Error to_error(std::uint32_t errs) {
    if (errs & OVR)
      return hal::Error::buffer_overflow;
    if (errs & MODF)
      return hal::Error::bus_error;
    if (errs & CRCERR)
      return hal::Error::crc_error;
    if (errs & FRE)
      return hal::Error::frame_error;
    return hal::Error::none;
  }

  hal::Error write(std::span<const std::uint8_t> buffer) {
    if (buffer.size() == 0) {
      return hal::Error::invalid_param;
    }

    if (is_enabled() or is_busy()) {
      return hal::Error::already_in_use;
    }

    enable();

    for (const auto &byte : buffer) {
      reinterpret_cast<volatile uint8_t &>(DR) = byte;
      while (tx_fifo_level() == FifoLevel::fifo_full) {
        auto errs = errors();
        if (errs != 0) {
          disable();
          return to_error(errs);
        }
      }
    }

    while (not tx_fifo_empty()) {
      auto errs = errors();
      if (errs != 0)
        return to_error(errs);
    }
    disable();
    return to_error(errors());
  }

  hal::Error read(std::span<std::uint8_t> buffer) {
    if (buffer.size() == 0) {
      return hal::Error::invalid_param;
    }

    if (is_enabled() or is_busy()) {
      return hal::Error::already_in_use;
    }

    mmio::set_bits(CR1, CR1::RXONLY);
    enable();
    for (std::size_t i = 0; i < buffer.size(); ++i) {
      // wait for data to be ready
      while (not rx_fifo_not_empty()) {
      }
      // read data
      buffer[i] = *reinterpret_cast<volatile std::uint8_t *>(&DR);
      auto errs = errors();
      if (errs != 0)
        return to_error(errs);
    }
    disable();
    mmio::reset_bits(CR1, CR1::RXONLY);
    return to_error(errors());
  }

  hal::Error transceive(std::span<const std::uint8_t> write_buf,
                        std::span<std::uint8_t> read_buf) {
    if (write_buf.size() != read_buf.size() or write_buf.size() == 0) {
      return hal::Error::invalid_param;
    }

    if (is_enabled() or is_busy()) {
      return hal::Error::already_in_use;
    }

    enable();
    for (std::size_t w_idx = 0, r_idx = 0;
         w_idx < write_buf.size() or r_idx < read_buf.size();) {
      // put as much data in tx fifo as possible
      while (tx_fifo_level() != fifo_full and w_idx < write_buf.size()) {
        reinterpret_cast<volatile uint8_t &>(DR) = write_buf[w_idx++];
        if (auto errs = errors(); errs != 0) {
          disable();
          return to_error(errs);
        }
      }

      // wait for data to be received
      while (not rx_fifo_not_empty()) {
        if (auto errs = errors(); errs != 0) {
          disable();
          return to_error(errs);
        }
      }
      // read as much data as possible
      while (rx_fifo_not_empty() and r_idx < read_buf.size()) {
        read_buf[r_idx++] = reinterpret_cast<volatile uint8_t &>(DR);
        if (auto errs = errors(); errs != 0) {
          disable();
          return to_error(errs);
        }
      }
    }

    disable();
    return to_error(errors());
  }
};

ConfigResult<Device> hal::spi::configure(const Config &cfg) noexcept {
  // first, validate the config

  // check the id
  if (cfg.id != Id::A)
    return ConfigError::invalid_id;

  // check the gpios
  using namespace gpio;
  // map of gpio ids to alternate function number
  constexpr std::pair<gpio::Id, uint8_t> sclk_pins[] = {{Port::A | Pin1, 0},
                                                        {Port::A | Pin5, 0},
                                                        {Port::B | Pin3, 0},
                                                        {Port::B | Pin6, 10}};

  constexpr std::pair<gpio::Id, uint8_t> mosi_pins[] = {{Port::A | Pin2, 0},
                                                        {Port::A | Pin7, 0},
                                                        {Port::A | Pin12, 0},
                                                        {Port::B | Pin5, 0},
                                                        {Port::B | Pin6, 8}};

  constexpr std::pair<gpio::Id, uint8_t> miso_pins[] = {{Port::A | Pin6, 0},
                                                        {Port::A | Pin11, 0},
                                                        {Port::B | Pin4, 0},
                                                        {Port::B | Pin6, 9}};

  constexpr std::pair<gpio::Id, uint8_t> nss_pins[] = {{Port::A | Pin4, 0},
                                                       {Port::A | Pin8, 8},
                                                       {Port::A | Pin14, 8},
                                                       {Port::A | Pin15, 0},
                                                       {Port::B | Pin0, 0}};

  // SCLK
  gpio::Config sclk_cfg{cfg.sclk,
                        gpio::Function::alternate,
                        gpio::Mode::push_pull,
                        gpio::Speed::fast,
                        gpio::Pull::none,
                        cfg.polarity == Polarity::low ? gpio::State::reset
                                                      : gpio::State::set,
                        0};
  bool found = false;
  for (const auto &pair : sclk_pins)
    if (pair.first == cfg.sclk) {
      found = true;
      sclk_cfg.alternate = pair.second;
      break;
    }

  if (not found)
    return ConfigError::invalid_sclk;

  // MOSI
  gpio::Config mosi_cfg{cfg.mosi,
                        gpio::Function::alternate,
                        gpio::Mode::push_pull,
                        gpio::Speed::fast,
                        gpio::Pull::none,
                        gpio::State::reset,
                        0};
  found = false;
  for (const auto &pair : mosi_pins)
    if (pair.first == cfg.mosi) {
      found = true;
      mosi_cfg.alternate = pair.second;
      break;
    }

  if (not found)
    return ConfigError::invalid_mosi;

  // MISO
  gpio::Config miso_cfg{cfg.miso,
                        gpio::Function::alternate,
                        gpio::Mode::none,
                        gpio::Speed::slow,
                        gpio::Pull::none,
                        gpio::State::reset,
                        0};
  found = false;
  for (const auto &pair : miso_pins)
    if (pair.first == cfg.miso) {
      found = true;
      miso_cfg.alternate = pair.second;
      break;
    }

  if (not found)
    return ConfigError::invalid_miso;

  // CS/NSS
  gpio::Config cs_cfg{cfg.cs,
                      cfg.use_hw_cs ? gpio::Function::alternate
                                    : gpio::Function::output,
                      gpio::Mode::push_pull,
                      gpio::Speed::slow,
                      gpio::Pull::none,
                      gpio::State::set,
                      0};
  if (cfg.use_hw_cs) {
    for (const auto &pair : nss_pins)
      if (pair.first == cfg.cs) {
        found = true;
        cs_cfg.alternate = pair.second;
        break;
      }

    if (not found)
      return ConfigError::invalid_cs;
  }

  // Actual configuration can start now

  // 1. configure gpio
  // TODO: deinit gpio if configuration of one fails
  auto res = gpio::configure(sclk_cfg);
  if (res.error != ConfigError::success)
    return res.error;

  res = gpio::configure(mosi_cfg);
  if (res.error != ConfigError::success)
    return res.error;

  res = gpio::configure(miso_cfg);
  if (res.error != ConfigError::success)
    return res.error;

  res = gpio::configure(cs_cfg);
  if (res.error != ConfigError::success)
    return res.error;
  hal::gpio::Pin cs = res.peripheral;

  std::uint32_t cr1 = 0;
  std::uint32_t cr2 = 0;
  std::uint32_t crcpr = 0;

  // 2. configure CR1
  // a) configure baudrate divisor
  const auto clk_rate = stm32c031xx::clock_tree.pclk;
  if (cfg.baudrate > clk_rate / 2u)
    return ConfigError::invalid_baudrate;

  auto div = 2u;
  while (clk_rate / div > cfg.baudrate and div < 256u) {
    div *= 2u;
  }
  if (clk_rate / div > cfg.baudrate)
    return ConfigError::invalid_baudrate;

  cr1 = div << BR_POS;

  // b) Configure the CPOL and CPHA
  cr1 |= cfg.polarity == Polarity::high ? CPOL : 0u;
  cr1 |= cfg.phase == Phase::high ? CPHA : 0u;

  // c) Select simplex or half-duplex mode by configuring RXONLY or BIDIMODE and
  // BIDIOE (RXONLY and BIDIMODE cannot be set at the same time)
  if (cfg.three_wire)
    cr1 |= BIDIMODE;

  // d) Configure the LSBFIRST bit to define the frame format
  if (cfg.format == Format::lsb_first)
    cr1 |= LSBFIRST;

  // e) Configure the CRCL and CRCEN bits
  switch (cfg.crc) {
  case Crc::eight_bit:
    cr1 |= CRCEN;
    break;
  case Crc::sixteen_bit:
    cr1 |= CRCEN | CRCL;
    break;
  case Crc::none:
    break;
  default:
    return ConfigError::invalid_crc;
  }

  // f) Configure SSM and SSI.
  if (cfg.use_hw_cs) {

  } else {
    // NSS pin is not used on master side in this
    // configuration. It has to be managed internally (SSM=1, SSI=1) to
    // prevent any MODF error
    cr1 |= SSM | SSI;
  }

  // g) Configure the MSTR bit
  cr1 |= MSTR;

  // 3. configure CR2
  // a) Configure the DS[3:0] bits to select the data length for the transfer
  cr2 |= DS & (cfg.data_size << DS_POS);
  // b) Configure SSOE
  if (cfg.use_hw_cs)
    cr2 |= SSOE;
  // c) Set the FRF bit if the TI protocol is required
  // -> not used here

  // d) Set the NSSP bit if the NSS pulse mode between two data units is
  // required
  if (cfg.cs_pulse)
    cr2 |= NSSP;

  // e) Configure the FRXTH bit. The RXFIFO threshold must be aligned to the
  // read access size for the SPIx_DR register -> 8 bit for data sizes <= 8, 16
  // bit for data sizes > 8.
  if (cfg.data_size <= 8)
    cr2 |= FRXTH;

  // f) TODO: Initialize LDMA_TX and LDMA_RX bits if DMA is used in packed mode.

  // 4. configure crc
  if (cfg.crc != Crc::none)
    crcpr = cfg.crc_polynomial;
  else
    crcpr = 0x7; // reset value

  // TODO:
  // 5. configure dma

  // 6. Enable the clock and actually apply the settings
  SPI1->CR1 = cr1;
  SPI1->CR2 = cr2;
  SPI1->CRCPR = crcpr;

  return hal::spi::Device{*SPI1, cs};
}

extern "C" void SPI1_IRQHandler(void) {
  // TODO: implement SPI IRQ handler
}
