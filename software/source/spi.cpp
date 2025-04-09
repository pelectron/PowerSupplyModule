#include "hal/spi.hpp"
#include "hal/clocks.hpp"
#include "hal/config.hpp"
#include "hal/gpio.hpp"
#include "hal/peripheral.hpp"
#include "mmio.hpp"
#include <cstdint>

#ifndef HAL_SPI_MAX_NUM_TRANSACTIONS
#define HAL_SPI_MAX_NUM_TRANSACTIONS 5
#endif

#define SPI1 reinterpret_cast<port_type *>(SPI1_ADDR)

namespace hal::spi {

enum : uint32_t { SPI1_ADDR = 0x40012C00UL };

enum CR1 : uint32_t {
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
enum CR2 : uint32_t {
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
enum SR : uint32_t {
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
enum : uint32_t { DR_MASK = 0xFF };
enum IRQs {
  IRQ_TX_EMPTY = TXEIE,
  IRQ_RX_NOT_EMPTY = RXNEIE,
  IRQ_ERROR = ERRIE
};

enum FifoLevel {
  fifo_empty = 0,
  fifo_one_fourth = 1,
  fifo_one_half = 2,
  fifo_full = 3
};

constexpr IRQs operator|(IRQs a, IRQs b) {
  return static_cast<IRQs>(static_cast<unsigned>(a) | static_cast<unsigned>(b));
}

static Operation *volatile current_transaction = nullptr;
static Operation *volatile last_transaction = nullptr;
struct port_type {
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
};

void add_operation(Operation *op) {
  if (op == nullptr)
    return;
  if (current_transaction == nullptr) {
    current_transaction = op;
    last_transaction = op;
    op->master->port->start(op);
  } else {
    last_transaction->next = op;
    last_transaction = op;
  }
}

void pop_operation_and_start_next() {
  const auto op = current_transaction;
  if (op == nullptr) {
    return;
  }
  current_transaction = op->next;

  if (op->next == nullptr) {
    last_transaction = nullptr;
    return;
  }

  op->next = nullptr;

  current_transaction->master->port->start(current_transaction);
}

ConfigResult<Master> configure(const MasterConfig &cfg) {
  // first, validate the config

  // check the id
  if (cfg.id != Id::A)
    return ConfigError::invalid_id;

  // check the gpios
  using namespace gpio;
  // map of gpio ids to alternate function number
  constexpr std::pair<gpio::Id, uint8_t> sclk_pins[] = {
      {Port::A | 1, 0}, {Port::A | 5, 0}, {Port::B | 3, 0}, {Port::B | 6, 10}};

  constexpr std::pair<gpio::Id, uint8_t> mosi_pins[] = {{Port::A | 2, 0},
                                                        {Port::A | 7, 0},
                                                        {Port::A | 12, 0},
                                                        {Port::B | 5, 0},
                                                        {Port::B | 6, 8}};

  constexpr std::pair<gpio::Id, uint8_t> miso_pins[] = {
      {Port::A | 6, 0}, {Port::A | 11, 0}, {Port::B | 4, 0}, {Port::B | 6, 9}};

  constexpr std::pair<gpio::Id, uint8_t> nss_pins[] = {{Port::A | 4, 0},
                                                       {Port::A | 8, 8},
                                                       {Port::A | 14, 8},
                                                       {Port::A | 15, 0},
                                                       {Port::B | 0, 0}};

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
  if (not gpio::pin_exists(cfg.sclk))
    return ConfigError::invalid_sclk;
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
  if (not gpio::pin_exists(cfg.mosi))
    return ConfigError::invalid_mosi;
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
  if (not gpio::pin_exists(cfg.miso))
    return ConfigError::invalid_miso;
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

  if (cs_cfg.id != gpio::Id::invalid) {
    res = gpio::configure(cs_cfg);
    if (res.error != ConfigError::success)
      return res.error;
  }

  std::uint32_t cr1 = 0;
  std::uint32_t cr2 = 0;
  std::uint32_t crcpr = 0;

  // 2. configure CR1
  // a) configure baudrate divisor
  std::uint32_t clk_rate = hal::clock::frequency(Peripheral::spi1);
  if (cfg.baudrate > clk_rate / 2)
    return ConfigError::invalid_baudrate;

  auto div = 2;
  while (clk_rate / div > cfg.baudrate and div < 256) {
    div *= 2;
  }
  if (clk_rate / div > cfg.baudrate)
    return ConfigError::invalid_baudrate;

  cr1 = div << BR_POS;

  // b) Configure the CPOL and CPHA
  cr1 |= cfg.polarity == Polarity::high ? CPOL : 0;
  cr1 |= cfg.phase == Phase::high ? CPHA : 0;

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
  default:
    break;
  }

  // f) Configure SSM and SSI
  //    -> not used here. Either controlled through hardware, or done through a
  //    hal::gpio::Output.

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

  // f) Initialize LDMA_TX and LDMA_RX bits if DMA is used in packed mode.
  // 4. configure crc
  if (cfg.crc != Crc::none)
    crcpr = cfg.crc_polynomial;
  else
    crcpr = 0x7; // reset value

  // TODO:
  // 5. configure dma

  // 6. Enable the clock and actually apply the settings
  clock::enable(Peripheral::spi1);
  SPI1->CR1 = cr1;
  SPI1->CR2 = cr2;
  SPI1->CRCPR = crcpr;

  Master master;
  master.port = SPI1;
  return master;
}

void Master::async_write(std::span<const uint8_t> buf, Callback &&cb) {
  if (buf.size() == 0)
    return;

  transaction.callback = std::move(cb);
  transaction.write_buf = buf;
  transaction.type = Operation::Type::Write;
  transaction.write_idx = 0;
  add_operation(&transaction);
}

void Master::async_read(std::span<uint8_t> buf, Callback &&cb) {
  if (buf.size() == 0)
    return;

  transaction.callback = std::move(cb);
  transaction.write_buf = buf;
  transaction.type = Operation::Type::Write;
  transaction.write_idx = 0;
  add_operation(&transaction);
}

void Master::async_transceive(std::span<const uint8_t> write_buf,
                              std::span<uint8_t> read_buf, Callback &&cb) {
  if (write_buf.size() == 0 and read_buf.size() == 0)
    return;

  transaction.callback = std::move(cb);
  transaction.write_buf = write_buf;
  transaction.read_buf = read_buf;
  transaction.type = Operation::Type::ReadWrite;
  transaction.write_idx = 0;
  transaction.read_idx = 0;
  add_operation(&transaction);
}

static MasterConfig config;

void spi_irq_callback() {
  Operation *t = current_transaction;
  if (t == nullptr)
    return;

  auto const port = t->master->port;

  // check interrupt source
  const auto errors = port->errors();

  if (errors != 0) {
    Error e = Error::success;
    if (errors & OVR)
      e = e | Error::rx_overrun;
    if (errors & MODF)
      e = e | Error::mode_fault;
    if (errors & CRCERR)
      e = e | Error::crc_error;
    if (errors & FRE)
      e = e | Error::frame_error;
    auto *t = current_transaction;
    current_transaction = current_transaction->next;
    t->next = nullptr;
    t->invoke_callback(e);
    return;
  }

  auto type = t->type;
  auto read_idx = t->read_idx;
  const auto read_data = t->read_buf.data();
  const auto read_size = t->read_buf.size();

  // check if should be receiving and rx fifo is not empty
  if ((type & Operation::Type::Read) == Operation::Type::Read and
      port->rx_fifo_not_empty()) {
    while (port->rx_fifo_level() != fifo_empty) {
      if (config.data_size <= 8) {
        read_data[read_idx++] = port->getc<uint8_t>();
      } else {
        *reinterpret_cast<uint16_t *>(read_data + read_idx) =
            port->getc<uint16_t>();
        read_idx += 2;
      }
    }
    t->read_idx = read_idx;
  }

  const auto wr_size = t->write_buf.size();
  const auto wr_data = t->write_buf.data();
  auto wr_idx = t->write_idx;
  // check if should be writing and tx fifo has space
  if ((type & Operation::Type::Write) == Operation::Type::Write and
      port->tx_fifo_empty()) {
    if (config.data_size <= 8)
      while (wr_idx < wr_size and port->tx_fifo_level() != fifo_full) {
        if (config.data_size <= 8) {
          port->putc(wr_data[wr_idx++]);
        } else {
          port->putc(*reinterpret_cast<const uint16_t *>(wr_data + wr_idx));
          wr_idx += 2;
        }
      }
    else
      while (wr_idx < wr_size and
             hal::mmio::get(SPI1->SR, FTLVL, FTLVL_POS) != 0b11) {
        SPI1->DR = *reinterpret_cast<const uint16_t *>(wr_data + wr_idx);
        wr_idx += 2;
      }
    t->write_idx = wr_idx;
  }

  bool finished = false;
  switch (type) {
  case Operation::Type::Read:
    if (read_size == read_idx) {
      // read finished
      finished = true;
      SPI1->disable_irqs(IRQ_RX_NOT_EMPTY | IRQ_ERROR);
    }
    break;
  case Operation::Type::Write:
    if (wr_size == wr_idx) {
      // write finished
      finished = true;
      SPI1->disable_irqs(IRQ_TX_EMPTY | IRQ_ERROR);
    }
    break;
  case Operation::Type::ReadWrite:
    if (wr_size == wr_idx and read_size == read_idx) {
      // read write finished
      finished = true;
    }
    break;
  default:
    break;
  }

  if (finished) {
    current_transaction = t->next;
    t->next = nullptr;
    t->invoke_callback(Error::success);

    if (current_transaction == nullptr) {
      SPI1->disable_irqs(IRQ_TX_EMPTY | IRQ_RX_NOT_EMPTY | IRQ_ERROR);
      return;
    }
  }
}
} // namespace hal::spi
using namespace hal::spi;

extern "C" void SPI1_IRQHandler() { hal::spi::spi_irq_callback(); }
