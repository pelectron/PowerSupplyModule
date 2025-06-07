#include "hal/spi.hpp"
#include "clocks.hpp"
#include "hal/config.hpp"
#include "hal/enums.hpp"
#include "hal/gpio.hpp"
#include "hal/mmio.hpp"
#include "hal/operations.hpp"
#include "units.hpp"
#include <cstdint>

#define SPI1 reinterpret_cast<Spi *>(SPI1_ADDR)

using namespace hal;
using namespace hal::spi;

enum : std::uint32_t { SPI1_ADDR = 0x40013000UL };

enum CR1 : std::uint32_t {
  SPI_CR1_BIDIMODE = 1 << 15,
  SPI_CR1_BIDIOE = 1 << 14,
  SPI_CR1_CRCEN = 1 << 13,
  SPI_CR1_CRCNEXT = 1 << 12,
  SPI_CR1_CRCL = 1 << 11,
  SPI_CR1_RXONLY = 1 << 10,
  SPI_CR1_SSM = 1 << 9,
  SPI_CR1_SSI = 1 << 8,
  SPI_CR1_LSBFIRST = 1 << 7,
  SPI_CR1_SPE = 1 << 6,
  SPI_CR1_BR = 0b111 << 3,
  SPI_CR1_BR_POS = 3,
  SPI_CR1_MSTR = 1 << 2,
  SPI_CR1_CPOL = 1 << 1,
  SPI_CR1_CPHA = 1 << 0
};
enum CR2 : std::uint32_t {
  SPI_CR2_LDMA_TX = 1 << 14,
  SPI_CR2_LDMA_RX = 1 << 13,
  SPI_CR2_FRXTH = 1 << 12,
  SPI_CR2_DS = 0xF << 8,
  SPI_CR2_DS_POS = 8,
  SPI_CR2_TXEIE = 1 << 7,
  SPI_CR2_RXNEIE = 1 << 6,
  SPI_CR2_ERRIE = 1 << 5,
  SPI_CR2_FRF = 1 << 4,
  SPI_CR2_NSSP = 1 << 3,
  SPI_CR2_SSOE = 1 << 2,
  SPI_CR2_TXDMAEN = 1 << 1,
  SPI_CR2_RXDMAEN = 1 << 0
};
enum SR : std::uint32_t {
  SPI_SR_FTLVL = 0b11 << 11,
  SPI_SR_FTLVL_POS = 11,
  SPI_SR_FRLVL = 0b11 << 9,
  SPI_SR_FRLVL_POS = 9,
  SPI_SR_FRE = 1 << 8,
  SPI_SR_BSY = 1 << 7,
  SPI_SR_OVR = 1 << 6,
  SPI_SR_MODF = 1 << 5,
  SPI_SR_CRCERR = 1 << 4,
  SPI_SR_UDR = 1 << 3,
  SPI_SR_CHSIDE = 1 << 2,
  SPI_SR_TXE = 1 << 1,
  SPI_SR_RXNE = 1 << 0,
};

enum I2SCFGR : std::uint32_t {
  SPI_I2SCFGR_ASTRTEN = 1u << 12u,
  SPI_I2SCFGR_I2SMOD = 1u << 11u,
  SPI_I2SCFGR_I2SE = 1u << 10u,
  SPI_I2SCFGR_I2SCFG = 0b11u << 8,
  SPI_I2SCFGR_I2SCFG_POS = 8,
  SPI_I2SCFGR_PCMSYNC = 1u << 7u,
  SPI_I2SCFGR_I2SSTD = 0b11u << 4u,
  SPI_I2SCFGR_I2SSTD_POS = 4u,
  SPI_I2SCFGR_CKPOL = 1u << 3u,
  SPI_I2SCFGR_DATLEN = 0b11u << 1,
  SPI_I2SCFGR_DATLEN_POS = 1,
  SPI_I2SCFGR_CHLEN = 1u << 0u
};
enum : std::uint32_t { DR_MASK = 0xFF };
enum IRQs : std::uint32_t {
  IRQ_TX_EMPTY = SPI_CR2_TXEIE,
  IRQ_RX_NOT_EMPTY = SPI_CR2_RXNEIE,
  IRQ_ERROR = SPI_CR2_ERRIE
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
  void enable() { mmio::set_bits(CR1, SPI_CR1_SPE); }

  void disable() {

    if (mmio::get(CR1, SPI_CR1_RXONLY)) {
      // 1. Interrupt the receive flow by disabling SPI (SPE=0) in the specific
      // time window while the last data frame is ongoing.
      mmio::reset_bits(CR1, SPI_CR1_SPE);
      // 2. Wait until BSY=0 (the last data frame is processed).
      while (mmio::get(CR1, SPI_SR_BSY) != 0) {
      }
      // 3. Read data until FRLVL[1:0] = 00 (read all the received data).
      while (auto frlvl = mmio::get(SR, SPI_SR_FRLVL)) {
        if (frlvl == 1 and data_size() <= 8) {
          // TODO: If packing mode is used and an odd number of data frames with
          // a format less than or equal to 8 bits (fitting into one byte) has
          // to be received, FRXTH must be set when FRLVL[1:0] = 01, in order to
          // generate the RXNE event to read the last odd data frame and to keep
          // good FIFO pointer alignment.
          mmio::set_bits(CR2, SPI_CR2_FRXTH);
        }
        mmio::get(DR, DR_MASK);
      }

    } else {
      mmio::reset_bits(CR1, SPI_CR1_SPE);
      // 1. Wait until FTLVL[1:0] = 00 (no more data to transmit).
      while (mmio::get(CR1, SPI_SR_FTLVL) != 0) {
      }
      // 2. Wait until BSY=0 (the last data frame is processed).
      while (mmio::get(CR1, SPI_SR_BSY) != 0) {
      }
      // 3. Disable the SPI (SPE=0).
      mmio::reset_bits(CR1, SPI_CR1_SPE);
      // 4. Read data until FRLVL[1:0] = 00 (read all the received data)
      auto frlvl = mmio::get(SR, SPI_SR_FRLVL, SPI_SR_FRLVL_POS);
      while (frlvl != 0) {
        if (frlvl == 1 and data_size() <= 8) {
          // TODO: If packing mode is used and an odd number of data frames with
          // a format less than or equal to 8 bits (fitting into one byte) has
          // to be received, FRXTH must be set when FRLVL[1:0] = 01, in order to
          // generate the RXNE event to read the last odd data frame and to keep
          // good FIFO pointer alignment.
          mmio::set_bits(CR2, SPI_CR2_FRXTH);
        }
        mmio::get(DR, DR_MASK);
        frlvl = mmio::get(SR, SPI_SR_FRLVL, SPI_SR_FRLVL_POS);
      }
    }
  }

  bool is_enabled() { return mmio::get(CR1, SPI_CR1_SPE) != 0; }

  void enable_irqs(IRQs irqs) { mmio::set_bits(CR2, irqs); }

  void disable_irqs(IRQs irqs) { mmio::reset_bits(CR2, irqs); }
  bool rx_fifo_not_empty() { return hal::mmio::get(SR, SPI_SR_RXNE) != 0; }
  bool tx_fifo_empty() { return hal::mmio::get(SR, SPI_SR_TXE) != 0; }
  std::uint32_t errors() {
    return hal::mmio::get(SR, SPI_SR_OVR | SPI_SR_MODF | SPI_SR_CRCERR |
                                  SPI_SR_FRE);
  }

  FifoLevel rx_fifo_level() {
    return static_cast<FifoLevel>(
        mmio::get(SR, SPI_SR_FRLVL, SPI_SR_FRLVL_POS));
  }

  FifoLevel tx_fifo_level() {
    return static_cast<FifoLevel>(
        mmio::get(SR, SPI_SR_FTLVL, SPI_SR_FTLVL_POS));
  }

  bool is_busy() { return hal::mmio::get(SR, SPI_SR_BSY) != 0; }

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

  void putc(uint8_t c) {
    volatile uint8_t *dr = reinterpret_cast<volatile uint8_t *>(&DR);
    *dr = c;
  }
  void putc(uint16_t c) {
    volatile uint16_t *dr = reinterpret_cast<volatile uint16_t *>(&DR);
    *dr = c;
  }

  template <typename T> T getc() {
    return *reinterpret_cast<volatile T *>(&DR);
  }

  constexpr static hal::Error to_error(std::uint32_t errs) {
    if (errs & SPI_SR_OVR)
      return hal::Error::buffer_overflow;
    if (errs & SPI_SR_MODF)
      return hal::Error::bus_error;
    if (errs & SPI_SR_CRCERR)
      return hal::Error::crc_error;
    if (errs & SPI_SR_FRE)
      return hal::Error::frame_error;
    return hal::Error::none;
  }

  std::size_t data_size() const {
    return hal::mmio::get(CR2, SPI_CR2_DS, SPI_CR2_DS_POS) + 1u;
  }

  hal::Error write(std::span<const std::uint8_t> buffer) {
    if (buffer.size() == 0) {
      return hal::Error::invalid_param;
    }

    if (is_enabled() or is_busy()) {
      return hal::Error::already_in_use;
    }

    enable();
    if (reinterpret_cast<std::uint32_t>(buffer.data()) % 2 != 0 or
        buffer.size() == 1) {
      // buffer is not 16 bit aligned or size is 1 -> write 8 bit data, then
      // write in 16 bit chunks
      putc(buffer[0]);
      buffer = buffer.subspan(1);
    } else {
      // 16 bit aligned and buffer size is at least two -> write in 16 bit
      // chunks
      putc(*reinterpret_cast<const std::uint16_t *>(buffer.data()));
      buffer = buffer.subspan(2);
    }

    while (buffer.size() != 0) {
      // wait for tx empty signal
      while (hal::mmio::get(SR, SPI_SR_TXE) == 0) {
      }
      // write data
      if (buffer.size() > 1) {
        // 16 bit chunk
        putc(*reinterpret_cast<const std::uint16_t *>(buffer.data()));
        buffer = buffer.subspan(2);
      } else {
        // 8 bit chunk
        putc(buffer[0]);
        buffer = buffer.subspan(1);
      }
    }

    if (hal::mmio::get(CR1, SPI_CR1_CRCEN)) {
      // enable crc transmission
      hal::mmio::set_bits(CR1, SPI_CR1_CRCNEXT);
    }

    wait_for_end_of_transaction();

    // clear OVR flag that gets tripped because we are not reading. Only applies
    // when four wire feature is used
    if (hal::mmio::get(CR1, SPI_CR1_BIDIMODE) == 0) {
      volatile auto tmp = DR;
      tmp = SR;
      (void)tmp;
    }

    disable();
    return hal::Error::none;
  }

  hal::Error read(std::span<std::uint8_t> buffer) {
    if (buffer.size() == 0) {
      return hal::Error::invalid_param;
    }

    if (is_enabled() or is_busy()) {
      return hal::Error::already_in_use;
    }

    if (hal::mmio::get(CR1, SPI_CR1_MSTR | SPI_CR1_BIDIMODE) == SPI_CR1_MSTR) {
      // master with four wire comminucation
      return transceive(buffer, buffer);
    } else {
      // set rx only and disable output in bidirectional mode
      mmio::set(CR1, SPI_CR1_RXONLY | SPI_CR1_BIDIOE, SPI_CR1_RXONLY);
      enable();

      const auto size = data_size();
      const auto crc_size = hal::mmio::get(CR1, SPI_CR1_CRCL) ? 16 : 8;
      const bool crc_enabled = hal::mmio::get(CR1, SPI_CR1_CRCEN);
      unsigned crc_num_elems =
          (size <= 8 and crc_size == 8) or size > 8 ? 1 : 2;
      // reset crc if it is used
      if (crc_enabled) {
        // reset crc
        hal::mmio::reset_bits(CR1, SPI_CR1_CRCEN);
        hal::mmio::set_bits(CR1, SPI_CR1_CRCEN);
        // this is done to handle crcnext before the transmission of the last
        // data
        buffer = buffer.subspan(0, buffer.size() - 1);
      }
      // set the fifo threashold according to data size
      if (size > 8) {
        hal::mmio::reset_bits(CR2, SPI_CR2_FRXTH);
      } else {
        hal::mmio::set_bits(CR2, SPI_CR2_FRXTH);
      }

      if (size <= 8) {
        for (auto &byte : buffer) {
          while (hal::mmio::get(SR, SPI_SR_RXNE) == 0) {
          }
          byte = reinterpret_cast<volatile std::uint8_t &>(DR);
        }
      } else {
        while (hal::mmio::get(SR, SPI_SR_RXNE) == 0) {
        }
        while (buffer.size() != 0) {
          *reinterpret_cast<std::uint16_t *>(buffer.data()) =
              static_cast<std::uint16_t>(DR);
          buffer = buffer.subspan(2);
        }
      }

      if (crc_enabled) {
        // freeze crc
        hal::mmio::set_bits(CR1, SPI_CR1_CRCNEXT);
        // wait for last data
        while (hal::mmio::get(SR, SPI_SR_RXNE) == 0) {
        }
        // read last data
        if (size > 8) {
          *reinterpret_cast<std::uint16_t *>(buffer.data()) =
              static_cast<std::uint16_t>(DR);
          buffer = buffer.subspan(2);
        } else {
          buffer[0] = reinterpret_cast<volatile std::uint8_t &>(DR);
          buffer = buffer.subspan(1);
        }
        // wait for crc data
        while (hal::mmio::get(SR, SPI_SR_RXNE) == 0) {
        }
        // read crc data
        if (size == 16) {
          volatile auto tmp = static_cast<std::uint16_t>(DR);
          (void)tmp;
        } else {
          volatile auto tmp = reinterpret_cast<volatile std::uint8_t &>(DR);
          (void)tmp;
          if (size == 8 and crc_size == 16) {
            // wait for crc data
            while (hal::mmio::get(SR, SPI_SR_RXNE) == 0) {
            }
            tmp = reinterpret_cast<volatile std::uint8_t &>(DR);
            (void)tmp;
          }
        }
      }

      wait_for_end_of_transaction();
      auto err = check_and_clear_crc(crc_enabled);

      disable();
      // reset rx only and enable output in bidirectional mode
      mmio::set(CR1, SPI_CR1_RXONLY | SPI_CR1_BIDIOE, SPI_CR1_BIDIOE);
      return hal::Error::none;
    }
    return to_error(errors());
  }

  hal::Error transceive(std::span<const std::uint8_t> write_buf,
                        std::span<std::uint8_t> read_buf) {
    // read and write buf must be the same size
    // and both must be 16 bit aligned
    if (write_buf.size() != read_buf.size() or write_buf.size() == 0 or
        reinterpret_cast<std::uint32_t>(write_buf.data()) % 2 != 0 or
        reinterpret_cast<std::uint32_t>(read_buf.data()) % 2 != 0) {
      return hal::Error::invalid_param;
    }

    enable();

    const auto data_size = this->data_size();
    const bool slave = hal::mmio::get(CR1, SPI_CR1_MSTR) == 0;
    const bool slave_and_pulse_mode =
        slave and hal::mmio::get(CR2, SPI_CR2_NSSP) == SPI_CR2_NSSP;
    const bool crc_enabled = hal::mmio::get(CR1, SPI_CR1_CRCEN);
    bool transmit = true;

    if (data_size > 8) {
      while (write_buf.size() > 0 or read_buf.size() > 0) {
        if (hal::mmio::get(SR, SPI_SR_TXE) and write_buf.size() > 0 and
            transmit) {
          // transmit data
          DR = *reinterpret_cast<const std::uint16_t *>(write_buf.data());
          write_buf = write_buf.subspan(2);
          transmit = false;

          if (write_buf.size() == 0) {
            setup_receive_in_slave_pulse_mode(crc_enabled,
                                              slave_and_pulse_mode);
          }
        }

        if (hal::mmio::get(SR, SPI_SR_RXNE) and read_buf.size() > 0) {
          // read data
          *reinterpret_cast<std::uint16_t *>(read_buf.data()) =
              static_cast<std::uint16_t>(DR);
          read_buf = read_buf.subspan(2);
          transmit = true;
        }
      }
    } else {
      if (slave or write_buf.size() == 1) {
        reinterpret_cast<volatile uint8_t &>(DR) = write_buf[0];
        write_buf = write_buf.subspan(1);
        setup_receive_in_slave_pulse_mode(crc_enabled, slave_and_pulse_mode);
      }

      while (write_buf.size() > 0 or read_buf.size() > 0) {
        if (hal::mmio::get(SR, SPI_SR_TXE) and write_buf.size() > 0 and
            transmit) {
          if (write_buf.size() > 1) {
            DR = *reinterpret_cast<const std::uint16_t *>(write_buf.data());
            write_buf = write_buf.subspan(2);
          } else {
            reinterpret_cast<volatile std::uint8_t &>(DR) = write_buf[0];
            write_buf = write_buf.subspan(1);
          }
          transmit = false;
          if (write_buf.size() == 0 and crc_enabled) {
            // set NSS soft to receive CRC correctly with pulse mode active
            if (slave_and_pulse_mode) {
              hal::mmio::set_bits(CR1, SPI_CR1_SSM);
            }
            // enable crc transmission
            hal::mmio::set_bits(CR1, SPI_CR1_CRCNEXT);
          }
        }

        if (hal::mmio::get(SR, SPI_SR_RXNE) and read_buf.size() > 0) {
          if (read_buf.size() > 1) {
            *reinterpret_cast<std::uint16_t *>(read_buf.data()) =
                reinterpret_cast<volatile std::uint16_t &>(DR);
            read_buf = read_buf.subspan(2);
          } else {
            read_buf[0] = reinterpret_cast<volatile std::uint8_t &>(DR);
            read_buf = read_buf.subspan(1);
          }
          transmit = true;
        }
      }
    }

    if (crc_enabled) {
      // wait for rcr data
      while (hal::mmio::get(SR, SPI_SR_RXNE)) {
      }

      if (data_size == 16) {
        volatile auto tmp = DR;
        (void)tmp;
      } else {
        volatile auto tmp = reinterpret_cast<volatile std::uint8_t &>(DR);
        (void)tmp;
        if (hal::mmio::get(CR1, SPI_CR1_CRCL)) {
          // 16 bit crc
          tmp = reinterpret_cast<volatile std::uint8_t &>(DR);
          (void)tmp;
        }
      }
    }
    wait_for_end_of_transaction();
    disable();
    return check_and_clear_crc(crc_enabled);
  }

  void wait_for_end_of_transaction() {
    while (tx_fifo_level() != fifo_empty) {
    }

    while (is_busy()) {
    }

    while (rx_fifo_level() != fifo_empty) {
      // flush by reading data register
      auto unused = reinterpret_cast<volatile uint8_t &>(DR);
      (void)unused;
    }
  }
  void setup_receive_in_slave_pulse_mode(bool crc_enabled,
                                         bool slave_and_pulse_mode) {
    if (crc_enabled) {
      // set NSS soft to receive CRC correctly with pulse mode active
      if (slave_and_pulse_mode) {
        hal::mmio::set_bits(CR1, SPI_CR1_SSM);
      }
      // enable crc transmission
      hal::mmio::set_bits(CR1, SPI_CR1_CRCNEXT);
    }
  }
  hal::Error check_and_clear_crc(bool crc_enabled) {
    if (crc_enabled and hal::mmio::get(SR, SPI_SR_CRCERR)) {
      SR = ~SPI_SR_CRCERR; // works because all bits in SR can only be set by
                           // hardware
      return hal::Error::crc_error;

    } else {
      return hal::Error::none;
    }
  }
};

static constexpr std::uint32_t next_bigger_pow2(std::uint32_t v) {
  // https://stackoverflow.com/questions/466204/rounding-up-to-next-power-of-2
  --v;
  v |= v >> 1;
  v |= v >> 2;
  v |= v >> 4;
  v |= v >> 8;
  v |= v >> 16;
  ++v;
  return v;
}

ConfigResult<Device> hal::spi::configure(const Config &cfg) noexcept {

  // check the id
  if (cfg.id != Id::A)
    return ConfigError::invalid_id;

  // if (SPI1->is_enabled())
  // return ConfigError::already_in_use;

  // check the gpios
  using namespace gpio;
  // map of gpio ids to alternate function number
  static constexpr std::pair<gpio::Id, uint8_t> sclk_pins[] = {
      {Port::A | Pin1, 0},
      {Port::A | Pin5, 0},
      {Port::B | Pin3, 0},
      {Port::B | Pin6, 10}};

  static constexpr std::pair<gpio::Id, uint8_t> mosi_pins[] = {
      {Port::A | Pin2, 0},
      {Port::A | Pin7, 0},
      {Port::A | Pin12, 0},
      {Port::B | Pin5, 0},
      {Port::B | Pin6, 8}};

  static constexpr std::pair<gpio::Id, uint8_t> miso_pins[] = {
      {Port::A | Pin6, 0},
      {Port::A | Pin11, 0},
      {Port::B | Pin4, 0},
      {Port::B | Pin6, 9}};

  static constexpr std::pair<gpio::Id, uint8_t> nss_pins[] = {
      {Port::A | Pin4, 0},
      {Port::A | Pin8, 8},
      {Port::A | Pin14, 8},
      {Port::A | Pin15, 0},
      {Port::B | Pin0, 0}};

  const auto speed =
      cfg.baudrate > 2'000'000 ? gpio::Speed::medium : gpio::Speed::slow;

  // SCLK
  gpio::Config sclk_cfg{
      cfg.sclk,
      gpio::Function::alternate,
      gpio::Mode::push_pull,
      speed,
      cfg.use_pupd
          ? cfg.polarity == Polarity::low ? gpio::Pull::down : gpio::Pull::up
          : gpio::Pull::none,
      cfg.polarity == Polarity::low ? gpio::State::reset : gpio::State::set,
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
                        speed,
                        cfg.use_pupd ? gpio::Pull::down : gpio::Pull::none,
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
                        gpio::Mode::push_pull,
                        gpio::Speed::slow,
                        cfg.use_pupd ? gpio::Pull::down : gpio::Pull::none,
                        gpio::State::reset,
                        0};

  if (not cfg.three_wire) {
    found = false;
    for (const auto &pair : miso_pins)
      if (pair.first == cfg.miso) {
        found = true;
        miso_cfg.alternate = pair.second;
        break;
      }

    if (not found)
      return ConfigError::invalid_miso;
  }

  // CS/NSS
  gpio::Config cs_cfg{
      cfg.cs,
      cfg.use_hw_cs ? gpio::Function::alternate : gpio::Function::output,
      gpio::Mode::push_pull,
      gpio::Speed::slow,
      cfg.use_hw_cs or cfg.use_pupd ? gpio::Pull::up : gpio::Pull::none,
      gpio::State::set,
      0};

  if (cfg.use_hw_cs) {
    found = false;
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

  if (not cfg.three_wire) {
    res = gpio::configure(miso_cfg);
    if (res.error != ConfigError::success)
      return res.error;
  }

  res = gpio::configure(cs_cfg);
  if (res.error != ConfigError::success)
    return res.error;

  hal::gpio::Pin cs = std::move(res.peripheral);
  if (not cfg.use_hw_cs)
    cs.set(hal::gpio::State::set);

  std::uint32_t CR1 = 0;
  std::uint32_t CR2 = 0;
  std::uint32_t CRCPR = 7;
  // 2. configure CR1
  // a) configure baudrate divisor
  const auto clk_rate = stm32c031xx::clock_tree().pclk;
  const auto div = clk_rate / cfg.baudrate;
  if (div > 256u or div < 2)
    return ConfigError::invalid_baudrate;

  if (div == 2) {
    CR1 |= 0u << SPI_CR1_BR_POS;
  } else if (div <= 4) {
    CR1 |= 1u << SPI_CR1_BR_POS;
  } else if (div <= 8) {
    CR1 |= 2u << SPI_CR1_BR_POS;
  } else if (div <= 16) {
    CR1 |= 3u << SPI_CR1_BR_POS;
  } else if (div <= 32) {
    CR1 |= 4u << SPI_CR1_BR_POS;
  } else if (div <= 64) {
    CR1 |= 5u << SPI_CR1_BR_POS;
  } else if (div <= 128) {
    CR1 |= 6u << SPI_CR1_BR_POS;
  } else {
    // div<=256
    CR1 |= 7 << SPI_CR1_BR_POS;
  }

  // b) Configure the CPOL and CPHA
  CR1 |= cfg.polarity == Polarity::high ? SPI_CR1_CPOL : 0u;
  CR1 |= cfg.phase == Phase::high ? SPI_CR1_CPHA : 0u;

  // c) Select simplex or half-duplex mode by configuring RXONLY or BIDIMODE and
  // BIDIOE (RXONLY and BIDIMODE cannot be set at the same time)
  if (cfg.three_wire)
    CR1 |= SPI_CR1_BIDIMODE;

  // d) Configure the LSBFIRST bit to define the frame format
  if (cfg.format == Format::lsb_first)
    CR1 |= SPI_CR1_LSBFIRST;

  // e) Configure the CRCL and CRCEN bits
  switch (cfg.crc) {
  case Crc::eight_bit:
    CR1 |= SPI_CR1_CRCEN;
    break;
  case Crc::sixteen_bit:
    CR1 |= SPI_CR1_CRCEN | SPI_CR1_CRCL;
    break;
  case Crc::none:
    break;
  default:
    return ConfigError::invalid_crc;
  }

  // f) Configure SSM and SSI.
  if (not cfg.use_hw_cs) {
    // NSS pin is not used on master side in this
    // configuration. It has to be managed internally (SSM=1, SSI=1) to
    // prevent any MODF error
    CR1 |= SPI_CR1_SSM | SPI_CR1_SSI;
  }

  // g) Configure the MSTR bit
  CR1 |= SPI_CR1_MSTR;

  // 3. configure CR2
  // a) Configure the DS[3:0] bits to select the data length for the transfer
  if (cfg.data_size < 4u or cfg.data_size > 16)
    return ConfigError::invalid_data_size;

  CR2 |= SPI_CR2_DS &
         (static_cast<std::uint32_t>(cfg.data_size - 1) << SPI_CR2_DS_POS);

  // b) Configure SSOE
  if (cfg.use_hw_cs)
    CR2 |= SPI_CR2_SSOE;
  // c) Set the FRF bit if the TI protocol is required
  // -> not used here

  // d) Set the NSSP bit if the NSS pulse mode between two data units is
  // required
  if (cfg.cs_pulse)
    CR2 |= SPI_CR2_NSSP;

  // e) Configure the FRXTH bit. The RXFIFO threshold must be aligned to the
  // read access size for the SPIx_DR register -> 8 bit for data sizes <= 8, 16
  // bit for data sizes > 8.
  if (cfg.data_size <= 8)
    CR2 |= SPI_CR2_FRXTH;

  // f) TODO: Initialize LDMA_TX and LDMA_RX bits if DMA is used in packed mode.
  // g) TODO: set fram eformat, for now only motorola is supported
  // 4. configure crc
  if (cfg.crc != Crc::none)
    CRCPR = cfg.crc_polynomial;

  // TODO:
  // 5. configure dma

  // 6. Enable the clock and actually apply the settings
  using namespace stm32c031xx;
  // enable the clock
  hal::mmio::set(RCC->CCIPR, RCC_CCIPR_I2S1SEL, 0, RCC_CCIPR_I2S1SEL_POS);
  hal::mmio::set_bits(RCC->APBENR2, RCC_APBENR2_SPI1EN);
  volatile auto tmp = hal::mmio::get(RCC->APBENR2, RCC_APBENR2_SPI1EN);
  (void)tmp;
  // apply register configuration
  SPI1->CR1 = CR1;
  SPI1->CR2 = CR2;
  SPI1->CRCPR = CRCPR;
  hal::mmio::reset_bits(SPI1->I2SCFGR, SPI_I2SCFGR_I2SMOD);
  return hal::spi::Device{*SPI1, std::move(cs)};
}

extern "C" void SPI1_IRQHandler(void) {
  // TODO: implement SPI IRQ handler
}
