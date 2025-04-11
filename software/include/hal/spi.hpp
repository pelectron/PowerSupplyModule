#ifndef HAL_SPI_HPP
#define HAL_SPI_HPP

#include "config.hpp"
#include "hal/gpio.hpp"
#include <cortex/function.hpp>
#include <span>
namespace hal::spi {
enum class Id {
  invalid = 0,
  A,
  B,
  C,
  D,
  E,
  F,
  G,
  H,
  I,
  J,
  K,
  L,
  M,
  N,
  O,
  P,
  Q,
  R,
  S,
  T,
  U,
  V,
  W,
  X,
  Y,
  Z
};

enum class Phase { low, high };

enum class Polarity { low, high };

enum class Format { lsb_first, msb_first };

enum class Crc { none, eight_bit, sixteen_bit };

enum class Error {
  success,
  busy,
  rx_overrun,
  mode_fault,
  crc_error,
  frame_error,
};

constexpr Error operator|(Error a, Error b) {
  return static_cast<Error>(static_cast<unsigned>(a) |
                            static_cast<unsigned>(b));
}

struct MasterConfig {
  Id id;
  hal::gpio::Id sclk;
  hal::gpio::Id mosi;
  hal::gpio::Id miso;
  hal::gpio::Id cs;
  Phase phase = Phase::low;
  Polarity polarity = Polarity::low;
  Format format = Format::msb_first;
  std::uint32_t baudrate = 1'000'000;
  uint8_t data_size = 8;
  bool use_hw_cs = false;
  Crc crc = Crc::none;
  std::uint32_t crc_polynomial = 0;
  bool three_wire = false;
  bool cs_pulse;
};

class BytesRead {
public:
  explicit constexpr BytesRead(std::uint32_t value) noexcept : value_(value) {}
  constexpr operator std::uint32_t() const noexcept { return value_; }
  constexpr std::uint32_t value() const { return value_; }

private:
  std::uint32_t value_;
};

struct BytesWritten {
public:
  explicit constexpr BytesWritten(std::uint32_t value) noexcept
      : value_(value) {}
  constexpr operator std::uint32_t() const noexcept { return value_; }
  constexpr std::uint32_t value() const { return value_; }

private:
  std::uint32_t value_;
};

struct port_type;
struct op_state;

enum class State {
  disabled = 1 << 0,
  idle = 1 << 2,
  busy = 1 << 3,
  error = 0b111
};

using Callback = cm::MoveOnlyFunction<uint8_t(Error, BytesRead, BytesWritten),
                                      16, alignof(void *)>;

class Master;
struct Operation {
  enum class Type { none = 0, Read = 0b01, Write = 0b10, ReadWrite = 0b11 };
  constexpr friend Type operator&(Type a, Type b) {
    return static_cast<Type>(static_cast<unsigned>(a) &
                             static_cast<unsigned>(b));
  }

  void invoke_callback(Error e) {
    callback(e, BytesRead(read_idx), BytesWritten(write_idx));
  }

  Master *master = nullptr;
  Callback callback{};
  std::span<const uint8_t> write_buf{};
  std::span<uint8_t> read_buf{};
  volatile size_t read_idx = 0;
  volatile size_t write_idx = 0;
  Operation *next = nullptr;
  Type type = Type::none;
};

class Master {
public:
  void async_write(std::span<const uint8_t> buf, Callback &&cb);
  void async_read(std::span<uint8_t> buf, Callback &&cb);
  void async_transceive(std::span<const uint8_t> write_buf,
                        std::span<uint8_t> read_buf, Callback &&cb);

private:
  friend ConfigResult<Master> configure(const MasterConfig &cfg);
  friend void spi_irq_callback();
  friend void add_operation(Operation *op);
  friend void pop_operation_and_start_next();
  port_type *port = nullptr;
  State state = State::disabled;
  gpio::Output cs{};
  Operation transaction{};
};

ConfigResult<Master> configure(const MasterConfig &cfg);
} // namespace hal::spi

#endif
