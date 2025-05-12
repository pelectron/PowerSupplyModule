#ifndef HAL_SPI_HPP
#define HAL_SPI_HPP

#include "hal/config.hpp"
#include "hal/gpio.hpp"
#include "hal/operations.hpp"

#include <span>

namespace hal::spi {

struct NullHandle {
  constexpr hal::Error write(std::span<const uint8_t>) {
    return hal::Error::none;
  }

  constexpr hal::Error read(std::span<uint8_t>) { return hal::Error::none; }
};

template <class Storage>
using Handle = poly::Struct<
    Storage, poly::type_list<>,
    poly::type_list<hal::Error(hal::write, std::span<const uint8_t> buffer),
                    hal::Error(hal::read, std::span<uint8_t> buffer)>>;

using HandleOwner = Handle<poly::move_only_local_storage<128>>;

using HandleRef = Handle<poly::ref_storage>;

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

// using Callback = cm::MoveOnlyFunction<uint8_t(Error, BytesRead,
// BytesWritten),
//                                       16, alignof(void *)>;

class Device;
struct Operation {
  enum class Type { none = 0, Read = 0b01, Write = 0b10, ReadWrite = 0b11 };
  constexpr friend Type operator&(Type a, Type b) {
    return static_cast<Type>(static_cast<unsigned>(a) &
                             static_cast<unsigned>(b));
  }

  void invoke_callback(Error e) {
    //  callback(e, BytesRead(read_idx), BytesWritten(write_idx));
  }

  Device *master = nullptr;
  // Callback callback{};
  std::span<const uint8_t> write_buf{};
  std::span<uint8_t> read_buf{};
  volatile size_t read_idx = 0;
  volatile size_t write_idx = 0;
  Operation *next = nullptr;
  Type type = Type::none;
};

class Device {
public:
  constexpr Device() = default;

  constexpr Device(HandleRef handle, gpio::Output chip_select)
      : handle_(handle), cs(std::move(chip_select)) {}

  constexpr bool is_valid() const noexcept {
    return static_cast<bool>(handle_);
  }

  constexpr hal::Error write(std::span<const uint8_t> buffer) {
    if (not handle_)
      return hal::Error::invalid_handle;
    cs.set(gpio::State::reset);
    auto err = handle_.write(buffer);
    cs.set(gpio::State::set);
    return err;
  }

  constexpr hal::Error read(std::span<uint8_t> buffer) {
    if (not handle_)
      return hal::Error::invalid_handle;
    cs.set(gpio::State::reset);
    auto err = handle_.read(buffer);
    cs.set(gpio::State::set);
    return err;
  }

private:
  HandleRef handle_{};
  gpio::Output cs{};
};

ConfigResult<HandleOwner> configure(const Config &cfg) noexcept;
} // namespace hal::spi

#endif
