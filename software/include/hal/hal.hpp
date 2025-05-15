#ifndef HAL_HAL_HPP
#define HAL_HAL_HPP

#include "hal/enums.hpp"
#include "hal/gpio.hpp"
#include "hal/i2c.hpp"
#include "hal/spi.hpp"
#include "hal/uart.hpp"

#include "poly.hpp"
#include "tl/expected.hpp"

namespace hal {

// clang-format off
template<class Storage>
using Handle = poly::Struct<
  Storage,
  poly::type_list<>,
  poly::type_list<
    Error (init),
    tl::expected<gpio::Pin, Error> (configure, const gpio::Config &cfg),
    tl::expected<i2c::Device, Error> (configure, const i2c::Config &cfg),
    tl::expected<spi::Device, Error> (configure, const spi::Config &cfg),
    tl::expected<uart::Device, Error> (configure, const uart::Config &cfg),
    gpio::Pin (create, gpio::Port port, uint32_t pins),
    i2c::Device (create, i2c::Id id, uint8_t address),
    spi::Device (create, spi::Id id, gpio::Pin chip_select),
    uart::Device (create, uart::Id id)
>>;
// clang-format on

using HandleRef = Handle<poly::ref_storage>;

struct NullHal {

  constexpr Error init() noexcept { return hal::Error::none; }

  constexpr tl::expected<gpio::Pin, Error>
  configure(const gpio::Config &) noexcept {
    return tl::unexpected(hal::Error::not_implemented);
  }

  tl::expected<i2c::Device, Error> configure(const i2c::Config &) noexcept {
    return tl::unexpected<Error>(hal::Error::not_implemented);
  }

  tl::expected<spi::Device, Error> configure(const spi::Config &) noexcept {
    return tl::unexpected(hal::Error::not_implemented);
  }

  tl::expected<uart::Device, Error> configure(const uart::Config &) noexcept {
    return tl::unexpected(hal::Error::not_implemented);
  }

  gpio::Pin create(gpio::Port, uint32_t pins) noexcept { return {gpio, pins}; }

  constexpr i2c::Device create(i2c::Id, uint8_t address) noexcept {
    return {i2c, address};
  }

  constexpr spi::Device create(spi::Id, gpio::Pin pin) noexcept {
    return {spi, std::move(pin)};
  }

  constexpr uart::Device create(uart::Id) noexcept {
    return uart::Device(uart);
  }

  static inline constinit gpio::NullHandle gpio{};
  static inline constinit i2c::NullHandle i2c{};
  static inline constinit spi::NullHandle spi{};
  static inline constinit uart::NullHandle uart{};
};

class Hal {
  HandleRef handle_;

public:
  constexpr Hal() = default;
  constexpr Hal(HandleRef handle) : handle_(handle) {}
  constexpr Hal(Hal &&) = default;
  constexpr Hal &operator=(Hal &&) = default;

  constexpr bool is_valid() const { return static_cast<bool>(handle_); }

  constexpr Error init() { return handle_.init(); }

  constexpr tl::expected<gpio::Pin, Error>
  configure(const gpio::Config &cfg) noexcept {
    return handle_.configure(cfg);
  }

  constexpr tl::expected<i2c::Device, Error>
  configure(const i2c::Config &cfg) noexcept {
    return handle_.configure(cfg);
  }

  constexpr tl::expected<spi::Device, Error>
  configure(const spi::Config &cfg) noexcept {
    return handle_.configure(cfg);
  }

  tl::expected<uart::Device, Error>
  configure(const uart::Config &cfg) noexcept {
    return handle_.configure(cfg);
  }

  constexpr gpio::Pin create(gpio::Port port, uint32_t pins) noexcept {
    return handle_.create(port, pins);
  }

  constexpr i2c::Device create(i2c::Id id, uint8_t address) noexcept {
    return handle_.create(id, address);
  }

  constexpr spi::Device create(spi::Id id, gpio::Pin pin) noexcept {
    return handle_.create(id, std::move(pin));
  }

  constexpr uart::Device create(uart::Id id) noexcept {
    return handle_.create(id);
  }
};

Hal get_hal() noexcept;

} // namespace hal

#endif
