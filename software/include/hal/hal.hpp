#ifndef HAL_HAL_HPP
#define HAL_HAL_HPP

#include "hal/adc.hpp"
#include "hal/enums.hpp"
#include "hal/gpio.hpp"
#include "hal/i2c.hpp"
#include "hal/spi.hpp"
#include "hal/uart.hpp"

#include <poly.hpp>
#include <tl/expected.hpp>

namespace hal {

// clang-format off
template<class Storage>
using Handle = poly::Struct<
  Storage,
  poly::type_list<>,
  poly::type_list<
    Error (init) noexcept,
    void (deinit) noexcept,
    tl::expected<adc::Adc, Error> (configure, const adc::Config &cfg) noexcept,
    tl::expected<gpio::Pin, Error> (configure, const gpio::Config &cfg) noexcept,
    tl::expected<i2c::Device, Error> (configure, const i2c::Config &cfg) noexcept,
    tl::expected<spi::Device, Error> (configure, const spi::Config &cfg) noexcept,
    tl::expected<uart::Device, Error> (configure, const uart::Config &cfg) noexcept
>>;
// clang-format on

using HandleRef = Handle<poly::ref_storage>;

struct NullHal {

  constexpr Error init() noexcept { return hal::Error::none; }

  constexpr void deinit() noexcept {}

  tl::expected<adc::Adc, Error> configure(const adc::Config &) noexcept {
    return tl::unexpected(hal::Error::not_implemented);
  }

  constexpr tl::expected<gpio::Pin, Error>
  configure(const gpio::Config &) noexcept {
    return tl::unexpected(hal::Error::not_implemented);
  }

  constexpr tl::expected<i2c::Device, Error>
  configure(const i2c::Config &) noexcept {
    return tl::unexpected<Error>(hal::Error::not_implemented);
  }

  constexpr tl::expected<spi::Device, Error>
  configure(const spi::Config &) noexcept {
    return tl::unexpected(hal::Error::not_implemented);
  }

  tl::expected<uart::Device, Error> configure(const uart::Config &) noexcept {
    return tl::unexpected(hal::Error::not_implemented);
  }
};

class Hal {
  HandleRef handle_;

public:
  constexpr Hal() = default;
  constexpr Hal(HandleRef handle) : handle_(handle) {}
  constexpr Hal(Hal &&) = default;
  constexpr Hal &operator=(Hal &&) = default;

  constexpr bool is_valid() const noexcept {
    return static_cast<bool>(handle_);
  }

  constexpr Error init() noexcept { return handle_.init(); }

  tl::expected<adc::Adc, Error> configure(const adc::Config &cfg) noexcept {
    return handle_.configure(cfg);
  }

  tl::expected<gpio::Pin, Error> configure(const gpio::Config &cfg) noexcept {
    return handle_.configure(cfg);
  }

  tl::expected<i2c::Device, Error> configure(const i2c::Config &cfg) noexcept {
    return handle_.configure(cfg);
  }

  tl::expected<spi::Device, Error> configure(const spi::Config &cfg) noexcept {
    return handle_.configure(cfg);
  }

  tl::expected<uart::Device, Error>
  configure(const uart::Config &cfg) noexcept {
    return handle_.configure(cfg);
  }
};

Hal get_hal() noexcept;

} // namespace hal

#endif
