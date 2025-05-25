#ifndef MOCK_HAL_HAL_HPP
#define MOCK_HAL_HAL_HPP
#include "hal/enums.hpp"
#include "hal/hal.hpp"
#include "hal/i2c.hpp"
#include "hal/spi.hpp"
#include "hal/uart.hpp"
#include "mock/hal/gpio.hpp"
#include "mock/hal/i2c.hpp"
#include "mock/hal/spi.hpp"
#include "mock/hal/uart.hpp"
#include <map>
namespace hal {

struct MockHal {

  constexpr Error init() noexcept { return hal::Error::none; }

  constexpr void deinit() noexcept {
    gpio_handles.clear();
    i2c_handles.clear();
    spi_handles.clear();
    uart_handles.clear();
  }

  tl::expected<adc::Adc, Error> configure(const adc::Config &) noexcept {
    return tl::unexpected(hal::Error::not_implemented);
  }

  tl::expected<gpio::Pin, Error> configure(const gpio::Config &cfg) noexcept {
    return gpio::Pin(gpio_handles[cfg.port] = gpio::MockHandle{cfg.port, 0},
                     cfg.pins);
  }

  tl::expected<i2c::Device, Error> configure(const i2c::Config &cfg) noexcept {
    return i2c::Device(i2c_handles[cfg.id] = i2c::MockHandle{cfg}, cfg.address);
  }

  tl::expected<spi::Device, Error> configure(const spi::Config &cfg) noexcept {
    return configure(
               gpio::OutputConfig{gpio::port(cfg.cs), gpio::pin_nr(cfg.cs),
                                  gpio::Mode::push_pull, gpio::Speed::slow,
                                  gpio::Pull::none, gpio::State::set})
        .and_then([&](gpio::Pin pin) -> tl::expected<spi::Device, Error> {
          return spi::Device(spi_handles[cfg.id] = spi::MockHandle{cfg},
                             std::move(pin));
        });
  }

  tl::expected<uart::Device, Error>
  configure(const uart::Config &cfg) noexcept {
    return uart::Device(uart_handles[cfg.id] = uart::MockHandle{cfg}, 0);
  }

  std::map<gpio::Port, gpio::MockHandle> gpio_handles;
  std::map<i2c::Id, i2c::HandleOwner> i2c_handles;
  std::map<spi::Id, spi::HandleOwner> spi_handles;
  std::map<uart::Id, uart::HandleOwner> uart_handles;
};
} // namespace hal
#endif
