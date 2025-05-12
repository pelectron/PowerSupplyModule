#ifndef MOCK_HAL_HAL_HPP
#define MOCK_HAL_HAL_HPP
#include "hal/hal.hpp"
#include "mock/hal/gpio.hpp"
#include "mock/hal/i2c.hpp"
#include "mock/hal/spi.hpp"
#include "mock/hal/uart.hpp"
#include <map>
namespace hal {

struct MockHandle {

  constexpr Error init() noexcept { return hal::Error::none; }

  Error configure(const gpio::Config &cfg) noexcept {
    auto res = gpio::configure(cfg);
    if (not res)
      return hal::Error::config /* res.error */;
    gpio_handles[cfg.port][cfg.pins] = std::move(res.peripheral);
    return hal::Error::none;
  }

  Error configure(const i2c::Config &cfg) noexcept {
    auto res = i2c::configure(cfg);
    if (not res)
      return hal::Error::config /* res.error */;
    i2c_handles[cfg.id] = std::move(res.peripheral);
    return hal::Error::none;
  }

  Error configure(const spi::Config &cfg) noexcept {
    auto res = spi::configure(cfg);
    if (not res)
      return hal::Error::config /* res.error */;
    spi_handles[cfg.id] = std::move(res.peripheral);
    return hal::Error::none;
  }

  Error configure(const uart::Config &cfg) noexcept {
    auto res = uart::configure(cfg);
    if (not res)
      return hal::Error::config /* res.error */;
    uart_handles[cfg.id] = std::move(res.peripheral);
    return hal::Error::none;
  }

  gpio::Pin create(gpio::Port port, uint32_t pins) noexcept {
    if (not gpio_handles.contains(port) or
        not gpio_handles[port].contains(pins))
      return {};
    return std::move(gpio_handles[port][pins]);
  }

  i2c::Device create(i2c::Id id, uint8_t address) noexcept {
    if (not i2c_handles.contains(id))
      return {};
    return {i2c_handles[id], address};
  }

  spi::Device create(spi::Id id, gpio::Pin pin) noexcept {
    if (not spi_handles.contains(id))
      return {};
    return {spi_handles[id], std::move(pin)};
  }

  uart::Device create(uart::Id id) noexcept {
    if (not uart_handles.contains(id))
      return {};
    return {uart_handles[id]};
  }

  std::map<gpio::Port, std::map<unsigned, gpio::Pin>> gpio_handles;
  std::map<i2c::Id, i2c::HandleOwner> i2c_handles;
  std::map<spi::Id, spi::HandleOwner> spi_handles;
  std::map<uart::Id, uart::HandleOwner> uart_handles;
};
} // namespace hal
#endif
