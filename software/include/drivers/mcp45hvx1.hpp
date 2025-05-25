#ifndef MCP45HVX1_HPP
#define MCP45HVX1_HPP

#include "hal/config.hpp"
#include "hal/enums.hpp"
#include "hal/gpio.hpp"
#include "hal/hal.hpp"
#include "hal/i2c.hpp"
#include "tl/expected.hpp"
#include <cstdint>
namespace mcp45hvx1 {

enum class Terminal { A = 1 << 2, W = 1 << 1, B = 1 << 0 };

constexpr Terminal operator|(Terminal t1, Terminal t2) {
  return static_cast<Terminal>(static_cast<std::uint8_t>(t1) |
                               static_cast<std::uint8_t>(t2));
}

struct RegisterMap {
  std::uint8_t wiper = 0x7Fu;
  std::uint8_t control = 0xFFu;
};

struct Settings {
  hal::i2c::Config i2c;
  hal::gpio::OutputConfig wlat;
  RegisterMap registers;
};

class MCP45HVX1 {
public:
  constexpr hal::Error init(hal::Hal &hal, const Settings &settings) {
    auto res =
        hal.configure(settings.i2c)
            .and_then([this, &settings, &hal](hal::i2c::Device i2c)
                          -> tl::expected<hal::gpio::Pin, hal::Error> {
              if (not i2c.is_valid())
                return tl::unexpected(hal::Error::invalid_handle);
              i2c_ = std::move(i2c);

              return hal.configure(settings.wlat);
            })
            .and_then(
                [this](hal::gpio::Pin wlat) -> tl::expected<void, hal::Error> {
                  if (not wlat.is_valid())
                    return tl::unexpected(hal::Error::invalid_handle);
                  wlat_ = std::move(wlat);
                  return {};
                });
    if (not res)
      return res.error();
    return hal::Error::none;
  }

  constexpr hal::Error enable() {
    std::uint8_t buf[2]{
        TCON << 4, static_cast<std::uint8_t>(register_map.control | (1u << 3))};
    hal::Error error = write(buf);
    if (error == hal::Error::none)
      register_map.control |= 1u << 3;
    return error;
  }

  constexpr hal::Error disable() {
    std::uint8_t buf[2]{TCON << 4, static_cast<std::uint8_t>(
                                       register_map.control &
                                       ~static_cast<std::uint8_t>(1u << 3))};
    hal::Error error = write(buf);
    if (error == hal::Error::none)
      register_map.control &= ~static_cast<std::uint8_t>(1u << 3);
    return error;
  }

  constexpr hal::Error wiper(std::uint8_t code) {
    std::uint8_t buf[2]{0, code};
    hal::Error error = write(buf);
    if (error == hal::Error::none)
      register_map.wiper = code;
    return error;
  }

  constexpr hal::Error increment() {
    std::uint8_t buf = Increment << 2;
    hal::Error error = write({&buf, 1});
    if (error == hal::Error::none)
      register_map.wiper++;
    return error;
  }

  constexpr hal::Error decrement() {
    std::uint8_t buf = Increment << 2;
    hal::Error error = write({&buf, 1});
    if (error == hal::Error::none)
      register_map.wiper--;
    return error;
  }

  constexpr hal::Error connect_terminals(Terminal terminals) {
    std::uint8_t buf[2]{TCON << 4, static_cast<std::uint8_t>(
                                       register_map.control |
                                       static_cast<std::uint8_t>(terminals))};
    hal::Error error = write(buf);
    if (error == hal::Error::none)
      register_map.control |= static_cast<std::uint8_t>(terminals);
    return error;
  }

  constexpr hal::Error disconnect_terminals(Terminal terminals) {
    std::uint8_t buf[2]{TCON << 4, static_cast<std::uint8_t>(
                                       register_map.control &
                                       ~static_cast<std::uint8_t>(terminals))};
    hal::Error error = write(buf);
    if (error == hal::Error::none)
      register_map.control &= ~static_cast<std::uint8_t>(terminals);
    return error;
  }

  constexpr std::uint8_t wiper() const { return register_map.wiper; }

  constexpr Terminal connected_terminals() const {
    return static_cast<Terminal>(register_map.control & 0b111u);
  }

private:
  constexpr hal::Error write(std::span<std::uint8_t> data) {
    wlat_.set(hal::gpio::State::reset);
    hal::Error error = i2c_.write(data);
    wlat_.set(hal::gpio::State::set);
    return error;
  }

  enum Cmd : std::uint8_t {
    WriteData = 0,
    Increment = 1,
    Decrement = 2,
    ReadData = 3
  };
  enum Addr { Wiper = 0x00, TCON = 0x04 };

  hal::i2c::Device i2c_;
  hal::gpio::Output wlat_;
  RegisterMap register_map{};
};
} // namespace mcp45hvx1

#endif
