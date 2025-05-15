#ifndef MCP45HVX1_HPP
#define MCP45HVX1_HPP

#include "hal/enums.hpp"
#include "hal/gpio.hpp"
#include "hal/i2c.hpp"
#include <cstdint>
namespace mcp45hvx1 {

enum class Terminal { A = 1 << 2, W = 1 << 1, B = 1 << 0 };

struct RegisterMap {
  std::uint8_t wiper = 0x7Fu;
  std::uint8_t control = 0xFFu;
};

class MCP54HVX1 {
public:
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
    wlat.set(hal::gpio::State::reset);
    hal::Error error = i2c_.write(data);
    wlat.set(hal::gpio::State::set);
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
  hal::gpio::Output wlat;
  RegisterMap register_map{};
};
} // namespace mcp45hvx1

#endif
