#ifndef AT24CS08_HPP
#define AT24CS08_HPP

#include "hal/gpio.hpp"
#include "hal/i2c.hpp"
#include "tl/expected.hpp"

#include <array>
#include <cstdint>
#include <span>

namespace at24cs08 {
struct SerialNumber {
  constexpr auto operator<=>(const SerialNumber &) const noexcept = default;

  std::array<std::uint8_t, 16> data{};
};

class AT24CS08 {
public:
  void init(hal::i2c::HandleRef i2c, hal::gpio::Pin wp, bool A2) {
    i2c_ = hal::i2c::Device(i2c, 0);
    wp_ = std::move(wp);
    A2_ = A2;
  }

  hal::Error write(std::uint32_t address, std::span<const uint8_t> data) {

    auto write_up_to_16 = [this](std::uint32_t mem_addr,
                                 std::span<const uint8_t> data,
                                 bool A2) -> hal::Error {
      std::uint8_t addr = make_address(mem_addr, A2);
      std::uint8_t buffer[17]{static_cast<std::uint8_t>(mem_addr)};
      for (std::size_t i = 0; i < data.size() and i < 16; ++i) {
        buffer[i + 1] = data[i];
      }
      i2c_.set_address(addr);
      wp_.set(hal::gpio::State::reset);
      hal::Error err = i2c_.write({buffer, data.size() + 1});
      wp_.set(hal::gpio::State::set);
      return err;
    };

    while (not data.empty()) {
      if (data.size() <= 16)
        return write_up_to_16(address, data, A2_);
      else {
        hal::Error err = write_up_to_16(address, data.subspan(0, 16), A2_);
        if (err != hal::Error::none) {
          return err;
        }
      }
      address += 16;
    }
    return hal::Error::none;
  }

  hal::Error read(std::uint32_t address, std::span<uint8_t> data) {
    const auto addr = make_address(address, A2_);
    const std::uint8_t wr_buf[1]{static_cast<std::uint8_t>(address)};
    i2c_.set_address(addr);
    hal::Error err = i2c_.write(wr_buf);
    if (err != hal::Error::none)
      return err;
    return i2c_.read(data);
  }

  tl::expected<SerialNumber, hal::Error> serial_number() {
    const auto dev_addr = make_address(0x80, A2_);
    const auto sn_addr = static_cast<std::uint8_t>(
        (0b1011u << 3u) | static_cast<std::uint8_t>(A2_ ? 1u << 2u : 0u));

    SerialNumber sn{};
    i2c_.set_address(dev_addr);
    const std::uint8_t wr_buf[1]{0x80u};
    hal::Error err = i2c_.write(wr_buf);
    if (err != hal::Error::none)
      return tl::unexpected{err};

    i2c_.set_address(sn_addr);
    err = i2c_.read(sn.data);
    if (err != hal::Error::none)
      return tl::unexpected{err};
    else
      return sn;
  }

private:
  static constexpr std::uint8_t make_address(std::uint32_t mem_addr, bool A2) {
    return static_cast<std::uint8_t>(
        (0b1010u << 3u) | static_cast<std::uint8_t>(A2 ? 1u << 2u : 0u) |
        static_cast<std::uint8_t>(0b11u & (mem_addr >> 8u)));
  }
  hal::gpio::Pin wp_;
  hal::i2c::Device i2c_;
  bool A2_ = false;
};
} // namespace at24cs08
#endif
