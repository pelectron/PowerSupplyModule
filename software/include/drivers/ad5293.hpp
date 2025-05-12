#ifndef AD5293_HPP
#define AD5293_HPP

#include "hal/enums.hpp"
#include "hal/spi.hpp"
#include <cstdint>
namespace ad5293 {

enum class Mode { performance = 0, normal = 1 };

class AD5293 {
public:
  constexpr hal::Error enable() {
    control &= ~1u;
    control &= 0b111;
    return write(SOFT_POWER_DOWN);
  }

  constexpr hal::Error disable() {
    control |= 1;
    control &= 0b111;
    return write(SOFT_POWER_DOWN | 1);
  }

  constexpr hal::Error reset() { return write(RESET); }

  constexpr hal::Error mode(Mode mode) {
    switch (mode) {
    case Mode::performance:
      control &= ~(1u << 2);
      break;
    case Mode::normal:
      control |= 1u << 2;
      break;
    default:
      return hal::Error::invalid_param;
    }
    control &= 0b111;
    return write(WRITE_CONTROL | control);
  }

  constexpr hal::Error write_protection(bool enable) {
    if (enable)
      control |= 1 << 1;
    else {
      control &= ~(1 << 1);
    }
    control &= 0b111;
    return write(WRITE_CONTROL | control);
  }

  constexpr hal::Error code(uint16_t code) {
    code &= code_mask;
    code |= WRITE_RDAC;
    if (auto err = write(code); err != hal::Error::none)
      return err;
    code_ = code & code_mask;
    return hal::Error::none;
  }

  constexpr bool is_enabled() const { return (control & 1u) == 0; }

  constexpr Mode mode() const {
    return (control >> 2) == 1 ? Mode::normal : Mode::performance;
  }

  constexpr bool write_protection() const { return (control & (1 << 1)) != 0; }

  constexpr uint16_t code() const { return code_; }

private:
  hal::Error write(uint16_t value) {
    uint8_t buf[2]{static_cast<uint8_t>(value >> 8),
                   static_cast<uint8_t>(value)};
    return spi.write(buf);
  }

  enum Cmd : uint16_t {
    NOP = 0,
    WRITE_RDAC = 1 << 10,
    READ_RDAC = 1 << 11,
    RESET = 1 << 13,
    WRITE_CONTROL = 0b110 << 10,
    READ_CONTROL = 0b111 << 10,
    SOFT_POWER_DOWN = 1 << 13
  };

  enum { code_mask = 0b1111111111u };

  hal::spi::Device spi;
  uint8_t control = 0;
  uint16_t code_ = 0;
};
} // namespace ad5293
#endif
