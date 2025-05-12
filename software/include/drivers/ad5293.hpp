/**
 * @file drivers/ad5293.hpp
 * @brief This file contains the driver for the AD5293 digital potentiometer
 * from Analog Devices.
 */

#ifndef AD5293_HPP
#define AD5293_HPP

#include "hal/enums.hpp"
#include "hal/spi.hpp"
#include <cstdint>
namespace ad5293 {

/// mode enumeration
enum class Mode {
  performance = 0, //< performance mode (default)
  normal = 1       //< normal mode
};

/**
 * @class AD5293
 * @brief This is a driver for the AD5293 10 bit digital potentiometer from
 * Analog Devices.
 * It uses a hal::spi::Device. It assumes that the RESET pin is permanently
 * pulled high.
 *
 * All operations that can fail because of io return a
 * hal::Error value.
 */
class AD5293 {
public:
  /**
   * @brief this will create an invalid driver. One of the init() overloads must
   * be called before using this.
   */
  constexpr AD5293() = default;

  /**
   * @brief create a device driver with an spi device. spi must be valid.
   *
   * @param spi the spi device to use for communication.
   */
  constexpr AD5293(hal::spi::Device spi) : spi_(std::move(spi)) {}

  /**
   * @brief create a device driver with an spi device. spi must be valid.
   *
   * @param spi the spi device to use for communication.
   */
  constexpr hal::Error init(hal::spi::Device spi) {
    spi_ = std::move(spi);
    if (not spi_.is_valid())
      return hal::Error::invalid_handle;

    control_ = 0;
    code_ = 512;
    return disable();
  }

  /// @brief enable the device. This will also release the write protection of
  /// the device.
  constexpr hal::Error enable() {
    control_ &= ~1u;
    control_ &= 0b111;
    hal::Error error = write(SOFT_POWER_DOWN);
    if (error != hal::Error::none)
      return error;
    return write_protection(false);
  }

  /// @brief disable the device
  constexpr hal::Error disable() {
    control_ |= 1;
    control_ &= 0b111;
    return write(SOFT_POWER_DOWN | 1);
  }

  /// @brief reset the device. This will set the wiper position to mid scale.
  constexpr hal::Error reset() {
    code_ = 512;
    return write(RESET);
  }

  /// @brief set the mode of the device to performance (default) or normal.
  constexpr hal::Error mode(Mode mode) {
    switch (mode) {
    case Mode::performance:
      control_ &= ~(1u << 2);
      break;
    case Mode::normal:
      control_ |= 1u << 2;
      break;
    default:
      return hal::Error::invalid_param;
    }
    control_ &= 0b111;
    return write(WRITE_CONTROL | control_);
  }

  /**
   * @brief enable or disable software write protection. If enabled, the wiper
   * position cannot be changed until the protection is disabled.
   */
  constexpr hal::Error write_protection(bool enable) {
    if (enable)
      control_ &= ~(1 << 1);
    else {
      control_ |= 1 << 1;
    }
    control_ &= 0b111;
    return write(WRITE_CONTROL | control_);
  }

  /**
   * @brief set the wiper position to code.
   *
   * R_wb = code / 1024 * R_ab
   * R_wa = (1024 - code) * R_ab
   */
  constexpr hal::Error code(uint16_t code) {
    code &= code_mask;
    code |= WRITE_RDAC;
    if (auto err = write(code); err != hal::Error::none)
      return err;
    code_ = code & code_mask;
    return hal::Error::none;
  }

  /// returns true if the device is enabled
  constexpr bool is_enabled() const { return (control_ & 1u) == 0; }

  /// returns the mode of the device
  constexpr Mode mode() const {
    return (control_ >> 2) == 1 ? Mode::normal : Mode::performance;
  }

  /// returns true if the device is write protected
  constexpr bool write_protection() const { return (control_ & (1 << 1)) == 0; }

  /// returns the current wiper position
  constexpr uint16_t code() const { return code_; }

private:
  hal::Error write(uint16_t value) {
    std::uint8_t buf[2]{static_cast<std::uint8_t>(value >> 8),
                        static_cast<std::uint8_t>(value)};
    return spi_.write(buf);
  }

  enum Cmd : std::uint16_t {
    NOP = 0,
    WRITE_RDAC = 1 << 10,
    READ_RDAC = 1 << 11,
    RESET = 1 << 13,
    WRITE_CONTROL = 0b110 << 10,
    READ_CONTROL = 0b111 << 10,
    SOFT_POWER_DOWN = 1 << 13
  };

  enum { code_mask = 0b1111111111u };

  hal::spi::Device spi_;
  uint8_t control_ = 0;
  uint16_t code_ = 512;
};
} // namespace ad5293
#endif
