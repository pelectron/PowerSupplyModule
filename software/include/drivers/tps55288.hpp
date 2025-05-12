/**
 * @file drivers/tps55288.hpp
 * @brief This file contains the driver for the TPS55288 buck boost converter IC
 * from TI.
 */

#ifndef PSM_TPS55288_HPP
#define PSM_TPS55288_HPP

#include "hal/config.hpp"
#include "hal/enums.hpp"
#include "hal/gpio.hpp"
#include "hal/hal.hpp"
#include "hal/i2c.hpp"
#include <csignal>
#include <cstdint>

namespace tps55288 {

enum class OcpDelay : std::uint8_t {
  delay_128us = 0,
  delay_3ms = 1,
  delay_6ms = 2,
  delay_12ms = 3
};

enum class SlewRate : std::uint8_t {
  slew_1_25mV = 0,
  slew_2_5mV = 1,
  slew_5mV = 2,
  slew_10mV = 3,
};

enum class FeedbackRatio : std::uint8_t {
  fb_0_2256 = 0, //< 0.2256
  fb_0_1128 = 1, //< 0.1128
  fb_0_0752 = 2, //< 0.0752
  fb_0_0564 = 3  //< 0.0564
};

enum class Errors : std::uint8_t {
  short_circuit = 1 << 2,
  overcurrent = 1 << 1,
  overvoltage = 1 << 0
};

constexpr Errors operator|(Errors f1, Errors f2) noexcept {
  return static_cast<Errors>(static_cast<std::uint8_t>(f1) |
                             static_cast<std::uint8_t>(f2));
}

enum class Address : std::uint8_t {
  address_0x74,
  address_0x75,
};

enum class LightLoadMode : std::uint8_t { PFM = 0, FPWM = 1 };

enum class ModeControl { external_resistor, internal_register };

enum class OperatingMode : std::uint8_t { boost = 0, buck = 1, buck_boost = 2 };

enum class StatusFlags : std::uint8_t {
  short_circuit,
  overcurrent,
  overvoltage
};

constexpr StatusFlags operator|(StatusFlags f1, StatusFlags f2) noexcept {
  return static_cast<StatusFlags>(static_cast<std::uint8_t>(f1) |
                                  static_cast<std::uint8_t>(f2));
}

constexpr StatusFlags operator&(StatusFlags f1, StatusFlags f2) noexcept {
  return static_cast<StatusFlags>(static_cast<std::uint8_t>(f1) |
                                  static_cast<std::uint8_t>(f2));
}

enum class StatusCode : std::uint8_t { invalid = 0b11100u };

constexpr StatusFlags get_flags(StatusCode code) {
  return static_cast<StatusFlags>(static_cast<std::uint8_t>(code) >> 5);
}

constexpr OperatingMode get_mode(StatusCode code) {
  return static_cast<OperatingMode>(static_cast<std::uint8_t>(code) & 0b11u);
}

enum class Mode : std::uint8_t {
  output_enable = 1 << 7,
  fsw_double = 1 << 6,
  hiccup = 1 << 5,
  output_discharge = 1 << 4,
  external_vcc = 1 << 3,
  i2c_address_0x75 = 1 << 2,
  PFM = 1 << 1,
  MODE = 1 << 0,
  MODE_RESET_VALUE = 0b00100000u
};

struct MemoryMap {
  uint16_t ref = 0b11010010u;
  uint8_t iout_limit = 0b11100100u;
  uint8_t vout_sr = 0b00000001u;
  uint8_t vout_fs = 0b00000011u;
  uint8_t cdc = 0b11100000u;
  uint8_t mode = 0b00100000u;
  uint8_t status = 0b00000011;
  constexpr auto operator<=>(const MemoryMap &) const = default;
};

struct Settings {
  hal::i2c::Config i2c;
  hal::gpio::Config enable;
  MemoryMap device_settings;
  constexpr auto operator<=>(const Settings &) const = default;
};

static_assert(sizeof(MemoryMap) == 8);

inline constexpr MemoryMap default_memory_map{};

class TPS55288 {
public:
  constexpr TPS55288() = default;
  constexpr TPS55288(hal::i2c::Device i2c, hal::gpio::Pin enable)
      : i2c_(std::move(i2c)), enable_(std::move(enable)) {
    i2c_.set_address(0x74u);
    if (not enable_.is_valid())
      enable_ = hal::gpio::nullpin;
  }

  constexpr hal::Error init(hal::Hal &hal, const Settings &settings) {
    hal::Error error = hal.configure(settings.i2c);
    if (error != hal::Error::none)
      return error;

    error = hal.configure(settings.enable);
    if (error != hal::Error::none)
      return error;

    i2c_ = hal.create(settings.i2c.id, 0x74u);
    enable_ = hal.create(settings.enable.port, settings.enable.pins);
    if (not i2c_.is_valid())
      return hal::Error::invalid_handle;
    if (not enable_.is_valid())
      return hal::Error::invalid_handle;

    enable();
    error = write_memory_map(settings.device_settings);
    disable();
    return error;
  }

  constexpr void init(hal::i2c::Device i2c, hal::gpio::Pin enable) {
    i2c_ = std::move(i2c);
    i2c_.set_address(0x74u);

    if (enable.is_valid())
      enable_ = std::move(enable);
    else
      enable_ = hal::gpio::nullpin;

    memory_map = default_memory_map;
  }

  /**
   * @brief take the converter out of its reset state.
   *
   * @note This does not enable the output.
   */
  constexpr hal::Error enable() {
    if (is_enabled())
      return hal::Error::none;
    enable_.set(hal::gpio::State::set);
    return write_memory_map(memory_map);
  }

  /**
   * @brief put converter in a reset state.
   */
  constexpr void disable() { enable_.set(hal::gpio::State::reset); }

  /**
   * @brief returns true if the converter is enabled
   */
  constexpr bool is_enabled() { return enable_.get() == hal::gpio::State::set; }

  /**
   * @brief read out the memory map of the converter.
   *
   * @param map pointer to the memory map to store the results in. If map is
   * nullptr, the results are only stored in the drivers internal memory map.
   */
  constexpr hal::Error read_memory_map(MemoryMap *map) {
    std::uint8_t buf[8]{0};
    auto error = i2c_.write({buf, 0});
    if (error != hal::Error::none)
      return error;
    error = i2c_.read({buf, 8});
    if (error != hal::Error::none)
      return error;
    memory_map.ref = buf[0] | (static_cast<uint16_t>(buf[1]) << 8);
    memory_map.iout_limit = buf[2];
    memory_map.vout_sr = buf[3];
    memory_map.vout_fs = buf[4];
    memory_map.cdc = buf[5];
    memory_map.mode = buf[6];
    memory_map.status = buf[7];
    *map = memory_map;
    return hal::Error::none;
  }

  /**
   * @brief writes map to the converter. Useful for quickly setting up a
   * converter.
   *
   * @param map the memory map to write
   */
  constexpr hal::Error write_memory_map(const MemoryMap &map) {
    const uint8_t buf[8]{
        0,
        static_cast<uint8_t>(map.ref),
        static_cast<uint8_t>(map.ref >> 8),
        map.iout_limit,
        map.vout_sr,
        map.vout_fs,
        static_cast<uint8_t>(map.cdc & ~OCP_MASK), // see section 7.6.5
        map.mode};

    hal::Error error = i2c_.write(buf);
    if (error != hal::Error::none)
      return error;

    if (map.cdc & OCP_MASK) {
      error = write_reg(Reg::CDC, map.cdc);
      if (error != hal::Error::none)
        return error;
    }

    if (map.mode & I2CADD) {
      i2c_.set_address(0x75u);
    } else {
      i2c_.set_address(0x74u);
    }
    memory_map = map;
    return hal::Error::none;
  }

  /**
   * @brief sets the internal reference voltage of the TPS55288 by setting the
   * registers REF_LSB and REF_MSB.
   *
   * One LSB stands for 1.129 mV of the internal reference
   * voltage. The default value is 00000000 11010010b (=282 mV). When
   * the register value is 00000000 00000000b, the reference voltage is 45 mV.
   * When the register value is 00000011 11111111b, the reference voltage
   * is 1.200 V. The output voltage of the TPS55288 also depends on the output
   * feedback ratio (see feedback_ratio) or an external resistor divider (see
   * external_feedback).
   *
   * @param code a 10 bit value
   */
  constexpr hal::Error vref(uint16_t code) { return write_reg(Reg::REF, code); }

  ///  Enable or disable current limit
  constexpr hal::Error enable_ilim(bool enable) {
    bool enabled = (memory_map.iout_limit & Current_Limit_EN) != 0;

    if (enabled == enable)
      return hal::Error::none;

    if (enable) {
      bool ocp_enabled = false;
      hal::Error error = hal::Error::none;
      if (memory_map.cdc & OCP_MASK) {
        ocp_enabled = true;
        error = disable_errors(Errors::overcurrent);
        if (error != hal::Error::none)
          return error;
      }

      error = write_reg(Reg::IOUT_LIMIT,
                        (memory_map.iout_limit | Current_Limit_EN));

      if (error != hal::Error::none)
        return error;

      if (ocp_enabled)
        return enable_errors(Errors::overcurrent);
      else
        return hal::Error::none;
    } else {
      return write_reg(Reg::IOUT_LIMIT,
                       (memory_map.iout_limit & ~Current_Limit_EN));
    }
  }

  /**
   * @brief Sets the current limit target voltage between the ISP pin and the
   * ISN pin.
   *
   * One LSB stands for 0.5 mV. The default value is 0b11100100 (=50 mV).
   * 0b1111111, the maximum code, stands for 63.5 mV
   *
   * @param code a 7 bit value
   */
  constexpr hal::Error ilim(uint8_t code) {
    return write_reg(Reg::IOUT_LIMIT,
                     (memory_map.iout_limit & ~Current_Limit_Setting) |
                         (code & Current_Limit_Setting));
  }

  /**
   * @brief Sets the response time of the device when the output overcurrent
   * limit is reached. Can be 128 microseconds, 3 millieconds, 6 milliseconds,
   * or 12 milliseconds.
   *
   * @param delay the delay
   */
  constexpr hal::Error ocp_delay(OcpDelay delay) {
    return write_reg(Reg::VOUT_SR,
                     (memory_map.vout_sr & ~OCP_DELAY) |
                         (static_cast<uint8_t>(delay) << OCP_DELAY_POS));
  }

  /**
   * @brief Sets the slew rate for output voltage change.
   *
   * The slew rate can be one of:
   * - 1.25mV/us (slew_1_25mV)
   * - 2.5mV/us  (slew_2_5mV)
   * - 5mV/us    (slew_5mV)
   * - 10mV/us   (slew_10mV)
   *
   * @param slew_rate the slew rate
   */
  constexpr hal::Error slew_rate(SlewRate slew_rate) {
    return write_reg(Reg::VOUT_SR, (memory_map.vout_sr & ~SR) |
                                       (static_cast<uint8_t>(slew_rate) & SR));
  }

  /**
   * @brief enable or disable external voltage feedback.
   *
   * If enabled, the FB/INT pin is the feedback input of the output voltage.
   *
   * Else the FB/INT pin is the indicator for output short circuit protection,
   * overcurrent status, and overvoltage status (Default).
   *
   * Note: the external feedback divider ratio, together with vref, sets the
   * final output voltage of the converter, given that external feedback is
   * used.
   *
   * @param enable if true, enables external voltage feedback, else disables it
   */
  constexpr hal::Error external_feedback(bool enable) {
    if (enable)
      return write_reg(Reg::VOUT_FS, (memory_map.vout_fs | FB));
    else
      return write_reg(Reg::VOUT_FS, (memory_map.vout_fs & ~FB));
  }

  /**
   * @brief Sets the internal feedback ratio. Together with vref, this will set
   * the output voltage, given that internal feedback is used.
   *
   * The ratio can be one of:
   * - 0.2256 (fb_0_2256)
   * - 0.1128 (fb_0_1128)
   * - 0.0752 (fb_0_0752)
   * - 0.0564 (fb_0_0564)
   *
   * @param ratio the feedback ratio
   * @return
   */
  constexpr hal::Error feedback_ratio(FeedbackRatio ratio) {
    return write_reg(Reg::VOUT_FS, (memory_map.vout_fs & ~INTFB) |
                                       (static_cast<uint8_t>(ratio) & INTFB));
  }

  /**
   * @brief Enable error indication for the errors specified.
   *
   * If an error is enabled and it occurs, its corresponding StatusFlags bit
   * will be set in its status, as well as the FB/INT pin asserting if internal
   * feedback is used.
   *
   * If an error is disabled, the corresponding StatusFlags bit will not be set
   * and de FB/INT pin will not assert.
   *
   * @param errors the errors to enable
   */
  constexpr hal::Error enable_errors(Errors errors) {
    return write_reg(Reg::CDC,
                     memory_map.cdc | (static_cast<uint8_t>(errors) << 5));
  }

  /**
   * @brief Disable error indication for the errors specified.
   *
   * If an error is enabled and it occurs, its corresponding StatusFlags bit
   * will be set in its status, as well as the FB/INT pin asserting if internal
   * feedback is used.
   *
   * If an error is disabled, the corresponding StatusFlags bit will not be set
   * and de FB/INT pin will not assert.
   *
   * @param errors the errors to enable
   */
  constexpr hal::Error disable_errors(Errors errors) {
    return write_reg(Reg::CDC,
                     memory_map.cdc & ~(static_cast<uint8_t>(errors) << 5));
  }

  /**
   * @brief Select the cable voltage drop compensation approach.
   *
   * If enable is true, external CDC compensation by a resistor at the CDC pin
   * is used. Else internal cable drop compensation is used (see
   * cable_drop_compensation_voltage).
   *
   * @param enable if true, use external compensation, else internal
   */
  constexpr hal::Error external_cable_drop_compensation(bool enable) {
    if (enable)
      return write_reg(Reg::CDC, (memory_map.cdc | CDC_OPTION));
    else
      return write_reg(Reg::CDC, (memory_map.cdc & ~CDC_OPTION));
  }

  /**
   * @brief Set the cable drop compensation voltage.
   *
   * code specifies the voltage rise for every 50mV between the ISP and ISN
   * pins. One LSB stands for 100mV.
   *
   * Examples:
   *
   * - code = 0: = 0 mV output voltage rise with 50 mV at VISP - VISN (Default)
   * -> no compensation
   * - code = 1: = 100 mV output voltage rise with 50 mV at VISP - VISN
   * - code = 7 (max): = 100 mV output voltage rise with 50 mV at VISP - VISN
   *
   * @param code a 3 bit number
   */
  constexpr hal::Error cable_drop_compensation_voltage(uint8_t code) {
    return write_reg(Reg::CDC, (memory_map.cdc & ~CDC) |
                                   (static_cast<uint8_t>(code) & CDC));
  }

  /**
   * @brief enable or disable the output of the converter.
   *
   * If enable is true, enables the converter output. Else it disables it.
   * The default is false.
   *
   * @param enable if true, enables the output, else disables it.
   */
  constexpr hal::Error output_enable(bool enable) {
    bool enabled = (memory_map.mode & OE) != 0;
    if (enabled == enable)
      return hal::Error::none;

    if (enable) {
      bool ocp_enabled = false;
      hal::Error error = hal::Error::none;
      if (memory_map.cdc & OCP_MASK) {
        ocp_enabled = true;
        error = disable_errors(Errors::overcurrent);
        if (error != hal::Error::none)
          return error;
      }

      error = write_reg(Reg::MODE, (memory_map.mode | OE));

      if (error != hal::Error::none)
        return error;

      if (ocp_enabled)
        return enable_errors(Errors::overcurrent);
      else
        return hal::Error::none;
    } else {
      return write_reg(Reg::MODE, (memory_map.mode & ~OE));
    }
  }

  /**
   * @brief Enable or disable switching frequency doubling in buck-boost mode.
   *
   * TI does not recommend using double frequency function at switching
   * frequency above 1.6 MHz. The default is false.
   *
   * @param enable if true, double the switching frequency during buck-boost
   * mode, else keep it unchanged.
   */
  constexpr hal::Error fsw_double(bool enable) {
    if (enable)
      return write_reg(Reg::MODE, (memory_map.mode | FSWDBL));
    else
      return write_reg(Reg::MODE, (memory_map.mode & ~FSWDBL));
  }

  /**
   * @brief Enable or disable hiccup mode. The default is true.
   *
   * @param enable if enabled, the converter will hiccup during short circuit
   * protection.
   */
  constexpr hal::Error hiccup(bool enable) {
    if (enable)
      return write_reg(Reg::MODE, (memory_map.mode | HICCUP));
    else
      return write_reg(Reg::MODE, (memory_map.mode & ~HICCUP));
  }

  /**
   * @brief Enable or disable output discharge during shutdown.
   *
   * If enabled, VOUT is discharged to ground by an internal 100-mA current
   * sink.
   *
   * The default is false.
   *
   * @param enable if true, enables output discharge during shutdown.
   */
  constexpr hal::Error output_discharge(bool enable) {
    if (enable)
      return write_reg(Reg::MODE, (memory_map.mode | DISCHG));
    else
      return write_reg(Reg::MODE, (memory_map.mode & ~DISCHG));
  }

  /**
   * @brief Select the internal LDO (false) or an external 5 V power supply
   * (true) as the power source for VCC.
   *
   * @param enable if true, the converter uses an external supply, else it uses
   * the internal LDO.
   */
  constexpr hal::Error external_vcc(bool enable) {
    if (enable)
      return write_reg(Reg::MODE, (memory_map.mode | VCC));
    else
      return write_reg(Reg::MODE, (memory_map.mode & ~VCC));
  }

  /**
   * @brief Set the I2C address to 0x74 or 0x75.
   *
   * @param addr the address
   */
  constexpr hal::Error i2c_address(Address addr) {
    switch (addr) {
    case Address::address_0x74:
      if (auto err = write_reg(Reg::MODE, memory_map.mode & ~I2CADD);
          err != hal::Error::none)
        return err;
      i2c_.set_address(0x74u);
      return hal::Error::none;
    case Address::address_0x75:
      if (auto err = write_reg(Reg::MODE, memory_map.mode | I2CADD);
          err != hal::Error::none)
        return err;
      i2c_.set_address(0x75u);
      return hal::Error::none;
    default:
      return hal::Error::invalid_param;
    }
  }

  /**
   * @brief Select operating mode at light load condition.
   *
   * Can be one of:
   * - PFM: PFM operating mode at light load condition (Default)
   * - FPWM: FPWM operating mode at light load condition
   *
   * @param mode the mode
   */
  constexpr hal::Error light_load_mode(LightLoadMode mode) {
    switch (mode) {
    case LightLoadMode::PFM:
      return write_reg(Reg::MODE, memory_map.mode & ~PFM);
    case LightLoadMode::FPWM:
      return write_reg(Reg::MODE, memory_map.mode | PFM);
    default:
      return hal::Error::invalid_param;
    }
  }

  /**
   * @brief Set the mode control approach.
   *
   * Can be one of:
   * - external_resistor: Set VCC, I2CADD, and PFM controlled by external
   * resistor (Default).
   * - internal_register: Set VCC, I2CADD, and PFM controlled by internal
   * register.
   *
   * @param ctrl the control approach
   */
  constexpr hal::Error internal_mode_control(ModeControl ctrl) {
    switch (ctrl) {
    case ModeControl::external_resistor:
      return write_reg(Reg::MODE, memory_map.mode & ~MODE);
    case ModeControl::internal_register:
      return write_reg(Reg::MODE, memory_map.mode & ~MODE);
    default:
      return hal::Error::invalid_param;
    }
  }

  constexpr StatusCode status() {
    uint16_t value = 0;
    hal::Error error = read_reg(Reg::STATUS, value);
    if (error != hal::Error::none)
      return StatusCode::invalid;
    return static_cast<StatusCode>(static_cast<uint8_t>(value));
  }

private:
  enum class Reg : std::uint8_t {
    REF = 0x0,
    IOUT_LIMIT = 0x2,
    VOUT_SR = 0x3,
    VOUT_FS = 0x4,
    CDC = 0x5,
    MODE = 0x6,
    STATUS = 0x7
  };

  enum Ref : std::uint16_t {
    VREF = 0b1111111111u,
    VREF_RESET_VALUE = 0b11010010u
  };

  enum IoutLImit : std::uint8_t {
    Current_Limit_EN = 1 << 7,
    Current_Limit_Setting = 0b1111111u,
    IOUT_LIMIT_RESET_VALUE = 0b11100100u
  };

  enum VoutSr : std::uint8_t {
    OCP_DELAY = 0b11 << 4,
    OCP_DELAY_POS = 4,
    SR = 0b11,
    SR_POS = 0,
    VOUT_SR_RESET_MASK = 0b110011u,
    VOUT_SR_RESET_VALUE = 0b00000001u
  };

  enum VoutFs : std::uint8_t {
    FB = 1 << 7,
    INTFB = 0b11,
    INTFB_POS = 0,
    VOUT_FS_RESET_MASK = 0b10000011u,
    VOUT_FS_RESET_VALUE = 0b00000011u
  };

  enum Cdc : std::uint8_t {
    SC_MASK = 1 << 7,
    OCP_MASK = 1 << 6,
    OVP_MASK = 1 << 5,
    CDC_OPTION = 1 << 3,
    CDC = 0b1111u,
    CDC_POS = 0,
    CDC_RESET_MASK = 0b11101111u,
    CDC_RESET_VALUE = 0b11100000u
  };

  enum Mode : std::uint8_t {
    OE = 1 << 7,
    FSWDBL = 1 << 6,
    HICCUP = 1 << 5,
    DISCHG = 1 << 4,
    VCC = 1 << 3,
    I2CADD = 1 << 2,
    PFM = 1 << 1,
    MODE = 1 << 0,
    MODE_RESET_VALUE = 0b00100000u
  };

  enum Status : std::uint8_t {
    SCP = 1 << 7,
    OCP = 1 << 6,
    OVP = 1 << 5,
    STATUS = 0b11,
    STATUS_POS = 0,
    STATUS_RESET_MASK = 0b11100011u,
    STATUS_RESET_VALUE = 0b00000011u
  };

  constexpr hal::Error read_reg(Reg reg, uint16_t &value) {
    uint8_t buf[2]{static_cast<uint8_t>(reg), 0};
    std::size_t read_size = 1;

    auto do_read_reg = [&](auto &member, uint16_t mask) {
      // transmit the register value
      hal::Error error = i2c_.write({buf, 1});
      if (error != hal::Error::none)
        return error;

      buf[0] = 0;
      error = i2c_.read({buf, read_size});
      if (error != hal::Error::none)
        return error;
      value = buf[0] | (static_cast<uint16_t>(buf[1]) << 8);
      value &= mask;
      member = value;
      return hal::Error::none;
    };

    switch (reg) {
    case Reg::REF:
      read_size = 2;
      return do_read_reg(memory_map.ref, VREF);
    case Reg::IOUT_LIMIT:
      return do_read_reg(memory_map.iout_limit, 0xFFu);
    case Reg::VOUT_SR:
      return do_read_reg(memory_map.vout_sr, VOUT_SR_RESET_MASK);
    case Reg::VOUT_FS:
      return do_read_reg(memory_map.vout_fs, VOUT_FS_RESET_MASK);
    case Reg::CDC:
      return do_read_reg(memory_map.cdc, CDC_RESET_MASK);
    case Reg::MODE:
      return do_read_reg(memory_map.mode, 0xFFu);
    case Reg::STATUS:
      return do_read_reg(memory_map.status, STATUS_RESET_MASK);
    default:
      return hal::Error::invalid_param;
    }
  }

  constexpr hal::Error write_reg(Reg reg, uint16_t value) {
    std::size_t write_size = 2;
    switch (reg) {
    case Reg::REF:
      value &= VREF;
      memory_map.ref = value;
      write_size = 3;
      break;
    case Reg::IOUT_LIMIT:
      memory_map.iout_limit = value;
      break;
    case Reg::VOUT_SR:
      value &= VOUT_SR_RESET_MASK;
      memory_map.vout_sr = value;
      break;
    case Reg::VOUT_FS:
      value &= VOUT_FS_RESET_MASK;
      memory_map.vout_fs = value;
      break;
    case Reg::CDC:
      value &= CDC_RESET_MASK;
      memory_map.cdc = value;
      break;
    case Reg::MODE:
      memory_map.mode = value;
      break;
    case Reg::STATUS:
      value &= STATUS_RESET_MASK;
      memory_map.status = value;
      break;
    default:
      return hal::Error::invalid_param;
    }

    const uint8_t buf[3]{static_cast<uint8_t>(reg), static_cast<uint8_t>(value),
                         static_cast<uint8_t>(value >> 8)};

    return i2c_.write({buf, write_size});
  }

  hal::i2c::Device i2c_{};
  hal::gpio::Pin enable_{hal::gpio::nullpin};
  MemoryMap memory_map{};
};

} // namespace tps55288

#endif
