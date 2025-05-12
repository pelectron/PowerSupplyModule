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

#include <cstdint>

namespace tps55288 {

namespace {
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

} // namespace

/// Over Current Protection Delay
enum class OcpDelay : std::uint8_t {
  delay_128us = 0, //< 128 microseconds (Default)
  delay_3ms = 1,   //< 3 * 1.024 milliseconds
  delay_6ms = 2,   //< 6 * 1.024 milliseconds
  delay_12ms = 3   //< 12 * 1.024 milliseconds
};

/// Output Slew Rate
enum class SlewRate : std::uint8_t {
  slew_1_25mV = 0, //< 1.25 mV/us
  slew_2_5mV = 1,  //< 2.5 mV/us (Default)
  slew_5mV = 2,    //< 5 mV/us
  slew_10mV = 3,   //< 10 mV/us
};

/// Internal Feedback Divider Ratio
enum class FeedbackRatio : std::uint8_t {
  fb_0_2256 = 0, //< 0.2256
  fb_0_1128 = 1, //< 0.1128
  fb_0_0752 = 2, //< 0.0752
  fb_0_0564 = 3  //< 0.0564 (Default)
};

/**
 * When an indicator for an event is enabled, then its activation, for example
 * short circuit, will set the corresponding bit in the status and assert the
 * FB/INT pin when using internal feedback. When the indicator is disabled, then
 * the corresponding bit will not be set and the FB/INT pin will not be
 * asserted, even if its event is active.
 */
enum class Indicator : std::uint8_t {
  short_circuit = 1 << 2,
  overcurrent = 1 << 1,
  overvoltage = 1 << 0
};

constexpr Indicator operator|(Indicator f1, Indicator f2) noexcept {
  return static_cast<Indicator>(static_cast<std::uint8_t>(f1) |
                                static_cast<std::uint8_t>(f2));
}

constexpr Indicator operator&(Indicator f1, Indicator f2) noexcept {
  return static_cast<Indicator>(static_cast<std::uint8_t>(f1) &
                                static_cast<std::uint8_t>(f2));
}

/// I2CADD bit
enum class Address : std::uint8_t {
  address_0x74,
  address_0x75,
};

/**
 * In light load condition, the TPS55288 can work in PFM or forced PWM mode to
 * meet different application requirements. The PFM mode decreases switching
 * frequency to reduce the switching loss thus it gets high efficiency at light
 * load condition. The FPWM mode keeps the switching frequency unchanged to
 * avoid undesired low switching frequency but the efficiency becomes lower than
 * that of PFM mode.
 */
enum class LightLoadMode : std::uint8_t { PFM = 0, FPWM = 1 };

/// how the VCC, I2CADD and PFM bits of the mode register are set
enum class ModeControl : std::uint8_t { external_resistor, internal_register };

/// the operation mode of the converter
enum class OperatingMode : std::uint8_t { boost = 0, buck = 1, buck_boost = 2 };

/// error flags of the status register.
enum class StatusFlags : std::uint8_t {
  short_circuit = 1 << 2,
  overcurrent = 1 << 1,
  overvoltage = 1 << 0
};

constexpr StatusFlags operator|(StatusFlags f1, StatusFlags f2) noexcept {
  return static_cast<StatusFlags>(static_cast<std::uint8_t>(f1) |
                                  static_cast<std::uint8_t>(f2));
}

constexpr StatusFlags operator&(StatusFlags f1, StatusFlags f2) noexcept {
  return static_cast<StatusFlags>(static_cast<std::uint8_t>(f1) |
                                  static_cast<std::uint8_t>(f2));
}

/**
 * opaque enum for the status register. Use get_flags and get_mode to get the
 * StatusFlags and OperatingMode in the status. StatusCode::invalid is returned
 * in case an error occurred when reading the status register.
 */
enum class StatusCode : std::uint8_t { invalid = 0b11100u };

/**
 * @brief returns the StatusFlags of status
 *
 * @param status the status code
 */
constexpr StatusFlags get_flags(StatusCode status) noexcept {
  return static_cast<StatusFlags>(static_cast<std::uint8_t>(status) >> 5);
}

/**
 * @brief returns the OperatingMode of status
 *
 * @param status the status code
 */
constexpr OperatingMode get_mode(StatusCode status) noexcept {
  return static_cast<OperatingMode>(static_cast<std::uint8_t>(status) & 0b11u);
}

/**
 * @brief The memory/register map of the TPS55288. A default constructed
 * instance will hold the register reset values.
 *
 * The struct only contains the raw register values. To set and access
 * individual fields, use the member functions.
 *
 * For a detailed description of the fields, see the TPS55288 driver class and
 * datasheet.
 */
struct RegisterMap {
  std::uint16_t ref = 0b11010010u;
  std::uint8_t iout_limit = 0b11100100u;
  std::uint8_t vout_sr = 0b00000001u;
  std::uint8_t vout_fs = 0b00000011u;
  std::uint8_t cdc = 0b11100000u;
  std::uint8_t mode = 0b00100000u;
  std::uint8_t status = 0b00000011;

  constexpr auto operator<=>(const RegisterMap &) const = default;

  constexpr void vref(std::uint16_t value) noexcept { ref = value & VREF; }

  constexpr void ilim_enable(bool enable) noexcept {
    if (enable)
      iout_limit |= Current_Limit_EN;
    else
      iout_limit &= ~Current_Limit_EN;
  }

  constexpr void ilim(std::uint8_t code) noexcept {
    iout_limit =
        (iout_limit & ~Current_Limit_Setting) | (code & Current_Limit_Setting);
  }

  constexpr void ocp_delay(OcpDelay delay) noexcept {
    vout_sr = (vout_sr & ~OCP_DELAY) | (static_cast<std::uint8_t>(delay) << 5u);
  }

  constexpr void slew_rate(SlewRate rate) noexcept {
    vout_sr = (vout_sr & ~SR) | (static_cast<std::uint8_t>(rate) & SR);
  }

  constexpr void external_feedback(bool enable) noexcept {
    if (enable)
      vout_fs |= FB;
    else
      vout_fs &= ~FB;
  }

  constexpr void feedback_ratio(FeedbackRatio ratio) noexcept {
    vout_fs = (vout_fs & ~INTFB) | (static_cast<std::uint8_t>(ratio) & INTFB);
  }

  constexpr void enable_indicators(Indicator indicators) noexcept {
    cdc = (cdc & ~(SC_MASK | OVP_MASK | OCP_MASK)) |
          (static_cast<std::uint8_t>(indicators) << 5);
  }

  constexpr void disable_indicators(Indicator indicators) noexcept {
    cdc &= ~(static_cast<std::uint8_t>(indicators) << 5);
  }

  constexpr void external_cable_drop_compensation(bool enable) noexcept {
    if (enable)
      cdc |= CDC_OPTION;
    else
      cdc &= ~CDC_OPTION;
  }

  constexpr void cable_drop_compensation_voltage(uint8_t code) noexcept {
    cdc = (cdc & ~CDC) | (code & CDC);
  }

  constexpr void output_enable(bool enable) noexcept {
    if (enable)
      mode |= OE;
    else
      mode &= ~OE;
  }

  constexpr void fsw_double(bool enable) noexcept {
    if (enable)
      mode |= FSWDBL;
    else
      mode &= ~FSWDBL;
  }

  constexpr void hiccup(bool enable) noexcept {
    if (enable)
      mode |= HICCUP;
    else
      mode &= ~HICCUP;
  }

  constexpr void output_discharge(bool enable) noexcept {
    if (enable)
      mode |= DISCHG;
    else
      mode &= ~DISCHG;
  }

  constexpr void external_vcc(bool enable) noexcept {
    if (enable)
      mode |= VCC;
    else
      mode &= ~VCC;
  }

  constexpr void i2c_address(Address addr) noexcept {
    switch (addr) {
    case Address::address_0x74:
      mode &= ~I2CADD;
      break;
    case Address::address_0x75:
      mode |= I2CADD;
      break;
    }
  }

  constexpr void light_load_mode(LightLoadMode ll_mode) noexcept {
    switch (ll_mode) {
    case LightLoadMode::PFM:
      mode &= ~PFM;
      break;
    case LightLoadMode::FPWM:
      mode |= PFM;
      break;
    }
  }

  constexpr void mode_control(ModeControl ctrl) noexcept {
    switch (ctrl) {
    case ModeControl::external_resistor:
      mode &= ~MODE;
      break;
    case ModeControl::internal_register:
      mode |= MODE;
      break;
    }
  }

  constexpr uint16_t vref() const noexcept { return ref; }

  constexpr bool ilim_enabled() const noexcept {
    return (iout_limit & Current_Limit_EN) != 0;
  }

  constexpr uint8_t ilim() const noexcept {
    return iout_limit & Current_Limit_Setting;
  }

  constexpr OcpDelay ocp_delay() const noexcept {
    return static_cast<OcpDelay>(vout_sr >> OCP_DELAY_POS);
  }

  constexpr SlewRate slew_rate() const noexcept {
    return static_cast<SlewRate>(vout_sr & SR);
  }

  constexpr bool external_feedback() const noexcept {
    return (vout_fs & FB) != 0;
  }

  constexpr FeedbackRatio feedback_ratio() const noexcept {
    return static_cast<FeedbackRatio>(vout_fs & INTFB);
  }

  constexpr Indicator enabled_indicators() const noexcept {
    return static_cast<Indicator>(cdc >> 5);
  }

  constexpr bool external_cable_drop_compensation() const noexcept {
    return (cdc & CDC_OPTION) != 0;
  }

  constexpr uint8_t cable_drop_compensation_voltage() const noexcept {
    return cdc & CDC;
  }

  constexpr bool output_enabled() const noexcept { return (mode & OE) != 0; }

  constexpr bool fsw_double() const noexcept { return (mode & FSWDBL) != 0; }

  constexpr bool hiccup() const noexcept { return (mode & HICCUP) != 0; }

  constexpr bool output_discharge() const noexcept {
    return (mode & DISCHG) != 0;
  }

  constexpr bool external_vcc() const noexcept { return (mode & VCC) != 0; }

  constexpr Address i2c_address() const noexcept {
    return (mode & VCC) == 0 ? Address::address_0x74 : Address::address_0x75;
  }

  constexpr LightLoadMode light_load_mode() const noexcept {
    return (mode & PFM) == 0 ? LightLoadMode::PFM : LightLoadMode::FPWM;
  }

  constexpr ModeControl mode_control() const noexcept {
    return (mode & MODE) == 0 ? ModeControl::external_resistor
                              : ModeControl::internal_register;
  }

  constexpr StatusCode status_code() const noexcept {
    return static_cast<StatusCode>(status);
  }
};

struct Settings {
  hal::i2c::Config i2c;
  hal::gpio::Config enable;
  RegisterMap device_settings;
  constexpr auto operator<=>(const Settings &) const = default;
};

static_assert(sizeof(RegisterMap) == 8);

inline constexpr RegisterMap default_memory_map{};

/**
 * @class TPS55288
 * @brief This is a driver for the TPS55288 buck boost converter IC from TI.
 *
 * It uses a hal:i2c::Device for communication and an optional hal::gpio::Pin.
 * Additionally, it uses a cached register map to reduce io operations.
 * All operations that can fail because of io return a hal::Error
 * value. The only exception is status(),which returns StatusCode::invalid in
 * case of an io error.
 */
class TPS55288 {
public:
  /**
   * @brief This will construct the driver with an invalid i2c device. One of
   * the init() overloads must be called before using the driver.
   */
  constexpr TPS55288() = default;

  /**
   * @brief constructs a new driver. i2c must be a valid device, else one of the
   * init() overloads must be called.
   *
   * @param i2c the I2C device used for communication
   * @param enable the gpio controlling the enable pin. Can be invalid.
   */
  constexpr TPS55288(hal::i2c::Device i2c, hal::gpio::Pin enable)
      : i2c_(std::move(i2c)), enable_(std::move(enable)) {
    i2c_.set_address(0x74u);
    if (not enable_.is_valid())
      enable_ = hal::gpio::nullpin;
  }

  /**
   * @brief initialize an invalid driver. If hal::Error::none is returned, the
   * driver can be used.
   *
   * @param hal the hal object
   * @param settings TPS55288 settings
   */
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
      enable_ = hal::gpio::nullpin;

    return error;
  }

  /**
   * @brief initialize an invalid driver. If hal::Error::none is returned, the
   * driver can be used.
   *
   * @param i2c the I2C device used for communication
   * @param enable the gpio controlling the enable pin. Can be invalid.
   */
  constexpr hal::Error init(hal::i2c::Device i2c, hal::gpio::Pin enable) {
    i2c_ = std::move(i2c);

    if (not i2c_.is_valid())
      return hal::Error::invalid_handle;

    i2c_.set_address(0x74u);

    enable_ = std::move(enable);

    if (not enable_.is_valid())
      enable_ = hal::gpio::nullpin;

    register_map_ = default_memory_map;

    return hal::Error::none;
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
    return write_register_map(register_map_);
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
  constexpr hal::Error read_register_map(RegisterMap *map) {
    std::uint8_t buf[8]{0};
    auto error = i2c_.write({buf, 0});
    if (error != hal::Error::none)
      return error;
    error = i2c_.read({buf, 8});
    if (error != hal::Error::none)
      return error;
    register_map_.ref = buf[0] | (static_cast<uint16_t>(buf[1]) << 8);
    register_map_.iout_limit = buf[2];
    register_map_.vout_sr = buf[3];
    register_map_.vout_fs = buf[4];
    register_map_.cdc = buf[5];
    register_map_.mode = buf[6];
    register_map_.status = buf[7];
    *map = register_map_;
    return hal::Error::none;
  }

  /**
   * @brief writes map to the converter. Useful for quickly setting up a
   * converter.
   *
   * @param map the register map to write
   */
  constexpr hal::Error write_register_map(const RegisterMap &map) {
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

    return hal::Error::none;
  }

  /// @brief returns the cached register map
  constexpr RegisterMap register_map() const { return register_map_; }

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
    bool enabled = register_map_.ilim_enabled();

    if (enabled == enable)
      return hal::Error::none;

    if (enable) {
      bool ocp_enabled = false;
      hal::Error error = hal::Error::none;
      if ((register_map_.enabled_indicators() & Indicator::overcurrent) ==
          Indicator::overcurrent) {
        ocp_enabled = true;
        error = disable_indicators(Indicator::overcurrent);
        if (error != hal::Error::none)
          return error;
      }

      error = write_reg(Reg::IOUT_LIMIT, register_map_.iout_limit);

      if (error != hal::Error::none)
        return error;

      if (ocp_enabled)
        return enable_indicators(Indicator::overcurrent);
      else
        return hal::Error::none;
    } else {
      return write_reg(Reg::IOUT_LIMIT, register_map_.iout_limit);
    }
  }

  /**
   * @brief Sets the current limit target voltage between the ISP pin and the
   * ISN pin.
   *
   * One LSB stands for 0.5 mV. The default value is 0b11100100 (=50 mV).
   * 0b1111111, the maximum code, stands for 63.5 mV.
   *
   * @param code a 7 bit value
   */
  constexpr hal::Error ilim(uint8_t code) {
    register_map_.ilim(code);
    return write_reg(Reg::IOUT_LIMIT, register_map_.iout_limit);
  }

  /**
   * @brief Sets the response time of the device when the output overcurrent
   * limit is reached. Can be 128 microseconds, 3 milliseconds, 6 milliseconds,
   * or 12 milliseconds.
   *
   * @param delay the delay
   */
  constexpr hal::Error ocp_delay(OcpDelay delay) {
    register_map_.ocp_delay(delay);
    return write_reg(Reg::VOUT_SR, register_map_.vout_sr);
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
  constexpr hal::Error slew_rate(SlewRate rate) {
    register_map_.slew_rate(rate);
    return write_reg(Reg::VOUT_SR, register_map_.vout_sr);
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
    register_map_.external_feedback(enable);
    return write_reg(Reg::VOUT_FS, register_map_.vout_fs);
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
    register_map_.feedback_ratio(ratio);
    return write_reg(Reg::VOUT_FS, register_map_.vout_fs);
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
  constexpr hal::Error enable_indicators(Indicator indicators) {
    register_map_.enable_indicators(indicators);
    return write_reg(Reg::CDC, register_map_.cdc);
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
  constexpr hal::Error disable_indicators(Indicator indicators) {
    register_map_.disable_indicators(indicators);
    return write_reg(Reg::CDC, register_map_.cdc);
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
    register_map_.external_cable_drop_compensation(enable);
    return write_reg(Reg::CDC, register_map_.cdc);
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
    register_map_.cable_drop_compensation_voltage(code);
    return write_reg(Reg::CDC, register_map_.cdc);
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
    bool enabled = register_map_.output_enabled();
    if (enabled == enable)
      return hal::Error::none;

    register_map_.output_enable(enable);

    if (enable) {
      bool ocp_enabled = false;
      hal::Error error = hal::Error::none;
      if ((register_map_.enabled_indicators() & Indicator::overcurrent) ==
          Indicator::overcurrent) {
        ocp_enabled = true;
        error = disable_indicators(Indicator::overcurrent);
        if (error != hal::Error::none)
          return error;
      }

      error = write_reg(Reg::MODE, register_map_.mode);

      if (error != hal::Error::none)
        return error;

      if (ocp_enabled)
        return enable_indicators(Indicator::overcurrent);
      else
        return hal::Error::none;
    } else {
      return write_reg(Reg::MODE, register_map_.mode);
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
    register_map_.fsw_double(enable);
    return write_reg(Reg::MODE, register_map_.mode);
  }

  /**
   * @brief Enable or disable hiccup mode. The default is true.
   *
   * @param enable if enabled, the converter will hiccup during short circuit
   * protection.
   */
  constexpr hal::Error hiccup(bool enable) {
    register_map_.hiccup(enable);
    return write_reg(Reg::MODE, register_map_.mode);
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
    register_map_.output_discharge(enable);
    return write_reg(Reg::MODE, register_map_.mode);
  }

  /**
   * @brief Select the internal LDO (false) or an external 5 V power supply
   * (true) as the power source for VCC.
   *
   * @param enable if true, the converter uses an external supply, else it uses
   * the internal LDO.
   */
  constexpr hal::Error external_vcc(bool enable) {
    register_map_.external_vcc(enable);
    return write_reg(Reg::MODE, register_map_.mode);
  }

  /**
   * @brief Set the I2C address to 0x74 or 0x75.
   *
   * @param addr the address
   */
  constexpr hal::Error i2c_address(Address addr) {
    register_map_.i2c_address(addr);
    auto err = write_reg(Reg::MODE, register_map_.mode);
    switch (addr) {
    case Address::address_0x74:
      i2c_.set_address(0x74u);
      break;
    case Address::address_0x75:
      i2c_.set_address(0x75u);
      break;
    default:
      return hal::Error::invalid_param;
    }
    return err;
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
    register_map_.light_load_mode(mode);
    return write_reg(Reg::MODE, register_map_.mode);
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
  constexpr hal::Error mode_control(ModeControl ctrl) {
    register_map_.mode_control(ctrl);
    return write_reg(Reg::MODE, register_map_.mode);
  }

  /**
   * @brief return the StatusCode of the converter, or StatusCode::invalid if an
   * error occurred during communication. The status is made up of StatusFlags
   * and an OperatingMode. Use get_flags(status) and get_mode(status) to access
   * the flags and mode.
   *
   * @return the StatusCode or StatusCode::invalid if an error occurred.
   */
  constexpr StatusCode status() {
    uint16_t value = 0;
    hal::Error error = read_reg(Reg::STATUS, value);
    if (error != hal::Error::none)
      return StatusCode::invalid;
    register_map_.status = value;
    return static_cast<StatusCode>(static_cast<uint8_t>(value));
  }

private:
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
      return do_read_reg(register_map_.ref, VREF);
    case Reg::IOUT_LIMIT:
      return do_read_reg(register_map_.iout_limit, 0xFFu);
    case Reg::VOUT_SR:
      return do_read_reg(register_map_.vout_sr, VOUT_SR_RESET_MASK);
    case Reg::VOUT_FS:
      return do_read_reg(register_map_.vout_fs, VOUT_FS_RESET_MASK);
    case Reg::CDC:
      return do_read_reg(register_map_.cdc, CDC_RESET_MASK);
    case Reg::MODE:
      return do_read_reg(register_map_.mode, 0xFFu);
    case Reg::STATUS:
      return do_read_reg(register_map_.status, STATUS_RESET_MASK);
    default:
      return hal::Error::invalid_param;
    }
  }

  constexpr hal::Error write_reg(Reg reg, uint16_t value) {
    std::size_t write_size = reg == Reg::REF ? 3 : 2;

    const uint8_t buf[3]{static_cast<uint8_t>(reg), static_cast<uint8_t>(value),
                         static_cast<uint8_t>(value >> 8)};

    return i2c_.write({buf, write_size});
  }

  hal::i2c::Device i2c_{};
  hal::gpio::Pin enable_{};
  RegisterMap register_map_{};
};

} // namespace tps55288

#endif
