#ifndef PSM_CONVERTER_HPP
#define PSM_CONVERTER_HPP

#include "drivers/ad5293.hpp"
#include "drivers/mcp45hvx1.hpp"
#include "drivers/tps55288.hpp"
#include "error.hpp"
#include "hal/config.hpp"
#include "hal/hal.hpp"
#include "poly.hpp"
#include "tl/expected.hpp"
#include "units.hpp"

#include <cstdint>

namespace psm {

namespace conv {

POLY_METHOD(enable)
POLY_METHOD(disable)
POLY_METHOD(set_voltage)
POLY_METHOD(get_voltage)
POLY_METHOD(get_voltage_limit)
POLY_METHOD(output_enable)
POLY_METHOD(set_current)
POLY_METHOD(get_current)
POLY_METHOD(get_current_limit)
POLY_METHOD(current_limit_enable)
POLY_METHOD(get_temperature)

POLY_PROPERTY(voltage);
POLY_PROPERTY(current);
POLY_PROPERTY(enabled);
POLY_PROPERTY(temperature);
POLY_PROPERTY(specs);

struct Specs {
  Voltage v_min;
  Voltage v_max;
  Voltage v_granularity;
  Current i_min;
  Current i_max;
  Current i_granularity;
};

template <poly::Storage S>
using Handle = poly::Struct<
    S,
    poly::type_list<voltage(Voltage), current(Current), enabled(bool),
                    const temperature(const Celcius),
                    const specs(const Specs &)>,
    poly::type_list<Error(enable), Error(disable), Error(set_voltage, Voltage),
                    tl::expected<Voltage, Error>(get_voltage),
                    tl::expected<Voltage, Error>(get_voltage_limit),
                    Error(output_enable, bool), Error(set_current, Current),
                    tl::expected<Current, Error>(get_current),
                    tl::expected<Current, Error>(get_current_limit),
                    Error(current_limit_enable, bool),
                    tl::expected<Temperature, Error>(get_temperature)>>;

template <poly::Storage S>
class BasicConverter
    : public poly::Struct<
          S,
          poly::type_list<voltage(Voltage), current(Current), enabled(bool),
                          const temperature(const Celcius),
                          const specs(const Specs &)>,
          poly::type_list<>> {
  Error enable();
  Error disable();
  Error output_enable(bool enable);
  Error set_voltage(Voltage v) {}
  tl::expected<Voltage, Error> get_voltage() {}
  tl::expected<Voltage, Error> get_voltage_limit() {}
  Error set_current(Voltage v) {}
  tl::expected<Current, Error> get_current() {}
  tl::expected<Current, Error> get_current_limit() {}
  Error current_limit_enable(bool enable);

  tl::expected<Temperature, Error> get_temperature();
};

using ConverterReference = BasicConverter<poly::ref_storage>;
} // namespace conv

inline namespace v1 {
/**
 * @class BuckBoostStage
 * @brief The buck boost stage driver for the PSM V1
 *
 */
struct BuckBoostStage {
  tps55288::TPS55288 driver_;
  Resistance r_sense;
  hal::adc::Channel cdc;
  hal::adc::Channel fb;

  struct Settings {
    tps55288::Settings tsp55288;
    Resistance r_sense;
    hal::adc::ChannelConfig cdc;
    hal::adc::ChannelConfig fb;
  };

  Error enable(bool enable) {
    if (enable)
      return from_hal_error(driver_.enable());
    driver_.disable();
    return Error::none;
  }

  /**
   * @brief set the output voltage (limit) of the converter
   *
   * @param v the maximum voltage
   */
  Error set_voltage(Voltage v) {
    // v * feedback_ratio = vref = code*1.129mV + 45mV
    // code = (v * feedback_ratio - 45mV) / 1.129mV
    // feedback ratio = 0.0564
    if (v < 800_mV or v > 22_V) {
      return Error::invalid_param;
    }
    std::uint16_t code = ((v * 0.0564_sf - 45_mV) / 1.129_mV);
    return from_hal_error(driver_.vref(code));
  }

  /**
   * @brief get the actual output voltage. This may be lower than the voltge set
   * with set_voltage if the converter is in constant current / current limit
   * mode.
   */
  tl::expected<Voltage, Error> get_voltage() {
    return fb.sample()
        .and_then([](Voltage v) -> tl::expected<Voltage, hal::Error> {
          // the input to the adc has a 7k/1k divider -> multiply by 8 to get
          // the actual voltage
          return 8 * v;
        })
        .map_error(from_hal_error);
  }

  /**
   * @brief get the output voltage setting, i.e. the voltage set with
   * set_voltage.
   */
  tl::expected<Voltage, Error> get_voltage_limit() {
    // v * feedback_ratio = vref = code*1.129mV + 45mV
    // v = (code*1.129mV + 45mV) / feedback_ratio
    // feedback ratio = 0.0564
    return ((driver_.vref() * 1129_uV + 45_mV) * au::micro(au::unos)(56400))
        .as<std::uint32_t>(au::micro(au::volts));
  }

  /**
   * @brief set the output current (limit) of the converter.
   *
   * @param i the maximum current
   */
  Error set_current(Current i) {
    const Voltage voltage = (r_sense * i).as<uint32_t>(au::micro(au::volts));
    if (voltage > 63.5_mV)
      return Error::out_of_range;

    auto code = voltage / 500_uV;
    if (code > 0x7Fu)
      return Error::out_of_range;

    uint8_t setting = code;

    return from_hal_error(driver_.ilim(setting));
  }

  /**
   * @brief get the actual output current. This may be lower than the current
   * set with set_current if the converter is in constant voltage mode.
   */
  tl::expected<Current, Error> get_current() {
    return cdc.sample()
        .and_then([this](Voltage v) -> tl::expected<Current, hal::Error> {
          // V_cdc = 20* (V_isp - V_isn)
          // current = (V_isp - V_isn)/r_sense = V_cdc/(20 * r_sense)
          return v / au::unblock_int_div(20 * r_sense);
        })
        .map_error(from_hal_error);
  }

  /**
   * @brief get the output current limit
   *
   * @return
   */
  tl::expected<Current, Error> get_current_limit() {
    return (driver_.ilim() * 500_uV) / au::unblock_int_div(r_sense);
  }

  /**
   * @brief enable or disable the current limit
   */
  Error current_limit_enable(bool enable) {
    return from_hal_error(driver_.enable_ilim(enable));
  }

  /**
   * @brief enable or disable the output
   */
  Error output_enable(bool enable) {
    return from_hal_error(driver_.output_enable(enable));
  }

  tl::expected<Temperature, Error> get_temperature() {
    return tl::unexpected(Error::unsupported_operation);
  }

  Error cable_drop_compensation(Resistance r) {
    if (r == au::ZERO)
      return from_hal_error(driver_.cable_drop_compensation_voltage(0));

    if (r > au::milli(au::ohms)(70)) {
      return Error::out_of_range;
    }

    uint8_t code = r.in(au::milli(au::ohms)) / 10;
    uint8_t digit = r.in(au::milli(au::ohms))-10 * code;
    if (digit >= 5)
      ++code;

    return from_hal_error(driver_.cable_drop_compensation_voltage(code));
  }
};

struct LdoStage {
  static constexpr Resistance nominal_ad5293_resistance =
      au::kilo(au::ohms)(20);
  static constexpr Current nominal_current = au::milli(au::amperes)(1);
  static constexpr Resistance nominal_mcp45_resistance = au::kilo(au::ohms)(5);

  static constexpr auto set_current_factor =
      (au::milli(au::amperes) / au::kilo(au::ohms))(350);
  static constexpr auto temp_factor =
      (au::micro(au::amperes) / au::celsius_qty)(1);

  struct Settings {
    Resistance ad5293_resistance = au::kilo(au::ohms)(20);
    Resistance mcp45_resistance = au::kilo(au::ohms)(20);
    Resistance imon_resistance = au::kilo(au::ohms)(10);
    Resistance temp_resistance = au::kilo(au::ohms)(20);
    Current lt3902_current = au::micro(au::amperes)(1010);
    uint8_t num_ldos = 7;
    ad5293::Settings ad5293_settings;
    mcp45hvx1::Settings mcp45_settings;
    hal::adc::ChannelConfig fb;
    hal::adc::ChannelConfig temp;
    hal::adc::ChannelConfig imon;
  };

  ad5293::AD5293 ad5293_;
  mcp45hvx1::MCP45HVX1 mcp45_;
  hal::adc::Channel fb;
  hal::adc::Channel temp;
  hal::adc::Channel imon;
  hal::gpio::Output n_en_ldo;
  Settings settings;
  Current v_set_current;

  Error set_num_ldos(uint8_t n) {
    v_set_current = n * au::micro(au::amperes)(50) + settings.lt3902_current;
    settings.num_ldos = 0;
    return Error::none;
  }
  Error enable(bool enable) {
    if (enable) {
      hal::Error e = ad5293_.enable();
      if (e != hal::Error::none)
        return from_hal_error(e);

      e = mcp45_.enable();
      if (e != hal::Error::none) {
        ad5293_.disable();
        return from_hal_error(e);
      }
    } else {
      mcp45_.disable();
      ad5293_.disable();
      n_en_ldo.set(hal::gpio::State::set);
    }
    return Error::none;
  }

  /**
   * @brief set the output voltage (limit) of the converter
   *
   * @param v the maximum voltage
   */
  Error set_voltage(Voltage v) {
    if (v > 20_V) {
      return Error::out_of_range;
    }
    const Resistance r = v / au::unblock_int_div(v_set_current);
    // R_wa = (1024-code)/1024*R_ab
    // code = 1024 - R_wa/R_ab*1024
    const uint16_t code = 1024 - 1024 * r / settings.ad5293_resistance;
    return from_hal_error(ad5293_.wiper(code));
  }

  /**
   * @brief get the actual output voltage. This may be lower than the voltge set
   * with set_voltage if the converter is in constant current / current limit
   * mode.
   */
  tl::expected<Voltage, Error> get_voltage() {
    return fb.sample()
        .and_then([](Voltage v) -> tl::expected<Voltage, hal::Error> {
          // the input to the adc has a 7k/1k divider -> multiply by 8 to get
          // the actual voltage
          return 8 * v;
        })
        .map_error(from_hal_error);
  }

  /**
   * @brief get the output voltage setting, i.e. the voltage set with
   * set_voltage.
   */
  tl::expected<Voltage, Error> get_voltage_limit() {
    auto pos = ad5293_.wiper();
    // R_wa = (1024-code)/1024*R_ab
    const auto r = (1024 - pos) * settings.ad5293_resistance / 1024;
    return (r * v_set_current).as<uint32_t>(au::micro(au::volts));
  }

  /**
   * @brief set the output current (limit) of the converter.
   *
   * @param i the maximum current
   */
  Error set_current(Current i) {
    // R = i/(350mA/kOhm) + 450Ohm
    const auto r_wa = i / au::unblock_int_div(set_current_factor) +
                      au::milli(au::ohms)(450'000);
    // R_wa = (256-code)/256*R_ab
    // code = 256 - R_wa/R_ab*256
    const uint8_t code = 256 - 256 * r_wa / settings.mcp45_resistance;
    return from_hal_error(mcp45_.wiper(code));
  }

  tl::expected<Current, Error> get_current() {
    // I_out = n_dlo * 5000 * I_imon = n_ldo *  5000 * V_imon / R_imon
    return imon.sample()
        .and_then([this](Voltage v_imon) -> tl::expected<Current, hal::Error> {
          return (settings.num_ldos * 5000 *
                  v_imon.as<uint32_t>(au::milli(au::volts))) /
                 au::unblock_int_div(
                     settings.imon_resistance.as<std::uint32_t>(au::ohms));
        })
        .map_error(from_hal_error);
  }

  /**
   * @brief get the output current limit
   *
   * @return
   */
  tl::expected<Current, Error> get_current_limit() {
    const std::uint8_t code = mcp45_.wiper();
    // R_wa = (256-code)/256*R_ab
    const Resistance r = (256u - code) * settings.mcp45_resistance / 256;
    // R = i/(350mA/kOhm) + 450Ohm
    // i = (R-450Ohm)*350mA/kOhm
    return (r.as<uint32_t>(au::ohms) - au::ohms(450)) * set_current_factor;
  }

  /**
   * @brief enable or disable the current limit
   */
  Error current_limit_enable(bool enable) {
    if (enable) {
      return from_hal_error(mcp45_.connect_terminals(mcp45hvx1::Terminal::W |
                                                     mcp45hvx1::Terminal::A));
    } else {
      return from_hal_error(mcp45_.disconnect_terminals(
          mcp45hvx1::Terminal::W | mcp45hvx1::Terminal::A));
    }
  }

  /**
   * @brief enable or disable the output
   */
  Error output_enable(bool enable) {
    if (enable) {
      n_en_ldo.set(hal::gpio::State::reset);
    } else {
      n_en_ldo.set(hal::gpio::State::set);
    }
    return Error::none;
  }

  tl::expected<Temperature, Error> get_temperature() {
    return temp.sample()
        .and_then([this](Voltage v) -> tl::expected<Temperature, hal::Error> {
          // V_temp = R_temp * n * 1uA/°C * Tj
          // Tj = V_temp/(R_temp * n * 1uA/°C)
          return (v /
                  au::unblock_int_div(settings.num_ldos *
                                      settings.temp_resistance * temp_factor)) +
                 au::celsius_pt(0u);
        })
        .map_error(from_hal_error);
  }

  Error cable_drop_compensation(Resistance) {
    return Error::unsupported_operation;
  }
};

} // namespace v1
} // namespace psm

#endif
