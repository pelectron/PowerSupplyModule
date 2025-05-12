#ifndef PSM_HPP
#define PSM_HPP

#include "cli/cli.hpp"
#include "converter.hpp"
#include "error.hpp"
#include "fixed_map.hpp"
#include "fixed_vector.hpp"
#include "hal/hal.hpp"
#include "poly.hpp"
#include "relay.hpp"
#include "units.hpp"

#include <cstdint>

namespace psm {

using float_t = float;
using MilliSeconds = std::uint32_t;

enum class CtrlFlags {
  OutputType,
  OutputEnable,
  SeriesEnable,
  OutputMode,
};

enum class OutputType { BUCK_BOOST, LDO };

enum class OutputMode { CV, CC };

template <class T> struct OffsetGainCalib {
  T offset;
  T gain;
};

using VoltageCalib = OffsetGainCalib<Voltage>;

using CurrentCalib = OffsetGainCalib<Current>;

struct Calibration {};

struct ModuleConfig {
  /**
   * if true, then the module has the series output relay populated.
   */
  bool has_series_support;

  /**
   * the number of LT3081 LDOs that are populated.
   */
  uint8_t number_of_ldos;

  /**
   * the modules numeric id.
   * This should be zero for PSMs operating as a signular power supply. In a
   * multi PSM configuration this number is used as the channel number and 0 is
   * reserved for the master.
   */
  uint8_t id;
};

struct Hardware {
  // com config
  hal::uart::Config uart;
  hal::spi::Config spi;
  hal::i2c::Config i2c;
  hal::gpio::Id n_wlat;

  /// the active high write protect pin for the external non volatile storage
  hal::gpio::Id wp;

  // enable pins
  hal::gpio::Id en_bb;
  hal::gpio::Id n_en_ldo;

  // analog pins
  hal::gpio::Id cdc;
  hal::gpio::Id temp;
  hal::gpio::Id imon;
  hal::gpio::Id switch_fb;
  hal::gpio::Id ldo_fb;

  // unused gpio
  FixedVec<hal::gpio::Config, 32> unused_gpio;

  // relay config
  FixedMap<Relay::Id, Relay::Settings, 8, binary_search> relays;
};

struct Specs {
  Voltage v_min;
  Voltage v_max;
  Current i_min;
  Current i_max;
};

struct Settings {
  OutputType type;
  OutputMode mode;
  Voltage vset;      //< the set voltage
  Current iset;      //< the set current
  Voltage vdrop;     //< the voltage drop over the ldo stage
  Resistance rcable; //< cable resistance compensation.
  MilliSeconds monitor_interval;
  bool series_output;
};

template <typename PropertyName> struct property_name;
template <typename MethodName> struct method_name;

#define PSM_PROPERTY(name)                                                     \
  POLY_PROPERTY(name);                                                         \
  template <> struct property_name<name> {                                     \
    static constexpr char value[]{#name};                                      \
  }

PSM_PROPERTY(type);
PSM_PROPERTY(mode);
PSM_PROPERTY(enable);
PSM_PROPERTY(voltage);
PSM_PROPERTY(current);
PSM_PROPERTY(settings);
PSM_PROPERTY(v_drop);
PSM_PROPERTY(r_cable);
PSM_PROPERTY(monitor_interval);
PSM_PROPERTY(config);

using storage = poly::move_only_local_storage<32, 4>;

class NonVolatileStorage {
public:
  enum class Error {

  };
  Error load(ModuleConfig &cfg);
  Error load(Settings &settings);
  Error load(Calibration &calibration);

private:
};

// methods related to Settings
POLY_METHOD(set_enable)
POLY_METHOD(get_enable)
POLY_METHOD(set_mode)
POLY_METHOD(get_mode)
POLY_METHOD(set_type)
POLY_METHOD(get_type)
POLY_METHOD(set_series)
POLY_METHOD(get_series)
POLY_METHOD(set_voltage)
POLY_METHOD(get_voltage)
POLY_METHOD(set_current)
POLY_METHOD(get_current)
POLY_METHOD(set_vdrop)
POLY_METHOD(get_vdrop)
POLY_METHOD(set_rcable)
POLY_METHOD(get_rcable)
POLY_METHOD(set_settings)
POLY_METHOD(get_settings)
POLY_METHOD(get_specs)

// Methods related to ModuleConfig
POLY_METHOD(set_config)
POLY_METHOD(get_config)
POLY_METHOD(set_series_support)
POLY_METHOD(get_series_support)
POLY_METHOD(set_num_ldos)
POLY_METHOD(get_num_ldos)
POLY_METHOD(set_id)
POLY_METHOD(get_id)

POLY_METHOD(init)
POLY_METHOD(loop)
POLY_METHOD(reset)

template <class Storage>
using Handle = poly::Struct<
    Storage, poly::type_list<>,
    poly::type_list<
        Error(init, hal::Hal &hal), Error(loop), void(reset, Error reason),
        Error(set_enable, bool enable, uint8_t id),
        Error(set_mode, OutputMode mode, uint8_t id),
        Error(set_type, OutputType type, uint8_t id),
        Error(set_series, bool enable, uint8_t id),
        Error(set_voltage, Voltage volts, uint8_t id),
        Error(set_current, Current amps, uint8_t id),
        Error(set_vdrop, Voltage volts, uint8_t id),
        Error(set_rcable, Resistance ohms, uint8_t id),
        Error(set_settings, const Settings &settings, uint8_t id),
        Error(set_config, const ModuleConfig &config, uint8_t id),
        Error(set_series_support, bool enable, uint8_t id),
        Error(set_num_ldos, uint8_t n, uint8_t id), Error(set_id, uint8_t id),
        bool(get_enable, uint8_t id) const,
        OutputMode(get_mode, uint8_t id) const,
        OutputType(get_type, uint8_t id) const,
        bool(get_series, uint8_t id) const,
        Voltage(get_voltage, uint8_t id) const,
        Current(get_current, uint8_t id) const,
        Voltage(get_vdrop, uint8_t id) const,
        Resistance(get_rcable, uint8_t id) const,
        const Settings &(get_settings, uint8_t id) const,
        const Specs &(get_specs, uint8_t id) const,
        const ModuleConfig &(get_config, uint8_t id) const,
        bool(get_series_support, uint8_t id) const,
        uint8_t(get_num_ldos, uint8_t id) const, uint8_t(get_id) const>>;

using HandleRef = Handle<poly::ref_storage>;

HandleRef get_psm();

constexpr Error to_error(hal::Error e) {
  // TODO: real conversion from hal::Error to psm::Error
  if (e == hal::Error::none)
    return Error::none;

  return Error::hal;
}

struct BasicHandle {
  bool enable{};
  ModuleConfig config{};
  Settings settings{};
  Specs specs{};

  Error init(hal::Hal &hal) {
    if (not hal.is_valid())
      return Error::invalid_handle;
    return Error::none;
  }

  Error loop() { return Error::none; }

  Error set_enable(bool enable, uint8_t = 0) {
    this->enable = enable;
    return Error::none;
  }

  bool get_enable(uint8_t id = 0) const { return enable; }

  Error set_mode(OutputMode mode, uint8_t = 0) {
    settings.mode = mode;
    return Error::none;
  }

  OutputMode get_mode(uint8_t id = 0) const { return OutputMode::CV; }

  Error set_voltage(Voltage volts, uint8_t = 0) {
    settings.vset = volts;
    return Error::none;
  }

  Voltage get_voltage(uint8_t = 0) const { return settings.vset; }

  Error set_current(Current amps, uint8_t = 0) {
    settings.iset = amps;
    return Error::none;
  }

  Current get_current(uint8_t = 0) const { return settings.iset; }

  Error set_type(OutputType type, uint8_t = 0) {
    settings.type = type;
    return Error::none;
  }

  OutputType get_type(uint8_t = 0) const { return settings.type; }

  Error set_series(bool enable, uint8_t = 0) {
    settings.series_output = enable;
    return Error::none;
  }

  bool get_series(uint8_t = 0) const { return settings.series_output; }

  Error set_config(const ModuleConfig &c, uint8_t id = 0) {
    config = c;
    return Error::none;
  }

  const ModuleConfig &get_config(uint8_t id = 0) const { return config; }

  Error set_series_support(bool supported, uint8_t = 0) {
    config.has_series_support = supported;
    return Error::none;
  }

  bool get_series_support(uint8_t = 0) const {
    return config.has_series_support;
  }

  Error set_num_ldos(uint8_t n, uint8_t = 0) {
    config.number_of_ldos = n;
    return Error::none;
  }

  uint8_t get_num_ldos(uint8_t id = 0) const { return config.number_of_ldos; }

  Error set_id(uint8_t id) {
    config.id = id;
    return Error::none;
  }

  uint8_t get_id() const { return config.id; }

  Error set_settings(const Settings &s, uint8_t = 0) {
    settings = s;
    return Error::none;
  }

  const Settings &get_settings(uint8_t = 0) const { return settings; }

  Error set_vdrop(Voltage volts, uint8_t = 0) {
    settings.vdrop = volts;
    return Error::none;
  }

  Voltage get_vdrop(uint8_t = 0) const { return settings.vdrop; }

  Error set_rcable(Resistance r, uint8_t = 0) {
    settings.rcable = r;
    return Error::none;
  }

  Resistance get_rcable(uint8_t = 0) const { return settings.rcable; }

  Specs get_specs(uint8_t = 0) const { return specs; }

  void reset(Error) {}
};

struct Psm {
  HandleRef handle_{};
  hal::Hal hal_{};
  hal::uart::Device uart{};

  Error init(HandleRef handle, hal::Hal hal, cli::io::Input in) {
    handle_ = handle;
    if (not handle_)
      return Error::invalid_handle;
    hal_ = std::move(hal);
    if (not hal_.is_valid())
      return Error::invalid_handle;
    uart.register_callback([in](char c) mutable { in.on_char(c); });
    return handle_.init(hal_);
  }

  Error loop() { return handle_.loop(); }

  Error set_enable(bool enable, uint8_t id = 0) {
    return handle_.set_enable(enable, id);
  }

  bool get_enable(uint8_t id = 0) const { return handle_.get_enable(id); }

  Error set_mode(OutputMode mode, uint8_t id = 0) {
    return handle_.set_mode(mode, id);
  }

  OutputMode get_mode(uint8_t id = 0) const { return handle_.get_mode(id); }

  Error set_voltage(Voltage volts, uint8_t id = 0) {
    return handle_.set_voltage(volts, id);
  }

  Voltage get_voltage(uint8_t id = 0) const { return handle_.get_voltage(id); }

  Error set_current(Current amps, uint8_t id = 0) {
    return handle_.set_current(amps, id);
  }

  Current get_current(uint8_t id = 0) const { return handle_.get_current(id); }

  Error set_type(OutputType type, uint8_t id = 0) {
    return handle_.set_type(type, id);
  }

  OutputType get_type(uint8_t id = 0) const { return handle_.get_type(id); }

  Error set_series(bool enable, uint8_t id = 0) {
    return handle_.set_series(enable, id);
  }

  bool get_series(uint8_t id = 0) const { return handle_.get_series(id); }

  Error set_config(const ModuleConfig &c, uint8_t id = 0) {
    return handle_.set_config(c, id);
  }

  const ModuleConfig &get_config(uint8_t id = 0) const {
    return handle_.get_config(id);
  }

  Error set_series_support(bool supported, uint8_t id = 0) {
    return handle_.set_series_support(supported, id);
  }

  bool get_series_support(uint8_t id = 0) const {
    return handle_.get_series_support(id);
  }

  Error set_num_ldos(uint8_t n, uint8_t id = 0) {
    return handle_.set_num_ldos(n, id);
  }

  uint8_t get_num_ldos(uint8_t id = 0) const {
    return handle_.get_num_ldos(id);
  }

  Error set_id(uint8_t id) { return handle_.set_id(id); }

  uint8_t get_id() const { return handle_.get_id(); }

  Error set_settings(const Settings &s, uint8_t id = 0) {
    return handle_.set_settings(s, id);
  }

  Settings get_settings(uint8_t id = 0) const {
    return handle_.get_settings(id);
  }

  Error set_vdrop(Voltage volts, uint8_t id = 0) {
    return handle_.set_vdrop(volts, id);
  }

  Voltage get_vdrop(uint8_t id = 0) const { return handle_.get_vdrop(id); }

  Error set_rcable(Resistance r, uint8_t id = 0) {
    return handle_.set_rcable(r, id);
  }

  Resistance get_rcable(uint8_t id = 0) const { return handle_.get_rcable(id); }

  Specs get_specs(uint8_t id = 0) const { return handle_.get_specs(id); }

  void reset(Error reason) { handle_.reset(reason); }
};

/**
 * @class PSM
 * @brief The PSM class acts as the API to a single PowerSupplyModule
 */
// class PSM {
// #define PSM_CHECK_ID(id) \
//   if (id != config_.id) \
//     return Error::invalid_id;
//   void apply_settings() {}
//
// public:
//   void init(NonVolatileStorage s) {
//     // storage = s;
//     // storage.load(config_);
//   }
//
//   void loop() {}
//
//   Error set_enable(bool enable, uint8_t id = 0) {
//     PSM_CHECK_ID(id);
//     if (enabled_ == enable)
//       return Error::none;
//     enabled_ = enable;
//     if (enabled_) {
//       buck_boost.enabled = true;
//       if (settings_.type == OutputType::LDO)
//         ldo.enabled = true;
//     } else {
//       if (settings_.type == OutputType::LDO)
//         ldo.enabled = false;
//       buck_boost.enabled = false;
//     }
//   }
//
//   bool get_enable(uint8_t id = 0) const;
//
//   Error set_mode(OutputMode mode, uint8_t id = 0);
//   OutputMode get_mode(uint8_t id = 0) const;
//
//   Error set_voltage(Voltage volts, uint8_t id = 0);
//   Voltage get_voltage(uint8_t id = 0) const;
//
//   Error set_current(Current amps, uint8_t id = 0);
//   Current get_current(uint8_t id = 0) const;
//
//   Error set_type(OutputType type, uint8_t id = 0);
//   OutputType get_type(uint8_t id = 0) const;
//
//   Error set_series(bool enable, uint8_t id = 0);
//   bool get_series(uint8_t id = 0) const;
//
//   Error set_config(const ModuleConfig &c, uint8_t id = 0);
//   ModuleConfig get_config(uint8_t id = 0) const;
//
//   Error set_series_support(bool supported, uint8_t id = 0);
//   bool get_series_support(uint8_t id = 0) const;
//
//   Error set_num_ldos(uint8_t n, uint8_t id = 0);
//   uint8_t get_num_ldos(uint8_t id = 0) const;
//
//   Error set_id(uint8_t id);
//   uint8_t get_id() const;
//
//   Error set_settings(const Settings &s, uint8_t id = 0);
//   Settings get_settings(uint8_t id = 0) const;
//
//   Error set_vdrop(Voltage volts, uint8_t id = 0);
//   Voltage get_vdrop(uint8_t id = 0);
//
//   Error set_rcable(Resistance r, uint8_t id = 0);
//   Voltage get_rcable(uint8_t id = 0);
//
//   Error set_specs(const Specs &specs, OutputType type, uint8_t id = 0);
//   Specs get_specs(OutputType type, uint8_t id = 0);
//
//   void reset();
//
// private:
//   struct {
//     ModuleConfig config_{};
//     Settings settings_{};
//     Calibration calib_{};
//   } sys_data{};
//   bool enabled_ = false;
//   ModuleConfig config_{};
//   Settings settings_{};
//   Calibration calib_{};
//   NonVolatileStorage storage{};
//   hal::spi::Device spi{};
//   hal::i2c::Device i2c{};
//   hal::gpio::Output wlat{};
//   hal::gpio::Output shdn{};
//   hal::gpio::Output en_bb{};
//   hal::gpio::Pin cdc{};
//   conv::BasicConverter<poly::move_only_local_storage<64>> ldo;
//   conv::BasicConverter<poly::move_only_local_storage<64>> buck_boost;
//   Relay out_p{};
//   Relay out_n{};
//   Relay out_series{};
//   Relay out_select{};
//   hal::gpio::Config gpios[29];
// };
// #undef PSM_CHECK_ID
} // namespace psm
#endif
