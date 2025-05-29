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
#include "tl/expected.hpp"
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

struct Calibration {
  Current ldo_current_offset;
};

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
class PSM {
#define PSM_CHECK_ID(id)                                                       \
  if (id != config_.id)                                                        \
    return Error::invalid_id;

  void apply_settings() {}

public:
  void init(NonVolatileStorage s) {
    // storage = s;
    // storage.load(config_);
  }

  void loop() {}

  Error set_enable(bool enable, uint8_t /* id */ = 0) {
    if (enabled_ == enable)
      return Error::none;
    enabled_ = enable;
    if (enabled_) {
      if (auto e = buck_boost.enable(true); e != Error::none)
        return e;
      if (settings_.type == OutputType::LDO)
        if (auto e = ldo.enable(true); e != Error::none) {
          buck_boost.enable(false);
          return e;
        }
    } else {
      if (settings_.type == OutputType::LDO)
        ldo.enable(false);
      buck_boost.enable(false);
      out_n.open();
      out_p.open();
    }
    return Error::none;
  }

  bool get_enable(uint8_t /* id */ = 0) const { return enabled_; }

  Error set_mode(OutputMode mode, uint8_t /* id */ = 0) {
    if (enabled_)
      return Error::unsupported_operation;
    settings_.mode = mode;
    return Error::none;
  }

  OutputMode get_mode(uint8_t id = 0) const { return settings_.mode; }

  Error set_voltage(Voltage volts, uint8_t id = 0) {
    Error e = Error::none;
    switch (settings_.type) {
    case psm::OutputType::BUCK_BOOST:
      e = buck_boost.set_voltage(volts);
      break;
    case psm::OutputType::LDO:
      e = buck_boost.set_voltage(volts + settings_.vdrop);
      if (e != Error::none)
        break;
      e = ldo.set_voltage(volts);
      if (e != Error::none) {
        buck_boost.set_voltage(settings_.vset);
      }
    default:
      return Error::invalid_param;
    }

    if (e != Error::none)
      settings_.vset = volts;
    return e;
  }

  Voltage get_voltage(uint8_t id = 0) {
    // if (not enabled_)
    //   return Error::unsupported_operation;
    switch (settings_.type) {
    case psm::OutputType::BUCK_BOOST:
      return buck_boost.get_voltage().value();
    case psm::OutputType::LDO:
      return ldo.get_voltage().value();
    }
  }

  Error set_current(Current amps, uint8_t id = 0) {
    Error e = Error::none;
    switch (settings_.type) {
    case psm::OutputType::BUCK_BOOST:
      e = buck_boost.set_current(amps);
      break;
    case psm::OutputType::LDO:
      e = buck_boost.set_current(amps + calib_.ldo_current_offset);
      if (e != Error::none)
        break;
      e = ldo.set_current(amps);
      break;
    }
    if (e == Error::none)
      settings_.iset = amps;
    return e;
  }

  Current get_current(uint8_t id = 0) {
    // if (not enabled_)
    //   return Error::unsupported_operation;
    switch (settings_.type) {
    case psm::OutputType::BUCK_BOOST:
      return buck_boost.get_current().value();
    case psm::OutputType::LDO:
      return ldo.get_current().value();
    }
  }

  Error set_type(OutputType type, uint8_t id = 0) {
    if (enabled_) {
      return Error::unsupported_operation;
    }
    settings_.type = type;
    return Error::none;
  }

  OutputType get_type(uint8_t id = 0) const { return settings_.type; }

  Error set_series(bool enable, uint8_t id = 0) {
    if (not config_.has_series_support)
      return Error::unsupported_operation;
    if (enable)
      out_series.close();
    else
      out_series.open();
    settings_.series_output = enable;
    return Error::none;
  }

  bool get_series(uint8_t id = 0) const { return settings_.series_output; }

  Error set_config(const ModuleConfig &c, uint8_t id = 0) {
    config_ = c;
    return Error::none;
  }

  ModuleConfig get_config(uint8_t id = 0) const { return config_; }

  Error set_series_support(bool supported, uint8_t id = 0) {
    config_.has_series_support = supported;
    return Error::none;
  }

  bool get_series_support(uint8_t id = 0) const {
    return config_.has_series_support;
  }

  Error set_num_ldos(uint8_t n, uint8_t id = 0) {
    config_.number_of_ldos = n;
    ldo.set_num_ldos(n);
    return Error::none;
  }

  uint8_t get_num_ldos(uint8_t id = 0) const { return config_.number_of_ldos; }

  Error set_id(uint8_t id) {
    config_.id = id;
    return Error::none;
  }

  uint8_t get_id() const { return config_.id; }

  Error set_settings(const Settings &s, uint8_t id = 0) {
    if (enabled_)
      return Error::unsupported_operation;
    settings_ = s;
    return Error::none;
  }

  Settings get_settings(uint8_t id = 0) const { return settings_; }

  Error set_vdrop(Voltage volts, uint8_t id = 0) {
    if (volts > 2_V)
      return Error::out_of_range;

    if (enabled_) {
      if (settings_.type == OutputType::LDO)
        if (auto err = buck_boost.set_voltage(settings_.vset + volts);
            err != Error::none)
          return err;
    }
    settings_.vdrop = volts;
    return Error::none;
  }

  Voltage get_vdrop(uint8_t id = 0) { return settings_.vdrop; }

  Error set_rcable(Resistance r, uint8_t id = 0) {
    Error e = buck_boost.cable_drop_compensation(r);
    if (e != Error::none)
      return e;
    settings_.rcable = r;
    if (not enabled_)
      return Error::none;
    switch (settings_.type) {
    case psm::OutputType::LDO: {
      auto res = ldo.get_current().and_then(
          [this](Current c) -> tl::expected<void, Error> {
            Error e =
                ldo.set_voltage((settings_.vset + settings_.rcable * c)
                                    .as<std::uint32_t>(au::micro(au::volts)));
            if (e != Error::none)
              return tl::unexpected(e);
            return {};
          });
      if (res)
        break;
      return res.error();
    }
    default:
      break;
    }
    return Error::none;
  }

  Resistance get_rcable(uint8_t id = 0) { return settings_.rcable; }

  Error set_specs(const Specs &specs, OutputType type, uint8_t id = 0);
  Specs get_specs(OutputType type, uint8_t id = 0);

  void reset();

private:
  struct {
    ModuleConfig config_{};
    Settings settings_{};
    Calibration calib_{};
  } sys_data{};
  bool enabled_ = false;
  ModuleConfig config_{};
  Settings settings_{};
  Calibration calib_{};
  NonVolatileStorage storage{};
  hal::spi::Device spi{};
  hal::i2c::Device i2c{};
  hal::gpio::Output wlat{};
  hal::gpio::Output shdn{};
  hal::gpio::Output en_bb{};
  hal::gpio::Pin cdc{};
  LdoStage ldo;
  BuckBoostStage buck_boost;
  Relay out_p{};
  Relay out_n{};
  Relay out_series{};
  Relay out_select{};
  hal::gpio::Config gpios[29];
};
#undef PSM_CHECK_ID
} // namespace psm
#endif
