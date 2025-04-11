#ifndef PSM_HPP
#define PSM_HPP

#include "com.hpp"
#include "converter.hpp"
#include "fixed_map.hpp"
#include "fixed_vector.hpp"
#include "hal/gpio.hpp"
#include "hal/i2c.hpp"
#include "hal/spi.hpp"
#include "hal/uart.hpp"
#include "poly.hpp"
#include "poly/property.hpp"
#include "poly/storage/local_storage.hpp"
#include "relay.hpp"
#include "units.hpp"

#include <chrono>
#include <cstdint>

namespace psm {

using float_t = float;
using MilliSeconds = std::chrono::milliseconds;

enum class CtrlFlags {
  OutputType,
  OutputEnable,
  SeriesEnable,
  OutputMode,
};

enum class OutputType { BUCK_BOOST, LDO };

enum class OutputMode { CV, CC };

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
   * This should be zero for PSMs operating as a whole power supply. In a multi
   * PSM configuration this number is used as the channel number.
   */
  uint8_t id;
};

struct Hardware {
  // com config
  hal::uart::Config uart;
  hal::spi::MasterConfig spi;
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

struct Settings {
  ModuleConfig module;
  OutputType type;
  OutputMode mode;
  Voltage vset;      //< the set voltage
  Current iset;      //< the set current
  Voltage vdrop;     //< the voltage drop over the ldo stage
  Resistance rcable; //< cable resistance compensation.
  MilliSeconds monitor_interval;
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

/**
 * @class PSM
 * @brief The PSM class acts as the API to a single PowerSupplyModule
 */
class PSM : public poly::Struct<
                storage,
                poly::type_list<type(OutputType), mode(OutputMode),
                                voltage(Voltage), current(Current),
                                settings(Settings), config(ModuleConfig)>,
                poly::type_list<>> {
public:
  void init(NonVolatileStorage s) {
    storage = s;
    storage.load(config_);
  }
  void loop();

  Error set_type(OutputType mode);
  OutputType get_type(uint8_t id = 0) const;

  Error set_enable(bool enable);
  bool get_enable(uint8_t id = 0) const;

  Error set_series(bool enable);
  bool get_series(uint8_t id = 0) const;

  Error set_mode(OutputMode mode);
  OutputMode get_mode(uint8_t id = 0) const;

  Error set_voltage(Voltage volts);
  Voltage get_voltage(uint8_t id = 0) const;

  Error set_current(Current amps);
  Current get_current() const;

  ModuleConfig get_config() const;
  Settings get_settings() const;

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
  hal::spi::Master spi{};
  hal::i2c::Master i2c{};
  hal::gpio::Output wlat{};
  hal::gpio::Output shdn{};
  hal::gpio::Output en_bb{};
  hal::gpio::Pin cdc{};
  conv::BasicConverter<poly::move_only_local_storage<64>> ldo;
  conv::BasicConverter<poly::move_only_local_storage<64>> buck_boost;
  Relay out_p{};
  Relay out_n{};
  Relay out_series{};
  Relay out_select{};
  hal::gpio::Config gpios[29];
};

} // namespace psm
#endif
