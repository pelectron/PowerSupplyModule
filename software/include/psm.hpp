#ifndef PSM_HPP
#define PSM_HPP

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

struct Settings {
  OutputType type;
  bool enable;
  bool series;
  OutputMode mode;

  float_t voltage;
  float_t current;

  MilliSeconds monitor_interval;
};

struct Error {};

struct Config {
  /**
   * if true, then the module has the series otuput relay populated.
   */
  bool has_series_support;
  /**
   * the number of LT3081 LDOs that are populated.
   */
  uint8_t number_of_ldos;
  /**
   * the modules numeric id
   */
  uint8_t id;
};

class PSM {
public:
  Error set_type(OutputType mode);
  OutputType get_type() const;

  Error set_enable(bool enable);
  bool get_enable() const;

  Error set_series(bool enable);
  bool get_series() const;

  Error set_mode(OutputMode mode);
  OutputMode get_mode() const;

  Error set_voltage(float_t volts);
  Voltage get_voltage() const;
  Error set_current(float_t amps);
  Current get_current() const;

private:
};

} // namespace psm
#endif
