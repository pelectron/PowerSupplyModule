#ifndef HAL_CONFIG_HPP
#define HAL_CONFIG_HPP

#include <utility>
namespace hal {

enum class ConfigError {
  success,
  already_locked,
  invalid_config,
  invalid_port,
  invalid_pin_nr,
  invalid_function,
  invalid_mode,
  invalid_speed,
  invalid_pull,
  invalid_state,
  invalid_alternate,
  invalid_enumerator,
  invalid_phase,
  invalid_polarity,
  invalid_format,
  invalid_baudrate,
  invalid_id,
  invalid_sclk,
  invalid_mosi,
  invalid_miso,
  invalid_cs,
};

template <typename Peripheral> struct ConfigResult {
  ConfigResult() = delete;
  constexpr ConfigResult(const ConfigResult &) = default;
  constexpr ConfigResult(ConfigResult &&) = default;
  constexpr ConfigResult(ConfigError error) : error(error), peripheral() {}
  constexpr ConfigResult(Peripheral &&p)
      : error(ConfigError::success), peripheral(std::move(p)) {}
  constexpr ConfigResult &operator=(const ConfigResult &) = default;
  constexpr ConfigResult &operator=(ConfigResult &&) = default;
  constexpr operator bool() const noexcept {
    return error == ConfigError::success;
  }

  ConfigError error;
  Peripheral peripheral;
};
} // namespace hal
#endif
