#ifndef PSM_ERROR_HPP
#define PSM_ERROR_HPP

namespace psm {
enum class Error {
  none,
  unsupported_operation,
  invalid_pin,
  invalid_tip_type,
  already_configured,
  peripheral_in_use,
  invalid_channel,
  invalid_power_control,
  disable_failed,
  disabled,
  power_input_voltage_too_high,
  analog_supply_voltage_out_of_range,
  power_board_not_supported,
  invalid_power_ctrl_type,
  required_analog_pin_not_provided,
  channel_unavailable,
  tip_type_not_supported,
  gain_out_of_range,
  invalid_state,
  buffer_empty,
  buffer_full,
  invalid_module_id,
  invalid_sensor_id,
  invalid_tip_id,
  module_unavailable,
  tip_type_does_not_fit_channel,
  duplicate_key,
  empty_result,
  invalid_id,
  unimplemented,
  invalid_pin_speed,
  invalid_power_id,
  invalid_temp_ctrl,
  invalid_slot,
  configure_failed,
  enable_failed,
  invalid_format,
  invalid_param,
  out_of_range,
  invalid_checksum
};

constexpr Error operator||(Error e1, Error e2) {
  return e1 == Error::none ? e2 : e1;
}

} // namespace psm

#endif
