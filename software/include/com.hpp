#ifndef PSM_COM_HPP
#define PSM_COM_HPP

#include "cli/enums.hpp"
#include "cli/io.hpp"
#include "cli/string.hpp"
#include "error.hpp"

#include "cli/cli.hpp"
#include "hal/uart.hpp"
#include "psm.hpp"
#include "relay.hpp"

namespace psm::cli {

struct UartStream {
  hal::uart::Device &uart;
  ::cli::Error operator()(char c) {
    using enum hal::Error;
    switch (uart.put(c)) {
    case none:
      return ::cli::Error::none;
    case invalid_handle:
      return ::cli::Error::implementation_error;
    // case buffer_overflow:
    //   return ::cli::Error::buffer_overflow;
    // case buffer_underflow:
    //   return ::cli::Error::buffer_underflow;
    // case io_error:
    //   return ::cli::Error::io_error;
    default:
      return ::cli::Error::unknown;
    }
  }

  ::cli::Error operator()(::cli::ByteView s) {
    using enum hal::Error;
    switch (uart.put(s.data(), s.size())) {
    case none:
      return ::cli::Error::none;
    case invalid_handle:
      return ::cli::Error::implementation_error;
    // case buffer_overflow:
    //   return ::cli::Error::buffer_overflow;
    // case buffer_underflow:
    //   return ::cli::Error::buffer_underflow;
    // case io_error:
    //   return ::cli::Error::io_error;
    default:
      return ::cli::Error::unknown;
    }
  }
};

template <class S> constexpr auto make_cli(psm::Psm &module, S &&stream) {
  using namespace ::cli;
  constexpr auto id = arg<uint8_t, 0u>("id"_sc, "the channel id"_sc);
  return Cli{
      ::cli::config{},
      std::forward<S>(stream),
      param(
          "config"_sc, "module configuration parameters"_sc,
          param(
              "set"_sc, "set psm module parameters"_sc,
              func(
                  "series-support"_sc,
                  "enable the series output support of the psm. should only be enabled if the hardware actually supports it."_sc,
                  module, &psm::Psm::set_series_support,
                  arg("value"_sc,
                      "if true, enables to series output support. Else it will be disabled."_sc),
                  id),
              func(
                  "num-ldos"_sc,
                  "set the number of LDOs. should be set to the actual number of LDOs populated."_sc,
                  module, &psm::Psm::set_num_ldos,
                  arg("value"_sc, "the number of ldos"_sc), id),
              func("id"_sc, "set the id of the psm."_sc, module,
                   &psm::Psm::set_num_ldos,
                   arg("value"_sc,
                       "the modules numeric id. This should be zero for "
                       "PSMs operating as a signular "
                       "power supply. In a multi PSM configuration "
                       "this number is used as the channel "
                       "number and 0 is reserved for the master."_sc),
                   id)),
          param("get"_sc, "retrieve psm module parameters"_sc,
                func("series-support"_sc,
                     "check if the psm support a series output"_sc, module,
                     &psm::Psm::get_series_support, id),
                func("num-ldos"_sc, "get the number of LDOs"_sc, module,
                     &psm::Psm::get_num_ldos, id),
                func("id"_sc, "get the id of the psm."_sc, module,
                     &psm::Psm::set_num_ldos,
                     arg("value"_sc, "the modules numeric id"_sc), id))),
      param(
          "set"_sc, "set psm parameters"_sc,
          func("enable"_sc, "enable the output of the psm"_sc, module,
               &psm::Psm::set_enable,
               arg("value"_sc,
                   "if true, enables to output. Else disables the output."_sc),
               id),
          func("voltage"_sc, "set the output voltage of the psm"_sc, module,
               &psm::Psm::set_voltage, arg("value"_sc, "the value in volts"_sc),
               id),
          func("current"_sc, "set the output current limit of the psm"_sc,
               module, &psm::Psm::set_current,
               arg("value"_sc, "the value in amps"_sc), id),
          func("type"_sc, "set the output type of the psm"_sc, module,
               &psm::Psm::set_type, arg("value"_sc, "the output type"_sc), id),
          func("mode"_sc, "set the output mode of the psm"_sc, module,
               &psm::Psm::set_mode, arg("value"_sc, "the output mode"_sc), id),
          func("dropout-voltage"_sc,
               "set the drop output voltage of the LDO stage"_sc, module,
               &psm::Psm::set_vdrop, arg("value"_sc, "the drop out voltage"_sc),
               id),
          func("cable-resistance"_sc,
               "set the cable resistance compensation"_sc, module,
               &psm::Psm::set_rcable,
               arg("value"_sc, "the drop out voltage"_sc), id),
          func(
              "series"_sc, "enable the series output of the psm"_sc, module,
              &psm::Psm::set_enable,
              arg("enable"_sc,
                  "if true, enables to series output. Else disables the series output."_sc),
              id)),
      param(
          "get"_sc, "retrieve psm parameters"_sc,
          func("enable"_sc,
               "returns true if the output is enabled, else false"_sc, module,
               &psm::Psm::get_enable, id),
          func("voltage"_sc, "the output voltage of the psm"_sc, module,
               &psm::Psm::get_voltage, id),
          func("current"_sc, "the output current of the psm"_sc, module,
               &psm::Psm::get_current, id),
          func("voltage-limit"_sc, "the output voltage limit of the psm"_sc,
               module, &psm::Psm::get_voltage, id),
          func("current-limit"_sc, "the output current limit of the psm"_sc,
               module, &psm::Psm::get_current, id),
          func("type"_sc, "the output type of the psm"_sc, module,
               &psm::Psm::get_type, id),
          func("type"_sc, "the output mode of the psm"_sc, module,
               &psm::Psm::get_mode, id),
          func("dropout-voltage"_sc,
               "the drop output voltage of the LDO stage"_sc, module,
               &psm::Psm::get_vdrop, id),
          func("cable-resistance"_sc, "the cable resistance compensation"_sc,
               module, &psm::Psm::get_rcable, id),
          func("specs"_sc, "get the output specs for an output type"_sc, module,
               &psm::Psm::get_specs, id),
          func(
              "series"_sc, "enable the series output of the psm"_sc, module,
              &psm::Psm::set_enable,
              arg("enable"_sc,
                  "if true, enables to series output. Else disables the series output."_sc),
              id)),
  };
}

constexpr auto make_cli(psm::Psm &module, hal::uart::Device &uart) {
  return make_cli(module, UartStream(uart));
}
} // namespace psm::cli

#endif
