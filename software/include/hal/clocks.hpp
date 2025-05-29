/**
 * @file
 * @brief
 */
#ifndef STM32C0_HAL_CLOCK_HPP
#define STM32C0_HAL_CLOCK_HPP

#include "hal/enums.hpp"
#include <cstdint>

namespace hal {
namespace clock {
using Frequency = std::uint32_t;
/**
 * @brief set the system clock to an internal source
 *
 * @param source an internal clock source
 */
hal::Error set_system_clock(Source source, std::uint32_t div = 1);

/**
 * @brief set the system clock to an internal source
 *
 * @param source an internal clock source
 */
hal::Error set_system_clock(Source source, Frequency f, std::uint32_t div = 1);

Frequency get_system_clock();

Frequency frequency(Peripheral p);

enum class Error { none, operation_unsupported, invalid_parameter };

/**
 *
 */
enum class Clock {
  invalid = 0,
  /// primary clocks
  /// @{
  HSI48,    //<  high-speed fully-integrated RC oscillator producing HSI48 clock
            //(48 MHz)
  HSIUSB48, //< a high-speed fully-integrated RC oscillator producing
            // HSIUSB48 clock for USB (about 48 MHz). Only applies to
            // STM32C071xx.
  HSE,      //< a high-speed oscillator with external crystal/ceramic resonator
            // or external clock source, producing HSE clock (4 to 48 MHz)
  LSI,      //< a low-speed fully-integrated RC oscillator producing LSI clock
            //(about 32 kHz)
  LSE,      //<  a low-speed oscillator with external crystal/ceramic resonator
       // or external clock source, producing LSE clock (accurate 32.768 kHz
       // or external clock up to 1 MHz)
  I2S_CKIN, //< - pin for direct clock input for I2S1 peripheral
  /// @}

  /// secondary clocks
  /// @{
  HSISYS, //< a clock derived from HSI48 through division by a factor
          // programmable from 1 to 128. Used as system clock source after
          // startup from reset, with the division by four (producing 12 MHz
          // frequency)
  SYSCLK, //<  a clock obtained through selecting one of LSE, LSI, HSE,
          // HSIUSB48, and HSISYS clocks
  HSIKER, //< a clock derived from HSI48 through division by a factor
          // programmable from 1 to 8
  HCLK,   //< a clock derived from SYSCLK through division by a factor
          // programmable from 1 to 512. Used for clocking the AHB domains.
  HCLK8,  //<  a clock derived from HCLK through division by eight
  PCLK, //< a clock derived from HCLK through division by a factor programmable
        // from 1 to 16. Used for clocking the APB domains.
  TIMPCLK, //< a clock derived from PCLK, running at PCLK frequency if the APB
           // prescaler division factor is set to 1, or at twice the PCLK
           // frequency otherwise
  /// @

  /// peripheral clocks
  /// @{
  RTCCLK,
  USART1CLK,
  I2C1CLK,
  ADCCLK,
  I2S1CLK,
  /// @
  FDCAN1, //< Only applies to STM32C092xx
};

struct ClockConfig {
  const ClockConfig *input_clock = nullptr;
  Clock name = Clock::invalid;
  bool enabled = false;
  std::uint32_t divider = 0;
  std::uint32_t multiplier = 0;
  std::uint32_t frequency = 0;
};

Error enable_irq(Clock clock);

Error disable_irq(Clock clock);
bool is_stable(Clock clock);
Error enable(Clock clock);
Error enable(Peripheral p);
Error disable(Clock clock);
Error set_source(Clock clock, Clock source);
// Error calibrate(Clock clock);

std::uint32_t frequency(Clock clock);
Error set_frequency(Clock clock, unsigned f);

} // namespace clock
} // namespace hal
#endif
