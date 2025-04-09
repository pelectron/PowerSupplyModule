/**
 * @file
 * @brief
 */
#ifndef STM32C0_HAL_CLOCK_HPP
#define STM32C0_HAL_CLOCK_HPP

#include "cortex/common.hpp"
#include "peripheral.hpp"
#include <cstdint>

namespace hal {
namespace clock {

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

/**
 * @sa section 6.2.1 of the reference manual
 */
struct HSE {
  enum Mode {
    crystal, //< crystal/ceramic resonator mode
    bypass   //< external clock mode
  };

  bool enabled = false;
  Mode mode = crystal;
  bool use_output_enable = false;
};

/**
 * @sa section 6.2.2 of the reference manual
 */
struct HSI : ClockConfig {
  static Error calibrate();

  // static Error enable() {
  // SET_BIT(RCC->CR, RCC_CR_HSION);
  // while (not is_stable()) {
  // }
  // return Error::none;
  // }

  // static Error disable() {
  // CLEAR_BIT(RCC->CR, RCC_CR_HSION);
  // while (is_stable()) {
  // }
  // return Error::none;
  // }

  // static void enable_irq() { SET_BIT(RCC->CIER, RCC_CIER_HSIRDYIE); }
  //
  // static void disable_irq() { CLEAR_BIT(RCC->CIER, RCC_CIER_HSIRDYIE); }
  //
  // static bool is_stable() {
  //   return READ_BIT(RCC->CR, RCC_CR_HSIRDY) == RCC_CR_HSIRDY;
  // }
};

/**
 * @sa section 6.2.4 of the reference manual
 */
struct LSE : ClockConfig {
  enum Mode {
    crystal, //< crystal/ceramic resonator mode
    bypass   //< external clock mode
  };
  enum DriveStrength { medium, high };

  Mode mode = crystal;
  DriveStrength drive_strength = medium;
  //
  // constexpr LSE(bool enabled = false, Mode mode = crystal,
  //               DriveStrength drv = medium)
  //     : ClockConfig{.input_clock = nullptr,
  //                   .name = Clock::LSE_OSC,
  //                   .enabled = enabled,
  //                   .frequency = 32768},
  //       mode{mode}, drive_strength{drv} {}
  //
  // static Error enable() {
  //   SET_BIT(RCC->CSR1, RCC_CSR1_LSEON);
  //   while (not is_stable()) {
  //   }
  //   return Error::none;
  // }
  //
  // static Error disable() {
  //   CLEAR_BIT(RCC->CSR1, RCC_CSR1_LSEON);
  //   while (is_stable()) {
  //   }
  //   return Error::none;
  // }
  //
  // static void enable_irq() { SET_BIT(RCC->CIER, RCC_CIER_LSERDYIE); }
  //
  // static void disable_irq() { CLEAR_BIT(RCC->CIER, RCC_CIER_LSERDYIE); }
  //
  // static bool is_stable() {
  //   return READ_BIT(RCC->CSR1, RCC_CSR1_LSERDY) == (RCC_CSR1_LSERDY);
  // }
};

/**
 * @sa section 6.2.5 of the reference manual
 */
struct LSI_t : ClockConfig {
  // constexpr LSI_t(bool enabled = false)
  //     : ClockConfig{.input_clock = nullptr,
  //                   .name = Clock::LSI_RC,
  //                   .enabled = enabled,
  //                   .frequency = 32768} {}
  //
  // static Error enable() {
  //   SET_BIT(RCC->CSR2, RCC_CSR2_LSION);
  //   while (not is_stable()) {
  //   }
  //   return Error::none;
  // }
  //
  // static Error disable() {
  //   CLEAR_BIT(RCC->CSR2, RCC_CSR2_LSION);
  //   while (is_stable()) {
  //   }
  //   return Error::none;
  // }
  //
  // static void enable_irq() { SET_BIT(RCC->CIER, RCC_CIER_LSIRDYIE); }
  //
  // static void disable_irq() { CLEAR_BIT(RCC->CIER, RCC_CIER_LSIRDYIE); }
  //
  // static bool is_stable() const {
  //   return READ_BIT(RCC->CSR2, RCC_CSR2_LSIRDY) == (RCC_CSR2_LSIRDY);
  // }
};

struct HSISYS_t : ClockConfig {
  // constexpr HSISYS_t(bool enabled = false)
  //     : ClockConfig{.input_clock = &HSI48,
  //                   .name = Clock::HSISYS,
  //                   .enabled = enabled,
  //                   .divider = 1,
  //                   .multiplier = 1} {}
  // constexpr Error set_multiplier(unsigned) {
  //   return Error::operation_unsupported;
  // }
  // static bool is_stable() const {
  //   return READ_BIT(RCC->CR, RCC_CR_HSIRDY) == RCC_CR_HSIRDY;
  // }
  //
  // static Error enable() {
  //   SET_BIT(RCC->CR, RCC_CR_HSION);
  //   while (not is_stable()) {
  //   }
  //   return Error::none;
  // }
  //
  // static Error disable() {
  //   CLEAR_BIT(RCC->CR, RCC_CR_HSION);
  //   while (is_stable()) {
  //   }
  //   return Error::none;
  // }
  //
  // static Error set_divider(unsigned div) {
  //   unsigned field = 0;
  //   switch (div) {
  //   case 1:
  //     field = 0;
  //     break;
  //   case 2:
  //     field = 1;
  //     break;
  //   case 4:
  //     field = 2;
  //     break;
  //   case 8:
  //     field = 3;
  //     break;
  //   case 16:
  //     field = 4;
  //     break;
  //   case 32:
  //     field = 5;
  //     break;
  //   case 64:
  //     field = 6;
  //     break;
  //   case 128:
  //     field = 7;
  //     break;
  //   default:
  //     return Error::invalid_parameter;
  //   }
  //   CLEAR_BIT(RCC->CR, RCC_CR_HSIDIV_Msk);
  //   SET_BIT(RCC->CR, field << RCC_CR_HSIDIV_Pos);
  //   return Error::none;
  // }
  //
  // static Error set_input_clock(Clock) { return Error::operation_unsupported;
  // }
};

struct HSIKER_t : ClockConfig {
  // constexpr HSIKER_t(bool enabled = false)
  //     : ClockConfig{.input_clock = &HSI48,
  //                   .name = Clock::HSIKER,
  //                   .enabled = enabled,
  //                   .divider = 1,
  //                   .multiplier = 1} {}
  //
  // static constexpr Error set_multiplier(unsigned) {
  //   return Error::operation_unsupported;
  // }
  // static Error enable() {
  //   SET_BIT(RCC->CR, RCC_CR_HSIKERON);
  //   return Error::none;
  // }
  //
  // static Error disable() {
  //   CLEAR_BIT(RCC->CR, RCC_CR_HSIKERON);
  //   return Error::none;
  // }
  //
  // static Error set_divider(unsigned div) {
  //   unsigned field = 0;
  //   switch (div) {
  //   case 1:
  //     field = 0;
  //     break;
  //   case 2:
  //     field = 1;
  //     break;
  //   case 4:
  //     field = 2;
  //     break;
  //   case 8:
  //     field = 3;
  //     break;
  //   case 16:
  //     field = 4;
  //     break;
  //   case 32:
  //     field = 5;
  //     break;
  //   case 64:
  //     field = 6;
  //     break;
  //   case 128:
  //     field = 7;
  //     break;
  //   default:
  //     return Error::invalid_parameter;
  //   }
  //   CLEAR_BIT(RCC->CR, RCC_CR_HSIKERDIV_Msk);
  //   SET_BIT(RCC->CR, field << RCC_CR_HSIKERDIV_Pos);
  //   return Error::none;
  // }
  //
  // static Error set_input_clock(Clock) { return Error::operation_unsupported;
  // }
};

struct SYSCLK_t : ClockConfig {
  //   constexpr SYSCLK_t(bool enabled = false)
  //       : ClockConfig{.input_clock = &HSI48,
  //                     .name = Clock::SYSCLK,
  //                     .enabled = enabled,
  //                     .divider = 1,
  //                     .multiplier = 1} {}
  //   constexpr Error set_multiplier(unsigned) {
  //     return Error::operation_unsupported;
  //   }
  //   /**
  //    * @note Only applies to STM32C051xx, STM32C071xx, and STM32C091xx/92xx.
  //    */
  //   static Error set_divider(unsigned div) {
  // #if defined(STM32C051XX) || defined(STM32C071XX) || defined(STM32C091XX) ||
  // \
//     defined(STM32C092XX)
  //     if (div == 0 or div > 8)
  //       return Error::invalid_parameter;
  //     SET_BIT(RCC->CR, ((div - 1) << (RCC_CR_SYSDIV_Pos)));
  //     return Error::none;
  // #else
  //     (void)div;
  //     return Error::operation_unsupported;
  // #endif
  //   }
  //
  //   static Error set_input_clock(Clock clock);
  //
  //   static Error set_input_clock(const LSI_t &LSI) {}
  //   static Error set_input_clock(const LSE &LSE) {}
  //   static Error set_input_clock(const HSI48_t &HSI) {}
  //   static Error set_input_clock(const HSE &HSE) {}
  // #ifdef STM32C071XX
  //   /**
  //    * @note Only available on STM32C071xx devices.
  //    */
  //   Error set_input_clock(const HSIUSB48_t &HSI) {}
  // #endif
};

struct HCLK_t : ClockConfig {
  // constexpr HCLK_t(bool enabled = false)
  //     : ClockConfig{.input_clock = &SYSCLK,
  //                   .name = Clock::HCLK,
  //                   .enabled = enabled,
  //                   .divider = 1,
  //                   .multiplier = 1,
  //                   .frequency = 0} {}
  // constexpr Error set_multiplier(unsigned) {
  //   return Error::operation_unsupported;
  // }
  // static Error set_divider(unsigned div);
  // static constexpr Error set_input_clock(Clock) {
  //   return Error::operation_unsupported;
  // }
};

struct HCLK8_t : ClockConfig {
  // constexpr HCLK8_t(bool enabled = false)
  //     : ClockConfig{.input_clock = &HCLK,
  //                   .name = Clock::HCLK8,
  //                   .enabled = enabled,
  //                   .divider = 8,
  //                   .multiplier = 1,
  //                   .frequency = 0} {}
  // static constexpr Error set_multiplier(unsigned) {
  //   return Error::operation_unsupported;
  // }
  // static constexpr Error set_divider(unsigned) {
  //   return Error::operation_unsupported;
  // }
  // static constexpr Error set_input_clock(Clock) {
  //   return Error::operation_unsupported;
  // }
};

struct PCLK_t : ClockConfig {
  //   constexpr PCLK_t(bool enabled = false)
  //       : ClockConfig{.input_clock = &HCLK,
  //                     .name = Clock::PCLK,
  //                     .enabled = enabled,
  //                     .divider = 1,
  //                     .multiplier = 1,
  //                     .frequency = 0} {}
  //   static constexpr Error set_multiplier(unsigned) {
  //     return Error::operation_unsupported;
  //   }
  //   static Error set_divider(unsigned div);
  //   static constexpr Error set_input_clock(Clock) {
  //     return Error::operation_unsupported;
  //   }
};

struct TIMPCLK_t : ClockConfig {
  //   constexpr TIMPCLK_t(bool enabled = false)
  //       : ClockConfig{.input_clock = &PCLK,
  //                     .name = Clock::TIMPCLK,
  //                     .enabled = enabled,
  //                     .divider = 1,
  //                     .multiplier = 1,
  //                     .frequency = 0} {}
  //   static Error set_multiplier(unsigned mult) {}
  //   static constexpr Error set_divider(unsigned) {
  //     return Error::operation_unsupported;
  //   }
  //   static constexpr Error set_input_clock(Clock) {
  //     return Error::operation_unsupported;
  //   }
};

struct RTCCLK : ClockConfig {
  // constexpr RTCCLK(bool enabled = false)
  //     : ClockConfig{.input_clock = nullptr,
  //                   .name = Clock::RTCCLK,
  //                   .enabled = enabled,
  //                   .divider = 1,
  //                   .multiplier = 1} {}
  // static Error enable();
  // static Error disable();
  //
  // static constexpr Error set_multiplier(unsigned) {
  //   return Error::operation_unsupported;
  // }
  //
  // static constexpr Error set_divider(unsigned) {
  //   return Error::operation_unsupported;
  // }
  //
  // static Error set_input_clock(Clock clock);
  //
  // static Error set_input_clock(const LSI_t &LSI) {}
  // static Error set_input_clock(const LSE &LSE) {}
  // static Error set_input_clock(const HSE &HSE) {}
};

struct USART1 : ClockConfig {
  // constexpr USART1(bool enabled = false)
  //     : ClockConfig{.input_clock = nullptr,
  //                   .name = Clock::USART1CLK,
  //                   .enabled = enabled,
  //                   .divider = 1,
  //                   .multiplier = 1} {}
  // static Error enable();
  // static Error disable();
  //
  // static constexpr Error set_multiplier(unsigned) {
  //   return Error::operation_unsupported;
  // }
  //
  // static constexpr Error set_divider(unsigned) {
  //   return Error::operation_unsupported;
  // }
  //
  // static Error set_input_clock(Clock clock);
  //
  // static Error set_input_clock(const PCLK_t &PCLK) {}
  // static Error set_input_clock(const LSE &LSE) {}
  // static Error set_input_clock(const HSIKER_t &HSIKER) {}
  // static Error set_input_clock(const SYSCLK_t &SYSCLK) {}
};

struct I2C1 : ClockConfig {
  // constexpr I2C1(bool enabled = false)
  //     : ClockConfig{.input_clock = nullptr,
  //                   .name = Clock::I2C1CLK,
  //                   .enabled = enabled,
  //                   .divider = 1,
  //                   .multiplier = 1} {}
  // static Error enable();
  // static Error disable();
  //
  // static constexpr Error set_multiplier(unsigned) {
  //   return Error::operation_unsupported;
  // }
  //
  // static constexpr Error set_divider(unsigned) {
  //   return Error::operation_unsupported;
  // }
  //
  // static Error set_input_clock(Clock clock);
  //
  // static Error set_input_clock(const PCLK_t &PCLK) {}
  // static Error set_input_clock(const HSIKER_t &HSIKER) {}
  // static Error set_input_clock(const SYSCLK_t &SYSCLK) {}
};

struct FDCAN : ClockConfig {
  // constexpr FDCAN(bool enabled = false)
  //     : ClockConfig{.input_clock = nullptr,
  //                   .name = Clock::FDCAN1CLK,
  //                   .enabled = enabled,
  //                   .divider = 1,
  //                   .multiplier = 1} {}
  // static Error enable();
  // static Error disable();
  //
  // static constexpr Error set_multiplier(unsigned) {
  //   return Error::operation_unsupported;
  // }
  //
  // static constexpr Error set_divider(unsigned) {
  //   return Error::operation_unsupported;
  // }
  //
  // static Error set_input_clock(Clock clock);
  //
  // static Error set_input_clock(const PCLK_t &PCLK) {}
  // static Error set_input_clock(const HSIKER_t &HSIKER) {}
  // static Error set_input_clock(const HSE &HSE) {}
};

struct ADC : ClockConfig {
  // constexpr ADC(bool enabled = false)
  //     : ClockConfig{.input_clock = nullptr,
  //                   .name = Clock::ADCCLK,
  //                   .enabled = enabled,
  //                   .divider = 1,
  //                   .multiplier = 1} {}
  // static Error enable();
  // static Error disable();
  //
  // static constexpr Error set_multiplier(unsigned) {
  //   return Error::operation_unsupported;
  // }
  //
  // static constexpr Error set_divider(unsigned) {
  //   return Error::operation_unsupported;
  // }
  //
  // static Error set_input_clock(Clock clock);
  //
  // static Error set_input_clock(const SYSCLK_t &SYSCLK) {}
  // static Error set_input_clock(const HSIKER_t &HSIKER) {}
};
struct I2S_CKIN : ClockConfig {
  // constexpr I2S_CKIN(bool enabled = false)
  //     : ClockConfig{.input_clock = nullptr,
  //                   .name = Clock::I2S_CKIN,
  //                   .enabled = enabled,
  //                   .divider = 1,
  //                   .multiplier = 1} {}
  // static Error enable();
  // static Error disable();
  //
  // static constexpr Error set_multiplier(unsigned) {
  //   return Error::operation_unsupported;
  // }
  //
  // static constexpr Error set_divider(unsigned) {
  //   return Error::operation_unsupported;
  // }
  //
  // static constexpr Error set_input_clock(Clock) {
  //   return Error::operation_unsupported;
  // }
};

struct I2S1 : ClockConfig {
  // constexpr I2S1(bool enabled = false)
  //     : ClockConfig{.input_clock = nullptr,
  //                   .name = Clock::I2S1CLK,
  //                   .enabled = enabled,
  //                   .divider = 1,
  //                   .multiplier = 1} {}
  // static Error enable();
  // static Error disable();
  //
  // static constexpr Error set_multiplier(unsigned) {
  //   return Error::operation_unsupported;
  // }
  //
  // static constexpr Error set_divider(unsigned) {
  //   return Error::operation_unsupported;
  // }
  //
  // static Error set_input_clock(Clock clock);
  //
  // static Error set_input_clock(const SYSCLK_t &SYSCLK) {}
  // static Error set_input_clock(const HSIKER_t &HSIKER) {}
  // static Error set_input_clock(const I2S_CKIN &I2S_CKIN) {}
};

struct USB : ClockConfig {
  // constexpr USB(bool enabled = false)
  //     : ClockConfig{.input_clock = nullptr,
  //                   .name = Clock::USBCLK,
  //                   .enabled = enabled,
  //                   .divider = 1,
  //                   .multiplier = 1} {}
  // static Error enable();
  // static Error disable();
  //
  // static constexpr Error set_multiplier(unsigned) {
  //   return Error::operation_unsupported;
  // }
  //
  // static constexpr Error set_divider(unsigned) {
  //   return Error::operation_unsupported;
  // }
  //
  // static Error set_input_clock(Clock clock);
  // static Error set_input_clock(const HSE &HSE) {}
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
std::uint32_t frequency(Peripheral p);
Error set_frequency(Clock clock, unsigned f);

} // namespace clock
} // namespace hal
#endif
