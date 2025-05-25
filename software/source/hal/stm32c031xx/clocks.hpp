#ifndef HAL_STM32C031XX_CLOCKS_HPP
#define HAL_STM32C031XX_CLOCKS_HPP

#include "au/au.hh"
#include "hal/config.hpp"
#include "hal/enums.hpp"
#include "hal/mmio.hpp"
#include "units.hpp"

#include <cstdint>

#define RCC reinterpret_cast<::stm32c031xx::Rcc *>(stm32c031xx::RCC_BASE)

namespace stm32c031xx {

enum Regs : std::uint32_t {
  RCC_BASE = 0x40021000,
  RCC_CR = RCC_BASE + 0x00,
  RCC_ICSCR = RCC_BASE + 0x04,
  RCC_CFGR = RCC_BASE + 0x08,
  RCC_CRRCR = RCC_BASE + 0x14,
  RCC_CIER = RCC_BASE + 0x18,
  RCC_CIFR = RCC_BASE + 0x1C,
  RCC_CICR = RCC_BASE + 0x20,
  RCC_IOPRSTR = RCC_BASE + 0x24,
  RCC_AHBRSTR = RCC_BASE + 0x28,
  RCC_APBRSTR1 = RCC_BASE + 0x2C,
  RCC_APBRSTR2 = RCC_BASE + 0x30,
  RCC_IOPENR = RCC_BASE + 0x34,
  RCC_AHBENR = RCC_BASE + 0x38,
  RCC_APBENR1 = RCC_BASE + 0x3C,
  RCC_APBENR2 = RCC_BASE + 0x40,
  RCC_IOPSMENR = RCC_BASE + 0x44,
  RCC_AHBSMENR = RCC_BASE + 0x48,
  RCC_APBSMENR1 = RCC_BASE + 0x4C,
  RCC_APBSMENR2 = RCC_BASE + 0x50,
  RCC_CCIPR = RCC_BASE + 0x54,
  RCC_CCIPR2 = RCC_BASE + 0x58,
  RCC_CSR1 = RCC_BASE + 0x5C,
  RCC_CSR2 = RCC_BASE + 0x60,
};

enum RCC_CR : std::uint32_t {
  RCC_CR_RESET_VALUE = 0x1540,
  RCC_CR_HSIUSB48RDY = 1 << 23,
  RCC_CR_HSIUSB48ON = 1 << 2,
  RCC_CR_CSSON = 1 << 19,
  RCC_CR_HSEBYP = 1 << 18,
  RCC_CR_HSERDY = 1 << 17,
  RCC_CR_HSEON = 1 << 16,
  RCC_CR_HSIDIV = 0b111 << 11,
  RCC_CR_HSIDIV_POS = 11,
  RCC_CR_HSIRDY = 1 << 10,
  RCC_CR_HSIKERON = 1 << 9,
  RCC_CR_HSION = 1 << 8,
  RCC_CR_HSIKERDIV = 0b111 << 5,
  RCC_CR_HSIKERDIV_POS = 5,
  RCC_CR_SYSDIV = 0b111 << 2,
  RCC_CR_SYSDIV_POS = 2
};

enum RCC_ICSR : std::uint32_t {
  RCC_ICSR_HSITRIM = 0x7F << 8,
  RCC_ICSR_HSITRIM_POS = 8,
  RCC_ICSR_HSI_CAL = 0xFF
};

enum RCC_CFGR : std::uint32_t {
  RCC_CFGR_MCOPRE = 0xFu << 28,
  RCC_CFGR_MCOPRE_POS = 28,
  RCC_CFGR_MCOSEL = 0xF << 24,
  RCC_CFGR_MCOSEL_POS = 24,
  RCC_CFGR_MCO2PRE = 0xF << 20,
  RCC_CFGR_MCO2PRE_POS = 20,
  RCC_CFGR_MCO2SEL = 0xF << 16,
  RCC_CFGR_MCO2SEL_POS = 16,
  RCC_CFGR_PPRE = 0b1111 << 12,
  RCC_CFGR_PPRE_POS = 12,
  RCC_CFGR_HPRE = 0xF << 8,
  RCC_CFGR_HPRE_POS = 8,
  RCC_CFGR_SWS = 0b111 << 3,
  RCC_CFGR_SWS_POS = 3,
  RCC_CFGR_SW = 0b111,
  RCC_CFGR_SW_POS = 0,
};

enum RCC_CRRCR : std::uint32_t { RCC_CRRCR_HSIUSB48CAL = 0xF };

enum RCC_CIER : std::uint32_t {
  RCC_CIER_HSERDYIE = 1 << 4,
  RCC_CIER_HSIRDYIE = 1 << 3,
  RCC_CIER_HSIUSB48RDYIE = 1 << 2,
  RCC_CIER_LSERDYIE = 1 << 1,
  RCC_CIER_LSIRDYIE = 1 << 0
};

enum RCC_CIFR : std::uint32_t {
  RCC_CIFR_LSECSSF = 1 << 9,
  RCC_CIFR_CSSF = 1 << 8,
  RCC_CIFR_HSERDYF = 1 << 4,
  RCC_CIFR_HSIRDYF = 1 << 3,
  RCC_CIFR_HSIUSB48RDYF = 1 << 2,
  RCC_CIFR_LSERDYF = 1 << 1,
  RCC_CIFR_LSIRDYF = 1 << 0
};

enum RCC_CICR : std::uint32_t {
  RCC_CICR_LSECSSC = 1 << 9,
  RCC_CICR_CSSC = 1 << 8,
  RCC_CICR_HSERDYC = 1 << 4,
  RCC_CICR_HSIRDYC = 1 << 3,
  RCC_CICR_HSIUSB48RDYC = 1 << 2,
  RCC_CICR_LSERDYC = 1 << 1,
  RCC_CICR_LSI = 1 << 0
};

enum RCC_IOPRSTR : std::uint32_t {
  RCC_IOPRSTR_GPIOFRST = 1 << 5,
  RCC_IOPRSTR_GPIODRST = 1 << 3,
  RCC_IOPRSTR_GPIOCRST = 1 << 2,
  RCC_IOPRSTR_GPIOBRST = 1 << 1,
  RCC_IOPRSTR_GPIOARST = 1 << 0
};

enum RCC_AHBRSTR : std::uint32_t {
  RCC_AHBRSTR_CRCRST = 1 << 12,
  RCC_AHBRSTR_FLASHRST = 1 << 8,
  RCC_AHBRSTR_DMA1RST = 1 << 0
};

enum RCC_APBRSTR1 : std::uint32_t {
  RCC_APBRSTR1_PWRRST = 1 << 28,
  RCC_APBRSTR1_DBGRST = 1 << 27,
  RCC_APBRSTR1_I2C1RST = 1 << 21,
  RCC_APBRSTR1_USART4RST = 1 << 19,
  RCC_APBRSTR1_USART3RST = 1 << 18,
  RCC_APBRSTR1_USART2RST = 1 << 17,
  RCC_APBRSTR1_CRSRST = 1 << 16,
  RCC_APBRSTR1_USBRST = 1 << 15,
  RCC_APBRSTR1_SPI2RST = 1 << 14,
  RCC_APBRSTR1_FDCAN1RST = 1 << 12,
  RCC_APBRSTR1_TIM3RST = 1 << 1,
  RCC_APBRSTR1_TIM2RST = 1 << 0
};

enum RCC_APBRSTR2 : std::uint32_t {
  RCC_APBRSTR2_ADCRST = 1 << 20,
  RCC_APBRSTR2_TIM17RST = 1 << 18,
  RCC_APBRSTR2_TIM16RST = 1 << 17,
  RCC_APBRSTR2_TIM15RST = 1 << 16,
  RCC_APBRSTR2_TIM14RST = 1 << 15,
  RCC_APBRSTR2_USART1RST = 1 << 14,
  RCC_APBRSTR2_SPI1RST = 1 << 12,
  RCC_APBRSTR2_TIM1RST = 1 << 11,
  RCC_APBRSTR2_SYSCFGRST = 1 << 0
};

enum RCC_IOPENR : std::uint32_t {
  RCC_IOPENR_GPIOFEN = 1 << 5,
  RCC_IOPENR_GPIODEN = 1 << 3,
  RCC_IOPENR_GPIOCEN = 1 << 2,
  RCC_IOPENR_GPIOBEN = 1 << 1,
  RCC_IOPENR_GPIOAEN = 1 << 0
};

enum RCC_AHBENR : std::uint32_t {
  RCC_AHBENR_CRCEN = 1 << 12,
  RCC_AHBENR_FLASHEN = 1 << 8,
  RCC_AHBENR_DMA1EN = 1 << 0
};

enum RCC_APBENR1 : std::uint32_t {
  RCC_APBENR1_PWREN = 1 << 28,
  RCC_APBENR1_DBGEN = 1 << 27,
  RCC_APBENR1_I2C2EN = 1 << 22,
  RCC_APBENR1_I2C1EN = 1 << 21,
  RCC_APBENR1_USART4EN = 1 << 19,
  RCC_APBENR1_USART3EN = 1 << 18,
  RCC_APBENR1_USART2EN = 1 << 17,
  RCC_APBENR1_CRSEN = 1 << 16,
  RCC_APBENR1_SPI2EN = 1 << 14,
  RCC_APBENR1_USBEN = 1 << 13,
  RCC_APBENR1_FDCAN1EN = 1 << 12,
  RCC_APBENR1_WWDGEN = 1 << 11,
  RCC_APBENR1_RTCAPBEN = 1 << 10,
  RCC_APBENR1_TIM3EN = 1 << 1,
  RCC_APBENR1_TIM2EN = 1 << 0
};

enum RCC_APBENR2 : std::uint32_t {
  RCC_APBENR2_ADCEN = 1 << 20,
  RCC_APBENR2_TIM17EN = 1 << 18,
  RCC_APBENR2_TIM16EN = 1 << 17,
  RCC_APBENR2_TIM15EN = 1 << 16,
  RCC_APBENR2_TIM14EN = 1 << 15,
  RCC_APBENR2_USART1EN = 1 << 14,
  RCC_APBENR2_SPI1EN = 1 << 12,
  RCC_APBENR2_TIM1EN = 1 << 11,
  RCC_APBENR2_SYSCFGEN = 1 << 0
};

enum RCC_IOPSMENR : std::uint32_t {
  RCC_IOPSMENR_GPIOSMEN = 1 << 5,
  RCC_IOPSMENR_GPIODSMEN = 1 << 3,
  RCC_IOPSMENR_GPIOCSMEN = 1 << 2,
  RCC_IOPSMENR_GPIOBSMEN = 1 << 1,
  RCC_IOPSMENR_GPIOASMEN = 1 << 0
};

enum RCC_AHBSMENR : std::uint32_t {
  RCC_AHBSMENR_CRCSMEN = 1 << 12,
  RCC_AHBSMENR_SRAMSMEN = 1 << 9,
  RCC_AHBSMENR_FLASHSMEN = 1 << 8,
  RCC_AHBSMENR_DMA1SMEN = 1 << 0
};

enum RCC_APBSMENR1 : std::uint32_t {
  RCC_APBSMENR1_PWRSMEN = 1 << 28,
  RCC_APBSMENR1_DBGSMEN = 1 << 27,
  RCC_APBSMENR1_I2C2SMEN = 1 << 22,
  RCC_APBSMENR1_I2C1SMEN = 1 << 21,
  RCC_APBSMENR1_USART4SMEN = 1 << 19,
  RCC_APBSMENR1_USART3SMEN = 1 << 18,
  RCC_APBSMENR1_USART2SMEN = 1 << 17,
  RCC_APBSMENR1_CRSSMEN = 1 << 16,
  RCC_APBSMENR1_SPI2SMEN = 1 << 14,
  RCC_APBSMENR1_USBSMEN = 1 << 13,
  RCC_APBSMENR1_FDCAN1SMEN = 1 << 12,
  RCC_APBSMENR1_WWDGSMEN = 1 << 11,
  RCC_APBSMENR1_RTCAPBSMEN = 1 << 10,
  RCC_APBSMENR1_TIM3SMEN = 1 << 1,
  RCC_APBSMENR1_TIM2SMEN = 1 << 0
};

enum RCC_APBSMENR2 : std::uint32_t {
  RCC_APBSMENR2_ADCSMEN = 1 << 20,
  RCC_APBSMENR2_TIM17SMEN = 1 << 18,
  RCC_APBSMENR2_TIM16SMEN = 1 << 17,
  RCC_APBSMENR2_TIM15SMEN = 1 << 16,
  RCC_APBSMENR2_TIM14SMEN = 1 << 15,
  RCC_APBSMENR2_USART1SMEN = 1 << 14,
  RCC_APBSMENR2_SPI1SMEN = 1 << 12,
  RCC_APBSMENR2_TIM1SMEN = 1 << 11,
  RCC_APBSMENR2_SYSCFGSMEN = 1 << 0

};

enum RCC_CCIPR : std::uint32_t {
  RCC_CCIPR_ADCSEL = 0b11u << 30,
  RCC_CCIPR_ADCSEL_POS = 30,
  RCC_CCIPR_I2S1SEL = 0b11 << 14,
  RCC_CCIPR_I2S1SEL_POS = 14,
  RCC_CCIPR_I2C1SEL = 0b1 << 12,
  RCC_CCIPR_I2C1SEL_POS = 12,
  RCC_CCIPR_FDCAN1SEL = 0b11 << 8,
  RCC_CCIPR_FDCAN1SEL_POS = 8,
  RCC_CCIPR_USART1SEL = 0b11,
  RCC_CCIPR_USART1SEL_POS = 0,
};

enum RCC_CCIPR2 : std::uint32_t { RCC_CCIPR2_USBSEL = 1 << 12 };

enum RCC_CSR1 : std::uint32_t {
  RCC_CSR1_LSCOSEL = 1 << 25,
  RCC_CSR1_LSCOEN = 1 << 24,
  RCC_CSR1_RTCRST = 1 << 16,
  RCC_CSR1_RTCEN = 1 << 15,
  RCC_CSR1_RTCSEL = 0b11 << 8,
  RCC_CSR1_RTCSEL_POS = 8,
  RCC_CSR1_LSECSSD = 1 << 6,
  RCC_CSR1_LSECSSON = 1 << 5,
  RCC_CSR1_LSEDRV = 1 << 3,
  RCC_CSR1_LSEBYP = 1 << 2,
  RCC_CSR1_LSERDY = 1 << 1,
  RCC_CSR1_LSEON = 1 << 0
};

enum RCC_CSR2 : std::uint32_t {
  LPWRRSTF = 1u << 31,
  WWDGRSTF = 1 << 30,
  IWDGRSTF = 1 << 29,
  SFTRSTF = 1 << 28,
  PWRRSTF = 1 << 27,
  PINRSTF = 1 << 26,
  OBLRSTF = 1 << 25,
  RMVF = 1 << 23,
  LSIRDY = 1 << 1,
  LSION = 1 << 0
};

/**
 *
 */
enum class Clock {
  none = 0,
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

struct Rcc {
  volatile std::uint32_t CR;
  volatile std::uint32_t ICSCR;
  volatile std::uint32_t CFGR;
  volatile std::uint32_t CRRCR;
  volatile std::uint32_t CIER;
  volatile std::uint32_t CIFR;
  volatile std::uint32_t CICR;
  volatile std::uint32_t IOPRSTR;
  volatile std::uint32_t AHBRSTR;
  volatile std::uint32_t APBRSTR1;
  volatile std::uint32_t APBRSTR2;
  volatile std::uint32_t IOPENR;
  volatile std::uint32_t AHBENR;
  volatile std::uint32_t APBENR1;
  volatile std::uint32_t APBENR2;
  volatile std::uint32_t IOPSMENR;
  volatile std::uint32_t AHBSMENR;
  volatile std::uint32_t APBSMENR1;
  volatile std::uint32_t APBSMENR2;
  volatile std::uint32_t CCIPR;
  volatile std::uint32_t CCIPR2;
  volatile std::uint32_t CSR1;
  volatile std::uint32_t CSR2;
};

struct ClockConfig {
  Frequency hse = au::ZERO;
  Frequency lse = au::ZERO;
  Frequency i2s_ckin = au::ZERO;
  Clock sysclock = Clock::HSISYS;
  unsigned sys_div = 1;
  unsigned hsisys_div = 4;
  unsigned hsiker_div = 3;
  unsigned ahb_prescaler = 3;
  unsigned apb_prescaler = 3;
  Clock rtc = Clock::none;
  Clock usart1 = Clock::PCLK;
  Clock i2c1 = Clock::PCLK;
  Clock fdcan1 = Clock::PCLK;
  Clock adc = Clock::SYSCLK;
  Clock i2s1 = Clock::HSIKER;
  Clock usb = Clock::HSIUSB48;
  bool usb_enabled;
  bool css_enabled;
  bool lse_bypass;
  bool lse_enabled;
  bool hse_bypass;
  bool hse_enabled;
  bool hsiker_enabled;
  bool hsi_enabled;
};

inline constinit struct ClockTree {
  Frequency hse = au::ZERO;
  Frequency lse = au::ZERO;
  Frequency i2s_ckin = au::ZERO;
  Frequency hsi = au::mega(au::hertz)(48u);
  Frequency hsi_usb = au::mega(au::hertz)(48u);
  Frequency hsiker = au::mega(au::hertz)(48u) / 3;
  Frequency lsi = au::kilo(au::hertz)(32u);
  Frequency hsisys = au::mega(au::hertz)(12u);
  Frequency sysclk = au::mega(au::hertz)(12u);
  Frequency hclk = au::mega(au::hertz)(12u);
  Frequency hclk8 = au::mega(au::hertz)(12u) / 8;
  Frequency pclk = au::mega(au::hertz)(12u);
  Frequency timpclk = au::mega(au::hertz)(12u);
  Frequency rtc = au::ZERO;
  Frequency usart1 = au::mega(au::hertz)(12u);
  Frequency i2c1 = au::mega(au::hertz)(12u);
  Frequency adc = au::mega(au::hertz)(12u);
  Frequency i2s1 = au::mega(au::hertz)(12u);
  Frequency fdcan1 = au::mega(au::hertz)(12u);

  hal::Error init(const ClockConfig &cfg) {
    std::uint32_t CR{};
    std::uint32_t ICSCR{};
    std::uint32_t CFGR{};
    std::uint32_t CRRCR{};
    std::uint32_t CIER{};
    std::uint32_t CIFR{};
    std::uint32_t CICR{};
    std::uint32_t IOPRSTR{};
    std::uint32_t AHBRSTR{};
    std::uint32_t APBRSTR1{};
    std::uint32_t APBRSTR2{};
    std::uint32_t IOPENR{};
    std::uint32_t AHBENR{};
    std::uint32_t APBENR1{};
    std::uint32_t APBENR2{};
    std::uint32_t IOPSMENR{};
    std::uint32_t AHBSMENR{};
    std::uint32_t APBSMENR1{};
    std::uint32_t APBSMENR2{};
    std::uint32_t CCIPR{};
    std::uint32_t CCIPR2{};
    std::uint32_t CSR1{};
    std::uint32_t CSR2{};

    i2s_ckin = cfg.i2s_ckin;
    // HSISYS setup
    unsigned field = 0;
    switch (cfg.hsisys_div) {
    case 1:
      field = 0;
      break;
    case 2:
      field = 1;
      break;
    case 4:
      field = 2;
      break;
    case 8:
      field = 3;
      break;
    case 16:
      field = 4;
      break;
    case 32:
      field = 5;
      break;
    case 64:
      field = 6;
      break;
    case 128:
      field = 7;
      break;
    default:
      return hal::Error::invalid_param;
    }
    CR |= field << RCC_CR_HSIDIV_POS;
    if (cfg.hsi_enabled) {
      CR |= RCC_CR_HSION;
      hsisys = au::hertz(48'000'000u) / cfg.hsisys_div;
    } else {
      hsisys = au::ZERO;
    }

    // HSIKER setup
    if (cfg.hsiker_div == 0 or cfg.hsiker_div > 8)
      return hal::Error::invalid_param;
    CR |= cfg.hsiker_div << RCC_CR_HSIKERDIV_POS;
    if (cfg.hsiker_enabled) {
      hsiker = au::hertz(48'000'000u) / cfg.hsiker_div;
    } else {
      hsiker = au::ZERO;
    }

    // LSE setup
    if (cfg.lse_enabled) {
      if (cfg.lse < au::kilo(au::hertz)(32'000u) or
          cfg.lse > au::mega(au::hertz)(1u))
        return hal::Error::invalid_param;
      lse = cfg.lse;
      CSR1 |= RCC_CSR1_LSEON;
    }
    if (cfg.lse_bypass)
      CSR1 |= RCC_CSR1_LSEBYP;

    // HSE setup
    if (cfg.hse_enabled) {
      if (cfg.hse < au::mega(au::hertz)(4u) or
          cfg.hse > au::mega(au::hertz)(48u))
        return hal::Error::invalid_param;
      hse = cfg.hse;
      CR |= RCC_CR_HSEON;
    }
    if (cfg.hse_bypass)
      CR |= RCC_CR_HSEBYP;

    // TODO: USB clock setup
    // switch (cfg.usb) {
    // case Clock::HSE:
    //   break;
    // case Clock::HSIUSB48:
    //   CCIPR2 |= RCC_CCIPR2_USBSEL;
    //   break;
    // default:
    //   return hal::Error::invalid_param;
    // }

    // RTC setup
    switch (cfg.rtc) {
    case Clock::none:
      rtc = au::ZERO;
      break;
    case Clock::LSI:
      rtc = lsi;
      CSR1 |= 0b10u << RCC_CSR1_RTCSEL_POS;
      break;
    case Clock::LSE:
      rtc = lse;
      CSR1 |= 0b01u << RCC_CSR1_RTCSEL_POS;
      break;
    case Clock::HSE:
      rtc = hse / 32;
      CSR1 |= 0b11u << RCC_CSR1_RTCSEL_POS;
      break;
    default:
      return hal::Error::invalid_param;
    }

    // SYSCLK setup
    if (cfg.sys_div != 1) {
      // TODO: This bitfield is only available on STM32C051xx, STM32C071xx, and
      // STM32C091xx/92xx
      return hal::Error::invalid_param;
    }
    switch (cfg.sysclock) {
    case Clock::LSE:
      sysclk = lse / cfg.sys_div;
      CFGR |= 0b100u << RCC_CFGR_SW_POS;
      break;
    case Clock::LSI:
      sysclk = lsi / cfg.sys_div;
      CFGR |= 0b011u << RCC_CFGR_SW_POS;
      break;
    case Clock::HSE:
      sysclk = hse / cfg.sys_div;
      CFGR |= 0b001u << RCC_CFGR_SW_POS;
      break;
    case Clock::HSISYS:
      [[fallthrough]];
    case Clock::HSI48:
      sysclk = hsi / cfg.sys_div;
      break;
    case Clock::HSIUSB48:
      // TODO: STM32C071xx
      [[fallthrough]];
    default:
      return hal::Error::invalid_param;
    }

    // HCLK setup
    field = 0;
    switch (cfg.ahb_prescaler) {
    case 1:
      field = 0;
      break;
    case 2:
      field = 0b1000u;
      break;
    case 4:
      field = 0b1001u;
      break;
    case 8:
      field = 0b1010u;
      break;
    case 16:
      field = 0b1011u;
      break;
    case 64:
      field = 0b1100u;
      break;
    case 128:
      field = 0b1101u;
      break;
    case 256:
      field = 0b1110u;
      break;
    case 512:
      field = 0b1111u;
      break;
    default:
      return hal::Error::invalid_param;
    }
    hclk = sysclk / cfg.ahb_prescaler;
    hclk8 = hclk / 8;
    CFGR |= field << RCC_CFGR_HPRE_POS;

    // PCLK setup
    field = 0;
    switch (cfg.apb_prescaler) {
    case 1:
      field = 0;
      break;
    case 2:
      field = 0b0100u;
      break;
    case 4:
      field = 0b0101u;
      break;
    case 8:
      field = 0b0110u;
      break;
    case 16:
      field = 0b0111u;
      break;
    default:
      return hal::Error::invalid_param;
    }
    CFGR |= field << RCC_CFGR_PPRE_POS;
    pclk = hclk / cfg.apb_prescaler;

    // TIMPCLK setup
    timpclk = cfg.apb_prescaler == 1 ? pclk : 2 * pclk;

    // USART1 setup
    switch (cfg.usart1) {
    case Clock::PCLK:
      usart1 = pclk;
      break;
    case Clock::LSE:
      usart1 = lse;
      CCIPR |= 0b11u;
      break;
    case Clock::HSIKER:
      usart1 = hsiker;
      CCIPR |= 0b10u;
      break;
    case Clock::SYSCLK:
      usart1 = sysclk;
      CCIPR |= 0b01u;
      break;
    default:
      return hal::Error::invalid_param;
    }

    // I2C1 setup
    switch (cfg.i2c1) {
    case Clock::PCLK:
      i2c1 = pclk;
      CCIPR |= 0b00u << RCC_CCIPR_I2C1SEL_POS;
      break;
    case Clock::HSIKER:
      i2c1 = hsiker;
      CCIPR |= 0b10u << RCC_CCIPR_I2C1SEL_POS;
      break;
    case Clock::SYSCLK:
      i2c1 = sysclk;
      CCIPR |= 0b01u << RCC_CCIPR_I2C1SEL_POS;
      break;
    default:
      return hal::Error::invalid_param;
    }

    // TODO: FDCAN1 clock setup

    // ADC
    switch (cfg.adc) {
    case Clock::SYSCLK:
      adc = sysclk;
      break;
    case Clock::HSIKER:
      adc = hsiker;
      CCIPR |= 0b10u << RCC_CCIPR_ADCSEL_POS;
      break;
    default:
      return hal::Error::invalid_param;
    }

    // TODO: I2S1 clock setup

    // TODO: USB clock setup

    // apply the configuration
    RCC->CR = CR;
    RCC->ICSCR = ICSCR;
    RCC->CFGR = CFGR;
    RCC->CRRCR = CRRCR;
    RCC->CIER = CIER;
    RCC->CICR = CICR;
    RCC->CIFR = CIFR;
    RCC->IOPRSTR = IOPRSTR;
    RCC->AHBRSTR = AHBRSTR;
    RCC->APBRSTR1 = APBRSTR1;
    RCC->APBRSTR2 = APBRSTR2;
    RCC->IOPENR = IOPENR;
    RCC->AHBENR = AHBENR;
    RCC->APBENR1 = APBENR1;
    RCC->APBENR2 = APBENR2;
    RCC->IOPSMENR = IOPSMENR;
    RCC->AHBSMENR = AHBSMENR;
    RCC->APBSMENR1 = APBSMENR1;
    RCC->APBSMENR2 = APBSMENR2;
    RCC->CCIPR = CCIPR;
    RCC->CCIPR2 = CCIPR2;
    RCC->CSR1 = CSR1;
    RCC->CSR2 = CSR2;
    return hal::Error::none;
  }

  void update() noexcept {
    const std::uint32_t cr = RCC->CR;
    const std::uint32_t cfgr = RCC->CFGR;
    const std::uint32_t ccipr = RCC->CCIPR;
    const std::uint32_t csr1 = RCC->CSR1;

    if ((cr & RCC_CR_HSIUSB48RDY) == 0)
      hsi_usb = au::ZERO;

    if ((cr & RCC_CR_HSERDY) == 0)
      hse = au::ZERO;

    if ((cr & RCC_CR_HSIRDY) == 0)
      hsi = au::ZERO;

    if ((csr1 & RCC_CSR1_LSERDY) == 0)
      lsi = au::ZERO;

    std::uint32_t ahb_pre = (cfgr >> 8) & 0x0Fu;
    if (ahb_pre & 0b1000u)
      ahb_pre = (ahb_pre & 0b111u) + 1;
    else
      ahb_pre = 0;

    std::uint32_t apb_pre = (cfgr >> 12) & 0b111u;
    if (apb_pre & 0b100u)
      apb_pre = (apb_pre & 0b111u) + 1;
    else
      apb_pre = 0;

    // dividers
    const std::uint32_t hsidiv = (cr & RCC_CR_HSIDIV) >> RCC_CR_HSIDIV_POS;
    const std::uint32_t hsikerdiv =
        (cr & RCC_CR_HSIKERDIV) >> RCC_CR_HSIKERDIV_POS;
    const std::uint32_t sysdiv = (cr & RCC_CR_SYSDIV) >> RCC_CR_SYSDIV_POS;

    // clock frequencies
    const std::uint32_t hsisys = 48'000'000u >> hsidiv;
    const std::uint32_t sysclk = [&]() -> std::uint32_t {
      switch ((cfgr & RCC_CFGR_SWS) >> RCC_CFGR_SWS_POS) {
      case 0:
        return hsisys >> sysdiv;
      case 1:
        return hse.in(au::hertz) / (1u << sysdiv);
      case 2:
        return 48'000'000u >> sysdiv;
      case 3:
        return 32'000 >> sysdiv;
      case 4:
        return lse.in(au::hertz) / (1u << sysdiv);
      }
      return 0;
    }();
    const std::uint32_t hsiker = 48'000'000u >> hsikerdiv;
    const std::uint32_t hclk = sysclk >> ahb_pre;
    const std::uint32_t hclk8 = hclk / 8u;
    const std::uint32_t pclk = hclk >> apb_pre;
    const std::uint32_t timpclk = apb_pre == 0 ? pclk : 2 * pclk;
    const std::uint32_t rtc = [&]() -> std::uint32_t {
      if (csr1 & RCC_CSR1_RTCEN)
        switch ((csr1 & RCC_CSR1_RTCSEL) >> RCC_CSR1_RTCSEL_POS) {
        case 0:
          return 0;
        case 1:
          return lse.in(au::hertz) / (1u << sysdiv);
        case 2:
          return 32'000u;
        case 3:
          return hse.in(au::hertz) / 32;
        }
      return 0;
    }();
    const std::uint32_t usart1 = [&]() -> std::uint32_t {
      switch ((ccipr & RCC_CCIPR_USART1SEL) >> RCC_CCIPR_USART1SEL_POS) {
      case 0:
        return pclk;
      case 1:
        return sysclk;
      case 2:
        return hsiker;
      case 3:
        return lse.in(au::hertz);
      }
      return 0;
    }();
    const std::uint32_t fdcan1 = [&]() -> std::uint32_t {
      switch ((ccipr & RCC_CCIPR_FDCAN1SEL) >> RCC_CCIPR_FDCAN1SEL_POS) {
      case 0:
        return pclk;
      case 1:
        return hsiker;
      case 2:
        return hse.in(au::hertz);
      }
      return 0;
    }();
    const std::uint32_t i2c1 = [&]() -> std::uint32_t {
      switch ((ccipr & RCC_CCIPR_I2C1SEL) >> RCC_CCIPR_I2C1SEL_POS) {
      case 0:
        return pclk;
      case 1:
        return sysclk;
      case 2:
        return hsiker;
      }
      return 0;
    }();
    const std::uint32_t i2s1 = [&]() -> std::uint32_t {
      switch ((ccipr & RCC_CCIPR_I2S1SEL) >> RCC_CCIPR_I2S1SEL_POS) {
      case 0:
        return sysclk;
      case 2:
        return hsiker;
      case 3:
        return i2s_ckin.in(au::hertz);
      }
      return 0;
    }();
    const std::uint32_t adc = [&]() -> std::uint32_t {
      switch ((ccipr & RCC_CCIPR_ADCSEL) >> RCC_CCIPR_ADCSEL_POS) {
      case 0:
        return sysclk;
      case 2:
        return hsiker;
      }
      return 0;
    }();

    this->hsisys = au::hertz(hsisys);
    this->hclk = au::hertz(hclk);
    this->hclk8 = au::hertz(hclk8);
    this->pclk = au::hertz(pclk);
    this->timpclk = au::hertz(timpclk);
    this->rtc = au::hertz(rtc);
    this->usart1 = au::hertz(usart1);
    this->i2c1 = au::hertz(i2c1);
    this->adc = au::hertz(adc);
    this->i2s1 = au::hertz(i2s1);
    this->fdcan1 = au::hertz(fdcan1);
  }

  /**
   * @brief set the system clock to an internal source
   *
   * @param source an internal clock source
   */
  hal::Error set_system_clock(hal::clock::Source source,
                              unsigned divider = 1) noexcept {
    using enum hal::clock::Source;
    switch (source) {
    case low_speed_internal:
      if (divider != 1)
        return hal::Error::invalid_param;
      hal::mmio::set(RCC->CFGR, RCC_CFGR_SW, 0b11u, RCC_CFGR_SW_POS);
      if (hal::mmio::get(RCC->CFGR, RCC_CFGR_SWS, RCC_CFGR_SWS_POS) != 0b11u)
        return hal::Error::clock_error;
      break;
    case high_speed_internal:
      switch (divider) {
      case 1:
        hal::mmio::set(RCC->CR, RCC_CR_HSIDIV, 0u, RCC_CR_HSIDIV_POS);
        break;
      case 2:
        hal::mmio::set(RCC->CR, RCC_CR_HSIDIV, 1u, RCC_CR_HSIDIV_POS);
        break;
      case 4:
        hal::mmio::set(RCC->CR, RCC_CR_HSIDIV, 2u, RCC_CR_HSIDIV_POS);
        break;
      case 8:
        hal::mmio::set(RCC->CR, RCC_CR_HSIDIV, 3u, RCC_CR_HSIDIV_POS);
        break;
      case 16:
        hal::mmio::set(RCC->CR, RCC_CR_HSIDIV, 4u, RCC_CR_HSIDIV_POS);
        break;
      case 32:
        hal::mmio::set(RCC->CR, RCC_CR_HSIDIV, 5u, RCC_CR_HSIDIV_POS);
        break;
      case 64:
        hal::mmio::set(RCC->CR, RCC_CR_HSIDIV, 6u, RCC_CR_HSIDIV_POS);
        break;
      case 128:
        hal::mmio::set(RCC->CR, RCC_CR_HSIDIV, 7u, RCC_CR_HSIDIV_POS);
        break;
      default:
        return hal::Error::invalid_param;
      }
      hal::mmio::set(RCC->CFGR, RCC_CFGR_SW, 0u, RCC_CFGR_SW_POS);
      if (hal::mmio::get(RCC->CFGR, RCC_CFGR_SWS, RCC_CFGR_SWS_POS) != 0)
        return hal::Error::clock_error;
      break;
    default:
      return hal::Error::invalid_param;
    }
    update();
    return hal::Error::none;
  }

  /**
   * @brief set the system clock to an external source
   *
   * @param source an external clock source
   */
  hal::Error set_system_clock(hal::clock::Source source, Frequency f,
                              unsigned divider = 1) noexcept {
    using enum hal::clock::Source;
    switch (source) {
    case low_speed_internal:
      [[fallthrough]];
    case high_speed_internal:
      return set_system_clock(source, divider);
    case low_speed_external:
      if (divider != 1)
        return hal::Error::invalid_param;
      hal::mmio::set(RCC->CFGR, RCC_CFGR_SW, 0b100u, RCC_CFGR_SW_POS);
      if (hal::mmio::get(RCC->CFGR, RCC_CFGR_SWS, RCC_CFGR_SWS_POS) != 0b100u)
        return hal::Error::clock_error;
      break;
    case high_speed_external:
      if (divider != 1)
        return hal::Error::invalid_param;
      hal::mmio::set(RCC->CFGR, RCC_CFGR_SW, 1u, RCC_CFGR_SW_POS);
      if (hal::mmio::get(RCC->CFGR, RCC_CFGR_SWS, RCC_CFGR_SWS_POS) != 1u)
        return hal::Error::clock_error;
      break;
    default:
      return hal::Error::invalid_param;
    }
    update();
    return hal::Error::none;
  }

  constexpr Frequency system_frequency() const noexcept { return sysclk; }

  /**
   * @brief set the divider a a clock
   * @param clock one of HSISYS, HSIKER, SYSCLK, HCLK, PCLK
   * @param divider the division ratio
   */
  hal::Error set_divider(Clock clock, unsigned divider) {
    unsigned field = 0;
    switch (clock) {
    case Clock::HSISYS:
      switch (divider) {
      case 1:
        field = 0;
        break;
      case 2:
        field = 1;
        break;
      case 4:
        field = 2;
        break;
      case 8:
        field = 3;
        break;
      case 16:
        field = 4;
        break;
      case 32:
        field = 5;
        break;
      case 64:
        field = 6;
        break;
      case 128:
        field = 7;
        break;
      default:
        return hal::Error::invalid_param;
      }
      hal::mmio::set(RCC->CR, RCC_CR_HSIDIV, field, RCC_CR_HSIDIV_POS);
      break;
    case Clock::HSIKER:
      switch (divider) {
      case 1:
        break;
      case 2:
        break;
      case 3:
        break;
      case 4:
        break;
      case 5:
        break;
      case 6:
        break;
      case 7:
        break;
      case 8:
        break;
      default:
        return hal::Error::invalid_param;
      }
      hal::mmio::set(RCC->CR, RCC_CR_HSIKERDIV, divider, RCC_CR_HSIKERDIV_POS);
      break;
    case Clock::SYSCLK:
      // Only applies to STM32C051xx, STM32C071xx, and STM32C091xx/92xx.
      // TODO: check which stm32c0 this is running on
      if (divider != 1)
        return hal::Error::invalid_param;
      else
        return hal::Error::none;
    case Clock::HCLK:
      switch (divider) {
      case 1:
        field = 0;
        break;
      case 2:
        field = 1;
        break;
      case 4:
        field = 2;
        break;
      case 8:
        field = 3;
        break;
      case 16:
        field = 4;
        break;
      case 32:
        field = 5;
        break;
      case 64:
        field = 6;
        break;
      case 128:
        field = 7;
        break;
      case 256:
        field = 8;
        break;
      case 512:
        field = 9;
        break;
      default:
        return hal::Error::invalid_param;
      }
      hal::mmio::set(RCC->CFGR, RCC_CFGR_HPRE, field, RCC_CFGR_HPRE_POS);
      break;
    case Clock::PCLK:
      switch (divider) {
      case 1:
        field = 0;
        break;
      case 2:
        field = 1;
        break;
      case 4:
        field = 2;
        break;
      case 8:
        field = 3;
        break;
      case 16:
        field = 4;
        break;
      default:
        return hal::Error::invalid_param;
      }
      hal::mmio::set(RCC->CFGR, RCC_CFGR_PPRE, field, RCC_CFGR_PPRE_POS);
      break;
    default:
      return hal::Error::invalid_param;
    }
    update();
    return hal::Error::none;
  }

  /// get the clock frequency of a peripheral.
  /// returns 0 Hz if the peripheral does not exist or the its clock is not
  /// enabled.
  Frequency frequency(hal::Peripheral p) const noexcept {
    // TODO: CAN and I2S
    switch (p) {
    case hal::Peripheral::rtc:
      return rtc;
    case hal::Peripheral::crc:
      return hclk;
    case hal::Peripheral::pwr:
      return sysclk;
    case hal::Peripheral::dbg:
      return hclk;
    case hal::Peripheral::flash:
      return hclk;
    case hal::Peripheral::dma:
      return hclk;
    case hal::Peripheral::adc_a:
      // TODO: check ADC input clock
      return adc;
    case hal::Peripheral::gpio_a:
      [[fallthrough]];
    case hal::Peripheral::gpio_b:
    case hal::Peripheral::gpio_c:
    case hal::Peripheral::gpio_d:
    case hal::Peripheral::gpio_f:
    case hal::Peripheral::gpio_all:
      return hclk;
    case hal::Peripheral::i2c_a:
      return i2c1;
    case hal::Peripheral::spi_a:
      return pclk;
    case hal::Peripheral::tim_a:
      [[fallthrough]];
    case hal::Peripheral::tim_c:
      [[fallthrough]];
    case hal::Peripheral::tim_n:
      [[fallthrough]];
    case hal::Peripheral::tim_p:
      [[fallthrough]];
    case hal::Peripheral::tim_q:
      [[fallthrough]];
    case hal::Peripheral::tim_all:
      // TODO: check if PCLK multiplier is two or one
      return timpclk;
    case hal::Peripheral::uart_a:
      return usart1;
    case hal::Peripheral::uart_b:
      return pclk;
    case hal::Peripheral::iwdg:
      return lsi;
      // case hal:::Peripheral::can_a:
    default:
      return au::ZERO;
    }
  }

  /// enable the clock for a peripheral
  hal::Error enable(hal::Peripheral p) noexcept {
    // TODO: CAN and I2S
    switch (p) {
    case hal::Peripheral::crc:
      hal::mmio::set_bits(RCC->AHBENR, RCC_AHBENR_CRCEN);
      return hal::Error::none;
    case hal::Peripheral::pwr:
      hal::mmio::set_bits(RCC->APBENR1, RCC_APBENR1_PWREN);
      return hal::Error::none;
    case hal::Peripheral::dbg:
      hal::mmio::set_bits(RCC->APBENR1, RCC_APBENR1_DBGEN);
      return hal::Error::none;
    case hal::Peripheral::flash:
      hal::mmio::set_bits(RCC->AHBENR, RCC_AHBENR_FLASHEN);
      return hal::Error::none;
    case hal::Peripheral::dma:
      hal::mmio::set_bits(RCC->AHBENR, RCC_AHBENR_DMA1EN);
      return hal::Error::none;
    // WATCHDOG
    case hal::Peripheral::wwdg:
      hal::mmio::set_bits(RCC->APBENR1, RCC_APBENR1_WWDGEN);
      return hal::Error::none;
    // ADC
    case hal::Peripheral::adc_a:
      [[fallthrough]];
    case hal::Peripheral::adc_all:
      hal::mmio::set_bits(RCC->APBENR2, RCC_APBENR2_ADCEN);
      return hal::Error::none;
    // GPIO
    case hal::Peripheral::gpio_a:
      hal::mmio::set_bits(RCC->IOPENR, RCC_IOPENR_GPIOAEN);
      return hal::Error::none;
    case hal::Peripheral::gpio_b:
      hal::mmio::set_bits(RCC->IOPENR, RCC_IOPENR_GPIOBEN);
      return hal::Error::none;
    case hal::Peripheral::gpio_c:
      hal::mmio::set_bits(RCC->IOPENR, RCC_IOPENR_GPIOCEN);
      return hal::Error::none;
    case hal::Peripheral::gpio_d:
      hal::mmio::set_bits(RCC->IOPENR, RCC_IOPENR_GPIODEN);
      return hal::Error::none;
    case hal::Peripheral::gpio_f:
      hal::mmio::set_bits(RCC->IOPENR, RCC_IOPENR_GPIOFEN);
      return hal::Error::none;
    case hal::Peripheral::gpio_all:
      hal::mmio::set_bits(RCC->IOPENR, RCC_IOPENR_GPIOAEN | RCC_IOPENR_GPIOBEN |
                                           RCC_IOPENR_GPIOCEN |
                                           RCC_IOPENR_GPIODEN |
                                           RCC_IOPENR_GPIOFEN);
      return hal::Error::none;
      // I2C
    case hal::Peripheral::i2c_a:
      [[fallthrough]];
    case hal::Peripheral::i2c_all:
      hal::mmio::set_bits(RCC->APBENR1, RCC_APBENR1_I2C1EN);
      return hal::Error::none;
      // SPI
    case hal::Peripheral::spi_a:
    case hal::Peripheral::spi_all:
      hal::mmio::set_bits(RCC->APBENR2, RCC_APBENR2_SPI1EN);
      return hal::Error::none;
      // TIMERS
    case hal::Peripheral::tim_a:
      hal::mmio::set_bits(RCC->APBENR2, RCC_APBENR2_TIM1EN);
      return hal::Error::none;
    case hal::Peripheral::tim_c:
      hal::mmio::set_bits(RCC->APBENR1, RCC_APBENR1_TIM2EN);
      return hal::Error::none;
    case hal::Peripheral::tim_n:
      hal::mmio::set_bits(RCC->APBENR2, RCC_APBENR2_TIM14EN);
      return hal::Error::none;
    case hal::Peripheral::tim_p:
      hal::mmio::set_bits(RCC->APBENR2, RCC_APBENR2_TIM16EN);
      return hal::Error::none;
    case hal::Peripheral::tim_q:
      hal::mmio::set_bits(RCC->APBENR2, RCC_APBENR2_TIM17EN);
      return hal::Error::none;
      // UARTS
    case hal::Peripheral::uart_a:
      hal::mmio::set_bits(RCC->APBENR2, RCC_APBENR2_USART1EN);
      return hal::Error::none;
    case hal::Peripheral::uart_b:
      hal::mmio::set_bits(RCC->APBENR1, RCC_APBENR1_USART2EN);
      return hal::Error::none;
    default:
      return hal::Error::invalid_param;
    }
  }

  /// disable a peripherals clock
  hal::Error disable(hal::Peripheral p) noexcept {
    // TODO: CAN and I2S
    switch (p) {
    case hal::Peripheral::crc:
      hal::mmio::reset_bits(RCC->AHBENR, RCC_AHBENR_CRCEN);
      return hal::Error::none;
    case hal::Peripheral::pwr:
      hal::mmio::reset_bits(RCC->APBENR1, RCC_APBENR1_PWREN);
      return hal::Error::none;
    case hal::Peripheral::dbg:
      hal::mmio::reset_bits(RCC->APBENR1, RCC_APBENR1_DBGEN);
      return hal::Error::none;
    case hal::Peripheral::flash:
      hal::mmio::reset_bits(RCC->AHBENR, RCC_AHBENR_FLASHEN);
      return hal::Error::none;
    case hal::Peripheral::dma:
      hal::mmio::reset_bits(RCC->AHBENR, RCC_AHBENR_DMA1EN);
      return hal::Error::none;
    // WATCHDOG
    case hal::Peripheral::wwdg:
      hal::mmio::reset_bits(RCC->APBENR1, RCC_APBENR1_WWDGEN);
      return hal::Error::none;
    // ADC
    case hal::Peripheral::adc_a:
      [[fallthrough]];
    case hal::Peripheral::adc_all:
      hal::mmio::reset_bits(RCC->APBENR2, RCC_APBENR2_ADCEN);
      return hal::Error::none;
    // GPIO
    case hal::Peripheral::gpio_a:
      hal::mmio::reset_bits(RCC->IOPENR, RCC_IOPENR_GPIOAEN);
      return hal::Error::none;
    case hal::Peripheral::gpio_b:
      hal::mmio::reset_bits(RCC->IOPENR, RCC_IOPENR_GPIOBEN);
      return hal::Error::none;
    case hal::Peripheral::gpio_c:
      hal::mmio::reset_bits(RCC->IOPENR, RCC_IOPENR_GPIOCEN);
      return hal::Error::none;
    case hal::Peripheral::gpio_d:
      hal::mmio::reset_bits(RCC->IOPENR, RCC_IOPENR_GPIODEN);
      return hal::Error::none;
    case hal::Peripheral::gpio_f:
      hal::mmio::reset_bits(RCC->IOPENR, RCC_IOPENR_GPIOFEN);
      return hal::Error::none;
    case hal::Peripheral::gpio_all:
      hal::mmio::reset_bits(RCC->IOPENR,
                            RCC_IOPENR_GPIOAEN | RCC_IOPENR_GPIOBEN |
                                RCC_IOPENR_GPIOCEN | RCC_IOPENR_GPIODEN |
                                RCC_IOPENR_GPIOFEN);
      return hal::Error::none;
      // I2C
    case hal::Peripheral::i2c_a:
      [[fallthrough]];
    case hal::Peripheral::i2c_all:
      hal::mmio::reset_bits(RCC->APBENR1, RCC_APBENR1_I2C1EN);
      return hal::Error::none;
      // SPI
    case hal::Peripheral::spi_a:
    case hal::Peripheral::spi_all:
      hal::mmio::reset_bits(RCC->APBENR2, RCC_APBENR2_SPI1EN);
      return hal::Error::none;
      // TIMERS
    case hal::Peripheral::tim_a:
      hal::mmio::reset_bits(RCC->APBENR2, RCC_APBENR2_TIM1EN);
      return hal::Error::none;
    case hal::Peripheral::tim_c:
      hal::mmio::reset_bits(RCC->APBENR1, RCC_APBENR1_TIM2EN);
      return hal::Error::none;
    case hal::Peripheral::tim_n:
      hal::mmio::reset_bits(RCC->APBENR2, RCC_APBENR2_TIM14EN);
      return hal::Error::none;
    case hal::Peripheral::tim_p:
      hal::mmio::reset_bits(RCC->APBENR2, RCC_APBENR2_TIM16EN);
      return hal::Error::none;
    case hal::Peripheral::tim_q:
      hal::mmio::reset_bits(RCC->APBENR2, RCC_APBENR2_TIM17EN);
      return hal::Error::none;
      // UARTS
    case hal::Peripheral::uart_a:
      hal::mmio::reset_bits(RCC->APBENR2, RCC_APBENR2_USART1EN);
      return hal::Error::none;
    case hal::Peripheral::uart_b:
      hal::mmio::reset_bits(RCC->APBENR1, RCC_APBENR1_USART2EN);
      return hal::Error::none;
    default:
      return hal::Error::invalid_param;
    }
  }

  /// reset a peripheral
  hal::Error reset(hal::Peripheral p) noexcept {
    // TODO: CAN and I2S
    switch (p) {
    case hal::Peripheral::crc:
      RCC->AHBRSTR |= RCC_AHBRSTR_CRCRST;
      return hal::Error::none;
    case hal::Peripheral::pwr:
      RCC->APBRSTR1 |= RCC_APBRSTR1_PWRRST;
      return hal::Error::none;
    case hal::Peripheral::dbg:
      RCC->APBRSTR1 |= RCC_APBRSTR1_DBGRST;
      return hal::Error::none;
    case hal::Peripheral::flash:
      RCC->AHBRSTR |= RCC_AHBRSTR_FLASHRST;
      return hal::Error::none;
    case hal::Peripheral::dma:
      RCC->AHBRSTR |= RCC_AHBRSTR_DMA1RST;
      return hal::Error::none;
    // ADC
    case hal::Peripheral::adc_a:
      [[fallthrough]];
    case hal::Peripheral::adc_all:
      RCC->APBRSTR2 |= RCC_APBRSTR2_ADCRST;
      return hal::Error::none;
    // GPIO
    case hal::Peripheral::gpio_a:
      RCC->IOPRSTR |= RCC_IOPRSTR_GPIOARST;
      return hal::Error::none;
    case hal::Peripheral::gpio_b:
      RCC->IOPRSTR |= RCC_IOPRSTR_GPIOBRST;
      return hal::Error::none;
    case hal::Peripheral::gpio_c:
      RCC->IOPRSTR |= RCC_IOPRSTR_GPIOCRST;
      return hal::Error::none;
    case hal::Peripheral::gpio_d:
      RCC->IOPRSTR |= RCC_IOPRSTR_GPIODRST;
      return hal::Error::none;
    case hal::Peripheral::gpio_f:
      RCC->IOPRSTR |= RCC_IOPRSTR_GPIOFRST;
      return hal::Error::none;
    case hal::Peripheral::gpio_all:
      RCC->IOPRSTR |= RCC_IOPRSTR_GPIOARST | RCC_IOPRSTR_GPIOBRST |
                      RCC_IOPRSTR_GPIOCRST | RCC_IOPRSTR_GPIODRST |
                      RCC_IOPRSTR_GPIOFRST;
      return hal::Error::none;
    // I2C
    case hal::Peripheral::i2c_a:
      [[fallthrough]];
    case hal::Peripheral::i2c_all:
      RCC->APBRSTR1 |= RCC_APBRSTR1_I2C1RST;
      return hal::Error::none;
    // SPI
    case hal::Peripheral::spi_a:
    case hal::Peripheral::spi_all:
      RCC->APBRSTR2 |= RCC_APBRSTR2_SPI1RST;
      return hal::Error::none;
    // TIMERS
    case hal::Peripheral::tim_a:
      RCC->APBRSTR2 |= RCC_APBRSTR2_TIM1RST;
      return hal::Error::none;
    case hal::Peripheral::tim_c:
      RCC->APBRSTR1 |= RCC_APBRSTR1_TIM2RST;
      return hal::Error::none;
    case hal::Peripheral::tim_n:
      RCC->APBRSTR2 |= RCC_APBRSTR2_TIM14RST;
      return hal::Error::none;
    case hal::Peripheral::tim_p:
      RCC->APBRSTR2 |= RCC_APBRSTR2_TIM16RST;
      return hal::Error::none;
    case hal::Peripheral::tim_q:
      RCC->APBRSTR2 |= RCC_APBRSTR2_TIM17RST;
      return hal::Error::none;
    // UARTS
    case hal::Peripheral::uart_a:
      RCC->APBRSTR2 |= RCC_APBRSTR2_USART1RST;
      return hal::Error::none;
    case hal::Peripheral::uart_b:
      RCC->APBRSTR1 |= RCC_APBRSTR1_USART2RST;
      return hal::Error::none;
    default:
      return hal::Error::invalid_param;
    }
  }

  /**
   * @brief set the inpout clock for a peripheral
   *
   * @param p one of adc_a, i2c_a, uart_a, can_a, i2s_a
   * @param clock see the reference manual Section 6.2. Figure 9. for valid
   * input clocks
   */
  hal::Error set_clock(hal::Peripheral p, Clock clock) {
    switch (p) {
    case hal::Peripheral::adc_a:
      [[fallthrough]];
    case hal::Peripheral::adc_all:
      switch (clock) {
      case Clock::SYSCLK:
        hal::mmio::set(RCC->CCIPR, RCC_CCIPR_ADCSEL, 0u, RCC_CCIPR_ADCSEL_POS);
        break;
      case Clock::HSIKER:
        hal::mmio::set(RCC->CCIPR, RCC_CCIPR_ADCSEL, 0b10u,
                       RCC_CCIPR_ADCSEL_POS);
      default:
        return hal::Error::invalid_param;
      }
      return hal::Error::none;
    case hal::Peripheral::i2c_a:
      switch (clock) {
      case Clock::PCLK:
        hal::mmio::set(RCC->CCIPR, RCC_CCIPR_ADCSEL, 0u, RCC_CCIPR_ADCSEL_POS);
        break;
      case Clock::SYSCLK:
        hal::mmio::set(RCC->CCIPR, RCC_CCIPR_ADCSEL, 1u, RCC_CCIPR_ADCSEL_POS);
        break;
      case Clock::HSIKER:
        hal::mmio::set(RCC->CCIPR, RCC_CCIPR_ADCSEL, 0b10u,
                       RCC_CCIPR_ADCSEL_POS);
        break;
      default:
        return hal::Error::invalid_param;
      }
      return hal::Error::none;
    case hal::Peripheral::uart_a:
      switch (clock) {
      case Clock::PCLK:
        hal::mmio::set(RCC->CCIPR, RCC_CCIPR_ADCSEL, 0u, RCC_CCIPR_ADCSEL_POS);
        break;
      case Clock::SYSCLK:
        hal::mmio::set(RCC->CCIPR, RCC_CCIPR_ADCSEL, 1u, RCC_CCIPR_ADCSEL_POS);
        break;
      case Clock::HSIKER:
        hal::mmio::set(RCC->CCIPR, RCC_CCIPR_ADCSEL, 0b10u,
                       RCC_CCIPR_ADCSEL_POS);
        break;
      case Clock::LSE:
        hal::mmio::set(RCC->CCIPR, RCC_CCIPR_ADCSEL, 0b11u,
                       RCC_CCIPR_ADCSEL_POS);
        break;
      default:
        return hal::Error::invalid_param;
      }
      return hal::Error::none;
    default:
      return hal::Error::invalid_param;
    }
  }
} clock_tree;

} // namespace stm32c031xx
#endif
