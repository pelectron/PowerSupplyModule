#ifndef HAL_STM32C031XX_CLOCKS_HPP
#define HAL_STM32C031XX_CLOCKS_HPP

#include "hal/config.hpp"
#include "hal/enums.hpp"
#include "hal/mmio.hpp"

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
  RCC_CFGR_MCOPRE = 0xFu << 28u,
  RCC_CFGR_MCOPRE_POS = 28u,
  RCC_CFGR_MCOSEL = 0xFu << 24u,
  RCC_CFGR_MCOSEL_POS = 24u,
  RCC_CFGR_MCO2PRE = 0xFu << 20u,
  RCC_CFGR_MCO2PRE_POS = 20u,
  RCC_CFGR_MCO2SEL = 0xF << 16u,
  RCC_CFGR_MCO2SEL_POS = 16u,
  RCC_CFGR_PPRE = 0b111u << 12u,
  RCC_CFGR_PPRE_POS = 12u,
  RCC_CFGR_HPRE = 0xFu << 8u,
  RCC_CFGR_HPRE_POS = 8u,
  RCC_CFGR_SWS = 0b111u << 3u,
  RCC_CFGR_SWS_POS = 3u,
  RCC_CFGR_SW = 0b111u,
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
  RCC_CCIPR_ADCSEL = 0b11u << 30u,
  RCC_CCIPR_ADCSEL_POS = 30u,
  RCC_CCIPR_I2S1SEL = 0b11u << 14u,
  RCC_CCIPR_I2S1SEL_POS = 14u,
  RCC_CCIPR_I2C1SEL = 0b11u << 12u,
  RCC_CCIPR_I2C1SEL_POS = 12u,
  RCC_CCIPR_FDCAN1SEL = 0b11u << 8u,
  RCC_CCIPR_FDCAN1SEL_POS = 8u,
  RCC_CCIPR_USART1SEL = 0b11u,
  RCC_CCIPR_USART1SEL_POS = 0u,
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
  RCC_CSR2_LPWRRSTF = 1u << 31,
  RCC_CSR2_WWDGRSTF = 1 << 30,
  RCC_CSR2_IWDGRSTF = 1 << 29,
  RCC_CSR2_SFTRSTF = 1 << 28,
  RCC_CSR2_PWRRSTF = 1 << 27,
  RCC_CSR2_PINRSTF = 1 << 26,
  RCC_CSR2_OBLRSTF = 1 << 25,
  RCC_CSR2_RMVF = 1 << 23,
  RCC_CSR2_LSIRDY = 1 << 1,
  RCC_CSR2_LSION = 1 << 0
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
  uint32_t reserved0[3];
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
  std::uint32_t reserved2;
  volatile std::uint32_t CSR1;
  volatile std::uint32_t CSR2;
};

struct ClockConfig {
  std::uint32_t hse = 0u;
  std::uint32_t lse = 0u;
  std::uint32_t i2s_ckin = 0u;
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
  bool usb_enabled = false;
  bool css_enabled = false;
  bool lse_bypass = false;
  bool lse_enabled = false;
  bool hse_bypass = false;
  bool hse_enabled = false;
  bool hsiker_enabled = false;
  bool hsi_enabled = true;
};

struct ClockTree {
  std::uint32_t hse = 0u;
  std::uint32_t lse = 0u;
  std::uint32_t i2s_ckin = 0u;
  std::uint32_t hsi = 48'000'000;
  std::uint32_t hsi_usb = 48'000'000;
  std::uint32_t hsiker = 48'000'000 / 3;
  std::uint32_t lsi = 32'000;
  std::uint32_t hsisys = 12'000'000;
  std::uint32_t sysclk = 12'000'000;
  std::uint32_t hclk = 12'000'000;
  std::uint32_t hclk8 = 12'000'000 / 8;
  std::uint32_t pclk = 12'000'000;
  std::uint32_t timpclk = 12'000'000;
  std::uint32_t rtc = 0u;
  std::uint32_t usart1 = 12'000'000;
  std::uint32_t i2c1 = 12'000'000;
  std::uint32_t adc = 12'000'000;
  std::uint32_t i2s1 = 12'000'000;
  std::uint32_t fdcan1 = 12'000'000;
  void init() {
    // enable hsi
    hal::mmio::set_bits(RCC->CR, RCC_CR_HSION);
    while (hal::mmio::get(RCC->CR, RCC_CR_HSIRDY) == 0) {
    }

    // set hsidiv 1
    hal::mmio::reset_bits(RCC->CR, RCC_CR_HSIDIV);
    // set hsikerdiv to 1
    hal::mmio::reset_bits(RCC->CR, RCC_CR_HSIKERDIV);

    // enable lsi
    // hal::mmio::set_bits(RCC->CSR2, RCC_CSR2_LSION);
    // while (hal::mmio::get(RCC->CSR2, RCC_CSR2_LSIRDY) == 0) {
    // }

    // set ahb prescaler to 1
    hal::mmio::reset_bits(RCC->CR, RCC_CFGR_HPRE);

    // set system clock source to hsi
    hal::mmio::reset_bits(RCC->CR, RCC_CFGR_SW);
    while (hal::mmio::get(RCC->CFGR, RCC_CFGR_SWS, RCC_CFGR_SWS_POS) != 0) {
    }

    // set apb prescaler to 1
    hal::mmio::reset_bits(RCC->CFGR, RCC_CFGR_PPRE);

    // enable syscfg clock
    hal::mmio::set_bits(RCC->APBENR2, RCC_APBENR2_SYSCFGEN);

    // enable pwr clock
    hal::mmio::set_bits(RCC->APBENR1, RCC_APBENR1_PWREN);

    const std::uint32_t cr = RCC->CR;
    const std::uint32_t csr1 = RCC->CSR1;

    const std::uint32_t ahb_pre = 0;
    const std::uint32_t apb_pre = 0;
    // dividers
    const std::uint32_t hsidiv = 0;
    const std::uint32_t hsikerdiv = 0;
    const std::uint32_t sysdiv = 0;

    // clock frequencies
    hsisys = 48'000'000u >> hsidiv;
    sysclk = hsisys >> sysdiv;
    hsiker = 48'000'000u >> hsikerdiv;
    hclk = sysclk >> ahb_pre;
    hclk8 = hclk / 8u;
    pclk = hclk >> apb_pre;
    timpclk = apb_pre == 0 ? pclk : 2 * pclk;
    rtc = 0;
    usart1 = pclk;
    fdcan1 = 0;
    i2c1 = pclk;
    i2s1 = sysclk;
    adc = sysclk;
    hsi_usb = 0u;
    hse = 0u;
    lsi = 0u;
  }

  hal::Error init(const ClockConfig &cfg);
  void update() noexcept;

  /**
   * @brief set the system clock to an internal source
   *
   * @param source an internal clock source
   */
  hal::Error set_system_clock(hal::clock::Source source,
                              unsigned divider = 1) noexcept;
  /**
   * @brief set the system clock to an external source
   *
   * @param source an external clock source
   */
  hal::Error set_system_clock(hal::clock::Source source, std::uint32_t f,
                              unsigned divider = 1) noexcept;
  std::uint32_t system_frequency() const noexcept;

  /**
   * @brief set the divider a a clock
   * @param clock one of HSISYS, HSIKER, SYSCLK, HCLK, PCLK
   * @param divider the division ratio
   */
  hal::Error set_divider(Clock clock, unsigned divider);
  /// get the clock frequency of a peripheral.
  /// returns 0 Hz if the peripheral does not exist or the its clock is not
  /// enabled.
  std::uint32_t frequency(hal::Peripheral p) const noexcept;
  /// enable the clock for a peripheral
  hal::Error enable(hal::Peripheral p) noexcept;
  /// disable a peripherals clock
  hal::Error disable(hal::Peripheral p) noexcept;
  /// reset a peripheral
  hal::Error reset(hal::Peripheral p) noexcept;
  /**
   * @brief set the inpout clock for a peripheral
   *
   * @param p one of adc_a, i2c_a, uart_a, can_a, i2s_a
   * @param clock see the reference manual Section 6.2. Figure 9. for valid
   * input clocks
   */
  hal::Error set_clock(hal::Peripheral p, Clock clock);
}; // clock_tree;

ClockTree &clock_tree();

} // namespace stm32c031xx
#endif
