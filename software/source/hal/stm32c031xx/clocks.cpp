#include "clocks.hpp"

namespace stm32c031xx {
hal::Error ClockTree::init(const ClockConfig &cfg) {
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
    hsisys = 48'000'000u / cfg.hsisys_div;
  } else {
    hsisys = 0;
  }

  // HSIKER setup
  if (cfg.hsiker_div == 0 or cfg.hsiker_div > 8)
    return hal::Error::invalid_param;
  CR |= cfg.hsiker_div << RCC_CR_HSIKERDIV_POS;
  if (cfg.hsiker_enabled) {
    CR |= RCC_CR_HSIKERON;
    hsiker = 48'000'000u / cfg.hsiker_div;
  } else {
    hsiker = 0u;
  }

  // LSE setup
  if (cfg.lse_enabled) {
    if (cfg.lse < 32'000u or cfg.lse > 1'000'000)
      return hal::Error::invalid_param;
    lse = cfg.lse;
    CSR1 |= RCC_CSR1_LSEON;
  }
  if (cfg.lse_bypass)
    CSR1 |= RCC_CSR1_LSEBYP;

  // HSE setup
  if (cfg.hse_enabled) {
    if (cfg.hse < 4'000'000 or cfg.hse > 48'000'000)
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
    rtc = 0u;
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
  RCC->CSR1 = CSR1;
  RCC->CSR2 = CSR2;
  return hal::Error::none;
}

void ClockTree::update() noexcept {
  const std::uint32_t cr = RCC->CR;
  const std::uint32_t cfgr = RCC->CFGR;
  const std::uint32_t ccipr = RCC->CCIPR;
  const std::uint32_t csr1 = RCC->CSR1;

  if ((cr & RCC_CR_HSIUSB48RDY) == 0)
    hsi_usb = 0u;

  if ((cr & RCC_CR_HSERDY) == 0)
    hse = 0u;

  if ((cr & RCC_CR_HSIRDY) == 0)
    hsi = 0u;

  if ((csr1 & RCC_CSR1_LSERDY) == 0)
    lsi = 0u;

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
  const std::uint32_t hsisys_ = 48'000'000u >> hsidiv;
  const std::uint32_t sysclk_ = [&]() -> std::uint32_t {
    switch ((cfgr & RCC_CFGR_SWS) >> RCC_CFGR_SWS_POS) {
    case 0:
      return hsisys_ >> sysdiv;
    case 1:
      return hse / (1u << sysdiv);
    case 2:
      return 48'000'000u >> sysdiv;
    case 3:
      return 32'000 >> sysdiv;
    case 4:
      return lse / (1u << sysdiv);
    default:
      break;
    }
    return 0;
  }();
  const std::uint32_t hsiker_ = 48'000'000u >> hsikerdiv;
  const std::uint32_t hclk_ = sysclk_ >> ahb_pre;
  const std::uint32_t hclk8_ = hclk_ / 8u;
  const std::uint32_t pclk_ = hclk_ >> apb_pre;
  const std::uint32_t timpclk_ = apb_pre == 0 ? pclk_ : 2 * pclk_;
  const std::uint32_t rtc_ = [&]() -> std::uint32_t {
    if (csr1 & RCC_CSR1_RTCEN)
      switch ((csr1 & RCC_CSR1_RTCSEL) >> RCC_CSR1_RTCSEL_POS) {
      case 0:
        return 0;
      case 1:
        return lse / (1u << sysdiv);
      case 2:
        return 32'000u;
      case 3:
        return hse / 32;
      default:
        break;
      }
    return 0;
  }();
  const std::uint32_t usart1_ = [&]() -> std::uint32_t {
    switch ((ccipr & RCC_CCIPR_USART1SEL) >> RCC_CCIPR_USART1SEL_POS) {
    case 0:
      return pclk_;
    case 1:
      return sysclk_;
    case 2:
      return hsiker_;
    case 3:
      return lse;
    default:
      break;
    }
    return 0;
  }();
  const std::uint32_t fdcan1_ = [&]() -> std::uint32_t {
    switch ((ccipr & RCC_CCIPR_FDCAN1SEL) >> RCC_CCIPR_FDCAN1SEL_POS) {
    case 0:
      return pclk_;
    case 1:
      return hsiker_;
    case 2:
      return hse;
    default:
      break;
    }
    return 0;
  }();
  const std::uint32_t i2c1_ = [&]() -> std::uint32_t {
    switch ((ccipr & RCC_CCIPR_I2C1SEL) >> RCC_CCIPR_I2C1SEL_POS) {
    case 0:
      return pclk_;
    case 1:
      return sysclk_;
    case 2:
      return hsiker_;
    default:
      break;
    }
    return 0;
  }();
  const std::uint32_t i2s1_ = [&]() -> std::uint32_t {
    switch ((ccipr & RCC_CCIPR_I2S1SEL) >> RCC_CCIPR_I2S1SEL_POS) {
    case 0:
      return sysclk_;
    case 2:
      return hsiker_;
    case 3:
      return i2s_ckin;
    default:
      break;
    }
    return 0;
  }();
  const std::uint32_t adc_ = [&ccipr, &sysclk_, &hsiker_]() -> std::uint32_t {
    switch ((ccipr & RCC_CCIPR_ADCSEL) >> RCC_CCIPR_ADCSEL_POS) {
    case 0:
      return sysclk_;
    case 2:
      return hsiker_;
    default:
      break;
    }
    return 0;
  }();

  this->hsisys = hsisys_;
  this->sysclk = sysclk_;
  this->hclk = hclk_;
  this->hclk8 = hclk8_;
  this->pclk = pclk_;
  this->timpclk = timpclk_;
  this->rtc = rtc_;
  this->usart1 = usart1_;
  this->i2c1 = i2c1_;
  this->adc = adc_;
  this->i2s1 = i2s1_;
  this->fdcan1 = fdcan1_;
}

/**
 * @brief set the system clock to an internal source
 *
 * @param source an internal clock source
 */
hal::Error ClockTree::set_system_clock(hal::clock::Source source,
                                       unsigned divider) noexcept {
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
hal::Error ClockTree::set_system_clock(hal::clock::Source source,
                                       std::uint32_t f,
                                       unsigned divider) noexcept {
  using enum hal::clock::Source;
  switch (source) {
  case low_speed_internal:
    [[fallthrough]];
  case high_speed_internal:
    return set_system_clock(source, divider, 1);
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

std::uint32_t ClockTree::system_frequency() const noexcept { return sysclk; }

/**
 * @brief set the divider a a clock
 * @param clock one of HSISYS, HSIKER, SYSCLK, HCLK, PCLK
 * @param divider the division ratio
 */
hal::Error ClockTree::set_divider(Clock clock, unsigned divider) {
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
std::uint32_t ClockTree::frequency(hal::Peripheral p) const noexcept {
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
    return 0;
  }
}

/// enable the clock for a peripheral
hal::Error ClockTree::enable(hal::Peripheral p) noexcept {
  std::uint32_t tmp;
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
    tmp = hal::mmio::get(RCC->IOPENR, RCC_IOPENR_GPIOAEN);
    (void)tmp;
    return hal::Error::none;
  case hal::Peripheral::gpio_b:
    hal::mmio::set_bits(RCC->IOPENR, RCC_IOPENR_GPIOBEN);
    tmp = hal::mmio::get(RCC->IOPENR, RCC_IOPENR_GPIOBEN);
    (void)tmp;
    return hal::Error::none;
  case hal::Peripheral::gpio_c:
    hal::mmio::set_bits(RCC->IOPENR, RCC_IOPENR_GPIOCEN);
    tmp = hal::mmio::get(RCC->IOPENR, RCC_IOPENR_GPIOBEN);
    (void)tmp;
    return hal::Error::none;
  case hal::Peripheral::gpio_d:
    hal::mmio::set_bits(RCC->IOPENR, RCC_IOPENR_GPIODEN);
    tmp = hal::mmio::get(RCC->IOPENR, RCC_IOPENR_GPIODEN);
    (void)tmp;
    return hal::Error::none;
  case hal::Peripheral::gpio_f:
    hal::mmio::set_bits(RCC->IOPENR, RCC_IOPENR_GPIOFEN);
    tmp = hal::mmio::get(RCC->IOPENR, RCC_IOPENR_GPIOFEN);
    (void)tmp;
    return hal::Error::none;
  case hal::Peripheral::gpio_all:
    hal::mmio::set_bits(RCC->IOPENR, RCC_IOPENR_GPIOAEN | RCC_IOPENR_GPIOBEN |
                                         RCC_IOPENR_GPIOCEN |
                                         RCC_IOPENR_GPIODEN |
                                         RCC_IOPENR_GPIOFEN);
    tmp = hal::mmio::get(RCC->IOPENR, RCC_IOPENR_GPIOAEN | RCC_IOPENR_GPIOBEN |
                                          RCC_IOPENR_GPIOCEN |
                                          RCC_IOPENR_GPIODEN |
                                          RCC_IOPENR_GPIOFEN);
    (void)tmp;
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
    tmp = hal::mmio::get(RCC->APBENR2, RCC_APBENR2_USART1EN);
    (void)tmp;
    return hal::Error::none;
  case hal::Peripheral::uart_b:
    hal::mmio::set_bits(RCC->APBENR1, RCC_APBENR1_USART2EN);
    tmp = hal::mmio::get(RCC->APBENR1, RCC_APBENR1_USART2EN);
    (void)tmp;
    return hal::Error::none;
  default:
    return hal::Error::invalid_param;
  }
}

/// disable a peripherals clock
hal::Error ClockTree::disable(hal::Peripheral p) noexcept {
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
    hal::mmio::reset_bits(RCC->IOPENR, RCC_IOPENR_GPIOAEN | RCC_IOPENR_GPIOBEN |
                                           RCC_IOPENR_GPIOCEN |
                                           RCC_IOPENR_GPIODEN |
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
hal::Error ClockTree::reset(hal::Peripheral p) noexcept {
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
hal::Error ClockTree::set_clock(hal::Peripheral p, Clock clock) {
  switch (p) {
  case hal::Peripheral::adc_a:
    [[fallthrough]];
  case hal::Peripheral::adc_all:
    switch (clock) {
    case Clock::SYSCLK:
      hal::mmio::set(RCC->CCIPR, RCC_CCIPR_ADCSEL, 0u, RCC_CCIPR_ADCSEL_POS);
      break;
    case Clock::HSIKER:
      hal::mmio::set(RCC->CCIPR, RCC_CCIPR_ADCSEL, 0b10u, RCC_CCIPR_ADCSEL_POS);
      break;
    default:
      return hal::Error::invalid_param;
    }
    return hal::Error::none;
  case hal::Peripheral::i2c_a:
    switch (clock) {
    case Clock::PCLK:
      hal::mmio::set(RCC->CCIPR, RCC_CCIPR_I2C1SEL, 0u, RCC_CCIPR_I2C1SEL_POS);
      break;
    case Clock::SYSCLK:
      hal::mmio::set(RCC->CCIPR, RCC_CCIPR_I2C1SEL, 1u, RCC_CCIPR_I2C1SEL_POS);
      break;
    case Clock::HSIKER:
      hal::mmio::set(RCC->CCIPR, RCC_CCIPR_I2C1SEL, 0b10u,
                     RCC_CCIPR_I2C1SEL_POS);
      break;
    default:
      return hal::Error::invalid_param;
    }
    return hal::Error::none;
  case hal::Peripheral::uart_a:
    switch (clock) {
    case Clock::PCLK:
      hal::mmio::set(RCC->CCIPR, RCC_CCIPR_USART1SEL, 0u,
                     RCC_CCIPR_USART1SEL_POS);
      break;
    case Clock::SYSCLK:
      hal::mmio::set(RCC->CCIPR, RCC_CCIPR_USART1SEL, 1u,
                     RCC_CCIPR_USART1SEL_POS);
      break;
    case Clock::HSIKER:
      hal::mmio::set(RCC->CCIPR, RCC_CCIPR_USART1SEL, 0b10u,
                     RCC_CCIPR_USART1SEL_POS);
      break;
    case Clock::LSE:
      hal::mmio::set(RCC->CCIPR, RCC_CCIPR_USART1SEL, 0b11u,
                     RCC_CCIPR_USART1SEL_POS);
      break;
    default:
      return hal::Error::invalid_param;
    }
    return hal::Error::none;
  default:
    return hal::Error::invalid_param;
  }
  return hal::Error::none;
}

} // namespace stm32c031xx
