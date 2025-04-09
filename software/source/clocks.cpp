#include "hal/clocks.hpp"
#include "cortex/common.hpp"
#include "hal/peripheral.hpp"

namespace hal::clock {

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
  RCC_CR_HSIDIV_POS = 1 << 11,
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

Error enable_irq(Clock clock) {
  switch (clock) {
  case Clock::HSI48:
    cm::volatile_set_bits<std::uint32_t>(RCC_CIER, RCC_CIER_HSIRDYIE);
    return Error::none;
  case Clock::HSIUSB48:
    cm::volatile_set_bits<std::uint32_t>(RCC_CIER, RCC_CIER_HSIUSB48RDYIE);
    return Error::none;
  case Clock::HSE:
    cm::volatile_set_bits<std::uint32_t>(RCC_CIER, RCC_CIER_HSERDYIE);
    return Error::none;
  case Clock::LSI:
    cm::volatile_set_bits<std::uint32_t>(RCC_CIER, RCC_CIER_LSIRDYIE);
    return Error::none;
  case Clock::LSE:
    cm::volatile_set_bits<std::uint32_t>(RCC_CIER, RCC_CIER_LSERDYIE);
    return Error::none;
  case Clock::HSISYS:
    cm::volatile_set_bits<std::uint32_t>(RCC_CIER, RCC_CIER_HSIRDYIE);
    return Error::none;
  default:
    return Error::invalid_parameter;
  }
}

Error disable_irq(Clock clock) {
  switch (clock) {
  case Clock::HSE:
    cm::volatile_reset_bits<std::uint32_t>(RCC_CIER, RCC_CIER_HSERDYIE);
    return Error::none;
  default:
    break;
  }
  return Error::invalid_parameter;
}

bool is_stable(Clock clock) {
  switch (clock) {
  case Clock::HSE:
    return cm::volatile_read_bits<std::uint32_t>(RCC_CR, RCC_CR_HSERDY) != 0;
  default:
    break;
  }
  return true;
}

Error enable(Clock clock) {
  switch (clock) {
  case Clock::HSE:
    cm::volatile_set_bits<std::uint32_t>(RCC_CR, RCC_CR_HSEON);
    while (cm::volatile_read_bits<std::uint32_t>(RCC_CR, RCC_CR_HSERDY) == 0) {
      // wait for HSE to be stable/ready
    }
    return Error::none;
  default:
    break;
  }
  return Error::invalid_parameter;
}
Error enable(Peripheral p) { return Error::operation_unsupported; }
Error disable(Clock clock) {
  switch (clock) {
  case Clock::HSE:
    cm::volatile_reset_bits<std::uint32_t>(RCC_CR, RCC_CR_HSEON);
    while (cm::volatile_read_bits<std::uint32_t>(RCC_CR, RCC_CR_HSERDY) != 0) {
      // wait for the clock disable
    }
    return Error::none;
  default:
    break;
  }
  return Error::invalid_parameter;
}

std::uint32_t frequency(Peripheral p) { return 0; }
} // namespace hal::clock
