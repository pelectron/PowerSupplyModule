#include "hal/adc.hpp"
#include "clocks.hpp"
#include "hal/config.hpp"
#include "hal/enums.hpp"
#include "hal/mmio.hpp"
#include "tl/expected.hpp"
#include <cstdint>

#define ADC1 reinterpret_cast<::stm32c031xx::Adc *>(0x40012400u)

namespace stm32c031xx {

enum ADC_ISR : std::uint32_t {
  ADC_ISR_CCRDY = 1u << 13u,
  ADC_ISR_EOCAL = 1u << 11u,
  ADC_ISR_AWD3 = 1u << 9u,
  ADC_ISR_AWD2 = 1u << 8u,
  ADC_ISR_AWD1 = 1u << 7u,
  ADC_ISR_OVR = 1u << 4u,
  ADC_ISR_EOS = 1u << 3u,
  ADC_ISR_EOC = 1u << 2u,
  ADC_ISR_EOSMP = 1u << 1u,
  ADC_ISR_ADRDY = 1u << 0u,
};

enum ADC_IER : std::uint32_t {
  ADC_IER_CCRDYIE = 1u << 13u,
  ADC_IER_EOCALIE = 1u << 11u,
  ADC_IER_AWD3IE = 1u << 9u,
  ADC_IER_AWD2IE = 1u << 8u,
  ADC_IER_AWD1IE = 1u << 7u,
  ADC_IER_OVRIE = 1u << 4u,
  ADC_IER_EOSIE = 1u << 3u,
  ADC_IER_EOCIE = 1u << 2u,
  ADC_IER_EOSMIE = 1u << 1u,
  ADC_IER_ADRDYIE = 1u << 0u,
};

enum ADC_CR : std::uint32_t {
  ADC_CR_ADCAL = 1u << 31u,
  ADC_CR_ADVREGEN = 1u << 28u,
  ADC_CR_ADSTP = 1u << 4u,
  ADC_CR_ADSTART = 1u << 2u,
  ADC_CR_ADDIS = 1u << 1u,
  ADC_CR_ADEN = 1u << 0u,
};

enum ADC_CFGR1 : std::uint32_t {
  ADC_CFGR1_AWD1CH = 0x1Fu << 26,
  ADC_CFGR1_AD1CH_POS = 26,
  ADC_CFGR1_AWD1EN = 1u << 23u,
  ADC_CFGR1_AWD1SGL = 1u << 22u,
  ADC_CFGR1_CHSELRMOD = 1u << 21u,
  ADC_CFGR1_DISCEN = 1u << 16u,
  ADC_CFGR1_AUTOFF = 1u << 15u,
  ADC_CFGR1_WAIT = 1u << 14u,
  ADC_CFGR1_CONT = 1u << 13u,
  ADC_CFGR1_OVRMOD = 1 << 12u,
  ADC_CFGR1_EXTEN = 0b11u << 10,
  ADC_CFGR1_EXTEN_POS = 10u,
  ADC_CFGR1_EXTSEL = 0b111u << 6,
  ADC_CFGR1_EXTSEL_POS = 6,
  ADC_CFGR1_ALIGN = 1u << 5u,
  ADC_CFGR1_RES = 0b11u << 3,
  ADC_CFGR1_RES_POS = 3u,
  ADC_CFGR1_SCANDIR = 1u << 2u,
  ADC_CFGR1_DMACFG = 1u << 1u,
  ADC_CFGR1_DMAEN = 1u << 0u
};

enum ADC_CFGR2 : std::uint32_t {
  ADC_CFGR2_CKMODE = 0b11u << 30,
  ADC_CFGR2_CKMODE_POS = 30u,
  ADC_CFGR2_LFTRIG = 1u << 29u,
  ADC_CFGR2_TOVS = 1u << 9u,
  ADC_CFGR2_OVSS = 0xFu << 5u,
  ADC_CFGR2_OVSS_POS = 5u,
  ADC_CFGR2_OVSR = 0b111u << 2u,
  ADC_CFGR2_OVSR_POS = 2u,
  ADC_CFGR2_OVSE = 1u << 0u
};

enum ADC_SMPR : std::uint32_t {
  ADC_SMPR_SMPSEL22 = 1u << 30u,
  ADC_SMPR_SMPSEL21 = 1u << 29u,
  ADC_SMPR_SMPSEL20 = 1u << 28u,
  ADC_SMPR_SMPSEL19 = 1u << 27u,
  ADC_SMPR_SMPSEL18 = 1u << 26u,
  ADC_SMPR_SMPSEL17 = 1u << 25u,
  ADC_SMPR_SMPSEL16 = 1u << 24u,
  ADC_SMPR_SMPSEL15 = 1u << 23u,
  ADC_SMPR_SMPSEL14 = 1u << 22u,
  ADC_SMPR_SMPSEL13 = 1u << 21u,
  ADC_SMPR_SMPSEL12 = 1u << 20u,
  ADC_SMPR_SMPSEL11 = 1u << 19u,
  ADC_SMPR_SMPSEL10 = 1u << 18u,
  ADC_SMPR_SMPSEL9 = 1u << 17u,
  ADC_SMPR_SMPSEL8 = 1u << 16u,
  ADC_SMPR_SMPSEL7 = 1u << 15u,
  ADC_SMPR_SMPSEL6 = 1u << 14u,
  ADC_SMPR_SMPSEL5 = 1u << 13u,
  ADC_SMPR_SMPSEL4 = 1u << 12u,
  ADC_SMPR_SMPSEL3 = 1u << 11u,
  ADC_SMPR_SMPSEL2 = 1u << 10u,
  ADC_SMPR_SMPSEL1 = 1u << 9u,
  ADC_SMPR_SMPSEL0 = 1u << 8u,
  ADC_SMPR_SMP2 = 0b111u << 4u,
  ADC_SMPR_SMP2_POS = 4u,
  ADC_SMPR_SMP1 = 0b111u,
  ADC_SMPR_SMP1_POS = 0u
};

enum ADC_AWDXTR : std::uint32_t {
  ADC_AWDXTR_HT = 0x7FFu << 16u,
  ADC_AWDXTR_HT_POS = 16u,
  ADC_AWDXTR_LT = 0x7FFu << 0u,
  ADC_AWDXTR_LT_POS = 0u
};

enum ADC_CHSELR : std::uint32_t {
  // CHSELRMOD = 0 in ADC_CFGR1
  ADC_CHSELR_CHSEL22 = 1u << 22u,
  ADC_CHSELR_CHSEL21 = 1u << 21u,
  ADC_CHSELR_CHSEL20 = 1u << 20u,
  ADC_CHSELR_CHSEL19 = 1u << 19u,
  ADC_CHSELR_CHSEL18 = 1u << 18u,
  ADC_CHSELR_CHSEL17 = 1u << 17u,
  ADC_CHSELR_CHSEL16 = 1u << 16u,
  ADC_CHSELR_CHSEL15 = 1u << 15u,
  ADC_CHSELR_CHSEL14 = 1u << 14u,
  ADC_CHSELR_CHSEL13 = 1u << 13u,
  ADC_CHSELR_CHSEL12 = 1u << 12u,
  ADC_CHSELR_CHSEL11 = 1u << 11u,
  ADC_CHSELR_CHSEL10 = 1u << 10u,
  ADC_CHSELR_CHSEL9 = 1u << 9u,
  ADC_CHSELR_CHSEL8 = 1u << 8u,
  ADC_CHSELR_CHSEL7 = 1u << 7u,
  ADC_CHSELR_CHSEL6 = 1u << 6u,
  ADC_CHSELR_CHSEL5 = 1u << 5u,
  ADC_CHSELR_CHSEL4 = 1u << 4u,
  ADC_CHSELR_CHSEL3 = 1u << 3u,
  ADC_CHSELR_CHSEL2 = 1u << 2u,
  ADC_CHSELR_CHSEL1 = 1u << 1u,
  ADC_CHSELR_CHSEL0 = 1u << 0u,
  // CHSELRMOD = 1 in ADC_CFGR1
  ADC_CHSELR_SQ8 = 0xFFu << 28u,
  ADC_CHSELR_SQ8_POS = 28u,
  ADC_CHSELR_SQ7 = 0xFFu << 24u,
  ADC_CHSELR_SQ7_POS = 24u,
  ADC_CHSELR_SQ6 = 0xFFu << 20u,
  ADC_CHSELR_SQ6_POS = 20u,
  ADC_CHSELR_SQ5 = 0xFFu << 16u,
  ADC_CHSELR_SQ5_POS = 16u,
  ADC_CHSELR_SQ4 = 0xFFu << 12u,
  ADC_CHSELR_SQ4_POS = 12u,
  ADC_CHSELR_SQ3 = 0xFFu << 8u,
  ADC_CHSELR_SQ3_POS = 8u,
  ADC_CHSELR_SQ2 = 0xFFu << 4u,
  ADC_CHSELR_SQ2_POS = 4u,
  ADC_CHSELR_SQ1 = 0xFFu << 0u,
  ADC_CHSELR_SQ1_POS = 0u,
};

enum ADC_AWD2CR : std::uint32_t {
  ADC_AWD2CR_CH22 = 1u << 22u,
  ADC_AWD2CR_CH21 = 1u << 21u,
  ADC_AWD2CR_CH20 = 1u << 20u,
  ADC_AWD2CR_CH19 = 1u << 19u,
  ADC_AWD2CR_CH18 = 1u << 18u,
  ADC_AWD2CR_CH17 = 1u << 17u,
  ADC_AWD2CR_CH16 = 1u << 16u,
  ADC_AWD2CR_CH15 = 1u << 15u,
  ADC_AWD2CR_CH14 = 1u << 14u,
  ADC_AWD2CR_CH13 = 1u << 13u,
  ADC_AWD2CR_CH12 = 1u << 12u,
  ADC_AWD2CR_CH11 = 1u << 11u,
  ADC_AWD2CR_CH10 = 1u << 10u,
  ADC_AWD2CR_CH9 = 1u << 9u,
  ADC_AWD2CR_CH8 = 1u << 8u,
  ADC_AWD2CR_CH7 = 1u << 7u,
  ADC_AWD2CR_CH6 = 1u << 6u,
  ADC_AWD2CR_CH5 = 1u << 5u,
  ADC_AWD2CR_CH4 = 1u << 4u,
  ADC_AWD2CR_CH3 = 1u << 3u,
  ADC_AWD2CR_CH2 = 1u << 2u,
  ADC_AWD2CR_CH1 = 1u << 1u,
  ADC_AWD2CR_CH0 = 1u << 0u,
};

enum ADC_AWD3CR : std::uint32_t {
  ADC_AWD3CR_CH22 = 1u << 22u,
  ADC_AWD3CR_CH21 = 1u << 21u,
  ADC_AWD3CR_CH20 = 1u << 20u,
  ADC_AWD3CR_CH19 = 1u << 19u,
  ADC_AWD3CR_CH18 = 1u << 18u,
  ADC_AWD3CR_CH17 = 1u << 17u,
  ADC_AWD3CR_CH16 = 1u << 16u,
  ADC_AWD3CR_CH15 = 1u << 15u,
  ADC_AWD3CR_CH14 = 1u << 14u,
  ADC_AWD3CR_CH13 = 1u << 13u,
  ADC_AWD3CR_CH12 = 1u << 12u,
  ADC_AWD3CR_CH11 = 1u << 11u,
  ADC_AWD3CR_CH10 = 1u << 10u,
  ADC_AWD3CR_CH9 = 1u << 9u,
  ADC_AWD3CR_CH8 = 1u << 8u,
  ADC_AWD3CR_CH7 = 1u << 7u,
  ADC_AWD3CR_CH6 = 1u << 6u,
  ADC_AWD3CR_CH5 = 1u << 5u,
  ADC_AWD3CR_CH4 = 1u << 4u,
  ADC_AWD3CR_CH3 = 1u << 3u,
  ADC_AWD3CR_CH2 = 1u << 2u,
  ADC_AWD3CR_CH1 = 1u << 1u,
  ADC_AWD3CR_CH0 = 1u << 0u,
};

enum ADC_CALFACT : std::uint32_t {
  ADC_CALFACT_CALFACT = 0x3Fu,
  ADC_CALFACT_CALFACT_POS = 0
};

enum ADC_CCR : std::uint32_t {
  ADC_CCR_TSEN = 1u << 23u,
  ADC_CCR_VREFEN = 1u << 22u,
  ADC_CCR_PRESC = 0xFFu << 18u,
  ADC_CCR_PRESC_POS = 18u
};

/**
 * @brief Analog to Digital Converter
 */
struct ADC {
  volatile std::uint32_t
      ISR; /*!< ADC interrupt and status register, Address offset: 0x00 */
  volatile std::uint32_t
      IER; /*!< ADC interrupt enable register,     Address offset: 0x04 */
  volatile std::uint32_t
      CR; /*!< ADC control register,              Address offset: 0x08 */
  volatile std::uint32_t
      CFGR1; /*!< ADC configuration register 1,      Address offset: 0x0C */
  volatile std::uint32_t
      CFGR2; /*!< ADC configuration register 2,      Address offset: 0x10 */
  volatile std::uint32_t
      SMPR; /*!< ADC sampling time register,        Address offset: 0x14 */
  std::uint32_t RESERVED1;       /*!< Reserved, 0x18 */
  std::uint32_t RESERVED2;       /*!< Reserved, 0x1C */
  volatile std::uint32_t AWD1TR; /*!< ADC analog watchdog 1 threshold register,
                               Address offset: 0x20 */
  volatile std::uint32_t AWD2TR; /*!< ADC analog watchdog 2 threshold register,
                               Address offset: 0x24 */
  volatile std::uint32_t CHSELR; /*!< ADC group regular sequencer register,
                               Address offset: 0x28 */
  volatile std::uint32_t AWD3TR; /*!< ADC analog watchdog 3 threshold register,
                               Address offset: 0x2C */
  std::uint32_t RESERVED3[4];    /*!< Reserved, 0x30 - 0x3C */
  volatile std::uint32_t
      DR; /*!< ADC group regular data register, Address offset: 0x40 */
  std::uint32_t RESERVED4[23];   /*!< Reserved,   0x44 - 0x9C */
  volatile std::uint32_t AWD2CR; /*!< ADC analog watchdog 2 configuration
                               register, Address offset: 0xA0 */
  volatile std::uint32_t AWD3CR; /*!< ADC analog watchdog 3 configuration
                               register, Address offset: 0xA4 */
  std::uint32_t RESERVED5[3];    /*!< Reserved, 0xA8 - 0xB0 */
  volatile std::uint32_t
      CALFACT; /*!< ADC Calibration factor register, Address offset: 0xB4 */
  std::uint32_t RESERVED6[3]; /*!< Reserved, 0xA8 - 0xB0 */
  volatile std::uint32_t
      CCR; /*!< ADC Calibration factor register, Address offset: 0x308 */

  void calibrate() {
    // section 16.4.3 of the TRM:
    // ADC must be disabled
    hal::mmio::reset_bits(CR, ADC_CR_ADEN);
    // auto off mode and dma must be disabled
    hal::mmio::reset_bits(CFGR1, ADC_CFGR1_AUTOFF | ADC_CFGR1_DMAEN);

    // calfact is the calibration factor
    std::uint32_t calfact = 0;
    // averaging 8 calibration factor values
    for (std::size_t i = 0; i < 8; ++i) {
      // start the calibration
      hal::mmio::set_bits(CR, ADC_CR_ADCAL);
      // wait for calibration  to end
      while (hal::mmio::get(CR, ADC_CR_ADCAL) != 0) {
      }
      calfact += DR + 1;
    }

    if ((calfact % 8) > 4)
      calfact = calfact / 8 + 1;
    else
      calfact /= 8;

    if (calfact > 0x7Fu)
      calfact = 0x7Fu;

    // set the calibration factor
    hal::mmio::set(CALFACT, ADC_CALFACT_CALFACT, calfact,
                   ADC_CALFACT_CALFACT_POS);
  }

  static inline constinit std::span<std::int32_t> channel_values{};
  static inline constinit volatile std::uint8_t index = 0;
  static inline constinit hal::adc::CallbackTable *callbacks{nullptr};
  static inline constinit const hal::adc::Config *config{nullptr};

  hal::Error start(const hal::adc::Config &cfg,
                   hal::adc::CallbackTable &callbacks,
                   std::span<std::int32_t> buffer) {
    channel_values = buffer;
    ADC::callbacks = &callbacks;
    ADC::config = &cfg;
    index = 0;
    // enable interrupts
    hal::mmio::set_bits(IER, ADC_IER_ADRDYIE | ADC_IER_CCRDYIE |
                                 ADC_IER_EOCALIE | ADC_IER_EOSIE |
                                 ADC_IER_OVRIE);
    enable();

    ADC::callbacks->monitor(hal::adc::Event::started);
    // start a conversion
    hal::mmio::set_bits(CR, ADC_CR_ADSTART);
    while (hal::mmio::get(CR, ADC_CR_ADSTART)) {
    };
    if (cfg.has_option(hal::adc::Options::continous)) {
      return hal::Error::none;
    }
    if (cfg.has_option(hal::adc::Options::sequence_conversion)) {
      // sequence conversion
      while (hal::mmio::get(ISR, ADC_ISR_EOS) == 0) {
      }
    } else {
      // single conversion
      while (hal::mmio::get(ISR, ADC_ISR_EOC) == 0) {
      }
    }

    return hal::Error::none;
  }

  hal::Error stop() {
    // stop any ongoing conversion by writing 1 to the ADSTP bit in the ADC_CR
    // register and waiting until this bit is read at 0.
    hal::mmio::set(CR, ADC_CR_ADSTP);
    while (hal::mmio::get(CR, ADC_CR_ADSTP) != 0) {
    }
    hal::mmio::reset_bits(IER, ADC_IER_ADRDYIE | ADC_IER_CCRDYIE |
                                   ADC_IER_EOCALIE | ADC_IER_EOSIE |
                                   ADC_IER_OVRIE);
    callbacks->monitor(hal::adc::Event::stopped);
    callbacks->end_of_conversion(
        tl::unexpected(hal::Error::operation_cancelled));
    callbacks->end_of_sequence(tl::unexpected(hal::Error::operation_cancelled));
    callbacks = nullptr;
    config = nullptr;
    return hal::Error::none;
  }

  hal::Error trigger() {
    hal::mmio::set_bits(CR, ADC_CR_ADSTART);
    return hal::Error::none;
  }

  void enable_regulator() {
    hal::mmio::set_bits(CR, ADC_CR_ADVREGEN);
    //   while (hal::mmio::get(CR, ADC_CR_ADVREGEN) == 0) {
    //   }
  }

  void disable_regulator() {
    hal::mmio::reset_bits(CR, ADC_CR_ADEN);
    hal::mmio::reset_bits(CR, ADC_CR_ADVREGEN);
  }

  hal::Error enable() {
    enable_regulator();
    // set the enable bit
    hal::mmio::set_bits(CR, ADC_CR_ADEN);
    // wait for the adc to be ready
    while (hal::mmio::get(ISR, ADC_ISR_ADRDY) == 0) {
    }
    //  clear the ready bit in ADC_ISR register by programming this bit to 1
    hal::mmio::set_bits(ISR, ADC_ISR_ADRDY);
    return hal::Error::none;
  }

  bool is_enabled() const noexcept {
    return hal::mmio::get(CR, ADC_CR_ADEN) != 0;
  }

  hal::Error disable() {
    if (hal::mmio::get(CR, ADC_CR_ADSTART)) {
      // stop any ongoing conversion by writing 1 to the ADSTP bit in the ADC_CR
      // register and waiting until this bit is read at 0.
      hal::mmio::set(CR, ADC_CR_ADSTP);
      while (hal::mmio::get(CR, ADC_CR_ADSTP) != 0) {
      }
    }
    // set the disable bit
    hal::mmio::set_bits(CR, ADC_CR_ADDIS);
    // wait unitl enable and disable bit are cleared
    while (hal::mmio::get(CR, ADC_CR_ADEN) != 0) {
    }
    //  clear the ready bit in ADC_ISR register by programming this bit to 1
    hal::mmio::set_bits(ISR, ADC_ISR_ADRDY);
    disable_regulator();
    return hal::Error::none;
  }
};

} // namespace stm32c031xx

namespace hal::adc {

tl::expected<Adc, hal::ConfigError> configure(const Config &cfg) {
  using namespace stm32c031xx;
  if (cfg.id != Id::A) {
    return tl::unexpected(hal::ConfigError::invalid_id);
  }

  if (cfg.channels >= ChannelId::Channel23) {
    return tl::unexpected(hal::ConfigError::invalid_channel);
  }

  hal::mmio::set_bits(RCC->APBENR2, RCC_APBENR2_ADCEN);
  auto tmp = hal::mmio::get(RCC->APBENR2, RCC_APBENR2_ADCEN);
  (void)tmp;

  auto *adc = reinterpret_cast<stm32c031xx::ADC *>(0x40012400u);

  // if (adc->is_enabled())
  //   return tl::unexpected(hal::ConfigError::already_in_use);
  adc->disable();
  adc->enable_regulator();
  std::uint32_t CFGR1 = 0;
  std::uint32_t CFGR2 = 0;
  std::uint32_t CCR = 0;
  std::uint32_t CHSELR = 0;
  std::uint32_t SMPR = 0;
  std::uint32_t IER = 0;

  // set the resolution
  switch (cfg.num_bits) {
  case 6:
    hal::mmio::set(CFGR1, ADC_CFGR1_RES, 0b11u, ADC_CFGR1_RES_POS);
    break;
  case 8:
    hal::mmio::set(CFGR1, ADC_CFGR1_RES, 0b10u, ADC_CFGR1_RES_POS);
    break;
  case 10:
    hal::mmio::set(CFGR1, ADC_CFGR1_RES, 0b01u, ADC_CFGR1_RES_POS);
    break;
  case 12:
    hal::mmio::set(CFGR1, ADC_CFGR1_RES, 0b00u, ADC_CFGR1_RES_POS);
    break;
  default:
    return tl::unexpected(ConfigError::invalid_resolution);
  }

  // set the oversampling
  // the right shift is set to keep the original resolution
  // uses the low frequency trigger and trigered oversampling
  switch (cfg.oversampling) {
  case 0:
    [[fallthrough]];
  case 1:
    mmio::set(CFGR2, ADC_CFGR2_OVSR, 0u, ADC_CFGR2_OVSR_POS);
    mmio::set(CFGR2, ADC_CFGR2_OVSS, 0u, ADC_CFGR2_OVSS_POS);
    mmio::reset_bits(CFGR2, ADC_CFGR2_OVSE | ADC_CFGR2_TOVS);
    break;
  case 2:
    mmio::set(CFGR2, ADC_CFGR2_OVSR, 0u, ADC_CFGR2_OVSR_POS);
    mmio::set(CFGR2, ADC_CFGR2_OVSS, 1u, ADC_CFGR2_OVSS_POS);
    mmio::set_bits(CFGR2, ADC_CFGR2_OVSE | ADC_CFGR2_TOVS);
    break;
  case 4:
    mmio::set(CFGR2, ADC_CFGR2_OVSR, 1u, ADC_CFGR2_OVSR_POS);
    mmio::set(CFGR2, ADC_CFGR2_OVSS, 2u, ADC_CFGR2_OVSS_POS);
    mmio::set_bits(CFGR2, ADC_CFGR2_OVSE | ADC_CFGR2_TOVS);
    break;
  case 8:
    mmio::set(CFGR2, ADC_CFGR2_OVSR, 2u, ADC_CFGR2_OVSR_POS);
    mmio::set(CFGR2, ADC_CFGR2_OVSS, 3u, ADC_CFGR2_OVSS_POS);
    mmio::set_bits(CFGR2, ADC_CFGR2_OVSE | ADC_CFGR2_TOVS);
    break;
  case 16:
    mmio::set(CFGR2, ADC_CFGR2_OVSR, 3u, ADC_CFGR2_OVSR_POS);
    mmio::set(CFGR2, ADC_CFGR2_OVSS, 4u, ADC_CFGR2_OVSS_POS);
    mmio::set_bits(CFGR2, ADC_CFGR2_OVSE | ADC_CFGR2_TOVS);
    break;
  case 32:
    mmio::set(CFGR2, ADC_CFGR2_OVSR, 4u, ADC_CFGR2_OVSR_POS);
    mmio::set(CFGR2, ADC_CFGR2_OVSS, 5u, ADC_CFGR2_OVSS_POS);
    mmio::set_bits(CFGR2, ADC_CFGR2_OVSE | ADC_CFGR2_TOVS);
    break;
  case 64:
    mmio::set(CFGR2, ADC_CFGR2_OVSR, 5u, ADC_CFGR2_OVSR_POS);
    mmio::set(CFGR2, ADC_CFGR2_OVSS, 6u, ADC_CFGR2_OVSS_POS);
    mmio::set_bits(CFGR2, ADC_CFGR2_OVSE | ADC_CFGR2_TOVS);
    break;
  case 128:
    mmio::set(CFGR2, ADC_CFGR2_OVSR, 6u, ADC_CFGR2_OVSR_POS);
    mmio::set(CFGR2, ADC_CFGR2_OVSS, 7u, ADC_CFGR2_OVSS_POS);
    mmio::set_bits(CFGR2, ADC_CFGR2_OVSE | ADC_CFGR2_TOVS);
    break;
  case 256:
    mmio::set(CFGR2, ADC_CFGR2_OVSR, 7u, ADC_CFGR2_OVSR_POS);
    mmio::set(CFGR2, ADC_CFGR2_OVSS, 8u, ADC_CFGR2_OVSS_POS);
    mmio::set_bits(CFGR2, ADC_CFGR2_OVSE | ADC_CFGR2_TOVS);
    break;
  default:
    return tl::unexpected(ConfigError::invalid_oversampling_ratio);
  }
  mmio::set_bits(CFGR2, ADC_CFGR2_LFTRIG);

  // set the sampling time
  if (cfg.sampling_time <= 2) {
    hal::mmio::set(SMPR, ADC_SMPR_SMP1, 0u, ADC_SMPR_SMP1_POS);
  } else if (cfg.sampling_time <= 4) {
    hal::mmio::set(SMPR, ADC_SMPR_SMP1, 1u, ADC_SMPR_SMP1_POS);
  } else if (cfg.sampling_time <= 8) {
    hal::mmio::set(SMPR, ADC_SMPR_SMP1, 2u, ADC_SMPR_SMP1_POS);
  } else if (cfg.sampling_time <= 13) {
    hal::mmio::set(SMPR, ADC_SMPR_SMP1, 3u, ADC_SMPR_SMP1_POS);
  } else if (cfg.sampling_time <= 20) {
    hal::mmio::set(SMPR, ADC_SMPR_SMP1, 4u, ADC_SMPR_SMP1_POS);
  } else if (cfg.sampling_time <= 40) {
    hal::mmio::set(SMPR, ADC_SMPR_SMP1, 5u, ADC_SMPR_SMP1_POS);
  } else if (cfg.sampling_time <= 80) {
    hal::mmio::set(SMPR, ADC_SMPR_SMP1, 6u, ADC_SMPR_SMP1_POS);
  } else {
    hal::mmio::set(SMPR, ADC_SMPR_SMP1, 7u, ADC_SMPR_SMP1_POS);
  }

  // set the prescaler
  const auto div = clock_tree().sysclk / cfg.clock_rate;
  if (div <= 1) {
    mmio::set(CCR, ADC_CCR_PRESC, 0u, ADC_CCR_PRESC_POS);
  } else if (div <= 2) {
    mmio::set(CCR, ADC_CCR_PRESC, 1u, ADC_CCR_PRESC_POS);
  } else if (div <= 4) {
    mmio::set(CCR, ADC_CCR_PRESC, 2u, ADC_CCR_PRESC_POS);
  } else if (div <= 6) {
    mmio::set(CCR, ADC_CCR_PRESC, 3u, ADC_CCR_PRESC_POS);
  } else if (div <= 8) {
    mmio::set(CCR, ADC_CCR_PRESC, 4u, ADC_CCR_PRESC_POS);
  } else if (div <= 10) {
    mmio::set(CCR, ADC_CCR_PRESC, 5u, ADC_CCR_PRESC_POS);
  } else if (div <= 12) {
    mmio::set(CCR, ADC_CCR_PRESC, 6u, ADC_CCR_PRESC_POS);
  } else if (div <= 16) {
    mmio::set(CCR, ADC_CCR_PRESC, 7u, ADC_CCR_PRESC_POS);
  } else if (div <= 32) {
    mmio::set(CCR, ADC_CCR_PRESC, 8u, ADC_CCR_PRESC_POS);
  } else if (div <= 64) {
    mmio::set(CCR, ADC_CCR_PRESC, 9u, ADC_CCR_PRESC_POS);
  } else if (div <= 128) {
    mmio::set(CCR, ADC_CCR_PRESC, 10u, ADC_CCR_PRESC_POS);
  } else if (div <= 256) {
    mmio::set(CCR, ADC_CCR_PRESC, 11u, ADC_CCR_PRESC_POS);
  } else {
    return tl::unexpected(ConfigError::invalid_clock_frequency);
  }

  if (cfg.has_option(Options::continous)) {
    mmio::set_bits(CFGR1, ADC_CFGR1_CONT);
  }

  if (not cfg.has_option(Options::sequence_conversion)) {
    // single conversion
    mmio::set_bits(CFGR1, ADC_CFGR1_DISCEN);
    mmio::reset_bits(CFGR1, ADC_CFGR1_CONT);
  }

  // set channel configuration
  mmio::set(CHSELR, static_cast<std::uint32_t>(cfg.channels));

  adc->enable_regulator();
  adc->CFGR1 = CFGR1;
  adc->CFGR2 = CFGR2;
  adc->CHSELR = CHSELR;
  adc->SMPR = SMPR;
  adc->IER = IER;
  // adc->calibrate();
  return Adc(HandleRef(*adc), cfg);
}

} // namespace hal::adc

extern "C" {

void ADC1_IRQHandler(void) {
  using namespace stm32c031xx;
  using namespace hal::adc;

  auto *const adc = reinterpret_cast<ADC *>(0x40012400u);
  if (ADC::callbacks == nullptr)
    return;

  const auto isr = adc->ISR;

  if (isr & ADC_ISR_EOCAL) {
    ADC::callbacks->monitor(Event::end_of_calibration);
    hal::mmio::set_bits(adc->ISR, ADC_ISR_EOCAL);
  }
  if (isr & ADC_ISR_ADRDY) {
    ADC::callbacks->monitor(Event::ready);
  }
  if (isr & ADC_ISR_EOC) {
    ADC::channel_values[ADC::index] = adc->DR;
    ADC::callbacks->end_of_conversion(ADC::channel_values[ADC::index]);
    ADC::index += 1;
    ADC::callbacks->monitor(Event::end_of_conversion);
  }
  if (isr & ADC_ISR_EOS) {
    ADC::channel_values[ADC::index] = adc->DR;
    ADC::index = 0;
    ADC::callbacks->end_of_sequence(ADC::channel_values);
    ADC::callbacks->monitor(Event::end_of_conversion);
  }
  if (isr & ADC_ISR_CCRDY) {
    hal::mmio::set_bits(adc->ISR, ADC_ISR_CCRDY);
  }
  if (isr & ADC_ISR_EOSMP) {
    hal::mmio::set_bits(adc->ISR, ADC_ISR_EOSMP);
  }
  if (isr & ADC_ISR_OVR) {
    hal::mmio::set_bits(adc->ISR, ADC_ISR_OVR);
    ADC::callbacks->monitor(Event::overrun);
  }
}
}
