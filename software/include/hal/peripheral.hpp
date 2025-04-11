#ifndef HAL_PERIPHERAL_HPP
#define HAL_PERIPHERAL_HPP

namespace hal {

enum class Peripheral {
  flash,
  pwr,
  rcc,
  crs,
  gpio,
  syscfg,
  dma,
  dmamux,
  nvic,
  exti,
  adc,
  tim1,
  tim2,
  tim3,
  tim14,
  tim15,
  tim16,
  tim17,
  irtim,
  iwdg,
  wwdg,
  rtc,
  i2c,
  uart,
  spi,
  dbg
};
}

#endif
