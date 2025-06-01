hal_src = files(
  'clocks.cpp',
  'gpio.cpp',
  'hal.cpp',
  'i2c.cpp',
  'spi.cpp',
  'startup_stm32c031xx.s',
  'uart.cpp',
)

linker_args = ['-Wl,-T,@0@'.format(files('STM32C031XX_FLASH.ld')[0].full_path())]

hal_lib = static_library(
  'hal',
  hal_src,
  include_directories: inc,
  native: false,
  dependencies: deps,
)

hal_dep = declare_dependency(link_with: hal_lib, include_directories: inc, dependencies: deps)
