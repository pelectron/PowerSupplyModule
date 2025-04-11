poly_dep = dependency('poly', default_options: ['tests=false'])
cm_dep = dependency('cm')
gcem_dep = dependency('gcem')
deps = [poly_dep, cm_dep, gcem_dep]

inc = include_directories('include')

src = [
  'source/clocks.cpp',
  'source/gpio.cpp',
  'source/startup_stm32c031xx.s',
  'source/spi.cpp',
  'source/main.cpp',
]

executable(
  'compile_test',
  sources: src,
  include_directories: inc,
  dependencies: deps,
)
