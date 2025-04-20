inc = include_directories('include')
ctre_dep = dependency('ctre', method:'cmake',cmake_args:['CTRE_BUILD_TESTS=OFF','CTRE_BUILD_PACKAGE=OFF' ])
if(meson.is_cross_build())
  poly_dep = dependency('poly', default_options: ['tests=false'])
  cm_dep = dependency('cm')
  gcem_dep = dependency('gcem')
  deps = [poly_dep, cm_dep, gcem_dep]

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
    override_options:['cpp_std=c++20'],
    dependencies: deps,
  )
  executable('cli-test','source/tests/cli/cli.cpp', include_directories: inc, dependencies: ctre_dep)
  executable('ctti-test','source/tests/cli/ctti.cpp', include_directories: inc, dependencies: ctre_dep)
else
  executable('cli-test','source/tests/cli/cli.cpp', include_directories: inc, native:true, dependencies: ctre_dep)
  executable('ctti-test','source/tests/cli/ctti.cpp', include_directories: inc, native:true, dependencies: ctre_dep)
endif
