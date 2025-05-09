inc = include_directories('include')

ctre_dep = dependency(
  'ctre',
  method: 'cmake',
  cmake_args: ['CTRE_BUILD_TESTS=OFF', 'CTRE_BUILD_PACKAGE=OFF'],
)

catch_dep = dependency(
  'catch2',
  default_options: ['tests=false', 'buildtype=debug'],
  fallback: ['catch2', 'catch2_with_main_dep'],
  version: '>=3.4.0',
  required: true,
)

term_dep = dependency('cpp-terminal', fallback: ['cpp-terminal', 'cpp_terminal_dep'], static:true)

gcem_dep = dependency('gcem')

if (meson.is_cross_build())
  poly_dep = dependency('poly', default_options: ['tests=false'])
  cm_dep = dependency('cm')
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
    override_options: ['cpp_std=c++20'],
    dependencies: deps,
  )
  executable(
    'cli-test',
    'source/tests/cli/cli.cpp',
    include_directories: inc,
    dependencies: ctre_dep,
  )
  executable(
    'ctti-test',
    'source/tests/cli/ctti.cpp',
    include_directories: inc,
    dependencies: ctre_dep,
  )
  executable(
    'unit-test',
    'source/tests/cli/parse/int.cpp',
    dependencies: [catch_dep, ctre_dep],
  )
else
  executable(
    'cli-test',
    'source/tests/cli/cli.cpp',
    include_directories: inc,
    native: true,
    dependencies: [ctre_dep, gcem_dep, term_dep],
  )
  executable(
    'ctti-test',
    'source/tests/cli/ctti.cpp',
    include_directories: inc,
    native: true,
    dependencies: ctre_dep,
  )
  executable(
    'unit-test',
    'source/tests/cli/format/bool.cpp',
    'source/tests/cli/format/enum.cpp',
    'source/tests/cli/format/fixpoint.cpp',
    'source/tests/cli/format/int.cpp',
    'source/tests/cli/format/sequence.cpp',
    'source/tests/cli/format/string.cpp',
    'source/tests/cli/format/struct.cpp',
    'source/tests/cli/parse/bool.cpp',
    'source/tests/cli/parse/enum.cpp',
    'source/tests/cli/parse/fixpoint.cpp',
    'source/tests/cli/parse/int.cpp',
    'source/tests/cli/parse/sequence.cpp',
    'source/tests/cli/parse/string.cpp',
    'source/tests/cli/parse/struct.cpp',
    include_directories: [inc, include_directories('source/tests/cli')],
    dependencies: [catch_dep, ctre_dep, gcem_dep],
  )
endif
