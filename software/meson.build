inc = include_directories('include')

native_deps = [
  dependency('gcem', native: true),
  dependency('poly', native: true, default_options: ['tests=false']),
  dependency('tl-expected', native: true),
]

deps = [
  dependency('gcem', native: false),
  dependency('poly', native: false, default_options: ['tests=false']),
  dependency('tl-expected', native: false),
  dependency('cm', native: false),
]

target = get_option('target').to_lower()
subdir('source' / 'hal' / target)
# hal_dep is now available

if (meson.is_cross_build())
  executable(
    'main.elf',
    'source/main.cpp',
    hal_src,
    include_directories: inc,
    dependencies: [deps],
    link_args: linker_args,
    native: false,
  )
elif (target == 'mock')
  executable(
    'main',
    'source/main.cpp',
    include_directories: inc,
    dependencies: hal_dep,
    native: true,
  )
else
  warning(
    'Cannot build a main executable because this is a native build without moch hal selected',
  )
endif

if (meson.is_cross_build())
  #   deps = [poly_dep, cm_dep, gcem_dep]
  #
  #   src = [
  #     'source/clocks.cpp',
  #     'source/gpio.cpp',
  #     'source/startup_stm32c031xx.s',
  #     'source/spi.cpp',
  #     'source/main.cpp',
  #   ]
  #
  #   executable(
  #     'compile_test',
  #     sources: src,
  #     include_directories: inc,
  #     override_options: ['cpp_std=c++20'],
  #     dependencies: deps,
  #   )
  #   executable(
  #     'cli-test',
  #     'source/tests/cli/cli.cpp',
  #     include_directories: inc,
  #     dependencies: ctre_dep,
  #   )
  #   executable(
  #     'ctti-test',
  #     'source/tests/cli/ctti.cpp',
  #     include_directories: inc,
  #     dependencies: ctre_dep,
  #   )
  #   executable(
  #     'unit-test',
  #     'source/tests/cli/parse/int.cpp',
  #     dependencies: [catch_dep, ctre_dep],
  #   )
else
  catch_dep = dependency(
    'catch2',
    default_options: ['tests=false', 'buildtype=debug'],
    fallback: ['catch2', 'catch2_with_main_dep'],
    version: '>=3.4.0',
    required: true,
    native: true,
  )

  executable(
    'cli-test',
    'source/tests/cli/cli.cpp',
    include_directories: inc,
    native: true,
    dependencies: native_deps,
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
    dependencies: [catch_dep, gcem_dep],
  )
endif
