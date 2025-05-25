hal_src = files(
  'gpio.cpp',
  'hal.cpp',
  'i2c.cpp',
  'spi.cpp',
  'uart.cpp',
)

  term_dep = dependency(
    'cpp-terminal',
    fallback: ['cpp-terminal', 'cpp_terminal_dep'],
    static: true,
    native: true,
  )

hal_lib = static_library('hal', hal_src, include_directories: inc, native:true, dependencies: [native_deps, term_dep])

hal_dep = dependency(link_with:hal_lib, include_directories: inc, dependencies: [native_deps, term_dep])

