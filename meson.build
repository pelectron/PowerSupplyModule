project(
  'PowerSupplyModule',
  'c',
  'cpp',
  default_options: ['cpp_std=c++20', 'c_std=c11'],
  meson_version: '>=1.1',
)

if get_option('hardware')
  subdir('hardware')
endif

if get_option('software')
  subdir('software')
endif
