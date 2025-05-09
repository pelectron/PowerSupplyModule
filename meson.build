project(
  'PowerSupplyModule',
  'c',
  'cpp',
  default_options: ['cpp_std=c++20', 'c_std=c11'],
  meson_version: '>=1.1',
)

cxx = meson.get_compiler('cpp')
if cxx.get_id().contains('clang') or cxx.get_id() == 'gcc'
  add_global_arguments('-Wno-undefined-var-template', language: 'cpp')
endif
if cxx.get_id() == 'msvc'
  add_global_link_arguments('/NODEFAULTLIB:msvcrt', language: 'cpp')
endif
if get_option('hardware')
  subdir('hardware')
endif

if get_option('software')
  subdir('software')
endif
