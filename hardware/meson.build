# the directories to search for the kicad-cli executable
kicad_cli_dirs = [
  'C:/Program Files/KiCad/9.0/bin',
  '/Applications/KiCad/KiCad.app/Contents/MacOS/',
]

# the jobfile which defines all actions to take
jobset_input = files('fabricate.kicad_jobset.in')[0]
jobset_file = configure_file(
  configuration: {'BUILD_ROOT': meson.current_build_dir()},
  input: jobset_input,
  output: 'fabricate.kicad_jobset',
)

# the project file
kicad_project = files('PowerSupplyModule.kicad_pro')

# the kicad-cli executable
kicad_cli = find_program('kicad-cli', dirs: kicad_cli_dirs)

#the command to invoke to generate the jobset output
kicad_cmd = [
  kicad_cli,
  'jobset',
  'run',
  kicad_project,
  '--file', jobset_file,
  '--stop-on-error',
]

# dependencies to tell meson when to regenerate output
kicad_deps = [
  'LDO.kicad_sch',
  'Library.kicad_sym',
  'PowerSupplyModule.kicad_pcb',
  'PowerSupplyModule.kicad_pro',
  'PowerSupplyModule.kicad_sch',
  'aux_buck_boost.kicad_sch',
  'buck_boost.kicad_sch',
  'low_noise_output.kicad_sch',
  # jobset_input.full_path(),
  # jobset_file.full_path(),
]

run_target(
  'hardware',
  command: kicad_cmd,
  # depends: kicad_deps,
)
