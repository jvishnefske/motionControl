project('flight-simulator', 'cpp',
  version : '0.1',
  default_options : ['warning_level=3'])

dep_boost = dependency('boost')
exe = executable('motion_simulator', 'main.cpp',
  install : true, dependencies : [dep_boost])

test('basic', exe)
