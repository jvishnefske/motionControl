include("/home/j/.cache/CPM/cpm/CPM_0.38.7.cmake")
CPMAddPackage("NAME;benchmark;GITHUB_REPOSITORY;google/benchmark;VERSION;1.7.1;OPTIONS;BENCHMARK_ENABLE_TESTING Off")
set(benchmark_FOUND TRUE)