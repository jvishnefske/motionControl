include("/home/j/.cache/CPM/cpm/CPM_0.38.7.cmake")
CPMAddPackage("NAME;googletest;GITHUB_REPOSITORY;google/googletest;GIT_TAG;release-1.12.1;VERSION;1.12.1;OPTIONS;INSTALL_GTEST OFF;gtest_force_shared_crt")
set(googletest_FOUND TRUE)