cmake_minimum_required(VERSION 3.25)
include(FetchContent)

FetchContent_Declare(Boost
        GIT_REPOSITORY https://github.com/jvishnefske/boost-headers
        GIT_TAG main
        OVERRIDE_FIND_PACKAGE
        )
FETCHCONTENT_MAKEAVAILABLE(Boost)
#add_subdirectory(${FETCHCONTENT_SOURCE_DIR_BOOST})
project(motion_control)

find_package(Boost REQUIRED)

add_executable(motion_control main.cpp)

target_link_libraries(motion_control PRIVATE Boost::boost)
message(STATUS "${CMAKE_CXX_COMPILE_FEATURES}")
target_compile_features(motion_control PRIVATE cxx_std_17)
target_compile_options(motion_control PRIVATE -Werror=all -Werror=extra -Werror=pedantic -fsanitize=undefined)
target_link_options(motion_control PRIVATE -fsanitize=undefined)