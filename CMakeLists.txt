cmake_minimum_required(VERSION 3.25)
include (cmake/CPM.cmake)
CPMAddPackage("gh:nlohmann/json@3.10.5")
CPMAddPackage("gh:gabime/spdlog@1.8.2")
CPMAddPackage("gh:jvishnefske/boost-headers@1.78.0")
CPMAddPackage(
        NAME googletest
        GITHUB_REPOSITORY google/googletest
        GIT_TAG release-1.12.1
        VERSION 1.12.1
        OPTIONS "INSTALL_GTEST OFF" "gtest_force_shared_crt"
)
CPMAddPackage(
        NAME benchmark
        GITHUB_REPOSITORY google/benchmark
        VERSION 1.7.1
        OPTIONS "BENCHMARK_ENABLE_TESTING Off"
)
if(benchmark_ADDED)
    # enable c++11 to avoid compilation errors
    set_target_properties(benchmark PROPERTIES CXX_STANDARD 11)
endif()
set(dependencies Boost::boost spdlog::spdlog nlohmann_json::nlohmann_json)
#
project(motion_control)

enable_testing()
find_package(Boost REQUIRED)

add_executable(motion_control src/main.cpp)
target_include_directories(motion_control PUBLIC ${CMAKE_SOURCE_DIR}/include)
target_link_libraries(motion_control PRIVATE Boost::boost)
message(STATUS "${CMAKE_CXX_COMPILE_FEATURES}")
target_compile_features(motion_control PRIVATE cxx_std_17)
target_compile_options(motion_control PRIVATE -Werror=all -Werror=extra -Werror=pedantic -fsanitize=undefined)
target_link_options(motion_control PRIVATE -fsanitize=undefined)
add_subdirectory(test)