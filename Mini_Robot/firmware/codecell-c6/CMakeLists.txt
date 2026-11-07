cmake_minimum_required(VERSION 3.24)
project(codecell_firmware_host_tests LANGUAGES CXX)
# Independent host entry point: no PlatformIO, Arduino SDK, Qt or OpenCV discovery.
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
enable_testing()
find_package(GTest CONFIG QUIET)
if(NOT GTest_FOUND)
    include(FetchContent)
    FetchContent_Declare(googletest
        URL https://github.com/google/googletest/archive/refs/tags/v1.17.0.tar.gz
        DOWNLOAD_EXTRACT_TIMESTAMP TRUE TIMEOUT 60 INACTIVITY_TIMEOUT 15)
    FetchContent_MakeAvailable(googletest)
endif()
include(GoogleTest)
add_subdirectory(tests)
