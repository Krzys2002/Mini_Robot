cmake_minimum_required(VERSION 3.24)
project(codecell_firmware_host_tests LANGUAGES CXX)
# Independent host entry point: no PlatformIO, Arduino SDK, Qt or OpenCV discovery.
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
enable_testing()
# Host sanitizer build for the codec bounds/fuzz tests (#161). Off by default; the
# README documents the reproducible run. GCC and Clang only.
option(CODECELL_FIRMWARE_HOST_SANITIZERS "Build host tests with AddressSanitizer and UndefinedBehaviorSanitizer" OFF)
if(CODECELL_FIRMWARE_HOST_SANITIZERS)
    if(NOT CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
        message(FATAL_ERROR "CODECELL_FIRMWARE_HOST_SANITIZERS needs GCC or Clang")
    endif()
    add_compile_options(-fsanitize=address,undefined -fno-sanitize-recover=all -fno-omit-frame-pointer)
    add_link_options(-fsanitize=address,undefined)
endif()
find_package(GTest CONFIG QUIET)
if(NOT GTest_FOUND)
    include(FetchContent)
    FetchContent_Declare(googletest
        URL https://github.com/google/googletest/archive/refs/tags/v1.17.0.tar.gz
        URL_HASH SHA256=65fab701d9829d38cb77c14acdc431d2108bfdbf8979e40eb8ae567edf10b27c
        DOWNLOAD_EXTRACT_TIMESTAMP TRUE TIMEOUT 60 INACTIVITY_TIMEOUT 15)
    FetchContent_MakeAvailable(googletest)
endif()
include(GoogleTest)
add_subdirectory(tests)
