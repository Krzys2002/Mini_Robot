cmake_minimum_required(VERSION 3.24)

project(MicroRobot VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")
include(MicroRobotPlatform)

option(MICRO_ROBOT_BUILD_DESKTOP "Build the Qt desktop application and UI tests" ON)
option(MICRO_ROBOT_BUILD_REPLAY "Build the OpenCV recorded-video adapter and tests" ON)
if(MICRO_ROBOT_BUILD_REPLAY)
    find_package(OpenCV REQUIRED COMPONENTS core imgproc videoio)
endif()
if(MICRO_ROBOT_BUILD_DESKTOP)
    find_package(Qt6 REQUIRED COMPONENTS Widgets Multimedia)
endif()

add_subdirectory(src)

include(CTest)
if(BUILD_TESTING)
    find_package(GTest CONFIG QUIET)
    if(NOT GTest_FOUND)
        include(FetchContent)
        # Match the default MSVC runtime used by the application.
        set(gtest_force_shared_crt ON CACHE BOOL "Use shared MSVC runtime" FORCE)
        set(INSTALL_GTEST OFF CACHE BOOL "Disable GoogleTest installation" FORCE)
        set(BUILD_GMOCK OFF CACHE BOOL "GoogleMock is not used by this project" FORCE)
        FetchContent_Declare(
            googletest
            URL https://github.com/google/googletest/archive/refs/tags/v1.17.0.tar.gz
            DOWNLOAD_EXTRACT_TIMESTAMP TRUE
            TIMEOUT 60
            INACTIVITY_TIMEOUT 15
        )
        FetchContent_MakeAvailable(googletest)
    endif()
    add_subdirectory(tests)
endif()
