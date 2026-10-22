cmake_minimum_required(VERSION 3.24)

project(MicroRobot VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")
include(MicroRobotPlatform)
include(MicroRobotBoundaries)

option(MICRO_ROBOT_BUILD_DESKTOP "Build the Qt desktop application and UI tests" ON)
option(MICRO_ROBOT_BUILD_BLE "Build the opt-in Qt Bluetooth transport and hardware-free lifecycle tests" OFF)
option(MICRO_ROBOT_BUILD_REPLAY "Build the OpenCV recorded-video adapter and tests" ON)
option(MICRO_ROBOT_BUILD_MARKER_TOOLS "Build offline AprilTag print/verification tools" OFF)
option(MICRO_ROBOT_BUILD_CALIBRATION "Build OpenCV calibration detection/solving without the desktop" OFF)
if(MICRO_ROBOT_BUILD_REPLAY OR MICRO_ROBOT_BUILD_DESKTOP OR MICRO_ROBOT_BUILD_CALIBRATION)
    find_package(OpenCV REQUIRED COMPONENTS core imgproc)
    set(_opencv_components core imgproc)
    if(MICRO_ROBOT_BUILD_REPLAY
       OR (MICRO_ROBOT_BUILD_DESKTOP AND MICRO_ROBOT_CAMERA_BACKEND STREQUAL "V4L2"))
        list(APPEND _opencv_components videoio)
    endif()
    if(MICRO_ROBOT_BUILD_DESKTOP OR MICRO_ROBOT_BUILD_CALIBRATION)
        # Chessboard detection moved from calib3d to objdetect in OpenCV 5.
        if(OpenCV_VERSION VERSION_GREATER_EQUAL 5)
            list(APPEND _opencv_components objdetect calib geometry)
        else()
            list(APPEND _opencv_components calib3d)
        endif()
    endif()
    find_package(OpenCV REQUIRED COMPONENTS ${_opencv_components})
endif()
if(MICRO_ROBOT_BUILD_DESKTOP)
    find_package(Qt6 REQUIRED COMPONENTS Widgets Multimedia)
    message(STATUS "Micro Robot: live camera backend ${MICRO_ROBOT_CAMERA_BACKEND}")
endif()
if(MICRO_ROBOT_BUILD_BLE)
    find_package(Qt6 6.2 REQUIRED COMPONENTS Core Bluetooth)
endif()

add_subdirectory(src)
if(MICRO_ROBOT_BUILD_MARKER_TOOLS)
    add_subdirectory(tools/markers)
endif()
# Fails configuration if a platform-independent target links a platform library or a
# concrete backend.
micro_robot_check_platform_boundaries("${PROJECT_SOURCE_DIR}/src")

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
