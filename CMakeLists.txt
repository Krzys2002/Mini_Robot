cmake_minimum_required(VERSION 3.24)

project(TermDesign LANGUAGES CXX)

# Enable CTest at the repository root so tests registered by Mini_Robot are
# discoverable from this build directory as well as from standalone builds.
include(CTest)

# Firmware projects have independent PlatformIO toolchains. The root CMake
# project deliberately builds only the desktop application and its host tests.
add_subdirectory(Mini_Robot)
