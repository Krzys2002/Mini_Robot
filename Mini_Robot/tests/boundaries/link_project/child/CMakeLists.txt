add_library(fixture_child_core INTERFACE)
target_sources(fixture_child_core INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/Core.hpp")
micro_robot_platform_boundary(fixture_child_core)

add_library(fixture_child_simulation INTERFACE)
target_sources(fixture_child_simulation INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/Simulation.hpp")
micro_robot_platform_boundary(fixture_child_simulation BACKEND)

add_subdirectory(adapter)
