# Link-check fixture project: INTERFACE targets only, so no compiler is needed.
# VerifyBoundaryConfigure.cmake configures it once per SCENARIO.
cmake_minimum_required(VERSION 3.24)
project(MicroRobotBoundaryLinkFixture LANGUAGES NONE)

include("${BOUNDARIES_MODULE}")

add_library(fixture_core INTERFACE)
micro_robot_platform_boundary(fixture_core)

# Defines fixture_child_core (registered), fixture_child_simulation (registered
# BACKEND), and, one level deeper, fixture_child_backend (unregistered adapter).
add_subdirectory(child)

if(SCENARIO STREQUAL "unregistered_child_backend")
    target_link_libraries(fixture_core INTERFACE fixture_child_backend)
elseif(SCENARIO STREQUAL "registered_child_backend")
    target_link_libraries(fixture_core INTERFACE fixture_child_simulation)
elseif(SCENARIO STREQUAL "registered_child_dependency")
    target_link_libraries(fixture_core INTERFACE fixture_child_core)
else()
    message(FATAL_ERROR "Unknown SCENARIO '${SCENARIO}'")
endif()

micro_robot_check_platform_boundaries("${CMAKE_CURRENT_SOURCE_DIR}")
