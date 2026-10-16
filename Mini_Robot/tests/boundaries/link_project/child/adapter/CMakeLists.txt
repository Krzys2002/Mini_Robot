# Unregistered first-party target: treated as a platform adapter.
add_library(fixture_child_backend INTERFACE)
target_sources(fixture_child_backend INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/Backend.hpp")
