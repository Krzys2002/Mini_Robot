include(GoogleTest)

if(MICRO_ROBOT_BUILD_REPLAY)
    add_executable(micro_robot_replay_tests camera/RecordedVideoSourceTest.cpp)
    target_link_libraries(micro_robot_replay_tests PRIVATE
        micro_robot_camera_replay GTest::gtest_main ${OpenCV_LIBS}
    )
    target_include_directories(micro_robot_replay_tests SYSTEM PRIVATE ${OpenCV_INCLUDE_DIRS})
    micro_robot_target_defaults(micro_robot_replay_tests)
    gtest_discover_tests(micro_robot_replay_tests DISCOVERY_MODE PRE_TEST
        PROPERTIES LABELS "replay")
endif()

if(MICRO_ROBOT_BUILD_DESKTOP)
    micro_robot_add_desktop_packaging_tests(micro_robot_desktop)
    add_executable(micro_robot_tests
        main.cpp
        ui/QtCameraSourceTest.cpp
        ui/MainWindowTest.cpp
        ui/CameraSelectionControllerTest.cpp
        ui/ManualBackends.hpp
    )
    target_link_libraries(micro_robot_tests PRIVATE
        micro_robot_ui micro_robot_simulation micro_robot_camera_discovery GTest::gtest
    )
    micro_robot_target_defaults(micro_robot_tests)
    gtest_discover_tests(micro_robot_tests DISCOVERY_MODE PRE_TEST PROPERTIES LABELS "ui")
endif()

# Domain tests do not link Qt or construct a QApplication.
add_executable(micro_robot_common_tests common/TypesTest.cpp)
target_link_libraries(micro_robot_common_tests PRIVATE micro_robot_common GTest::gtest_main)
micro_robot_target_defaults(micro_robot_common_tests)
gtest_discover_tests(micro_robot_common_tests DISCOVERY_MODE PRE_TEST PROPERTIES LABELS "core")

# Camera contract tests compile without Qt, transport, or a concrete backend.
add_executable(micro_robot_camera_tests camera/CameraTypesTest.cpp camera/CameraSelectionTest.cpp)
target_link_libraries(micro_robot_camera_tests PRIVATE
    micro_robot_camera_selection GTest::gtest_main
)
micro_robot_target_defaults(micro_robot_camera_tests)
gtest_discover_tests(micro_robot_camera_tests DISCOVERY_MODE PRE_TEST)

add_executable(micro_robot_interface_tests interfaces/InterfacesTest.cpp)
target_link_libraries(micro_robot_interface_tests PRIVATE
    micro_robot_camera_api micro_robot_transport_api GTest::gtest_main
)
micro_robot_target_defaults(micro_robot_interface_tests)
gtest_discover_tests(micro_robot_interface_tests DISCOVERY_MODE PRE_TEST PROPERTIES LABELS "core")

add_executable(micro_robot_transport_tests transport/TransportTypesTest.cpp)
target_link_libraries(micro_robot_transport_tests PRIVATE micro_robot_transport_api GTest::gtest_main)
micro_robot_target_defaults(micro_robot_transport_tests)
gtest_discover_tests(micro_robot_transport_tests DISCOVERY_MODE PRE_TEST PROPERTIES LABELS "core")

add_executable(micro_robot_simulation_tests simulation/SimulationTest.cpp)
target_link_libraries(micro_robot_simulation_tests PRIVATE
    micro_robot_simulation GTest::gtest_main
)
micro_robot_target_defaults(micro_robot_simulation_tests)
gtest_discover_tests(micro_robot_simulation_tests DISCOVERY_MODE PRE_TEST PROPERTIES LABELS "core")

add_executable(micro_robot_world_tests world/WorldModelTest.cpp)
target_link_libraries(micro_robot_world_tests PRIVATE micro_robot_world GTest::gtest_main)
micro_robot_target_defaults(micro_robot_world_tests)
gtest_discover_tests(micro_robot_world_tests DISCOVERY_MODE PRE_TEST PROPERTIES LABELS "core")

add_executable(micro_robot_logging_tests logging/LoggerTest.cpp)
target_link_libraries(micro_robot_logging_tests PRIVATE micro_robot_logging GTest::gtest_main)
micro_robot_target_defaults(micro_robot_logging_tests)
gtest_discover_tests(micro_robot_logging_tests DISCOVERY_MODE PRE_TEST PROPERTIES LABELS "core")

add_executable(micro_robot_configuration_tests configuration/ConfigLoaderTest.cpp)
target_link_libraries(micro_robot_configuration_tests PRIVATE
    micro_robot_configuration GTest::gtest_main
)
# The shipped example file must stay valid and match the built-in defaults.
target_compile_definitions(micro_robot_configuration_tests PRIVATE
    MICRO_ROBOT_EXAMPLE_CONFIG="${PROJECT_SOURCE_DIR}/config/micro_robot.example.ini"
)
micro_robot_target_defaults(micro_robot_configuration_tests)
gtest_discover_tests(micro_robot_configuration_tests DISCOVERY_MODE PRE_TEST PROPERTIES LABELS "core")

# Platform-boundary scans run on every host without building or linking anything.
micro_robot_add_platform_boundary_tests()
add_test(NAME PlatformBoundary.CheckerSelfTest
    COMMAND "${CMAKE_COMMAND}"
        "-DCHECKER=${PROJECT_SOURCE_DIR}/cmake/CheckPlatformBoundary.cmake"
        "-DFIXTURES=${CMAKE_CURRENT_SOURCE_DIR}/boundaries/fixtures"
        "-DWORK_DIR=${CMAKE_CURRENT_BINARY_DIR}/boundaries"
        -P "${CMAKE_CURRENT_SOURCE_DIR}/boundaries/VerifyBoundaryCheck.cmake")
set_tests_properties(PlatformBoundary.CheckerSelfTest PROPERTIES LABELS "core;boundary")
add_test(NAME PlatformBoundary.LinkCheckSelfTest
    COMMAND "${CMAKE_COMMAND}"
        "-DMODULE=${PROJECT_SOURCE_DIR}/cmake/MicroRobotBoundaries.cmake"
        "-DFIXTURE_PROJECT=${CMAKE_CURRENT_SOURCE_DIR}/boundaries/link_project"
        "-DWORK_DIR=${CMAKE_CURRENT_BINARY_DIR}/boundaries/link"
        "-DGENERATOR=${CMAKE_GENERATOR}"
        "-DGENERATOR_PLATFORM=${CMAKE_GENERATOR_PLATFORM}"
        "-DGENERATOR_TOOLSET=${CMAKE_GENERATOR_TOOLSET}"
        "-DMAKE_PROGRAM=${CMAKE_MAKE_PROGRAM}"
        -P "${CMAKE_CURRENT_SOURCE_DIR}/boundaries/VerifyBoundaryConfigure.cmake")
set_tests_properties(PlatformBoundary.LinkCheckSelfTest PROPERTIES LABELS "core;boundary")
