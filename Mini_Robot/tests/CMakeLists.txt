include(GoogleTest)

if(MICRO_ROBOT_BUILD_DESKTOP)
    add_executable(micro_robot_tests
        main.cpp
        ui/MainWindowTest.cpp
        ui/ManualBackends.hpp
    )
    target_link_libraries(micro_robot_tests PRIVATE micro_robot_ui micro_robot_simulation GTest::gtest)
    micro_robot_target_defaults(micro_robot_tests)
    gtest_discover_tests(micro_robot_tests DISCOVERY_MODE PRE_TEST PROPERTIES LABELS "ui")
endif()

# Domain tests do not link Qt or construct a QApplication.
add_executable(micro_robot_common_tests common/TypesTest.cpp)
target_link_libraries(micro_robot_common_tests PRIVATE micro_robot_common GTest::gtest_main)
micro_robot_target_defaults(micro_robot_common_tests)
gtest_discover_tests(micro_robot_common_tests DISCOVERY_MODE PRE_TEST PROPERTIES LABELS "core")

# Camera contract tests compile without Qt, transport, or a concrete backend.
add_executable(micro_robot_camera_tests camera/CameraTypesTest.cpp)
target_link_libraries(micro_robot_camera_tests PRIVATE micro_robot_camera_api GTest::gtest_main)
micro_robot_target_defaults(micro_robot_camera_tests)
gtest_discover_tests(micro_robot_camera_tests DISCOVERY_MODE PRE_TEST)

add_executable(micro_robot_interface_tests interfaces/InterfacesTest.cpp)
target_link_libraries(micro_robot_interface_tests PRIVATE
    micro_robot_camera_api micro_robot_transport_api GTest::gtest_main
)
micro_robot_target_defaults(micro_robot_interface_tests)
gtest_discover_tests(micro_robot_interface_tests DISCOVERY_MODE PRE_TEST PROPERTIES LABELS "core")

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
