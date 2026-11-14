include(GoogleTest)

if(MICRO_ROBOT_BUILD_DESKTOP OR MICRO_ROBOT_BUILD_VISION)
    add_executable(micro_robot_vision_tests vision/AprilTagDetectorTest.cpp)
    target_link_libraries(micro_robot_vision_tests PRIVATE micro_robot_vision_opencv
        micro_robot_vision_observation GTest::gtest_main ${OpenCV_LIBS})
    target_include_directories(micro_robot_vision_tests SYSTEM PRIVATE ${OpenCV_INCLUDE_DIRS})
    micro_robot_target_defaults(micro_robot_vision_tests)
    if(MICRO_ROBOT_BUILD_REPLAY)
        target_sources(micro_robot_vision_tests PRIVATE vision/AprilTagReplayTest.cpp)
        target_link_libraries(micro_robot_vision_tests PRIVATE micro_robot_camera_replay)
    endif()
    gtest_discover_tests(micro_robot_vision_tests DISCOVERY_MODE PRE_TEST PROPERTIES LABELS "vision")
endif()

add_executable(micro_robot_ble_contract_tests transport/BleTypesTest.cpp transport/BleProtocolTest.cpp
    transport/BleLifecycleFixtureTest.cpp)
target_link_libraries(micro_robot_ble_contract_tests PRIVATE micro_robot_ble_contract GTest::gtest_main)
micro_robot_target_defaults(micro_robot_ble_contract_tests)
gtest_discover_tests(micro_robot_ble_contract_tests DISCOVERY_MODE PRE_TEST)

if(MICRO_ROBOT_BUILD_BLE)
    add_executable(micro_robot_ble_tests transport/ble-main.cpp transport/BleTransportTest.cpp)
    target_link_libraries(micro_robot_ble_tests PRIVATE micro_robot_ble_qt GTest::gtest)
    if(MICRO_ROBOT_BUILD_DESKTOP)
        target_link_libraries(micro_robot_ble_tests PRIVATE micro_robot_desktop_controller)
        target_compile_definitions(micro_robot_ble_tests PRIVATE MICRO_ROBOT_BLE_WITH_DESKTOP)
    endif()
    target_link_libraries(micro_robot_ble_tests PRIVATE micro_robot_robot_manager)
    micro_robot_target_defaults(micro_robot_ble_tests)
    gtest_discover_tests(micro_robot_ble_tests DISCOVERY_MODE PRE_TEST PROPERTIES LABELS "ble")
endif()

if(MICRO_ROBOT_BUILD_MARKER_TOOLS)
    add_executable(micro_robot_marker_tests vision/MarkerPrintTest.cpp)
    target_link_libraries(micro_robot_marker_tests PRIVATE micro_robot_marker_print GTest::gtest_main)
    target_compile_definitions(micro_robot_marker_tests PRIVATE
        MICRO_ROBOT_MARKER_ASSETS="${PROJECT_SOURCE_DIR}/docs/vision/markers")
    micro_robot_target_defaults(micro_robot_marker_tests)
    gtest_discover_tests(micro_robot_marker_tests DISCOVERY_MODE PRE_TEST PROPERTIES LABELS "markers")
endif()

add_executable(micro_robot_calibration_tests calibration/CalibrationCaptureTest.cpp
    calibration/IntrinsicCalibrationTest.cpp calibration/CalibrationProfileTest.cpp
    calibration/WorkspaceCalibrationTest.cpp calibration/TabletopHomographyTest.cpp
    calibration/WorkspaceAcquisitionTest.cpp calibration/WorkspaceDiagnosticsTest.cpp calibration/WorkspaceDistanceTest.cpp)
target_link_libraries(micro_robot_calibration_tests PRIVATE micro_robot_calibration GTest::gtest_main)
micro_robot_target_defaults(micro_robot_calibration_tests)
gtest_discover_tests(micro_robot_calibration_tests DISCOVERY_MODE PRE_TEST)
# Set list-valued labels after discovery; semicolon escaping through
# GoogleTest.cmake differs between supported CMake versions.
set_property(DIRECTORY APPEND PROPERTY TEST_INCLUDE_FILES
    "${CMAKE_CURRENT_SOURCE_DIR}/DiscoveredCoreLabels.cmake")

if(MICRO_ROBOT_BUILD_DESKTOP OR MICRO_ROBOT_BUILD_CALIBRATION)
    add_executable(micro_robot_calibration_detector_tests calibration/OpenCvCalibrationDetectorTest.cpp
        calibration/OpenCvWorkspaceHomographyTest.cpp calibration/OpenCvMetricObservationTest.cpp
        calibration/OpenCvIntrinsicCalibrationSolverTest.cpp calibration/OpenCvCalibrationUndistorterTest.cpp calibration/OpenCvCalibrationCornerTrackerTest.cpp)
    target_link_libraries(micro_robot_calibration_detector_tests PRIVATE
        micro_robot_calibration_opencv GTest::gtest_main ${OpenCV_LIBS})
    target_include_directories(micro_robot_calibration_detector_tests SYSTEM PRIVATE ${OpenCV_INCLUDE_DIRS})
    micro_robot_target_defaults(micro_robot_calibration_detector_tests)
    gtest_discover_tests(micro_robot_calibration_detector_tests DISCOVERY_MODE PRE_TEST PROPERTIES LABELS "calibration")
endif()

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
    # Runs the production composition offscreen and exits through the normal shutdown.
    add_test(NAME DesktopStartup.StartsAndShutsDownCleanly
        COMMAND "${CMAKE_COMMAND}"
            "-DAPP_BINARY=$<TARGET_FILE:micro_robot_desktop>"
            "-DWORK_DIR=${CMAKE_CURRENT_BINARY_DIR}/startup"
            -P "${CMAKE_CURRENT_SOURCE_DIR}/app/VerifyDesktopStartup.cmake")
    set_tests_properties(DesktopStartup.StartsAndShutsDownCleanly PROPERTIES LABELS "ui")
    add_executable(micro_robot_tests
        main.cpp
        calibration/QtCalibrationProfileStoreTest.cpp
        calibration/QtWorkspaceProfileStoreTest.cpp
        ui/VisionControllerTest.cpp
        ui/VisionDebugViewTest.cpp
        ui/WorkspaceDistanceTest.cpp
        ui/WorkspaceDiagnosticsTest.cpp
        ui/WorkspaceCalibrationTest.cpp
        ui/CalibrationWizardTest.cpp
        ui/QtCameraSourceTest.cpp
        ui/RobotControlPanelTest.cpp
        ui/MainWindowTest.cpp
        ui/CameraSelectionControllerTest.cpp
        ui/GuiCompatibilityTest.cpp
        ui/ManualBackends.hpp
    )
    # Explicit synthetic audit tool; not part of timed regression assertions.
    add_executable(micro_robot_calibration_audit EXCLUDE_FROM_ALL calibration/LiveCalibrationAudit.cpp)
    target_link_libraries(micro_robot_calibration_audit PRIVATE micro_robot_ui
        micro_robot_calibration_opencv micro_robot_vision_opencv)
    micro_robot_target_defaults(micro_robot_calibration_audit)
    find_package(Qt6 REQUIRED COMPONENTS Test)
    target_link_libraries(micro_robot_tests PRIVATE
        micro_robot_ui micro_robot_simulation micro_robot_camera_discovery micro_robot_calibration_profiles GTest::gtest Qt6::Test
    )
    if(TARGET micro_robot_camera_v4l2)
        target_sources(micro_robot_tests PRIVATE camera/V4l2CameraTest.cpp)
        target_link_libraries(micro_robot_tests PRIVATE micro_robot_camera_v4l2)
    endif()
    micro_robot_target_defaults(micro_robot_tests)
    # Physical-device cases are labelled "hardware" so that selecting the hardware-free
    # regression suite (-LE hardware) never depends on a connected camera.
    set(_micro_robot_hardware_cases "V4l2CameraHardwareTest.*")
    gtest_discover_tests(micro_robot_tests DISCOVERY_MODE PRE_TEST
        TEST_FILTER "-${_micro_robot_hardware_cases}" PROPERTIES LABELS "ui")
    if(TARGET micro_robot_camera_v4l2)
        gtest_discover_tests(micro_robot_tests DISCOVERY_MODE PRE_TEST
            TEST_FILTER "${_micro_robot_hardware_cases}" PROPERTIES LABELS "hardware")
    endif()
    # Repeats the GUI compatibility checks at fractional and integer high-DPI scale factors.
    foreach(factor IN ITEMS 1.5 2)
        string(REPLACE "." "_" suffix "${factor}")
        add_test(NAME GuiCompatibility.ScaleFactor${suffix}
            COMMAND micro_robot_tests --gtest_filter=GuiCompatibilityTest.*)
        set_tests_properties(GuiCompatibility.ScaleFactor${suffix} PROPERTIES
            ENVIRONMENT "QT_SCALE_FACTOR=${factor}" LABELS "ui;gui")
        add_test(NAME VisionDebug.ScaleFactor${suffix}
            COMMAND micro_robot_tests --gtest_filter=VisionDebugViewTest.*)
        set_tests_properties(VisionDebug.ScaleFactor${suffix} PROPERTIES
            ENVIRONMENT "QT_SCALE_FACTOR=${factor}" LABELS "ui;gui")
    endforeach()
endif()

# Domain tests do not link Qt or construct a QApplication.
add_executable(micro_robot_robot_manager_tests app/RobotManagerTest.cpp)
target_link_libraries(micro_robot_robot_manager_tests PRIVATE
    micro_robot_robot_manager micro_robot_simulation GTest::gtest_main)
micro_robot_target_defaults(micro_robot_robot_manager_tests)
gtest_discover_tests(micro_robot_robot_manager_tests DISCOVERY_MODE PRE_TEST PROPERTIES LABELS "core")

add_executable(micro_robot_control_tests control/DifferentialDriveTest.cpp)
target_link_libraries(micro_robot_control_tests PRIVATE micro_robot_control GTest::gtest_main)
micro_robot_target_defaults(micro_robot_control_tests)
gtest_discover_tests(micro_robot_control_tests DISCOVERY_MODE PRE_TEST PROPERTIES LABELS "core")

add_executable(micro_robot_common_tests common/TypesTest.cpp common/AngleTest.cpp)
target_link_libraries(micro_robot_common_tests PRIVATE micro_robot_common GTest::gtest_main)
micro_robot_target_defaults(micro_robot_common_tests)
gtest_discover_tests(micro_robot_common_tests DISCOVERY_MODE PRE_TEST PROPERTIES LABELS "core")

# Camera contract tests compile without Qt, transport, or a concrete backend.
add_executable(micro_robot_camera_tests camera/CameraTypesTest.cpp camera/CameraSelectionTest.cpp)
target_link_libraries(micro_robot_camera_tests PRIVATE
    micro_robot_camera_selection GTest::gtest_main
)
micro_robot_target_defaults(micro_robot_camera_tests)
gtest_discover_tests(micro_robot_camera_tests DISCOVERY_MODE PRE_TEST PROPERTIES LABELS "core")

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

# Robot discovery and the shared transport contract, exercised through simulation
# and a test-only radio backend; no Bluetooth adapter, Qt, or robot is needed.
add_executable(micro_robot_transport_mock_tests
    transport/MockRadioLink.hpp
    transport/RobotDiscoveryTest.cpp
    transport/TransportContractTest.cpp
)
target_link_libraries(micro_robot_transport_mock_tests PRIVATE
    micro_robot_transport_api micro_robot_simulation GTest::gtest_main
)
micro_robot_target_defaults(micro_robot_transport_mock_tests)
gtest_discover_tests(micro_robot_transport_mock_tests DISCOVERY_MODE PRE_TEST PROPERTIES LABELS "core")

add_executable(micro_robot_simulation_tests simulation/SimulationTest.cpp)
target_link_libraries(micro_robot_simulation_tests PRIVATE
    micro_robot_simulation GTest::gtest_main
)
micro_robot_target_defaults(micro_robot_simulation_tests)
gtest_discover_tests(micro_robot_simulation_tests DISCOVERY_MODE PRE_TEST PROPERTIES LABELS "core")

# Robot identity, center and heading from raw markers; no detector backend.
add_executable(micro_robot_vision_observation_tests vision/RobotObservationTest.cpp vision/VisionDebugFrameTest.cpp vision/MetricObservationTest.cpp)
target_link_libraries(micro_robot_vision_observation_tests PRIVATE
    micro_robot_vision_observation GTest::gtest_main
)
micro_robot_target_defaults(micro_robot_vision_observation_tests)
gtest_discover_tests(micro_robot_vision_observation_tests DISCOVERY_MODE PRE_TEST)
# Apply list-valued properties after discovery, without escape-dependent parsing
# through GoogleTest.cmake (including the supported CMake 3.24 baseline).
set_property(DIRECTORY APPEND PROPERTY TEST_INCLUDE_FILES
    "${CMAKE_CURRENT_SOURCE_DIR}/vision/RobotObservationLabels.cmake")

add_executable(micro_robot_world_tests world/WorldModelTest.cpp world/WorkspaceTest.cpp world/RobotTrackerTest.cpp)
target_link_libraries(micro_robot_world_tests PRIVATE micro_robot_world micro_robot_tracking GTest::gtest_main)
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

# Filesystem-only sysfs identity tests run on every host, without Linux APIs or devices.
add_executable(micro_robot_camera_identity_tests camera/V4l2IdentityTest.cpp
    ../src/camera/V4l2Identity.cpp)
target_link_libraries(micro_robot_camera_identity_tests PRIVATE micro_robot_camera_api GTest::gtest_main)
micro_robot_target_defaults(micro_robot_camera_identity_tests)
gtest_discover_tests(micro_robot_camera_identity_tests DISCOVERY_MODE PRE_TEST PROPERTIES LABELS "core")

# Compile the real legacy controller against deterministic Arduino/CodeCell fakes.
# This complements firmware compilation; it does not establish physical stop time.
add_executable(micro_robot_firmware_safety_tests firmware/ActuatorSafetyTest.cpp
    "${PROJECT_SOURCE_DIR}/../T1-ESP-C6/src/controller.cpp")
target_include_directories(micro_robot_firmware_safety_tests PRIVATE firmware/fakes
    "${PROJECT_SOURCE_DIR}/../T1-ESP-C6/include" "${PROJECT_SOURCE_DIR}/../T1-ESP-C6/src")
target_link_libraries(micro_robot_firmware_safety_tests PRIVATE GTest::gtest_main)
micro_robot_target_defaults(micro_robot_firmware_safety_tests)
gtest_discover_tests(micro_robot_firmware_safety_tests DISCOVERY_MODE PRE_TEST PROPERTIES LABELS "core")
