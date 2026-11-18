# Can also be included by the desktop test build, without embedded source linkage.
add_executable(codecell_firmware_host_tests FirmwareClockTest.cpp LatencyHistogramTest.cpp TimingBudgetTest.cpp
    RuntimeComponentsTest.cpp FirmwareRuntimeTest.cpp RobotConfigRecordTest.cpp ConfigStoreTest.cpp
    MaintenanceConsoleTest.cpp ProvisioningRuntimeTest.cpp DriveKinematicsTest.cpp WheelRateLimiterTest.cpp
    BoundedDriveOutputTest.cpp ActuatorRuntimeTest.cpp ProtocolCodecTest.cpp ProtocolSequenceTest.cpp
    ProtocolSessionTest.cpp ProtocolFuzzTest.cpp SafetyEnforcementTest.cpp BleProfileTest.cpp
    BlePeripheralTest.cpp TelemetrySchedulerTest.cpp BatteryMonitorTest.cpp TelemetryPublicationTest.cpp
    DiagnosticsReportTest.cpp RestrictedLabEnforcementTest.cpp)
target_include_directories(codecell_firmware_host_tests PRIVATE ../include)
target_compile_features(codecell_firmware_host_tests PRIVATE cxx_std_20)
if(MSVC)
    target_compile_options(codecell_firmware_host_tests PRIVATE /W4)
else()
    target_compile_options(codecell_firmware_host_tests PRIVATE -Wall -Wextra -Wpedantic)
endif()
# Shared v1 byte vectors (#161), also read by the desktop codec tests.
target_compile_definitions(codecell_firmware_host_tests PRIVATE
    CODECELL_PROTOCOL_V1_VECTORS="${CMAKE_CURRENT_SOURCE_DIR}/../../../docs/protocol/fixtures/v1-frames.txt")
target_link_libraries(codecell_firmware_host_tests PRIVATE GTest::gtest_main)
gtest_discover_tests(codecell_firmware_host_tests DISCOVERY_MODE PRE_TEST PROPERTIES LABELS "core")
