# Can also be included by the desktop test build, without embedded source linkage.
add_executable(codecell_firmware_host_tests FirmwareClockTest.cpp)
target_include_directories(codecell_firmware_host_tests PRIVATE ../include)
target_compile_features(codecell_firmware_host_tests PRIVATE cxx_std_20)
target_link_libraries(codecell_firmware_host_tests PRIVATE GTest::gtest_main)
gtest_discover_tests(codecell_firmware_host_tests DISCOVERY_MODE PRE_TEST PROPERTIES LABELS "core")
