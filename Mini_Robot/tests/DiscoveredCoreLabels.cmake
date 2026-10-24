# CTest includes this after both GoogleTest discovery scripts populate their lists.
if(micro_robot_ble_contract_tests_TESTS)
    set_tests_properties(${micro_robot_ble_contract_tests_TESTS}
        PROPERTIES LABELS "core;ble")
endif()
if(micro_robot_calibration_tests_TESTS)
    set_tests_properties(${micro_robot_calibration_tests_TESTS}
        PROPERTIES LABELS "core;calibration")
endif()
