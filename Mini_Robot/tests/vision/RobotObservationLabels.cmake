# CTest loads this after GoogleTest discovery has populated the test list.
if(micro_robot_vision_observation_tests_TESTS)
    set_tests_properties(${micro_robot_vision_observation_tests_TESTS}
        PROPERTIES LABELS "core;vision")
endif()
