# Self-test for the configure-time link check in cmake/MicroRobotBoundaries.cmake.
# A registered core target must be rejected when it links an unregistered backend
# or a BACKEND target defined in a child directory, and accepted when it links a
# registered platform-independent target there. Backend headers from nested
# directories must reach the source-scan manifest.
#
#   cmake -DMODULE=<module> -DFIXTURE_PROJECT=<dir> -DWORK_DIR=<dir>
#         -DGENERATOR=<generator> [-DGENERATOR_PLATFORM=<p>] [-DGENERATOR_TOOLSET=<t>]
#         [-DMAKE_PROGRAM=<program>] -P VerifyBoundaryConfigure.cmake
cmake_minimum_required(VERSION 3.24)

set(failures)

function(configure_fixture scenario out_result out_output)
    set(binary "${WORK_DIR}/${scenario}")
    file(REMOVE_RECURSE "${binary}")
    set(arguments -S "${FIXTURE_PROJECT}" -B "${binary}" -G "${GENERATOR}"
        "-DBOUNDARIES_MODULE=${MODULE}" "-DSCENARIO=${scenario}")
    if(GENERATOR_PLATFORM)
        list(APPEND arguments -A "${GENERATOR_PLATFORM}")
    endif()
    if(GENERATOR_TOOLSET)
        list(APPEND arguments -T "${GENERATOR_TOOLSET}")
    endif()
    if(MAKE_PROGRAM)
        list(APPEND arguments "-DCMAKE_MAKE_PROGRAM=${MAKE_PROGRAM}")
    endif()
    execute_process(COMMAND "${CMAKE_COMMAND}" ${arguments}
        RESULT_VARIABLE result OUTPUT_VARIABLE output ERROR_VARIABLE output)
    set(${out_result} "${result}" PARENT_SCOPE)
    set(${out_output} "${output}" PARENT_SCOPE)
endfunction()

function(expect_rejected scenario dependency)
    configure_fixture(${scenario} result output)
    set(expected "fixture_core -> ${dependency} (concrete backend or unregistered target)")
    string(FIND "${output}" "${expected}" position)
    if(result EQUAL 0 OR position EQUAL -1)
        set(failures ${failures}
            "${scenario}: expected configuration to fail with '${expected}':\n${output}"
            PARENT_SCOPE)
    endif()
endfunction()

expect_rejected(unregistered_child_backend fixture_child_backend)
expect_rejected(registered_child_backend fixture_child_simulation)

configure_fixture(registered_child_dependency result output)
if(NOT result EQUAL 0)
    list(APPEND failures
        "registered_child_dependency: registered dependency was rejected:\n${output}")
else()
    include("${WORK_DIR}/registered_child_dependency/platform_boundaries/fixture_core.cmake")
    foreach(header IN ITEMS "child/adapter/Backend.hpp" "child/Simulation.hpp")
        if(NOT header IN_LIST BOUNDARY_BACKEND_HEADERS)
            list(APPEND failures "registered_child_dependency: '${header}' is not a forbidden include")
        endif()
    endforeach()
    if("child/Core.hpp" IN_LIST BOUNDARY_BACKEND_HEADERS)
        list(APPEND failures "registered_child_dependency: registered header 'child/Core.hpp' is forbidden")
    endif()
endif()

if(failures)
    list(JOIN failures "\n  " details)
    message(FATAL_ERROR "Platform-boundary link-check self-test failed:\n  ${details}")
endif()
message(STATUS "Platform-boundary link-check self-test passed")
