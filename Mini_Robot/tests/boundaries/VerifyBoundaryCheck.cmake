# Self-test for cmake/CheckPlatformBoundary.cmake: forbidden directives in a
# fixture are each reported and fail the check; a platform-neutral fixture passes.
# Backend headers are forbidden only for targets that are not backends themselves.
#
#   cmake -DCHECKER=<script> -DFIXTURES=<dir> -DWORK_DIR=<dir> -P VerifyBoundaryCheck.cmake
cmake_minimum_required(VERSION 3.24)

set(failures)

set(backend_headers "camera/QtCameraDeviceDiscovery.hpp;simulation/FakeCameraSource.hpp")

function(run_checker name allow_qt sources backends out_result out_output)
    set(manifest "${WORK_DIR}/${name}.cmake")
    file(WRITE "${manifest}"
        "set(BOUNDARY_TARGET [==[${name}]==])\n"
        "set(BOUNDARY_ALLOW_QT ${allow_qt})\n"
        "set(BOUNDARY_SOURCES [==[${sources}]==])\n"
        "set(BOUNDARY_BACKEND_HEADERS [==[${backends}]==])\n")
    execute_process(
        COMMAND "${CMAKE_COMMAND}" "-DMANIFEST=${manifest}" -P "${CHECKER}"
        RESULT_VARIABLE result OUTPUT_VARIABLE output ERROR_VARIABLE output)
    set(${out_result} "${result}" PARENT_SCOPE)
    set(${out_output} "${output}" PARENT_SCOPE)
endfunction()

function(expect_reported output label)
    foreach(fragment IN LISTS ARGN)
        string(FIND "${output}" "${fragment}" position)
        if(position EQUAL -1)
            set(failures ${failures} "${label}: '${fragment}' was not reported" PARENT_SCOPE)
            return()
        endif()
    endforeach()
endfunction()

set(leak "${FIXTURES}/PlatformLeak.hpp")
set(neutral "${FIXTURES}/PlatformNeutral.hpp")
set(platform_directives
    "#include <QtMultimedia/QCamera>"
    "#include <unistd.h>"
    "#include <sys/ioctl.h>"
    "#include <CoreBluetooth/CoreBluetooth.h>"
    "#include <opencv2/videoio.hpp>"
    "#if defined(__APPLE__)"
    "#ifdef _WIN32"
)

run_checker(core_leak OFF "${leak}" "${backend_headers}" result output)
if(result EQUAL 0)
    list(APPEND failures "core_leak: checker passed a target with platform dependencies")
endif()
expect_reported("${output}" core_leak ${platform_directives} "#include <QObject>"
    "concrete backend include '#include \"camera/QtCameraDeviceDiscovery.hpp\"'")

# Qt adapters may include Qt Core, but device backends and OS APIs stay forbidden.
run_checker(qt_leak ON "${leak}" "" result output)
if(result EQUAL 0)
    list(APPEND failures "qt_leak: checker passed a Qt target with platform dependencies")
endif()
expect_reported("${output}" qt_leak ${platform_directives})
foreach(fragment IN ITEMS "#include <QObject>" "QtCameraDeviceDiscovery.hpp")
    string(FIND "${output}" "${fragment}" position)
    if(NOT position EQUAL -1)
        list(APPEND failures "qt_leak: '${fragment}' was reported without being forbidden")
    endif()
endforeach()

run_checker(objc OFF "${WORK_DIR}/Adapter.mm" "" result output)
if(result EQUAL 0)
    list(APPEND failures "objc: checker accepted an Objective-C++ source")
endif()
expect_reported("${output}" objc "Objective-C source")

run_checker(neutral OFF "${neutral}" "${backend_headers}" result output)
if(NOT result EQUAL 0)
    list(APPEND failures "neutral: checker rejected a platform-independent source:\n${output}")
endif()

if(failures)
    list(JOIN failures "\n  " details)
    message(FATAL_ERROR "Platform-boundary checker self-test failed:\n  ${details}")
endif()
message(STATUS "Platform-boundary checker self-test passed")
