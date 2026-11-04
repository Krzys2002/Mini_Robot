# Build-only entry points for independent PlatformIO projects. No firmware is
# compiled by CMake, and these targets are never part of ALL or desktop links.
include_guard(GLOBAL)

option(MICRO_ROBOT_ENABLE_FIRMWARE_TOOLS
    "Expose build-only targets for PlatformIO projects under firmware/" ON)
if(NOT MICRO_ROBOT_ENABLE_FIRMWARE_TOOLS)
    return()
endif()

find_program(MICRO_ROBOT_PLATFORMIO_EXECUTABLE NAMES pio platformio
    DOC "PlatformIO Core executable (optional; used only by firmware targets)")
if(NOT MICRO_ROBOT_PLATFORMIO_EXECUTABLE)
    message(STATUS "Micro Robot: PlatformIO not found; optional firmware targets disabled")
    return()
endif()

file(GLOB _micro_robot_firmware_manifests CONFIGURE_DEPENDS
    "${PROJECT_SOURCE_DIR}/firmware/*/platformio.ini")
foreach(_manifest IN LISTS _micro_robot_firmware_manifests)
    get_filename_component(_firmware_dir "${_manifest}" DIRECTORY)
    get_filename_component(_firmware_name "${_firmware_dir}" NAME)
    string(MAKE_C_IDENTIFIER "firmware_${_firmware_name}" _target)
    if(TARGET "${_target}")
        message(FATAL_ERROR "PlatformIO firmware target name collision: ${_target}")
    endif()
    add_custom_target("${_target}"
        COMMAND "${MICRO_ROBOT_PLATFORMIO_EXECUTABLE}" run --project-dir "${_firmware_dir}"
        WORKING_DIRECTORY "${_firmware_dir}"
        COMMENT "Building independent PlatformIO project ${_firmware_name} (no upload)"
        SOURCES "${_manifest}"
        USES_TERMINAL VERBATIM)
    set_target_properties("${_target}" PROPERTIES FOLDER "Firmware (PlatformIO)")
    message(STATUS "Micro Robot: ${_target} -> ${_manifest}")
endforeach()
