# Platform and toolchain configuration for the Micro Robot desktop build.
#
# Operating-system and compiler conditionals belong in this module. Target
# definitions in src/ and tests/ stay platform-neutral and call the helpers
# below instead of branching on APPLE, WIN32, or compiler IDs themselves.
include_guard(GLOBAL)

# Detect the target operating system once and expose it to the rest of the build.
if(APPLE)
    set(MICRO_ROBOT_PLATFORM "macOS")
elseif(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    set(MICRO_ROBOT_PLATFORM "Linux")
elseif(WIN32)
    set(MICRO_ROBOT_PLATFORM "Windows")
else()
    set(MICRO_ROBOT_PLATFORM "${CMAKE_SYSTEM_NAME}")
endif()

# Supported compilers per platform, with the oldest version expected to provide
# the C++20 language and library features the sources use. Windows/MSVC remains
# a portability target; its listing does not imply that it has been validated.
set(_micro_robot_compilers_macOS AppleClang)
set(_micro_robot_minimum_AppleClang 14.0)
set(_micro_robot_compilers_Linux GNU Clang)
set(_micro_robot_minimum_GNU 11.0)
set(_micro_robot_minimum_Clang 14.0)
set(_micro_robot_compilers_Windows MSVC)
set(_micro_robot_minimum_MSVC 19.29)

set(_micro_robot_compiler "${CMAKE_CXX_COMPILER_ID}")
if(NOT DEFINED _micro_robot_compilers_${MICRO_ROBOT_PLATFORM})
    message(WARNING
        "Micro Robot: platform '${MICRO_ROBOT_PLATFORM}' is not a supported target; "
        "macOS, Linux, and Windows are. Continuing without platform guarantees."
    )
elseif(NOT _micro_robot_compiler IN_LIST _micro_robot_compilers_${MICRO_ROBOT_PLATFORM})
    list(JOIN _micro_robot_compilers_${MICRO_ROBOT_PLATFORM} ", " _micro_robot_expected)
    message(WARNING
        "Micro Robot: compiler '${_micro_robot_compiler}' is not supported on "
        "${MICRO_ROBOT_PLATFORM} (expected: ${_micro_robot_expected}). Continuing untested."
    )
elseif(CMAKE_CXX_COMPILER_VERSION VERSION_LESS _micro_robot_minimum_${_micro_robot_compiler})
    message(WARNING
        "Micro Robot: ${_micro_robot_compiler} ${CMAKE_CXX_COMPILER_VERSION} is older than "
        "the supported minimum ${_micro_robot_minimum_${_micro_robot_compiler}}; "
        "C++20 support may be incomplete."
    )
endif()

message(STATUS
    "Micro Robot: ${MICRO_ROBOT_PLATFORM} with ${_micro_robot_compiler} "
    "${CMAKE_CXX_COMPILER_VERSION}, C++${CMAKE_CXX_STANDARD}"
)

# Live camera backend wired into the desktop composition root. Linux uses OpenCV's
# V4L2 backend on device nodes; other platforms use Qt Multimedia, which selects
# the native backend (AVFoundation on macOS). Qt remains selectable on Linux.
if(MICRO_ROBOT_PLATFORM STREQUAL "Linux")
    set(_micro_robot_camera_backend_default V4L2)
else()
    set(_micro_robot_camera_backend_default Qt)
endif()
set(MICRO_ROBOT_CAMERA_BACKEND "${_micro_robot_camera_backend_default}" CACHE STRING
    "Live camera backend of the desktop application: V4L2 (Linux only, needs OpenCV) or Qt")
set_property(CACHE MICRO_ROBOT_CAMERA_BACKEND PROPERTY STRINGS V4L2 Qt)
if(NOT MICRO_ROBOT_CAMERA_BACKEND MATCHES "^(V4L2|Qt)$")
    message(FATAL_ERROR
        "Micro Robot: MICRO_ROBOT_CAMERA_BACKEND must be V4L2 or Qt, not '${MICRO_ROBOT_CAMERA_BACKEND}'")
endif()
if(MICRO_ROBOT_CAMERA_BACKEND STREQUAL "V4L2" AND NOT MICRO_ROBOT_PLATFORM STREQUAL "Linux")
    message(FATAL_ERROR
        "Micro Robot: the V4L2 camera backend requires Linux; use -DMICRO_ROBOT_CAMERA_BACKEND=Qt")
endif()

# Common compile settings for first-party targets.
function(micro_robot_target_defaults target)
    target_compile_features(${target} PRIVATE cxx_std_${CMAKE_CXX_STANDARD})
    if(MSVC)
        target_compile_options(${target} PRIVATE /W4 /permissive-)
    else()
        target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic)
    endif()
endfunction()

# Platform packaging for the desktop GUI executable: an application bundle on
# macOS and a GUI-subsystem executable on Windows. Linux needs no extra settings.
function(micro_robot_configure_desktop_executable target info_plist)
    if(APPLE)
        set_target_properties(${target} PROPERTIES
            MACOSX_BUNDLE TRUE
            MACOSX_BUNDLE_BUNDLE_NAME "Micro Robot"
            MACOSX_BUNDLE_GUI_IDENTIFIER "org.microrobot.desktop"
            MACOSX_BUNDLE_BUNDLE_VERSION "${PROJECT_VERSION}"
            MACOSX_BUNDLE_SHORT_VERSION_STRING "${PROJECT_VERSION}"
            MACOSX_BUNDLE_INFO_PLIST "${info_plist}"
        )
    elseif(WIN32)
        set_target_properties(${target} PROPERTIES WIN32_EXECUTABLE TRUE)
    endif()
endfunction()

# On macOS Qt >= 6.5, camera authorization needs a statically imported native
# permission plugin as well as the usage description. Verify the production app,
# since hardware-free camera tests deliberately never request authorization.
function(micro_robot_add_desktop_packaging_tests target)
    if(APPLE AND Qt6Core_VERSION VERSION_GREATER_EQUAL 6.5)
        add_test(NAME DesktopPackaging.CameraPermission
            COMMAND "${CMAKE_COMMAND}"
                "-DAPP_BINARY=$<TARGET_FILE:${target}>"
                "-DAPP_PLIST=$<TARGET_BUNDLE_DIR:${target}>/Contents/Info.plist"
                "-DNM=${CMAKE_NM}"
                -P "${PROJECT_SOURCE_DIR}/tests/platform/VerifyMacCameraPermission.cmake"
        )
        set_tests_properties(DesktopPackaging.CameraPermission PROPERTIES LABELS "packaging")
        if(MICRO_ROBOT_BUILD_BLE)
            add_test(NAME DesktopPackaging.BluetoothPermission
                COMMAND "${CMAKE_COMMAND}"
                    "-DAPP_BINARY=$<TARGET_FILE:${target}>"
                    "-DAPP_PLIST=$<TARGET_BUNDLE_DIR:${target}>/Contents/Info.plist"
                    "-DNM=${CMAKE_NM}"
                    -P "${PROJECT_SOURCE_DIR}/tests/platform/VerifyMacBluetoothPermission.cmake")
            set_tests_properties(DesktopPackaging.BluetoothPermission PROPERTIES LABELS "packaging;ble")
        endif()
    endif()
endfunction()
