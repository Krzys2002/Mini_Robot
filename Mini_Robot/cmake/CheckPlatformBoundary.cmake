# Scans the sources of one platform-independent target for operating-system and
# device-backend dependencies. Run by CTest through MicroRobotBoundaries.cmake:
#
#   cmake -DMANIFEST=<build>/platform_boundaries/<target>.cmake -P CheckPlatformBoundary.cmake
#
# The manifest sets BOUNDARY_TARGET, BOUNDARY_ALLOW_QT, BOUNDARY_SOURCES, and
# BOUNDARY_BACKEND_HEADERS (concrete backend headers, relative to the include
# root, that the target must reach only through interfaces). Only preprocessor
# lines are inspected, so comments and documentation may mention platform names.
cmake_minimum_required(VERSION 3.24)

if(NOT MANIFEST OR NOT EXISTS "${MANIFEST}")
    message(FATAL_ERROR "CheckPlatformBoundary: MANIFEST '${MANIFEST}' does not exist")
endif()
include("${MANIFEST}")

# Operating-system/platform conditionals. Platform branches belong in adapters
# or the build configuration, never in platform-independent code.
set(platform_macro_pattern
    "(__APPLE__|__MACH__|TARGET_OS_[A-Z_]+|_WIN32|_WIN64|__CYGWIN__|__linux__|__linux|__unix__|__gnu_linux__|__ANDROID__)")

# Operating-system, native-framework, and device-backend headers.
set(include_prefix "^[ \t]*#[ \t]*(include|import)[ \t]*[<\"]")
set(forbidden_include_patterns
    # POSIX/Linux/Windows system headers.
    "${include_prefix}(unistd|fcntl|dlfcn|pthread|termios|dirent|poll|libudev|windows|winsock2|ws2tcpip|dshow|mfapi|mfidl|setupapi|bluetoothapis)\\.h[>\"]"
    "${include_prefix}(sys|linux|asm|mach|dispatch|objc|bluetooth|winrt)/"
    # Apple frameworks.
    "${include_prefix}(Foundation|CoreFoundation|AVFoundation|CoreMedia|CoreVideo|CoreBluetooth|IOKit|AppKit|Cocoa|ARKit)/"
    # Capture/communication backend libraries and their Qt modules. Other Qt
    # modules are governed by the target's allowed link dependencies.
    "${include_prefix}(opencv2/|opencv/|gst/|libavformat/|libavcodec/)"
    "${include_prefix}(QtMultimedia|QtBluetooth|QtSerialPort)[/>\"]"
    "${include_prefix}(QCamera|QMedia|QVideo|QImageCapture|QBluetooth|QLowEnergy|QSerialPort)[A-Za-z]*[>\"]"
)
if(NOT BOUNDARY_ALLOW_QT)
    list(APPEND forbidden_include_patterns "${include_prefix}Q[A-Za-z]*[/>\"]")
endif()

set(violations)
foreach(source IN LISTS BOUNDARY_SOURCES)
    if(source MATCHES "\\.(m|mm)$")
        list(APPEND violations "${source}: Objective-C source")
        continue()
    endif()
    if(NOT EXISTS "${source}")
        list(APPEND violations "${source}: listed source does not exist")
        continue()
    endif()
    file(STRINGS "${source}" directives REGEX "^[ \t]*#")
    foreach(directive IN LISTS directives)
        if(directive MATCHES "^[ \t]*#[ \t]*(if|ifdef|ifndef|elif|elifdef|elifndef|define|undef)[ \t(]"
           AND directive MATCHES "${platform_macro_pattern}")
            list(APPEND violations "${source}: platform conditional '${directive}'")
            continue()
        endif()
        if(directive MATCHES "^[ \t]*#[ \t]*include[ \t]*\"([^\"]+)\""
           AND CMAKE_MATCH_1 IN_LIST BOUNDARY_BACKEND_HEADERS)
            list(APPEND violations "${source}: concrete backend include '${directive}'")
            continue()
        endif()
        foreach(pattern IN LISTS forbidden_include_patterns)
            if(directive MATCHES "${pattern}")
                list(APPEND violations "${source}: platform or backend include '${directive}'")
                break()
            endif()
        endforeach()
    endforeach()
endforeach()

list(LENGTH BOUNDARY_SOURCES source_count)
if(violations)
    list(JOIN violations "\n  " details)
    message(FATAL_ERROR
        "${BOUNDARY_TARGET} must stay platform-independent and backend-neutral; move "
        "these into an adapter or the composition root:\n"
        "  ${details}")
endif()
message(STATUS "${BOUNDARY_TARGET}: ${source_count} source(s) platform-independent")
