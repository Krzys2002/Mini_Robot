# Platform-boundary enforcement for first-party targets.
#
# Targets registered with micro_robot_platform_boundary() form the
# platform-independent part of the application. Configuration fails when one of
# them links a third-party or system library that it has not explicitly allowed,
# directly or through another first-party target. Registered targets that are not
# backends must also not depend on a concrete backend, so backends stay
# replaceable behind their interfaces. CTest additionally scans registered
# sources for operating-system headers/macros, device-backend APIs, and includes
# of concrete backend headers. Platform adapters (Qt Multimedia, OpenCV, future
# BLE) and the composition root are not registered: they are the only places
# where such dependencies belong. See docs/overview/platform-boundaries.md.
include_guard(GLOBAL)

set(_MICRO_ROBOT_BOUNDARY_SCRIPT "${CMAKE_CURRENT_LIST_DIR}/CheckPlatformBoundary.cmake")

# micro_robot_platform_boundary(<target> [BACKEND] [ALLOW <imported-target>...])
#
# BACKEND marks a platform-independent implementation of an interface, such as a
# simulation backend: only the composition root and tests may depend on it.
# ALLOW lists the only non-first-party link dependencies the target may use, for
# example Qt6::Core for a Qt application adapter. Without ALLOW, the target and
# its first-party dependencies may use the C++ standard library only. Qt headers
# are permitted in sources only when a Qt target is allowed.
function(micro_robot_platform_boundary target)
    cmake_parse_arguments(PARSE_ARGV 1 arg "BACKEND" "" "ALLOW")
    if(arg_UNPARSED_ARGUMENTS)
        message(FATAL_ERROR "micro_robot_platform_boundary: unexpected '${arg_UNPARSED_ARGUMENTS}'")
    endif()
    set_property(TARGET ${target} PROPERTY MICRO_ROBOT_BOUNDARY_ALLOW "${arg_ALLOW}")
    set_property(TARGET ${target} PROPERTY MICRO_ROBOT_BOUNDARY_BACKEND ${arg_BACKEND})
    set_property(GLOBAL APPEND PROPERTY MICRO_ROBOT_BOUNDARY_TARGETS ${target})
endfunction()

# Walks link items through first-party (non-imported) targets. Imported targets
# and plain library names are reported in out_external but not expanded; every
# first-party target reached is reported in out_internal.
function(_micro_robot_collect_links target out_internal out_external)
    set(pending ${target})
    set(visited)
    set(external)
    while(pending)
        list(POP_FRONT pending current)
        if(current IN_LIST visited)
            continue()
        endif()
        list(APPEND visited ${current})
        set(items)
        foreach(property IN ITEMS LINK_LIBRARIES INTERFACE_LINK_LIBRARIES)
            get_target_property(values ${current} ${property})
            if(values)
                list(APPEND items ${values})
            endif()
        endforeach()
        foreach(item IN LISTS items)
            string(REGEX REPLACE "^\\$<LINK_ONLY:(.+)>$" "\\1" item "${item}")
            if(TARGET ${item})
                get_target_property(aliased ${item} ALIASED_TARGET)
                if(aliased)
                    set(item ${aliased})
                endif()
                get_target_property(imported ${item} IMPORTED)
                if(NOT imported)
                    list(APPEND pending ${item})
                    continue()
                endif()
            endif()
            list(APPEND external ${item})
        endforeach()
    endwhile()
    list(REMOVE_ITEM visited ${target})
    list(REMOVE_DUPLICATES external)
    set(${out_internal} "${visited}" PARENT_SCOPE)
    set(${out_external} "${external}" PARENT_SCOPE)
endfunction()

# Absolute paths of a target's listed sources, excluding generator expressions.
function(_micro_robot_target_sources target out_var)
    get_target_property(source_dir ${target} SOURCE_DIR)
    set(sources)
    foreach(property IN ITEMS SOURCES INTERFACE_SOURCES)
        get_target_property(values ${target} ${property})
        if(NOT values)
            continue()
        endif()
        foreach(source IN LISTS values)
            if(source MATCHES "^\\$<")
                continue()
            endif()
            cmake_path(ABSOLUTE_PATH source BASE_DIRECTORY "${source_dir}" NORMALIZE)
            list(APPEND sources "${source}")
        endforeach()
    endforeach()
    list(REMOVE_DUPLICATES sources)
    set(${out_var} "${sources}" PARENT_SCOPE)
endfunction()

# Targets defined in a directory and, recursively, in its subdirectories.
function(_micro_robot_directory_targets directory out_var)
    get_property(found DIRECTORY "${directory}" PROPERTY BUILDSYSTEM_TARGETS)
    get_property(children DIRECTORY "${directory}" PROPERTY SUBDIRECTORIES)
    foreach(child IN LISTS children)
        _micro_robot_directory_targets("${child}" child_targets)
        list(APPEND found ${child_targets})
    endforeach()
    set(${out_var} "${found}" PARENT_SCOPE)
endfunction()

# A first-party target is a concrete backend unless it is registered without BACKEND.
function(_micro_robot_is_backend target out_var)
    get_property(targets GLOBAL PROPERTY MICRO_ROBOT_BOUNDARY_TARGETS)
    set(is_backend ON)
    if(target IN_LIST targets)
        get_target_property(is_backend ${target} MICRO_ROBOT_BOUNDARY_BACKEND)
    endif()
    set(${out_var} ${is_backend} PARENT_SCOPE)
endfunction()

# micro_robot_check_platform_boundaries(<source-root>)
#
# Validates link dependencies of every registered target and writes the source
# manifests used by the CTest scan. Every first-party dependency is classified by
# its own registration, wherever it is defined: unregistered targets are treated
# as platform adapters. <source-root> is the include root for first-party headers;
# headers of backends defined in it or its subdirectories are forbidden includes.
# Call after all targets have been defined.
function(micro_robot_check_platform_boundaries source_root)
    get_property(targets GLOBAL PROPERTY MICRO_ROBOT_BOUNDARY_TARGETS)
    _micro_robot_directory_targets("${source_root}" all_targets)

    set(backend_headers)
    foreach(target IN LISTS all_targets)
        _micro_robot_is_backend(${target} is_backend)
        if(NOT is_backend)
            continue()
        endif()
        _micro_robot_target_sources(${target} sources)
        foreach(source IN LISTS sources)
            if(source MATCHES "\\.(h|hh|hpp|hxx)$")
                file(RELATIVE_PATH header "${source_root}" "${source}")
                list(APPEND backend_headers "${header}")
            endif()
        endforeach()
    endforeach()

    set(violations)
    foreach(target IN LISTS targets)
        get_target_property(allowed ${target} MICRO_ROBOT_BOUNDARY_ALLOW)
        if(NOT allowed)
            set(allowed)
        endif()
        get_target_property(is_backend ${target} MICRO_ROBOT_BOUNDARY_BACKEND)
        _micro_robot_collect_links(${target} internal external)
        foreach(link IN LISTS external)
            if(NOT link IN_LIST allowed)
                list(APPEND violations "  ${target} -> ${link} (platform or third-party library)")
            endif()
        endforeach()
        set(forbidden_headers)
        if(NOT is_backend)
            set(forbidden_headers "${backend_headers}")
            foreach(dependency IN LISTS internal)
                _micro_robot_is_backend(${dependency} dependency_is_backend)
                if(dependency_is_backend)
                    list(APPEND violations
                        "  ${target} -> ${dependency} (concrete backend or unregistered target)")
                endif()
            endforeach()
        endif()

        set(allow_qt OFF)
        foreach(dependency IN LISTS allowed)
            if(dependency MATCHES "^Qt[0-9]*::")
                set(allow_qt ON)
            endif()
        endforeach()
        _micro_robot_target_sources(${target} sources)
        file(WRITE "${PROJECT_BINARY_DIR}/platform_boundaries/${target}.cmake"
            "set(BOUNDARY_TARGET [==[${target}]==])\n"
            "set(BOUNDARY_ALLOW_QT ${allow_qt})\n"
            "set(BOUNDARY_SOURCES [==[${sources}]==])\n"
            "set(BOUNDARY_BACKEND_HEADERS [==[${forbidden_headers}]==])\n")
    endforeach()
    if(violations)
        list(JOIN violations "\n" details)
        message(FATAL_ERROR
            "Platform-independent targets must not depend on platform libraries or concrete "
            "backends. Move the dependency into an adapter or the composition root, list a "
            "portable library with micro_robot_platform_boundary(... ALLOW ...), or register a "
            "new platform-independent target with micro_robot_platform_boundary():\n${details}")
    endif()
endfunction()

# Registers one CTest per boundary target that scans its sources.
function(micro_robot_add_platform_boundary_tests)
    get_property(targets GLOBAL PROPERTY MICRO_ROBOT_BOUNDARY_TARGETS)
    foreach(target IN LISTS targets)
        add_test(NAME PlatformBoundary.${target}
            COMMAND "${CMAKE_COMMAND}"
                "-DMANIFEST=${PROJECT_BINARY_DIR}/platform_boundaries/${target}.cmake"
                -P "${_MICRO_ROBOT_BOUNDARY_SCRIPT}")
        set_tests_properties(PlatformBoundary.${target} PROPERTIES LABELS "core;boundary")
    endforeach()
endfunction()
