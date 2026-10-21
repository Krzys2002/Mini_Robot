# Starts the production desktop composition headlessly and lets it exit through the
# normal window-close shutdown path. Uses a private configuration file (file logging
# off) in a non-ASCII directory with spaces, passed relative to the working directory,
# so the run neither reads nor writes per-user files.
#
# Inputs: APP_BINARY (desktop executable), WORK_DIR (scratch directory).
set(config_dir "${WORK_DIR}/startup config ü")
file(REMOVE_RECURSE "${WORK_DIR}")
file(MAKE_DIRECTORY "${config_dir}")
file(WRITE "${config_dir}/micro_robot.ini"
    "[logging]\n"
    "level = debug\n"
    "file_enabled = false\n"
    "[robot]\n"
    "id = 7\n"
    "[simulation]\n"
    "update_interval_ms = 10\n"
)

set(ENV{QT_QPA_PLATFORM} offscreen)
execute_process(
    COMMAND "${APP_BINARY}" --exit-after-startup --config micro_robot.ini
    WORKING_DIRECTORY "${config_dir}"
    TIMEOUT 60
    RESULT_VARIABLE result
    OUTPUT_VARIABLE output
    ERROR_VARIABLE log
)
if(NOT result STREQUAL "0")
    message(FATAL_ERROR "Desktop exited with '${result}'.\nstdout:\n${output}\nstderr:\n${log}")
endif()

# Log records go to stderr; each fragment must appear after the previous one.
set(expected
    "[INFO] [app] Application starting (version "
    "[INFO] [config] Configuration loaded from "
    "[INFO] [app] File logging disabled by configuration"
    "[INFO] [controller] robot=7 Controller initialized; robot registered"
    "[INFO] [app] Application started"
    "[INFO] [app] Application shutting down"
    "[INFO] [app] Application shutdown complete (exit code 0)"
)
set(remaining "${log}")
foreach(fragment IN LISTS expected)
    string(FIND "${remaining}" "${fragment}" position)
    if(position EQUAL -1)
        message(FATAL_ERROR "Missing or out-of-order log record '${fragment}' in:\n${log}")
    endif()
    string(SUBSTRING "${remaining}" ${position} -1 remaining)
endforeach()

# The relative --config argument is resolved against the working directory.
if(NOT log MATCHES "Configuration loaded from [^\n]*startup config ü[/\\\\]micro_robot\\.ini\n")
    message(FATAL_ERROR "Configuration path was not resolved to the private file:\n${log}")
endif()
if(log MATCHES "\\[(WARNING|ERROR)\\] ")
    message(FATAL_ERROR "Startup reported warnings or errors:\n${log}")
endif()
