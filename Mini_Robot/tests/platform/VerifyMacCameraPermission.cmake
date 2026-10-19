# Inspect the production app without starting capture or requesting permission.
execute_process(
    COMMAND /usr/libexec/PlistBuddy -c "Print :NSCameraUsageDescription" "${APP_PLIST}"
    RESULT_VARIABLE plist_result
    OUTPUT_VARIABLE usage_description
    ERROR_VARIABLE plist_error
    OUTPUT_STRIP_TRAILING_WHITESPACE
)
if(NOT plist_result EQUAL 0 OR usage_description STREQUAL "")
    message(FATAL_ERROR "Camera usage description is missing: ${plist_error}")
endif()

# Darwin permission plugins are static even in shared Qt installations. The
# native plugin must be linked into this executable by Qt target finalization.
execute_process(
    COMMAND "${NM}" "${APP_BINARY}"
    RESULT_VARIABLE nm_result
    OUTPUT_VARIABLE symbols
    ERROR_VARIABLE nm_error
)
if(NOT nm_result EQUAL 0)
    message(FATAL_ERROR "Could not inspect desktop executable: ${nm_error}")
endif()
if(NOT symbols MATCHES "[ \t][Tt][ \t]+[^\n]*qt_static_plugin_QDarwinCameraPermissionPlugin")
    message(FATAL_ERROR
        "Camera permission plugin is missing from the desktop executable. "
        "Create the target with qt_add_executable() so Qt imports the plugin from Info.plist."
    )
endif()
