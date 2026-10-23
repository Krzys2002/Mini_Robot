# Inspect the bundle without starting discovery or requesting permission.
execute_process(
    COMMAND /usr/libexec/PlistBuddy -c "Print :NSBluetoothAlwaysUsageDescription" "${APP_PLIST}"
    RESULT_VARIABLE plist_result OUTPUT_VARIABLE description ERROR_VARIABLE plist_error
    OUTPUT_STRIP_TRAILING_WHITESPACE)
if(NOT plist_result EQUAL 0 OR description STREQUAL "")
    message(FATAL_ERROR "Bluetooth usage description is missing: ${plist_error}")
endif()
execute_process(COMMAND "${NM}" "${APP_BINARY}"
    RESULT_VARIABLE nm_result OUTPUT_VARIABLE symbols ERROR_VARIABLE nm_error)
if(NOT nm_result EQUAL 0)
    message(FATAL_ERROR "Could not inspect desktop executable: ${nm_error}")
endif()
if(NOT symbols MATCHES "[ \t][Tt][ \t]+[^\n]*qt_static_plugin_QDarwinBluetoothPermissionPlugin")
    message(FATAL_ERROR "Darwin Bluetooth permission plugin is missing from the production executable")
endif()
