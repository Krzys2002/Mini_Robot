add_library(micro_robot_common INTERFACE)
target_include_directories(micro_robot_common INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}")
target_compile_features(micro_robot_common INTERFACE cxx_std_20)
target_sources(micro_robot_common INTERFACE
    "${CMAKE_CURRENT_SOURCE_DIR}/common/Types.hpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/common/RobotId.hpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/common/Pose2D.hpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/common/Units.hpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/common/Time.hpp"
)

add_library(micro_robot_camera_api INTERFACE)
target_link_libraries(micro_robot_camera_api INTERFACE micro_robot_common)
target_sources(micro_robot_camera_api INTERFACE
    "${CMAKE_CURRENT_SOURCE_DIR}/camera/CameraTypes.hpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/camera/ICameraSource.hpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/camera/ICameraDeviceDiscovery.hpp"
)

add_library(micro_robot_transport_api INTERFACE)
target_link_libraries(micro_robot_transport_api INTERFACE micro_robot_common)
target_sources(micro_robot_transport_api INTERFACE
    "${CMAKE_CURRENT_SOURCE_DIR}/transport/TransportTypes.hpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/transport/IRobotTransport.hpp"
)

add_library(micro_robot_world STATIC
    world/RobotState.hpp
    world/WorldSnapshot.hpp
    world/WorldModel.hpp
    world/WorldModel.cpp
)
target_link_libraries(micro_robot_world PUBLIC micro_robot_common)
micro_robot_target_defaults(micro_robot_world)

add_library(micro_robot_logging STATIC
    infrastructure/logging/LogRecord.hpp
    infrastructure/logging/Logger.hpp
    infrastructure/logging/Logger.cpp
    infrastructure/logging/LogSinks.hpp
    infrastructure/logging/LogSinks.cpp
)
target_link_libraries(micro_robot_logging PUBLIC micro_robot_common)
micro_robot_target_defaults(micro_robot_logging)

add_library(micro_robot_configuration STATIC
    infrastructure/configuration/ApplicationConfig.hpp
    infrastructure/configuration/ConfigLoader.hpp
    infrastructure/configuration/ConfigLoader.cpp
)
target_link_libraries(micro_robot_configuration PUBLIC micro_robot_common micro_robot_logging)
micro_robot_target_defaults(micro_robot_configuration)

add_library(micro_robot_simulation STATIC
    simulation/FakeCameraSource.hpp
    simulation/FakeCameraSource.cpp
    simulation/SimulationTransport.hpp
    simulation/SimulationTransport.cpp
)
target_link_libraries(micro_robot_simulation PUBLIC
    micro_robot_camera_api micro_robot_transport_api micro_robot_logging
)
micro_robot_target_defaults(micro_robot_simulation)

if(MICRO_ROBOT_BUILD_REPLAY)
    add_library(micro_robot_camera_replay STATIC
        camera/RecordedVideoSource.hpp
        camera/RecordedVideoSource.cpp
    )
    target_link_libraries(micro_robot_camera_replay PUBLIC micro_robot_camera_api
        PRIVATE ${OpenCV_LIBS}
    )
    target_include_directories(micro_robot_camera_replay SYSTEM PRIVATE ${OpenCV_INCLUDE_DIRS})
    micro_robot_target_defaults(micro_robot_camera_replay)
endif()

if(MICRO_ROBOT_BUILD_DESKTOP)
    add_library(micro_robot_camera_discovery STATIC
        camera/QtCameraDeviceDiscovery.hpp
        camera/QtCameraDeviceDiscovery.cpp
    )
    target_link_libraries(micro_robot_camera_discovery PUBLIC
        micro_robot_camera_api Qt6::Multimedia
    )
    micro_robot_target_defaults(micro_robot_camera_discovery)

    add_library(micro_robot_desktop_controller STATIC
        app/DesktopController.hpp
        app/DesktopController.cpp
        app/CameraDiscoveryController.hpp
        app/CameraDiscoveryController.cpp
    )
    set_target_properties(micro_robot_desktop_controller PROPERTIES AUTOMOC ON)
    target_link_libraries(micro_robot_desktop_controller PUBLIC
        micro_robot_camera_api micro_robot_transport_api micro_robot_world micro_robot_logging
        Qt6::Core
    )
    micro_robot_target_defaults(micro_robot_desktop_controller)

    add_library(micro_robot_ui STATIC
        ui/MainWindow.cpp
        ui/MainWindow.hpp
    )
    set_target_properties(micro_robot_ui PROPERTIES AUTOMOC ON)
    target_include_directories(micro_robot_ui PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
    target_link_libraries(micro_robot_ui PUBLIC Qt6::Widgets micro_robot_desktop_controller)
    micro_robot_target_defaults(micro_robot_ui)

    add_executable(micro_robot_desktop WIN32 MACOSX_BUNDLE app/main.cpp)
    target_link_libraries(micro_robot_desktop PRIVATE
        micro_robot_ui micro_robot_simulation micro_robot_camera_discovery
        micro_robot_logging micro_robot_configuration
    )
    target_compile_definitions(micro_robot_desktop PRIVATE
        MICRO_ROBOT_VERSION="${PROJECT_VERSION}"
    )
    set_target_properties(micro_robot_desktop PROPERTIES
        MACOSX_BUNDLE_BUNDLE_NAME "Micro Robot"
        MACOSX_BUNDLE_GUI_IDENTIFIER "org.microrobot.desktop"
        MACOSX_BUNDLE_BUNDLE_VERSION "${PROJECT_VERSION}"
        MACOSX_BUNDLE_SHORT_VERSION_STRING "${PROJECT_VERSION}"
        MACOSX_BUNDLE_INFO_PLIST "${CMAKE_CURRENT_SOURCE_DIR}/app/Info.plist.in"
    )
    micro_robot_target_defaults(micro_robot_desktop)
endif()
