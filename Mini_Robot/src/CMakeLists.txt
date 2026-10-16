# Targets registered with micro_robot_platform_boundary() are platform-independent:
# no OS APIs or device-backend libraries, and no dependency on concrete backends
# (cmake/MicroRobotBoundaries.cmake). Platform adapters and the composition root
# are deliberately not registered.
add_library(micro_robot_common INTERFACE)
target_include_directories(micro_robot_common INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}")
target_compile_features(micro_robot_common INTERFACE cxx_std_${CMAKE_CXX_STANDARD})
target_sources(micro_robot_common INTERFACE
    "${CMAKE_CURRENT_SOURCE_DIR}/common/Types.hpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/common/RobotId.hpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/common/Pose2D.hpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/common/Units.hpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/common/Time.hpp"
)
micro_robot_platform_boundary(micro_robot_common)

add_library(micro_robot_camera_api INTERFACE)
target_link_libraries(micro_robot_camera_api INTERFACE micro_robot_common)
target_sources(micro_robot_camera_api INTERFACE
    "${CMAKE_CURRENT_SOURCE_DIR}/camera/CameraTypes.hpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/camera/ICameraSource.hpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/camera/ICameraDeviceDiscovery.hpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/camera/ICameraSourceFactory.hpp"
)
micro_robot_platform_boundary(micro_robot_camera_api)

# Qt-free selection rules shared by presentation code and core tests.
add_library(micro_robot_camera_selection STATIC
    camera/CameraSelection.hpp
    camera/CameraSelection.cpp
)
target_link_libraries(micro_robot_camera_selection PUBLIC micro_robot_camera_api)
micro_robot_target_defaults(micro_robot_camera_selection)
micro_robot_platform_boundary(micro_robot_camera_selection)

add_library(micro_robot_transport_api INTERFACE)
target_link_libraries(micro_robot_transport_api INTERFACE micro_robot_common)
target_sources(micro_robot_transport_api INTERFACE
    "${CMAKE_CURRENT_SOURCE_DIR}/transport/TransportTypes.hpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/transport/IRobotTransport.hpp"
)
micro_robot_platform_boundary(micro_robot_transport_api)

add_library(micro_robot_world STATIC
    world/RobotState.hpp
    world/WorldSnapshot.hpp
    world/WorldModel.hpp
    world/WorldModel.cpp
)
target_link_libraries(micro_robot_world PUBLIC micro_robot_common)
micro_robot_target_defaults(micro_robot_world)
micro_robot_platform_boundary(micro_robot_world)

add_library(micro_robot_logging STATIC
    infrastructure/logging/LogRecord.hpp
    infrastructure/logging/Logger.hpp
    infrastructure/logging/Logger.cpp
    infrastructure/logging/LogSinks.hpp
    infrastructure/logging/LogSinks.cpp
)
target_link_libraries(micro_robot_logging PUBLIC micro_robot_common)
micro_robot_target_defaults(micro_robot_logging)
micro_robot_platform_boundary(micro_robot_logging)

add_library(micro_robot_configuration STATIC
    infrastructure/configuration/ApplicationConfig.hpp
    infrastructure/configuration/ConfigLoader.hpp
    infrastructure/configuration/ConfigLoader.cpp
)
target_link_libraries(micro_robot_configuration PUBLIC micro_robot_common micro_robot_logging)
micro_robot_target_defaults(micro_robot_configuration)
micro_robot_platform_boundary(micro_robot_configuration)

add_library(micro_robot_simulation STATIC
    simulation/FakeCameraSource.hpp
    simulation/FakeCameraSource.cpp
    simulation/FakeCameraSourceFactory.hpp
    simulation/FakeCameraSourceFactory.cpp
    simulation/SimulationTransport.hpp
    simulation/SimulationTransport.cpp
)
target_link_libraries(micro_robot_simulation PUBLIC
    micro_robot_camera_api micro_robot_transport_api micro_robot_logging
)
micro_robot_target_defaults(micro_robot_simulation)
# Platform-independent, but a concrete backend: selected by composition/tests only.
micro_robot_platform_boundary(micro_robot_simulation BACKEND)

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
        camera/QtCameraSource.hpp
        camera/QtCameraSource.cpp
    )
    target_link_libraries(micro_robot_camera_discovery PUBLIC
        micro_robot_camera_api Qt6::Multimedia
    )
    micro_robot_target_defaults(micro_robot_camera_discovery)

    add_library(micro_robot_desktop_controller STATIC
        app/DesktopController.hpp
        app/DesktopController.cpp
        app/CameraSelectionController.hpp
        app/CameraSelectionController.cpp
    )
    set_target_properties(micro_robot_desktop_controller PROPERTIES AUTOMOC ON)
    target_link_libraries(micro_robot_desktop_controller PUBLIC
        micro_robot_camera_api micro_robot_camera_selection micro_robot_transport_api
        micro_robot_world micro_robot_logging Qt6::Core
    )
    micro_robot_target_defaults(micro_robot_desktop_controller)
    # Application adapters use Qt Core only; device backends stay in adapter targets.
    micro_robot_platform_boundary(micro_robot_desktop_controller ALLOW Qt6::Core)

    add_library(micro_robot_ui STATIC
        ui/MainWindow.cpp
        ui/MainWindow.hpp
    )
    set_target_properties(micro_robot_ui PROPERTIES AUTOMOC ON)
    target_include_directories(micro_robot_ui PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
    target_link_libraries(micro_robot_ui PUBLIC Qt6::Widgets micro_robot_desktop_controller)
    micro_robot_target_defaults(micro_robot_ui)
    micro_robot_platform_boundary(micro_robot_ui ALLOW Qt6::Core Qt6::Widgets)

    add_executable(micro_robot_desktop app/main.cpp)
    target_link_libraries(micro_robot_desktop PRIVATE
        micro_robot_ui micro_robot_simulation micro_robot_camera_discovery
        micro_robot_logging micro_robot_configuration
    )
    target_compile_definitions(micro_robot_desktop PRIVATE
        MICRO_ROBOT_VERSION="${PROJECT_VERSION}"
    )
    micro_robot_configure_desktop_executable(micro_robot_desktop
        "${CMAKE_CURRENT_SOURCE_DIR}/app/Info.plist.in"
    )
    micro_robot_target_defaults(micro_robot_desktop)
endif()
