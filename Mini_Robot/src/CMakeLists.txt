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
    "${CMAKE_CURRENT_SOURCE_DIR}/common/Angle.hpp"
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

# Capture policy/state remains independent of widgets, hardware and OpenCV.
add_library(micro_robot_calibration STATIC
    calibration/CalibrationCapture.hpp
    calibration/CalibrationCapture.cpp
    calibration/IntrinsicCalibration.hpp
    calibration/IntrinsicCalibration.cpp
    calibration/CalibrationProfile.hpp
    calibration/CalibrationProfile.cpp
    calibration/WorkspaceCalibration.hpp
    calibration/WorkspaceCalibration.cpp
    calibration/TabletopHomography.cpp
    calibration/TabletopHomography.hpp
    calibration/WorkspaceProfile.cpp
    calibration/WorkspaceProfile.hpp
)
target_link_libraries(micro_robot_calibration PUBLIC micro_robot_camera_api micro_robot_world)
micro_robot_target_defaults(micro_robot_calibration)
micro_robot_platform_boundary(micro_robot_calibration)

if(MICRO_ROBOT_BUILD_DESKTOP OR MICRO_ROBOT_BUILD_CALIBRATION)
    add_library(micro_robot_calibration_opencv STATIC
        calibration/OpenCvWorkspaceHomographySolver.cpp
        calibration/OpenCvWorkspaceHomographySolver.hpp
        calibration/OpenCvCalibrationDetector.hpp
        calibration/OpenCvCalibrationDetector.cpp
        calibration/OpenCvIntrinsicCalibrationSolver.hpp
        calibration/OpenCvIntrinsicCalibrationSolver.cpp
        calibration/OpenCvCalibrationUndistorter.hpp
        calibration/OpenCvCalibrationUndistorter.cpp
    )
    target_link_libraries(micro_robot_calibration_opencv PUBLIC micro_robot_calibration
        PRIVATE ${OpenCV_LIBS})
    target_include_directories(micro_robot_calibration_opencv SYSTEM PRIVATE ${OpenCV_INCLUDE_DIRS})
    micro_robot_target_defaults(micro_robot_calibration_opencv)
endif()

# Observation values and detector interface expose neither OpenCV nor Qt types.
add_library(micro_robot_vision_api INTERFACE)
target_link_libraries(micro_robot_vision_api INTERFACE micro_robot_camera_api)
target_sources(micro_robot_vision_api INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/vision/MarkerDetection.hpp")
micro_robot_platform_boundary(micro_robot_vision_api)
# Robot identity and pixel measurements from raw markers; no detector backend.
add_library(micro_robot_vision_observation STATIC
    vision/RobotObservation.hpp vision/RobotObservation.cpp
    vision/VisionDebugFrame.hpp vision/VisionDebugFrame.cpp)
target_link_libraries(micro_robot_vision_observation PUBLIC micro_robot_vision_api)
micro_robot_target_defaults(micro_robot_vision_observation)
micro_robot_platform_boundary(micro_robot_vision_observation)
if(MICRO_ROBOT_BUILD_DESKTOP OR MICRO_ROBOT_BUILD_VISION)
    add_library(micro_robot_vision_opencv STATIC
        vision/OpenCvAprilTagDetector.hpp vision/OpenCvAprilTagDetector.cpp)
    target_link_libraries(micro_robot_vision_opencv PUBLIC micro_robot_vision_api PRIVATE ${OpenCV_LIBS})
    target_include_directories(micro_robot_vision_opencv SYSTEM PRIVATE ${OpenCV_INCLUDE_DIRS})
    micro_robot_target_defaults(micro_robot_vision_opencv)
endif()

add_library(micro_robot_transport_api INTERFACE)
target_link_libraries(micro_robot_transport_api INTERFACE micro_robot_common)
target_sources(micro_robot_transport_api INTERFACE
    "${CMAKE_CURRENT_SOURCE_DIR}/transport/TransportTypes.hpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/transport/IRobotTransport.hpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/transport/IRobotDiscovery.hpp"
)
micro_robot_platform_boundary(micro_robot_transport_api)

add_library(micro_robot_ble_contract STATIC transport/ble/BleTypes.hpp transport/ble/BleTypes.cpp
    transport/ble/BleProtocol.hpp transport/ble/BleProtocol.cpp)
target_link_libraries(micro_robot_ble_contract PUBLIC micro_robot_transport_api)
micro_robot_target_defaults(micro_robot_ble_contract)
micro_robot_platform_boundary(micro_robot_ble_contract)

if(MICRO_ROBOT_BUILD_BLE)
    add_library(micro_robot_ble_lifecycle STATIC
        transport/ble/IBleBackend.hpp
        transport/ble/BleTransport.hpp
        transport/ble/BleTransport.cpp)
    target_sources(micro_robot_ble_lifecycle PRIVATE transport/ble/BleTransportProtocol.cpp)
    set_target_properties(micro_robot_ble_lifecycle PROPERTIES AUTOMOC ON)
    target_link_libraries(micro_robot_ble_lifecycle PUBLIC micro_robot_ble_contract micro_robot_logging Qt6::Core)
    micro_robot_target_defaults(micro_robot_ble_lifecycle)
    micro_robot_platform_boundary(micro_robot_ble_lifecycle BACKEND ALLOW Qt6::Core)

    add_library(micro_robot_ble_qt STATIC transport/ble/QtBleBackend.hpp transport/ble/QtBleBackend.cpp)
    target_link_libraries(micro_robot_ble_qt PUBLIC micro_robot_ble_lifecycle PRIVATE Qt6::Bluetooth)
    micro_robot_target_defaults(micro_robot_ble_qt)
endif()

add_library(micro_robot_control STATIC
    control/DifferentialDrive.hpp
    control/DifferentialDrive.cpp
)
target_link_libraries(micro_robot_control PUBLIC micro_robot_common micro_robot_transport_api)
micro_robot_target_defaults(micro_robot_control)
micro_robot_platform_boundary(micro_robot_control)

add_library(micro_robot_world STATIC
    world/RobotState.hpp
    world/WorldSnapshot.hpp
    world/WorldModel.hpp
    world/WorldModel.cpp
    world/Workspace.hpp
    world/Workspace.cpp
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

# Transport-independent session/application service; no Qt or radio APIs.
add_library(micro_robot_robot_manager STATIC app/RobotManager.hpp app/RobotManager.cpp)
target_link_libraries(micro_robot_robot_manager PUBLIC
    micro_robot_transport_api micro_robot_world micro_robot_logging)
micro_robot_target_defaults(micro_robot_robot_manager)
micro_robot_platform_boundary(micro_robot_robot_manager)

add_library(micro_robot_configuration STATIC
    infrastructure/configuration/ApplicationConfig.hpp
    infrastructure/configuration/ConfigLoader.hpp
    infrastructure/configuration/ConfigLoader.cpp
)
target_link_libraries(micro_robot_configuration PUBLIC
    micro_robot_common micro_robot_camera_api micro_robot_logging
)
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
    add_library(micro_robot_calibration_profiles STATIC
        calibration/QtWorkspaceProfileStore.cpp
        calibration/QtWorkspaceProfileStore.hpp
        calibration/QtCalibrationProfileStore.hpp
        calibration/QtCalibrationProfileStore.cpp)
    target_link_libraries(micro_robot_calibration_profiles PUBLIC micro_robot_calibration Qt6::Core)
    micro_robot_target_defaults(micro_robot_calibration_profiles)

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

    if(MICRO_ROBOT_CAMERA_BACKEND STREQUAL "V4L2")
        # Linux inventory from V4L2 device nodes; capture through OpenCV's V4L2 backend.
        # Qt Core provides the owner-thread event delivery and the capture worker.
        add_library(micro_robot_camera_v4l2 STATIC
            camera/V4l2Device.hpp
            camera/V4l2Device.cpp
        camera/V4l2Identity.cpp camera/V4l2Identity.hpp
            camera/V4l2CameraDeviceDiscovery.hpp
            camera/V4l2CameraDeviceDiscovery.cpp
            camera/V4l2CameraSource.hpp
            camera/V4l2CameraSource.cpp
        )
        target_link_libraries(micro_robot_camera_v4l2
            PUBLIC micro_robot_camera_api Qt6::Core
            PRIVATE ${OpenCV_LIBS}
        )
        target_include_directories(micro_robot_camera_v4l2 SYSTEM PRIVATE ${OpenCV_INCLUDE_DIRS})
        micro_robot_target_defaults(micro_robot_camera_v4l2)
        set(MICRO_ROBOT_LIVE_CAMERA_TARGET micro_robot_camera_v4l2)
    else()
        set(MICRO_ROBOT_LIVE_CAMERA_TARGET micro_robot_camera_discovery)
    endif()

    add_library(micro_robot_desktop_controller STATIC
        app/VisionController.hpp
        app/VisionController.cpp
        app/WorkspaceCalibrationController.cpp
        app/WorkspaceCalibrationController.hpp
        app/CalibrationController.hpp
        app/CalibrationController.cpp
        app/CalibrationControllerProfiles.cpp
        app/DesktopController.hpp
        app/DesktopController.cpp
        app/CameraSelectionController.hpp
        app/CameraSelectionController.cpp
    )
    set_target_properties(micro_robot_desktop_controller PROPERTIES AUTOMOC ON)
    target_link_libraries(micro_robot_desktop_controller PUBLIC
        micro_robot_camera_api micro_robot_camera_selection micro_robot_transport_api
        micro_robot_world micro_robot_logging micro_robot_calibration micro_robot_vision_api
        micro_robot_vision_observation micro_robot_robot_manager Qt6::Core
    )
    micro_robot_target_defaults(micro_robot_desktop_controller)
    # Application adapters use Qt Core only; device backends stay in adapter targets.
    micro_robot_platform_boundary(micro_robot_desktop_controller ALLOW Qt6::Core)

    add_library(micro_robot_ui STATIC
        ui/WorkspaceCalibrationDialog.cpp
        ui/WorkspaceCalibrationDialog.hpp
        ui/CalibrationWizard.hpp
        ui/CalibrationWizard.cpp
        ui/MainWindow.cpp
        ui/MainWindow.hpp
        ui/VisionDebugView.hpp
        ui/VisionDebugView.cpp
    )
    set_target_properties(micro_robot_ui PROPERTIES AUTOMOC ON)
    target_include_directories(micro_robot_ui PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
    target_link_libraries(micro_robot_ui PUBLIC Qt6::Widgets micro_robot_desktop_controller)
    micro_robot_target_defaults(micro_robot_ui)
    micro_robot_platform_boundary(micro_robot_ui ALLOW Qt6::Core Qt6::Widgets)

    # Qt finalization imports the macOS permission plugins selected by Info.plist.
    qt_add_executable(micro_robot_desktop app/main.cpp)
    target_link_libraries(micro_robot_desktop PRIVATE
        micro_robot_ui micro_robot_simulation ${MICRO_ROBOT_LIVE_CAMERA_TARGET} micro_robot_calibration_opencv
        micro_robot_logging micro_robot_configuration micro_robot_calibration_profiles micro_robot_vision_opencv
    )
    target_compile_definitions(micro_robot_desktop PRIVATE
        MICRO_ROBOT_VERSION="${PROJECT_VERSION}"
        $<$<STREQUAL:${MICRO_ROBOT_CAMERA_BACKEND},V4L2>:MICRO_ROBOT_CAMERA_BACKEND_V4L2>
    )
    if(MICRO_ROBOT_BUILD_BLE)
        target_link_libraries(micro_robot_desktop PRIVATE micro_robot_ble_qt)
        target_compile_definitions(micro_robot_desktop PRIVATE MICRO_ROBOT_HAS_BLE)
    endif()
    micro_robot_configure_desktop_executable(micro_robot_desktop
        "${CMAKE_CURRENT_SOURCE_DIR}/app/Info.plist.in"
    )
    micro_robot_target_defaults(micro_robot_desktop)
endif()
