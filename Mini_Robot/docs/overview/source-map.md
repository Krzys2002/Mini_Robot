# Source and documentation map

This map describes the implemented source tree. Planning catalogs and archived
reports are scope/history, not evidence that future modules are present.

| Source file | Behavior guide |
| --- | --- |
| [src/app/CameraSelectionController.cpp](../../src/app/CameraSelectionController.cpp) | [Camera selection](camera-selection.md) |
| [src/app/CameraSelectionController.hpp](../../src/app/CameraSelectionController.hpp) | [Camera selection](camera-selection.md) |
| [src/app/Info.plist.in](../../src/app/Info.plist.in) | [macOS discovery metadata](camera-discovery.md#macos-and-verification) |
| [src/app/DesktopController.cpp](../../src/app/DesktopController.cpp) | [app](desktop-demo.md) |
| [src/app/DesktopController.hpp](../../src/app/DesktopController.hpp) | [app](desktop-demo.md) |
| [src/app/CalibrationController.hpp](../../src/app/CalibrationController.hpp) | [Calibration capture](calibration-capture.md) |
| [src/app/CalibrationController.cpp](../../src/app/CalibrationController.cpp) | [Calibration capture](calibration-capture.md) |
| [src/calibration/CalibrationCapture.hpp](../../src/calibration/CalibrationCapture.hpp) | [Calibration capture](calibration-capture.md) |
| [src/calibration/CalibrationCapture.cpp](../../src/calibration/CalibrationCapture.cpp) | [Calibration capture](calibration-capture.md) |
| [src/calibration/OpenCvCalibrationDetector.hpp](../../src/calibration/OpenCvCalibrationDetector.hpp) | [Calibration capture](calibration-capture.md) |
| [src/calibration/OpenCvCalibrationDetector.cpp](../../src/calibration/OpenCvCalibrationDetector.cpp) | [Calibration capture](calibration-capture.md) |
| [src/ui/CalibrationWizard.hpp](../../src/ui/CalibrationWizard.hpp) | [Calibration capture](calibration-capture.md) |
| [src/ui/CalibrationWizard.cpp](../../src/ui/CalibrationWizard.cpp) | [Calibration capture](calibration-capture.md) |
| [src/app/main.cpp](../../src/app/main.cpp) | [app](desktop-demo.md) |
| [src/camera/CameraSelection.cpp](../../src/camera/CameraSelection.cpp) | [Selection rules](camera-selection.md#architecture-and-api) |
| [src/camera/CameraSelection.hpp](../../src/camera/CameraSelection.hpp) | [Selection rules](camera-selection.md#architecture-and-api) |
| [src/camera/CameraTypes.hpp](../../src/camera/CameraTypes.hpp) | [camera](camera-source.md) |
| [src/camera/ICameraSource.hpp](../../src/camera/ICameraSource.hpp) | [camera](camera-source.md) |
| [src/camera/ICameraDeviceDiscovery.hpp](../../src/camera/ICameraDeviceDiscovery.hpp) | [Discovery contract](camera-discovery.md) |
| [src/camera/ICameraSourceFactory.hpp](../../src/camera/ICameraSourceFactory.hpp) | [Source factory](camera-selection.md#architecture-and-api) |
| [src/camera/QtCameraSource.hpp](../../src/camera/QtCameraSource.hpp) | [Native camera streaming](camera-streaming.md) |
| [src/camera/QtCameraSource.cpp](../../src/camera/QtCameraSource.cpp) | [Native camera streaming](camera-streaming.md) |
| [src/camera/QtCameraDeviceDiscovery.cpp](../../src/camera/QtCameraDeviceDiscovery.cpp) | [Qt discovery adapter](camera-discovery.md) |
| [src/camera/QtCameraDeviceDiscovery.hpp](../../src/camera/QtCameraDeviceDiscovery.hpp) | [Qt discovery adapter](camera-discovery.md) |
| [src/camera/RecordedVideoSource.cpp](../../src/camera/RecordedVideoSource.cpp) | [Replay adapter](recorded-video.md) |
| [src/camera/RecordedVideoSource.hpp](../../src/camera/RecordedVideoSource.hpp) | [Replay configuration and driving](recorded-video.md) |
| [src/common/Pose2D.hpp](../../src/common/Pose2D.hpp) | [common](architecture.md#domain-conventions) |
| [src/common/RobotId.hpp](../../src/common/RobotId.hpp) | [common](architecture.md#domain-conventions) |
| [src/common/Time.hpp](../../src/common/Time.hpp) | [common](architecture.md#domain-conventions) |
| [src/common/Types.hpp](../../src/common/Types.hpp) | [common](architecture.md#domain-conventions) |
| [src/common/Units.hpp](../../src/common/Units.hpp) | [common](architecture.md#domain-conventions) |
| [src/control/DifferentialDrive.hpp](../../src/control/DifferentialDrive.hpp) | [Differential-drive model](differential-drive.md) |
| [src/control/DifferentialDrive.cpp](../../src/control/DifferentialDrive.cpp) | [Differential-drive model](differential-drive.md) |
| [src/infrastructure/configuration/ApplicationConfig.hpp](../../src/infrastructure/configuration/ApplicationConfig.hpp) | [configuration](configuration.md) |
| [src/infrastructure/configuration/ConfigLoader.cpp](../../src/infrastructure/configuration/ConfigLoader.cpp) | [configuration](configuration.md) |
| [src/infrastructure/configuration/ConfigLoader.hpp](../../src/infrastructure/configuration/ConfigLoader.hpp) | [configuration](configuration.md) |
| [src/infrastructure/logging/LogRecord.hpp](../../src/infrastructure/logging/LogRecord.hpp) | [logging](logging.md) |
| [src/infrastructure/logging/LogSinks.cpp](../../src/infrastructure/logging/LogSinks.cpp) | [logging](logging.md) |
| [src/infrastructure/logging/LogSinks.hpp](../../src/infrastructure/logging/LogSinks.hpp) | [logging](logging.md) |
| [src/infrastructure/logging/Logger.cpp](../../src/infrastructure/logging/Logger.cpp) | [logging](logging.md) |
| [src/infrastructure/logging/Logger.hpp](../../src/infrastructure/logging/Logger.hpp) | [logging](logging.md) |
| [src/simulation/FakeCameraSource.cpp](../../src/simulation/FakeCameraSource.cpp) | [simulation](simulation.md) |
| [src/simulation/FakeCameraSource.hpp](../../src/simulation/FakeCameraSource.hpp) | [simulation](simulation.md) |
| [src/simulation/FakeCameraSourceFactory.cpp](../../src/simulation/FakeCameraSourceFactory.cpp) | [Stand-in capture backend](camera-selection.md#stand-in-capture-backend) |
| [src/simulation/FakeCameraSourceFactory.hpp](../../src/simulation/FakeCameraSourceFactory.hpp) | [Stand-in capture backend](camera-selection.md#stand-in-capture-backend) |
| [src/simulation/SimulationTransport.cpp](../../src/simulation/SimulationTransport.cpp) | [simulation](simulation.md) |
| [src/simulation/SimulationTransport.hpp](../../src/simulation/SimulationTransport.hpp) | [simulation](simulation.md) |
| [src/transport/IRobotTransport.hpp](../../src/transport/IRobotTransport.hpp) | [transport](transport.md) |
| [src/transport/TransportTypes.hpp](../../src/transport/TransportTypes.hpp) | [transport](transport.md) |
| [src/ui/MainWindow.cpp](../../src/ui/MainWindow.cpp) | [ui](desktop-demo.md) |
| [src/ui/MainWindow.hpp](../../src/ui/MainWindow.hpp) | [ui](desktop-demo.md) |
| [src/world/RobotState.hpp](../../src/world/RobotState.hpp) | [world](world-model.md) |
| [src/world/WorldModel.cpp](../../src/world/WorldModel.cpp) | [world](world-model.md) |
| [src/world/WorldModel.hpp](../../src/world/WorldModel.hpp) | [world](world-model.md) |
| [src/world/WorldSnapshot.hpp](../../src/world/WorldSnapshot.hpp) | [world](world-model.md) |
| [src/CMakeLists.txt](../../src/CMakeLists.txt) | [Build targets](../development/build.md) |
| [cmake/MicroRobotBoundaries.cmake](../../cmake/MicroRobotBoundaries.cmake) | [Platform boundaries](platform-boundaries.md#enforcement) |
| [cmake/CheckPlatformBoundary.cmake](../../cmake/CheckPlatformBoundary.cmake) | [Platform boundaries](platform-boundaries.md#enforcement) |

## Target boundaries

| Target | Direct dependencies |
| --- | --- |
| micro_robot_common | C++20 headers only |
| micro_robot_camera_api | micro_robot_common |
| micro_robot_camera_selection | camera API |
| micro_robot_transport_api | micro_robot_common |
| micro_robot_calibration | camera API, C++20 only |
| micro_robot_calibration_opencv | calibration, private OpenCV core/imgproc and calib3d (4.x) or objdetect (5.x) |
| micro_robot_control | micro_robot_common, micro_robot_transport_api (domain values only) |
| micro_robot_camera_discovery | camera API, Qt6::Multimedia |
| micro_robot_camera_replay | camera API, private OpenCV core/imgproc/videoio |
| micro_robot_world | micro_robot_common |
| micro_robot_logging | micro_robot_common |
| micro_robot_configuration | micro_robot_common, micro_robot_logging |
| micro_robot_simulation | camera and transport API targets, logging |
| micro_robot_desktop_controller | camera API, camera selection, transport API, world, logging, calibration, Qt6::Core |
| micro_robot_ui | controller, Qt6::Widgets |
| micro_robot_desktop | UI, simulation, camera discovery, calibration detector, logging, configuration |

See [root configuration](../../CMakeLists.txt), [platform/toolchain module](../../cmake/MicroRobotPlatform.cmake), and [test targets](../../tests/CMakeLists.txt).
Desktop targets are omitted when MICRO_ROBOT_BUILD_DESKTOP=OFF.
Each target's layer and the configure/test-time checks that enforce it are listed
in [platform boundaries](platform-boundaries.md).

## Implementation limits

Only common types, asynchronous camera/transport contracts, deterministic fakes,
robot pose storage, system device discovery and selection, human-readable application
logging, INI configuration loading, recorded-video replay, pure differential-drive
kinematics, and a Qt simulation
window are implemented. Discovery diagnostics use a separate Qt logging category.
[Calibration capture](calibration-capture.md) supplies a chessboard detector and capture wizard.
There are no intrinsic solver/profile persistence, robot tracking, planning, supervisory controller,
physical BLE, Zigbee, or structured measurement-recording modules. Qt Multimedia
is isolated in the concrete discovery and [streaming](camera-streaming.md) adapters. OpenCV is isolated in the optional
replay adapter and calibration detector; other proposed dependencies are absent from current CMake targets.
The desktop
does not perform autonomous motion or implement real safety/watchdog behavior.

## Documentation maintenance

When a public API changes, update its guide and source links, the architecture
overview, and relevant test instructions. Check behavior against implementation
and tests, not only declarations. Preserve historical reports as dated records.
New capabilities remain issue-scoped; documenting a planned module must not imply
that it already exists.
