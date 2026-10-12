# Source and documentation map

This map describes the implemented source tree. Planning catalogs and archived
reports are scope/history, not evidence that future modules are present.

| Source file | Behavior guide |
| --- | --- |
| [src/app/CameraDiscoveryController.cpp](../../src/app/CameraDiscoveryController.cpp) | [Discovery presentation](camera-discovery.md) |
| [src/app/CameraDiscoveryController.hpp](../../src/app/CameraDiscoveryController.hpp) | [Discovery presentation](camera-discovery.md) |
| [src/app/Info.plist.in](../../src/app/Info.plist.in) | [macOS discovery metadata](camera-discovery.md#macos-and-verification) |
| [src/app/DesktopController.cpp](../../src/app/DesktopController.cpp) | [app](desktop-demo.md) |
| [src/app/DesktopController.hpp](../../src/app/DesktopController.hpp) | [app](desktop-demo.md) |
| [src/app/main.cpp](../../src/app/main.cpp) | [app](desktop-demo.md) |
| [src/camera/CameraTypes.hpp](../../src/camera/CameraTypes.hpp) | [camera](camera-source.md) |
| [src/camera/ICameraSource.hpp](../../src/camera/ICameraSource.hpp) | [camera](camera-source.md) |
| [src/camera/ICameraDeviceDiscovery.hpp](../../src/camera/ICameraDeviceDiscovery.hpp) | [Discovery contract](camera-discovery.md) |
| [src/camera/QtCameraDeviceDiscovery.cpp](../../src/camera/QtCameraDeviceDiscovery.cpp) | [Qt discovery adapter](camera-discovery.md) |
| [src/camera/QtCameraDeviceDiscovery.hpp](../../src/camera/QtCameraDeviceDiscovery.hpp) | [Qt discovery adapter](camera-discovery.md) |
| [src/common/Pose2D.hpp](../../src/common/Pose2D.hpp) | [common](architecture.md#domain-conventions) |
| [src/common/RobotId.hpp](../../src/common/RobotId.hpp) | [common](architecture.md#domain-conventions) |
| [src/common/Time.hpp](../../src/common/Time.hpp) | [common](architecture.md#domain-conventions) |
| [src/common/Types.hpp](../../src/common/Types.hpp) | [common](architecture.md#domain-conventions) |
| [src/common/Units.hpp](../../src/common/Units.hpp) | [common](architecture.md#domain-conventions) |
| [src/simulation/FakeCameraSource.cpp](../../src/simulation/FakeCameraSource.cpp) | [simulation](simulation.md) |
| [src/simulation/FakeCameraSource.hpp](../../src/simulation/FakeCameraSource.hpp) | [simulation](simulation.md) |
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

## Target boundaries

| Target | Direct dependencies |
| --- | --- |
| micro_robot_common | C++20 headers only |
| micro_robot_camera_api | micro_robot_common |
| micro_robot_transport_api | micro_robot_common |
| micro_robot_camera_discovery | camera API, Qt6::Multimedia |
| micro_robot_world | micro_robot_common |
| micro_robot_simulation | camera and transport API targets |
| micro_robot_desktop_controller | camera API, transport API, world, Qt6::Core |
| micro_robot_ui | controller, Qt6::Widgets |
| micro_robot_desktop | UI, simulation, camera discovery |

See [root configuration](../../CMakeLists.txt) and [test targets](../../tests/CMakeLists.txt).
Desktop targets are omitted when MICRO_ROBOT_BUILD_DESKTOP=OFF.

## Implementation limits

Only common types, asynchronous camera/transport contracts, deterministic fakes,
robot pose storage, read-only system device discovery, and a Qt simulation window
are implemented. Discovery logs use an issue-scoped Qt category; there is no
application-wide structured logging service. There are no
vision, calibration, tracking, planning, supervisory controller, physical BLE,
Zigbee, replay decoding, settings loader, or structured logging modules. Qt Multimedia is used only by the concrete discovery adapter; OpenCV
and other proposed dependencies are absent from current CMake targets. The desktop
does not perform autonomous motion or implement real safety/watchdog behavior.

## Documentation maintenance

When a public API changes, update its guide and source links, the architecture
overview, and relevant test instructions. Check behavior against implementation
and tests, not only declarations. Preserve historical reports as dated records.
New capabilities remain issue-scoped; documenting a planned module must not imply
that it already exists.
