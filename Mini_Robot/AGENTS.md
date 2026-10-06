# Micro-Robot Swarm Project

## Project purpose

Desktop application for controlling and tracking a swarm of small
ESP32-C6 differential-drive robots using external visual feedback.

Primary development platform:
- macOS
- CLion
- C++20
- Qt 6
- CMake

Future components include:
- Qt Multimedia
- OpenCV
- AprilTag/fiducial detection
- BLE
- path planning
- multi-robot coordination

## Architecture rules

Keep hardware-specific code isolated behind interfaces.

Core application logic must not depend directly on:
- physical cameras
- BLE
- Zigbee
- ESP32 hardware

Use abstractions such as:
- ICameraSource
- IRobotTransport

Simulation/fake implementations must use the same interfaces
as real implementations.

Prefer dependency injection instead of global objects.

Use RAII and standard smart pointers.

Avoid unnecessary singletons.

Do not introduce hardware dependencies during Phase 0.

## Project structure

Prefer:

src/
app/
core/
camera/
transport/
world/
simulation/
ui/

tests/

## C++

Use C++20.

Prefer:
- std::chrono for time
- strong domain types where useful
- enum class
- const correctness
- RAII
- std::unique_ptr for ownership

Avoid raw owning pointers.

## Qt

Qt GUI objects remain on the GUI thread.

Use signals/slots for asynchronous communication.

Do not put application/business logic directly in MainWindow.

## Testing

Use GoogleTest.

Core logic should be testable without Qt GUI or physical hardware.

## Build

The project must build with CMake from the command line.

After meaningful changes:
1. configure
2. build
3. run tests

Do not claim success unless the relevant build/tests were actually run.

## Current development phase

Phase 0 only.

Implement:
- CMake project
- Qt main window
- logging
- configuration loading
- common types
- ICameraSource
- IRobotTransport
- WorldModel skeleton
- SimulationTransport/fake data
- GoogleTest target

Do NOT implement yet:
- OpenCV
- AprilTag
- real camera capture
- BLE
- Zigbee
- robot control
- path planning