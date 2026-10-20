# Camera device discovery

Source: [discovery interface](../../src/camera/ICameraDeviceDiscovery.hpp),
[Qt adapter header](../../src/camera/QtCameraDeviceDiscovery.hpp),
[Qt adapter implementation](../../src/camera/QtCameraDeviceDiscovery.cpp),
[presenter header](../../src/app/CameraSelectionController.hpp), and
[presenter implementation](../../src/app/CameraSelectionController.cpp).

Issue #29 adds a read-only inventory of operating-system cameras. Discovery itself
does not open a camera or capture frames; [camera selection](camera-selection.md)
(Issue #30) consumes its snapshots.

## Architecture and API

[ICameraDeviceDiscovery](../../src/camera/ICameraDeviceDiscovery.hpp) is a Qt-free
boundary separate from ICameraSource. CameraDevice holds an opaque device ID and
a display name. devices() returns a value copy of the latest inventory. refresh()
requests a complete snapshot asynchronously; an empty result is valid. Backends
retain a weak event sink and dispatch on their owning thread. Empty subscriptions
detach; event handlers must be prompt/non-throwing and must not destroy the backend.

[QtCameraDeviceDiscovery](../../src/camera/QtCameraDeviceDiscovery.cpp) queries
QMediaDevices::videoInputs() and refreshes on videoInputsChanged. Queries use Qt's
inventory API; no QCamera/capture session, camera reads, or polling loop is added.
Refresh requests coalesce through a context-bound queued timer, cancelled on
adapter destruction. Device additions/removals replace the entire snapshot,
including transition to zero devices. Old copied records are not live handles.

The adapter retains description() and hex-encodes id() byte-for-byte. IDs distinguish
same-name cameras and may remain stable within a backend, but Qt does not guarantee
persistence across OS/backend changes. An unavailable ID remains empty; it is not
fabricated from a name or list index. Consumers must not use the display name as
identity. Every reported device is retained, including duplicate names.

[CameraSelectionController](../../src/app/CameraSelectionController.cpp) (formerly
CameraDiscoveryController) subscribes through the interface and turns snapshots
into selector rows; MainWindow binds to it and contains no device queries or
discovery logic. Names are displayed as plain text with the ID as a tooltip.
Empty lists show “No cameras available”; missing names/IDs have explicit
presentation fallbacks. Physical capture is provided by [Issue #31 streaming](camera-streaming.md).

## Linux V4L2 adapter

With the default Linux build, the desktop uses `V4l2CameraDeviceDiscovery` instead
(Issue #128). It lists V4L2 capture nodes from sysfs, hides metadata nodes, uses the
node path (`/dev/video0`) as the ID, and refreshes when `/dev` changes. See
[cross-platform camera backends](camera-backends.md#linux-discovery). The Qt adapter
described above remains the macOS backend and can be selected on Linux.

## Build and logging

Desktop builds now require Qt6 Widgets and Multimedia. The concrete adapter is
isolated in micro_robot_camera_discovery; controller/UI targets do not depend on
Qt Multimedia. MICRO_ROBOT_BUILD_DESKTOP=OFF still configures without finding Qt.
Install the Qt Multimedia component alongside the matching Qt installation.

Inventory logs use the Qt category micro_robot.camera.discovery, emitting
camera_inventory (count) and camera_discovered (ID/name) records with local
steady-clock milliseconds. From Mini_Robot after the Debug recipe in
[build setup](../development/build.md), enable explicitly with:

```sh
QT_LOGGING_RULES='micro_robot.camera.discovery.info=true' ./build/debug/src/micro_robot_desktop.app/Contents/MacOS/micro_robot_desktop
```

This category is issue-scoped Qt diagnostics, separate from the
[application Logger](logging.md) and its file sink. It does not provide structured
experiment measurements or configuration. No camera buffers or images are logged.

## macOS and verification

The [bundle template](../../src/app/Info.plist.in) includes
NSCameraUsageDescription and the Continuity Camera device-type
opt-in. Built-in, USB, and Continuity devices are listed when Qt/macOS exposes them;
no Apple API is called from the domain core. Discovery does not start capture or
request capture authorization. A missing/empty list is handled safely, and visibility
may depend on OS permissions, hardware connection, Qt version, and backend.

[Automated selection tests](../../tests/ui/CameraSelectionControllerTest.cpp) inject a
manual discovery implementation, without cameras, and
cover initial/empty snapshots, duplicate names/distinct IDs, add/remove/rename
updates, missing metadata, plain-text display, and subscription teardown. They do not prove physical device visibility or hotplug behavior.

The production adapter probe detected the FaceTime HD and an exposed iPhone
camera on macOS, with distinct IDs and discovery logs. USB visibility/hotplug
remains unchecked; the app launched, but native visual inspection was unavailable.
Windows hardware/runtime behavior has not been executed; Linux results for the V4L2
adapter are in [camera backends](camera-backends.md#verification). These limits
do not change the following outstanding manual acceptance procedure.

Manual acceptance checks:

1. Launch the app and inspect the Camera selection list and discovery logs.
2. Confirm the Mac internal camera's description and ID appear.
3. Connect a USB camera; confirm it appears with a distinguishable ID.
4. Disconnect it; confirm the list updates without a crash.
5. If available, expose an iPhone through Continuity Camera and check its entry.
6. On a host with no exposed cameras, confirm the explicit empty-state message.

References: [QMediaDevices](https://doc.qt.io/qt-6/qmediadevices.html),
[QCameraDevice](https://doc.qt.io/qt-6/qcameradevice.html), and
[Apple camera usage description](https://developer.apple.com/documentation/bundleresources/information-property-list/nscamerausagedescription).
