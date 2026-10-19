# Cross-platform camera backends

Source: [V4L2 discovery header](../../src/camera/V4l2CameraDeviceDiscovery.hpp),
[V4L2 discovery implementation](../../src/camera/V4l2CameraDeviceDiscovery.cpp),
[V4L2 node probe](../../src/camera/V4l2Device.cpp),
[OpenCV V4L2 source header](../../src/camera/V4l2CameraSource.hpp),
[OpenCV V4L2 source implementation](../../src/camera/V4l2CameraSource.cpp),
[Qt capture adapter](../../src/camera/QtCameraSource.cpp),
[composition root](../../src/app/main.cpp), and
[V4L2 tests](../../tests/camera/V4l2CameraTest.cpp).

Issue #128 (LNX-04) adds Linux camera discovery and capture through OpenCV's V4L2
backend, keeps Qt Multimedia as the macOS backend, and adds a configurable capture
resolution for both. Calibration and marker detection remain M2 work.

## Backend selection

| Platform | Discovery | Capture | Selected by |
| --- | --- | --- | --- |
| Linux | `V4l2CameraDeviceDiscovery`: sysfs plus `VIDIOC_QUERYCAP` | `V4l2CameraSource`: OpenCV `videoio` with `cv::CAP_V4L2` | `MICRO_ROBOT_CAMERA_BACKEND=V4L2` (Linux default) |
| macOS | `QtCameraDeviceDiscovery` | `QtCameraSource`: Qt Multimedia (AVFoundation) | `MICRO_ROBOT_CAMERA_BACKEND=Qt` (only choice) |
| Windows | `QtCameraDeviceDiscovery` | `QtCameraSource`: Qt Multimedia | `Qt`; not validated |

Both pairs implement [ICameraDeviceDiscovery](camera-discovery.md),
[ICameraSourceFactory](camera-selection.md), and [ICameraSource](camera-source.md).
`CameraSelectionController`, `DesktopController`, and `MainWindow` are unchanged and
do not know which backend runs. Frames reach them through the same interface on
every platform. The V4L2 adapter delivers packed `Rgb8` frames and the Qt adapter
`Rgba8`; both pass `isValidFrame()`.

Device IDs are backend-specific (a node path for V4L2, hex-encoded Qt IDs for Qt),
so the discovery adapter and the factory must come from the same backend. The
choice is made when CMake configures the build (see [build options](../development/build.md#platform-and-compiler-configuration));
[main.cpp](../../src/app/main.cpp) is the only source with the conditional and logs
the active backend at startup (`Camera backend: OpenCV V4L2`). On Linux,
`-DMICRO_ROBOT_CAMERA_BACKEND=Qt` restores the Qt Multimedia backend, for example
when OpenCV is not installed.

## OpenCV backend review

Findings that shaped the adapters:

- **No enumeration API.** `cv::VideoCapture` opens an index or a name but cannot
  list devices. On Linux, the V4L2 backend accepts a device node path, so discovery
  reads `/sys/class/video4linux` and IDs are node paths that the source opens directly.
- **macOS.** OpenCV's AVFoundation backend opens cameras by index only. Indexes
  have no stable identity and can point to a different camera after hotplug.
  OpenCV also offers no permission request. Qt Multimedia provides device IDs,
  hotplug notifications, and `QCameraPermission`, and capture through it was
  confirmed on macOS for #31. It therefore stays the macOS backend.
- **Errors.** OpenCV reports only whether opening succeeded. The V4L2 source first
  opens the node itself to tell a missing device, a permission error, a busy driver,
  and a non-capture node apart.
- **Timeouts.** `CAP_PROP_READ_TIMEOUT_MSEC` applies to the FFmpeg and GStreamer
  backends only. The V4L2 backend blocks in `select()` for up to 10 s by default.
  The adapter instead waits with `VideoCapture::waitAny()` in 100 ms slices, so stop
  requests are served promptly and `CameraFrameTimeout` (3 s) is enforced.
- **Resolution.** OpenCV applies `CAP_PROP_FRAME_WIDTH`/`HEIGHT` through `VIDIOC_S_FMT`.
  The driver adjusts an unsupported size to the nearest mode it offers. Qt instead
  needs an exact `QCameraFormat` from the device's list.
- **Pixel formats and rate.** The V4L2 backend prefers uncompressed modes such as
  YUYV over MJPEG and converts to BGR. Many USB cameras offer high resolutions at
  full frame rate only as MJPEG. Frame-rate and pixel-format selection are not
  configurable yet. On the test camera both 640×480 and 1280×720 were YUYV at
  about 10 fps under room lighting.
- **Timestamps.** Both adapters stamp frames with the local steady clock when the
  worker receives them, so the values are receipt-time estimates. Driver buffer
  timestamps are not used.
- **Qt Multimedia on Linux** also captures through V4L2 (through its FFmpeg backend in
  Qt 6.5+). It remains selectable and is built and unit-tested, but this issue
  validates the OpenCV path on Linux hardware.

## Linux discovery

`scanV4l2CaptureDevices()` lists `video<N>` entries in `/sys/class/video4linux`,
ordered by node number (so `video10` follows `video2`). For each entry it opens
`/dev/video<N>` read/write and non-blocking, queries `VIDIOC_QUERYCAP`, and closes it
without streaming:

| Node | Listed |
| --- | --- |
| Offers single- or multi-planar video capture | Yes |
| Other V4L2 node, for example the UVC metadata node that follows each camera | No |
| Cannot be opened because of permissions, or the driver is busy | Only when sysfs `index` is 0 (the device's first node), so starting it reports the actual problem |
| Node absent (unplugged, or not yet created) | No |

The ID is the node path, for example `/dev/video0`. The display name is the sysfs
`name`, or the driver's card name when that is empty. A missing name stays empty,
and the selector shows its usual fallback. Node numbers can change after replugging,
so IDs are not persistent across sessions; persistent `/dev/v4l/by-id` mapping is
calibration-profile work for M2.

`V4l2CameraDeviceDiscovery` watches `/dev` with `QFileSystemWatcher` (inotify).
When nodes appear or disappear it rescans in one coalesced, queued step and
publishes only if the inventory changed. Unrelated `/dev` activity therefore causes
no updates, while `refresh()` always publishes. If `/dev` cannot be watched, it logs
`camera_watch_unavailable`, and devices then update only on refresh. Logs use the
`micro_robot.camera.discovery` category (`camera_inventory` with `backend=v4l2`,
and `camera_discovered`). Scanning opens nodes on the owning (GUI) thread. That is
the same short open and capability query Qt performs for its own inventory, but a
USB camera may need tens of milliseconds to resume from autosuspend.

## Linux capture and failures

`V4l2CameraSource` follows the [Qt adapter's lifecycle](camera-streaming.md):
requests are asynchronous and idempotent, events are delivered on the owner thread,
request generations discard late results, and sequence numbers increase across
restarts. Opening, format negotiation, capture, and RGB conversion run on a dedicated
`QThread` worker. A single latest-frame slot drops old frames for slow consumers.
The source enters Streaming when the first frame arrives and delivers that frame
right after the state event. Stop and failure release the device on the worker
before the quiescent state is delivered.

| Situation | Error code | Final state |
| --- | --- | --- |
| Node missing at Start | `Unavailable` | Fault |
| Permission denied (`EACCES`/`EPERM`) | `PermissionDenied`, naming the node and the access fix | Fault |
| Driver busy, or OpenCV cannot open the node (for example, another application is streaming) | `BackendFailure` | Fault |
| Node is not a capture device | `BackendFailure` | Fault |
| Node removed while Starting or Streaming | `Unavailable` | Disconnected |
| Read exception while the node exists, or unsupported pixel layout | `BackendFailure` / `InvalidFrame` | Fault |
| No frame for `CameraFrameTimeout` (3 s), including during startup | `BackendFailure` | Fault |

Every case is recoverable: after the device or permission is fixed, Start reopens
it. Combined with discovery's removal notification, unplugging follows the
[#46 recovery flow](camera-streaming.md#source-loss-and-recovery-issue-46). OpenCV may also
print its own `[ WARN ]` lines to stderr when opening fails.

## Capture resolution

`camera.resolution` in the [configuration file](configuration.md) is `auto` (the
default, the device's own default mode) or `<width>x<height>`, for example
`1280x720`. The composition root passes it to the selected factory as
`CameraCaptureSettings`, and the factory applies it to every source it creates:

- **V4L2:** requests the size from the driver. If the driver picks another mode,
  `camera_resolution_fallback` is logged with both sizes (category
  `micro_robot.camera.capture`, warning level) and capture continues at the driver's
  size.
- **Qt:** chooses the fastest `QCameraFormat` with exactly that size. If none
  exists, Qt's default format is kept and `camera_resolution_fallback` is logged.

The delivered size is always the frame's width and height, which the desktop
shows with each frame. A resolution mismatch is not a camera error. The setting
does not change the frame rate or pixel format.

## Device permissions

Linux checks access to `/dev/video*` at open. See
[camera device access](../development/linux-setup.md#camera-device-access-devvideo)
for diagnosis and setup. The application never asks for elevated privileges.

## Verification

Automated tests ([V4l2CameraTest](../../tests/camera/V4l2CameraTest.cpp), built into
`micro_robot_tests` when the V4L2 backend is selected) need no camera. They use
temporary sysfs/dev trees and cover numeric ordering, metadata-node filtering,
listing inaccessible primary nodes, missing names, vanished nodes, asynchronous and
coalesced refresh, hotplug add/remove, and ignoring unrelated `/dev` changes. Source
tests cover factory ID rules, asynchronous missing-device failure and retry,
permission denial, non-capture nodes, and cancelling a pending start. Tests that
need unreadable files skip when run as root. Configuration tests cover parsing and
rejecting `camera.resolution`.

Two hardware tests run only when `MICRO_ROBOT_TEST_V4L2_DEVICE` names a node. The
optional `MICRO_ROBOT_TEST_V4L2_RESOLUTION` adds a size check:

```sh
MICRO_ROBOT_TEST_V4L2_DEVICE=/dev/video0 MICRO_ROBOT_TEST_V4L2_RESOLUTION=1280x720 \
  ctest --preset linux-gcc -R '^V4l2CameraHardwareTest\.' --output-on-failure
```

Executed on 2026-10-19 on Ubuntu 26.04.1 LTS (kernel 7.0.0, Qt 6.10.2, OpenCV 4.10.0)
with the laptop's integrated camera, a USB UVC device (`13d3:56ff`) exposing
`/dev/video0` (capture) and `/dev/video1` (metadata):

- Discovery listed only `/dev/video0` as "Integrated Camera: Integrated C" in the
  hardware test and in the desktop app's startup inventory.
- Two Start/Stop cycles each streamed at least 10 valid `Rgb8` frames with
  increasing sequences: default mode (640×480), `1280x720`, and `640x480`.
- Requesting `1000x700` streamed at 960×540 and logged `camera_resolution_fallback`.
- With another process streaming the camera, Start failed with "OpenCV V4L2 backend
  cannot open /dev/video0; it may be in use by another application", without a crash.
- The desktop app (offscreen platform) logged `Camera backend: OpenCV V4L2`, the
  configured resolution, and a one-device inventory.
- The Qt adapter, driven directly against the same camera through Qt Multimedia's
  Linux backend, delivered 1280×720 by default and 640×480 when requested. A
  `1000x700` request logged `camera_resolution_fallback` and kept 1280×720.

Not yet verified: an external USB camera and its hotplug on Linux, a real
permission denial on Linux hardware (covered by an automated test with an
unreadable node), interactive GUI use on X11/Wayland, the Qt resolution setting
and a USB camera on macOS, and Windows. Manual acceptance procedure:

1. Linux: plug in a USB camera and confirm it appears once (no metadata node);
   select it, Start, and check increasing frame sequences. Unplug and replug while
   streaming; confirm Disconnected, refreshed inventory, and that a new Start works.
2. Linux: remove your access (for example, log in over SSH as a user without the
   `video` group) and confirm the permission message; restore access and retry.
3. Linux and macOS: set `camera.resolution = 1280x720` and confirm the frame size;
   set a size the camera does not offer and confirm the fallback warning.
4. macOS: repeat step 1 with a USB camera on the Qt backend.
