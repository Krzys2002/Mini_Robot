# Timestamped AprilTag frame detection

Issue #50 implements the `tag36h11` detector selected in
[the marker profile](apriltag-selection.md). Camera acquisition stays behind
`ICameraSource`; detection does not own a camera or depend on widgets.

## API and coordinates

[`MarkerDetection.hpp`](../../src/vision/MarkerDetection.hpp) defines the Qt/OpenCV-free
`IMarkerDetector` and immutable result values. Each `MarkerDetections` contains
the source frame sequence, original monotonic capture timestamp, dimensions,
processing duration, zero or more markers, and an error string. An empty vector
without an error is a successful negative observation. Error results contain no
partial observations.

Each observation has a family, dictionary ID and four pixel corners, in decoded
canonical **top-left, top-right, bottom-right, bottom-left** order. The order
rotates with the tag; it is never sorted by screen position. Image origin is
top-left, +u right and +v down. The canonical top edge identifies the marker's
front edge under the mounting profile. No ordering among separate markers is
guaranteed. Duplicate and unassigned IDs remain separate raw detections; robot
association must apply the profile's ambiguity/allocation rules in a later issue.

The optional detector-native decision margin is empty: OpenCV's ArUco API does
not provide it. No confidence score or pose is fabricated. This stage neither
undistorts pixels nor converts them into world coordinates, updates WorldModel,
tracks robots or sends motion commands.

## Backend and resource limits

[`OpenCvAprilTagDetector`](../../src/vision/OpenCvAprilTagDetector.hpp) caches the
AprilTag 36h11 dictionary and detector configuration. It uses OpenCV 4.2–4.6's
legacy ArUco API or the modern objdetect API from 4.7 onward. The desktop now
requires that component; Ubuntu's `libopencv-dev` supplies the development
modules. No additional AprilTag library is introduced.

Gray8 input is a read-only stride-aware view over retained immutable storage.
RGB8/RGBA8 input is converted to grayscale on the worker. No published pixels
are modified. One black border cell, subpixel corner refinement and zero
data-bit correction are selected conservatively. These settings do not establish
a measured false-positive rate or physical operating envelope.

Constructor-injected `AprilTagDetectorConfig` defaults to 64,000,000 pixels,
16,384 pixels per dimension and minimum marker perimeter/image-maximum-dimension
ratio 0.005. This admits 8K input without the calibration wizard's independent
4 MP limit. Metadata/storage validation and resource checks precede OpenCV work;
oversized/invalid input returns an error. Input is not silently downsampled.
Limits are resource bounds, not a frame-rate guarantee. Detector processing time
uses the steady clock and includes conversion, detection and result mapping;
queue delay and acquisition latency are excluded.

## Application worker and use

[`VisionController`](../../src/app/VisionController.hpp) receives the existing
`DesktopController::cameraInputChanged` and `frameAvailable` signals. All service
calls and result callbacks belong to the owning application thread. One dedicated
worker runs the injected detector serially. There is at most one active job and
one newest pending frame; superseded pending frames are released.

Source/stream-state/resolution changes clear observations and invalidate in-flight
results. Older/duplicate sequence or capture timestamps are ignored within a
source generation. Null observation delivery clears consumers. Detector exceptions
become error results, and later frames can recover. No worker exception escapes
into the camera pipeline.

Start a selected camera in the desktop: the main window shows the last processed
frame number, detected count and processing milliseconds. Structured observation
values are delivered through `observationsAvailable(sourceId, detections)`;
debug-level `vision` logs include source, sequence, count and `processing_us`.
Calibration remains an independent consumer/worker. There is no AprilTag overlay
or new robot control UI in this issue.

`shutdown()` invalidates pending work and requests worker exit without waiting on
the GUI thread. The production event loop remains alive until `stopped()` before
destroying the service. An in-flight OpenCV call finishes normally; it is not
forcibly interrupted. Destruction joins as a fallback, so production callers must
follow that shutdown contract.

Recorded input uses the same frame/detector contract. Advance `RecordedVideoSource`
on an acquisition/test thread; bridge its owner-thread frames to the application
service when needed. File decoding must not run on the GUI thread. The desktop
does not yet offer a recorded-file selector; the replay integration test drives
the existing source with a deterministic generated MJPEG file.

## Build and verification

The normal desktop enables the detector. To build it with replay and no Qt:

```sh
cmake -S . -B build/vision-headless -G Ninja -DCMAKE_BUILD_TYPE=Debug \
  -DMICRO_ROBOT_BUILD_DESKTOP=OFF -DMICRO_ROBOT_BUILD_CALIBRATION=OFF \
  -DMICRO_ROBOT_BUILD_VISION=ON -DMICRO_ROBOT_BUILD_REPLAY=ON
cmake --build build/vision-headless --parallel 3
ctest --test-dir build/vision-headless -L vision --output-on-failure --no-tests=error
```

With a desktop build, select detector/replay and worker tests together using
`ctest --test-dir build -R '^(AprilTag|VisionControllerTest)' --output-on-failure --no-tests=error`.
Set desktop/replay/calibration/vision OFF for a Qt/OpenCV-free core build.

Tests exercise padded Gray/RGB/RGBA inputs, unchanged storage/capture metadata,
all four canonical rotations, multiple/duplicate/unassigned IDs, transformed
markers, blank/noise/occluded images, malformed/resource-limited inputs, MJPEG
replay, bounded pending frames, GUI heartbeat during blocked detection,
source/stop/resolution invalidation, exception recovery and asynchronous shutdown.

### Local verification record (2026-10-20)

On macOS / AppleClang 21 / Qt 6 / OpenCV 5.0.0, Debug with Ninja and cached GoogleTest:

- Desktop configure/build in `build/issue-50` passed. Final
  `ctest --test-dir build/issue-50 --output-on-failure --no-tests=error`:
  **205/205 passed**, including seven detector/replay and eight worker tests.
- Headless detector/replay configure/build in `build/issue-50-headless`, with
  desktop/calibration OFF and vision/replay ON, passed. Unfiltered CTest:
  **144/144 passed**, without Qt.
- Core configure/build in `build/issue-50-core`, with desktop/calibration/replay/
  vision OFF, passed. Unfiltered CTest: **126/126 passed**, without Qt or OpenCV.
- The first full desktop run had one failure in the existing camera-silence
  timer test. Its isolated rerun and the final full 205-test run passed; the
  unrelated test/implementation were not changed.
- Diff whitespace checks and all 253 local link targets in changed documentation
  passed. Physical and other-platform results below remain unverified.

## Physical acceptance: NOT RUN

Generated images and generated-video tests do not prove detection of printed tags
from a physical camera. Issue #50's printed-tag live/recorded acceptance remains
unverified. Use the profile's commissioning procedure and retain:

1. Actual print/black-edge measurements, camera/OS/OpenCV versions, delivered
   resolution, lighting, coverage and source commit/configuration.
2. Live detections of several distinct printed tags, including turning a tag to
   verify canonical corner/front orientation. Confirm the GUI stays interactive.
3. A real recording of the same scene replayed through the detector, with frame
   IDs, ordered corners, capture times and processing-time logs.
4. Blank/occluded/moving scenes and observed misses/false positives, plus shutdown
   and source-switch behavior while processing. Do not infer tracking or control
   suitability from synthetic results.

Linux/Windows runtime, legacy OpenCV execution and physical throughput remain
unverified by the local macOS work. Linux CI must validate the installed OpenCV API path;
its result must be checked on the eventual PR before claiming success.

OpenCV references: [marker detection and corner order](https://docs.opencv.org/4.x/d5/dae/tutorial_aruco_detection.html),
[detector parameters](https://docs.opencv.org/4.x/d1/dcd/structcv_1_1aruco_1_1DetectorParameters.html).
