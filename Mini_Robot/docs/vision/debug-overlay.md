# Vision debug overlay and diagnostics

Issue [#52](https://github.com/Krzys2002/Mini_Robot/issues/52) presents raw detector
output and the [robot observation model](robot-observation.md) on the **exact
image that was processed**. It supplies diagnostics, not metric localization,
tracking or control. Physical printed-tag acceptance is separate in
[#211](https://github.com/Krzys2002/Mini_Robot/issues/211), as approved by the
maintainer. **All physical checks are NOT RUN.**

## Use

Start the selected camera and open **Vision debug…** in the main window. The
independent, resizable dialog shows the newest completed detection image. Closing
it does not stop the camera or detection worker; closing the main window also
closes the debug dialog and uses the existing worker-draining shutdown path.
The hidden dialog stops its age timer and defers diagnostic text formatting until
it is shown, when it rechecks retained preview age before display. It retains at
most one matched snapshot. Detection and worker-side overlay preparation continue
for service subscribers regardless of dialog visibility. `VisionController` accepts
an injected immutable `RobotMarkerMap`, so composition can share the same identity
allocation with observation consumers.

- Green solid boundary: assigned robot; label contains both dictionary Tag ID and
  RobotId (default tag36h11 IDs 0–15 map to robots 1–16).
- Orange dashed boundary: rejected/unassigned/duplicate marker; diagnostic reason
  replaces RobotId. Rejected tags never acquire a valid robot identity here.
- Cyan circle: projective marker center.
- Yellow arrow: decoded canonical front, with the provisional image heading from
  #51. For raw image coordinates, +u is right and +v down; positive heading is
  displayed counterclockwise. Up/right/down/left arrows rotate with the tag.
- Non-finite corner geometry cannot be painted; its rejection remains in the
  diagnostic list. A detector error retains its matching raw image with no partial
  overlays and an error message. A successful frame with no tags has no overlays.

Each label is placed outside its own boundary/arrow extent, with collision
avoidance for preceding labels. Labels are elided to the viewport if necessary;
the scrollable diagnostic list retains each full identity/rejection string.

## Timing and quality

Diagnostics identify source, frame sequence, raw dimensions and:

| Value | Meaning |
| --- | --- |
| Age (ms) | Current local steady time minus the matched frame's original capture timestamp; updated while no new result arrives |
| Detector (ms) | Detector-call/result preparation duration measured by the worker, excluding GUI paint and overlay preparation |
| Output FPS | Cadence of accepted completed result publications, averaged over at most 16 completion timestamps; not camera FPS, detection-success rate or reciprocal detector time |
| Minimum edge (px) | Shortest projected marker edge from the observation geometry; no automatic sampling threshold is enforced |
| Margin | Detector-native decision margin when supplied; OpenCV ArUco leaves it unavailable; no confidence score is invented |

FPS is `n/a` until two output completion times span a positive interval. Skipped
input frames are not counted, and input/source/resolution invalidation resets the
rate window. Frame capture time must already be mapped into the local steady-clock
domain by the source; an unmapped virtual/media epoch is not a latency clock.

A preview older than the shared `CameraFrameTimeout` (currently three seconds)
expires: image/overlays and their retained frame are cleared with an explicit
expired message. Stop, source replacement, resolution change and shutdown clear
old snapshots immediately. A late worker result from a previous input generation
is discarded, including when an observer changes input during a signal callback.

## Frame pairing and display mapping

[VisionDebugFrame.hpp](../../src/vision/VisionDebugFrame.hpp) and
[VisionDebugFrame.cpp](../../src/vision/VisionDebugFrame.cpp) are Qt/OpenCV-free
presentation data/helpers. Construction rejects mismatched sequence, timestamp or
image dimensions. [VisionController](../../src/app/VisionController.cpp) performs
robot association and diagnostic geometry on its worker, then publishes a const
snapshot containing the original immutable raw frame and its detections. It keeps
the existing one-running/one-newest-pending policy. The dialog retains only its
newest completed snapshot; QWidget coalesces paint requests.

[VisionDebugView](../../src/ui/VisionDebugView.cpp) only paints prepared values.
Image and geometry use the same uniform scale and centered letterbox rectangle.
Detector `(u,v)` addresses pixel centers, while the QImage target rectangle starts
at pixel edges, so the mapping is `(left + (u+0.5)*scale, top + (v+0.5)*scale)`.
Resizing recalculates that mapping rather than stretching old overlay coordinates.
Qt device-pixel scaling stays at the presentation boundary. Borders/arrows are
clipped to the image rectangle; labels and the diagnostic list remain readable.

This is always the **raw, distorted detector image**. It is not the separate
intrinsic-calibration dialog's optional undistorted verification preview. There is
no camera-to-world transform here: never interpret the displayed pixel heading as
a calibrated world pose or draw these coordinates over an undistorted/cropped
image without the corresponding transform.

## Verification

[VisionDebugFrameTest](../../tests/vision/VisionDebugFrameTest.cpp) covers exact
frame pairing, independent identities/rejections, error frames and bounded/reset
output cadence without Qt/OpenCV. Extended
[VisionControllerTest](../../tests/ui/VisionControllerTest.cpp) checks the newest
pending frame, immutable matching pixels, source resets and observer invalidation.
[VisionDebugViewTest](../../tests/ui/VisionDebugViewTest.cpp) renders padded Gray,
RGB and RGBA images in wide/tall viewports, compares image patches and overlays
against independent expected pixel positions, exercises four heading rotations,
IDs/quality/timing, stale expiry and dialog lifecycle. The same tests run at scale
factors 1.5 and 2; main-window GUI compatibility checks include the debug action
and its keyboard order.

Useful commands (after building the relevant configuration):

```sh
ctest --preset core -R 'VisionDebugFrameTest|VisionOutputRateTest' --no-tests=error
ctest --preset macos -R 'VisionDebug|VisionController|GuiCompatibility' --no-tests=error
```

For physical acceptance, retain the camera/resolution/OS/OpenCV/print details and
raw recording, align several real printed markers at different positions and
orientations, repeat wide/tall/high-DPI resizing, then exercise slow processing,
source changes, stop and camera silence. Record actual outcomes and screenshots
in #211, reusing detector commissioning evidence from #191. Synthetic rendered
images do not establish physical-camera performance or Linux/Windows runtime.
