# Intrinsic calibration profiles and verification preview

Issue #55 extends the [intrinsic calculation workflow](intrinsic-calibration.md)
with durable profiles for an exact camera identity and delivered image size.

## Operator workflow

1. Select and start the camera, then open **Intrinsic camera calibration**.
   The first valid frame supplies the actual negotiated resolution. The controller
   automatically looks up that camera/resolution's saved profile; the dialog shows
   its identity, dimensions, RMS and loaded/missing/rejected status.
2. To create a replacement, collect and inspect diverse board views and calculate
   intrinsics as before. A calculated profile is explicitly **not saved** until
   **Save profile** succeeds. Saving atomically replaces only this exact key.
   High reprojection RMS remains an advisory warning; inspect/recapture before use.
3. **Reload saved profile** discards the active preview profile and reads the exact
   key again. It does not fabricate captured observations or a new solver result.
   The saved model loads again after camera stop/start or application restart.
4. Toggle **Undistorted preview** to compare raw and corrected pixels. It is
   available only with a valid compatible model and a delivered frame, defaults
   OFF and resets on dialog close, reload, camera loss/change or resolution change. A newly
   calculated, unsaved model can also be previewed before saving.

Profiles live in the application's per-user Qt `AppConfigLocation`, under
`calibration/`. This is independent of a custom settings file passed with
`--config`. The composition root injects the directory into the store; tests use
private temporary directories. Disk filenames are SHA-256 keys of the length-
prefixed camera identity plus width/height. They never contain a camera-supplied
path. One camera can retain profiles at several resolutions.

A device or resolution mismatch is rejected, even if a file is copied into another
key's location. There is no implicit scaling, nearest-resolution fallback or
cross-device override. A missing profile leaves correction disabled; a malformed,
unsupported, oversized or unreadable file produces a rejection message. Save
failure is visible and preserves the previous file. Recalibrate after changing
lens, focus, zoom or capture processing: matching ID/resolution alone cannot
establish unchanged optics. Backends whose device IDs change across restarts need
a new calibration; the application does not guess identity from display names.

## Physical camera identity and review corrections

Capture access paths and physical profile identity are separate. `CameraInfo`
exposes optional `persistentCalibrationId`; composition passes that identity and
its persistence eligibility through `calibrationInputChanged`. Calibration's
`setInput` defaults to session-only unless the caller explicitly attests identity.
On Linux V4L2, the resolver uses USB vendor/product/serial, interface number and
video index from sysfs. Renumbering `/dev/videoN` preserves the key; another camera
at the old node gets another key. Missing serial/metadata or duplicate identities
disable saved-profile loading and saving. Capture and in-session calculation and
undistorted preview still work. The source rechecks identity around device open;
a changed/ambiguous identity faults and requires reselection. Qt's Linux adapter
also disables persistence until it can resolve physical identity. Existing
node-path-keyed Linux profiles are intentionally not migrated or auto-loaded.

The other native Qt backends retain their opaque device identity; those IDs are
backend-specific and do not promise cross-OS portability. Identity equality cannot
prove unchanged optics or camera placement. The USB resolver uses Linux's
[documented USB attributes](https://github.com/torvalds/linux/blob/master/Documentation/ABI/testing/sysfs-bus-usb)
without treating a port path or camera display name as a physical serial.

After solving, select a poor view and choose **Remove view**. The capture session
returns to Collecting and retains the other observations. The computed result,
active calibration and undistorted preview are invalidated; save stays disabled
until recalculation succeeds. Previously saved files are preserved. Removal is
blocked while worker/profile I/O is active. Capture more views if the remaining
set no longer meets count/diversity requirements, then calculate again.

## Version 1 format and validation

`CalibrationProfile.hpp` defines Qt/OpenCV-free serialization, parsing, exact
compatibility checks and injected worker-only storage/undistortion interfaces.
The UTF-8 text format starts with `MICRO_ROBOT_INTRINSICS 1`. Fixed ordered fields:

| Field | Meaning |
| --- | --- |
| `model pinhole_brown_conrady_5` | Supported lens model; no fisheye/rational substitution |
| `camera "..."` | Quoted device/source ID with quote/backslash escaping |
| `resolution width height` | Delivered dimensions in pixels |
| `matrix` | Nine row-major doubles: `[fx,0,cx; 0,fy,cy; 0,0,1]`, pixel units |
| `distortion` | Five dimensionless doubles in `k1,k2,p1,p2,k3` order |
| `rms` | Aggregate point-weighted reprojection RMS, pixels |
| `views count` | Followed by `sequence point_count rms_pixels` for every view |
| `end` | Only whitespace may follow |

Numeric encoding uses the classic locale and enough digits for exact double
round trips. The reader rejects unknown versions/models, missing or extra data,
non-finite/implausible values and inconsistent errors using the existing
[intrinsic validation policy](intrinsic-calibration.md#validation-policy).
Resource bounds: 64 KiB file, 4096-byte ID without control characters, 3–24 views,
at most 1,000,000 points per view, dimensions ≤4096 and image area ≤4 megapixels.
No image storage, worker generation or monotonic timestamp is persisted; capture
sequence values are error-report provenance rather than current frame freshness.

`QtCalibrationProfileStore` performs file I/O via Qt Core on the existing
calibration worker. `QSaveFile` uses atomic replacement with direct-write fallback
disabled. The domain core has no Qt, filesystem-location or OpenCV dependency.

## Preview and lifecycle

`OpenCvCalibrationUndistorter` operates behind `ICalibrationUndistorter` on the
live-preview worker, separate from detection/solving and profile storage, with
newest-frame replacement. Optional optical flow shares this serialized worker.
The Live preview selector switches between holding, tracking and matched-frame
presentation without changing saved profiles or captured observations. In held-overlay mode, the last detected
raw coordinates are remapped with the current image until the next detection;
they can lag a moving board. It supports padded Gray8/RGB8/RGBA8,
produces independent immutable storage and preserves capture timestamp, sequence,
resolution and pixel format. It keeps the original camera matrix and full image
extent. Black borders and interpolation are expected; there is no automatic crop
or new camera model. Corner overlays are transformed with `undistortPoints`
using the same original camera matrix as the image remap. Each presentation keeps
its source frame, transformed corners and acceptance colour together. Adapters
without point remapping suppress the overlay instead of painting raw coordinates
on corrected pixels.

Only presentation is corrected. Calibration observations retain original pixels;
this issue does not alter detection, robot state, world coordinates or control.
The controller retains one active remap and the newest pending raw preview,
serializes file operations and coalesces repeated reload requests. Camera/
resolution changes discard late file/remap results. Capture restarts leave an
in-flight lookup for the same camera valid. Toggle/review changes invalidate
older preview work. Remap failure returns to raw preview with an actionable error;
shutdown drains both calibration workers while the GUI event loop continues.

## Verification and physical checks

Pure GoogleTests cover exact field/precision round trips, malformed/truncated/
unknown/oversized formats, invalid numerics and exact compatibility. Temporary-file
tests cover Unicode paths, camera/resolution key isolation, store recreation,
replacement, failed saves and tampered identity/size. Analytical remap tests cover
all pixel formats, row padding, immutability, timestamps, zero distortion and the
Brown–Conrady sampling equations. Offscreen dialog/controller tests cover save and
reload across controller recreation, raw capture preservation, bounded preview
work, worker exceptions and stale results after input/mode/shutdown changes.

Run the full suite using the [test guide](../development/testing.md), or select:

```sh
ctest --test-dir build/issue-55 -R 'CalibrationProfile|QtCalibrationProfileStore|OpenCvCalibrationUndistorter' --output-on-failure --no-tests=error
```

Physical camera/lens verification is **NOT RUN**. With a real calibrated camera,
compare straight board/table edges in raw/corrected mode, stop/restart the
application and confirm the same profile reloads, then change camera or resolution
and confirm correction becomes unavailable until an exact profile is present.
Inspect black borders and check corner alignment in both raw and corrected modes. Record OS, Qt,
OpenCV, device ID, negotiated resolution and model/RMS. Linux X11/Wayland, Windows
runtime and physical optical accuracy require their own manual verification;
synthetic tests do not establish those results.

### Verification record (2026-10-21)

Executed on macOS, Apple Clang 21, Qt 6 and OpenCV 5.0.0. Debug/Ninja
configurations used the existing local GoogleTest source checkout (no download):

- Desktop configure/build in `build/issue-55`: passed, including production app.
  `ctest --test-dir build/issue-55 --output-on-failure --no-tests=error`:
  **259/259 passed**.
- Core configure with desktop/replay/calibration OFF and build in
  `build/issue-55-core`: passed without Qt/OpenCV.
  `ctest --test-dir build/issue-55-core --output-on-failure --no-tests=error`:
  **164/164 passed**.
- Standalone OpenCV configure with desktop/replay OFF, calibration ON, and build
  in `build/issue-55-headless`: passed without Qt.
  `ctest --test-dir build/issue-55-headless --output-on-failure --no-tests=error`:
  **175/175 passed**.
- `ctest --test-dir build/issue-55 -R '^CalibrationProfileUiTest\.' --repeat
  until-fail:5 --output-on-failure --no-tests=error`: all eight cases passed five
  consecutive runs. A new test observer's teardown lifetime was corrected before
  this repeat and the final full desktop run.
- Offscreen completed dialog rendered and visually inspected: profile status,
  controls, model quality and accepted-corner preview fit at the default size.
- `git diff --check`: passed. Maintainer workspace documentation checker passed
  all 37 maintained Markdown files/anchors; its five tooling tests also passed.
  The documentation CMake target is not present in this branch's base, so no
  documentation target/CTest result is claimed.

Physical camera/lens acceptance, Linux native GUI/BlueZ-independent runtime,
Windows runtime and OpenCV 4 execution were **NOT RUN**. Follow the physical
procedure above; automated geometry/persistence results do not establish optical
accuracy or native platform interaction. At that verification stage the issue branch remained local pending
maintainer push approval.

### CI compiler compatibility follow-up (2026-10-22)

The initial macOS CI build used Apple Clang 17 / Xcode 16.4, whose libc++ lacks
floating-point `std::from_chars`. Local Apple Clang 21 supported the overload,
so the earlier local builds did not reveal the compilation failure. The parser
now uses classic-locale stream conversion for floating-point fields, with strict
decimal syntax and explicit handling of finite subnormals. Unsigned integer
fields retain `from_chars`. No compiler/OS branches or dependencies were added.

Added regression tests preserve signed zero, normal/subnormal values under a
comma-decimal global locale, and rejection of partial, locale-specific, hex,
non-finite, overflowing and underflow-to-zero inputs. Local desktop build and
full CTest passed **261/261**; Qt/OpenCV-free build and full CTest passed
**166/166**. `git diff --check` passed. Updated remote CI is required to verify
the older runner toolchain; these local results do not claim that verification.

Capture results in raw and corrected mode return to live preview after five
seconds from presentation. The capture reason remains readable separately;
explicitly reviewing a stored view cancels automatic return. See the
[capture guide](calibration-capture.md) for timer, camera-loss and lifecycle behavior.

For target setup, image-region examples and clearly labelled board X/Y tilts and
Z rotation, use the wizard's **Board placement and capture help** and the
[placement guide](calibration-capture.md#placement-rotation-and-next-capture-help).
Next-capture hints use accepted-view count/region/scale/in-plane-angle measurements;
they do not measure out-of-plane tilt or certify physical profile accuracy. Keep
focus, zoom and camera settings fixed; changed optics require fresh calibration.
