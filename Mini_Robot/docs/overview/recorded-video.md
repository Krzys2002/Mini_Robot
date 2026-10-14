# Recorded-video replay

Issue #47 adds [RecordedVideoSource](../../src/camera/RecordedVideoSource.hpp)
in `micro_robot_camera_replay`. Its implementation privately uses OpenCV videoio
and imgproc; its public header and the existing `ICameraSource`/`ICameraEvents`
boundary expose only standard C++ and domain values. A vision consumer subscribes
to the same immutable `FramePtr` events as for live or simulated capture.

## Selecting a file and driving playback

Composition selects a local file through `RecordedVideoConfig::file`. The source
normalizes it to an absolute path when constructed, so changing the working
directory later does not change the selection. The path must name a regular file;
URLs, camera indices, and image-sequence patterns are not accepted. Select another
file by constructing another source. The source ID remains stable for its lifetime.
There is no file-picker UI in this issue; the desktop demo continues using its fake
source. Camera selection/preview remain separate work.

```cpp
using namespace std::chrono_literals;
micro_robot::RecordedVideoSource replay({"/path/to/experiment.avi", 40ms, false});
micro_robot::ICameraSource& camera = replay;
camera.setEventSink(consumer); // shared_ptr<ICameraEvents>, retained by the owner
camera.start();
replay.advanceTo(epoch);       // Streaming event, then frame zero
replay.advanceTo(epoch + 40ms); // frame one
camera.stop();
replay.advanceTo(epoch + 80ms); // Stopped event; no further frames
```

All calls and callbacks occur on one owning acquisition worker or test thread.
`start()` and `stop()` only record requests and never perform file I/O or dispatch
callbacks. The driver calls `advanceTo()` to complete them and decode frames.
Decoding/opening may block: do not drive replay from a GUI timer. A future GUI
composition must marshal requests to the owning worker and bridge frame events
to the GUI using a bounded latest-frame handoff. The adapter has no internal
frame queue; it delivers at most one frame per advancement.

## Deterministic timing and lifecycle

- Playback uses a positive fixed `frameInterval` (default 40 ms / 25 Hz). It
  deliberately normalizes media timing; it does not read wall time, use embedded
  timestamps, infer variable frame rate, or sleep. Configure the interval for the
  desired debugging cadence.
- First start maps frame zero to the supplied virtual steady-clock epoch. Each
  decoded frame receives the next scheduled timestamp, even if advancement is
  late. Calls before that time deliver nothing. Late calls consume one sequential
  frame at a time, with no automatic dropping or unbounded catch-up loop. Use
  scheduled advances when evaluating freshness; late driving can intentionally
  produce old virtual observations.
- Repeating an input file, interval, epoch, and request schedule on fresh objects
  reproduces pixels, sequences, and timestamps with the same decoder/backend.
  Cross-codec/platform bitwise equality is not promised. Retained pixels are
  independent copies in packed RGB8, never borrowed OpenCV buffers.
- Requests are idempotent. Stop cancels a pending start. Restart after completed
  stop, EOF, or failure reopens frame zero. Calling start while streaming does
  not rewind. Sequences increase across the object's lifetime. Restart maps frame
  zero to the later of the supplied time and the previous capture plus one interval.
- Non-looping EOF is detected on the next due advancement after the last frame;
  it releases the decoder and emits `Stopped`, without an error or implicit
  restart. Looping reopens the file at EOF and continues sequence/time cadence.
- Missing/non-regular files emit `Fault` plus `Unavailable`. Open/decoder failures
  and empty videos emit `Fault` plus `BackendFailure`; unsupported pixel layouts
  emit `Fault` plus `InvalidFrame`. Failures are quiescent until an explicit start.
  OpenCV's unsuccessful read after some frames is treated as EOF: this backend
  cannot reliably distinguish all truncated streams from normal EOF.
- Time cannot go backward, and recursive advancement is rejected. Callbacks may
  request start/stop or replace the weak sink without nested dispatch. Stopping
  in the Streaming callback suppresses the first frame; stopping in a frame
  callback completes on the next advancement. Destroy on the owning thread,
  outside callbacks, after stop has completed. No background work survives teardown.

## Fixtures and verification

[Replay integration tests](../../tests/camera/RecordedVideoSourceTest.cpp) generate
a three-frame 32 by 24 MJPEG AVI at 25 Hz in an exclusively created temporary
directory and remove it after each case. Fixture creation must succeed; missing
codec support fails the test rather than skipping it. No binary video fixture or
hardware is needed. Tests check channel order with codec tolerance and compare
repeat/loop pixels exactly using the same decoder.

```sh
cmake --build build/debug --target micro_robot_replay_tests
ctest --test-dir build/debug -L replay --output-on-failure --no-tests=error
```

Enable `MICRO_ROBOT_BUILD_REPLAY` (default ON) with OpenCV core/imgproc/videoio.
It also works with `MICRO_ROBOT_BUILD_DESKTOP=OFF`, without Qt. Disable replay
explicitly for a build without OpenCV. macOS automated decoding/lifecycle tests
were executed; Linux/Windows builds and other installed codecs remain unverified.
No physical camera or robot check is required for this local-file adapter.
