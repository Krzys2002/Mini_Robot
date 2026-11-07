# Review acceptance: metric observations, tracking and safety

## Evidence and scope

The review correction tree combines metric PR #225, tracker PR #227 and desktop
safety PR #229, including the result-notification safety corrections. No physical
setup is available (confirmed by the operator). All physical checks below are
**NOT RUN**. This record does not close #224, #226, #228, #162 or #166.

Software coverage adds independently constructed perspective/oblique/off-center
poses, radial plus tangential distortion, finite near-horizon mapping and horizon
rejection; bounded per-frame rejection counters; and the production vision/tracker
bridge with controlled capture/delivery time, multiple robots, empty frames,
calibration replacement during processing, camera restart/sequence reset, delayed
expired results and shutdown invalidation. These are software tests only.

## Physical metric procedure (#224)

1. Record exact desktop revision, OS, Qt/OpenCV, camera stable identity, resolution,
   capture timestamp method, tag family/IDs/size/mounting, lighting and table setup.
2. Retain active intrinsic and tabletop profiles and independently measured table
   references. Select position/heading limits from the experiment requirements
   before collecting results; record reference uncertainty separately.
3. Place a stationary tag at independently measured center/edge/corner positions
   and quarter/intermediate headings, including angle wrap. Record expected and
   measured x/y/heading, capture time, position norm error, wrapped heading error,
   sampling/quality and diagnostic rejection counts. Retain raw frames and logs.
4. Repeat with multiple tags and unavailable/incompatible calibration, changed
   resolution, camera identity or placement. Verify no incompatible metric pose
   is published; reactivate/recalibrate before continuing.
5. Record each result against the predeclared limits and link failures. A fit
   residual on calibration points is not an independent physical reference.

## Physical tracking procedure (#226)

Use the same unpowered printed tags and evidence setup. Record configured stale
and lost ages and timestamp provenance. Observe two tags; cover only one and log
capture times, delivery times, age and Fresh/Stale/Lost transition times for both.
Reveal before and after loss; require a new valid capture to recover. Stop/restart
and replace the camera, change resolution and invalidate/replace calibration;
verify immediate loss and rejection of pre-change results, including sequence
reset on restart. Repeat representative placement, lighting and cadence cases.
Retain recorded video and event traces; evaluate timing uncertainty explicitly.

## Safety interoperability procedure (#228/#162/#166)

Blocked by production firmware and hardware acceptance. Complete #154's actual
board/supply/independent disable gates and #156's measured output/stop budgets;
implement and verify #162 and the #166 conformance fixture before powered tests.
Do not use the connection-only BLE fixture as production motor-safety evidence.

With the approved wheels-raised fixture and independent electrical instrumentation,
record the exact desktop/firmware/toolchain/board/BLE adapter/configuration revisions.
Exercise fresh Velocity, ordinary Stop, E-stop, accepted/rejected/timed-out clear,
late clear after fault or renewed E-stop, disconnect/reconnect and expired/old
commands. Reproduce both telemetry and CommandResult safety reports. Inject
malformed traffic, queue saturation, stalled ordinary control and radio loss using
#166's approved scenarios. Measure electrical disable and mechanical stopping
separately against every #156 budget, retaining raw timing traces and failures.
Clear/reconnect must remain zero-output until new intent. Repeat macOS/Ubuntu
interoperability separately. All of these results remain NOT RUN here.

## Control gate

The required future autonomous tracking-loss gate and single-camera ownership
contract are specified in [robot tracking](../overview/robot-tracking.md).
Implement and test that command gate before a pose-based autonomous controller is
enabled. Provisional age thresholds and component tests establish neither physical
accuracy nor safe stopping.
