# Project audit corrections — 2026-10-27

Maintainer-requested audit work is isolated on `fix/project-audit`, initially from
master `4ac7d54`, then refreshed with `badd1dd` (merged PR #213). The audit's
`21b6628` snapshot predates the merged workspace, RobotManager and debug-overlay
features; findings were checked against current source before editing.
No hardware was flashed, no new physical acceptance was performed, and no
production firmware/backlog or end-to-end control issue is claimed complete.

## Findings and disposition

| Finding | Correction / boundary | Regression evidence |
| --- | --- | --- |
| Linux camera node reuse loads another camera's calibration | Separate optional physical profile identity from the capture path. USB vendor/product/serial + interface + video index survives renumbering; missing/duplicate metadata disables persistence. Recheck around capture open. Never migrate old node-path profiles implicitly. | Synthetic sysfs/device trees cover renumbering, reused nodes, missing/duplicate serials and stream indices. Service tests verify session-only fallback and camera-key isolation; a reentrant source change cannot republish the old identity. |
| BLE descriptor discovery failure | PR #213 merged during this work. Preserve its explicit full discovery and retained physical connection evidence; property, descriptor and Robot Info validation stay intact. | BLE-enabled production build and real-adapter fake-radio tests. The old audit finding is resolved upstream, not presented as a new physical run. |
| Prototype has no independent actuator expiry; clear replays intent; browser overwrites target mode | Motor task checks its own receipt-relative expiry and accepted-command generation, without waiting for control work. Clear invalidates old intent/output; contention means zero, task/mutex initialization failure latches fault. Browser sends periodic motion only during an explicit joystick gesture and serializes mode changes. | Actual controller compiled against fake clock/semaphore/scheduler/drives; exact expiry, wrap, stalled control, stale snapshots, clear/Stop race, contention and fault checks. Five asynchronous tests execute the actual browser script. Firmware cross-build passed. |
| No pre-merge build, BLE excluded, initial daily checkpoint skips old merges | One read-only Ubuntu GCC PR/manual job enables BLE and marker tools and runs CTest, Python automation and browser tests. Daily macOS/GCC/Clang desktop builds enable BLE. Until a daily suite succeeds, always validate the full current baseline without a time cutoff. | 28 repository automation tests including bootstrap/outage recovery; all four workflow YAML files parse and their expected trigger/job structure was checked. Remote job execution and account billing are separate. |
| Simulation replaces a pending Stop with Velocity | Shared bounded-pending policy: newest Velocity replaces Velocity; Stop supersedes unsent motion and cannot be replaced before dispatch. | The same five burst scenarios run against simulation and production BleTransport with a fake radio; existing codec/freshness/rate-limit behavior remains adapter-owned. |
| Solved calibration cannot remove a bad view | Remove from Complete reopens collection and retains good observations. Invalidate the computed result/active profile/remap, and block saving until recalculation succeeds. Reject mutation during worker/profile I/O. | Pure capture transition and UI solve/remove/recalculate tests preserve raw frame ownership and verify new per-view result metadata. |
| Contradictory current-capability claims | One concise [capability summary](../overview/capabilities.md), linked from overview/architecture/source map. Correct hardware/CI guides and label the root architecture sketch historical. Preserve dated evidence in its owning guides. | Maintained local file/anchor checker and final diff review; historical evidence is retained rather than rewritten as new validation. |

Qt's [discovery contract](https://doc.qt.io/qt-6/qlowenergyservice.html#discoverDetails)
explains full versus skipped value/descriptor reads; PR #213 owns the reported
macOS comparison. Linux physical identity uses the documented
[USB sysfs attributes](https://github.com/torvalds/linux/blob/master/Documentation/ABI/testing/sysfs-bus-usb).
These software identities still cannot prove unchanged optics, camera placement,
or truthful serial numbers from arbitrary hardware.

## Validation actually executed

Host: macOS Apple Silicon, AppleClang 21, Qt 6.12, OpenCV 5.0, CMake 4.4.4,
Ninja/Debug. The existing local GoogleTest source cache was supplied at configure
time with `FETCHCONTENT_SOURCE_DIR_GOOGLETEST` instead of downloading it.

- `cmake --preset macos -DMICRO_ROBOT_BUILD_BLE=ON -DMICRO_ROBOT_BUILD_MARKER_TOOLS=ON -DCMAKE_PREFIX_PATH=/opt/homebrew/opt/qt -DFETCHCONTENT_SOURCE_DIR_GOOGLETEST=/Users/krzysztofsawicki/Documents/Put/sem6/Term_Design/Mini_Robot/build/_deps/googletest-src`
- `cmake --build --preset macos --parallel 3` — passed, including production native BLE.
- `ctest --preset macos` — **407/407 passed**, including 46 BLE, four marker-tool
  tests, GUI rendering/scaling, camera identity, calibration review and real
  prototype-controller host tests. The earlier pre-reentrancy run passed 406/406.
- `cmake --preset core -DFETCHCONTENT_SOURCE_DIR_GOOGLETEST=/Users/krzysztofsawicki/Documents/Put/sem6/Term_Design/Mini_Robot/build/_deps/googletest-src`,
  core build and `ctest --preset core` — **242/242 passed**, without Qt/OpenCV.
- After adding a portable skip for hosts without symlink privileges, core rebuild
  and `ctest --preset core -R V4l2Identity --no-tests=error` — **4/4 passed**.
- `python3 -m unittest discover -s .github/scripts/tests -v` — **28/28 passed**.
- Five browser control scenarios passed in the embedded V8 runtime using the
  checked-in harness and actual page script. The equivalent Node runner is wired
  into PR CI; a standalone Node executable is not installed on this local host.
- V4L2 source syntax check with AppleClang C++20 and installed Qt/OpenCV headers —
  passed; this does not establish a native Linux build or camera behavior.
- `pio run -d T1-ESP-C6` — passed, compile only. Resolved platform 55.3.37,
  Arduino 3.3.7, CodeCell 2.0.8 (`8d57989`), DriveCell 1.1.3; final RAM 48,748 bytes,
  flash 1,129,896 bytes. One upstream CodeCell deprecated-LED API warning was
  observed in the initial dependency build. The inherited dependency URLs remain
  floating; record the resolved versions rather than assuming a reproducible pin.
- Maintained Markdown link/anchor checks, workflow YAML/structure checks and
  `git diff --check` — passed. Local checker tooling came from the pending #196
  worktree; this patch does not claim that PR is merged.

## Unverified work and integration limits

Native Linux/BlueZ/X11/Wayland, Windows and legacy OpenCV 4 execution were NOT RUN
for this patch. Docker CLI exists, but its daemon socket is unavailable. The
filesystem identity resolver and actual firmware logic were tested on the host;
that does not prove physical cameras or electrical output timing.

Physical actuator stopping, scheduler/driver stalls, power/reset behavior and
production protocol/session authorization remain the acceptance work in #156,
#158 and #162. The prototype's actuator-loop gate does not cover a stalled motor
task, failed CPU or failed driver, and is not an immediate electrical E-stop
claim. Experimental targets retain the existing bounded command validity rather
than silently gaining an autonomous timeout exemption.

The requested audit does not substitute for the still-open calibrated metric
localization, tracking freshness, manual/domain safety, closed-loop control and
structured experiment-recording roadmap. Component tests and closed milestones
are not end-to-end acceptance evidence. New GitHub workflows have not run for
this unpublished branch; billing/spending restrictions must be addressed by the
account administrator if they prevent runner startup.
