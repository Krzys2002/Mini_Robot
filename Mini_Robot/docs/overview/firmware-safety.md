# CodeCell C6 safety: command watchdog, E-stop, faults and recovery

Status: implemented by [#162](https://github.com/Krzys2002/Mini_Robot/issues/162) on
top of the [runtime](firmware-runtime.md) (#158), the [bounded drive](firmware-drive.md)
(#160) and the [v1 codec](firmware-protocol.md) (#161). It is host-tested on a fake clock
only. **No hardware-in-the-loop check has run.** Physical output timing, reset pin levels
and stopping belong to [#238](https://github.com/Krzys2002/Mini_Robot/issues/238) and
[#166](https://github.com/Krzys2002/Mini_Robot/issues/166).

## The invariant

The robot stays safe without the desktop, without radio callbacks and without progress
of the ordinary control task:

- Nonzero output needs all of the following, checked in the actuator gate at the moment
  of every write: the runtime is ready, a session is open, the state is `Moving`, the
  setpoint's generation is the current one, and `now < deadline`.
- The safety supervisor (priority 22, every 5 ms) ends motion at the deadline and
  enforces the stop ramp and the control heartbeat. The control task is not involved.
- The task watchdog resets the chip if the supervisor itself stops.
- Every boot starts from zero output with nothing restored.

There is still exactly one safety and actuator authority, `ActuatorGate`, and one command
authority, `CommandProcessor` ([components](firmware-runtime.md#components-and-ownership)).

## States and readiness

The public states are the v1 states. Two internal conditions also gate motion.

| Condition | Set by | Cleared by | Without it |
| --- | --- | --- | --- |
| Start-up readiness | `start()` completed with every task running | Any fault, maintenance, shutdown; never set again before a reset | Velocity → `RejectedNotReady` |
| Session readiness | `openSession()` when the session becomes ready (both CCCDs, #163) | `sessionEnded()` for that session, or a newer `openSession()` | Velocity and ClearEmergencyStop → `RejectedSessionNotReady` |

Both outcomes map to v1 `RobotFault`. v1 sends no result while the session is not ready
([BLE session establishment](../protocol/ble-gatt.md#session-establishment)). Stop and
EmergencyStop need neither condition: they always take effect.

### Transition table

`SafetyTransitionTest.EveryStateAndEventFollowsTheTable` checks every cell. *Stale* means
a token from an ended or replaced session.

| From \ event | Velocity | Velocity (stale) | Stop | E-stop | Clear | Clear (stale) | Expiry | Open session | End session | End (stale) | Fault | Maintenance | Shutdown |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| Idle | Moving | Idle | Idle | E-stop | Idle | Idle | Idle | Idle | Idle | Idle | Fault | Idle | Idle |
| Moving | Moving (new intent) | Moving | Idle | E-stop | Moving (no-op) | Moving | Idle | Idle | Idle | Moving | Fault | Idle | Idle |
| E-stop | E-stop | E-stop | E-stop | E-stop | **Idle** | E-stop | E-stop | E-stop | E-stop | E-stop | Fault | E-stop | E-stop |
| Fault | Fault | Fault | Fault | Fault | Fault | Fault | Fault | Fault | Fault | Fault | Fault | Fault | Fault |

After every event, the newest generation is drivable only in `Moving`. The original
setpoint of a `Moving` robot survives only the no-op events: a stale Velocity, a no-op
Clear, a stale Clear and a stale session end.

## Command watchdog

- **Deadline.** On admission, `deadline = min(receipt, now) + min(validFor, ceiling)` on
  the robot's 64-bit microsecond clock. It replaces the previous deadline. `now` is the
  gate's own clock reading. A receipt stamped later than `now` is therefore not trusted
  beyond it, and no stamp can extend validity past `now + ceiling`.
- **Expiry** happens at `now >= deadline`. The supervisor then starts the normal stop
  ramp (B6), enters `Idle` and sets `commandExpired`. The gate also refuses a snapshot at
  `now >= deadline` on its own.
- **A command already past its deadline** when it is admitted (a delayed callback) still
  supersedes the previous command. It expires in the same call, so the old command stops
  and the new one is never driven.
- **Nothing else extends a deadline.** Rejected frames, duplicate or older sequences, a
  stale Clear, telemetry, CommandResults, diagnostics and status reads leave it unchanged.
  See `SafetyTelemetryTest.RejectedDuplicateAndDiagnosticTrafficNeverExtendTheDeadline`.
- **Clock.** `deadlineAfter()` refuses a sum the clock cannot represent
  (`RejectedInvalid`, no motion) instead of wrapping into the past or saturating into
  a deadline that never comes. The `esp_timer` counter cannot get near that limit. The
  tests put deadlines across the uint32 microsecond and millisecond wrap points and near
  the end of the int64 range. Telemetry `uptimeMs` and `millis()` are never used for
  safety.
- **`commandExpired`** (telemetry flag, `RuntimeStatus.commandExpired`) is set when the
  watchdog ends motion. It is cleared by the next accepted Velocity or Stop, as v1 says,
  and when a new session opens. The flag reports on the session's own commands; this is
  a firmware choice, v1 does not say. `CommandSession::telemetry()` takes `state` and
  this flag from the runtime, so a caller cannot report stale values.

### Bounds

| Bound (#156) | How it holds without the control task | Host evidence |
| --- | --- | --- |
| B4: deadline → expiry decision ≤ 7 ms | One 5 ms supervisor period plus ≤ 2 ms lateness | `ExpiryHappensExactlyAtTheDeadlineAcrossClockWrapPoints` |
| B5: last valid Velocity → output zero or disabled ≤ 400 ms | Expiry, ramp enforcement (120 ms), one more check, disable | `LastValidCommandIsBoundedAtTheOutputsWithControlRunningStalledOrPaused` (motor fake: last duty zero and drive disabled) |
| B6: normal stop ramp | Enforced by the supervisor (`StopRampOverrun`) | #160 tests, and the paused-control case above |
| B7: stalled control → disabled ≤ 40 ms | Heartbeat check in the supervisor | `StalledControlTaskIsDisabledByTheSupervisorWithinB7`, `DeadControlTaskAtRestIsATaskFailure` |
| B8: E-stop callback entry → disable ≤ 1 ms | Disable inside the call, before parsing, results or mailbox | `EmergencyStopDisablesBeforeAnyResultOrStatusIsProduced`, `EmergencyStopWithAStalledControlPathCoastsInsideTheCall` |
| B17: stalled supervisor → reset | Task watchdog, 50 ms | Not host-testable; HIL NOT RUN |

On the fake clock these bounds are checked on the commanded output only. Electrical and
physical timing are not covered here.

## Sessions

`IMotionCommandSink::openSession()` returns a `SessionToken`. Velocity and
ClearEmergencyStop carry it, and only the current token is admitted. The BLE adapter
(#163) drives the lifecycle through `protocol::v1::CommandSession`:

- `open()` restarts the sequence counters and calls `openSession()`. A second `open()`
  is a **replacement**: the previous session ends as for Stop, and its token is never
  current again.
- `end(reason)` restarts the counters and calls `sessionEnded(token, reason)` for
  disconnect, notification (CCCD) disable or result overload. Motion ends as for Stop
  (B6 ramp, B19). The E-stop latch and faults are kept.
- A late end for a token that is no longer current changes nothing and is not counted.
  That session's intent was already invalidated when it stopped being current.
- Every session end and every replacement empties the setpoint mailbox. Active intent (a
  setpoint being driven) and pending intent (admitted but not yet read by the control
  task) are both invalidated by the generation check, and nothing stays buffered.

A frame from an earlier session therefore cannot move the robot, even if the adapter
delivered one late. The robot also keeps no command to replay after a reconnect. The
desktop still has to discard its own queue (v1 session rules).

## Emergency stop

- `emergencyStop()` disables the drive inside the gate's critical section before the
  latch is visible. Only after that does it empty the mailbox and count the event. On
  the Emergency Stop characteristic, `onEmergencyStopWrite()` calls it before parsing,
  results or telemetry. A Velocity flood, a full mailbox or a stalled control task
  cannot delay it.
- While latched, Velocity is `EmergencyStopLatched`. Stop is accepted with no effect,
  and EmergencyStop is accepted again.
- ClearEmergencyStop from the current session releases the latch to `Idle` with the
  drive disabled. The mailbox is already empty, and the old generation is dead, so
  motion resumes only with a new Velocity. A Clear while not latched is a no-op and
  does not touch a running setpoint.
- **The latch persists across disconnect, reconnect and session replacement.** It is
  held in RAM only. As v1 specifies, **a reset clears it**: the robot boots `Idle` with
  outputs off, and after any reconnect the desktop must re-assert E-stop if its own
  safety state is in E-stop ([robot states](../protocol/robot-protocol.md#robot-states)).
  No persistent latch exists beyond v1.

## Faults and task failures

| Failure | Detection | Fault code |
| --- | --- | --- |
| Start-up step | `start()` checks each step | `DriveDisableFailed`, `ConfigurationUnavailable`, `ConfigurationInvalid`, `DriveConfigurationRejected`, `SynchronizationUnavailable`, `SupervisorTaskStartFailed`, `ControlTaskStartFailed` |
| Control task stalled while driving (B7), or while ready at rest | Supervisor heartbeat: no control update for 30 ms | `ControlStalled` |
| Stop ramp not at zero 120 ms after the decision (B6) | Supervisor | `StopRampOverrun` |
| Drive refused a permitted write or a ramp step | Gate | `DriveWriteFailed` |
| Drive could not confirm a disable | Gate | `DriveDisableFailed` |
| Supervisor stalled | Task watchdog (below) | Reset; reason `task_watchdog` |

- A dead control task at rest is a task failure too. Otherwise Velocity would be
  `Accepted` and could never be executed.
- **Every fault disables the drive, then powers it down.** `IDriveOutput::powerDown()`
  puts both DRV8837 bridges to sleep by driving GPIO18 low, which switches off their
  nSLEEP supply. That works independently of the PWM peripheral. If a disable cannot be
  confirmed, the power-down is the last resort.
- Faults are sticky and keep the first code. v1 has no desktop recovery, so the only
  recovery is a reset. Reconnecting, Clear, maintenance and the console change nothing.
  A test runs for a second after a fault and finds no write.
- The optional inertial sensor is not used for control. A failed read is counted
  (`sensor_unavailable`) and has no safety effect. A read that blocks stalls the control
  task, which the heartbeat catches. The production image wires no sensor.

## Supervisor watchdog (B17)

- The FreeRTOS platform adapter subscribes only `mr_safety` to the ESP-IDF task
  watchdog (TWDT). It sets the timeout to 50 ms, with panic enabled and the idle task
  not watched; the pinned sdkconfig does not watch idle either. Subscription happens
  before `startPeriodicTask()` returns. If the subscription fails, the start fails
  (`SupervisorTaskStartFailed`), so an unwatched supervisor never runs.
- The task is fed only after each completed `step()`. A supervisor that blocks or stops
  completing steps stops feeding it.
- When the watchdog expires, the ESP-IDF hook `esp_task_wdt_isr_user_handler()` (in
  `main.cpp`) first sleeps both bridges with one register write, then the panic and the
  reset follow. The pinned IDF calls the hook before the panic: this was checked in the
  disassembly of `task_wdt_isr`. Without the hook, the outputs would keep their last
  state through the panic dump, which the #156 pilot measured at about 0.41 s.
- Maintenance releases the watchdog once the drive is confirmed off. NVS writes and
  erases stall the whole single-core chip for longer than 50 ms, and maintenance
  refuses motion until reset anyway. The supervisor unsubscribes itself on its next
  step, so no feed races the removal. Any future flash user while motion is possible,
  such as BLE bonding (#163/#165), must keep within the watchdog timeout.

## Reset, brownout and boot

- **Hardware.** GPIO18 has a 10 kΩ pull-down, so through reset and boot both bridges
  should be asleep and their outputs high-impedance until firmware wakes them
  ([board facts](firmware-drive.md#motor-hal-and-board-facts)). This is a source fact,
  **not a measurement** (#238). It covers every reset cause: power-on, brownout
  (hardware reset or the IDF brownout interrupt), task or interrupt watchdog, panic and
  `esp_restart()`.
- **Boot.** `start()` disables the drive first, and only a valid provisioned profile
  wakes the bridges. Every boot starts `Idle` with no session, no setpoint, no latch and
  `commandExpired` clear (`ResetReturnsToZeroOutputIdleAndRestoresNothing`). A token
  from before the reset is not a session of the new boot.
- **Deliberate restart.** `main.cpp` registers `FirmwareRuntime::shutdown()` as an
  ESP-IDF shutdown handler, so `esp_restart()` (for example the maintenance `restart`)
  first ends motion and the session and disables and powers down the drive.
- **Reset reason.** The boot line (`BOOT reset_reason=…`) and every status line
  (`reset=…`) report `esp_reset_reason()`, for example `power_on`, `task_watchdog`,
  `brownout` or `panic`. It is diagnostic only: no reason restores or refuses anything.
- **Not hooked.** A generic panic, the interrupt watchdog and a brownout have no
  application hook before the reset. Until the reset, the outputs keep their last
  written state. The duration of that window and the electrical state are unresolved
  (#238).

## Verification

Host tests (`core` label), in
[`SafetyEnforcementTest.cpp`](../../firmware/codecell-c6/tests/SafetyEnforcementTest.cpp),
plus the adapted #158–#161 suites:

- deadline arithmetic and the boundary before, at and after the deadline, at four clock
  points × four validity windows; an unrepresentable deadline; a future receipt; a
  command already past its deadline;
- sessions: no session, ended, replaced, a late stale end; active and pending intent for
  disconnect, CCCD disable, result overload and replacement; the latch across
  disconnect, reconnect and replacement, and clear to zero idle;
- races: E-stop and Clear between the control task's copy and write; E-stop ahead of
  any result or status read behind a flood with control stalled;
- outputs at the motor fake: B5 for control running, stalled and paused at four
  validity windows; E-stop with the control path stalled mid-acceleration;
- faults: the watchdog subscription, maintenance release, a dead control task at rest,
  disable before power-down, faults that never recover, shutdown, and reset that
  restores nothing;
- the mailbox after every motion-ending transition, the full transition table, the
  not-ready states, and telemetry flags across every transition.

A deliberate break of each new guard (receipt clamp, session check, rest heartbeat,
E-stop mailbox clear, fault power-down, expiry flag, expiry at admission, watchdog flag)
makes at least one of these tests fail.

**NOT RUN**, owned by #238 (powered gate) and then #166 (HIL suite):

- pausing and failing control work on the board;
- suppressing commands and measuring expiry at the pins;
- triggering the task watchdog, and checking that the ISR hook sleeps the bridges
  before the panic;
- reset and power-drop or brownout pin levels;
- E-stop and clear electrical timing.

Each of these must measure outputs, not status flags.

Bench check, 2026-11-15 (USB only, battery and motors disconnected, one CodeCell C6 Drive;
production image SHA-256 `c1558a83…`). This shows serial status and reset behaviour only;
no pin was measured.

1. A full 8 MB flash backup was taken.
2. **Unprovisioned boot:** `BOOT reset_reason=usb shutdown_handler=registered`, then
   `start=faulted state=Fault output=disabled fault=ConfigurationUnavailable session=0`.
3. **Provisioned** with the host-test fixture values (`validConfig()`, RobotId 0). The
   commit restarted through `esp_restart()` with `reset_reason=software`, and the board
   reported `start=running ready=1 state=Idle output=disabled fault=None`. The supervisor
   ran on the 50 ms task watchdog for 60 s with no reset.
4. **Reprovisioned** to RobotId 1 from maintenance, which released the watchdog. The NVS
   commit (`generation=2 slot=B`) completed without a watchdog reset, and the next boot
   ran another 30 s. The console `restart` booted again with `reset_reason=software`.
5. **Supervisor stall:** a local, uncommitted variant of this image (one extra console
   command that makes the watched task spin; SHA-256 `a6a14a50…`) was flashed. Twice, the
   watchdog reported `mr_safety` about 50 ms after the stall line, measured on the host
   clock, so USB latency adds a few milliseconds of uncertainty. It aborted with a panic
   dump of about 0.41 s and rebooted with `reset_reason=task_watchdog`, `state=Idle
   output=disabled session=0`.
6. **Restore:** the original flash was written back and read back with a matching
   SHA-256.

The bridges' sleep in the watchdog hook, and the pin levels throughout, were not
observable without instruments and remain NOT RUN.

## Not in this issue

- The GATT server, CCCD callbacks and advertising that call `open()` and `end()`: #163.
- Telemetry publication and scheduling, and the reset or fault reason in telemetry: #164.
- Security and authorization of the controller: #165.
- Any persistent E-stop or recovery beyond v1.
