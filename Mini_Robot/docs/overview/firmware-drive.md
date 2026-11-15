# CodeCell C6 bounded drive (actuator contract)

Status: implemented by [#160](https://github.com/Krzys2002/Mini_Robot/issues/160) on top of
the [production runtime](firmware-runtime.md) (#158) and
[identity provisioning](firmware-provisioning.md) (#159). It is open-loop actuation with
software bounds on the **command**. No powered or motor test has been run: every physical
claim below comes from vendor sources or is marked as unverified, and powered acceptance
belongs to [#238](https://github.com/Krzys2002/Mini_Robot/issues/238). Nothing here claims
a measured wheel speed, body speed, acceleration or stopping distance.

## Units on the way to the motors

| Stage | Type | Unit | Meaning |
| --- | --- | --- | --- |
| Body command | `runtime::BodyVelocity` | m/s, rad/s (counter-clockwise positive) | Supervisory setpoint from the desktop |
| Wheel intent | `drive::WheelSpeeds` | m/s of wheel surface, positive rolls the robot forward | Commanded, open loop. Never a measurement |
| Output fraction | `double` | share of the H-bridge supply, `[-1, 1]` | Provisional linear actuator map |
| Duty | `drive::MotorDuty` | PWM counts, `[-255, 255]`, per bridge M1/M2 | The value actually written |

Percent appears only in the stored record (`max_output_percent`, DriveCell's unit) and is
converted to a fraction when the profile is built.

## Profile

`drive::DriveProfile` (in [`include/drive/DriveProfile.hpp`](../../firmware/codecell-c6/include/drive/DriveProfile.hpp))
is built from the validated provisioned record by `provisioning::driveProfileFor()`:

| Profile field | Record field | Use |
| --- | --- | --- |
| `trackWidthMetres` | `track_width_um` | Inverse kinematics |
| `maxWheelSpeedMetresPerSecond` | `max_wheel_speed_mm_s` | Wheel saturation and the full scale of the actuator map |
| `maxOutputFraction` | `max_output_percent` / 100 | Approved per-motor bound at the final write |
| `maxLinearMetresPerSecond`, `maxAngularRadiansPerSecond` | `max_linear_mm_s`, `max_angular_mrad_s` | Body command acceptance (also advertised in Robot Info) |
| `leftReversed`, `rightReversed` | `drive_polarity` bits 0/1 | Wiring correction at the final write |

A default profile is invalid, so an unprovisioned robot never wakes the bridges. The
runtime validates the profile again (`ConfigError::DriveProfileInvalid`), and the drive
validates it a third time in `configure()` (`FaultCode::DriveConfigurationRejected`).
The wheel diameter is stored but not used: open-loop control needs only surface speeds.

## Conversion pipeline

Each control step (10 ms) for a permitted setpoint:

1. **Validation before actuation.** `CommandProcessor` rejects a non-finite velocity
   (`RejectedInvalid`). The gate then checks state, validity and range in the v1 order;
   `|v| > maxLinear` or `|omega| > maxAngular` is `RejectedOutOfRange` (protocol
   `OutOfRange`). A rejected command never changes the setpoint or its deadline.
2. **Inverse kinematics:** `left = v - omega b / 2`, `right = v + omega b / 2`.
3. **Saturation.** If either wheel exceeds `maxWheelSpeed`, both are multiplied by the
   same factor. Signs and path curvature are kept, only the speed drops (as in the
   desktop [differential-drive helpers](differential-drive.md)).
4. **Rate and reversal limits** per wheel (`WheelRateLimiter`): a growing magnitude
   changes by at most the acceleration times the elapsed time, and a shrinking one by
   at most the deceleration. Elapsed time is clamped to one control period, so a late
   write never earns a bigger step. A sign change first ramps to exactly zero; the
   opposite direction starts on a later step, so a zero duty is always written between
   the two directions.
5. **Actuator map** (provisional, linear): `fraction = clamp(speed / maxWheelSpeed, -1, 1)
   × maxOutputFraction`.
6. **Polarity, quantization and final enforcement:** the sign is flipped for a reversed
   wheel, the fraction is rounded to counts and clamped to
   `floor(maxOutputFraction × 255)`. Rounding can therefore never add a count above the
   approved bound. Left goes to M1 and right to M2.

There is **no startup boost**. The retired prototype raised small commands to 70 % at
motion start; that is removed, because it exceeded the intent and the approved bound.
The consequence is a physical limitation: a small command may not overcome static
friction, and the robot may not move at all near zero. A deadband or minimum-output
calibration would be [#167](https://github.com/Krzys2002/Mini_Robot/issues/167) work
within the approved bound.

### Signs

With no polarity bit set:

| Command | Left wheel intent | Right wheel intent | M1 duty | M2 duty |
| --- | --- | --- | --- | --- |
| Forward `v > 0` | + | + | + | + |
| Reverse `v < 0` | − | − | − | − |
| Counter-clockwise `omega > 0` | − | + | − | + |
| Clockwise `omega < 0` | + | − | + | − |
| Zero | 0 | 0 | 0 (coast) | 0 (coast) |

A positive duty drives the bridge's OUT1 (M+) high. A set polarity bit flips that
wheel's duty only. The retired prototype drove both bridges M−-positive for forward
(`Drive1.Drive(!forward)`, `Drive2.Drive(forward)` with Drive1's pins swapped), so its
wiring suggests both polarity bits set. That is source analysis of the old code, not a
measurement; the powered polarity check in #238 decides.

### Rates (provisional)

There is no motor, wheel or chassis acceleration evidence. The normal acceleration and
deceleration therefore reuse the #156 stop-ramp budget instead of inventing a value: a
change of the full provisioned wheel speed takes at least `kNormalFullScaleRampTime` =
100 ms (`maxWheelSpeed / 0.1 s`). The stop ramp uses `kStopFullScaleRampTime` = 100 ms,
which `static_assert` ties to `timing::kStopRampMax`. Both are commanded-output limits;
real deceleration, slip and coasting are unmeasured. #167 refines them from calibration.

## Two ways to end motion

| Event | Path | Output |
| --- | --- | --- |
| EmergencyStop, any fault, maintenance, boot, latch clear | Immediate `disable()` inside the gate call | Every bridge input low at once, **no ramp step**, ramp state reset |
| Stop, expiry, session end (disconnect, notifications off, result overload), a refused stale snapshot | Normal stop ramp (#156 B6) | Each control step moves the output toward zero at the stop rate; at zero the drive is disabled |

- The gate records the stop decision time. If the output is not at zero
  `kStopEnforcementLimit` (120 ms) after it, the supervisor faults (`StopRampOverrun`)
  and disables. A control task that stalls during the ramp faults as before
  (`ControlStalled`, B7).
- A ramp step never uses the old setpoint, and its magnitude never increases. An
  E-stop or fault during a ramp disables immediately and the ramp never resumes.
- A new accepted Velocity during a ramp continues from the ramped output at the normal
  acceleration, without a jump.
- From full scale the ramp takes 10 control steps, so the output is zero within
  100 ms plus at most one control period after the decision (B6: ≤ 112 ms).

`RuntimeStatus.stopRamp` and the serial status line (`output=stop_ramp`) show a running
ramp. The status line also prints the last written duty (`duty_m1`, `duty_m2`).

## Motor HAL and board facts

`drive::IMotorDriver` (`enable()`, `coast()`, `write(MotorDuty)`) is the hardware boundary.
The production adapter is
[`src/CodeCellMotorDriver.hpp`](../../firmware/codecell-c6/src/CodeCellMotorDriver.hpp).
Board facts come from the vendor schematic in the pinned CodeCell 2.0.4 library
(`hardware/CodeCell C6 Drive/CodeCell C6 Drive Schematics.pdf`, commit `795616b`):

| Item | Source fact |
| --- | --- |
| Bridges | Two TI DRV8837: U7 → M1, U8 → M2 |
| M1 inputs | IN1 = GPIO21, IN2 = GPIO22 (CodeCell's `Drive1(22, 21)` names them the other way round) |
| M2 inputs | IN1 = GPIO2, IN2 = GPIO3 |
| Bridge sleep | Both nSLEEP on `+3.3V_SENS`, an LDO enabled by GPIO18 (SENS_PWR) with a 10 kΩ pull-down. The same rail powers the BNO085 and VCNL4040 |
| PWM | 20 kHz, 8 bit, as in DriveCell 1.1.3. A new duty applies from the next PWM period (50 µs) |

DRV8837 IN/IN control: both inputs low **coasts** (outputs high-impedance), one input
PWM with the other low drives that direction with coasting in the off phase, and both
high **brakes**. The adapter never brakes. Zero duty and every disable coast. Within a
channel the input that must be low is written first, so both are never high together.

Start-up order: `begin()` in `setup()` attaches the four inputs to LEDC at zero duty
while the bridges are still asleep. `FirmwareRuntime::start()` then disables and loads
the record. Only a valid profile calls `enable()`, which raises GPIO18 to wake the
bridges with every input low. An unprovisioned board therefore never wakes them, and a
failed attach makes the boot disable fail, so the runtime faults.

From these sources the bridges should be asleep through reset and boot: GPIO18 is
pulled down on the schematic, and the TI DRV8837 datasheet specifies internal
pull-downs on its inputs. That, the actual
coasting behaviour and the output timing are **not measured** (#238). Waking the
bridges also powers the on-board sensors; a future IMU user must keep that in mind.

The channel assignment (left = M1, right = M2) follows the prototype's joystick mixing
and is **unverified**. If the rotation-sign check in #238 fails, swap
`kLeftWheelChannel`/`kRightWheelChannel` in `DriveProfile.hpp`. The record has no field
for it.

## Not covered

- Any powered, wheels-raised or bench motor test, polarity, the electrical output
  against the bound, boot/reset pin levels, disable timing and physical stopping:
  [#238](https://github.com/Krzys2002/Mini_Robot/issues/238), then
  [#166](https://github.com/Krzys2002/Mini_Robot/issues/166).
- Calibrated wheel speed, deadband and rates: [#167](https://github.com/Krzys2002/Mini_Robot/issues/167).
- Wheel feedback control: none. The board has no wheel encoders (#154).
- The remaining watchdog and fault recovery: [#162](https://github.com/Krzys2002/Mini_Robot/issues/162).
  Commands reach the drive only once the BLE peripheral exists ([#163](https://github.com/Krzys2002/Mini_Robot/issues/163)).

## Verification

Host tests (`core` label, [`tests/`](../../firmware/codecell-c6/tests/DriveKinematicsTest.cpp)):

- `DriveProfileTest`, `DriveKinematicsTest`: profile rules and exact boundaries; signs
  for forward, reverse, both rotations and zero; the combined-arc values; saturation
  that keeps curvature; extreme inputs; quantization never above the bound; polarity
  and channel assignment.
- `WheelRateLimiterTest`, `BoundedDriveOutputTest`: acceleration and deceleration
  boundaries, the elapsed-time clamp, reversal through an exact zero write, the stop
  ramp within B6, no startup boost, an immediate disable that resets the ramps, refusal
  of non-finite and overflowing input, an input sweep against the bound, and HAL
  failures.
- `ActuatorRuntimeTest`: the real drive in the runtime. E-stop during acceleration and
  E-stop or maintenance during a stop ramp coast inside the call with no later write.
  Stop and expiry ramp monotonically to zero within the budget. A stall during the ramp
  faults. A ramp that never ends is enforced (`StopRampOverrun`). Out-of-range commands
  are rejected before the actuator path. A refused HAL write faults and coasts.
  Profile rejection faults before any task starts.
- `ProvisioningRuntimeTest`: the provisioned record configures the drive profile, and
  records at their exact limits stay valid profiles.

Bench check (USB only, no battery, no motors, 2026-11-15): the production image was
flashed to the unprovisioned board. Status lines reported `start=faulted state=Fault
output=disabled duty_m1=0 duty_m2=0 fault=ConfigurationUnavailable`, so the bridges
were never woken. The board's previous full flash (the `ble-lifecycle` fixture) was
then written back, and an 8 MB read-back matched its SHA-256. This shows the commanded
state only.
