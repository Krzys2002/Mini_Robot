# CodeCell C6 hardware validation profile and safety envelope (issue #154)

Purpose: record verified hardware facts, unresolved blockers, and safe bench
steps for [#154](https://github.com/Krzys2002/Mini_Robot/issues/154) without
inventing limits. This document is the versioned profile baseline for the
current T1-ESP-C6 firmware at repository revision time.

Status: **partial hardware profile completed, physical polarity/zero-output
measurements pending bench execution**. Motion tuning and powered acceptance for
follow-up issues remain blocked until the unresolved rows below are measured.

## Version and scope

- Profile version: `codecell-c6-v2` (2026-10-27)
- Firmware scope: `T1-ESP-C6/`
- Build target scope: `platformio.ini` environment `esp32-c6-devkitc-1`
- Out of scope: ESP32-S3 video extension (tracked separately by #110)

## 1. Board/build identity and compatibility

| Item | Verified value | Source | Confidence |
| --- | --- | --- | --- |
| Physical module family | CodeCell C6 Drive | User-provided board link; vendor product page | High |
| MCU package | ESP32-C6-MINI-1-H8, single-core RISC-V | Vendor product page | High |
| Memory | 8 MB flash, 512 KB SRAM | Vendor product page | High |
| Wireless capabilities | Wi-Fi 6, BLE 5, Zigbee | Vendor product page | High |
| Firmware build board in PlatformIO | `board = esp32-c6-devkitc-1` | `T1-ESP-C6/platformio.ini` | High |
| Reconciliation statement | DevKitC board setting is used as a PlatformIO Arduino build target and does **not** by itself prove the physical board is a DevKitC. Robot IO is performed through the CodeCell library API (`CodeCell.h`) and must be validated on the actual CodeCell C6 hardware. | `T1-ESP-C6/platformio.ini`, `T1-ESP-C6/src/Main.cpp` | High |
| Reset/boot pin mapping for this deployed board revision | **UNRESOLVED** (not documented in repository evidence) | No board-revision photo/schematic pinned in repository | Low |
| Deployed board revision identifier (silkscreen) | **UNRESOLVED** | Physical inspection not yet attached | Low |

References:

- <https://microbots.io/products/codecell-c6-drive>
- <https://github.com/microbotsio/CodeCell#readme>

## 2. Actuation, geometry, and power facts

### 2.1 Verified motors and wheel geometry

The motor details below are translated to English from the provided source:

Motor model and listing:

- Model: **N20-BT41 micro geared motor, 100:1, 150 RPM, 6 V**
- Product link: <https://botland.com.pl/silniki-micro-n20-seria-mp-medium-power/12603-silnik-n20-bt41-micro-1001-150rpm-6v-5904422306700.html#product_tab_faq>

Translated mechanical/electrical listing values:

| Field | Value |
| --- | --- |
| Motor diameter | 12.0 mm |
| Motor body length | 24 mm |
| Supply voltage range | 1.0 V to 6.0 V |
| Current consumption (listing) | 40 mA |
| Gear ratio | 100:1 |
| Nominal speed | 150 rpm |
| Torque (listing) | 0.3 kg*cm |
| Motor mass | 10 g |
| Shaft diameter | 3.0 mm |
| Wheel diameter | 17 mm |
| Wheel width | 6 mm |
| Track width (wheel-center distance) | **UNRESOLVED / not chosen yet** |

### 2.2 Driver and power system

| Item | Verified value | Source |
| --- | --- | --- |
| Motor driver topology | Two independent onboard H-bridge channels | Vendor product page |
| Advertised driver current capability (not a verified enforced bench limit) | Up to 1500 mA when using one driver channel; approximately 750 mA per channel when both active | Vendor product page |
| Power source in current robot | LiPo battery, 170 mAh, 20C | User-provided hardware data |
| Charging current (board) | Approximately 90 mA | Vendor product page |
| Advertised module supply capability (not a verified enforced bench limit) | 1500 mA from battery or 450 mA from USB | Vendor product page |
| Battery protection board details (cell UVLO/OCP/short protection IC) | **UNRESOLVED** | Not yet evidenced in repository/user artifacts |

## 3. Feedback availability and classification

| Feedback channel | Current status | Evidence | Classification |
| --- | --- | --- | --- |
| IMU yaw/rotation | Used by firmware (`Motion_RotationRead`) for heading control | `T1-ESP-C6/src/controller.cpp` | **Available: IMU heading feedback** |
| IMU linear acceleration | Used for optional distance integration (`Motion_LinearAccRead`) | `T1-ESP-C6/src/controller.cpp` | **Available: IMU linear-accel estimate (drift-prone)** |
| Wheel encoders / wheel speed sensors | No encoder API usage or wheel feedback signal in current firmware | `T1-ESP-C6/src/controller.cpp`, `T1-ESP-C6/src/controller.h` | **Not available (open-loop wheel drive)** |
| Battery voltage and level API | Exposed through CodeCell API (`BatteryVoltageRead`, `BatteryLevelRead`) | `T1-ESP-C6/src/controller.cpp` | **Available but accuracy UNVERIFIED** |
| IMU axis orientation against robot forward | Current firmware assumes forward axis is sensor X; comment notes Y may be required depending on mounting | `T1-ESP-C6/src/controller.cpp` | **UNRESOLVED mounting verification** |

Resulting control classification for #154 acceptance:

- Wheel-speed PID control is **not** justified.
- Current control mode is open-loop differential drive with IMU-assisted heading
  correction and optional IMU-distance estimate.

## 4. Initialization and zero-output behavior audit (code-level)

Code audit refreshed against the merged #214 safety correction; these are source
observations, not electrical verification:

- Setup explicitly writes zero to both drives before starting tasks. Mutex/task
  initialization failures latch a controller fault; no startup motion command is
  issued.
- Motor output buffer initializes to zero speed (`{true, true, 0, 0}`).
- E-stop latches independently of the command handoff mutex. The actuator task
  checks the latch, receipt-relative command expiry and output generation on each
  nominal 20 ms cycle; contention produces zero rather than waiting for control.
- Clear invalidates pending command/computed output and requires new post-clear
  intent. A stale computation cannot restore the old generation.
- This remains the [legacy HTTP prototype](../../../T1-ESP-C6/README.md), not a
  production BLE endpoint or a measured immediate electrical-disable guarantee.
  The nominal 300 ms validity plus scheduler/driver behavior requires physical
  measurement; it does not satisfy the supply/output gates below.

Evidence files:

- `T1-ESP-C6/src/Main.cpp`
- `T1-ESP-C6/src/controller.cpp`

Physical electrical confirmation status:

- Boot-time H-bridge electrical levels at outputs: **UNRESOLVED**
- Actual wheel stationary behavior at boot and under E-stop: **UNRESOLVED**
- Left/right polarity mapping versus signed command: **UNRESOLVED**

These must be measured on wheels-raised hardware before powered motion
acceptance is claimed.

## 5. Safe wheels-raised bench procedure (current-limited)

This is the required safety-gated sequence for #154 and follow-up motion issues.
**Current status: BLOCKED for all powered steps, including flashing, boot and
E-stop checks with the motor supply connected.** The approved supply voltage,
current limit, supply wiring and actuator-level output bound are unresolved.
Only unpowered inspection and preparation may proceed until the following gates
have evidence attached to #154. This document does not establish numerical limits.

### 5.1 Required supply and output evidence before power is applied

- Record an approved supply voltage and current limit, their component-specific
  rationale (board, driver, motors and wiring), and the person approving them.
  The motor listing's 40 mA, driver ratings, USB capability and battery 20C rating
  are not enforced safe bench limits.
- Use a current-limited bench supply or an independently verified inline current
  limiter. Document its model, settings, connection point, polarity, grounding
  and all power paths. Isolate the battery and any USB/charging path that could
  bypass the limiter or backfeed a supply; have the wiring configuration reviewed
  against the actual board. A fresh battery alone is not an acceptable supply.
- Verify the limiter before connecting the robot with a suitable test load and
  meter: record setpoint, measured limiting current, voltage and transient behavior
  against the approved bounds. Do not use a stalled motor as the test load.
  Keep current monitoring and immediate power disconnect available during tests.
- Establish an approved maximum output for **each motor at the actuator write**,
  with explicit units and mapping to the driver API/PWM. Verify enforcement on
  the exact test firmware after heading correction, mixing, startup boost and
  every other transformation, covering all output paths, both signs, startup,
  sustained commands, stop/restart, timeout and E-stop. Attach firmware/library
  revisions, limit configuration and captured final-write traces; confirm the
  electrical output against the bound before allowing wheel motion using an
  approved motor-disconnected measurement setup under the verified supply limit.
  Neither measurement setup nor firmware enforcement is provided by this profile.

The current firmware does not provide a verified low-output bench mode.
`controlTask()` applies `applyStartRamp()` after motor mixing and heading
correction: a mixed wheel target of 10 becomes 70 at movement start and 67 at
100 ms (driver command units). Heading correction can also change each wheel
target. Therefore `move=10` or `rotation=10` does **not** establish an output
bound. A final actuator-level limiter or a separately reviewed bench firmware
must be implemented and verified before this sequence is enabled; lowering the
requested command or merely disabling startup boost is insufficient evidence.

### 5.2 Gated powered sequence

Preconditions (must all be true):

1. Wheels are raised and cannot contact the table.
2. Robot is physically restrained (non-conductive fixture or clamp).
3. All section 5.1 evidence is recorded and approved; the verified limiter is
   active with the documented supply wiring and no bypass power path.
4. One operator has immediate access to supply disconnect and E-stop endpoint.
5. No stall/blocked-rotor test is attempted.

Procedure:

1. Flash the exact reviewed, output-bounded test firmware and open serial monitor
   at 115200, preserving the approved supply configuration throughout.
2. Boot without sending motion command; record motor behavior for 15 s.
3. Trigger E-stop endpoint, confirm both wheels stay at zero command.
4. Clear E-stop, then test forward/reverse translation and both rotation signs
   using the command values and durations approved in the section 5.1 test
   record. Monitor both final actuator outputs and supply current throughout;
   neither wheel may exceed its verified bound. No command magnitude is approved
   by this profile.
5. For each step, record:
   - observed left/right wheel direction,
   - sign-to-direction mapping,
   - measured stop timing after command expiry, including actuator scheduling and
     driver response; compare with separately approved acceptance bounds.
6. Record battery API readings before and after sequence and compare to a
   multimeter reading at battery leads.
7. Disconnect power immediately if current/output exceeds an approved bound,
   the limiter or monitoring fails, or unexpected sustained output, overheating,
   or unstable behavior appears. Record the failure and block further tests.

Blocking rule:

- Missing supply/limiter/output evidence blocks every powered step above.
- If polarity, zero-output behavior, or watchdog stop behavior is not confirmed,
  powered motion acceptance for #154 remains blocked. Issue #154 must remain
  open until required board, polarity, zero-output and feedback measurements
  exist; merging this partial profile does not complete physical acceptance.

## 6. Legacy behavior matrix (keep/adapt/disable)

| Legacy element | Decision | Reason |
| --- | --- | --- |
| IMU yaw heading loop (`yawPidCorrection`) | **KEEP (with validation gate)** | Required for heading-stabilized control; uses available IMU rotation API. |
| IMU linear-accel distance mode (`useImuDistance`) | **ADAPT** | Keep as experimental/diagnostic only until mounting orientation and drift limits are physically characterized. |
| Time-based distance estimator | **KEEP (bounded use)** | Deterministic fallback when wheel feedback is absent; must remain documented as estimate, not odometry truth. |
| Wheel-speed PID assumptions | **DISABLE / not applicable** | No verified encoder feedback path in current hardware/firmware. |
| Startup boost, heading correction and raw motor command range up to 100 | **ADAPT / BLOCKED for bench motion** | Requested command magnitude does not bound final output; require verified per-motor actuator bounds after all transformations and an enforced supply current limit. |

## 7. Explicit unresolved blockers and follow-up actions

| Blocker | Why unresolved | Follow-up action |
| --- | --- | --- |
| Board revision, boot/reset pin evidence | No board photos/schematic revision captured in repository | Add top/bottom board photos with readable silkscreen and pin labels to issue evidence |
| Approved supply voltage/current and enforced limiter configuration | No approved limits, reviewed wiring or limiter measurements exist | Complete section 5.1 supply record and independent limiter verification before any powered step |
| Per-motor actuator-level output bound | Startup boost and heading correction invalidate small-command assumptions; no verified bench limiter exists | Implement/review enforcement and capture final-write and electrical evidence per section 5.1 before wheel motion |
| Left/right polarity map on actual build | Not measured on physical robot in this issue update | Execute section 5 bounded wheels-raised test and attach video/logs |
| Track width | Not selected yet | Measure wheel-center distance in mm after chassis selection and update profile |
| Battery API accuracy/error bounds | No multimeter cross-check evidence captured | Record paired API vs multimeter samples at rest and under low load |
| IMU forward-axis mounting confirmation | Assumed in code comments only | Run axis-orientation check and update `readForwardLinearAccelerationMps2` mapping if needed |
| Battery protection IC capabilities | Not yet documented from battery datasheet/PCM board | Add battery datasheet/protection board data and limits |

## 8. Contributor note

If this document is merged via a dedicated PR, request attribution to the user
who supplied the hardware evidence for this update.
