# CodeCell C6 identity provisioning and validated configuration

Status: implemented by [#159](https://github.com/Krzys2002/Mini_Robot/issues/159) in
[`firmware/codecell-c6`](../../firmware/codecell-c6/README.md), on top of the
[production runtime](firmware-runtime.md) (#158). This page covers the persistent
`RobotId`, the validated configuration record, the USB maintenance path, and how robot
identities relate to the desktop's AprilTag association. Nothing here establishes physical
motion, stopping or calibration. The drive adapter stays inhibited until #160, and BLE
publication of Robot Info and advertising is #161/#163.

## What a robot stores

Each robot stores one record. A default (all-zero) record fails validation, so there is
no built-in fallback configuration and an unset value can never arm the outputs.

| Field | Console key | Stored unit | Rule (`validate()`) | Used by |
| --- | --- | --- | --- | --- |
| `robotId` | `robot_id` | `uint32` | Any value, **including 0**; must be set and confirmed explicitly | Robot Info, advertised name, logs |
| Board profile | `board_profile` | enum | `1` = CodeCell C6 Drive ([#154 profile](../testing/codecell-c6-hardware-154.md)) | Rejects a record made for other hardware |
| Required capabilities | `capabilities` | bit set | bit 0 differential drive (required), bit 1 BNO085 IMU; other bits reserved. A capability this image cannot serve is refused (the IMU is not wired, #158). | Start-up check |
| Drive polarity | `drive_polarity` | bit set | bit 0 left reversed, bit 1 right reversed; others reserved | Drive adapter (#160) |
| Per-motor output bound | `max_output_percent` | DriveCell percent | `1..100` | Drive adapter (#160) |
| Wheel diameter | `wheel_diameter_um` | µm | nonzero | #160/#167 |
| Track width | `track_width_um` | µm (wheel-centre distance) | nonzero | #160/#167 |
| Wheel speed at the bound | `max_wheel_speed_mm_s` | mm/s (wheel surface) | nonzero | Limit consistency, #160 |
| `maxLinear` | `max_linear_mm_s` | mm/s | `1..32767` and ≤ wheel speed | Robot Info |
| `maxAngular` | `max_angular_mrad_s` | mrad/s | `1..32767` and `ω·track/2` ≤ wheel speed (rotation on the spot) | Robot Info |
| `maxCommandValidityMs` | `max_validity_ms` | ms | `1..250` (`kCommandValidityCeiling`, #156) | Robot Info **and** the enforced watchdog ceiling |
| `telemetryPeriodMs` | `telemetry_period_ms` | ms | equal to `kTelemetryPeriod` (100 ms, #156) | Robot Info, telemetry (#164) |

The ranges come only from the [v1 protocol](../protocol/robot-protocol.md#robot-info-record)
and the [timing budgets](../protocol/timing-budgets.md). The firmware does not invent physical
limits. The calibrated values themselves (geometry, polarity, the output bound, and the wheel
speed reached at that bound) are measured and entered per robot. Until #160/#167 and the
powered gate [#238](https://github.com/Krzys2002/Mini_Robot/issues/238) supply measurements,
any value entered is provisional and the drive stays inhibited.

Advertised limits are consistent with enforcement by construction:

- The runtime's validity ceiling is taken from `maxCommandValidityMs`. Robot Info therefore
  never advertises a longer window than the watchdog applies.
- `maxLinear` and `maxAngular` must be reachable within the provisioned wheel speed.
  Combined-motion saturation remains the drive adapter's job (#160).

The validated view (`ValidatedRobotConfig`) returns SI values: metres, metres per second,
radians per second and `std::chrono` durations. Integer wire and storage units stay in the
record and codec.

## Stored layout and versions

Little-endian and packed, 42 bytes for schema 1
([`RobotConfigRecord.hpp`](../../firmware/codecell-c6/include/provisioning/RobotConfigRecord.hpp)):

| Offset | Size | Field |
| --- | --- | --- |
| 0 | 4 | magic `MRCF` |
| 4 | 2 | schema version (`1`) |
| 6 | 2 | record length in bytes, CRC included |
| 8 | 4 | generation (incremented by every provisioning write) |
| 12 | 26 | schema 1 payload, in the order of the table above |
| 38 | 4 | CRC-32 (IEEE) of bytes 0–37 |

The header and the trailing CRC keep their positions in every schema version, so the
firmware can check a record's integrity and age before it interprets the payload.

- A record with another schema version is `UnsupportedVersion`. It is never reinterpreted.
- A future schema adds its own decoder and migrates by re-validating into the current
  structure. It must not silently reuse old limits.

## Storage states and rollback

Two NVS slots (`rec_a`, `rec_b` in namespace `mr_config`) hold records. The
[`ConfigStore`](../../firmware/codecell-c6/include/provisioning/ConfigStore.hpp) loads
both slots, and the **newest intact record** (highest generation, wrap-safe) decides the
state:

| Stored situation | State | Runtime |
| --- | --- | --- |
| Both slots empty (new board, after factory reset) | `Unprovisioned` | `Fault` / `ConfigurationUnavailable`, outputs off |
| Newest intact record is valid | `Provisioned` | Starts; becomes ready |
| Newest intact record is out of version or has invalid content | `Invalid` | Fault, outputs off. The older record is **not** used. |
| Two intact records, same generation, different content | `Invalid` (`ConflictingRecords`) | Fault, outputs off |
| Only torn/corrupt records (CRC, length, magic) | `Invalid` | Fault, outputs off |
| A slot cannot be read | `StorageUnavailable` | Fault, outputs off; provisioning refused |

Writes follow these rules:

- **Provisioning and reprovisioning.** The draft is validated before anything is written.
  The new record goes to the slot that does **not** hold the newest record, with the next
  generation, and is then read back. A torn or corrupt slot is skipped on load, because it
  can only be that interrupted write. Power loss at any point therefore leaves either the
  previous record or the new one. If the board had no record before, the result is
  `Unprovisioned` or `Invalid`, and neither can arm.
- **Factory reset** erases BLE bonds first, then the older slot, then the newest one. An
  interruption leaves either the unchanged newest record or nothing, never an older record.
  Running the reset again completes it.
- **No resurrection.** A superseded record never becomes active again, even when the newer
  one is unusable (for example after a downgrade to an image that does not understand the
  newer schema).

NVS also recovers the old or new value of a single key after power loss. The CRC and the
generations are an independent check. Arduino reformats an NVS partition that it cannot
initialize; the robot then reports `Unprovisioned`.

### Wear policy

- Flash is written only by `commit` (one ~42-byte blob) and `factory-reset` (two key erases
  plus the bond namespace).
- Boot only reads.
- Commands, telemetry, E-stop, session ends and status reporting never write. This is
  host-tested: 200 control cycles, stops, E-stop and session end performed zero writes.
- NVS spreads entries across its pages (the default `nvs` partition is 20 KiB). Even
  thousands of deliberate reprovisionings stay far below the flash erase endurance.

Erased NVS entries stay physically in the page until NVS compacts it. A raw flash dump can
therefore show old record bytes after a factory reset. The record holds no secret
(identity and limits only, #157), so this is acceptable. Use
`pio run -t erase` for a full wipe.

## USB maintenance path

The production image accepts maintenance commands on USB serial only (115200 baud,
newline-terminated, at most 96 characters). There is no motion, clear or JSON input.
The [console](../../firmware/codecell-c6/include/provisioning/MaintenanceConsole.hpp) runs
in Arduino `loopTask` (priority 1).

| Command | Needs maintenance | Effect |
| --- | --- | --- |
| `help` | no | Lists commands and field keys |
| `status` | no | Stored state (`PROV ...`), identity, name and every field (`CONFIG ...`) |
| `maintenance` | — | Stops the runtime, clears active intent, disables the drive and refuses motion until restart. The draft starts as a copy of the valid stored record, or empty. Refused if the drive disable cannot be confirmed. |
| `draft` | yes | Shows the draft; fields not yet set print `unset` |
| `set <key> <decimal>` | yes | Edits one draft field (range-checked for its stored width) |
| `commit <robot_id>` | yes | Needs every field set and the RobotId typed again. Validates, writes atomically, reads back, then restarts. |
| `factory-reset ERASE` | yes | Erases bonds and both slots, verifies `Unprovisioned`, then restarts |
| `restart` | no | Clears intent, then restarts |

Maintenance is one-way until restart, and a successful commit or reset always restarts. After
a restart the robot boots `Idle` with outputs disabled, and only a new BLE session sending new
Velocity frames can move it. The RAM E-stop latch does not survive the restart, as in v1. The
desktop re-sends E-stop on reconnect if its own safety state is latched
([BLE session safety](../protocol/ble-gatt.md#session-safety)).

Entering maintenance also keeps an existing E-stop latch or fault. Flash writes therefore
happen only with the drive already off. A flash stall cannot delay a safety decision that
matters, because no output is enabled during maintenance.

### Procedure

USB only, with the battery and motors disconnected, as for every upload
([firmware guide](../../firmware/codecell-c6/README.md#device-discovery-upload-monitor-and-recovery)):

```text
maintenance
set robot_id 7
set board_profile 1
set capabilities 1
set drive_polarity <bits>
set max_output_percent <approved bound>
set wheel_diameter_um <measured>
set track_width_um <measured>
set max_wheel_speed_mm_s <measured at the bound>
set max_linear_mm_s <advertised>
set max_angular_mrad_s <advertised>
set max_validity_ms 250
set telemetry_period_ms 100
draft
commit 7
```

To reprovision, run `maintenance`, change only the fields that differ, then run
`commit <robot_id>`. To decommission, run `maintenance` then `factory-reset ERASE`. Record
the `status` output, together with the firmware provenance, for every robot you provision.

## Controller credentials

The restricted-lab v1 profile proposed by #157 (PR #274, not yet merged) defines no controller
keys, passkeys or passwords. The record therefore contains no credential, and `status` prints
identity and limits only. Factory reset also erases NimBLE's bond store (NVS namespace
`nimble_bond`, the same store `NimBLEDevice::deleteAllBonds()` clears). Bonds left by an earlier
image, such as the BLE fixture, therefore do not survive. This image links no BLE stack, so
it erases that namespace directly. Any future credential belongs to #165 and needs its own
reviewed storage (NVS encryption). It must never be added to this record or printed.

## Robot identity management and the desktop

- **One `RobotId` per physical robot, unique within the arena.** The firmware cannot check
  uniqueness across robots. The operator keeps an inventory (robot label, `RobotId`, marker
  tag, firmware provenance and the `status` output). A duplicate is caught on the desktop:
  two advertisers with the same `MR-<id>` name in one scan are reported as ambiguous and
  refused ([BLE advertising](../protocol/ble-gatt.md#advertising-and-discovery)).
- **Identity is independent of BLE addresses**, platform peripheral UUIDs and discovery order
  ([robot protocol](../protocol/robot-protocol.md#roles-and-identity)). Moving a record to a new
  board means provisioning that board with the same values and factory-resetting the old one.
- **Marker association (#49).** The desktop vision profile maps AprilTag IDs 0–15 to
  `RobotId{tagId + 1}` ([AprilTag selection](../vision/apriltag-selection.md#robot-identity-and-mounting-convention)).
  A robot that carries tag *n* must therefore be provisioned with `robot_id` *n + 1*, so
  IDs 1–16 are the vision-trackable range.
  - `RobotId 0` and IDs above 16 are valid identities, but they have no marker in the current
    vision profile. Such a robot can be connected, but it is not tracked.
  - The desktop registry (#97) owns any further mapping. The firmware stores only the
    `RobotId`.
- **Publication.** `ProvisionedConfiguration::robotInfo()` returns the `robotId` and the four
  advertised limits for Robot Info, and `advertisedName()` returns `MR-<robotId>`. Both come
  from the same boot record, so the identity a desktop reads cannot differ from the advertised
  one, nor from the enforced validity ceiling. An unprovisioned or invalid robot returns no
  identity, and the BLE peripheral (#163) must not advertise in that case. Robot Info stays
  unchanged until the next restart.

## Verification

Host tests (`core` label, [`tests/`](../../firmware/codecell-c6/tests/ConfigStoreTest.cpp)):

- `RobotConfigRecordTest`: every range rule and its boundary values; RobotId 0, 1, 2³¹ and
  2³²−1 round-trips; the stored byte layout; every single-bit flip and every truncation
  detected; other schema versions intact but unsupported; SI accessors; Robot Info values and
  `MR-<id>` names from the same record.
- `ConfigStoreTest`:
  - absent, partial, corrupt, oversized, unreadable, conflicting and out-of-version slots;
  - no fallback past an unusable newer record;
  - A/B alternation and generation wrap;
  - failed and silently lost writes;
  - a power cut after each mutation, with the interrupted write torn at every byte, for
    provisioning, repeated reprovisioning and factory reset;
  - one write per provision and none on load.
- `MaintenanceConsoleTest`:
  - maintenance is entered before any write;
  - a refused disable blocks writes;
  - unset fields, wrong confirmations, invalid drafts and out-of-width values are rejected;
  - reprovisioning starts from the stored record;
  - reset needs `ERASE`;
  - no motion command exists, and no credential-like field is printed.
- `ProvisioningRuntimeTest`:
  - no configuration (each bad case) arms outputs, and the drive is disabled before the load;
  - the advertised validity is the enforced ceiling;
  - maintenance clears an active setpoint, keeps the latch or fault, and cannot be followed by
    readiness;
  - provision, reboot, reprovision, reset and reboot through the console, with no resumed
    motion;
  - no writes in ordinary operation.

Bench check, 2026-11-15 (USB only, battery and motors disconnected, one CodeCell C6 Drive,
image SHA-256 `d0876c5f…`):

1. A full 8 MB flash backup was taken.
2. **First boot:** `start=faulted state=Fault output=disabled fault=ConfigurationUnavailable
   provisioning=Unprovisioned robot_id=none`.
3. **Provisioned `robot_id 0`.** `commit 0` reported `generation=1 slot=A` and restarted. The
   new boot reported `start=running ready=1 state=Idle output=disabled
   provisioning=Provisioned robot_id=0`, and `status` showed `name=MR-0`.
4. **Reprovisioned to `4294967295`.** The commit reported `generation=2 slot=B`. After the
   restart and after a separate RTS hardware reset, the status showed `robot_id=4294967295`
   and `name=MR-4294967295`.
5. **Rejected commands:**
   - `set` and `commit` outside maintenance: `ERR not in maintenance`;
   - `velocity 0.1 0`: unknown command;
   - `commit 99` for a draft with RobotId 4294967295: confirmation error;
   - a draft with `telemetry_period_ms 50`: `RejectedInvalid ... TelemetryPeriodMismatch`, and
     the stored record was unchanged.
6. **Factory reset:**
   - `factory-reset yes` was refused;
   - `factory-reset ERASE` reported `Unprovisioned bonds=erased`;
   - the next boot was in `Fault` with outputs disabled and `provisioning=Unprovisioned`.
7. **Restore:** the original flash was written back in 1 MB chunks. A single 8 MB write had
   stopped responding near the end. A full read-back matched the backup's SHA-256, and the
   restored `ble-lifecycle` fixture reported `id=1 profile=normal`.

This shows the commanded state, persistence and console behaviour on one board only. Not run:

- a true power-supply interruption during a write (simulated on the host only);
- distinct IDs on several robots (only one board is available; #168);
- Robot Info and advertising on air (#163);
- electrical pin state (#238);
- any powered or motor test.
