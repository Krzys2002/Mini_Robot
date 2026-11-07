# Physical BLE connection test: issue #187

Purpose: execute [#187](https://github.com/Krzys2002/Mini_Robot/issues/187) on a
CodeCell ESP32-C6 and the actual desktop Bluetooth adapter. This procedure tests
discovery, connection readiness, failures, cancellation and teardown. It does
not establish command delivery, motor safety, reconnect policy or latency
acceptance. Keep #187 open until its physical acceptance evidence is recorded.

**Initial status: every physical test is NOT RUN.** Building firmware or passing
fake-radio tests cannot change that status. The source of truth for expected
bytes/properties remains [BLE GATT](../protocol/ble-gatt.md) and the
[robot protocol](../protocol/robot-protocol.md). The desktop lifecycle is described
in [BLE connections](../overview/ble-connection.md).

The [2026-10-26 physical run report](ble-connection-187-results-2026-10-26.md)
records the first CodeCell execution, the discovery defect/fix, passed cases
and remaining NOT RUN checks. The initial status above describes preparation;
use the report for actual recorded outcomes.

## 1. Equipment and evidence directory

Use a CodeCell ESP32-C6, a USB data cable, a macOS computer with Bluetooth, Qt 6
including Qt Bluetooth, the desktop build dependencies, and PlatformIO Core
(`pio`). A second independent BLE board is needed only for duplicate identity
testing; if unavailable, record that case NOT RUN. Disconnect any attached motor
driver/actuator while using this connection fixture.

Run commands from this repository directory (the directory containing the root
`CMakeLists.txt`). Keep evidence under ignored `build/` and copy it to durable
storage or attach it to the issue after testing:

```sh
mkdir -p build/acceptance-187
git rev-parse HEAD > build/acceptance-187/desktop-commit.txt
git status --short > build/acceptance-187/worktree.txt
sw_vers > build/acceptance-187/macos.txt
system_profiler SPHardwareDataType SPBluetoothDataType > build/acceptance-187/hardware.txt
pio --version > build/acceptance-187/platformio.txt
```

Record the CodeCell revision/flash size, power source, desktop adapter, actual Qt
version (from the CMake configuration/package installation), operator and date.
Review hardware reports for private device information before sharing them.

## 2. Compile and upload the fixture

The [fixture README](../../firmware/ble-lifecycle/README.md) states its exact
limitations. Its PlatformIO settings are independent of the production firmware;
the platform/library releases are pinned. Run:

```sh
pio run -d firmware/ble-lifecycle
pio device list
```

Connect the CodeCell over USB and identify its port. Replace `/dev/cu.usbmodemXXXX`
below with the actual port, not a Bluetooth incoming port:

```sh
pio run -d firmware/ble-lifecycle -t upload --upload-port /dev/cu.usbmodemXXXX
pio device monitor -d firmware/ble-lifecycle --port /dev/cu.usbmodemXXXX --filter log2file
```

The monitor uses 115200 baud from `platformio.ini`; `log2file` creates a timestamped
serial log. Copy that file into the evidence directory after each session. If
uploading fails, close other serial monitors, verify the data cable, and use the
CodeCell revision's documented bootloader procedure. Do not erase flash as a
routine recovery step. Re-uploading usually preserves the fixture's NVS settings.

Send newline-terminated commands through the serial monitor:

```text
id 1
profile normal
status
```

`id` and `profile` persist immediately and reboot the board; reopen the monitor
if USB disconnects. Send each command after the board has rebooted, rather than
pasting all three at once. `status` reports the loaded values without rebooting.
`id` accepts decimal 0–4294967295. Start with ID 1, matching the desktop below.
Initial boot messages can be missed before USB attaches; `status` recovers the
configuration. Record firmware Git revision plus binary hash and package versions:

```sh
git rev-parse HEAD > build/acceptance-187/firmware-commit.txt
shasum -a 256 firmware/ble-lifecycle/.pio/build/codecell-c6/firmware.bin > build/acceptance-187/firmware-sha256.txt
pio pkg list -d firmware/ble-lifecycle > build/acceptance-187/firmware-packages.txt
```

Normal Robot Info is firmware 0.1.0, protocol range 1–1, provisioned ID, maximum
linear 300 mm/s, angular 6000 mrad/s, validity 500 ms and telemetry period 100 ms.
For ID 1 the complete record is:

```text
01 01 01 01 00 00 00 00 01 00 2C 01 70 17 F4 01 64 00
```

Use a BLE GATT inspector if available to save the actual five characteristic
UUIDs, properties, raw Robot Info and both 0x2902 CCCDs. The full UUID table is in
the protocol document. Disconnect the inspector before testing the desktop:
the fixture accepts only one central. A macOS peripheral identifier may be a UUID;
do not require a MAC address that the native API does not expose.

## 3. Build and launch the real desktop

```sh
cmake -S . -B build/issue-187 -G Ninja -DCMAKE_BUILD_TYPE=Debug -DMICRO_ROBOT_BUILD_BLE=ON
cmake --build build/issue-187 --parallel 3
ctest --test-dir build/issue-187 -L ble --output-on-failure --no-tests=error
```

Supply `-DCMAKE_PREFIX_PATH=/path/to/Qt` if needed, using your installed Qt path.
An offline build can use `-DFETCHCONTENT_SOURCE_DIR_GOOGLETEST=/path/to/cached/googletest`.
Save command output and actual results. Automated checks remain separate evidence.

Create `build/acceptance-187/desktop.ini` with:

```ini
[robot]
id = 1
[logging]
level = debug
file_enabled = true
file_path = build/acceptance-187/desktop.log
```

Launch from the repository directory so the relative log path resolves there:

```sh
build/issue-187/src/micro_robot_desktop.app/Contents/MacOS/micro_robot_desktop --ble --config build/acceptance-187/desktop.ini
```

Click **Connect robot** to request real Bluetooth permission and begin discovery.
Capture the permission dialog and grant access for the first positive test. If
permission was already granted, label the prompt NOT RUN for this pass; perform
the separate permission test below. Do not launch without `--ble`, which selects
simulation. Keep cameras stopped; camera acquisition is unnecessary here.

## 4. Positive lifecycle and cancellation

1. With `profile normal`, connect and retain desktop plus serial logs. Expect
   `Scanning -> Connecting -> DiscoveringServices -> DiscoveringCharacteristics
   -> ReadingIdentity -> EnablingNotifications -> Ready`. Domain state stays
   Connecting until Ready, then Connected. Record the service-filtered peripheral
   name `MR-1`, available native identifier/address metadata, requested ID and
   authoritative Robot Info ID. Where discovery metadata is not printed by the
   production UI/log, retain an inspector capture; label any unobservable desktop
   metadata check explicitly rather than claiming it was seen.
2. Verify serial `Robot Info read`, then `CCCD result=1`, then
   `CCCD telemetry=1 ready=1`, and desktop Ready/Connected afterward. Cross-check
   with a GATT capture when available. Desktop and firmware clocks have different
   origins: use event order within each log, not direct timestamp subtraction.
3. Click **Disconnect robot**, require terminal Disconnected, and reconnect.
   Complete **three full connect/disconnect cycles**. Retain identity, readiness
   and terminal state evidence for every cycle. Recovery is an explicit click;
   do not expect automatic reconnect.
4. Click Connect and then Disconnect while the desktop reports Scanning. Expect
   bounded cancellation and terminal Disconnected (or a logged Fault on backend
   failure), with no late Ready/Connected. Observe for at least 15 seconds.
5. Select `profile slow-info` by serial command. Connect; when the log reports
   ReadingIdentity or serial reports the read, click Disconnect during the
   three-second read delay. Record time to terminal state; observe another
   15 seconds and require that late callbacks never restore Connected. Repeat
   if you missed the window. The fixture deliberately stalls its BLE callback;
   this makes cancellation reproducible and is not production firmware behavior.
6. Restore `profile normal` and explicitly reconnect successfully.

Defaults: scan/permission deadline 10 s, each setup phase deadline 10 s,
disconnect deadline 3 s. Record elapsed times and the phase being tested; allow
small event-loop/scheduler overhead, document significant excess as a defect.
These are phase bounds, not a promise that total connection setup takes 10 s.

## 5. Failure matrix and explicit recovery

Select one profile at a time, wait for its reboot, check `status`, then click
Connect. Record the exact desktop error and terminal state. After **each** case,
restore normal conditions/profile and explicitly connect successfully. Save each
attempt's log interval and retry outcome separately.

| Case | Action | Required observation |
| --- | --- | --- |
| Robot Info identity mismatch | `profile wrong-id` | Name stays MR-1, record ID becomes 0 (ID XOR 1); identity mismatch, no Connected, terminal Fault |
| Unsupported version | `profile wrong-version` | Robot Info min/max become 2; UnsupportedVersion, no Connected, terminal Fault |
| Missing characteristic | `profile missing-command` | Command absent; GATT validation failure and Fault |
| Wrong properties | `profile bad-properties` | Command is Read only; required Write/Write Without Response rejected, Fault |
| Notify/CCCD unavailable | `profile no-telemetry-notify` | Telemetry Read only, no automatic CCCD; property validation failure and Fault; this does not isolate descriptor validation |
| Wrong service filter | `profile wrong-service` | Advertises reserved service instead; excluded from robot discovery, requested ID not found, Fault |
| Unreachable peripheral | Normal profile, remove all board power before Connect | Empty/no matching scan terminates with error and Fault; no infinite scan |
| Bluetooth off | Switch Bluetooth off in macOS settings before Connect | Clear unavailable/backend error and Fault; turn it back on and retry |
| Permission denied | Deny the actual prompt or revoke access in macOS Privacy & Security > Bluetooth | Clear permission/backend failure and Fault; restore access and retry (relaunch if macOS requires it) |
| Duplicate advertised ID | Two independent boards, both `id 1` and `profile normal`, both disconnected/advertising | Ambiguous/duplicate identity, no arbitrary device selected, Fault |
| Notify present but CCCD missing | Use a separately controlled malformed GATT server as described below | Missing-descriptor validation error, no Connected, Fault |

Permission prompting is application/OS identity dependent. For repeatable first
prompt testing, quit the app and, if appropriate on your macOS version, reset
Bluetooth permission for the bundle identifier in the built app's `Info.plist`
using macOS `tccutil`, then relaunch that same bundle. Reset only that app's
permission, not the entire system database. If policy prevents a reset or no real
prompt appears, record NOT RUN and the reason. An already-authorized launch does
not prove permission-prompt acceptance.

For duplicate IDs, one board cannot provide two independent native identities.
Keep both peripherals advertising for the entire discovery interval and confirm
both with an inspector if needed. With one CodeCell record NOT RUN and retain
the second-board requirement; sequentially changing one board's name is not this
test.

**Isolated missing CCCD remains a separate fixture requirement.** NimBLE creates
CCCDs automatically for Notify; this fixture cannot keep Notify while removing
only 0x2902. To run this case, provision a controllable malformed GATT server
that permits an explicit attribute table: copy the normal service, all five
characteristics/properties and Robot Info above, omit only Telemetry's CCCD,
and verify the malformed layout with an inspector before connecting the desktop.
Then repeat omitting only Command Result's CCCD. Restore both descriptors for
each recovery retry. If the server stack automatically inserts them, it cannot
execute this test: mark NOT RUN, identify the limitation in #187 and leave its
descriptor acceptance unchecked. Fake-radio tests do not fill this evidence gap.

Changing GATT layouts under one BLE address can expose OS service caching.
Disconnect all centrals and restart the desktop between profiles. If a capture
still shows the old table, power-cycle the board and refresh the OS/inspector
cache using supported platform tools. Record the actual observed layout before
attributing a failure to desktop validation. Never claim a profile ran merely
because its serial setting changed.

## 6. Link loss and shutdown

1. Restore normal, connect, then remove **all** CodeCell power, including any
   battery. Removing USB alone is insufficient if a battery powers the board.
   Expect unexpected link loss, error and terminal Fault. Record detection time;
   do not substitute a serial reboot for the complete power-loss test.
2. Power it on, confirm it advertises, click Connect and require a fresh successful
   session with correct ID and both CCCDs. There is no automatic recovery policy
   in this acceptance test.
3. Quit while Scanning; repeat while `slow-info` is in ReadingIdentity. Require
   bounded window/process exit with no hang/crash. Record wall time, process
   disappearance and final logs. Use five seconds as an observation threshold;
   if exceeded, retain logs and report a teardown defect for investigation.
4. Restore normal, connect, then quit while Connected. Confirm bounded process
   exit and firmware disconnect/re-advertising. Relaunch and connect successfully.
   Closing the monitor is not the same as closing the desktop application.

## 7. Results to attach to #187

Copy this record into your evidence notes and fill it from actual execution.
Use PASS / FAIL / NOT RUN, never an unchecked assumption. Attach the completed
record, desktop/serial logs, GATT captures and any defect links to #187. Do not
close #187 merely because these preparation files were merged.

```text
Date/operator:
Desktop commit / dirty changes:
Firmware commit / binary SHA-256 / PlatformIO packages:
CodeCell revision / flash size / power / number of boards:
macOS / Qt / Bluetooth adapter:
Advertised name / requested ID / native identifier / optional address:
Robot Info raw bytes / decoded limits:
GATT properties / result CCCD / telemetry CCCD evidence:

Test                              Result     Evidence / elapsed / defect
Actual permission prompt          NOT RUN
Service-filtered discovery        NOT RUN
Native metadata                   NOT RUN
Identity and readiness ordering   NOT RUN
Connect/disconnect cycle 1        NOT RUN
Connect/disconnect cycle 2        NOT RUN
Connect/disconnect cycle 3        NOT RUN
Cancel scanning / no late ready   NOT RUN
Cancel GATT / no late ready       NOT RUN
Wrong identity + explicit retry   NOT RUN
Wrong version + explicit retry    NOT RUN
Missing char + explicit retry     NOT RUN
Wrong properties + retry          NOT RUN
Notify absent + retry             NOT RUN
Wrong service + retry             NOT RUN
Unreachable + retry               NOT RUN
Bluetooth off + retry             NOT RUN
Permission denied + retry         NOT RUN
Duplicate ID + retry              NOT RUN
Telemetry CCCD absent + retry     NOT RUN
Result CCCD absent + retry        NOT RUN
Full power loss / Fault / recovery NOT RUN
Quit scanning                     NOT RUN
Quit during GATT setup            NOT RUN
Quit connected / reconnect        NOT RUN
Linux BlueZ / X11 / Wayland        NOT RUN — record environment/follow-up
Windows                           NOT RUN — record environment/follow-up

Failures: exact steps, expected/actual state, log interval, reproducibility,
linked fix issue/PR, and retest result.
Unexecuted cases: reason, required equipment/environment, follow-up owner/link.
```

On Linux, use the same firmware/config and the Linux executable from your build
(`build/issue-187/src/micro_robot_desktop`), with BlueZ running and the required
permissions; follow [Linux setup](../development/linux-setup.md). Record actual
adapter/session and GUI results separately. Windows remains a target platform;
no successful Windows build/runtime validation is implied by this guide.

## Preparation validation

Executed on macOS during preparation (not physical acceptance):

- `pio run -d firmware/ble-lifecycle` — passed using the pinned packages;
  binary generated, 23,856 bytes RAM and 731,704 bytes application flash reported.
- Desktop configured with the section 3 CMake command plus
  `-DCMAKE_PREFIX_PATH=/opt/homebrew/opt/qt` and the local cached GoogleTest source;
  `cmake --build build/issue-187 --parallel 3` — passed.
- `ctest --test-dir build/issue-187 -L ble --output-on-failure --no-tests=error`
  — 44/44 passed, including three new hardware-free checks of fixture provisioning
  and actual firmware data against the production desktop decoder/validator.
- Local Markdown file targets and `git diff --check` — passed.

Not executed: upload, physical advertising/GATT/subscriptions, actual permissions,
radio cancellation/loss/recovery/shutdown, Linux and Windows. No connected CodeCell
serial port was available during preparation. These remain NOT RUN in #187.
