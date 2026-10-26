# BLE discovery and connection lifecycle

Issue #65 adds the Qt Bluetooth Central adapter behind
[`IRobotTransport`](transport.md). It discovers ESP32-C6 peripherals, validates
their GATT service and identity, enables notifications and publishes domain
connection/error events. Radio-specific objects stay inside `QtBleBackend`.

At the maintainer's request, #65 tracks software implementation and automated
acceptance. Physical ESP32-C6 acceptance is tracked separately in
[#187](https://github.com/Krzys2002/Mini_Robot/issues/187).

Issue #66 adds [command writes and telemetry/result parsing](ble-messages.md),
including an adapter E-stop primitive. RobotManager, reconnect policy and
high-level safety/operator controls remain their assigned issues. No firmware
is added. This guide describes the connection layer; the message guide records
its additional behavior and outstanding physical command-delivery acceptance.

## Build and use

The feature is opt-in so existing desktop and Qt/OpenCV-free builds do not gain
a mandatory Bluetooth dependency. Install Qt Bluetooth from the same Qt
distribution as Qt Core. On Ubuntu the additional development package is
`qt6-connectivity-dev`; real Linux BLE also needs an available BlueZ adapter,
service and suitable system permissions. Linux hardware and Windows remain
unverified by this macOS implementation work.

```sh
cmake -S . -B build/issue-65 -G Ninja -DCMAKE_BUILD_TYPE=Debug \
  -DMICRO_ROBOT_BUILD_BLE=ON
cmake --build build/issue-65 --parallel 3
ctest --test-dir build/issue-65 -L ble --output-on-failure --no-tests=error
```

Supply `CMAKE_PREFIX_PATH` for a nonstandard Qt installation. Offline test builds
can set `FETCHCONTENT_SOURCE_DIR_GOOGLETEST` to a cached GoogleTest checkout.
`MICRO_ROBOT_BUILD_BLE=ON` also works with desktop/replay/calibration OFF, for
headless Qt Core lifecycle tests without Widgets, OpenCV or radio hardware.

Launch the built desktop executable with **`--ble`** and the existing
`--config <file>` if needed. On macOS this is
`build/issue-65/src/micro_robot_desktop.app/Contents/MacOS/micro_robot_desktop --ble`.
Set `[robot] id` in the configuration to the intended robot's decimal ID. Click
**Connect robot** to scan and establish the connection; **Disconnect robot**
also cancels setup. Phase transitions and errors are logged with robot identity.
Simulation remains the default when `--ble` is absent. In BLE mode, composition
does not advance simulation or inject demo robot poses/telemetry. With BLE built
OFF, requesting `--ble` produces an error and exits with code 2.

Qt 6.5+ uses the Qt Bluetooth Access permission API. Permission is requested only
after discovery is explicitly requested, never by constructing the adapter.
The macOS app includes `NSBluetoothAlwaysUsageDescription` and Qt's Darwin
Bluetooth permission plugin. Packaging tests inspect the production bundle and
plugin without triggering a prompt. Older supported Qt versions rely on their
platform backend's permission handling.

## Discovery and identity

`startDiscovery()` and `cancelDiscovery()` are asynchronous diagnostic operations
on `BleTransport`. `discovering()`, `devices()` and `discoveryChanged()` expose
the scan inventory. These APIs are for composition/discovery presentation;
application connection consumers continue to use `IRobotTransport`.

- Filter native results by Low Energy capability and the Robot Control service
  UUID `4d520001-1f2a-4032-8415-63a5fef8a0dc`.
- Each `BleDevice` contains an opaque scan-local ID, display name, optional
  address and optional advertised RobotId hint. macOS peripheral UUIDs are
  supported without fabricating a Bluetooth address.
- `MR-<uint32>` names are selection hints. `connect(RobotId)` performs a fresh
  scan, waits for its completion, then requires exactly one matching hint.
  Duplicate advertisers fail with an ambiguity error. Anonymous service-matching
  devices are listed but cannot be selected by `connect(RobotId)` in this version.
- Repeated updates replace one inventory entry by opaque ID. Both native and
  adapter inventories use the same configurable device-count bound.
- Robot Info is the authoritative identity. A mismatch fails the attempt and
  tears down the link, even if the advertising name matched.

Multiple robots can share a scan; each has independent connection tokens,
timeouts and domain state. A later scan refreshes the inventory without closing
existing links. Discovery cancellation disconnects robots awaiting that scan;
it does not cancel links already undergoing GATT setup.

## States, ownership and failure

Normal setup is:

```text
Scanning -> Connecting -> DiscoveringServices -> DiscoveringCharacteristics
         -> ReadingIdentity -> EnablingNotifications -> Ready
```

The domain remains `Connecting` through all setup phases and becomes `Connected`
only at Ready. `phase()`/`phaseChanged()` expose detailed diagnostics. The backend
requires all five characteristics, their documented properties, and CCCDs on
Command Result and Telemetry. It reads the exact 18-byte Robot Info record,
validates format/version range, unsigned robot ID and nonzero bounded limits,
then awaits CCCD write confirmations: **Command Result first, Telemetry second**.
With #66, latched adapter E-stop intent is reasserted and confirmed before
publishing Connected; see [message handling](ble-messages.md).

`BleTransportConfig` centralizes limits:

| Parameter | Default |
| --- | --- |
| Scan/permission deadline | 10 s |
| Each connection/GATT setup phase | 10 s |
| Disconnect deadline | 3 s |
| Scan inventory limit | 128 devices |

Empty scans, duplicate IDs, missing service/characteristic/property/descriptor,
identity mismatch, incompatible Robot Info, denied permission, disabled/unavailable
adapter, backend errors and timeouts end the attempt with a domain error and
`Fault`. Unknown Robot Info format/version/length reports `UnsupportedVersion`.
An unexpected link loss also faults. Fault releases native resources, clears the
token and allows an explicit new connection attempt; no automatic reconnect
occurs. Disconnect ends in `Disconnected`, or `Fault` on error/timeout.

All public calls and callbacks belong to the owning Qt event-loop thread.
Requests queue their work; native events are queued across the internal
`IBleBackend` boundary. No blocking wait, sleep or polling loop runs in the
adapter. Generation tokens discard events after scan cancellation, disconnect,
failure and reconnect. Domain event sinks remain weak and are locked per event;
queries reflect state before their callback. Callback requests are deferred.
Copy discovered values before retaining them across inventory refreshes or
passing them to another thread.

The composition root owns the adapter, and the injected backend has exclusive
ownership. Native link contexts own their Qt controller/service children through
QObject ownership. Stop active connections, keep the event loop alive until
terminal states and destroy on the owner thread outside callbacks. Destructor
teardown invalidates native callbacks and pending permission work without waiting.

## Tests and physical acceptance

Core tests validate advertised names, full unsigned identity range, the protocol's
Robot Info example, every truncated record, versions, limits, characteristic
properties and notification descriptors. Injected-radio tests exercise deferred
events, duplicate/missing IDs, bounded inventory, cancellation, late callbacks,
every setup timeout, disconnect timeout, independent robots, recovery, sink
replacement/expiry and destruction. The [message tests](ble-messages.md) cover
command dispatch, notification parsing and E-stop behavior. They do not
open Bluetooth hardware or request permission.

Physical acceptance remains incomplete. The
[2026-10-26 CodeCell run](../testing/ble-connection-187-results-2026-10-26.md)
records repeated connections, cancellation, fault/recovery and teardown, plus
the full-discovery fix required for real CCCDs on the tested macOS/Qt 6.12 setup.
Permission/adapter-off, duplicate-ID, isolated missing-CCCD and other-platform
checks remain NOT RUN. The following procedure remains the acceptance checklist:

The [CodeCell test procedure for #187](../testing/ble-connection-187.md) provides
build/flash commands, a connection-only [ESP32-C6 fixture](../../firmware/ble-lifecycle/README.md),
fault profiles and a results template. Preparation does not complete physical
acceptance; isolated missing-CCCD and two-board cases need the stated equipment.

1. Provision an ESP32-C6 with the documented advertising, GATT, Robot Info and
   CCCDs. Confirm its `MR-<id>` name and configuration ID match.
2. Launch the macOS app with `--ble`, grant Bluetooth access, click Connect and
   record each setup phase, matching identity and Connected transition.
3. Disconnect/reconnect at least three times. Cancel during scan and GATT setup;
   verify late results never restore Connected.
4. Try duplicate advertised IDs, wrong Robot Info identity/version, missing
   characteristics/descriptors, Bluetooth off, denied permission and an unreachable
   peripheral. Record error, terminal state and successful explicit retry.
5. Power off a connected peripheral. Verify Fault and recovery after power-on.
6. Close the window while connecting/connected; confirm bounded teardown and exit.
7. Record actual hardware, firmware, Qt/OS versions and logs. Repeat on Linux
   with BlueZ and later Windows; no platform success is inferred from fake tests.

The ESP32-C6 repeated discovery/connect acceptance criterion and unexecuted
hardware checks are tracked in [#187](https://github.com/Krzys2002/Mini_Robot/issues/187).
Record actual physical results and logs there; #65's deterministic acceptance
does not claim physical success. Physical command delivery/telemetry validation is tracked separately in
[#189](https://github.com/Krzys2002/Mini_Robot/issues/189), with the
[message procedure](ble-messages.md#verification-and-remaining-hardware-acceptance).

Qt references: [device discovery](https://doc.qt.io/qt-6/qbluetoothdevicediscoveryagent.html),
[Central controller](https://doc.qt.io/qt-6/qlowenergycontroller.html),
[GATT service](https://doc.qt.io/qt-6/qlowenergyservice.html).

## Verification record (2026-10-20)

On macOS with Apple Clang 21, Qt 6 and OpenCV 5.0.0:

- Desktop configure/build with `MICRO_ROBOT_BUILD_BLE=ON` in `build/issue-65`:
  passed, including the production app and native Qt Bluetooth adapter.
- `ctest --test-dir build/issue-65 --output-on-failure --no-tests=error`:
  **211/211 passed**; this includes 20 BLE-labelled tests/checks and both macOS
  camera/Bluetooth permission packaging checks.
- `ctest --test-dir build/issue-65 -L ble --output-on-failure --no-tests=error`:
  **20/20 passed** (five pure contract tests, fourteen fake/native-construction
  lifecycle tests and one packaging check).
- Qt/OpenCV-free configure/build in `build/issue-65-core`, with desktop/replay/
  calibration/BLE OFF: passed; unfiltered CTest **131/131 passed**.
- Default desktop configure/build with BLE OFF in `build/issue-65-default`:
  passed; `ctest --test-dir build/issue-65-default -L 'ui|packaging'
  --output-on-failure --no-tests=error`: **46/46 passed**.
- Offscreen production `--help` lists `--ble`: passed, without starting discovery.
- `git diff --check` and local documentation-link checks: passed.
- Not run: physical ESP32-C6 discovery/connect, actual permission prompts,
  real GATT/CCCD transactions, radio-loss/cancellation/shutdown on hardware,
  Linux BlueZ/X11/Wayland and Windows runtime. Physical acceptance remains open
  in [#187](https://github.com/Krzys2002/Mini_Robot/issues/187).
