# Cross-platform robot communication

Source: [discovery interface](../../src/transport/IRobotDiscovery.hpp),
[transport interface](../../src/transport/IRobotTransport.hpp),
[test radio backend](../../tests/transport/MockRadioLink.hpp),
[discovery tests](../../tests/transport/RobotDiscoveryTest.cpp), and
[transport contract tests](../../tests/transport/TransportContractTest.cpp).

Issue #129 (LNX-05) prepares the desktop-to-ESP32-C6 communication architecture
for Linux and macOS without a hardware BLE implementation. It defines a
platform-neutral robot discovery boundary, keeps
[IRobotTransport](transport.md) unchanged for its callers, and records the Qt Bluetooth platform review below. The opt-in BLE adapter is
documented in the [connection guide](ble-connection.md) and
[message guide](ble-messages.md), implemented by
[#65](https://github.com/Krzys2002/Mini_Robot/issues/65) and
[#66](https://github.com/Krzys2002/Mini_Robot/issues/66). Zigbee
([#107](https://github.com/Krzys2002/Mini_Robot/issues/107)) is a separate
future transport behind the same interfaces. Linux packages, services, and
permissions are listed in the [Ubuntu setup guide](../development/linux-setup.md#bluetooth-and-ble).

## Robot discovery

[IRobotDiscovery](../../src/transport/IRobotDiscovery.hpp) follows the project's
[device-discovery pattern](platform-boundaries.md#platform-neutral-device-discovery).
It is a header-only part of `micro_robot_transport_api`, with no Qt or radio types.

| Element | Contract |
| --- | --- |
| `DiscoveredRobot` | `RobotId`, advertised display name, desktop receipt time (`lastSeenAt`, monotonic), and an `ambiguous` flag. It has no radio address, peripheral identifier, or UUID. |
| `startScan()` / `stopScan()` | Prompt, idempotent requests. Stop cancels a pending start. A backend may end a bounded scan itself by publishing `Idle`. |
| `scanState()` | `Idle` or `Scanning`. The query already reflects the new state when its event runs. |
| `robots()` | Latest snapshot. It is kept after a scan ends. The first snapshot of a new scan replaces it. |
| `onRobotsChanged` | Complete snapshot, ordered by `RobotId`, one entry per ID. An empty list is a valid result. |
| `onDiscoveryError` | `Unavailable` (no usable adapter or service), `PermissionDenied`, or `BackendFailure`. A failure that ends a scan also publishes `Idle`. A refused start reports only the error. |

Discovery never connects. `IRobotTransport::connect(RobotId)` connects, and the
backend resolves the ID through its private mapping from the most recent scan.
A backend that locates robots this way fails `connect` with `Unavailable` and
`Fault` when the ID was not discovered or is ambiguous. It never picks one of
several peers that claim the same ID. Telemetry from the radio is published under
the resolved `RobotId`.

The advertised ID is a hint only. Under the [BLE GATT contract](../protocol/ble-gatt.md#advertising-and-discovery),
the scan filters by the Robot Control service UUID and reads the ID from the
`MR-<robotId>` name. After connecting, the Robot Info record is authoritative, and
a mismatch ends the session with an error. A Zigbee gateway would fill the same
fields from its own device table.

Threading follows the transport rules. Calls and callbacks run on the backend's
owning thread, event sinks are weak subscriptions, and requests never dispatch
events inline. A backend that implements both interfaces, keeps two independent event sinks. The current `BleTransport` implements
`IRobotTransport` and exposes its scan inventory through adapter-local Qt signals;
it does not yet implement `IRobotDiscovery`. The shared discovery contract and
its test backend remain available independently.

## Qt 6 Bluetooth platform review

Qt Bluetooth is the implementation for #65/#66, because one Qt source
tree covers the BLE central role on macOS, Linux, and Windows. The review was
based on the Qt 6 documentation, the [platform support table](https://doc.qt.io/qt-6/qtbluetooth-index.html),
[QBluetoothDeviceInfo](https://doc.qt.io/qt-6/qbluetoothdeviceinfo.html), and
[QBluetoothPermission](https://doc.qt.io/qt-6/qbluetoothpermission.html). On
Linux it was also checked against the installed BlueZ stack, as described in
[verification](#verification).

| Topic | Linux | macOS | Windows |
| --- | --- | --- | --- |
| Qt backend for BLE central | BlueZ 5 through the `bluetoothd` D-Bus API | CoreBluetooth | Qt's Windows backend (not reviewed in detail) |
| Peer identity Qt exposes | `QBluetoothDeviceInfo::address()` (Bluetooth device address) | `address()` is invalid. `deviceUuid()` is an identifier assigned by the host. | `address()` |
| Permission | No Qt permission. Access goes through the system D-Bus policy for `org.bluez`; root and `CAP_NET_ADMIN` are not needed for the central role. | `NSBluetoothAlwaysUsageDescription` in the bundle `Info.plist`, and a `QBluetoothPermission` request. Since Qt 6.6, Qt no longer checks it for the application. | Not reviewed; Windows validation is M8 |
| Runtime prerequisites | `bluetoothd` running, adapter present, powered, and not rfkill-blocked | Bluetooth enabled, permission granted | Not reviewed |
| Build package | `qt6-connectivity-dev` (Qt6::Bluetooth) | Qt Connectivity module from the same Qt installation | Same |
| Link parameters | Central may accept the robot's request | Managed by macOS | Not reviewed |

Consequences for the architecture:

- **Peer identity is platform-specific.** Linux reports a device address, and
  macOS reports a per-host UUID. Neither crosses `IRobotDiscovery` or
  `IRobotTransport`. Inside the adapter, a peer key is the address where it is
  valid and `deviceUuid()` otherwise. The key is used only for the current scan
  and session, and is never persisted as robot identity.
- **Platform differences stay inside the adapter target and the build.** The
  macOS usage description belongs in [Info.plist.in](../../src/app/Info.plist.in)
  for BLE builds, next to the existing camera key. Permission
  requests are issued by the composition root or the adapter. A denial maps to
  `PermissionDenied` (discovery) or `Unavailable` (connect). No `Q_OS_*` branch
  is needed in domain or application code.
- **Qt Bluetooth is asynchronous and signal-based.** The adapter can live on the
  GUI/owning thread without blocking it. Its slots translate Qt signals into the
  deferred interface events and do not call business or UI logic.
- **CI has no Bluetooth hardware.** Automated tests use `SimulationTransport` and
  the test radio backend. A BLE adapter test must be opt-in, like the
  [V4L2 hardware tests](camera-backends.md#verification).

### Adapter placement

The opt-in `micro_robot_ble_qt` platform adapter implements
`IRobotTransport`, links `Qt6::Bluetooth` privately, and is not registered with
`micro_robot_platform_boundary()`. Only the composition root
([main.cpp](../../src/app/main.cpp)) selects it instead of `SimulationTransport`.
The Qt-free `micro_robot_ble_contract` target owns protocol encoding/decoding.
The Connectivity module is required only with `MICRO_ROBOT_BUILD_BLE=ON`;
see the [build guide](../development/build.md). Integration of the adapter's
scan inventory with `IRobotDiscovery` remains separate work.

## Isolation from robot control

| Rule | Enforcement |
| --- | --- |
| `micro_robot_control` uses only domain values (`VelocityCommand`) from `TransportTypes.hpp`. It does not include `IRobotTransport.hpp`, discovery, or a backend. | The [source map](source-map.md#target-boundaries) lists the target's dependencies. The target links only `micro_robot_common` and `micro_robot_transport_api`. |
| Control and all other core targets cannot link Qt Bluetooth, BlueZ, D-Bus, or a concrete transport. | Registered targets fail configuration on any undeclared imported library or backend target ([enforcement](platform-boundaries.md#enforcement)). |
| Core sources cannot include CoreBluetooth, IOBluetooth, Qt Bluetooth/`QLowEnergy*`, BlueZ (`bluetooth/`), D-Bus (`dbus/`, `gio/`, `systemd/`, `QtDBus`), or WinRT headers, or test Apple/Linux/Windows macros. | `PlatformBoundary.<target>` scans. The [checker self-test](../../tests/boundaries/VerifyBoundaryCheck.cmake) fails when one of these patterns is not reported. |
| A future BLE adapter's headers are forbidden in core sources. | Headers of every unregistered first-party target become forbidden includes automatically. |

The `RobotManager` application service ([#67](https://github.com/Krzys2002/Mini_Robot/issues/67))
holds the injected transport and routes commands by `RobotId`; its
[behavior guide](robot-manager.md) defines per-robot sessions, telemetry, ownership
and shutdown. Control code
produces semantic commands and never addresses a radio peer.

## Mock transport tests

`micro_robot_transport_mock_tests` (label `core`) needs no Qt, Bluetooth adapter,
or robot.

- [MockRadioLink](../../tests/transport/MockRadioLink.hpp) is a test-only stand-in
  for a radio backend. One object implements both interfaces and maps `RobotId`
  to private peer address strings. Tests control advertisements, adapter
  availability, link loss, and incoming notifications. `deliver(now)` completes
  queued requests, standing in for asynchronous radio completion.
- `RobotDiscoveryTest` covers deferred scans, ordered complete snapshots,
  idempotent start/stop, snapshot replacement on a new scan, duplicate-ID
  ambiguity, unavailable adapters, callbacks that stop a scan, weak sinks,
  connect resolution through discovery, address privacy, link loss as `Fault`,
  and recovery.
- `TransportContractTest` is a typed suite that runs the same `IRobotTransport`
  checks against `SimulationTransport` and `MockRadioLink`: unknown IDs, deferred
  and idempotent connect, cancellation, disconnect, `NotConnected`,
  `InvalidCommand`, and `UnsupportedVersion` rejections, telemetry identity, and
  expired or replaced sinks. A hardware adapter must satisfy the same checks.

```sh
ctest --test-dir build/linux-gcc -R '^(RobotDiscoveryTest|TransportContractTest)' --output-on-failure --no-tests=error
```

## Verification

On Ubuntu 26.04.1 LTS (GCC 15.2, Clang 21.1, Qt 6.10.2), the `linux-gcc` and
`linux-clang` presets and a configuration without Qt and OpenCV were built, and
their full CTest suites passed. See the
[setup guide](../development/linux-setup.md#verification-status) for the counts.
The same host has BlueZ 5.85. `bluetoothd` was active, and adapter `hci0` was
powered and unblocked, with the central and peripheral roles. Its system D-Bus
policy allows any local user to call `org.bluez`. No BLE scan or connection was
run, because no adapter code exists yet. Qt Bluetooth (`qt6-connectivity-dev`)
was not installed or compiled against.

macOS was not built or tested for this issue. The interface and tests use only
standard C++ and are compiled by the existing macOS CI workflow. The macOS
permission and identity behavior above comes from Qt's documentation and has
not been observed on hardware.
