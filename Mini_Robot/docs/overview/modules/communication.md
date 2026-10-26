# Robot communication

Milestone scope: M1 / M3. [Module index](README.md).

## Capability and boundaries

IRobotTransport and IRobotDiscovery define platform-neutral asynchronous contracts.
Simulation and a test-only radio backend exercise domain behavior. Version 1
commands/telemetry and BLE GATT are specified.

The opt-in Qt BLE Central adapter implements discovery, identity/GATT readiness,
CCCD setup, cancellation, timeouts and weak subscriptions behind the same domain
boundary. Its version 1 codec handles command writes, telemetry/results, sequence
wrap and bounded dispatch with freshness checks at native handoff. Adapter E-stop
retry/reassertion is a transport primitive, not a domain safety latch or operator
UI. Composition selects BLE with `--ble`; simulation remains the default. Radio
objects and endpoint identity verification stay private.

[RobotManager](../robot-manager.md) coordinates injected sessions by RobotId and
routes connection intent, commands, Stop requests, telemetry and errors. This
Qt-free application service owns the transport event subscription and detached
communication snapshots; WorldModel retains pose/workspace authority. Sessions
are independent and shutdown drains all managed robots. Shutdown is teardown,
not an implicit Stop-delivery protocol; firmware watchdog/stop-on-disconnect
remains required. Operator motion/safety policy remains separate.

## Behavior guides

- [RobotManager sessions and application ownership](../robot-manager.md)
- [Transport contract](../transport.md)
- [Discovery/platform preparation](../robot-communication.md)
- [Wire protocol/GATT](../../protocol/README.md)
- [BLE discovery and connection lifecycle](../ble-connection.md)
- [Command/telemetry codec and dispatch](../ble-messages.md)

## Verification and remaining acceptance

Domain, mock discovery and shared contract tests need neither Qt nor radio
hardware. Opt-in Qt Core fake-radio tests exercise lifecycle, identity, command
freshness, malformed notifications, result correlation and recovery. The guides
own test selection, BlueZ requirements and physical acceptance (#187/#189).
Fake-radio results do not prove over-air delivery, watchdog or physical E-stop.

Manager tests cover independent routing, stale/invalid telemetry, removal,
cancellation, callback reentrancy, retained-sink teardown and two fake BLE links.
Physical RobotManager/multi-link acceptance remains [#209](https://github.com/Krzys2002/Mini_Robot/issues/209),
with all physical checks NOT RUN. Manual motion UI, domain E-stop, reconnect
policy and multi-robot coordination remain separate issues.
