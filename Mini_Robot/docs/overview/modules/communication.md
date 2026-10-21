# Robot communication

Milestone scope: M1 / M3. [Module index](README.md).

## Capability and boundaries

IRobotTransport and IRobotDiscovery define platform-neutral asynchronous contracts.
Simulation and a test-only radio backend exercise domain behavior. Version 1
commands/telemetry and BLE GATT are specified.

This source snapshot contains contracts, simulation and mock tests; real BLE
integration is separate scope (#65/#66). RobotManager, operator motion/safety controls and
physical acceptance remain issue-scoped. Radio identities/mechanics stay in adapters.

## Behavior guides

- [Transport contract](../transport.md)
- [Discovery/platform preparation](../robot-communication.md)
- [Wire protocol/GATT](../../protocol/README.md)

## Verification and remaining acceptance

Domain, mock discovery and shared contract tests need neither Qt nor radio
hardware. The communication guide owns test details and BlueZ requirements.
Fake-radio results do not prove over-air delivery, watchdog or physical E-stop.
