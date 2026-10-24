# Communication module

[RobotManager](../robot-manager.md) coordinates injected transport sessions by
RobotId and supplies the application boundary for connection intent, commands,
Stop requests, telemetry and errors. Its state and tests need no Qt or hardware.
The manager stores communication snapshots; the authoritative WorldModel retains
poses and workspace state.

- [Transport contract](../transport.md): domain units, commands, weak events and
  asynchronous lifetime rules.
- [Robot communication](../robot-communication.md): discovery abstraction and
  platform-boundary enforcement.
- [BLE lifecycle](../ble-connection.md) and [messages](../ble-messages.md): physical
  adapter, identity validation, protocol and fake-radio tests.
- [Simulation](../simulation.md): deterministic alternative behind the same API.

Physical multi-link interoperability remains a hardware check. Manual motion UI,
domain E-stop, reconnection policy and multi-robot coordination remain separate.
