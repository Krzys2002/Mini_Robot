# Robot protocol

Wire-level contract between the desktop and ESP32-C6 robots. Protocol version 1
is specified here. The desktop [BLE lifecycle](../overview/ble-connection.md)
implements service/identity/CCCD readiness; [message handling](../overview/ble-messages.md)
implements command writes and validated telemetry/results. The robot-side
[codec and command validation](../overview/firmware-protocol.md) (#161) is portable
firmware code served by the [C6 BLE peripheral](../overview/firmware-ble.md) (#163); powered
and desktop command-delivery acceptance remains outstanding (#166, #238).

- [Robot message protocol](robot-protocol.md): identity, frame header, message
  types, units and scaling, byte order, sequence numbers, acknowledgements, result
  codes, rejection rules, freshness and watchdog, emergency stop, domain mapping,
  example frames, JSON bring-up form, and version evolution.
- [Shared v1 byte vectors](fixtures/v1-frames.txt): the [example frames](robot-protocol.md#example-frames)
  in a machine-readable form, plus derived sentinel and extrema frames. Both endpoint test
  suites read this file; version 1 lines are never edited.
- [BLE GATT contract](ble-gatt.md): service and characteristic UUIDs, advertising,
  session establishment, disconnect and E-stop safety, link parameters, and
  implementation checklists.
- [Restricted-lab security profile](security-profile.md): threat matrix,
  operating restrictions, enforceable firmware rules, controller lifecycle,
  provisioning/revocation policy, residual risks, attack tests and the stack
  capability review for a future secure profile.
- [Firmware timing, resource and stopping budgets](timing-budgets.md) (#156,
  proposed): the robot-side timing path, the budget table with owners and
  pass/fail methods, overload policy, desktop queue requirements,
  instrumentation points and stress conditions. It chooses advertised v1 values
  and does not change the protocol.

The domain-facing [transport contract](../overview/transport.md) is independent of
this format. Only protocol adapters behind `IRobotTransport` and robot firmware
use it.
