# Robot protocol

Wire-level contract between the desktop and ESP32-C6 robots. Protocol version 1
is specified here. The desktop [BLE lifecycle](../overview/ble-connection.md)
implements service/identity/CCCD readiness; [message handling](../overview/ble-messages.md)
implements command writes and validated telemetry/results. Firmware and physical
command-delivery acceptance remain outstanding.

- [Robot message protocol](robot-protocol.md): identity, frame header, message
  types, units and scaling, byte order, sequence numbers, acknowledgements, result
  codes, rejection rules, freshness and watchdog, emergency stop, domain mapping,
  example frames, JSON bring-up form, and version evolution.
- [BLE GATT contract](ble-gatt.md): service and characteristic UUIDs, advertising,
  session establishment, disconnect and E-stop safety, link parameters, and
  implementation checklists.
- [Restricted-lab security profile](security-profile.md): threat matrix,
  operating restrictions, enforceable firmware rules, controller lifecycle,
  provisioning/revocation policy, residual risks, attack tests and the stack
  capability review for a future secure profile.

The domain-facing [transport contract](../overview/transport.md) is independent of
this format. Only protocol adapters behind `IRobotTransport` and robot firmware
use it.
