# Restricted-lab security spike: CodeCell C6 and Linux BlueZ (2026-11-14)

Status: **bounded non-motor spike for
[#157](https://github.com/Krzys2002/Mini_Robot/issues/157)**. It checks the
stack facts that the [restricted-lab security profile](../protocol/security-profile.md)
relies on. It used the connection-only
[`ble-lifecycle` fixture](../../firmware/ble-lifecycle/README.md), not production
firmware, so it is not evidence for any F-rule or AT case in that profile, nor
for motion, watchdog, E-stop or #165 acceptance.

## Setup

| Item | Value |
| --- | --- |
| Repository revision | `3a3c8700ba28c34a82c20111d8c3ccdd2b692429` (origin/master at branch creation) |
| Firmware | `ble-lifecycle` fixture 0.1.0, built locally with `pio run -d Mini_Robot/firmware/ble-lifecycle -e codecell-c6` (PlatformIO Core 6.1.19, pinned pioarduino 55.03.37, NimBLE-Arduino 2.3.6) |
| Image SHA-256 | `de79c7c4e7b48ffad2171b60cc0d8d346f9ea821b1bd5009f69d1a1bbfbc1173` (`firmware.bin`); the uploader verified the hash of each written region |
| Board | CodeCell C6, USB `303A:1001` (Espressif USB JTAG/serial), flash auto-detected as 8 MB; physical revision not inspected |
| Power and actuators | USB data cable only. The operator disconnected the battery and unplugged the motors before flashing. The fixture never initializes motor, GPIO or CodeCell APIs. |
| Host | Ubuntu 26.04.1 LTS, kernel 7.0.0-43-generic, BlueZ 5.85 (`bluetoothctl`), Intel AX200 adapter |
| Prior board image | Unknown: it did not answer the fixture `status` command and no `MR-*` advertiser was found; it was not read back before flashing |
| Not available | HCI capture (`btmon` requires root here), macOS, a second independent central |

The robot's public address and the host adapter address appear in the local
logs. They are not secrets, but they are omitted here.

## Results

| Check | Result | Observation |
| --- | --- | --- |
| Unpaired discovery and connect from BlueZ | PASS | `MR-1` advertised the Robot Control service. `bluetoothctl connect` succeeded with `Paired: no`, `Bonded: no`, and no pairing prompt appeared. |
| Unauthenticated GATT use | PASS (documents TM-01/R1) | Robot Info read returned `01 01 01 01 00 00 00 00 01 00 2c 01 70 17 f4 01 64 00` (same as the #187 record). Enabling Command Result and then Telemetry notifications produced fixture `ready=1` and Telemetry notifications. A 4-byte Command write (`01 01 00 00`, a Stop frame) reached the firmware (`WRITE IGNORED … bytes=4`), with no security required. |
| Advertising while connected | PASS, same-adapter limitation | During an 8 s BlueZ LE scan with the session open, the robot's address produced no advertising/RSSI events while other nearby advertisers did. Once idle, the same scan reported it within the first seconds. This is consistent with F1 but is not a second-central test (AT-03). |
| Pairing request from the central | Observed | `bluetoothctl pair` with a NoInputNoOutput agent completed as Just Works: `Pairing successful`, `Paired: yes`, `Bonded: no`, `Trusted: no`. No passkey or confirmation was requested. |
| Pairing persistence | Observed | After disconnect, BlueZ reported `Paired: no`, `Bonded: no`: no long-term key was kept by the host. The fixture logged no security events (it has no security callbacks). |
| Disconnect handling | PASS | Fixture: both CCCDs disabled, `link disconnected reason=531; advertising resumes`. |
| USB serial provisioning | PASS (documents TM-12/R7) | `id 157` saved to NVS and rebooted; `status` and the scan response showed `id=157` / `MR-157`. `id 1` restored the original value, confirmed after reboot. The serial console requires no authentication. |
| Cleanup | Done | BlueZ device record removed. The board is left running the fixture with ID 1 and profile `normal`. |

## Interpretation for the profile

- **Pairing is possible but meaningless.** The pinned NimBLE-Arduino 2.3.6
  initializes NoInputNoOutput, no bonding, no MITM and no Secure Connections
  (`NimBLEDevice::init()` in
  [NimBLEDevice.cpp](https://github.com/h2zero/NimBLE-Arduino/blob/2.3.6/src/NimBLEDevice.cpp)).
  Any central can therefore negotiate Just Works link encryption without
  authentication. Because Secure Connections is disabled, this is LE Legacy
  pairing. That conclusion comes from the source configuration. No HCI capture
  confirmed it. This is why profile rule F6 forbids any behavior that depends on
  encryption or pairing state.
- **Connection-count default.** NimBLE-Arduino 2.3.6 defaults to three
  connections (`CONFIG_BT_NIMBLE_MAX_CONNECTIONS` in
  [nimconfig.h](https://github.com/h2zero/NimBLE-Arduino/blob/2.3.6/src/nimconfig.h)).
  Session exclusivity currently relies on advertising stopping. Profile rule F1
  therefore requires an explicit limit of one connection in production.
- **Provisioning has no access control beyond USB custody**, as profile O8 and
  R7 assume.
- No result here shows MITM-resistant pairing. As the profile states, the
  CodeCell has no documented input or display for it.

## Not run

| Check | Reason |
| --- | --- |
| HCI-level confirmation of LE Legacy vs Secure Connections and encryption key size | `btmon` needs root; not run |
| Second independent central while connected (AT-03) | Only one adapter available |
| macOS CoreBluetooth pairing behavior | No macOS host in this session |
| Desktop application (Qt) against the fixture on Linux | Not part of this security spike; Linux/BlueZ desktop lifecycle remains with #187/#71 follow-ups |
| Secure Boot / flash encryption / NVS encryption | Deferred by the profile; eFuse changes are irreversible and need a separate decision |
| Any production-firmware attack test (AT-01…AT-15) | Production firmware does not exist yet (#163/#165) |
