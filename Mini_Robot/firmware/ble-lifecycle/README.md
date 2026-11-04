# CodeCell ESP32-C6 BLE connection fixture

For CLion, use this folder's `platformio.ini` as the native PlatformIO entry
point. The repository-root CMake project also exposes the build-only
`firmware_ble_lifecycle` target when PlatformIO Core is available. See the
[same-window desktop/firmware workflow](../../docs/development/clion-workspace.md).

Bench firmware for [#187](https://github.com/Krzys2002/Mini_Robot/issues/187).
Follow the [physical test instructions](../../docs/testing/ble-connection-187.md)
for flashing, provisioning, fault profiles and evidence collection.

This is an Arduino/PlatformIO project based on the supplied T1-ESP-C6
configuration: `esp32-c6-devkitc-1`, USB CDC, USB mode 1, 115200 baud and
`default.csv`. The pioarduino platform is pinned to 55.03.37 and
[NimBLE-Arduino](https://github.com/h2zero/NimBLE-Arduino/releases/tag/2.3.6)
to 2.3.6 for reproducible builds. The DevKitC board definition is the same
ESP32-C6 build target used by the reference project; it does not imply that the
physical CodeCell is a DevKitC. The [manufacturer's setup guide](https://github.com/microbotsio/CodeCell#readme)
lists the CodeCell C6 with 8 MB flash and USB CDC enabled. This small fixture
retains the reference project's `default.csv` partition rather than using the
manufacturer's larger application partition; its image fits the default slot.

No CodeCell sensor, LED, motor or GPIO libraries are used. WebServer/Wi-Fi are
unnecessary for this test. Firmware advertises `MR-1` initially and persists its
ID and fault profile in NVS. It accepts a single BLE connection and resumes
advertising after disconnect. Native USB does not need an attached serial monitor
to boot. One USB data cable is sufficient; no camera or robot drive is required.

Normal mode implements the GATT layout, Robot Info read and notification
subscriptions required by the desktop connection lifecycle. It emits synthetic
Idle telemetry every 100 ms once both subscriptions are enabled; battery and
measured velocities are explicitly unknown. Limits in Robot Info are protocol
test values, not CodeCell actuator capabilities. Serial logs expose identity
reads, CCCD changes and link loss, with milliseconds since boot.

This fixture **does not implement the command protocol**: Command and Emergency
Stop writes are logged and ignored, no CommandResult frames are emitted, and no
motor output is initialized. Do not use it for #189, motion, watchdog, E-stop or
latency acceptance. Desktop readiness with an already-latched adapter E-stop
would require a CommandResult and is outside this fixture. Start a fresh desktop
process for these connection-only tests. It is not production robot firmware.

NimBLE automatically creates CCCDs for Notify properties, as documented in its
[migration guide](https://github.com/h2zero/NimBLE-Arduino/blob/2.3.6/docs/Migration_guide.md).
The `no-telemetry-notify` profile removes both Notify and its auto-generated CCCD;
it is a property-failure test, **not an isolated missing-CCCD test**. The physical
test guide records that remaining requirement explicitly.
