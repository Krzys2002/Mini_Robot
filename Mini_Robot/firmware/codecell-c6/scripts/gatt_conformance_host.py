#!/usr/bin/env python3
"""On-device v1 GATT conformance check for the production C6 image (#163).

Linux/BlueZ only. Needs pyserial and the system D-Bus bindings (python3-dbus); it
downloads nothing. It connects to `MR-<robot id>` as a generic central, checks the
GATT layout, Robot Info, readiness, CommandResult counting, the shared #64 vectors,
the dedicated Emergency Stop path with malformed payloads, CCCD disable and repeated
reconnects, and cross-checks the robot's serial status lines. Every line exchanged is
written with host monotonic and wall-clock stamps to the output log.

Motion: the only accepted Velocity it sends is 0 mm/s, 0 mrad/s, so the commanded drive
output stays at zero. Use it USB-only with the battery and motors disconnected; it is
not a powered or HIL test (#238/#166).

Example (from Mini_Robot/firmware/codecell-c6):
  scripts/gatt_conformance_host.py --port /dev/serial/by-id/usb-Espressif_* --robot-id 7 \
      --max-validity-ms 200 --cycles 20 --log gatt-163.log
"""
import argparse
import os
import struct
import sys
import threading
import time
from datetime import datetime, timezone

import serial  # pyserial

BASE = "-1f2a-4032-8415-63a5fef8a0dc"
SERVICE = "4d520001" + BASE
INFO, COMMAND, ESTOP, TELEMETRY, RESULT = ("4d52000%d" % n + BASE for n in range(2, 7))
CCCD = "00002902-0000-1000-8000-00805f9b34fb"
EXPECTED_FLAGS = {
    INFO: {"read"},
    COMMAND: {"write", "write-without-response"},
    ESTOP: {"write", "write-without-response"},
    TELEMETRY: {"read", "notify"},
    RESULT: {"notify"},
}
STOP, VELOCITY, EMERGENCY_STOP, CLEAR = 0x01, 0x02, 0x03, 0x04
ACCEPTED, STALE, INVALID_VALIDITY, MALFORMED, UNSUPPORTED = 0x00, 0x04, 0x05, 0x03, 0x01
IDLE, MOVING, LATCHED = 0, 1, 2
# Shared vectors (docs/protocol/fixtures/v1-frames.txt).
VECTOR_VELOCITY_258 = bytes.fromhex("0102020196000CFEFA00")
VECTOR_STOP_259 = bytes.fromhex("01010301")
VECTOR_ESTOP_260 = bytes.fromhex("01030401")
DBUS_TIMEOUT_S = 30
DISCOVERY_TIMEOUT_S = 20
RESULT_WAIT_S = 1.5
QUIET_S = 0.6
SERIAL_WAIT_S = 3.0
NOTIFY_READ_BYTES = 64


class Log:
    def __init__(self, path):
        self.file = open(path, "a", encoding="utf-8")
        self.lock = threading.Lock()

    def write(self, source, text):
        stamp = f"{time.monotonic():.6f} {datetime.now(timezone.utc).isoformat()} {source} {text}"
        with self.lock:
            self.file.write(stamp + "\n")
            self.file.flush()
        print(stamp, flush=True)


class SerialWatch:
    """Keeps the newest RUNTIME and BLE status lines the robot printed."""

    def __init__(self, port, log):
        self.device = serial.Serial(port, 115200, timeout=0.2)
        self.log = log
        self.lines = []
        self.lock = threading.Lock()
        self.stop = threading.Event()
        threading.Thread(target=self._read, daemon=True).start()

    def _read(self):
        buffer = b""
        while not self.stop.is_set():
            buffer += self.device.read(256)
            while b"\n" in buffer:
                raw, buffer = buffer.split(b"\n", 1)
                line = raw.decode("utf-8", "replace").strip()
                if line:
                    self.log.write("robot", line)
                    with self.lock:
                        self.lines.append((time.monotonic(), line))

    def wait_for(self, prefix, fields, since, timeout=SERIAL_WAIT_S):
        """Newest line after `since` with the prefix and every key=value in `fields`."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            with self.lock:
                for at, line in reversed(self.lines):
                    if at < since or not line.startswith(prefix):
                        continue
                    values = dict(p.split("=", 1) for p in line.split()[1:] if "=" in p)
                    if all(values.get(k) == v for k, v in fields.items()):
                        return values
            time.sleep(0.05)
        return None


class Central:
    """Minimal BlueZ D-Bus GATT client (no event loop needed)."""

    def __init__(self, name, adapter="hci0"):
        import dbus
        self.dbus = dbus
        self.bus = dbus.SystemBus()
        self.name = name
        self.adapter_path = f"/org/bluez/{adapter}"
        self.manager = dbus.Interface(self.bus.get_object("org.bluez", "/"), "org.freedesktop.DBus.ObjectManager")
        self.notify_fds = {}
        self.started = set()

    def objects(self):
        return self.manager.GetManagedObjects()

    def device_path(self):
        for path, interfaces in self.objects().items():
            device = interfaces.get("org.bluez.Device1")
            if device and str(device.get("Name", "")) == self.name and path.startswith(self.adapter_path):
                return path
        return None

    def discover(self):
        adapter = self.dbus.Interface(self.bus.get_object("org.bluez", self.adapter_path), "org.bluez.Adapter1")
        adapter.SetDiscoveryFilter({"UUIDs": [SERVICE], "Transport": "le"})
        adapter.StartDiscovery()
        try:
            deadline = time.monotonic() + DISCOVERY_TIMEOUT_S
            while time.monotonic() < deadline and not self.device_path():
                time.sleep(0.2)
        finally:
            adapter.StopDiscovery()
        path = self.device_path()
        if not path:
            return None
        device = self.objects()[path]["org.bluez.Device1"]
        return {"path": path, "uuids": [str(u).lower() for u in device.get("UUIDs", [])]}

    def connect(self):
        path = self.device_path()
        self.device = self.dbus.Interface(self.bus.get_object("org.bluez", path), "org.bluez.Device1")
        props = self.dbus.Interface(self.bus.get_object("org.bluez", path), "org.freedesktop.DBus.Properties")
        self.device.Connect(timeout=DBUS_TIMEOUT_S)
        deadline = time.monotonic() + DBUS_TIMEOUT_S
        while not props.Get("org.bluez.Device1", "ServicesResolved"):
            if time.monotonic() > deadline:
                raise RuntimeError("services not resolved")
            time.sleep(0.1)
        self.chars, self.flags, self.descriptors = {}, {}, {}
        for object_path, interfaces in self.objects().items():
            if not object_path.startswith(path):
                continue
            gatt = interfaces.get("org.bluez.GattCharacteristic1")
            if gatt:
                uuid = str(gatt["UUID"]).lower()
                self.chars[uuid] = self.dbus.Interface(self.bus.get_object("org.bluez", object_path),
                                                       "org.bluez.GattCharacteristic1")
                self.flags[uuid] = {str(f) for f in gatt["Flags"]}
                self.descriptors.setdefault(object_path, [])
            descriptor = interfaces.get("org.bluez.GattDescriptor1")
            if descriptor:
                self.descriptors.setdefault(str(descriptor["Characteristic"]), []).append(
                    str(descriptor["UUID"]).lower())
        self.char_paths = {uuid: str(char.object_path) for uuid, char in self.chars.items()}

    def cccd_present(self, uuid):
        return CCCD in self.descriptors.get(self.char_paths.get(uuid, ""), [])

    def read(self, uuid):
        return bytes(self.chars[uuid].ReadValue({}, timeout=DBUS_TIMEOUT_S))

    def write(self, uuid, data, request=True):
        self.chars[uuid].WriteValue(self.dbus.Array(list(data), signature="y"),
                                    {"type": "request" if request else "command"}, timeout=DBUS_TIMEOUT_S)

    def subscribe(self, uuid, sink):
        """AcquireNotify writes the CCCD; closing the descriptor disables it again."""
        fd, mtu = self.chars[uuid].AcquireNotify({})
        fd = fd.take()
        os.set_blocking(fd, True)
        stop = threading.Event()
        self.notify_fds[uuid] = (fd, stop)
        threading.Thread(target=reader, args=(fd, sink, stop), daemon=True).start()
        return int(mtu)

    def unsubscribe(self, uuid):
        fd, stop = self.notify_fds.pop(uuid)
        stop.set()
        os.close(fd)

    def start_notify(self, uuid):
        """StartNotify writes the CCCD; StopNotify writes it back to 0. Used for Telemetry:
        BlueZ releases an acquired notification only when a later notification fails to be
        delivered, so closing an AcquireNotify descriptor of a silent characteristic leaves
        its CCCD enabled. The values are not read here (no event loop)."""
        self.chars[uuid].StartNotify(timeout=DBUS_TIMEOUT_S)
        self.started.add(uuid)

    def stop_notify(self, uuid):
        self.chars[uuid].StopNotify(timeout=DBUS_TIMEOUT_S)
        self.started.discard(uuid)

    def disconnect(self):
        for uuid in list(self.notify_fds):
            self.unsubscribe(uuid)
        for uuid in list(self.started):
            self.stop_notify(uuid)
        self.device.Disconnect(timeout=DBUS_TIMEOUT_S)


def reader(fd, sink, stop):
    while not stop.is_set():
        try:
            data = os.read(fd, NOTIFY_READ_BYTES)
        except OSError:
            return
        if not data:
            return
        sink.append(data)


def header(message_type, sequence):
    return struct.pack("<BBH", 1, message_type, sequence & 0xFFFF)


def velocity(sequence, linear, angular, valid_for_ms):
    return struct.pack("<BBHhhH", 1, VELOCITY, sequence & 0xFFFF, linear, angular, valid_for_ms)


def parse_result(data):
    if len(data) != 9 or data[0] != 1 or data[1] != 0x81:
        return None
    seq, cmd_seq, cmd_type, code, state = struct.unpack("<HHBBB", data[2:9])
    return {"seq": seq, "cmd_seq": cmd_seq, "cmd_type": cmd_type, "code": code, "state": state}


class Checks:
    def __init__(self, log):
        self.log, self.failed, self.passed = log, [], 0

    def check(self, name, condition, detail=""):
        self.log.write("check", f"{'PASS' if condition else 'FAIL'} {name} {detail}".rstrip())
        if condition:
            self.passed += 1
        else:
            self.failed.append(name)
        return condition


def wait_results(sink, count, timeout=RESULT_WAIT_S):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline and len(sink) < count:
        time.sleep(0.02)
    return [parse_result(d) for d in sink[:count]]


def exchange(central, sink, uuid, data, request=True):
    before = len(sink)
    central.write(uuid, data, request)
    results = wait_results(sink, before + 1)
    return results[before] if len(results) > before else None


def run(args, log, watch, checks):
    central = Central(f"MR-{args.robot_id}")
    found = central.discover()
    if not checks.check("C1 discovery by service UUID and MR-<id> name", found is not None):
        return
    checks.check("C1 advertised service UUID", SERVICE in found["uuids"], str(found["uuids"]))

    central.connect()
    for uuid, flags in EXPECTED_FLAGS.items():
        checks.check(f"C2 properties {uuid[:8]}", central.flags.get(uuid) == flags, str(central.flags.get(uuid)))
    checks.check("C2 CCCDs on Telemetry and Command Result",
                 central.cccd_present(TELEMETRY) and central.cccd_present(RESULT))

    info = central.read(INFO)
    ok = len(info) == 18
    fields = struct.unpack("<BBBI3BHHHH", info) if ok else ()
    checks.check("C3 Robot Info length and format", ok and fields[0] == 1 and fields[1] == 1 and fields[2] == 1,
                 info.hex())
    checks.check("C3 Robot Info robotId", ok and fields[3] == args.robot_id)
    checks.check("C3 Robot Info validity ceiling", ok and fields[9] == args.max_validity_ms)

    results = []
    # Partial subscription: Command Result only. Velocity is ignored without a result.
    since = time.monotonic()
    mtu = central.subscribe(RESULT, results)
    log.write("host", f"result notify mtu={mtu}")
    central.write(COMMAND, velocity(1, 0, 0, args.max_validity_ms))
    time.sleep(QUIET_S)
    checks.check("C4 no result before both CCCDs", not results)
    checks.check("C4 robot reports cccd_result=1 not ready", watch.wait_for(
        "BLE", {"state": "connected", "cccd_result": "1", "cccd_telemetry": "0"}, since) is not None)
    since = time.monotonic()
    central.write(ESTOP, b"")
    checks.check("C4 E-stop before readiness latches", watch.wait_for(
        "RUNTIME", {"state": "EmergencyStop"}, since) is not None)
    time.sleep(QUIET_S)
    checks.check("C4 still no result before readiness", not results)

    since = time.monotonic()
    central.start_notify(TELEMETRY)
    checks.check("C5 both CCCDs make the session ready", watch.wait_for(
        "RUNTIME", {"session": "1"}, since) is not None)
    sequence = 0
    r = exchange(central, results, COMMAND, header(CLEAR, 1))
    checks.check("C5 explicit clear in the ready session", r is not None and r["code"] == ACCEPTED
                 and r["state"] == IDLE and r["seq"] == sequence, str(r))
    sequence += 1

    r = exchange(central, results, COMMAND, VECTOR_VELOCITY_258)
    expected = INVALID_VALIDITY if args.max_validity_ms < 250 else ACCEPTED
    checks.check("C6 vector velocity_seq258", r is not None and r["cmd_seq"] == 258 and r["code"] == expected, str(r))
    if expected == ACCEPTED:
        central.write(COMMAND, VECTOR_STOP_259)  # Never leave a nonzero setpoint running.
    r = exchange(central, results, COMMAND, VECTOR_STOP_259)
    checks.check("C6 vector stop_seq259", r is not None and r["cmd_seq"] == 259 and r["code"] == ACCEPTED, str(r))
    r = exchange(central, results, COMMAND, VECTOR_VELOCITY_258)
    checks.check("C6 older velocity is StaleSequence", r is not None and r["code"] == STALE, str(r))
    r = exchange(central, results, ESTOP, VECTOR_ESTOP_260)
    checks.check("C6 vector estop_seq260 on Emergency Stop", r is not None and r["cmd_seq"] == 260
                 and r["cmd_type"] == EMERGENCY_STOP and r["code"] == ACCEPTED and r["state"] == LATCHED, str(r))
    r = exchange(central, results, COMMAND, header(CLEAR, 261))
    checks.check("C6 clear after vector E-stop", r is not None and r["state"] == IDLE, str(r))
    r = exchange(central, results, COMMAND, velocity(262, 0, 0, args.max_validity_ms))
    checks.check("C6 zero Velocity accepted and Moving", r is not None and r["code"] == ACCEPTED
                 and r["state"] == MOVING, str(r))
    since = time.monotonic()
    checks.check("C6 watchdog expiry sets the flag", watch.wait_for(
        "RUNTIME", {"state": "Idle", "expired_flag": "1"}, since) is not None)

    # Dedicated E-stop: any payload latches and is answered with seq 0 unless well-formed.
    payloads = [b"", b"\x01", b"\x02\x03\x04\x01", b'{"v":1}', bytes(range(20)), bytes(64)]
    clear_sequence = 300
    for payload in payloads:
        try:
            r = exchange(central, results, ESTOP, payload)
        except central.dbus.exceptions.DBusException as error:
            checks.check(f"C7 E-stop payload len={len(payload)} written", False, str(error))
            continue
        checks.check(f"C7 E-stop payload len={len(payload)}", r is not None and r["cmd_type"] == EMERGENCY_STOP
                     and r["cmd_seq"] == 0 and r["code"] == ACCEPTED and r["state"] == LATCHED, str(r))
        r = exchange(central, results, COMMAND, header(CLEAR, clear_sequence))
        clear_sequence += 1
        checks.check(f"C7 clear after payload len={len(payload)}", r is not None and r["state"] == IDLE, str(r))
    r = exchange(central, results, COMMAND, bytes([1, VELOCITY]) + bytes(38))
    checks.check("C7 oversized Command write is MalformedLength", r is not None and r["code"] == MALFORMED, str(r))
    r = exchange(central, results, COMMAND, b'{"v":1,"type":"stop"}')
    checks.check("C7 JSON on Command is UnsupportedVersion", r is not None and r["code"] == UNSUPPORTED, str(r))

    # One result per frame on one counter, with Write Without Response.
    before = len(results)
    for i in range(args.burst):
        central.write(COMMAND, header(STOP, 400 + i), request=False)
    burst = wait_results(results, before + args.burst, timeout=RESULT_WAIT_S + args.burst * 0.05)[before:]
    time.sleep(QUIET_S)
    sequences = [r["seq"] for r in burst if r]
    checks.check("C8 exactly one result per write-without-response frame",
                 len(results) - before == args.burst, f"{len(results) - before}/{args.burst}")
    all_seqs = [parse_result(d)["seq"] for d in results]
    checks.check("C8 shared counter has no gap or repeat", all_seqs == list(range(len(all_seqs))),
                 f"first={all_seqs[:3]} last={all_seqs[-3:]}")
    del sequences

    # CCCD disable ends the session; re-enable starts a new one at sequence 0.
    since = time.monotonic()
    central.stop_notify(TELEMETRY)
    checks.check("C9 Telemetry CCCD disable ends the session", watch.wait_for(
        "RUNTIME", {"session": "0"}, since) is not None)
    before = len(results)
    central.write(COMMAND, header(STOP, 500))
    time.sleep(QUIET_S)
    checks.check("C9 no result after CCCD disable", len(results) == before)
    central.start_notify(TELEMETRY)
    r = exchange(central, results, COMMAND, header(STOP, 7))
    checks.check("C9 re-enable starts a new session at sequence 0", r is not None and r["seq"] == 0, str(r))
    central.disconnect()
    since = time.monotonic()
    checks.check("C10 disconnect ends the session and advertises again", watch.wait_for(
        "BLE", {"state": "advertising"}, since, timeout=SERIAL_WAIT_S * 2) is not None)

    failures = 0
    for cycle in range(args.cycles):
        try:
            central.connect()
            sink = []
            central.subscribe(RESULT, sink)
            central.start_notify(TELEMETRY)
            r = exchange(central, sink, COMMAND, header(STOP, 1000 + cycle))
            if r is None or r["seq"] != 0 or r["code"] != ACCEPTED:
                failures += 1
                log.write("host", f"cycle {cycle} result {r}")
            central.disconnect()
            time.sleep(0.5)
        except central.dbus.exceptions.DBusException as error:
            failures += 1
            log.write("host", f"cycle {cycle} error {error}")
            time.sleep(1.0)
    checks.check(f"C11 {args.cycles} reconnect cycles, each a fresh session", failures == 0, f"failures={failures}")
    since = time.monotonic()
    checks.check("C11 robot idle, no session, advertising after the cycles", watch.wait_for(
        "RUNTIME", {"state": "Idle", "session": "0"}, since - 5.0, timeout=SERIAL_WAIT_S * 3) is not None)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", required=True)
    parser.add_argument("--robot-id", type=int, required=True)
    parser.add_argument("--max-validity-ms", type=int, required=True, help="provisioned maxCommandValidityMs")
    parser.add_argument("--cycles", type=int, default=20)
    parser.add_argument("--burst", type=int, default=50)
    parser.add_argument("--log", default="gatt-conformance.log")
    args = parser.parse_args()
    log = Log(args.log)
    watch = SerialWatch(args.port, log)
    checks = Checks(log)
    try:
        run(args, log, watch, checks)
    finally:
        watch.stop.set()
    log.write("host", f"SUMMARY passed={checks.passed} failed={len(checks.failed)} {' | '.join(checks.failed)}")
    return 1 if checks.failed else 0


if __name__ == "__main__":
    sys.exit(main())
