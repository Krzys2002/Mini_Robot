#!/usr/bin/env python3
"""On-device v1 GATT conformance check for the production C6 image (#163, #164).

Linux/BlueZ only. Needs pyserial and the system D-Bus bindings (python3-dbus); it
downloads nothing. It connects to `MR-<robot id>` as a generic central, checks the
GATT layout, Robot Info, readiness, CommandResult counting, the shared #64 vectors,
the dedicated Emergency Stop path with malformed payloads, CCCD disable and repeated
reconnects, and cross-checks the robot's serial status lines. Since #164 it also reads
Telemetry: v1 bytes and unknown sentinels, the periodic schedule on the robot clock,
state-change frames after E-stop and expiry, the Read value, and the robot counter shared
with CommandResults. A write flood and a paused serial reader stress notifications and
logging; the robot's TELEMETRY, TASK and RESOURCES lines are then checked against the #156
bounds. Every line exchanged is written with host monotonic and wall-clock stamps to the
output log.

Motion: the only accepted Velocity it sends is 0 mm/s, 0 mrad/s, so the commanded drive
output stays at zero. The shared vector velocity_seq258 (150 mm/s, -500 mrad/s, valid for
250 ms) is nonzero, so it is sent only when the robot's Robot Info reports a validity
ceiling below 250 ms, which guarantees InvalidValidity; otherwise it is skipped and the
check fails. --max-validity-ms must therefore be below 250. Use it USB-only with the
battery and motors disconnected; it is not a powered or HIL test (#238/#166).

Battery (#309): the full run expects the battery as unknown in every frame, with batteryLow
never set: the default image does not read the battery, and on USB the battery-bench image
reads the USB rail, which is outside the provisional plausible range. --battery-present is a
separate BLE-only pass for the battery-bench image on its LiPo with USB unplugged (no serial
cross-checks are possible then). It sends no command, requires a plausible voltage in every
frame, a batteryLow flag consistent with the provisional thresholds, and the B10 periodic
spacing on the robot clock.

Example (from Mini_Robot/firmware/codecell-c6):
  scripts/gatt_conformance_host.py --port /dev/serial/by-id/usb-Espressif_* --robot-id 7 \
      --max-validity-ms 200 --cycles 20 --log gatt-163.log
  scripts/gatt_conformance_host.py --battery-present --robot-id 7 --duration-s 30 --log battery-309.log
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
TELEMETRY_TYPE, RESULT_TYPE = 0x80, 0x81
FLAG_EXPIRED, FLAG_BATTERY_LOW, FLAG_HAS_ACCEPTED = 0x01, 0x02, 0x04
UNKNOWN_BATTERY, UNKNOWN_VELOCITY = 0xFFFF, -32768
# #309 provisional battery evidence (include/telemetry/CodeCellBattery.hpp), battery-bench image
# only: a published value lies in this range, batteryLow is set at or below the enter threshold
# and cleared at or above the exit threshold.
BATTERY_PLAUSIBLE_MV = (3000, 4400)
BATTERY_LOW_ENTER_MV, BATTERY_LOW_EXIT_MV = 3350, 3450
EXPECTED_FIRMWARE = (0, 1, 0)
# #156 budgets the robot reports on (docs/protocol/timing-budgets.md).
TELEMETRY_PERIOD_MS = 100
PERIOD_DEVIATION_MS = 2 + 1        # B10 robot-side deviation, plus 1 ms of uptime truncation.
STATE_SPACING_MS = 20              # B10 state-change spacing.
SILENCE_PERIODS = 3                # Desktop telemetry-silence threshold.
SAFETY_LATENESS_US = 2000          # B3.
MIN_FREE_HEAP = 64 * 1024          # B15.
MIN_STACK_FREE_PCT = 25            # B16.
ACCEPTED, STALE, INVALID_VALIDITY, MALFORMED, UNSUPPORTED = 0x00, 0x04, 0x05, 0x03, 0x01
IDLE, MOVING, LATCHED = 0, 1, 2
# Shared vectors (docs/protocol/fixtures/v1-frames.txt).
VECTOR_VELOCITY_258 = bytes.fromhex("0102020196000CFEFA00")
VECTOR_VELOCITY_258_VALID_FOR_MS = 250  # Nonzero motion: sent only below this ceiling.
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
        self.paused = threading.Event()
        threading.Thread(target=self._read, daemon=True).start()

    def _read(self):
        buffer = b""
        while not self.stop.is_set():
            if self.paused.is_set():  # The host stops reading; the port stays open (#156 S5).
                time.sleep(0.05)
                continue
            buffer += self.device.read(256)
            while b"\n" in buffer:
                raw, buffer = buffer.split(b"\n", 1)
                line = raw.decode("utf-8", "replace").strip()
                if line:
                    self.log.write("robot", line)
                    with self.lock:
                        self.lines.append((time.monotonic(), line))

    def all_since(self, prefix, since):
        """key=value fields of every line with the prefix read after `since`, oldest first."""
        with self.lock:
            lines = [(at, line) for at, line in self.lines if at >= since and line.startswith(prefix)]
        return [dict(p.split("=", 1) for p in line.split()[1:] if "=" in p) for _, line in lines]

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


class TimedSink(list):
    """Notification payloads with their host receive times."""

    def __init__(self):
        super().__init__()
        self.times = []

    def append(self, data):
        self.times.append(time.monotonic())
        super().append(data)


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
    if len(data) != 9 or data[0] != 1 or data[1] != RESULT_TYPE:
        return None
    seq, cmd_seq, cmd_type, code, state = struct.unpack("<HHBBB", data[2:9])
    return {"seq": seq, "cmd_seq": cmd_seq, "cmd_type": cmd_type, "code": code, "state": state}


def battery_low_consistent(frame):
    """batteryLow only with a published voltage, and on the right side of the #309 thresholds."""
    low = bool(frame["flags"] & FLAG_BATTERY_LOW)
    mv = frame["battery"]
    if mv == UNKNOWN_BATTERY:
        return not low
    if mv <= BATTERY_LOW_ENTER_MV:
        return low
    if mv >= BATTERY_LOW_EXIT_MV:
        return not low
    return True  # Hysteresis band: either is valid.


def battery_plausible(millivolts):
    return millivolts != UNKNOWN_BATTERY and BATTERY_PLAUSIBLE_MV[0] <= millivolts <= BATTERY_PLAUSIBLE_MV[1]


def parse_telemetry(data):
    if len(data) != 18 or data[0] != 1 or data[1] != TELEMETRY_TYPE:
        return None
    seq, state, flags, last, uptime, battery, linear, angular = struct.unpack("<HBBHIHhh", data[2:18])
    return {"seq": seq, "state": state, "flags": flags, "last": last, "uptime": uptime, "battery": battery,
            "linear": linear, "angular": angular}


def robot_sequences(*sinks):
    """Every robot frame sequence seen on the notification sinks (results and telemetry)."""
    out = []
    for sink in sinks:
        for data in sink:
            frame = parse_result(data) or parse_telemetry(data)
            out.append(frame["seq"] if frame else None)
    return out


def gap_free_from_zero(sequences):
    return None not in sequences and sorted(sequences) == list(range(len(sequences)))


def wait_telemetry(sink, predicate, start, timeout=RESULT_WAIT_S):
    """First telemetry frame at index >= start that satisfies `predicate`, with its index."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        for i in range(start, len(sink)):
            t = parse_telemetry(sink[i])
            if t and predicate(t):
                return i, t
        time.sleep(0.02)
    return None, None


def periodic_deltas(frames):
    """Robot-clock gaps (ms) between consecutive frames that carry the same state and flags."""
    out = []
    for a, b in zip(frames, frames[1:]):
        if a and b and (a["state"], a["flags"]) == (b["state"], b["flags"]):
            out.append((b["uptime"] - a["uptime"]) & 0xFFFFFFFF)
    return out


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
    checks.check("C3 Robot Info firmware version", ok and tuple(fields[4:7]) == EXPECTED_FIRMWARE,
                 str(tuple(fields[4:7]) if ok else None))
    checks.check("C3 Robot Info telemetry period", ok and fields[10] == TELEMETRY_PERIOD_MS)
    checks.check("C3 Telemetry Read is empty outside a session", len(central.read(TELEMETRY)) == 0)
    # The robot's own ceiling decides whether the nonzero shared vector is certain to be refused.
    vector_258_refused = ok and fields[9] < VECTOR_VELOCITY_258_VALID_FOR_MS

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
    session_start = since
    telemetry = TimedSink()
    central.subscribe(TELEMETRY, telemetry)
    checks.check("C5 both CCCDs make the session ready", watch.wait_for(
        "RUNTIME", {"session": "1"}, since) is not None)

    # T1: the session's first Telemetry frame, while still latched from C4.
    index, first = wait_telemetry(telemetry, lambda t: True, 0)
    checks.check("T1 first telemetry is sequence 0 of the session", first is not None and first["seq"] == 0,
                 str(first))
    checks.check("T1 telemetry state and flags (latched, nothing accepted)", first is not None
                 and first["state"] == LATCHED and first["flags"] == 0 and first["last"] == 0, str(first))
    checks.check("T1 battery and measured velocity are the unknown sentinels", first is not None
                 and first["battery"] == UNKNOWN_BATTERY and first["linear"] == UNKNOWN_VELOCITY
                 and first["angular"] == UNKNOWN_VELOCITY, str(first))
    # T2: periodic frames on the robot clock while nothing changes.
    time.sleep(2.0)
    frames = [parse_telemetry(d) for d in telemetry]
    deltas = periodic_deltas(frames)
    checks.check("T2 periodic telemetry every telemetryPeriodMs on the robot clock", len(deltas) >= 15 and all(
        abs(d - TELEMETRY_PERIOD_MS) <= PERIOD_DEVIATION_MS for d in deltas), f"n={len(deltas)} deltas={sorted(set(deltas))}")
    host_gaps = [b - a for a, b in zip(telemetry.times, telemetry.times[1:])]
    checks.check("T2 host never sees a telemetry silence of 3 periods", host_gaps and max(host_gaps)
                 < SILENCE_PERIODS * TELEMETRY_PERIOD_MS / 1000.0, f"max_gap_s={max(host_gaps) if host_gaps else None}")
    value = central.read(TELEMETRY)
    latest = parse_telemetry(value)
    newest = max(f["seq"] for f in frames if f)
    checks.check("T3 Telemetry Read returns a recent frame with its own sequence", latest is not None
                 and newest - 1 <= latest["seq"] <= newest + 2, f"read={latest} newest_notified={newest}")

    r = exchange(central, results, COMMAND, header(CLEAR, 1))
    checks.check("C5 explicit clear in the ready session", r is not None and r["code"] == ACCEPTED
                 and r["state"] == IDLE, str(r))
    index, cleared = wait_telemetry(telemetry, lambda t: t["state"] == IDLE and t["seq"] > r["seq"], 0) if r else (None, None)
    checks.check("T4 state change after the clear is published", cleared is not None
                 and cleared["flags"] & FLAG_HAS_ACCEPTED and cleared["last"] == 1, str(cleared))

    if vector_258_refused:
        r = exchange(central, results, COMMAND, VECTOR_VELOCITY_258)
        checks.check("C6 vector velocity_seq258", r is not None and r["cmd_seq"] == 258
                     and r["code"] == INVALID_VALIDITY, str(r))
    else:
        checks.check("C6 vector velocity_seq258 not sent: the robot's validity ceiling would accept "
                     "its nonzero velocity", False, f"ceiling={fields[9] if ok else 'unknown'}")
    r = exchange(central, results, COMMAND, VECTOR_STOP_259)
    stop_259 = r is not None and r["cmd_seq"] == 259 and r["code"] == ACCEPTED
    checks.check("C6 vector stop_seq259", stop_259, str(r))
    if vector_258_refused and stop_259:
        r = exchange(central, results, COMMAND, VECTOR_VELOCITY_258)
        checks.check("C6 older velocity is StaleSequence", r is not None and r["code"] == STALE, str(r))
    else:
        checks.check("C6 older velocity not sent: refusal is not guaranteed", False)
    r = exchange(central, results, ESTOP, VECTOR_ESTOP_260)
    checks.check("C6 vector estop_seq260 on Emergency Stop", r is not None and r["cmd_seq"] == 260
                 and r["cmd_type"] == EMERGENCY_STOP and r["code"] == ACCEPTED and r["state"] == LATCHED, str(r))
    index, latched = (wait_telemetry(telemetry, lambda t: t["state"] == LATCHED and t["seq"] > r["seq"], 0)
                      if r else (None, None))
    checks.check("T4 E-stop result first, then telemetry showing EmergencyStop", latched is not None, str(latched))
    r = exchange(central, results, COMMAND, header(CLEAR, 261))
    checks.check("C6 clear after vector E-stop", r is not None and r["state"] == IDLE, str(r))
    start = len(telemetry)
    r = exchange(central, results, COMMAND, velocity(262, 0, 0, args.max_validity_ms))
    checks.check("C6 zero Velocity accepted and Moving", r is not None and r["code"] == ACCEPTED
                 and r["state"] == MOVING, str(r))
    since = time.monotonic()
    checks.check("C6 watchdog expiry sets the flag", watch.wait_for(
        "RUNTIME", {"state": "Idle", "expired_flag": "1"}, since) is not None)
    _, moving = wait_telemetry(telemetry, lambda t: t["state"] == MOVING, start)
    checks.check("T5 Moving telemetry reports velocity unknown, not the commanded zero", moving is not None
                 and moving["linear"] == UNKNOWN_VELOCITY and moving["angular"] == UNKNOWN_VELOCITY, str(moving))
    _, expired = wait_telemetry(telemetry, lambda t: t["state"] == IDLE and t["flags"] & FLAG_EXPIRED, start)
    checks.check("T5 expiry is published with commandExpired", expired is not None and expired["last"] == 262,
                 str(expired))
    if moving and expired:
        checks.check("T5 expiry frame within the state spacing of the robot deadline", 0 < (
            (expired["uptime"] - moving["uptime"]) & 0xFFFFFFFF) <= args.max_validity_ms + STATE_SPACING_MS
            + TELEMETRY_PERIOD_MS, f"moving_uptime={moving['uptime']} expired_uptime={expired['uptime']}")

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
    checks.check("C8 exactly one result per write-without-response frame",
                 len(results) - before == args.burst, f"{len(results) - before}/{args.burst}")
    all_seqs = robot_sequences(results, telemetry)
    checks.check("C8 results and telemetry share one counter with no gap or repeat", gap_free_from_zero(all_seqs),
                 f"frames={len(all_seqs)} telemetry={len(telemetry)}")

    stress_notifications(args, log, watch, checks, central, results, telemetry, session_start)

    # CCCD disable ends the session; re-enable starts a new one at sequence 0.
    # Closing the acquired Telemetry descriptor makes BlueZ write the CCCD off at the next
    # notification it cannot deliver, at most one telemetry period later.
    since = time.monotonic()
    central.unsubscribe(TELEMETRY)
    checks.check("C9 Telemetry CCCD disable ends the session", watch.wait_for(
        "RUNTIME", {"session": "0"}, since) is not None)
    time.sleep(QUIET_S)
    before = len(results)
    central.write(COMMAND, header(STOP, 500))
    time.sleep(QUIET_S)
    checks.check("C9 no result after CCCD disable", len(results) == before)
    # Command Result stays acquired (BlueZ would refuse a second AcquireNotify); only the
    # results after the re-enable belong to the new session.
    first_new = len(results)
    telemetry = TimedSink()
    central.subscribe(TELEMETRY, telemetry)
    r = exchange(central, results, COMMAND, header(STOP, 7))
    time.sleep(QUIET_S)
    seqs = robot_sequences(results[first_new:], telemetry)
    checks.check("C9 re-enable starts a new session: its frames count from sequence 0",
                 r is not None and gap_free_from_zero(seqs), f"result={r} sequences={sorted(s for s in seqs if s is not None)[:6]}")
    central.disconnect()
    since = time.monotonic()
    checks.check("C10 disconnect ends the session and advertises again", watch.wait_for(
        "BLE", {"state": "advertising"}, since, timeout=SERIAL_WAIT_S * 2) is not None)

    failures = 0
    for cycle in range(args.cycles):
        try:
            central.connect()
            sink, telemetry_sink = TimedSink(), TimedSink()
            central.subscribe(RESULT, sink)
            central.subscribe(TELEMETRY, telemetry_sink)
            r = exchange(central, sink, COMMAND, header(STOP, 1000 + cycle))
            time.sleep(0.2)
            seqs = robot_sequences(sink, telemetry_sink)
            if r is None or r["code"] != ACCEPTED or not gap_free_from_zero(seqs) or not telemetry_sink:
                failures += 1
                log.write("host", f"cycle {cycle} result {r} sequences {sorted(s for s in seqs if s is not None)}")
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


def stress_notifications(args, log, watch, checks, central, results, telemetry, session_start):
    """C12/C13: notification and logging pressure, then the robot's own timing and resources.

    C12 floods Command with write-without-response Stops while telemetry flows. C13 stops
    reading the serial port for a while (the port stays open), as #156 S5. Neither moves the
    robot: Stop only. Afterwards the TELEMETRY, TASK and RESOURCES lines are checked against
    B3, B10, B15 and B16."""
    before_results, before_telemetry = len(results), len(telemetry)
    since = time.monotonic()
    for i in range(args.flood):
        central.write(COMMAND, header(STOP, 2000 + i), request=False)
    wait_results(results, before_results + args.flood, timeout=RESULT_WAIT_S + args.flood * 0.05)
    time.sleep(QUIET_S)
    flood_s = time.monotonic() - since
    answered = len(results) - before_results
    log.write("host", f"flood writes={args.flood} results={answered} telemetry={len(telemetry) - before_telemetry} "
                      f"duration_s={flood_s:.2f}")
    checks.check("C12 every flood frame answered, or the overload policy ended the session",
                 answered == args.flood or watch.wait_for("BLE", {"state": "closing"}, since, timeout=0.1) is not None,
                 f"{answered}/{args.flood}")
    times = telemetry.times[before_telemetry:]
    gaps = [b - a for a, b in zip(times, times[1:])]
    checks.check("C12 telemetry keeps flowing during the flood", len(times) >= flood_s * 1000 / TELEMETRY_PERIOD_MS / 2,
                 f"frames={len(times)} max_host_gap_s={max(gaps) if gaps else None}")
    checks.check("C12 shared counter still gap-free after the flood",
                 gap_free_from_zero(robot_sequences(results, telemetry)))

    watch.paused.set()
    log.write("host", f"serial reading paused for {args.serial_pause_s} s")
    before_telemetry = len(telemetry)
    for i in range(int(args.serial_pause_s * 10)):  # Keep the radio side busy meanwhile.
        central.write(COMMAND, header(STOP, 6000 + i), request=False)
        time.sleep(0.1)
    watch.paused.clear()
    log.write("host", "serial reading resumed")
    paused_times = telemetry.times[before_telemetry:]
    gaps = [b - a for a, b in zip(paused_times, paused_times[1:])]
    checks.check("C13 telemetry unaffected while the serial host does not read", gaps and max(gaps)
                 < SILENCE_PERIODS * TELEMETRY_PERIOD_MS / 1000.0, f"max_host_gap_s={max(gaps) if gaps else None}")

    # The next report round after both stresses (one every 5 s).
    since = time.monotonic()
    report = watch.wait_for("TELEMETRY", {}, since, timeout=12.0)
    safety = watch.wait_for("TASK", {"name": "mr_safety"}, since, timeout=12.0)
    resources = watch.wait_for("RESOURCES", {}, since, timeout=12.0)
    tasks = [watch.wait_for("TASK", {"name": n}, since, timeout=1.0)
             for n in ("mr_safety", "mr_control", "mr_telemetry", "nimble_host", "loopTask")]
    log.write("host", f"robot report telemetry={report} safety={safety} resources={resources}")
    checks.check("C14 robot TELEMETRY line reports unknown battery and velocity", report is not None
                 and report.get("battery_mv") == "unknown" and report.get("velocity") == "unknown", str(report))
    # #309: across every frame of this session, not only T1, the battery stays unknown and
    # batteryLow is never set (the flag needs a published voltage).
    session_frames = [f for f in (parse_telemetry(d) for d in telemetry) if f]
    bad = [f for f in session_frames if f["battery"] != UNKNOWN_BATTERY or f["flags"] & FLAG_BATTERY_LOW]
    checks.check("C14 every telemetry frame reports unknown battery without batteryLow",
                 session_frames and not bad, f"n={len(session_frames)} first_bad={bad[0] if bad else None}")
    # Every 5 s report window of this session, not only the last: B10 is a bound under each load.
    windows = [int(v["periodic_dev_max_us"]) for v in watch.all_since("TELEMETRY", session_start)
               if v.get("periodic_dev_max_us", "none") != "none"]
    checks.check("C14 robot-side periodic deviation within B10 in every report window", windows and max(windows)
                 <= 2000, f"windows={len(windows)} max_us={max(windows) if windows else None} all_us={windows}")
    checks.check("C14 supervisor wake lateness within B3 (worst since boot)", safety is not None and int(
        safety.get("late_worst_us", 10 ** 9)) <= SAFETY_LATENESS_US, str(safety))
    checks.check("C14 minimum free heap within B15", resources is not None and int(
        resources.get("heap_min_free", 0)) >= MIN_FREE_HEAP, str(resources))
    for name, task in zip(("mr_safety", "mr_control", "mr_telemetry", "nimble_host", "loopTask"), tasks):
        checks.check(f"C14 {name} stack free within B16", task is not None and int(
            task.get("stack_free_pct", 0)) >= MIN_STACK_FREE_PCT, str(task))


def run_battery_present(args, log, checks):
    """BLE-only #309 pass on the battery-bench image, LiPo only, USB unplugged. Sends no command."""
    central = Central(f"MR-{args.robot_id}")
    found = central.discover()
    if not checks.check("P1 discovery by service UUID and MR-<id> name", found is not None):
        return
    central.connect()
    try:
        info = central.read(INFO)
        ok = len(info) == 18
        fields = struct.unpack("<BBBI3BHHHH", info) if ok else ()
        checks.check("P1 Robot Info robotId and telemetry period", ok and fields[3] == args.robot_id
                     and fields[10] == TELEMETRY_PERIOD_MS, info.hex())
        results, telemetry = [], TimedSink()
        central.subscribe(RESULT, results)
        central.subscribe(TELEMETRY, telemetry)
        time.sleep(args.duration_s)
        frames = [parse_telemetry(d) for d in telemetry]
        valid = [f for f in frames if f]
        expected = args.duration_s * 1000 / TELEMETRY_PERIOD_MS
        log.write("host", f"battery frames={len(valid)} mv={sorted({f['battery'] for f in valid})}")
        checks.check("P2 telemetry frames received for the whole window", len(valid) == len(frames)
                     and len(valid) >= 0.9 * expected, f"n={len(valid)} expected~{expected:.0f}")
        implausible = [f for f in valid if not battery_plausible(f["battery"])]
        checks.check("P3 every frame publishes a plausible battery voltage", valid and not implausible,
                     f"range={BATTERY_PLAUSIBLE_MV} first_bad={implausible[0] if implausible else None}")
        inconsistent = [f for f in valid if not battery_low_consistent(f)]
        checks.check("P4 batteryLow matches the published voltage and thresholds", valid and not inconsistent,
                     f"first_bad={inconsistent[0] if inconsistent else None}")
        checks.check("P5 measured velocity stays the unknown sentinel", valid and all(
            f["linear"] == UNKNOWN_VELOCITY and f["angular"] == UNKNOWN_VELOCITY for f in valid))
        deltas = periodic_deltas(frames)
        checks.check("P6 periodic telemetry within B10 on the robot clock with the battery read",
                     len(deltas) >= 0.8 * expected and all(abs(d - TELEMETRY_PERIOD_MS) <= PERIOD_DEVIATION_MS
                                                           for d in deltas),
                     f"n={len(deltas)} deltas={sorted(set(deltas))}")
        host_gaps = [b - a for a, b in zip(telemetry.times, telemetry.times[1:])]
        checks.check("P7 host never sees a telemetry silence of 3 periods", host_gaps and max(host_gaps)
                     < SILENCE_PERIODS * TELEMETRY_PERIOD_MS / 1000.0,
                     f"max_gap_s={max(host_gaps) if host_gaps else None}")
        checks.check("P8 no command result without a command", not results, str(len(results)))
    finally:
        central.disconnect()


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--battery-present", action="store_true",
                        help="BLE-only #309 battery pass (battery-bench image, LiPo only, USB unplugged)")
    parser.add_argument("--duration-s", type=float, default=30.0, help="--battery-present: seconds of telemetry")
    parser.add_argument("--port", help="robot USB serial port (full run only)")
    parser.add_argument("--robot-id", type=int, required=True)
    parser.add_argument("--max-validity-ms", type=int, help="provisioned maxCommandValidityMs (full run only)")
    parser.add_argument("--cycles", type=int, default=20)
    parser.add_argument("--burst", type=int, default=50)
    parser.add_argument("--flood", type=int, default=500, help="write-without-response Stops in the C12 flood")
    parser.add_argument("--serial-pause-s", type=float, default=30.0, help="C13: seconds without reading serial")
    parser.add_argument("--log", default="gatt-conformance.log")
    args = parser.parse_args()
    if args.battery_present:
        if args.port:
            parser.error("--battery-present is BLE-only: unplug USB (it powers the battery node) and omit --port")
        if args.duration_s < 5:
            parser.error("--duration-s must be at least 5")
        log = Log(args.log)
        checks = Checks(log)
        run_battery_present(args, log, checks)
        log.write("host", f"SUMMARY passed={checks.passed} failed={len(checks.failed)} {' | '.join(checks.failed)}")
        return 1 if checks.failed else 0
    if args.port is None or args.max_validity_ms is None:
        parser.error("the full run needs --port and --max-validity-ms")
    if args.max_validity_ms >= VECTOR_VELOCITY_258_VALID_FOR_MS:
        parser.error(f"--max-validity-ms must be below {VECTOR_VELOCITY_258_VALID_FOR_MS}: with a higher "
                     "ceiling the robot would accept the nonzero shared vector velocity_seq258")
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
