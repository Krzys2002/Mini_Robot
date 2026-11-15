#!/usr/bin/env python3
"""Host driver for the #156 timing pilot image (`pio run -e timing-pilot`).

Linux/BlueZ only. Needs pyserial and the system D-Bus bindings (python3-dbus);
it downloads nothing. Every line exchanged is written with host monotonic and
wall-clock stamps to the output log, so a run can be archived and re-read.
The pilot image has no actuator or sensor access; use it USB-only with the
battery and motors disconnected (docs/testing/timing-pilot-156.md).

Example (from Mini_Robot/firmware/codecell-c6):
  scripts/timing_pilot_host.py --port /dev/serial/by-id/usb-Espressif_* \
      "info" "ble on 20 5" "telem 100" ble-connect "ble:periodic:100:60" "ble" ble-disconnect
"""
import argparse
import os
import statistics
import struct
import sys
import threading
import time
from datetime import datetime, timezone

import serial  # pyserial

SERVICE = "8d1e0001-5c3a-4b8e-9f6a-6f9c2b0d1560"
COMMAND = "8d1e0002-5c3a-4b8e-9f6a-6f9c2b0d1560"
ESTOP = "8d1e0003-5c3a-4b8e-9f6a-6f9c2b0d1560"
RESULT = "8d1e0004-5c3a-4b8e-9f6a-6f9c2b0d1560"
TELEMETRY = "8d1e0005-5c3a-4b8e-9f6a-6f9c2b0d1560"
PILOT_NAME = "MR-PILOT-156"
VALIDITY_MS = 250          # Placeholder field value; the pilot never interprets frames.
SERIAL_COMMAND_TIMEOUT_S = 180
DBUS_TIMEOUT_S = 30
DISCOVERY_TIMEOUT_S = 20
NOTIFY_READ_BYTES = 64
RESULT_TAIL_S = 1.0
SEQUENCE_MODULUS = 1 << 16


class Log:
    def __init__(self, path):
        self.file = open(path, "a", encoding="utf-8")
        self.lock = threading.Lock()

    def write(self, source, text):
        stamp = f"{time.monotonic():.6f} {datetime.now(timezone.utc).isoformat(timespec='milliseconds')}"
        with self.lock:
            self.file.write(f"{stamp} {source} {text}\n")
            self.file.flush()
        if not text.startswith(("LOADLOG", "LOGTEST")):
            print(f"{source} {text}", flush=True)


def open_serial(port):
    """Open the pilot port. On the #156 Ubuntu host every open reset the C6."""
    device = serial.Serial()
    device.port = port
    device.baudrate = 115200
    device.timeout = 0.2
    device.dtr = False  # Deasserted DTR/RTS do not reset the USB Serial/JTAG chip.
    device.rts = False
    device.open()
    return device


def run_serial(device, log, command, timeout=SERIAL_COMMAND_TIMEOUT_S):
    """Send one pilot command and return its PILOT lines (until 'PILOT done')."""
    log.write("host>", command)
    device.write((command + "\n").encode())
    return wait_done(device, log, timeout)


def wait_done(device, log, timeout=SERIAL_COMMAND_TIMEOUT_S):
    lines, deadline = [], time.monotonic() + timeout
    while time.monotonic() < deadline:
        raw = device.readline()
        if not raw:
            continue
        text = raw.decode(errors="replace").rstrip()
        log.write("robot", text)
        if text == "PILOT done":
            return lines
        if text.startswith("PILOT"):
            lines.append(text)
    raise TimeoutError("no 'PILOT done' before the timeout")


def expect(device, log, text, timeout):
    """Read until a line contains `text` (for example the boot banner after a reset)."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        raw = device.readline()
        if raw:
            line = raw.decode(errors="replace").rstrip()
            log.write("robot", line)
            if text in line:
                return line
    raise TimeoutError(f"{text!r} not seen within {timeout} s")


class Pilot:
    """Minimal BlueZ D-Bus GATT client for the pilot service (no event loop needed)."""

    def __init__(self, adapter="hci0"):
        import dbus  # System bindings; imported here so serial-only use needs no D-Bus.
        self.dbus = dbus
        self.bus = dbus.SystemBus()
        self.adapter_path = f"/org/bluez/{adapter}"
        self.manager = dbus.Interface(self.bus.get_object("org.bluez", "/"),
                                      "org.freedesktop.DBus.ObjectManager")

    def _objects(self):
        return self.manager.GetManagedObjects()

    def connect(self):
        adapter = self.dbus.Interface(self.bus.get_object("org.bluez", self.adapter_path), "org.bluez.Adapter1")
        adapter.SetDiscoveryFilter({"UUIDs": [SERVICE], "Transport": "le"})
        adapter.StartDiscovery()
        try:
            deadline = time.monotonic() + DISCOVERY_TIMEOUT_S
            while time.monotonic() < deadline and not self._device_path():
                time.sleep(0.2)
        finally:
            adapter.StopDiscovery()
        path = self._device_path()
        if not path:
            raise RuntimeError(f"{PILOT_NAME} not found")
        self.device_path = path
        device = self.dbus.Interface(self.bus.get_object("org.bluez", path), "org.bluez.Device1")
        props = self.dbus.Interface(self.bus.get_object("org.bluez", path), "org.freedesktop.DBus.Properties")
        device.Connect(timeout=DBUS_TIMEOUT_S)
        deadline = time.monotonic() + DBUS_TIMEOUT_S
        while not props.Get("org.bluez.Device1", "ServicesResolved"):
            if time.monotonic() > deadline:
                raise RuntimeError("services not resolved")
            time.sleep(0.1)
        self.device = device
        self.chars = {}
        for object_path, interfaces in self._objects().items():
            gatt = interfaces.get("org.bluez.GattCharacteristic1")
            if gatt and object_path.startswith(path):
                self.chars[str(gatt["UUID"]).lower()] = self.dbus.Interface(
                    self.bus.get_object("org.bluez", object_path), "org.bluez.GattCharacteristic1")

    def _device_path(self):
        for object_path, interfaces in self._objects().items():
            device = interfaces.get("org.bluez.Device1")
            if device and str(device.get("Name", "")) == PILOT_NAME and object_path.startswith(self.adapter_path):
                return object_path
        return None

    def acquire_write(self, uuid):
        fd, mtu = self.chars[uuid].AcquireWrite({})
        fd = fd.take()
        os.set_blocking(fd, True)  # BlueZ returns non-blocking sockets; block on backpressure.
        return fd, int(mtu)

    def acquire_notify(self, uuid):
        fd, mtu = self.chars[uuid].AcquireNotify({})
        fd = fd.take()
        os.set_blocking(fd, True)  # One read returns one notification.
        return fd, int(mtu)

    def write_request(self, uuid, data):
        self.chars[uuid].WriteValue(self.dbus.Array(data, signature="y"), {"type": "request"},
                                    timeout=DBUS_TIMEOUT_S)

    def open_channels(self):
        """Acquire once per connection: BlueZ refuses a second AcquireNotify until release."""
        self.write_fd, self.write_mtu = self.acquire_write(COMMAND)
        result_fd, _ = self.acquire_notify(RESULT)
        telemetry_fd, _ = self.acquire_notify(TELEMETRY)
        self.results, self.telemetry, self.stop = [], [], threading.Event()
        self.fds = (self.write_fd, result_fd, telemetry_fd)
        for fd, sink in ((result_fd, self.results), (telemetry_fd, self.telemetry)):
            threading.Thread(target=reader, args=(fd, sink, self.stop), daemon=True).start()

    def disconnect(self):
        self.stop.set()
        for fd in self.fds:
            os.close(fd)
        self.device.Disconnect(timeout=DBUS_TIMEOUT_S)


def frame(message_type, sequence):
    if message_type == 2:
        return struct.pack("<BBHhhH", 1, 2, sequence & 0xFFFF, 0, 0, VALIDITY_MS)
    return struct.pack("<BBH", 1, message_type, sequence & 0xFFFF)


def summarize(name, values_ms, log):
    if not values_ms:
        log.write("host", f"SUMMARY {name} n=0")
        return
    ordered = sorted(values_ms)

    def pct(q):
        return ordered[min(len(ordered) - 1, max(0, int(q * len(ordered) + 0.999999) - 1))]
    log.write("host", f"SUMMARY {name} n={len(ordered)} min={ordered[0]:.3f} p50={pct(0.5):.3f} "
                      f"p99={pct(0.99):.3f} max={ordered[-1]:.3f} mean={statistics.fmean(ordered):.3f} unit=ms")


def reader(fd, sink, stop):
    while not stop.is_set():
        try:
            data = os.read(fd, NOTIFY_READ_BYTES)
        except OSError:
            return
        if not data:
            return
        sink.append((time.monotonic_ns(), data))


def ble_scenario(pilot, mode, period_ms, seconds, log, on_ready=None):
    """One BLE load pattern on an open pilot connection; returns after the result tail."""
    write_fd, mtu = pilot.write_fd, pilot.write_mtu
    first_result, first_telemetry = len(pilot.results), len(pilot.telemetry)
    log.write("host", f"BLE start mode={mode} period_ms={period_ms} seconds={seconds} write_mtu={mtu}")
    if on_ready:
        on_ready()  # Starts a concurrent robot-side measurement once the link is up.
    sent, sequence = {}, 0
    start = time.monotonic_ns()
    end = start + int(seconds * 1e9)
    if mode == "estop":
        request_rtt = []
        while time.monotonic_ns() < end:
            sequence += 1
            t0 = time.monotonic_ns()
            sent[sequence] = t0
            try:
                pilot.write_request(ESTOP, frame(3, sequence))
            except pilot.dbus.exceptions.DBusException as error:
                # A queued backlog ahead of this request can exceed the D-Bus timeout.
                log.write("host", f"estop seq={sequence} write_request failed after "
                                  f"{(time.monotonic_ns() - t0) / 1e6:.1f} ms: {error.get_dbus_name()}")
                continue
            request_rtt.append((time.monotonic_ns() - t0) / 1e6)
            time.sleep(period_ms / 1000)
        summarize("estop_write_request_response", request_rtt, log)
    else:
        next_send = start
        while True:
            now = time.monotonic_ns()
            if now >= end:
                break
            if mode == "periodic" and now < next_send:
                time.sleep((next_send - now) / 1e9)
                continue
            sequence += 1
            sent[sequence] = time.monotonic_ns()
            os.write(write_fd, frame(2, sequence))  # Write Without Response; blocks on backpressure.
            next_send += int(period_ms * 1e6)
    send_duration_s = (time.monotonic_ns() - start) / 1e9
    time.sleep(RESULT_TAIL_S)
    results = pilot.results[first_result:]
    telemetry = pilot.telemetry[first_telemetry:]
    rtt, unmatched = [], 0
    if len(sent) >= SEQUENCE_MODULUS // 2:
        # 16-bit sequences repeat; matching would be ambiguous, so report counts only.
        log.write("host", "RTT not computed: more frames sent than the sequence space disambiguates")
        results_for_rtt = []
    else:
        results_for_rtt = results
    for received, data in results_for_rtt:
        if len(data) < 6:
            unmatched += 1
            continue
        command_sequence = struct.unpack_from("<H", data, 4)[0]
        candidates = [s for s in sent if s % SEQUENCE_MODULUS == command_sequence]
        if not candidates:
            unmatched += 1
            continue
        rtt.append((received - sent[max(candidates)]) / 1e6)
    log.write("host", f"BLE mode={mode} period_ms={period_ms} sent={len(sent)} "
                      f"results={len(results)} unmatched={unmatched} telemetry={len(telemetry)} "
                      f"send_duration_s={send_duration_s:.3f} rate_hz={len(sent) / send_duration_s:.1f}")
    summarize("write_to_result_notification", rtt, log)
    if len(telemetry) > 1:
        gaps = [(b[0] - a[0]) / 1e6 for a, b in zip(telemetry, telemetry[1:])]
        summarize("telemetry_host_interarrival", gaps, log)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", required=True, help="pilot USB serial device (opening it resets the C6)")
    parser.add_argument("--adapter", default="hci0")
    parser.add_argument("--log", default="timing-pilot-156.log", help="append raw stamped lines here")
    parser.add_argument("steps", nargs="+", help=(
        "in order: a pilot serial command; pause:<s> (stop reading); send:<command> (no wait); "
        "wait; expect:<text>:<s>; ble-connect; ble-disconnect; ble:<periodic|flood|estop>:<period ms>:<seconds>[:<serial command>]"))
    args = parser.parse_args()
    log = Log(args.log)
    device = open_serial(args.port)  # One session: reopening would reset the robot.
    pilot = None
    for step in args.steps:
        if step.startswith("pause:"):
            log.write("host", f"pause reading {step[6:]} s")  # Host buffers fill: USB backpressure.
            time.sleep(float(step[6:]))
        elif step.startswith("send:"):
            log.write("host>", step[5:])
            device.write((step[5:] + "\n").encode())
        elif step == "wait":
            wait_done(device, log)
        elif step.startswith("expect:"):
            text, seconds = step[7:].rsplit(":", 1)
            expect(device, log, text, float(seconds))
        elif step == "ble-connect":
            pilot = Pilot(args.adapter)
            pilot.connect()
            pilot.open_channels()
            log.write("host", f"connected {pilot.device_path}")
        elif step == "ble-disconnect":
            t0 = time.monotonic_ns()
            pilot.disconnect()
            log.write("host", f"disconnect call_ms={(time.monotonic_ns() - t0) / 1e6:.3f}")
            time.sleep(RESULT_TAIL_S)
            while device.in_waiting:
                wait_line = device.readline().decode(errors="replace").rstrip()
                log.write("robot", wait_line)
        elif step.startswith("ble:"):
            mode, period, seconds, *during = step[4:].split(":", 3)
            concurrent = None
            if during:
                concurrent = threading.Thread(target=run_serial, args=(device, log, during[0]), daemon=True)
            ble_scenario(pilot, mode, float(period), float(seconds), log,
                         concurrent.start if concurrent else None)
            if concurrent:
                concurrent.join(SERIAL_COMMAND_TIMEOUT_S)
        else:
            run_serial(device, log, step)
    return 0


if __name__ == "__main__":
    sys.exit(main())
