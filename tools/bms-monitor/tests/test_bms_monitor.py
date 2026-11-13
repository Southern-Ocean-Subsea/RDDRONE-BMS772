#!/usr/bin/env python3
"""Tests for bms_monitor.py. Run:  python3 -m unittest discover -s tests

vectors.json holds messages serialised by the firmware's own Nunavut headers
(see gen_vectors.c), so the decoder is checked against the real wire format.
"""

import json
import math
import os
import socket
import struct
import sys
import tempfile
import threading
import time
import unittest
from unittest import mock
import urllib.error
import urllib.request

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

import bms_monitor as bm  # noqa: E402

with open(os.path.join(HERE, "vectors.json")) as handle:
    VECTORS = json.load(handle)


def payload(name):
    return bytes.fromhex(VECTORS[name]["payload"])


class FirmwareVectors(unittest.TestCase):
    def check(self, name, decoder):
        decoded = decoder(payload(name))
        self.assertIsNotNone(decoded, name)
        for key, expected in VECTORS[name]["expect"].items():
            if isinstance(expected, float):
                self.assertAlmostEqual(decoded[key], expected, places=3, msg=key)
            elif isinstance(expected, list):
                self.assertEqual(len(decoded[key]), len(expected), key)
                for got, want in zip(decoded[key], expected):
                    self.assertAlmostEqual(got, want, places=4, msg=key)
            else:
                self.assertEqual(decoded[key], expected, key)
        return decoded

    def test_battery_info(self):
        decoded = self.check("info", bm.decode_battery_info)
        self.assertEqual(decoded["statusFlagNames"], ["In use", "Temperature high", "BMS error"])
        self.assertEqual(bm.encode_battery_info(decoded), payload("info"))

    def test_status(self):
        decoded = self.check("status", bm.decode_status)
        self.assertEqual(decoded["readinessName"], "Engaged")
        self.assertEqual(bm.decode_status(bm.encode_status(decoded))["cellVoltages"], decoded["cellVoltages"])

    def test_source(self):
        decoded = self.check("source", bm.decode_source)
        self.assertEqual(bm.encode_source(decoded), payload("source"))

    def test_parameters(self):
        decoded = self.check("params", bm.decode_parameters)
        self.assertEqual(decoded["technologyName"], "LiPo (LCO pouch)")
        self.assertEqual(decoded["uniqueIdText"], "1234605616436508552")
        self.assertEqual(bm.encode_parameters(decoded), payload("params"))

    def test_register_replies(self):
        for name in ("faultCodeResponse", "portResponse"):
            decoded = bm.decode_register_response(payload(name))
            self.assertEqual(decoded["tag"], VECTORS[name]["expect"]["tag"])
            self.assertEqual(decoded["values"], VECTORS[name]["expect"]["values"])
            self.assertEqual(bm.encode_register_response(decoded["tag"], decoded["values"]), payload(name))

    def test_register_read_request_matches_firmware_serialiser(self):
        self.assertEqual(bm.encode_register_request(VECTORS["faultCodeRequest"]["name"]),
                         payload("faultCodeRequest"))

    def test_wrong_sizes_are_rejected(self):
        self.assertIsNone(bm.decode_battery_info(payload("info")[:-1]))
        self.assertIsNone(bm.decode_status(payload("status") + b"\x00"))
        self.assertIsNone(bm.decode_source(payload("source")[:-1]))
        self.assertIsNone(bm.decode_parameters(payload("params")[:-1]))

    def test_messages_are_recognised_by_shape(self):
        for name in ("info", "status", "source", "params"):
            self.assertEqual(bm.identify_message(payload(name)), name)
        self.assertIsNone(bm.identify_message(bytes(7)))

    def test_fault_code_names(self):
        self.assertEqual(bm.decode_fault_code(0x00200000), ["AFE woken by SPI (CSB wake-up)"])
        self.assertEqual(bm.decode_fault_code((1 << 2) | (1 << 10)), ["Cell 3 fault", "Cell undervoltage"])
        self.assertEqual(bm.decode_fault_code(0), [])


class Transport(unittest.TestCase):
    def test_identifiers_match_a_frame_captured_from_a_real_bms(self):
        # Reset-fault request and its reply, captured on the ROV's CAN bus
        # (apama-control-system docs/bms772-can2.md, 10 September 2026).
        request = bm.service_id(435, source=124, destination=123, request=True)
        self.assertEqual(request | bm.CAN_EFF_FLAG, 0x936CFDFC)
        self.assertEqual(bm.make_frames(request, bytes.fromhex("640000"), 3)[0][1], bytes.fromhex("640000e3"))
        reply = bm.parse_can_id(0x926CFE7B)
        self.assertEqual((reply["kind"], reply["port"], reply["source"], reply["destination"]),
                         ("response", 435, 123, 124))
        transfer = bm.Reassembler().accept(0x926CFE7B, bytes.fromhex("00e3"), 0.0)
        self.assertEqual((transfer["payload"], transfer["transferId"]), (b"\x00", 3))

    def test_message_identifier_round_trip(self):
        info = bm.parse_can_id(bm.message_id(4099, 123))
        self.assertEqual((info["kind"], info["port"], info["source"], info["anonymous"]),
                         ("message", 4099, 123, False))

    def test_round_trip_all_sizes(self):
        for size in list(range(0, 40)) + [58, 63, 64, 200]:
            data = bytes((7 * i + size) & 0xFF for i in range(size))
            rx, result = bm.Reassembler(), None
            frames = bm.make_frames(bm.message_id(4097, 123), data, size)
            for can_id, frame in frames:
                self.assertLessEqual(len(frame), 8)
                self.assertIsNone(result)
                result = rx.accept(can_id, frame, 1.0)
            self.assertIsNotNone(result, size)
            self.assertEqual(result["payload"], data)
            self.assertEqual(len(frames), 1 if size <= 7 else math.ceil((size + 2) / 7))

    def test_corruption_and_loss_are_dropped(self):
        frames = bm.make_frames(bm.message_id(4099, 123), payload("info"), 5)
        rx = bm.Reassembler()
        damaged = [(i, bytes([d[0] ^ 1]) + d[1:]) if n == 1 else (i, d) for n, (i, d) in enumerate(frames)]
        self.assertTrue(all(rx.accept(i, d, 0.0) is None for i, d in damaged))
        self.assertEqual(rx.crc_errors, 1)
        rx = bm.Reassembler()
        self.assertTrue(all(rx.accept(i, d, 0.0) is None for i, d in frames[:2] + frames[3:]))
        rx = bm.Reassembler()
        self.assertTrue(all(rx.accept(i, d, 3.0 * n) is None for n, (i, d) in enumerate(frames)))  # too slow

    def test_dronecan_v0_frames_are_ignored(self):
        # v0 starts its toggle at 0; a single-frame v0 transfer has tail 0b110xxxxx.
        self.assertIsNone(bm.Reassembler().accept(bm.message_id(4099, 123), b"\x01\x02\xc5", 0.0))

    def test_interleaved_subjects_do_not_mix(self):
        a = bm.make_frames(bm.message_id(4097, 123), payload("status"), 1)
        b = bm.make_frames(bm.message_id(4099, 123), payload("info"), 2)
        rx, done = bm.Reassembler(), []
        for pair in zip(a, b):
            for can_id, frame in pair:
                result = rx.accept(can_id, frame, 0.0)
                if result:
                    done.append(result["payload"])
        self.assertEqual(done, [payload("status"), payload("info")])


def run(monitor, bms, seconds, start=1000.0, step=0.02, feed=True):
    """Drive the monitor against the simulated BMS with a fake clock. Returns frames the monitor sent."""
    sent, now = [], start
    for _ in range(int(seconds / step)):
        now += step
        if feed:
            for can_id, data in bms.poll(now):
                monitor.on_frame(can_id, data, now)
        for can_id, data in monitor.tick(now):
            sent.append((can_id, data))
            bms.on_frame(can_id, data, now)
    return sent, now


class MonitorAgainstSimulatedBms(unittest.TestCase):
    def test_everything_arrives(self):
        monitor, bms = bm.Monitor(node_id=126), bm.SimulatedBms(start=1000.0)
        bms.fault_code = (1 << 2) | (1 << 10)
        sent, now = run(monitor, bms, 12)
        state = monitor.snapshot(now=now)
        json.dumps(state, allow_nan=False)
        self.assertEqual(len(state["nodes"]), 1)
        node = state["nodes"][0]
        self.assertEqual(node["nodeId"], 123)
        self.assertTrue(node["online"])
        self.assertEqual(sorted(node["messages"]), ["info", "params", "source", "status"])
        summary = node["summary"]
        self.assertEqual(len(summary["cells"]), 4)
        self.assertAlmostEqual(summary["packVoltage"], sum(summary["cells"]), places=6)
        self.assertEqual(summary["phase"], "discharging")
        self.assertTrue(summary["outputOn"])
        self.assertEqual(summary["modelName"], "SOSUB 4S demo")
        self.assertEqual(summary["cycleCount"], 37)
        self.assertAlmostEqual(summary["cellLimitHigh"], 4.2, places=3)
        self.assertEqual(node["faults"]["faultCode"], (1 << 2) | (1 << 10))
        self.assertEqual(node["faults"]["faultCodeNames"], ["Cell 3 fault", "Cell undervoltage"])
        self.assertEqual(node["faults"]["faultCodeCells"], [3])
        self.assertTrue(node["subjectsFromRegisters"])
        self.assertEqual(node["subjects"], {"4096": "source", "4097": "status", "4098": "params", "4099": "info"})
        self.assertEqual(node["registerTimeouts"], 0)
        self.assertGreaterEqual(len(node["history"]), 10)
        self.assertTrue(any("Last fault code stored" in e["text"] for e in state["events"]))
        self.assertGreater(len(sent), 0)

    def test_it_only_ever_sends_heartbeats_and_register_reads(self):
        monitor, bms = bm.Monitor(node_id=126), bm.SimulatedBms(start=1000.0)
        sent, _now = run(monitor, bms, 40)
        rx, kinds = bm.Reassembler(), set()
        for can_id, data in sent:
            info = bm.parse_can_id(can_id)
            self.assertEqual(info["source"], 126)
            transfer = rx.accept(can_id, data, 0.0)
            if not transfer:
                continue
            if transfer["kind"] == "message":
                self.assertEqual(transfer["port"], bm.HEARTBEAT_SUBJECT)
                kinds.add("heartbeat")
            else:
                self.assertEqual((transfer["kind"], transfer["port"], transfer["destination"]),
                                 ("request", bm.REGISTER_ACCESS_SERVICE, 123))
                request = bm.decode_register_request(transfer["payload"])
                self.assertEqual(request["tag"], bm.VALUE_TAG_EMPTY, "a non-empty value would write the register")
                self.assertEqual(len(transfer["payload"]), len(request["name"]) + 2)
                kinds.add(request["name"])
        self.assertEqual(kinds, {"heartbeat", bm.FAULT_CODE_REGISTER} | set(bm.PORT_REGISTERS.values()))

    def test_listen_only_sends_nothing(self):
        monitor, bms = bm.Monitor(listen_only=True), bm.SimulatedBms(start=1000.0)
        sent, now = run(monitor, bms, 8)
        self.assertEqual(sent, [])
        node = monitor.snapshot(now=now)["nodes"][0]
        self.assertTrue(node["online"])
        self.assertIsNone(node["faults"]["faultCode"])
        self.assertFalse(node["faults"]["faultCodeAvailable"])

    def test_nothing_is_sent_to_a_silent_bus(self):
        monitor, bms = bm.Monitor(), bm.SimulatedBms(start=1000.0)
        sent, _now = run(monitor, bms, 5, feed=False)
        self.assertEqual(sent, [])

    def test_bms_going_quiet(self):
        monitor, bms = bm.Monitor(), bm.SimulatedBms(start=1000.0)
        _sent, now = run(monitor, bms, 5)
        sent, now = run(monitor, bms, 8, start=now, feed=False)
        state = monitor.snapshot(now=now)
        node = state["nodes"][0]
        self.assertFalse(node["online"])
        self.assertIsNone(node["summary"]["stateOfChargePct"])
        self.assertEqual(node["summary"]["cells"], [])
        self.assertTrue(any("stopped sending" in e["text"] for e in state["events"]))
        self.assertLess(len(sent), 40)            # a few seconds of heartbeats at most, then silence
        self.assertEqual(monitor.tick(now + 1), [])

    def test_custom_subject_ids_and_a_disabled_message(self):
        monitor, bms = bm.Monitor(), bm.SimulatedBms(start=1000.0)
        bms.subjects = {"source": 5000, "status": 5001, "params": 5002, "info": bm.SUBJECT_UNSET}
        _sent, now = run(monitor, bms, 12)
        node = monitor.snapshot(now=now)["nodes"][0]
        self.assertEqual(sorted(node["messages"]), ["params", "source", "status"])
        self.assertEqual(node["subjects"], {"5000": "source", "5001": "status", "5002": "params"})
        self.assertEqual(node["disabledMessages"], ["info"])
        self.assertEqual(len(node["summary"]["cells"]), 4)
        self.assertIsNotNone(node["summary"]["current"])      # falls back to the energy-source message
        self.assertIsNotNone(node["summary"]["remainingWh"])

    def test_node_id_clash_stops_transmission(self):
        monitor, bms = bm.Monitor(node_id=126), bm.SimulatedBms(start=1000.0)
        _sent, now = run(monitor, bms, 4)
        monitor.on_frame(bm.message_id(bm.HEARTBEAT_SUBJECT, 126), bm.encode_heartbeat(5) + b"\xe0", now)
        sent, now = run(monitor, bms, 5, start=now)
        self.assertEqual(sent, [])
        state = monitor.snapshot(now=now)
        self.assertTrue(state["app"]["transmitPaused"])
        self.assertTrue(any("node ID 126" in e["text"] for e in state["events"]))
        sent, _now = run(monitor, bms, 8, start=now + 10)
        self.assertGreater(len(sent), 0)

    def test_unanswered_requests_time_out_and_carry_on(self):
        monitor, bms = bm.Monitor(), bm.SimulatedBms(start=1000.0)
        bms.on_frame = lambda *args: None                    # a BMS that never answers
        _sent, now = run(monitor, bms, 8)
        node = monitor.snapshot(now=now)["nodes"][0]
        self.assertGreaterEqual(node["registerTimeouts"], 3)
        self.assertIsNone(node["faults"]["faultCode"])
        self.assertTrue(node["online"])

    def test_fault_flag_and_fault_code_changes_are_logged(self):
        monitor, bms = bm.Monitor(), bm.SimulatedBms(start=1000.0)
        _sent, now = run(monitor, bms, 6)
        bms.flags |= 32
        bms.fault_code = 1 << 18
        _sent, now = run(monitor, bms, 6, start=now)
        texts = [e["text"] for e in monitor.snapshot(now=now)["events"]]
        self.assertTrue(any(t == "Fault flag set: Overload" for t in texts), texts)
        self.assertTrue(any("new fault code" in t and "Peak overcurrent" in t for t in texts), texts)
        node = monitor.snapshot(now=now)["nodes"][0]
        self.assertIn("Overload", node["faults"]["activeFaults"])

    def test_history_is_incremental(self):
        monitor, bms = bm.Monitor(), bm.SimulatedBms(start=1000.0)
        _sent, now = run(monitor, bms, 6)
        first = monitor.snapshot(now=now)["nodes"][0]["history"]
        _sent, now = run(monitor, bms, 3, start=now)
        later = monitor.snapshot(since_ms=first[-1][0], now=now)["nodes"][0]["history"]
        self.assertTrue(2 <= len(later) <= 4, len(later))
        self.assertTrue(all(sample[0] > first[-1][0] for sample in later))

    def test_not_a_number_never_reaches_the_page(self):
        monitor = bm.Monitor()
        bad = bytearray(payload("info"))
        bad[2:4] = struct.pack("<e", float("nan"))
        for can_id, data in bm.make_frames(bm.message_id(4099, 123), bytes(bad), 0):
            monitor.on_frame(can_id, data, 50.0)
        state = monitor.snapshot(now=50.5)
        json.dumps(state, allow_nan=False)
        self.assertIsNone(state["nodes"][0]["messages"]["info"]["data"]["voltage"])

    def test_other_traffic_is_listed_not_decoded(self):
        monitor = bm.Monitor()
        for can_id, data in bm.make_frames(bm.message_id(bm.HEARTBEAT_SUBJECT, 1), bm.encode_heartbeat(77), 0):
            monitor.on_frame(can_id, data, 10.0)
        state = monitor.snapshot(now=10.1)
        self.assertEqual(state["nodes"], [])
        self.assertEqual((state["other"][0]["source"], state["other"][0]["heartbeat"]["uptimeS"]), (1, 77))


class AdapterDiscovery(unittest.TestCase):
    def test_finds_can_interfaces_in_sysfs(self):
        with tempfile.TemporaryDirectory() as root:
            for name, kind, flags in (("can0", "280", "0x40081"), ("eth0", "1", "0x1003"), ("can1", "280", "0x40080")):
                os.makedirs(os.path.join(root, name, "statistics"))
                for leaf, text in (("type", kind), ("flags", flags), ("statistics/rx_packets", "12")):
                    with open(os.path.join(root, name, leaf), "w") as handle:
                        handle.write(text + "\n")
            self.assertEqual(bm.list_can_interfaces(root), ["can0", "can1"])
            up, down = bm.interface_info("can0", root), bm.interface_info("can1", root)
            self.assertTrue(up["up"])
            self.assertFalse(down["up"])
            self.assertEqual(up["rx_packets"], 12)

    def test_setup_command(self):
        command = bm.setup_command("can0", 1000000)
        self.assertIn("link set can0 type can bitrate 1000000", command)
        self.assertTrue(command.rstrip().endswith("link set can0 up"))
        self.assertEqual(bm.run_setup("can0; rm -rf /", 1000000, False, False)[0], False)

    def test_socketcan_frame_layout(self):
        raw = bm.CAN_FRAME.pack(0x936CFDFC, 4, bytes.fromhex("640000e3"))
        self.assertEqual(len(raw), 16)                       # sizeof(struct can_frame)
        self.assertEqual(raw[:5], bytes.fromhex("fcfd6c9304") if sys.byteorder == "little" else raw[:5])


def info(name="can0", up=True, bitrate=1000000, virtual=False):
    return {"name": name, "up": up, "bitrate": bitrate, "virtual": virtual, "driver": "gs_usb",
            "product": "USB2CAN", "canState": "ERROR-ACTIVE"}


class AdapterStateMachine(unittest.TestCase):
    """SocketCAN itself needs real hardware; these cover the decisions around it."""

    def discover(self, interfaces, setup=(False, "Administrator rights are needed"), wanted=None):
        monitor = bm.Monitor()
        bus = bm.SocketCanBus(monitor, interface=wanted)
        bus._open = mock.Mock()
        with mock.patch.object(bm, "list_can_interfaces", return_value=[i["name"] for i in interfaces]), \
                mock.patch.object(bm, "interface_info", side_effect=lambda name: next(i for i in interfaces if i["name"] == name)), \
                mock.patch.object(bm, "run_setup", return_value=setup) as run_setup:
            bus._discover()
        return monitor.adapter, bus, run_setup

    def test_no_adapter(self):
        adapter, bus, _ = self.discover([])
        self.assertEqual(adapter["state"], "no-adapter")
        bus._open.assert_not_called()

    def test_ready_adapter_is_opened(self):
        _adapter, bus, run_setup = self.discover([info()])
        bus._open.assert_called_once()
        run_setup.assert_not_called()

    def test_stopped_adapter_is_started_once_without_a_prompt(self):
        adapter, bus, run_setup = self.discover([info(up=False, bitrate=None)])
        run_setup.assert_called_once_with("can0", 1000000, False, interactive=False)
        self.assertEqual(adapter["state"], "needs-setup")
        self.assertIn("link set can0 type can bitrate 1000000", adapter["command"])
        bus._open.assert_not_called()

    def test_running_adapter_at_another_bitrate_is_left_alone(self):
        adapter, bus, run_setup = self.discover([info(bitrate=500000)])
        run_setup.assert_not_called()
        bus._open.assert_not_called()
        self.assertEqual(adapter["state"], "needs-setup")
        self.assertIn("500000", adapter["message"])

    def test_the_ready_one_is_preferred_and_a_named_one_is_respected(self):
        _adapter, bus, _ = self.discover([info("can0", up=False), info("can1")])
        self.assertEqual(bus._open.call_args[0][0]["name"], "can1")
        adapter, bus, _ = self.discover([info("can0"), info("can1")], wanted="can7")
        self.assertEqual(adapter["state"], "no-adapter")

    def test_frames_flow_through_the_socket_pump(self):
        ours, theirs = socket.socketpair(socket.AF_UNIX, socket.SOCK_DGRAM)
        ours.setblocking(False)
        theirs.setblocking(False)
        monitor = bm.Monitor(node_id=126)
        bus = bm.SocketCanBus(monitor)
        bus.sock, bus.interface, bus.info, bus.last_info_at = ours, "can0", info(), time.time()
        bms = bm.SimulatedBms(start=time.time())
        bms.fault_code = 1 << 11
        theirs.send(bm.CAN_FRAME.pack(bm.CAN_ERR_FLAG | 0x20, 8, bytes(8)))            # error frame: no ACK
        theirs.send(bm.CAN_FRAME.pack(0x123, 2, b"\x01\x02".ljust(8, b"\x00")))         # 11-bit frame: ignored
        deadline = time.time() + 2.0
        with mock.patch.object(bm, "list_can_interfaces", return_value=["can0"]), \
                mock.patch.object(bm, "interface_info", return_value=info()):
            while time.time() < deadline:
                now = time.time()
                for can_id, data in bms.poll(now):
                    theirs.send(bm.CAN_FRAME.pack(can_id | bm.CAN_EFF_FLAG, len(data), data.ljust(8, b"\x00")))
                bus._pump()
                while True:
                    try:
                        can_id, length, data = bm.CAN_FRAME.unpack(theirs.recv(16))
                    except BlockingIOError:
                        break
                    self.assertTrue(can_id & bm.CAN_EFF_FLAG)
                    bms.on_frame(can_id & bm.CAN_EFF_MASK, data[:length], now)
            state = monitor.snapshot()
            node = state["nodes"][0]
            self.assertTrue(node["online"])
            self.assertEqual(len(node["summary"]["cells"]), 4)
            self.assertEqual(node["faults"]["faultCodeNames"], ["Cell overvoltage"])
            self.assertEqual(state["busErrors"][0]["name"], "No acknowledgement")
            self.assertGreater(state["counters"]["framesSent"], 0)
            # unplugging the adapter is noticed and reported
            bus.last_info_at = 0
            with mock.patch.object(bm, "list_can_interfaces", return_value=[]):
                bus._pump()
        self.assertIsNone(bus.sock)
        self.assertEqual(monitor.adapter["state"], "no-adapter")
        self.assertIn("unplugged", monitor.adapter["message"])
        theirs.close()


class WebServer(unittest.TestCase):
    def setUp(self):
        self.monitor = bm.Monitor(demo=True)
        self.bus = bm.DemoBus(self.monitor)
        self.server = bm.serve(self.monitor, self.bus, "127.0.0.1", 0)
        self.url = "http://127.0.0.1:%d" % self.server.server_address[1]
        threading.Thread(target=self.server.serve_forever, daemon=True).start()

    def tearDown(self):
        self.server.shutdown()
        self.server.server_close()

    def test_pages(self):
        bms = bm.SimulatedBms(start=1000.0)
        run(self.monitor, bms, 5)
        state = json.loads(urllib.request.urlopen(self.url + "/api/state").read())
        self.assertEqual(state["nodes"][0]["nodeId"], 123)
        page = urllib.request.urlopen(self.url + "/").read().decode()
        self.assertIn("BMS772 Monitor", page)
        download = urllib.request.urlopen(self.url + "/api/state?download=1")
        self.assertIn("attachment", download.headers["Content-Disposition"])
        with self.assertRaises(urllib.error.HTTPError):
            urllib.request.urlopen(self.url + "/nope")

    def test_adapter_start_needs_the_apps_own_header(self):
        request = urllib.request.Request(self.url + "/api/adapter/start", data=b"", method="POST")
        with self.assertRaises(urllib.error.HTTPError):
            urllib.request.urlopen(request)
        request.add_header("X-BMS-Monitor", "1")
        self.assertFalse(json.loads(urllib.request.urlopen(request).read())["ok"])


if __name__ == "__main__":
    unittest.main()
