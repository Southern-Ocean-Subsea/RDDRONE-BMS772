#!/usr/bin/env python3
"""BMS772 Monitor - standalone live dashboard for the RDDRONE-BMS772 battery.

Plug a SocketCAN USB-to-CAN adapter (for example the Innomaker USB2CAN) into a
Linux PC, run this file, and a browser page opens showing everything the BMS
publishes on its Cyphal/CAN interface: pack and cell voltages, current, charge,
health, temperatures, status flags and the last fault code it recorded.

Only the Python 3 standard library is used; there is nothing to pip-install.

    python3 bms_monitor.py            # real adapter, opens the dashboard
    python3 bms_monitor.py --demo     # simulated battery, no hardware needed

The decoding follows the firmware in this repository (src/uavcan.c and
src/UAVCAN/portid.c). Firmware quirks that matter are noted where handled.
"""

import argparse
import errno
import json
import math
import os
import random
import re
import select
import shutil
import socket
import struct
import subprocess
import sys
import threading
import time
import webbrowser
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

VERSION = "1.0.1"

DEFAULT_BITRATE = 1000000     # UAVCAN_BITRATE_DEFAULT in inc/BMS_data_types.h
DEFAULT_NODE_ID = 126         # Cyphal reserves 126/127 for diagnostic tools
DEFAULT_HTTP_PORT = 8772
STALE_S = 3.5                 # battery messages arrive at about 1 Hz
HISTORY_SAMPLES = 900         # 15 minutes at 1 Hz

# ---------------------------------------------------------------------------
# What the firmware publishes
# ---------------------------------------------------------------------------

# Default subject IDs (inc/BMS_data_types.h). The real values are read back
# from the BMS registers, because they can be changed from its CLI.
DEFAULT_SUBJECTS = {4096: "source", 4097: "status", 4098: "params", 4099: "info"}
SUBJECT_UNSET = 65535

HEARTBEAT_SUBJECT = 7509
REGISTER_ACCESS_SERVICE = 384

FAULT_CODE_REGISTER = "bms_fault_code"
PORT_REGISTERS = {
    "source": "uavcan.pub.energy_source.id",
    "status": "uavcan.pub.battery_status.id",
    "params": "uavcan.pub.battery_parameters.id",
    "info": "uavcan.pub.battery_info.id",
}

MESSAGE_TITLES = {
    "info": "Battery info (legacy BatteryInfo)",
    "status": "Battery status (cells)",
    "source": "Energy source",
    "params": "Battery parameters",
}

# The firmware converts Celsius to kelvin with 272.15 (src/uavcan.c), not
# 273.15. Undo it with the same constant so the value matches the BMS CLI.
FIRMWARE_KELVIN_OFFSET = 272.15

# legacy.equipment.power.BatteryInfo status_flags: (bit, key, label, kind)
STATUS_FLAGS = [
    (1, "inUse", "In use", "state"),
    (2, "charging", "Charging", "state"),
    (4, "charged", "Charged", "state"),
    (8, "tempHot", "Temperature high", "fault"),
    (16, "tempCold", "Temperature low", "fault"),
    (32, "overload", "Overload", "fault"),
    (64, "badBattery", "Bad battery", "fault"),
    (128, "needService", "Service required", "fault"),
    (256, "bmsError", "BMS error", "fault"),
    (512, "reservedA", "Reserved A", "fault"),
    (1024, "reservedB", "Reserved B", "fault"),
]
FAULT_FLAG_MASK = sum(bit for bit, _k, _l, kind in STATUS_FLAGS if kind == "fault")

# BMSFault_t in inc/batManagement.h: the value kept in the bms_fault_code register.
FAULT_CODE_BITS = [
    (0, "Cell 1 fault"), (1, "Cell 2 fault"), (2, "Cell 3 fault"),
    (3, "Cell 4 fault"), (4, "Cell 5 fault"), (5, "Cell 6 fault"),
    (6, "Sense resistor temperature"), (7, "Battery temperature"),
    (8, "Power switch temperature"), (9, "AFE temperature"),
    (10, "Cell undervoltage"), (11, "Cell overvoltage"),
    (12, "Cell overvoltage (software limit)"),
    (13, "PCB undervoltage"), (14, "PCB overvoltage"),
    (15, "Undertemperature"), (16, "Overtemperature"),
    (17, "Average overcurrent"), (18, "Peak overcurrent"),
    (19, "Sleep overcurrent"), (20, "Coulomb counter overflow"),
    (21, "AFE woken by SPI (CSB wake-up)"), (22, "Other AFE fault"),
]

TECHNOLOGY = {
    0: "Other", 10: "Li-SOCl2", 11: "Li-BCX", 12: "Li-MnO2", 20: "Zinc-air",
    21: "Aluminium-air", 30: "Zinc-carbon (NH4Cl)", 31: "Zinc-carbon (ZnCl2)",
    32: "Alkaline", 100: "Li-ion LCO", 101: "LiFePO4", 102: "Li-ion NMC",
    103: "Li-ion NCA", 104: "Li-ion LMO", 105: "Li-S", 110: "LiPo (LCO pouch)",
    111: "LiFePO4 pouch", 120: "NiMH", 121: "NiCd", 122: "NiZn", 123: "NiFe",
    130: "Lead-acid", 131: "Lead-acid sealed", 200: "Supercapacitor",
}

READINESS = {0: "Sleep", 2: "Standby", 3: "Engaged"}
HEALTH = {0: "Nominal", 1: "Advisory", 2: "Caution", 3: "Warning"}


def decode_fault_code(value):
    return [label for bit, label in FAULT_CODE_BITS if value & (1 << bit)]


def decode_status_flags(flags):
    return [label for bit, _key, label, _kind in STATUS_FLAGS if flags & bit]


# ---------------------------------------------------------------------------
# Cyphal/CAN transport (classic CAN, 29-bit identifiers)
# ---------------------------------------------------------------------------

CAN_EFF_FLAG = 0x80000000
CAN_RTR_FLAG = 0x40000000
CAN_ERR_FLAG = 0x20000000
CAN_EFF_MASK = 0x1FFFFFFF

PRIORITY_NOMINAL = 4


def crc16(data, crc=0xFFFF):
    """CRC-16/CCITT-FALSE, the Cyphal transfer CRC."""
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def message_id(subject, source, priority=PRIORITY_NOMINAL):
    return (priority << 26) | (3 << 21) | (subject << 8) | source


def service_id(service, source, destination, request, priority=PRIORITY_NOMINAL):
    return ((priority << 26) | (1 << 25) | ((1 << 24) if request else 0)
            | (service << 14) | (destination << 7) | source)


def parse_can_id(can_id):
    """Split a 29-bit Cyphal identifier, or return None if it is not one."""
    can_id &= CAN_EFF_MASK
    if can_id & (1 << 23):
        return None
    source = can_id & 0x7F
    priority = can_id >> 26
    if can_id & (1 << 25):
        return {"kind": "request" if can_id & (1 << 24) else "response",
                "port": (can_id >> 14) & 0x1FF, "source": source,
                "destination": (can_id >> 7) & 0x7F, "priority": priority,
                "anonymous": False}
    if can_id & (1 << 7):
        return None
    return {"kind": "message", "port": (can_id >> 8) & 0x1FFF, "source": source,
            "destination": None, "priority": priority,
            "anonymous": bool(can_id & (1 << 24))}


def make_frames(can_id, payload, transfer_id):
    """Split one transfer into classic CAN frames: [(can_id, data), ...]."""
    body = bytes(payload)
    if len(body) > 7:
        body += struct.pack(">H", crc16(body))
    chunks = [body[i:i + 7] for i in range(0, len(body), 7)] or [b""]
    frames = []
    toggle = True
    for index, chunk in enumerate(chunks):
        tail = transfer_id & 31
        if index == 0:
            tail |= 0x80
        if index == len(chunks) - 1:
            tail |= 0x40
        if toggle:
            tail |= 0x20
        frames.append((can_id, chunk + bytes([tail])))
        toggle = not toggle
    return frames


class Reassembler:
    """Rebuilds transfers from frames. One session per (kind, port, source, destination)."""

    TIMEOUT_S = 2.0
    MAX_PAYLOAD = 1024

    def __init__(self):
        self.sessions = {}
        self.crc_errors = 0
        self.dropped = 0

    def accept(self, can_id, data, now):
        info = parse_can_id(can_id)
        if info is None or not data or len(data) > 8:
            return None
        tail = data[-1]
        transfer_id = tail & 31
        start, end, toggle = bool(tail & 0x80), bool(tail & 0x40), bool(tail & 0x20)
        key = (info["kind"], info["port"], info["source"], info["destination"])
        if start:
            self.sessions.pop(key, None)
            # Cyphal starts the toggle at 1. DroneCAN (UAVCAN v0) starts at 0,
            # so this also keeps v0 traffic on a shared bus out of the decoder.
            if not toggle:
                return None
            if end:
                return dict(info, transferId=transfer_id, payload=bytes(data[:-1]))
            if info["anonymous"] or len(data) != 8:
                self.dropped += 1
                return None
            self.sessions[key] = {"tid": transfer_id, "toggle": False, "at": now,
                                  "buffer": bytearray(data[:-1])}
            return None
        session = self.sessions.get(key)
        if session is None:
            return None
        if (session["tid"] != transfer_id or session["toggle"] != toggle
                or now - session["at"] > self.TIMEOUT_S or now < session["at"]
                or (not end and len(data) != 8)):
            del self.sessions[key]
            self.dropped += 1
            return None
        session["buffer"] += data[:-1]
        session["toggle"] = not toggle
        if len(session["buffer"]) > self.MAX_PAYLOAD:
            del self.sessions[key]
            self.dropped += 1
            return None
        if not end:
            return None
        del self.sessions[key]
        buffer = session["buffer"]
        if len(buffer) < 3 or crc16(buffer) != 0:
            self.crc_errors += 1
            return None
        return dict(info, transferId=transfer_id, payload=bytes(buffer[:-2]))


# ---------------------------------------------------------------------------
# Message layouts (from the Nunavut headers in inc/UAVCAN)
# ---------------------------------------------------------------------------

def decode_battery_info(p):
    """legacy.equipment.power.BatteryInfo.1.0 as filled in by BatteryInfoToTransmitBuffer()."""
    if len(p) < 24 or p[23] > 31 or len(p) != 24 + p[23]:
        return None
    temperature, voltage, current, power, remaining, full, hours = struct.unpack_from("<7e", p, 0)
    packed = struct.unpack_from("<I", p, 14)[0]
    flags = packed & 0x7FF
    return {
        "temperatureC": temperature,           # raw Celsius in this firmware
        "voltage": voltage,                    # V_OUT: after the power switch
        "current": current,                    # I_BATT_AVG: + charging, - discharging
        "averagePower10sW": power,
        "remainingWh": remaining,
        "fullWh": full,
        "hoursToFull": hours,
        "statusFlags": flags,
        "statusFlagNames": decode_status_flags(flags),
        "stateOfHealthPct": (packed >> 11) & 0x7F,
        "stateOfChargePct": (packed >> 18) & 0x7F,
        "stateOfChargeStdevPct": (packed >> 25) & 0x7F,
        "batteryId": p[18],
        "modelInstanceId": struct.unpack_from("<I", p, 19)[0],
        "modelName": p[24:].decode("utf-8", "replace").rstrip("\x00"),
    }


def encode_battery_info(d):
    name = d.get("modelName", "").encode()[:31]
    packed = ((d["statusFlags"] & 0x7FF) | ((d["stateOfHealthPct"] & 0x7F) << 11)
              | ((d["stateOfChargePct"] & 0x7F) << 18)
              | ((d.get("stateOfChargeStdevPct", 0) & 0x7F) << 25))
    return (struct.pack("<7e", d["temperatureC"], d["voltage"], d["current"],
                        d["averagePower10sW"], d["remainingWh"], d["fullWh"], d["hoursToFull"])
            + struct.pack("<IBIB", packed, d.get("batteryId", 0) & 0xFF,
                          d.get("modelInstanceId", 0) & 0xFFFFFFFF, len(name)) + name)


def decode_status(p):
    """reg.drone.service.battery.Status.0.2: 8 reserved bytes sit before available_charge."""
    if len(p) < 24 or len(p) != 24 + 2 * p[23]:
        return None
    count = p[23]
    t_min, t_max = struct.unpack_from("<2f", p, 2)
    charge = struct.unpack_from("<f", p, 18)[0]
    return {
        "readiness": p[0] & 3,
        "readinessName": READINESS.get(p[0] & 3, "Unknown"),
        "health": p[1] & 3,
        "healthName": HEALTH.get(p[1] & 3, "Unknown"),
        "temperatureMinC": t_min - FIRMWARE_KELVIN_OFFSET,
        "temperatureMaxC": t_max - FIRMWARE_KELVIN_OFFSET,
        "availableChargeC": charge,
        "availableChargeAh": charge / 3600.0,
        "error": p[22],
        "cellVoltages": list(struct.unpack_from("<%de" % count, p, 24)),
    }


def encode_status(d):
    cells = d["cellVoltages"]
    return (bytes([d.get("readiness", 3) & 3, d.get("health", 0) & 3])
            + struct.pack("<2f", d["temperatureMinC"] + FIRMWARE_KELVIN_OFFSET,
                          d["temperatureMaxC"] + FIRMWARE_KELVIN_OFFSET)
            + bytes(8) + struct.pack("<f", d["availableChargeC"])
            + bytes([d.get("error", 0), len(cells)])
            + struct.pack("<%de" % len(cells), *cells))


def decode_source(p):
    """reg.drone.physics.electricity.SourceTs.0.1"""
    if len(p) != 23:
        return None
    current, voltage, energy, full = struct.unpack_from("<4f", p, 7)
    return {
        "timestampUs": int.from_bytes(p[0:7], "little"),   # time since the BMS booted
        "current": current,
        "voltage": voltage,
        "energyJ": energy,
        "fullEnergyJ": full,
        "energyWh": energy / 3600.0,
        "fullEnergyWh": full / 3600.0,
    }


def encode_source(d):
    return (int(d["timestampUs"]).to_bytes(7, "little")
            + struct.pack("<4f", d["current"], d["voltage"], d["energyJ"], d["fullEnergyJ"]))


def decode_parameters(p):
    """reg.drone.service.battery.Parameters.0.3 as filled in by BatteryParametersToTransmitBuffer()."""
    if len(p) != 58:
        return None
    (unique_id, mass, capacity, v_min, v_max, discharge, burst, charge, fast,
     termination, charge_voltage, cycles) = struct.unpack_from("<Q10fH", p, 0)
    technology = p[53]
    return {
        "uniqueId": unique_id,                          # MODEL_ID
        "uniqueIdText": str(unique_id),                 # 64-bit: too wide for a JavaScript number
        "massKg": mass,
        "designCapacityC": capacity,
        "designCapacityAh": capacity / 3600.0,
        "cellVoltageMin": v_min,                        # V_CELL_UV
        "cellVoltageMax": v_max,                        # V_CELL_OV
        "dischargeCurrent": discharge,
        "dischargeCurrentBurst": burst,
        "chargeCurrent": charge,
        "chargeCurrentFast": fast,
        "chargeTerminationThreshold": termination,
        "chargeVoltage": charge_voltage,                # per cell (V_CELL_OV)
        "cycleCount": cycles,
        "stateOfHealthPct": p[52] & 0x7F,
        "technology": technology,
        "technologyName": TECHNOLOGY.get(technology, "Unknown (%d)" % technology),
        "nominalVoltage": struct.unpack_from("<f", p, 54)[0],
    }


def encode_parameters(d):
    return (struct.pack("<Q10fH", d["uniqueId"], d["massKg"], d["designCapacityC"],
                        d["cellVoltageMin"], d["cellVoltageMax"], d["dischargeCurrent"],
                        d["dischargeCurrentBurst"], d["chargeCurrent"], d["chargeCurrentFast"],
                        d["chargeTerminationThreshold"], d["chargeVoltage"], d["cycleCount"])
            + bytes(2) + bytes([d["stateOfHealthPct"] & 0x7F, d["technology"] & 0xFF])
            + struct.pack("<f", d["nominalVoltage"]))


def decode_heartbeat(p):
    """uavcan.node.Heartbeat.1.0"""
    if len(p) != 7:
        return None
    return {"uptimeS": struct.unpack_from("<I", p, 0)[0], "health": p[4] & 3,
            "mode": p[5] & 7, "vendorStatus": p[6]}


def encode_heartbeat(uptime_s):
    return struct.pack("<IBBB", int(uptime_s) & 0xFFFFFFFF, 0, 0, 0)


VALUE_TAG_EMPTY, VALUE_TAG_NATURAL32, VALUE_TAG_NATURAL16 = 0, 9, 10


def encode_register_request(name):
    """uavcan.register.Access.Request with an empty value: a read, never a write."""
    raw = name.encode()
    return bytes([len(raw)]) + raw + bytes([VALUE_TAG_EMPTY])


def decode_register_request(p):
    if len(p) < 2 or len(p) < 1 + p[0] + 1:
        return None
    return {"name": p[1:1 + p[0]].decode("utf-8", "replace"), "tag": p[1 + p[0]]}


def decode_register_response(p):
    """uavcan.register.Access.Response: returns {'tag', 'values'}; values is None if not numeric."""
    if len(p) < 9:
        return None
    tag = p[8]
    result = {"tag": tag, "mutable": bool(p[7] & 1), "persistent": bool(p[7] & 2), "values": None}
    width = {VALUE_TAG_NATURAL32: ("I", 4), VALUE_TAG_NATURAL16: ("H", 2)}.get(tag)
    if width and len(p) >= 10:
        count = p[9]
        if len(p) >= 10 + count * width[1]:
            result["values"] = list(struct.unpack_from("<%d%s" % (count, width[0]), p, 10))
    return result


def encode_register_response(tag, values):
    body = bytes(7) + bytes([0, tag])
    if tag == VALUE_TAG_NATURAL32:
        body += bytes([len(values)]) + struct.pack("<%dI" % len(values), *values)
    elif tag == VALUE_TAG_NATURAL16:
        body += bytes([len(values)]) + struct.pack("<%dH" % len(values), *values)
    return body


DECODERS = {"info": decode_battery_info, "status": decode_status,
            "source": decode_source, "params": decode_parameters}


def identify_message(payload):
    """Recognise a battery message on a subject ID we were not told about.

    Only used when the BMS was configured with non-default subject IDs and the
    registers could not be read (listen-only mode). The four layouts are
    distinct enough to tell apart by size and reserved bytes.
    """
    size = len(payload)
    if size == 23:
        return "source"
    if size == 58:
        return "params"
    if size >= 24:
        reserved_zero = not any(payload[10:18])
        if (size == 24 + 2 * payload[23] and 1 <= payload[23] <= 6 and reserved_zero
                and payload[0] <= 3 and payload[1] <= 3):
            return "status"
        if size == 24 + payload[23] and payload[23] <= 31 and not reserved_zero:
            return "info"
    return None


# ---------------------------------------------------------------------------
# Monitor: turns frames into the state the dashboard shows
# ---------------------------------------------------------------------------

def _clean(value):
    """Make a structure safe for JSON: NaN and infinity become null."""
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    if isinstance(value, dict):
        return {key: _clean(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_clean(item) for item in value]
    return value


def _finite(value):
    return isinstance(value, (int, float)) and math.isfinite(value)


class Node:
    """Everything known about one BMS on the bus."""

    def __init__(self, node_id, now):
        self.node_id = node_id
        self.first_seen = now
        self.last_seen = now
        self.online = False
        self.messages = {}                  # kind -> {data, at, count, period, subject, raw}
        self.subjects = dict(DEFAULT_SUBJECTS)
        self.subjects_from_registers = False
        self.registers = {}                 # name -> {value, at, tag}
        self.register_attempt = {}          # name -> time of last request
        self.register_timeouts = 0
        self.register_mismatches = 0
        self.previous_flags = None
        self.previous_fault_code = None
        self.history = deque(maxlen=HISTORY_SAMPLES)
        self.last_history_at = 0.0

    def fresh(self, kind, now, minimum=STALE_S):
        entry = self.messages.get(kind)
        if not entry:
            return None
        limit = max(minimum, 3.0 * entry["period"]) if entry["period"] else minimum
        return entry["data"] if 0 <= now - entry["at"] <= limit else None


class Monitor:
    """Transport-independent core. Feed it frames; ask it what to transmit."""

    REQUEST_TIMEOUT_S = 1.0
    FAULT_CODE_PERIOD_S = 2.0
    PORT_REGISTER_PERIOD_S = 30.0
    COLLISION_HOLD_S = 10.0

    def __init__(self, node_id=DEFAULT_NODE_ID, listen_only=False, demo=False):
        self.lock = threading.RLock()
        self.node_id = node_id
        self.listen_only = listen_only
        self.demo = demo
        self.started = time.time()
        self.reassembler = Reassembler()
        self.nodes = {}
        self.other = {}                      # unrecognised traffic, by (source, kind, port)
        self.events = deque(maxlen=300)
        self.event_serial = 0
        self.adapter = {"state": "starting", "message": "Starting"}
        self.counters = {"framesReceived": 0, "framesSent": 0, "transfers": 0,
                         "errorFrames": 0, "sendErrors": 0}
        self.bus_errors = {}
        self.pending = None                  # outstanding register request
        self.next_request_at = 0.0
        self.last_heartbeat_at = 0.0
        self.collision_until = 0.0
        self.transfer_ids = {"heartbeat": random.randrange(32), "register": random.randrange(32)}

    # -- events ------------------------------------------------------------

    def event(self, level, text, node=None, now=None):
        with self.lock:
            self.event_serial += 1
            self.events.appendleft({"id": self.event_serial, "at": now if now is not None else time.time(),
                                    "level": level, "text": text, "node": node})

    def set_adapter(self, **fields):
        with self.lock:
            previous = self.adapter.get("state")
            self.adapter = fields
            state = fields.get("state")
            if state != previous and previous != "starting":
                if state == "connected":
                    self.event("info", "Adapter %s connected" % fields.get("interface", ""))
                elif previous == "connected":
                    self.event("warning", "Adapter disconnected")

    # -- receive -----------------------------------------------------------

    def on_error_frame(self, can_id, data, now):
        classes = [(0x1, "TX timeout"), (0x2, "Lost arbitration"), (0x4, "Controller problem"),
                   (0x8, "Protocol violation"), (0x10, "Transceiver problem"),
                   (0x20, "No acknowledgement"), (0x40, "Bus off"), (0x80, "Bus error"),
                   (0x100, "Controller restarted")]
        with self.lock:
            self.counters["errorFrames"] += 1
            for bit, label in classes:
                if can_id & bit:
                    entry = self.bus_errors.setdefault(label, {"count": 0, "at": 0})
                    entry["count"] += 1
                    if bit == 0x40 and now - entry["at"] > 5:
                        self.event("error", "CAN bus off: check wiring, termination and bitrate", now=now)
                    entry["at"] = now

    def on_frame(self, can_id, data, now):
        with self.lock:
            self.counters["framesReceived"] += 1
            info = parse_can_id(can_id)
            if (info and not info["anonymous"] and info["source"] == self.node_id
                    and not self.listen_only):
                if now >= self.collision_until:
                    self.event("error", "Another device is using node ID %d. This app has stopped "
                               "transmitting; restart it with --node-id set to a free ID." % self.node_id, now=now)
                self.collision_until = now + self.COLLISION_HOLD_S
                self.pending = None
                return
            transfer = self.reassembler.accept(can_id, data, now)
            if transfer:
                self.counters["transfers"] += 1
                self._on_transfer(transfer, now)

    def _on_transfer(self, transfer, now):
        kind, port, source, payload = transfer["kind"], transfer["port"], transfer["source"], transfer["payload"]
        if kind == "message" and not transfer["anonymous"]:
            node = self.nodes.get(source)
            subjects = node.subjects if node else DEFAULT_SUBJECTS
            message = subjects.get(port)
            decoded = DECODERS[message](payload) if message else None
            if decoded is None and port != HEARTBEAT_SUBJECT and not (node and node.subjects_from_registers):
                message = identify_message(payload)
                decoded = DECODERS[message](payload) if message else None
            if decoded is not None:
                if node is None:
                    node = self.nodes[source] = Node(source, now)
                self._on_battery_message(node, message, port, decoded, payload, now)
                return
        if kind == "response" and port == REGISTER_ACCESS_SERVICE and transfer["destination"] == self.node_id:
            self._on_register_response(source, payload, now)
            return
        key = "%d/%s/%d" % (source, kind, port)
        entry = self.other.get(key)
        if entry is None:
            if len(self.other) >= 64:
                return
            entry = self.other[key] = {"source": source, "kind": kind, "port": port, "count": 0,
                                       "anonymous": transfer["anonymous"]}
        entry["count"] += 1
        entry["at"] = now
        entry["raw"] = payload[:64].hex()
        entry["size"] = len(payload)
        if kind == "message" and port == HEARTBEAT_SUBJECT:
            entry["heartbeat"] = decode_heartbeat(payload)

    def _on_battery_message(self, node, message, port, decoded, payload, now):
        entry = node.messages.get(message)
        if entry is None:
            entry = node.messages[message] = {"count": 0, "period": None, "at": now}
        else:
            gap = now - entry["at"]
            if 0 < gap < 60:
                entry["period"] = gap if entry["period"] is None else 0.7 * entry["period"] + 0.3 * gap
        entry.update(data=decoded, at=now, subject=port, raw=payload.hex())
        entry["count"] += 1
        node.last_seen = now
        if not node.online:
            node.online = True
            self.event("info", "BMS node %d is sending data" % node.node_id, node.node_id, now)
        if message == "info":
            self._track_flags(node, decoded["statusFlags"], now)

    def _track_flags(self, node, flags, now):
        previous = node.previous_flags
        node.previous_flags = flags
        if previous is None:
            names = decode_status_flags(flags & FAULT_FLAG_MASK)
            if names:
                self.event("warning", "Fault flags already set when monitoring started: " + ", ".join(names),
                           node.node_id, now)
            return
        changed = previous ^ flags
        for bit, key, label, kind in STATUS_FLAGS:
            if not changed & bit:
                continue
            on = bool(flags & bit)
            if kind == "fault":
                self.event("error" if on else "info",
                           "Fault flag %s: %s" % ("set" if on else "cleared", label), node.node_id, now)
            elif key == "charging":
                self.event("info", "Charging started" if on else "Charging stopped", node.node_id, now)
            elif key == "charged" and on:
                self.event("info", "Charge complete", node.node_id, now)

    def _on_register_response(self, source, payload, now):
        pending = self.pending
        # The firmware numbers its replies with its own counter instead of
        # echoing the request's transfer ID, so replies are matched by order:
        # one request at a time, and the value type must be the expected one.
        if pending is None or pending["node"] != source:
            return
        node = self.nodes.get(source)
        response = decode_register_response(payload)
        self.pending = None
        self.next_request_at = now + 0.1
        if node is None or response is None:
            return
        name = pending["name"]
        if response["tag"] != pending["tag"] or not response["values"]:
            node.register_mismatches += 1
            return
        value = response["values"][0]
        node.registers[name] = {"value": value, "at": now, "tag": response["tag"]}
        if name == FAULT_CODE_REGISTER:
            previous = node.previous_fault_code
            node.previous_fault_code = value
            if previous is None and value:
                self.event("warning", "Last fault code stored in the BMS: 0x%08X (%s)"
                           % (value, ", ".join(decode_fault_code(value)) or "unknown bits"), source, now)
            elif previous is not None and value != previous:
                self.event("error" if value else "info", "BMS recorded a new fault code: 0x%08X (%s)"
                           % (value, ", ".join(decode_fault_code(value)) or "none"), source, now)
        else:
            self._apply_port_registers(node)

    def _apply_port_registers(self, node):
        if not all(name in node.registers for name in PORT_REGISTERS.values()):
            return
        subjects = {}
        for message, name in PORT_REGISTERS.items():
            subject = node.registers[name]["value"]
            if subject != SUBJECT_UNSET:
                subjects[subject] = message
        node.subjects = subjects
        node.subjects_from_registers = True

    # -- transmit ----------------------------------------------------------

    def tick(self, now, transmit=True):
        """Housekeeping. Returns the frames to transmit: [(can_id, data), ...]."""
        frames = []
        with self.lock:
            online = []
            for node in self.nodes.values():
                alive = any(node.fresh(kind, now) is not None for kind in ("info", "status", "source"))
                if node.online and not alive:
                    node.online = False
                    self.event("warning", "BMS node %d stopped sending. It may be asleep, switched off "
                               "or disconnected." % node.node_id, node.node_id, now)
                if alive:
                    online.append(node)
                    if now - node.last_history_at >= 1.0:
                        node.last_history_at = now
                        node.history.append(self._history_sample(node, now))
            if self.pending and now - self.pending["at"] > self.REQUEST_TIMEOUT_S:
                node = self.nodes.get(self.pending["node"])
                if node:
                    node.register_timeouts += 1
                self.pending = None
                self.next_request_at = now + 0.5
            # Transmit only while a BMS is talking: frames sent to an empty bus
            # are never acknowledged and just fill the adapter with errors.
            if not transmit or self.listen_only or not online or now < self.collision_until:
                return frames
            if now - self.last_heartbeat_at >= 1.0:
                self.last_heartbeat_at = now
                frames += make_frames(message_id(HEARTBEAT_SUBJECT, self.node_id),
                                      encode_heartbeat(now - self.started), self._next_tid("heartbeat"))
            if self.pending is None and now >= self.next_request_at:
                request = self._next_register_request(online, now)
                if request:
                    node, name, tag = request
                    node.register_attempt[name] = now
                    self.pending = {"node": node.node_id, "name": name, "tag": tag, "at": now}
                    frames += make_frames(
                        service_id(REGISTER_ACCESS_SERVICE, self.node_id, node.node_id, True),
                        encode_register_request(name), self._next_tid("register"))
            self.counters["framesSent"] += len(frames)
        return frames

    def _next_tid(self, name):
        value = self.transfer_ids[name]
        self.transfer_ids[name] = (value + 1) & 31
        return value

    def _next_register_request(self, online, now):
        best = None
        for node in online:
            wanted = [(FAULT_CODE_REGISTER, VALUE_TAG_NATURAL32, self.FAULT_CODE_PERIOD_S)]
            wanted += [(name, VALUE_TAG_NATURAL16, self.PORT_REGISTER_PERIOD_S)
                       for name in PORT_REGISTERS.values()]
            for name, tag, period in wanted:
                overdue = now - node.register_attempt.get(name, -1e9) - period
                if overdue >= 0 and (best is None or overdue > best[0]):
                    best = (overdue, node, name, tag)
        return best[1:] if best else None

    # -- views -------------------------------------------------------------

    def _summary(self, node, now):
        info = node.fresh("info", now)
        status = node.fresh("status", now)
        source = node.fresh("source", now)
        params = node.fresh("params", now, minimum=16.0)

        def pick(*candidates):
            for value in candidates:
                if _finite(value):
                    return value
            return None

        cells = [v for v in (status["cellVoltages"] if status else []) if _finite(v)]
        pack_voltage = sum(cells) if cells else None
        output_voltage = pick(info and info["voltage"], source and source["voltage"])
        current = pick(info and info["current"], source and source["current"])
        charging_flag = bool(info and info["statusFlags"] & 2)
        if current is None:
            phase = "unknown"
        elif current > 0.05:
            phase = "charging"
        elif current < -0.05:
            phase = "discharging"
        else:
            phase = "charging" if charging_flag else "idle"
        output_on = None
        if pack_voltage and output_voltage is not None:
            # V_OUT is measured after the power switch: near zero means the switch is open.
            output_on = output_voltage > 0.5 * pack_voltage
        summary = {
            "stateOfChargePct": info["stateOfChargePct"] if info else None,
            "stateOfHealthPct": pick(info and info["stateOfHealthPct"], params and params["stateOfHealthPct"]),
            "outputVoltage": output_voltage,
            "packVoltage": pack_voltage,
            "outputOn": output_on,
            "current": current,
            "phase": phase,
            "powerW": output_voltage * current if output_voltage is not None and current is not None else None,
            "averagePower10sW": info["averagePower10sW"] if info else None,
            "batteryTemperatureC": info["temperatureC"] if info else None,
            "temperatureMinC": status["temperatureMinC"] if status else None,
            "temperatureMaxC": status["temperatureMaxC"] if status else None,
            "remainingWh": pick(info and info["remainingWh"], source and source["energyWh"]),
            "fullWh": pick(info and info["fullWh"], source and source["fullEnergyWh"]),
            "remainingAh": status["availableChargeAh"] if status else None,
            "designCapacityAh": params["designCapacityAh"] if params else None,
            "hoursToFull": info["hoursToFull"] if info else None,
            "cycleCount": params["cycleCount"] if params else None,
            "cells": cells,
            "cellMin": min(cells) if cells else None,
            "cellMax": max(cells) if cells else None,
            "cellDelta": max(cells) - min(cells) if cells else None,
            "cellAverage": sum(cells) / len(cells) if cells else None,
            "cellLimitLow": params["cellVoltageMin"] if params else None,
            "cellLimitHigh": params["cellVoltageMax"] if params else None,
            "modelName": info["modelName"] if info else None,
            "bmsUptimeS": source["timestampUs"] / 1e6 if source else None,
        }
        return summary

    def _history_sample(self, node, now):
        s = self._summary(node, now)
        return [int(now * 1000), s["packVoltage"], s["outputVoltage"], s["current"],
                s["stateOfChargePct"], s["batteryTemperatureC"], s["cellMin"], s["cellMax"]]

    def _faults(self, node, now):
        info = node.fresh("info", now)
        flags = info["statusFlags"] if info else None
        register = node.registers.get(FAULT_CODE_REGISTER)
        code = register["value"] if register else None
        return {
            "statusFlags": flags,
            "flags": [{"key": key, "label": label, "kind": kind,
                       "active": bool(flags & bit) if flags is not None else None}
                      for bit, key, label, kind in STATUS_FLAGS if bit < 512 or (flags or 0) & bit],
            "activeFaults": decode_status_flags(flags & FAULT_FLAG_MASK) if flags is not None else [],
            "faultCode": code,
            "faultCodeHex": "0x%08X" % code if code is not None else None,
            "faultCodeNames": decode_fault_code(code) if code is not None else [],
            "faultCodeCells": [bit + 1 for bit in range(6) if code is not None and code & (1 << bit)],
            "faultCodeAgeS": now - register["at"] if register else None,
            "faultCodeAvailable": not self.listen_only,
        }

    def snapshot(self, since_ms=0, now=None):
        now = time.time() if now is None else now
        with self.lock:
            nodes = []
            for node in sorted(self.nodes.values(), key=lambda n: n.node_id):
                messages = {}
                for kind, entry in node.messages.items():
                    messages[kind] = {"title": MESSAGE_TITLES[kind], "data": entry["data"],
                                      "ageS": now - entry["at"], "count": entry["count"],
                                      "periodS": entry["period"], "subject": entry["subject"],
                                      "raw": entry["raw"]}
                registers = [{"name": name, "value": reg["value"], "ageS": now - reg["at"]}
                             for name, reg in sorted(node.registers.items())]
                nodes.append({
                    "nodeId": node.node_id,
                    "online": node.online,
                    "lastSeenAgeS": now - node.last_seen,
                    "summary": self._summary(node, now),
                    "faults": self._faults(node, now),
                    "messages": messages,
                    "registers": registers,
                    "registerTimeouts": node.register_timeouts,
                    "registerMismatches": node.register_mismatches,
                    "subjects": {str(subject): kind for subject, kind in sorted(node.subjects.items())},
                    "subjectsFromRegisters": node.subjects_from_registers,
                    "disabledMessages": [kind for kind in MESSAGE_TITLES
                                         if node.subjects_from_registers and kind not in node.subjects.values()],
                    "history": [sample for sample in node.history if sample[0] > since_ms],
                })
            other = [dict(entry, ageS=now - entry["at"]) for entry in self.other.values()]
            for entry in other:
                del entry["at"]
            state = {
                "app": {"version": VERSION, "now": now, "uptimeS": now - self.started,
                        "nodeId": self.node_id, "listenOnly": self.listen_only, "demo": self.demo,
                        "transmitPaused": now < self.collision_until},
                "adapter": dict(self.adapter),
                "counters": dict(self.counters, crcErrors=self.reassembler.crc_errors,
                                 droppedTransfers=self.reassembler.dropped),
                "busErrors": [{"name": name, "count": entry["count"], "ageS": now - entry["at"]}
                              for name, entry in sorted(self.bus_errors.items())],
                "nodes": nodes,
                "events": list(self.events),
                "other": sorted(other, key=lambda e: (e["source"], e["port"])),
            }
        return _clean(state)


# ---------------------------------------------------------------------------
# Simulated BMS (for --demo and the tests). Mirrors the firmware's behaviour.
# ---------------------------------------------------------------------------

class SimulatedBms:
    def __init__(self, node_id=123, cells=4, start=0.0):
        self.node_id = node_id
        self.cells = cells
        self.start = start
        self.message_tid = 0          # the firmware shares one counter across all subjects
        self.response_tid = 0         # and uses its own counter for service replies
        self.reassembler = Reassembler()
        self.subjects = {"source": 4096, "status": 4097, "params": 4098, "info": 4099}
        self.fault_code = 0
        self.flags = 0
        self.soc = 78.0
        self.next_fast = start
        self.next_slow = start
        self.ticks = 0
        self.inbox = []

    def values(self, now):
        t = now - self.start
        charging = int(t // 90) % 2 == 1
        current = 4.2 if charging else -(6.0 + 4.0 * math.sin(t / 7.0) + 1.5 * math.sin(t / 1.3))
        self.soc = max(0.0, min(100.0, self.soc + current * 0.004))
        base = 3.30 + 0.0085 * self.soc + (0.03 if charging else -0.02)
        cells = [round(base + 0.012 * math.sin(t / 11.0 + i * 1.7) - (0.035 if i == 2 else 0), 3)
                 for i in range(self.cells)]
        temperature = 24.0 + 6.0 * (1 - math.exp(-t / 120.0)) + 0.4 * math.sin(t / 9.0)
        if 40 <= t % 180 < 70:
            temperature += 34.0
            self.flags |= 8 | 256             # latched, as in the firmware
            self.fault_code = (1 << 7) | (1 << 16)
        flags = self.flags | (2 if charging else 0) | (1 if abs(current) > 0.05 else 0)
        return {"t": t, "charging": charging, "current": current, "cells": cells,
                "temperature": temperature, "flags": flags}

    def poll(self, now):
        """Frames the BMS transmits up to `now`: [(can_id, data), ...]."""
        frames = []
        while self.inbox:
            frames += self.inbox.pop(0)
        if now < self.next_fast:
            return frames
        self.next_fast = max(self.next_fast + 1.0, now - 1.0)
        v = self.values(now)
        pack = sum(v["cells"])
        full_wh, design_ah = 162.0, 12.0
        remaining_wh = full_wh * self.soc / 100.0
        frames += self._publish("source", encode_source({
            "timestampUs": int(v["t"] * 1e6), "current": v["current"], "voltage": pack - 0.05,
            "energyJ": remaining_wh * 3600, "fullEnergyJ": full_wh * 3600}))
        frames += self._publish("status", encode_status({
            "temperatureMinC": v["temperature"] - 2.5, "temperatureMaxC": v["temperature"] + 6.0,
            "availableChargeC": design_ah * 0.94 * self.soc / 100.0 * 3600, "cellVoltages": v["cells"]}))
        frames += self._publish("info", encode_battery_info({
            "temperatureC": v["temperature"], "voltage": pack - 0.05, "current": v["current"],
            "averagePower10sW": pack * v["current"], "remainingWh": remaining_wh, "fullWh": full_wh,
            "hoursToFull": (full_wh - remaining_wh) / (pack * 4.2) if v["charging"] else 0.0,
            "statusFlags": v["flags"], "stateOfHealthPct": 94, "stateOfChargePct": int(self.soc),
            "batteryId": 1, "modelInstanceId": 772, "modelName": "SOSUB 4S demo"}))
        self.ticks += 1
        if self.ticks % 5 == 1:
            frames += self._publish("params", encode_parameters({
                "uniqueId": 772, "massKg": 1.85, "designCapacityC": design_ah * 3600,
                "cellVoltageMin": 3.0, "cellVoltageMax": 4.2, "dischargeCurrent": 30.0,
                "dischargeCurrentBurst": 60.0, "chargeCurrent": 4.2, "chargeCurrentFast": 8.4,
                "chargeTerminationThreshold": 0.05, "chargeVoltage": 4.2, "cycleCount": 37,
                "stateOfHealthPct": 94, "technology": 110, "nominalVoltage": 3.7 * self.cells}))
        return frames

    def _publish(self, message, payload):
        subject = self.subjects[message]
        if subject == SUBJECT_UNSET:
            return []
        frames = make_frames(message_id(subject, self.node_id), payload, self.message_tid)
        self.message_tid = (self.message_tid + 1) & 31
        return frames

    def on_frame(self, can_id, data, now):
        """A frame sent to the BMS. Replies are queued for the next poll()."""
        transfer = self.reassembler.accept(can_id, data, now)
        if (not transfer or transfer["kind"] != "request" or transfer["destination"] != self.node_id
                or transfer["port"] != REGISTER_ACCESS_SERVICE):
            return
        request = decode_register_request(transfer["payload"])
        if request is None:
            return
        # uavcan_register_interface_access_response(): prefix-match the four
        # port registers in order; anything else falls through to the fault code.
        reply = encode_register_response(VALUE_TAG_NATURAL32, [self.fault_code])
        for message in ("source", "status", "params", "info"):
            if request["name"].startswith(PORT_REGISTERS[message]):
                reply = encode_register_response(VALUE_TAG_NATURAL16, [self.subjects[message]])
                break
        self.inbox.append(make_frames(
            service_id(REGISTER_ACCESS_SERVICE, self.node_id, transfer["source"], False),
            reply, self.response_tid))
        self.response_tid = (self.response_tid + 1) & 31


class DemoBus(threading.Thread):
    """Runs the simulated BMS against the monitor: the --demo mode."""

    def __init__(self, monitor):
        super().__init__(daemon=True)
        self.monitor = monitor
        self.stop_event = threading.Event()

    def request_setup(self):
        return {"ok": False, "message": "Demo mode has no adapter"}

    def run(self):
        bms = SimulatedBms(start=time.time())
        self.monitor.set_adapter(state="demo", message="Demo mode: simulated battery, no hardware",
                                 interface="demo")
        while not self.stop_event.wait(0.02):
            now = time.time()
            for can_id, data in bms.poll(now):
                self.monitor.on_frame(can_id, data, now)
            for can_id, data in self.monitor.tick(now):
                bms.on_frame(can_id, data, now)


# ---------------------------------------------------------------------------
# SocketCAN adapter handling
# ---------------------------------------------------------------------------

ARPHRD_CAN = 280
IFF_UP = 0x1
CAN_FRAME = struct.Struct("=IB3x8s")
INTERFACE_NAME = re.compile(r"^[A-Za-z0-9_.-]{1,15}$")


def find_ip():
    return shutil.which("ip") or next((p for p in ("/usr/sbin/ip", "/sbin/ip", "/usr/bin/ip")
                                       if os.path.exists(p)), None)


def _read(path):
    try:
        with open(path) as handle:
            return handle.read().strip()
    except OSError:
        return None


def list_can_interfaces(root="/sys/class/net"):
    names = []
    try:
        entries = sorted(os.listdir(root))
    except OSError:
        return names
    for name in entries:
        if _read(os.path.join(root, name, "type")) == str(ARPHRD_CAN):
            names.append(name)
    return names


def interface_info(name, root="/sys/class/net"):
    base = os.path.join(root, name)
    info = {"name": name, "up": False, "driver": None, "product": None, "bitrate": None,
            "canState": None, "virtual": False}
    flags = _read(os.path.join(base, "flags"))
    if flags:
        try:
            info["up"] = bool(int(flags, 16) & IFF_UP)
        except ValueError:
            pass
    device = os.path.join(base, "device")
    if os.path.exists(device):
        info["driver"] = os.path.basename(os.path.realpath(os.path.join(device, "driver"))) or None
        usb = os.path.dirname(os.path.realpath(device))
        parts = [_read(os.path.join(usb, "manufacturer")), _read(os.path.join(usb, "product"))]
        info["product"] = " ".join(part for part in parts if part) or None
    else:
        info["virtual"] = True
    for key in ("rx_packets", "tx_packets", "rx_errors", "tx_errors", "rx_dropped"):
        value = _read(os.path.join(base, "statistics", key))
        info[key] = int(value) if value and value.isdigit() else None
    ip = find_ip()
    if ip:
        try:
            output = subprocess.run([ip, "-details", "-json", "link", "show", "dev", name],
                                    capture_output=True, text=True, timeout=3).stdout
            link = json.loads(output)[0]
            data = (link.get("linkinfo") or {}).get("info_data") or {}
            info["bitrate"] = (data.get("bittiming") or {}).get("bitrate")
            info["canState"] = data.get("state")
            info["up"] = "UP" in (link.get("flags") or []) or info["up"]
        except (OSError, ValueError, IndexError, KeyError, subprocess.SubprocessError):
            pass
    return info


def setup_command(name, bitrate, virtual=False):
    """Shell command that (re)starts the interface at the bitrate.

    Taking the interface down and up also clears bus-off. Automatic bus-off
    recovery (restart-ms) is asked for where the driver supports it; gs_usb
    adapters such as the Innomaker USB2CAN refuse it ("Device doesn't support
    restart from Bus Off"), so the bitrate is then set without it.
    """
    ip = find_ip() or "ip"
    if virtual:
        return "%s link set %s up" % (ip, name)
    return ("{ip} link set {n} down; {{ {ip} link set {n} type can bitrate {b} restart-ms 100 2>/dev/null"
            " || {ip} link set {n} type can bitrate {b}; }} && {ip} link set {n} up"
            .format(ip=ip, n=name, b=bitrate))


def run_setup(name, bitrate, virtual, interactive):
    """Start the interface. Needs root: try directly, then sudo, then a graphical prompt."""
    if not INTERFACE_NAME.match(name):
        return False, "Invalid interface name"
    command = setup_command(name, bitrate, virtual)
    attempts = []
    if os.geteuid() == 0:
        attempts.append(["sh", "-c", command])
    else:
        if shutil.which("sudo"):
            attempts.append(["sudo", "-n", "sh", "-c", command])
        if interactive and shutil.which("pkexec") and (os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")):
            attempts.append(["pkexec", "sh", "-c", command])
    message = "Administrator rights are needed"
    for attempt in attempts:
        try:
            result = subprocess.run(attempt, capture_output=True, text=True, timeout=120)
        except (OSError, subprocess.SubprocessError) as error:
            message = str(error)
            continue
        if result.returncode == 0:
            return True, "Adapter started"
        message = (result.stderr or result.stdout or "Command failed").strip().splitlines()[-1]
    return False, message


class SocketCanBus(threading.Thread):
    """Finds the adapter, starts it, and pumps frames to and from the monitor."""

    def __init__(self, monitor, interface=None, bitrate=DEFAULT_BITRATE):
        super().__init__(daemon=True)
        self.monitor = monitor
        self.wanted_interface = interface
        self.bitrate = bitrate
        self.stop_event = threading.Event()
        self.sock = None
        self.interface = None
        self.info = None
        self.tried_setup = set()
        self.last_info_at = 0.0
        self.setup_lock = threading.Lock()

    # -- discovery -----------------------------------------------------------

    def _candidates(self):
        names = list_can_interfaces()
        if self.wanted_interface:
            return [name for name in names if name == self.wanted_interface]
        return names

    def _ready(self, info):
        if not info["up"] or info.get("canState") == "BUS-OFF":
            return False
        return info["virtual"] or info["bitrate"] in (None, self.bitrate)

    def _discover(self):
        if not hasattr(socket, "AF_CAN"):
            self.monitor.set_adapter(state="unsupported",
                                     message="This computer has no SocketCAN support. The app needs Linux.")
            return
        names = self._candidates()
        if not names:
            self.tried_setup.clear()
            wanted = " (%s)" % self.wanted_interface if self.wanted_interface else ""
            self.monitor.set_adapter(state="no-adapter",
                                     message="No USB-to-CAN adapter found%s. Plug it in." % wanted)
            return
        infos = [interface_info(name) for name in names]
        chosen = next((info for info in infos if self._ready(info)), None)
        if chosen is None:
            chosen = infos[0]
            # A stopped or bus-off adapter is (re)started automatically, once,
            # where that needs no password. One that is running at another
            # bitrate is left alone until the user asks: another program may
            # be using it.
            stuck = not chosen["up"] or chosen.get("canState") == "BUS-OFF"
            if stuck and chosen["name"] not in self.tried_setup:
                self.tried_setup.add(chosen["name"])
                ok, _message = run_setup(chosen["name"], self.bitrate, chosen["virtual"], interactive=False)
                if ok:
                    chosen = interface_info(chosen["name"])
        if not self._ready(chosen):
            bus_off = chosen["up"] and chosen.get("canState") == "BUS-OFF"
            if bus_off:
                message = ("Adapter %s is in bus-off: it stopped after too many CAN errors."
                           % chosen["name"])
            elif chosen["up"]:
                message = ("Adapter %s is running at %s bit/s but the BMS uses %d bit/s."
                           % (chosen["name"], chosen["bitrate"], self.bitrate))
            else:
                message = "Adapter %s found but not started yet." % chosen["name"]
            self.monitor.set_adapter(
                state="needs-setup", message=message, interface=chosen["name"], info=chosen,
                busOff=bus_off, restart=bool(chosen["up"]),
                command="sudo sh -c '%s'" % setup_command(chosen["name"], self.bitrate, chosen["virtual"]),
                canPrompt=bool(shutil.which("pkexec") and (os.environ.get("DISPLAY")
                                                           or os.environ.get("WAYLAND_DISPLAY"))))
            return
        self._open(chosen)

    def _open(self, info):
        try:
            sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
            # Also deliver error frames, so wiring and bitrate problems show up.
            sock.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_ERR_FILTER, 0x1FFFFFFF)
            sock.bind((info["name"],))
            sock.setblocking(False)
        except OSError as error:
            self.monitor.set_adapter(state="error", interface=info["name"],
                                     message="Could not open %s: %s" % (info["name"], error.strerror or error))
            return
        self.sock, self.interface, self.info = sock, info["name"], info
        self.last_info_at = time.time()
        self._publish_connected()

    def _publish_connected(self):
        self.monitor.set_adapter(state="connected", interface=self.interface, info=self.info,
                                 message="Adapter %s connected" % self.interface)

    def _close(self, reason, retry_setup=True):
        if self.sock:
            try:
                self.sock.close()
            except OSError:
                pass
        self.sock = None
        if retry_setup:
            self.tried_setup.discard(self.interface)
        self.monitor.set_adapter(state="no-adapter", message=reason)

    def request_setup(self):
        """Called from the dashboard's 'Start adapter' / 'Restart adapter' button."""
        with self.setup_lock:
            adapter = dict(self.monitor.adapter)
            if adapter.get("state") != "needs-setup":
                return {"ok": False, "message": "The adapter does not need starting"}
            info = adapter.get("info") or {}
            ok, message = run_setup(adapter["interface"], self.bitrate, info.get("virtual", False),
                                    interactive=True)
            return {"ok": ok, "message": message}

    # -- frame pump ----------------------------------------------------------

    def _pump(self):
        try:
            readable, _, _ = select.select([self.sock], [], [], 0.05)
            now = time.time()
            if readable:
                for _ in range(256):
                    try:
                        raw = self.sock.recv(CAN_FRAME.size)
                    except BlockingIOError:
                        break
                    if len(raw) < CAN_FRAME.size:
                        continue
                    can_id, length, data = CAN_FRAME.unpack(raw)
                    if can_id & CAN_ERR_FLAG:
                        self.monitor.on_error_frame(can_id, data[:length], now)
                    elif can_id & CAN_EFF_FLAG and not can_id & CAN_RTR_FLAG and length <= 8:
                        self.monitor.on_frame(can_id & CAN_EFF_MASK, data[:length], now)
            for can_id, data in self.monitor.tick(now):
                try:
                    self.sock.send(CAN_FRAME.pack(can_id | CAN_EFF_FLAG, len(data), data))
                except OSError as error:
                    if error.errno in (errno.ENETDOWN, errno.ENODEV, errno.ENXIO):
                        raise
                    with self.monitor.lock:      # full transmit queue: nothing is acknowledging
                        self.monitor.counters["sendErrors"] += 1
            if now - self.last_info_at > 2.0:
                self.last_info_at = now
                if self.interface not in list_can_interfaces():
                    raise OSError(errno.ENODEV, "unplugged")
                self.info = interface_info(self.interface)
                if not self.info["up"]:
                    raise OSError(errno.ENETDOWN, "stopped")
                if self.info.get("canState") == "BUS-OFF":
                    # Not restarted automatically here: if the wiring is at
                    # fault it would only go bus-off again. The page offers a
                    # restart button instead.
                    self.monitor.event("error", "Adapter %s went bus-off after too many CAN errors. Check "
                                       "wiring, termination and bitrate, then restart it." % self.interface, now=now)
                    self.tried_setup.add(self.interface)
                    self._close("Adapter %s went bus-off." % self.interface, retry_setup=False)
                    return
                self._publish_connected()
        except (OSError, ValueError) as error:
            self._close("Adapter %s was unplugged or stopped (%s)."
                        % (self.interface, getattr(error, "strerror", None) or error))

    def run(self):
        while not self.stop_event.is_set():
            if self.sock is None:
                self._discover()
                if self.sock is None:
                    self.monitor.tick(time.time(), transmit=False)
                    self.stop_event.wait(1.0)
            else:
                self._pump()


# ---------------------------------------------------------------------------
# Local web server
# ---------------------------------------------------------------------------

DASHBOARD = os.path.join(os.path.dirname(os.path.abspath(__file__)), "dashboard.html")


def make_handler(monitor, bus):
    class Handler(BaseHTTPRequestHandler):
        server_version = "BMSMonitor/" + VERSION

        def log_message(self, *args):
            pass

        def _send(self, status, body, content_type, extra=None):
            self.send_response(status)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            for key, value in (extra or {}).items():
                self.send_header(key, value)
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            url = urlparse(self.path)
            if url.path in ("/", "/index.html"):
                try:
                    with open(DASHBOARD, "rb") as handle:
                        self._send(200, handle.read(), "text/html; charset=utf-8")
                except OSError:
                    self._send(500, b"dashboard.html is missing next to bms_monitor.py", "text/plain")
            elif url.path == "/api/state":
                query = parse_qs(url.query)
                try:
                    since = int(query.get("since", ["0"])[0])
                except ValueError:
                    since = 0
                body = json.dumps(monitor.snapshot(since), allow_nan=False).encode()
                extra = None
                if "download" in query:
                    name = time.strftime("bms-snapshot-%Y%m%d-%H%M%S.json")
                    extra = {"Content-Disposition": 'attachment; filename="%s"' % name}
                self._send(200, body, "application/json", extra)
            else:
                self._send(404, b"Not found", "text/plain")

        def do_POST(self):
            # The custom header cannot be sent by another website without a
            # CORS preflight, which this server never answers.
            if self.path != "/api/adapter/start" or self.headers.get("X-BMS-Monitor") != "1":
                self._send(404, b"Not found", "text/plain")
                return
            self._send(200, json.dumps(bus.request_setup()).encode(), "application/json")

    return Handler


def serve(monitor, bus, host, port):
    last_error = None
    for candidate in range(port, port + 20):
        try:
            return ThreadingHTTPServer((host, candidate), make_handler(monitor, bus))
        except OSError as error:
            last_error = error
    raise last_error


def open_browser(url):
    user = os.environ.get("SUDO_USER")
    if not (os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")):
        return          # no desktop (for example over SSH): the address is printed instead
    try:
        if os.geteuid() == 0 and user and shutil.which("xdg-open"):
            # Browsers refuse to run as root: open the page as the real user.
            subprocess.Popen(["sudo", "-u", user, "xdg-open", url],
                             stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        else:
            webbrowser.open(url)
    except Exception:
        pass


def main(argv=None):
    parser = argparse.ArgumentParser(description="Live dashboard for the RDDRONE-BMS772 over a USB-to-CAN adapter.")
    parser.add_argument("--demo", action="store_true", help="show simulated data; no hardware needed")
    parser.add_argument("--interface", help="CAN interface to use (default: find it automatically)")
    parser.add_argument("--bitrate", type=int, default=DEFAULT_BITRATE,
                        help="CAN bitrate of the BMS (default: %(default)s)")
    parser.add_argument("--node-id", type=int, default=DEFAULT_NODE_ID,
                        help="Cyphal node ID this app uses on the bus (default: %(default)s)")
    parser.add_argument("--listen-only", action="store_true",
                        help="never transmit; the stored fault code cannot be read in this mode")
    parser.add_argument("--port", type=int, default=DEFAULT_HTTP_PORT, help="web page port (default: %(default)s)")
    parser.add_argument("--host", default="127.0.0.1",
                        help="address the web page listens on; use 0.0.0.0 to view from another computer")
    parser.add_argument("--no-browser", action="store_true", help="do not open the browser")
    args = parser.parse_args(argv)
    if not 1 <= args.node_id <= 127:
        parser.error("--node-id must be between 1 and 127")

    monitor = Monitor(node_id=args.node_id, listen_only=args.listen_only, demo=args.demo)
    bus = DemoBus(monitor) if args.demo else SocketCanBus(monitor, args.interface, args.bitrate)
    server = serve(monitor, bus, args.host, args.port)
    shown_host = "localhost" if args.host in ("127.0.0.1", "0.0.0.0") else args.host
    url = "http://%s:%d/" % (shown_host, server.server_address[1])
    bus.start()
    print("BMS772 Monitor %s" % VERSION)
    print("Dashboard: %s" % url)
    print("Press Ctrl+C to stop.")
    if not args.no_browser:
        threading.Timer(0.6, open_browser, [url]).start()
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        bus.stop_event.set()
        server.server_close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
