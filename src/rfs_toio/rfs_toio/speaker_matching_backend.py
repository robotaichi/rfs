#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Logic layer for automatic toio<->speaker pairing.

Handles announcements using the system TTS (spd-say) and getting the address/name
from BLE devices. Does not depend on ROS2 communication. Called from the
toio_speaker_match node.
"""

import asyncio
import os
import re
import subprocess
from typing import Tuple, Optional

from toio import ToioCoreCube

DEVICE_NAME_UUID = "00002a00-0000-1000-8000-00805f9b34fb"  # Generic Access: Device Name
CONNECT_TIMEOUT = 5


class SystemTTS:
    """Logic class that speaks through a detected speaker (PulseAudio sink) using spd-say."""

    def __init__(self, logger):
        self.logger = logger
        # Detect available sinks
        try:
            lines = subprocess.run(
                ["pactl", "list", "short", "sinks"], capture_output=True, text=True, check=True
            ).stdout.splitlines()
        except (subprocess.CalledProcessError, FileNotFoundError):
            self.logger.error("Failed to run pactl command. Ensure PulseAudio is installed.")
            self.device_map = {}
            self.sinks_in_priority = []
            return

        self.device_map = {}
        bt_sinks = []
        internal_sink = None

        for ln in lines:
            parts = ln.split()
            if len(parts) < 2:
                continue

            name = parts[1]

            if name.startswith(("bluez_output.", "bluez_sink.")):
                m = re.match(r"^(?:bluez_output|bluez_sink)\.([0-9A-F_]+)", name)
                if m:
                    mac = m.group(1).replace('_', ':').upper()
                    self.device_map[mac] = name
                    bt_sinks.append(name)
            elif name.startswith("alsa_output.") and ("analog-stereo" in name or "headphones" in name.lower()):
                internal_sink = name

        self.logger.info(f"Detected BT sinks: {bt_sinks}")
        self.logger.info(f"Detected internal sink: {internal_sink}")

        self.sinks_in_priority = bt_sinks
        if internal_sink:
            self.sinks_in_priority.append(internal_sink)

        self.logger.info(f"Speaker priority: {self.sinks_in_priority}")

    def speak(self, text: str, sink: str):
        try:
            self.logger.info(f"Speaking: {text} on {sink}")
            # Use spd-say for simple TTS
            env = os.environ.copy()
            if sink:
                env["PULSE_SINK"] = sink

            subprocess.run(["spd-say", "-e", text], env=env, check=True)
        except Exception as e:
            self.logger.error(f"Failed to speak: {e}")


def best_addr_name(dev) -> Tuple[Optional[str], Optional[str]]:
    """Extract the address and name, as best as possible, from a BLE scan result object (and its inner interface)."""

    def _get_addr_name_from_obj(obj) -> Tuple[Optional[str], Optional[str]]:
        addr = None
        name = None
        for a in ("address", "mac", "addr"):
            if hasattr(obj, a):
                addr = getattr(obj, a) or addr
        for n in ("name", "device_name"):
            if hasattr(obj, n):
                name = getattr(obj, n) or name
        if hasattr(obj, "__dict__"):
            d = obj.__dict__
            addr = d.get("address", addr)
            name = d.get("name", name)
        return addr, name

    addr, name = _get_addr_name_from_obj(dev.device)
    iface = getattr(dev, "interface", None)
    if iface:
        a2, n2 = _get_addr_name_from_obj(iface)
        addr = a2 or addr
        name = n2 or name
        for inner in ("device", "peripheral", "client", "_device"):
            inner_obj = getattr(iface, inner, None)
            if inner_obj:
                a3, n3 = _get_addr_name_from_obj(inner_obj)
                addr = a3 or addr
                name = n3 or name
    return addr, name


async def fallback_query_name_addr(dev) -> Tuple[Optional[str], Optional[str]]:
    """If the address/name can't be obtained from the scan result, connect and fetch it through GATT instead."""
    try:
        cube = ToioCoreCube(dev.interface)
        await asyncio.wait_for(cube.connect(), timeout=CONNECT_TIMEOUT)
        addr = None
        client = getattr(cube, "client", None)
        if client:
            addr = getattr(client, "address", None)
            if not addr:
                dev_inner = getattr(client, "_device", None)
                if dev_inner:
                    addr = getattr(dev_inner, "address", None)
        name = None
        if client:
            try:
                data = await asyncio.wait_for(client.read_gatt_char(DEVICE_NAME_UUID), timeout=CONNECT_TIMEOUT)
                if isinstance(data, (bytes, bytearray)):
                    name = data.decode("utf-8", errors="ignore").strip("\x00").strip()
            except Exception:
                pass
        await cube.disconnect()
        return addr, name
    except Exception:
        return None, None
