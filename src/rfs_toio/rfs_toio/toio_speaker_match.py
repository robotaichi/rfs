#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
import asyncio
import subprocess
import re
import os
import json
from typing import List, Tuple, Dict, Optional
from toio import BLEScanner, ToioCoreCube

# Configuration file path
CONFIG_PATH = os.path.expanduser('~/rfs/src/rfs_config/config/config.json')
DEVICE_NAME_UUID = "00002a00-0000-1000-8000-00805f9b34fb"  # Generic Access: Device Name
CONNECT_TIMEOUT = 5

class SystemTTS:
    def __init__(self, node: Node):
        self.node = node
        # Detect available sinks
        try:
            lines = subprocess.run(
                ["pactl", "list", "short", "sinks"], capture_output=True, text=True, check=True
            ).stdout.splitlines()
        except (subprocess.CalledProcessError, FileNotFoundError):
            self.node.get_logger().error("Failed to run pactl command. Ensure PulseAudio is installed.")
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

        self.node.get_logger().info(f"Detected BT sinks: {bt_sinks}")
        self.node.get_logger().info(f"Detected internal sink: {internal_sink}")

        self.sinks_in_priority = bt_sinks
        if internal_sink:
            self.sinks_in_priority.append(internal_sink)
        
        self.node.get_logger().info(f"Speaker priority: {self.sinks_in_priority}")

    def speak(self, text: str, sink: str):
        try:
            self.node.get_logger().info(f"Speaking: {text} on {sink}")
            # Use spd-say for simple TTS
            env = os.environ.copy()
            if sink:
                env["PULSE_SINK"] = sink
            
            subprocess.run(["spd-say", "-e", text], env=env, check=True)
        except Exception as e:
            self.node.get_logger().error(f"Failed to speak: {e}")

class ToioSpeakerMatcher(Node):
    def __init__(self):
        super().__init__('toio_speaker_matcher')
        self.tts = SystemTTS(self)
        self.roles = self.load_roles()
        self.get_logger().info("Starting automatic Toio and Speaker pairing.")

    def load_roles(self):
        try:
            with open(CONFIG_PATH, 'r', encoding='utf-8') as f:
                cfg = json.load(f)
            return cfg.get('family_config', [])
        except (FileNotFoundError, json.JSONDecodeError) as e:
            self.get_logger().error(f"Failed to load config '{CONFIG_PATH}': {e}")
            return []

    # --- Robust ID Utilities from toio_checkID.py ---
    def _best_addr_name(self, dev) -> Tuple[Optional[str], Optional[str]]:
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

    async def _fallback_query_name_addr(self, dev) -> Tuple[Optional[str], Optional[str]]:
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

    async def run_automatic_matching(self):
        # 1. Robust Device Scan
        self.get_logger().info("Confirming Toio cube order via initial scan...")
        try:
            self.get_logger().info(f"Scanning for {len(self.roles)} Toio cubes...")
            devs = await BLEScanner.scan(num=len(self.roles))
            if not devs:
                self.get_logger().error("No Toio cubes found. Aborting.")
                return
            
            toio_info_list = []
            for d in devs:
                addr, name = self._best_addr_name(d)
                if not name or name == "UNKNOWN":
                    self.get_logger().info(f"Retrieving name for {addr} via connection...")
                    addr2, name2 = await self._fallback_query_name_addr(d)
                    addr = addr2 or addr
                    name = name2 or "UNKNOWN"
                toio_info_list.append({"address": addr, "name": name, "device": d})
                self.get_logger().info(f"Found Toio: {name} ({addr})")

            toio_addresses = [info["address"] for info in toio_info_list]
            self.get_logger().info(f"Confirmed Toio pairing order: {toio_addresses}")
        except Exception as e:
            self.get_logger().error(f"Error during initial scan: {e}")
            return
        
        available_sinks = self.tts.sinks_in_priority.copy()
        if not available_sinks:
            self.get_logger().error("No available speakers found. Aborting.")
            return

        self.get_logger().info(f"Available speakers: {available_sinks}")

        match_list = []
        num_pairs = min(len(self.roles), len(toio_info_list), len(available_sinks))

        for i in range(num_pairs):
            role = self.roles[i]
            toio_info = toio_info_list[i]
            toio_address = toio_info["address"]
            toio_name = toio_info["name"]
            sink = available_sinks[i]

            self.get_logger().info(f"--- Pairing Start: Role='{role}', Toio='{toio_name} ({toio_address})' ---")

            connected_cube = None
            for attempt in range(3):
                try:
                    self.get_logger().info(f"Connecting to Toio {toio_name} ({toio_address})... (Attempt {attempt + 1}/3)")
                    cube = ToioCoreCube(toio_info["device"].interface)
                    await cube.connect()
                    connected_cube = cube
                    self.get_logger().info(f"Successfully connected to {toio_name}.")
                    break
                except Exception as e:
                    self.get_logger().warn(f"Connection attempt failed: {e}")
                    if attempt < 2:
                        await asyncio.sleep(1)
            
            if connected_cube:
                try:
                    speech = f"Speaker {role}. Please place this speaker on the rotating Toio."
                    self.tts.speak(speech, sink)
                    await connected_cube.api.motor.motor_control(50, -50, 1000)

                    match_info = { 
                        "role": role, 
                        "toio_id": toio_address, 
                        "toio_name": toio_name,
                        "speaker_id": sink 
                    }
                    match_list.append(match_info)
                    self.get_logger().info(f"Pair confirmed: {role} <-> {toio_name} ({toio_address}) <-> {sink}")
                
                finally:
                    await connected_cube.disconnect()
                    self.get_logger().info(f"Disconnected {toio_name}.")
                    await asyncio.sleep(1)
            else:
                self.get_logger().error(f"Failed to connect to {toio_name}. Skipping this pair.")

        # 3. Save Results
        if not match_list:
            self.get_logger().error("No pairs were formed. Config file not updated.")
            return
            
        try:
            with open(CONFIG_PATH, 'r+', encoding='utf-8') as f:
                config_data = json.load(f)
                config_data['toio_speaker_match'] = match_list
                f.seek(0)
                json.dump(config_data, f, ensure_ascii=False, indent=2)
                f.truncate()
            self.get_logger().info(f"Pairing results written to {CONFIG_PATH}")
        except (FileNotFoundError, json.JSONDecodeError) as e:
            self.get_logger().error(f"Failed to update config '{CONFIG_PATH}': {e}")

        self.get_logger().info("Pairing process completed.")


def main(args=None):
    rclpy.init(args=args)
    node = ToioSpeakerMatcher()
    try:
        asyncio.run(node.run_automatic_matching())
    except KeyboardInterrupt:
        node.get_logger().info("Interrupted by user.")
    except Exception as e:
        node.get_logger().fatal(f"Unexpected error during execution: {e}")
    finally:
        node.get_logger().info("Shutting down.")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
