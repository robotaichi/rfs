#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
import asyncio
import subprocess
import re
import os
import json
from bleak import BleakScanner
from toio import BLEScanner, ToioCoreCube

# Configuration file path
CONFIG_PATH = os.path.expanduser('~/rfs/src/rfs_config/config/config.json')

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

    async def run_automatic_matching(self):
        # 1. Device Scan
        self.get_logger().info("Confirming Toio cube order via initial scan...")
        try:
            self.get_logger().info("Scanning for all BLE devices...")
            initial_toio_devices = await BLEScanner.scan(num=len(self.roles))
            if not initial_toio_devices:
                self.get_logger().error("No Toio cubes found. Aborting.")
                return
            toio_addresses = [dev.device.address for dev in initial_toio_devices]
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
        
        # 2. Automatic Pairing Based on Order
        num_pairs = min(len(self.roles), len(toio_addresses), len(available_sinks))

        for i in range(num_pairs):
            role = self.roles[i]
            toio_address = toio_addresses[i]
            sink = available_sinks[i]

            self.get_logger().info(f"--- Pairing Start: Role='{role}', Toio='{toio_address}' ---")

            connected_cube = None
            for attempt in range(3): # Max 3 attempts
                try:
                    self.get_logger().info(f"Rescanning Toio {toio_address}... (Attempt {attempt + 1}/3)")
                    fresh_devices = await BLEScanner.scan(num=len(self.roles), timeout=3.0)
                    toio_dev = next((d for d in fresh_devices if d.device.address == toio_address), None)

                    if toio_dev is None:
                        raise ConnectionError(f"Toio {toio_address} not found in rescan.")

                    self.get_logger().info(f"Connecting to Toio {toio_address}...")
                    cube = ToioCoreCube(toio_dev.interface)
                    await cube.connect()
                    connected_cube = cube
                    self.get_logger().info(f"Successfully connected to Toio {toio_address}.")
                    break
                except Exception as e:
                    self.get_logger().warn(f"Connection attempt failed: {e}")
                    if attempt < 2:
                        await asyncio.sleep(1)
            
            if connected_cube:
                try:
                    # Speech and motor control after successful connection
                    speech = f"Speaker {role}. Please place this speaker on the rotating Toio."
                    self.tts.speak(speech, sink)
                    await connected_cube.api.motor.motor_control(50, -50, 1000)

                    # Add matching info to list
                    match_info = { "role": role, "toio_id": toio_address, "speaker_id": sink }
                    match_list.append(match_info)
                    self.get_logger().info(f"Pair confirmed: {role} <-> {toio_address} <-> {sink}")
                
                finally:
                    await connected_cube.disconnect()
                    self.get_logger().info(f"Disconnected Toio {toio_address}.")
                    await asyncio.sleep(1)
            else:
                self.get_logger().error(f"Failed to connect to Toio {toio_address}. Skipping this pair.")

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
