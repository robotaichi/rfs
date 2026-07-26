#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
toio_speaker_match node.

A manual calibration tool run once at setup time. Scans toio cubes over BLE and
pairs them, in detection order, with the family roles (family_config in
config.json) and available speakers, then writes the result back to
`toio_speaker_match` in `config.json`.
Not started by `launch all` (rfs_all.launch.py) — run by itself with
`ros2 run rfs_toio toio_speaker_match`.

System TTS announcements and getting BLE addresses are handled by
`speaker_matching_backend` (no rclpy dependency); this file handles the ROS2
node lifecycle and runs the pairing steps in order (communication layer +
control flow).
"""

import rclpy
from rclpy.node import Node
import asyncio
import os
import json
from toio import BLEScanner, ToioCoreCube

from rfs_toio.speaker_matching_backend import SystemTTS, best_addr_name, fallback_query_name_addr

# Configuration file path
CONFIG_PATH = os.path.expanduser('~/rfs/src/rfs_config/config/config.json')


class ToioSpeakerMatcher(Node):
    """ROS2 node that performs automatic pairing between toios and speakers."""

    def __init__(self):
        super().__init__('toio_speaker_matcher')
        self.tts = SystemTTS(self.get_logger())
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
        """The full process: BLE scan -> speaker assignment -> toio connection check (voice+rotation) -> write config.json."""
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
                addr, name = best_addr_name(d)
                if not name or name == "UNKNOWN":
                    self.get_logger().info(f"Retrieving name for {addr} via connection...")
                    addr2, name2 = await fallback_query_name_addr(d)
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
