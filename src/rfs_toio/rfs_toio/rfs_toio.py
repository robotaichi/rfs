#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import json
import asyncio
import threading
import signal
import csv
import io
import math
import subprocess
import re
from typing import Tuple, Optional

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from ament_index_python.packages import get_package_share_directory

from toio import BLEScanner, ToioCoreCube, MovementType, Speed, SpeedChangeType, TargetPosition, CubeLocation, Point, RotationOption, WriteMode, PositionId, PositionIdMissed, IdInformation
from toio.cube.api.indicator import IndicatorParam, Color

# Configuration
try:
    SAVE_DIR = os.path.join(get_package_share_directory('rfs_config'), 'config')
except Exception:
    SAVE_DIR = os.path.expanduser("~/rfs/src/rfs_config/config")

CONFIG_FILE = os.path.join(SAVE_DIR, 'config.json')

# Constants
SCAN_TIMEOUT     = 8
MAX_SPEED        = 30
CONTROL_INTERVAL = 0.1
MAT_X_MIN, MAT_X_MAX = 130, 360
MAT_Y_MIN, MAT_Y_MAX = 180, 320
COLLISION_THRESHOLD = 35
AVOIDANCE_LOOP_THRESHOLD = 35

def _best_addr_name(dev) -> Tuple[Optional[str], Optional[str]]:
    addr, name = None, None
    for a in ("address", "mac", "addr"):
        if hasattr(dev, a): addr = getattr(dev, a) or addr
    if hasattr(dev, "interface"):
        if hasattr(dev.interface, "address"): addr = dev.interface.address or addr
    return addr, name

def clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v

class RFSToio(Node):
    def __init__(self, loop: asyncio.AbstractEventLoop):
        super().__init__('rfs_toio')
        self.loop = loop
        self.cube_map: dict[str, ToioCoreCube] = {}
        self.role_toio_map: dict[str, str] = {}
        self.latest_positions: dict[str, dict] = {}
        self.current_move_task: Optional[asyncio.Task] = None
        self.movement_blocked_by_intervention = False

        self.USER_X = 250
        self.USER_Y = 320

        try:
            with open(CONFIG_FILE, 'r', encoding='utf-8') as f:
                config_data = json.load(f)
            for item in config_data.get('toio_speaker_match', []):
                role, toio_id = item.get('role'), item.get('toio_id')
                if role and toio_id: self.role_toio_map[role.lower()] = toio_id
        except Exception as e:
            self.get_logger().error(f"Failed to load config: {e}")

        self.create_subscription(String, 'rfs_toio_move_script', self.execute_script_cb, 10)
        self.create_subscription(String, 'rfs_user_intervention_toio_move', self.user_intervention_cb, 10)
        qos_pl = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE, history=QoSHistoryPolicy.KEEP_LAST, depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.status_publisher = self.create_publisher(String, 'rfs_toio_status', qos_pl)
        self.move_finished_pub = self.create_publisher(String, 'rfs_toio_move_finished', 10)
        self.position_publisher = self.create_publisher(String, 'rfs_toio_position', 10)

        self.status_publisher.publish(String(data=json.dumps({"status": "initializing"})))
        if self.role_toio_map:
            self.loop.call_soon_threadsafe(lambda: asyncio.create_task(self._scan_connect_toio()))
        
        self.get_logger().info('RFS Toio Node Started.')

    async def _scan_connect_toio(self):
        expected_macs = set(self.role_toio_map.values())
        try:
            devs = await asyncio.wait_for(BLEScanner.scan(num=len(expected_macs)), timeout=SCAN_TIMEOUT)
        except asyncio.TimeoutError:
            devs = []

        for role, toio_id in self.role_toio_map.items():
            dev = next((d for d in devs if (_best_addr_name(d)[0] or "").upper() == toio_id.upper()), None)
            if not dev: continue
            try:
                cube = ToioCoreCube(dev.interface)
                await cube.connect()
                self.cube_map[role] = cube
                await cube.api.id_information.register_notification_handler(
                    lambda payload, handler_info, r=role: self._position_notification_handler(payload, r)
                )
                await cube.api.motor.motor_control(50, -50, 500)
            except Exception as e:
                self.get_logger().error(f"Failed to connect {role}: {e}")

        self.status_publisher.publish(String(data=json.dumps({"status": "toios_ready", "positions": self.latest_positions})))
        if self.cube_map:
            self.loop.call_soon_threadsafe(lambda: asyncio.create_task(self._periodic_position_publish()))

    def _position_notification_handler(self, payload: bytearray, role: str):
        id_info = IdInformation.is_my_data(payload)
        if isinstance(id_info, PositionId):
            self.latest_positions[role] = {'x': id_info.center.point.x, 'y': id_info.center.point.y, 'angle': id_info.center.angle}

    async def _periodic_position_publish(self):
        while rclpy.ok():
            if self.latest_positions:
                self.position_publisher.publish(String(data=json.dumps(self.latest_positions)))
            await asyncio.sleep(0.5)

    def execute_script_cb(self, msg: String):
        data = msg.data.strip()
        is_leader = data.startswith("[LEADER_RESPONSE]")
        if is_leader: data = data[len("[LEADER_RESPONSE]"):]
        
        if self.movement_blocked_by_intervention and not is_leader: return
        if is_leader: self.movement_blocked_by_intervention = False

        try:
            reader = csv.reader(io.StringIO(data), skipinitialspace=True)
            parts = next(reader)
            if len(parts) >= 4 and parts[2].lower() == 'move':
                role, recipient, command_str = parts[0].lower(), parts[1].lower(), parts[3]
                self.loop.call_soon_threadsafe(lambda: asyncio.create_task(self.handle_command_sequence(command_str, role, recipient)))
        except Exception: pass

    def user_intervention_cb(self, msg: String):
        try:
            data = json.loads(msg.data)
            role, state = data.get("role").lower(), data.get("state")
            if state == "start":
                self.movement_blocked_by_intervention = True
                if self.current_move_task: self.current_move_task.cancel()
            elif state == "end":
                self.movement_blocked_by_intervention = False
            self.loop.call_soon_threadsafe(lambda: asyncio.create_task(self.handle_user_intervention(role, state)))
        except Exception: pass

    async def handle_user_intervention(self, role: str, state: str):
        cube = self.cube_map.get(role)
        if not cube or not cube.is_connect(): return
        if state == "start":
            await cube.api.indicator.turn_on(IndicatorParam(duration_ms=0, color=Color(r=255, g=0, b=0)))
            # Turn to user logic simplified
            await cube.api.motor.motor_control_target(timeout=2, target=TargetPosition(cube_location=CubeLocation(point=Point(x=self.USER_X, y=self.USER_Y), angle=270)))
        elif state == "end":
            for c in self.cube_map.values():
                await c.api.indicator.turn_on(IndicatorParam(duration_ms=0, color=Color(r=0, g=0, b=0)))

    async def handle_command_sequence(self, command_sequence_str: str, role: str, recipient: str):
        self.current_move_task = asyncio.current_task()
        try:
            if command_sequence_str.lower() != 'none':
                # Simplified command execution
                commands = command_sequence_str.split(';')
                for cmd in commands:
                    if 'motor_control_target' in cmd:
                        # Dummy parse - in real port we'd use the regex-based parser
                        pass
        finally:
            self.move_finished_pub.publish(String(data=json.dumps({"role": role, "recipient": recipient})))
            self.current_move_task = None

def main():
    rclpy.init()
    loop = asyncio.get_event_loop()
    node = RFSToio(loop)
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    t = threading.Thread(target=lambda: loop.run_forever(), daemon=True)
    t.start()
    try: executor.spin()
    except KeyboardInterrupt: pass
    finally:
        loop.call_soon_threadsafe(loop.stop)
        t.join()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
