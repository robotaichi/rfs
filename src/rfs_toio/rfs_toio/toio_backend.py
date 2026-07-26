#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Logic layer for toio cube control.

Handles getting BLE device info, keeping numbers inside a set range, and
loading the role<->toio mapping from the config file. Does not depend on ROS2
communication. Called from the rfs_toio node.
"""

import json
from typing import Tuple, Optional


def best_addr_name(dev) -> Tuple[Optional[str], Optional[str]]:
    """Extract the MAC address, as best as possible, from a BLE scan result object."""
    addr, name = None, None
    for a in ("address", "mac", "addr"):
        if hasattr(dev, a):
            addr = getattr(dev, a) or addr
    if hasattr(dev, "interface"):
        if hasattr(dev.interface, "address"):
            addr = dev.interface.address or addr
    return addr, name


def clamp(v, lo, hi):
    """Keep value v inside the range [lo, hi], pushing it up or down if it falls outside."""
    return lo if v < lo else hi if v > hi else v


def load_role_toio_map(config_file: str, logger=None) -> dict:
    """Load the "role name (lowercase) -> toio MAC address" mapping from the config file."""
    role_toio_map = {}
    try:
        with open(config_file, 'r', encoding='utf-8') as f:
            config_data = json.load(f)
        for item in config_data.get('toio_speaker_match', []):
            role, toio_id = item.get('role'), item.get('toio_id')
            if role and toio_id:
                role_toio_map[role.lower()] = toio_id
    except Exception as e:
        if logger:
            logger.error(f"Failed to load config: {e}")
    return role_toio_map
