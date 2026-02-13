#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import os

# Constants
HOME = os.path.expanduser("~")
DB_DIR = os.path.join(HOME, "rfs/src/rfs_database")
GLASS_CASTLE_FILE = os.path.join(DB_DIR, "glass_castle_analysis.md")
FACES_TABLES_FILE = os.path.join(DB_DIR, "faces_iv_tables.md")

class RFSDocumentProcessor(Node):
    def __init__(self):
        super().__init__('rfs_document_processor')
        
        # Knowledge Base
        self.faces_tables = self._load_faces_tables()
        self.glass_castle_data = self._load_glass_castle_analysis()
        
        # Publishers/Subscribers
        self.behavior_pub = self.create_publisher(String, 'rfs_behavioral_info_results', 10)
        self.few_shot_pub = self.create_publisher(String, 'rfs_few_shot_results', 10)
        
        self.create_subscription(String, 'rfs_behavioral_info_request', self.behavior_request_callback, 10)
        self.create_subscription(String, 'rfs_few_shot_request', self.few_shot_request_callback, 10)
        
        self.get_logger().info("RFS Document Processor Node Started.")

    def _load_faces_tables(self):
        tables = {"cohesion": {}, "flexibility": {}, "communication": {}}
        if not os.path.exists(FACES_TABLES_FILE):
            self.get_logger().error(f"FACES tables file not found: {FACES_TABLES_FILE}")
            return tables
        
        current_section = None
        try:
            with open(FACES_TABLES_FILE, 'r', encoding='utf-8') as f:
                lines = f.readlines()
                for line in lines:
                    line = line.strip()
                    if "## 1. Cohesion" in line: current_section = "cohesion"
                    elif "## 2. Flexibility" in line: current_section = "flexibility"
                    elif "## 3. Communication" in line: current_section = "communication"
                    
                    if line.startswith("|"):
                        parts = [p.strip() for p in line.split("|")]
                        if parts and not parts[0]: parts.pop(0)
                        if parts and not parts[-1]: parts.pop()
                        if len(parts) < 2 or "---" in parts[0] or "Level" in parts[1] or "Low" in parts[1]: continue
                        
                        cat = parts[0].replace("**", "")
                        if current_section:
                            tables[current_section][cat] = parts[1:]
        except Exception as e:
            self.get_logger().error(f"Failed to load FACES tables: {e}")
        return tables

    def _load_glass_castle_analysis(self):
        try:
            if not os.path.exists(GLASS_CASTLE_FILE): return ""
            with open(GLASS_CASTLE_FILE, "r", encoding="utf-8") as f:
                return f.read()
        except Exception as e:
            self.get_logger().error(f"Failed to load Glass Castle analysis: {e}")
            return ""

    def behavior_request_callback(self, msg: String):
        try:
            data = json.loads(msg.data)
            x = data.get("x", 50.0)
            y = data.get("y", 50.0)
            request_id = data.get("request_id", "")
            role = data.get("role", "")
            self.get_logger().info(f"Received behavioral info request for {role} (Req: {request_id})")

            # Logic ported from RFSMember
            def get_col(score):
                if score <= 15: return 0
                if score <= 35: return 1
                if score <= 65: return 2
                if score <= 85: return 3
                return 4
            
            c_idx = get_col(x)
            f_idx = get_col(y)
            
            dist = ((x-50)**2 + (y-50)**2)**0.5
            comm_pct = max(0, 100 - dist)
            comm_idx = 0
            if comm_pct <= 33: comm_idx = 0
            elif comm_pct <= 66: comm_idx = 1
            else: comm_idx = 2
            
            desc = "# Behavioral Guidelines\n"
            desc += "## Detailed Cohesion Guidelines\n"
            for cat, vals in self.faces_tables.get("cohesion", {}).items():
                if c_idx < len(vals): desc += f"- {cat}: {vals[c_idx]}\n"
            
            desc += "\n## Detailed Flexibility Guidelines\n"
            for cat, vals in self.faces_tables.get("flexibility", {}).items():
                if f_idx < len(vals): desc += f"- {cat}: {vals[f_idx]}\n"

            desc += "\n## Detailed Communication Guidelines\n"
            for cat, vals in self.faces_tables.get("communication", {}).items():
                if comm_idx < len(vals): desc += f"- {cat}: {vals[comm_idx]}\n"
            
            response = {
                "request_id": request_id,
                "role": role,
                "behavioral_descriptors": desc
            }
            self.behavior_pub.publish(String(data=json.dumps(response)))

        except Exception as e:
            self.get_logger().error(f"Error in behavior_request_callback: {e}")

    def few_shot_request_callback(self, msg: String):
        try:
            data = json.loads(msg.data)
            request_id = data.get("request_id", "")
            role = data.get("role", "")
            self.get_logger().info(f"Received few-shot request for {role} (Req: {request_id})")
            
            response = {
                "request_id": request_id,
                "role": role,
                "few_shot_context": self.glass_castle_data
            }
            self.few_shot_pub.publish(String(data=json.dumps(response)))
        except Exception as e:
            self.get_logger().error(f"Error in few_shot_request_callback: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = RFSDocumentProcessor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
