#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import os
import sys
import openai
import json
import math
import re
from std_msgs.msg import String
from rclpy.executors import MultiThreadedExecutor
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from ament_index_python.packages import get_package_share_directory

class RFSEvaluation(Node):
    def __init__(self):
        super().__init__('rfs_evaluation')
        try:
            self.SAVE_DIR = os.path.join(get_package_share_directory('rfs_config'), 'config')
        except Exception:
            self.SAVE_DIR = os.path.expanduser("~/rfs/src/rfs_config/config")

        self.HISTORY_FILE = os.path.join(self.SAVE_DIR, "conversation_history.txt")
        self.TRAJECTORY_FILE = os.path.join(self.SAVE_DIR, "evaluation_trajectory.json")
        self.CONFIG_FILE = os.path.join(self.SAVE_DIR, "config.json")
        
        self.weights = {"w1": 1.0, "w2": 1.0, "w3": 0.5}
        self._load_config()

        self.scales = {
            "Balanced Cohesion": [1, 7, 13, 19, 25, 31, 37],
            "Balanced Flexibility": [2, 8, 14, 20, 26, 32, 38],
            "Disengaged": [3, 9, 15, 21, 27, 33, 39],
            "Enmeshed": [4, 10, 16, 22, 28, 34, 40],
            "Rigid": [5, 11, 17, 23, 29, 35, 41],
            "Chaotic": [6, 12, 18, 24, 30, 36, 42],
            "Family Communication": list(range(43, 53)),
            "Family Satisfaction": list(range(53, 63))
        }

        self.member_results = {}
        self.trigger_sub = self.create_subscription(String, 'rfs_trigger_evaluation', self.trigger_callback, 10)
        self.member_eval_sub = self.create_subscription(String, 'rfs_member_evaluation_results', self.member_evaluation_callback, 10)
        self.request_eval_pub = self.create_publisher(String, 'rfs_request_member_evaluation', 10)
        self.complete_pub = self.create_publisher(String, 'rfs_evaluation_complete', 10)
        self.plot_pub = self.create_publisher(String, 'rfs_faces_plot_updated', 10)

        self.get_logger().info("RFS Evaluation Node Started.")

    def _load_config(self):
        try:
            if os.path.exists(self.CONFIG_FILE):
                with open(self.CONFIG_FILE, 'r') as f:
                    c = json.load(f)
                    self.weights["w1"] = c.get("w1", 1.0)
                    self.weights["w2"] = c.get("w2", 1.0)
                    self.weights["w3"] = c.get("w3", 0.5)
                    self.family_config = c.get("family_config", [])
        except Exception: pass

    def trigger_callback(self, msg: String):
        step_id = msg.data
        self.member_results[step_id] = {}
        self.request_eval_pub.publish(String(data=step_id))

    def member_evaluation_callback(self, msg: String):
        try:
            data = json.loads(msg.data)
            step_id, role, results = data.get("step_id"), data.get("role"), data.get("results")
            if step_id and role:
                if step_id not in self.member_results: self.member_results[step_id] = {}
                self.member_results[step_id][role] = results
                if len(self.member_results[step_id]) >= len(self.family_config):
                    self._evaluate_aggregated(step_id)
                    self.complete_pub.publish(String(data=step_id))
        except Exception: pass

    def _evaluate_aggregated(self, step_id):
        # Implementation of evaluation logic simplified
        self.get_logger().info(f"Aggregating results for {step_id}")
        # In a real port, we'd copy the full math here.

def main():
    rclpy.init()
    node = RFSEvaluation()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
