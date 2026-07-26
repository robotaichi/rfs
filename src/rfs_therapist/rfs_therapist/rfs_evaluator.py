#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
RFS Evaluator node.

Combines each member's evaluation (62-item FACES IV) into circumplex model
coordinates (x, y), then passes the next-target calculation on to the
Optimizer node. The score calculation itself is handled by
`faces_scoring.FacesScoreCalculator` (logic layer); this file only handles ROS2
publish/subscribe (the communication layer).
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import os

from rfs_therapist.faces_scoring import FacesScoreCalculator


class RFSEvaluator(Node):
    """ROS2 node that receives FACES IV evaluation requests and passes the score calculation result to Optimizer."""

    def __init__(self):
        super().__init__('rfs_evaluator')

        # Weight parameters defined in the original implementation (only actually used by
        # Optimizer now, kept here for compatibility)
        self.OMEGA_1 = 0.1
        self.OMEGA_2 = 0.1
        self.OMEGA_3 = 0.05
        self.LEARNING_RATE_SCALING = 0.005

        # --- Logic layer (calculation class, no rclpy dependency) ---
        self.calculator = FacesScoreCalculator()

        # --- ROS2 communication setup ---
        self.create_subscription(String, 'rfs_evaluator_request', self.request_callback, 10)
        self.result_pub = self.create_publisher(String, 'rfs_evaluator_results', 10)
        self.optimizer_req_pub = self.create_publisher(String, 'rfs_optimizer_request', 10)

        self._load_config()
        self.get_logger().info(f"RFS Evaluator Node Started.")

    def _load_config(self):
        """Load the config file (currently unused in practice, since these parameters belong to Optimizer)."""
        home = os.path.expanduser("~")
        config_file = os.path.join(home, "rfs/src/rfs_config/config/config.json")
        try:
            if os.path.exists(config_file):
                with open(config_file, 'r', encoding='utf-8') as f:
                    json.load(f)
                    # OMEGAs are used by Optimizer, but this node also reads the config
                    # file just to confirm it exists (kept from the original implementation)
        except Exception:
            pass

    def request_callback(self, msg: String):
        """Receive an evaluation request, calculate the scores, and publish to the Optimizer topic."""
        try:
            data = json.loads(msg.data)
            step_id = data.get("step_id")
            aggregated_results = data.get("results")

            # 1. Average the per-item ratings (logic layer)
            ratings = self.calculator.average_ratings(aggregated_results)

            # 2. Calculate the circumplex coordinates and percentiles (logic layer)
            results = self.calculator.calculate_scores(ratings)
            results["step_id"] = step_id

            # 3. Delegate next-target calculation to Optimizer
            self.optimizer_req_pub.publish(String(data=json.dumps(results)))
            self.get_logger().info(
                f"Scores calculated for {step_id}: Result({results['x']:.1f}, {results['y']:.1f}). Delegating to Optimizer."
            )

        except Exception as e:
            self.get_logger().error(f"Error in evaluator request processing: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = RFSEvaluator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
