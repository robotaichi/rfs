#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
RFS Optimizer node.

Takes the current evaluation result received from Evaluator and computes the next
session's target coordinates (tx, ty) using gradient descent. The gradient descent
calculation itself is handled by `gradient_optimizer.GradientOptimizer` (logic
layer); this file only handles ROS2 publish/subscribe (the communication layer).
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import os

from rfs_therapist.gradient_optimizer import GradientOptimizer


class RFSOptimizer(Node):
    """ROS2 node that computes and publishes the next target coordinates."""

    def __init__(self):
        super().__init__('rfs_optimizer')

        # Default values, overridden once the config file is loaded
        self.OMEGA_1 = 0.1
        self.OMEGA_2 = 0.1
        self.OMEGA_3 = 0.05
        self.LEARNING_RATE_SCALING = 0.005

        # --- ROS2 communication setup ---
        self.create_subscription(String, 'rfs_optimizer_request', self.request_callback, 10)
        self.result_pub = self.create_publisher(String, 'rfs_evaluator_results', 10)

        self._load_config()

        # --- Logic layer (calculation class, no rclpy dependency) ---
        self.optimizer = GradientOptimizer(
            omega_1=self.OMEGA_1,
            omega_2=self.OMEGA_2,
            omega_3=self.OMEGA_3,
            learning_rate_scaling=self.LEARNING_RATE_SCALING,
        )

        self.get_logger().info(f"RFS Optimizer Node Started. LR={self.LEARNING_RATE_SCALING}")

    def _load_config(self):
        """Load the gradient descent settings from the config file."""
        home = os.path.expanduser("~")
        config_file = os.path.join(home, "rfs/src/rfs_config/config/config.json")
        try:
            if os.path.exists(config_file):
                with open(config_file, 'r', encoding='utf-8') as f:
                    config = json.load(f)
                    self.OMEGA_1 = config.get("w1", 0.1)
                    self.OMEGA_2 = config.get("w2", 0.1)
                    self.OMEGA_3 = config.get("w3", 0.05)
                    self.LEARNING_RATE_SCALING = config.get("learning_rate_scaling", 0.005)
        except Exception:
            pass

    def request_callback(self, msg: String):
        """Receive an optimization request, compute the next target coordinates, and publish to the evaluator-results topic."""
        try:
            data = json.loads(msg.data)
            step_id = data.get("step_id")
            x = data.get("x")
            y = data.get("y")
            pcts = data.get("pcts")
            coh_ratio = data.get("coh_ratio")
            flex_ratio = data.get("flex_ratio")
            tot_ratio = data.get("tot_ratio")
            mean_ratings = data.get("mean_ratings")

            # Calculate next target subscale scores using gradient descent (logic layer)
            target_scores = self.optimizer.calculate_gradient(pcts, x, y)

            # Next session's target coordinates (tx, ty on the circumplex model)
            tx = target_scores["Balanced Cohesion"] + (target_scores["Enmeshed"] - target_scores["Disengaged"]) / 2.0
            ty = target_scores["Balanced Flexibility"] + (target_scores["Chaotic"] - target_scores["Rigid"]) / 2.0
            tx = max(5.0, min(95.0, tx))
            ty = max(5.0, min(95.0, ty))

            final_results = {
                "step_id": step_id,
                "x": x, "y": y,
                "tx": tx, "ty": ty,
                "pcts": pcts,
                "target_scores": target_scores,
                "coh_ratio": coh_ratio,
                "flex_ratio": flex_ratio,
                "tot_ratio": tot_ratio,
                "mean_ratings": mean_ratings,
            }

            self.result_pub.publish(String(data=json.dumps(final_results)))
            self.get_logger().info(f"Optimization calculated for {step_id}: Target({tx:.1f}, {ty:.1f})")

        except Exception as e:
            self.get_logger().error(f"Error in optimizer request processing: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = RFSOptimizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
