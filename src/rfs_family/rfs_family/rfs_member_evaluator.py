#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
RFS Member Evaluator node.

Has each family member rate the 62 FACES IV items from their own point of
view. Building the evaluation prompt and calling the LLM is handled by
`member_evaluation.MemberEvaluator` (no rclpy dependency); this file only
handles ROS2 publish/subscribe (the communication layer) and manages the work
that runs in the background.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import os
os.environ["NO_GCE_CHECK"] = "true"
import threading

from rfs_family.member_evaluation import MemberEvaluator


class RFSMemberEvaluator(Node):
    """ROS2 node that receives member subjective-evaluation requests and publishes the LLM's evaluation result."""

    def __init__(self):
        super().__init__('rfs_member_evaluator')

        # --- Logic layer (no rclpy dependency: evaluation prompt building, LLM call) ---
        self.evaluator = MemberEvaluator(self.get_logger())

        # --- ROS2 communication setup ---
        self.create_subscription(String, 'rfs_member_eval_request', self.request_callback, 10)
        self.result_pub = self.create_publisher(String, 'rfs_member_evaluation_results', 10)
        self.get_logger().info("RFS Member Evaluator Node Started.")

    def request_callback(self, msg: String):
        """Run evaluation on a separate thread so the ROS2 executor doesn't get blocked."""
        threading.Thread(target=self._process_request, args=(msg.data,), daemon=True).start()

    def _process_request(self, msg_data: str):
        try:
            data = json.loads(msg_data)
            step_id = data.get("step_id")
            role = data.get("role")
            history = data.get("history", "")
            llm_model = data.get("llm_model", "gpt-4o")
            llm_temperature = data.get("llm_temperature", 0.7)

            self.get_logger().info(f"Evaluating {role} for {step_id}...")

            results = self.evaluator.evaluate(role, history, llm_model, llm_temperature)

            result_payload = {
                "step_id": step_id,
                "role": role,
                "results": results
            }
            self.result_pub.publish(String(data=json.dumps(result_payload)))
            self.get_logger().info(f"Evaluation for {role} ({step_id}) complete.")

        except Exception as e:
            self.get_logger().error(f"Error in member evaluator: {e}")
            # Fallback empty results to avoid hanging
            if 'step_id' in locals() and 'role' in locals():
                dummy = {"step_id": step_id, "role": role, "results": {}}
                self.result_pub.publish(String(data=json.dumps(dummy)))


def main(args=None):
    rclpy.init(args=args)
    node = RFSMemberEvaluator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
