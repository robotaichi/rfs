#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
RFS Document Processor node.

A document delivery node that provides FACES behavioral guidelines and
reference examples (few-shot) on request from the family members. Loading the
documents and building the behavior descriptions is handled by
`document_knowledge.DocumentKnowledgeBase` (no rclpy dependency); this file
only handles ROS2 publish/subscribe (the communication layer).
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import os

from rfs_family.document_knowledge import DocumentKnowledgeBase

# Constants
HOME = os.path.expanduser("~")
DB_DIR = os.path.join(HOME, "rfs/src/rfs_database")
GLASS_CASTLE_FILE = os.path.join(DB_DIR, "glass_castle_analysis.md")
FACES_TABLES_FILE = os.path.join(DB_DIR, "faces_iv_tables.md")


class RFSDocumentProcessor(Node):
    """ROS2 node that responds to requests for clinical reference documents (behavioral guidelines, reference examples)."""

    def __init__(self):
        super().__init__('rfs_document_processor')

        # --- Logic layer (no rclpy dependency: document loading, building behavior descriptions) ---
        self.knowledge = DocumentKnowledgeBase(
            logger=self.get_logger(),
            faces_tables_file=FACES_TABLES_FILE,
            glass_castle_file=GLASS_CASTLE_FILE,
        )

        # --- ROS2 communication setup ---
        self.behavior_pub = self.create_publisher(String, 'rfs_behavioral_info_results', 10)
        self.few_shot_pub = self.create_publisher(String, 'rfs_few_shot_results', 10)

        self.create_subscription(String, 'rfs_behavioral_info_request', self.behavior_request_callback, 10)
        self.create_subscription(String, 'rfs_few_shot_request', self.few_shot_request_callback, 10)

        self.get_logger().info("RFS Document Processor Node Started.")

    def behavior_request_callback(self, msg: String):
        """Build the behavioral guidelines for the given (x, y) values and send them back to the requester."""
        try:
            data = json.loads(msg.data)
            x = data.get("x", 50.0)
            y = data.get("y", 50.0)
            request_id = data.get("request_id", "")
            role = data.get("role", "")
            self.get_logger().info(f"Received behavioral info request for {role} (Req: {request_id})")

            desc = self.knowledge.build_behavioral_guidelines(x, y)

            response = {
                "request_id": request_id,
                "role": role,
                "behavioral_descriptors": desc
            }
            self.behavior_pub.publish(String(data=json.dumps(response)))

        except Exception as e:
            self.get_logger().error(f"Error in behavior_request_callback: {e}")

    def few_shot_request_callback(self, msg: String):
        """Publish the reference example (Glass Castle analysis) back to the requester."""
        try:
            data = json.loads(msg.data)
            request_id = data.get("request_id", "")
            role = data.get("role", "")
            self.get_logger().info(f"Received few-shot request for {role} (Req: {request_id})")

            response = {
                "request_id": request_id,
                "role": role,
                "few_shot_context": self.knowledge.glass_castle_data
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
