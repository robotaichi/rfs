#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
RFS Therapist node.

The central "therapist" node: triggered by the family members' conversation, it
requests and collects each member's subjective evaluation (FACES IV), receives the
result (through Evaluator/Optimizer), and generates the plot image, logs to CSV, and
appends the target to the history file.

Visualization and file-logging logic that doesn't depend on ROS2 is handled by
`faces_plotter.FacesReportGenerator`; this file only handles ROS2 publish/subscribe
(the communication layer) and managing how the session moves forward.

Beginner's guide — one evaluation cycle, step by step:
  1. A family member publishes `rfs_trigger_evaluation`; `trigger_callback`
     asks every member for their own subjective rating (`rfs_request_member_evaluation`).
  2. `member_evaluation_callback` collects each member's answer. Once everyone
     has answered, it hands the combined results to the Evaluator node.
  3. The Evaluator and Optimizer nodes (separate files) turn those ratings
     into scores and a next-session target, then publish the result back here.
  4. `evaluator_result_callback` receives that result, saves it (CSV + JSON
     trajectory file), draws the new plot, and publishes `rfs_evaluation_complete`
     so the family members know they can resume the conversation.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys
import os
import json
import threading
import csv
import io
import datetime
import time
import shutil
from rfs_interfaces.srv import TTSService
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from ament_index_python.packages import get_package_share_directory

from rfs_therapist.faces_plotter import FacesReportGenerator

# Constants
HOME = os.path.expanduser("~")
DB_DIR = os.path.join(HOME, "rfs/src/rfs_database")

try:
    SHARE_DIR = get_package_share_directory('rfs_config')
    SAVE_DIR = os.path.join(SHARE_DIR, 'config')
except Exception:
    SAVE_DIR = os.path.join(os.path.dirname(DB_DIR), "rfs_config/config")

os.makedirs(DB_DIR, exist_ok=True)

HISTORY_FILE = os.path.join(DB_DIR, "conversation_history.txt")
CONFIG_FILE = os.path.join(SAVE_DIR, "config.json")


class TTSClient:
    """Thin client wrapper that calls the TTS service (`rfs_speak_text`) without waiting for a reply."""

    def __init__(self, node_name):
        self.node = Node(f'rfs_tts_client_{node_name}')
        self.cli = self.node.create_client(TTSService, 'rfs_speak_text')
        while not self.cli.wait_for_service(timeout_sec=3.0):
            self.node.get_logger().info('Waiting for rfs_speak_text service...')
        self.exec = SingleThreadedExecutor()
        self.exec.add_node(self.node)
        t = threading.Thread(target=self.exec.spin, daemon=True)
        t.start()

    def speak(self, text: str, delay: float = 0.0):
        req = TTSService.Request()
        req.text = text
        req.delay = delay
        self.cli.call_async(req)


class RFSTherapist(Node):
    """ROS2 node that issues evaluation requests as the conversation progresses and visualizes/logs the results."""

    def __init__(self):
        super().__init__('rfs_therapist')
        self.role = "therapist"
        self.tts = TTSClient(node_name=self.role)
        self.archival_lock = threading.Lock()

        self.TRAJECTORY_FILE = os.path.join(DB_DIR, "evaluation_trajectory.json")
        self.OMEGA_1 = 0.1
        self.OMEGA_2 = 0.1
        self.OMEGA_3 = 0.05  # Suppress center pull
        self.LEARNING_RATE_SCALING = 0.005  # Increased for faster progression
        self.family_config = []

        self.member_results = {}  # {step_id: {role: results}}
        self.processed_steps = set()

        # Item-number to subscale mapping (unused in this node currently, kept for compatibility with the original implementation)
        self.scales = {
            "Balanced Cohesion": [1, 7, 13, 19, 25, 31, 37],
            "Balanced Flexibility": [2, 8, 14, 20, 26, 32, 38],
            "Disengaged": [3, 9, 15, 21, 27, 33, 39],
            "Enmeshed": [4, 10, 16, 22, 28, 34, 40],
            "Rigid": [5, 11, 17, 23, 29, 35, 41],
            "Chaotic": [6, 12, 18, 24, 30, 36, 42],
            "Communication": list(range(43, 53)),
            "Satisfaction": list(range(53, 63)),
        }

        self._load_config()

        # --- Logic layer (no rclpy dependency: plot generation, CSV logging, clinical tables) ---
        self.report = FacesReportGenerator(logger=self.get_logger(), db_dir=DB_DIR)

        # --- ROS2 communication setup ---
        self.create_subscription(String, 'rfs_family_actions', self.family_actions_callback, 10)
        self.create_subscription(String, 'rfs_user_intervention', self.user_intervention_callback, 10)
        self.create_subscription(String, 'rfs_trigger_evaluation', self.trigger_callback, 10)
        self.create_subscription(String, 'rfs_member_evaluation_results', self.member_evaluation_callback, 10)
        self.create_subscription(String, 'rfs_evaluator_results', self.evaluator_result_callback, 10)

        self.family_publisher = self.create_publisher(String, 'rfs_family_actions', 10)
        self.request_eval_pub = self.create_publisher(String, 'rfs_request_member_evaluation', 10)
        self.evaluator_req_pub = self.create_publisher(String, 'rfs_evaluator_request', 10)
        self.complete_pub = self.create_publisher(String, 'rfs_evaluation_complete', 10)
        self.plot_pub = self.create_publisher(String, 'rfs_faces_plot_updated', 10)

        self.init_plot()
        self.get_logger().info("RFS Therapist Node Started.")

    def destroy_node(self):
        super().destroy_node()

    def _load_trajectory(self) -> list:
        """Load the saved trajectory list from `TRAJECTORY_FILE`, or return an empty list if it doesn't exist yet."""
        traj = []
        if os.path.exists(self.TRAJECTORY_FILE):
            try:
                with open(self.TRAJECTORY_FILE, 'r') as f:
                    traj = json.load(f)
            except Exception:
                pass
        return traj

    def init_plot(self):
        """On startup, show the existing trajectory if one exists, otherwise plot the initial coordinates."""
        # Notify viewer to clear state first
        reset_msg = String()
        reset_msg.data = "RESET"
        self.plot_pub.publish(reset_msg)

        traj = self._load_trajectory()
        if traj:
            last = traj[-1]
            x = last.get("result_x", last.get("x", 8.0))
            y = last.get("result_y", last.get("y", 8.0))
            tx = last.get("target_x")
            ty = last.get("target_y")
            self._generate_and_publish_plot(x, y, 1.0, 1.0, 1.0, traj, tx=tx, ty=ty)
        else:
            # Show S0 from initial_coords
            s0_x = self.initial_coords.get("x", 8.0)
            s0_y = self.initial_coords.get("y", 8.0)
            self._generate_and_publish_plot(s0_x, s0_y, 1.0, 1.0, 1.0, [{"target_x": s0_x, "target_y": s0_y}])

    def _generate_and_publish_plot(self, x, y, coh_ratio, flex_ratio, tot_ratio, trajectory=None, tx=None, ty=None):
        """Generate the plot image using the logic layer, and publish its path to the viewer only on success."""
        save_path = self.report.generate_plot(x, y, coh_ratio, flex_ratio, tot_ratio, trajectory, tx=tx, ty=ty)
        if save_path:
            self.plot_pub.publish(String(data=save_path))

    def _load_config(self):
        try:
            if os.path.exists(CONFIG_FILE):
                with open(CONFIG_FILE, 'r', encoding='utf-8') as f:
                    config = json.load(f)
                    self.family_config = config.get("family_config", [])
                    self.initial_coords = config.get("initial_coords", {"x": 8.0, "y": 8.0})
        except Exception:
            pass

    def trigger_callback(self, msg: String):
        """Start collecting member evaluations when a conversation step boundary (rfs_trigger_evaluation) is reached."""
        step_id = msg.data
        if step_id in self.processed_steps:
            return
        if step_id in self.member_results:
            return

        self.member_results[step_id] = {}
        self.request_eval_pub.publish(String(data=step_id))

    def member_evaluation_callback(self, msg: String):
        """Gather each member's evaluation, and once everyone has answered, hand the results to the Evaluator node."""
        try:
            data = json.loads(msg.data)
            step_id, role, results = data.get("step_id"), data.get("role"), data.get("results")
            if step_id and role:
                if step_id not in self.member_results:
                    self.member_results[step_id] = {}
                self.member_results[step_id][role.lower()] = results

                received = len(self.member_results[step_id])
                total = len(self.family_config)

                if received >= total:
                    if step_id not in self.processed_steps:
                        # DELEGATE TO EVALUATOR
                        req = {"step_id": step_id, "results": self.member_results[step_id]}
                        self.evaluator_req_pub.publish(String(data=json.dumps(req)))
        except Exception as e:
            self.get_logger().error(f"Error in evaluation aggregation: {e}")

    def evaluator_result_callback(self, msg: String):
        """Receive the final result computed by Evaluator/Optimizer and log, visualize, and announce session completion."""
        try:
            data = json.loads(msg.data)
            step_id = data.get("step_id")
            x, y = data.get("x"), data.get("y")
            tx, ty = data.get("tx"), data.get("ty")
            pcts = data.get("pcts")
            coh_ratio = data.get("coh_ratio")
            flex_ratio = data.get("flex_ratio")
            tot_ratio = data.get("tot_ratio")
            mean_ratings = data.get("mean_ratings")
            target_scores = data.get("target_scores")

            self.processed_steps.add(step_id)

            # Clinical Labeling Logic
            def get_label(val, labels):
                if val <= 15:
                    return labels[0]
                if val <= 35:
                    return labels[1]
                if val <= 65:
                    return labels[2]
                if val <= 85:
                    return labels[3]
                return labels[4]

            coh_label = get_label(x, ["Disengaged", "Somewhat Connected", "Connected", "Very Connected", "Enmeshed"])
            flex_label = get_label(y, ["Rigid", "Somewhat Flexible", "Flexible", "Very Flexible", "Chaotic"])
            family_type = f"{coh_label}-{flex_label}"

            self.get_logger().info(f"[{self.role}] Current Family Type: {family_type} at ({x:.1f}, {y:.1f})")
            details = self.report.get_detailed_behavior(x, y)
            if details:
                self.get_logger().info(f"[{self.role}] Clinical Behavioral Descriptions:\n{details}")

            self.report.update_history_with_targets(HISTORY_FILE, target_scores, tx, ty)

            # Trajectory update
            traj = self._load_trajectory()

            if not traj:
                traj.append({"step": "S0", "target_x": self.initial_coords.get("x", 8.0), "target_y": self.initial_coords.get("y", 8.0)})

            traj.append({"step": step_id, "result_x": x, "result_y": y, "target_x": tx, "target_y": ty})
            with open(self.TRAJECTORY_FILE, 'w') as f:
                json.dump(traj, f)

            # Logging
            self.report.log_evaluation_to_csv(step_id, self.member_results[step_id], mean_ratings, pcts, x, y, tx, ty, coh_ratio, flex_ratio, tot_ratio, target_scores)

            self.get_logger().info(f"FACES IV Succeeded: Result({x:.1f}, {y:.1f}), Target({tx:.1f}, {ty:.1f})")
            self._generate_and_publish_plot(x, y, coh_ratio, flex_ratio, tot_ratio, traj, tx=tx, ty=ty)

            # Finalize step
            self.complete_pub.publish(String(data=step_id))

        except Exception as e:
            self.get_logger().error(f"Error processing evaluator results: {e}")

    def family_actions_callback(self, msg: String):
        pass

    def user_intervention_callback(self, msg: String):
        pass


def main():
    rclpy.init()
    node = RFSTherapist()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("KeyboardInterrupt received, initiating shutdown...")
    except Exception as e:
        print(f"Unexpected error in spin: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
