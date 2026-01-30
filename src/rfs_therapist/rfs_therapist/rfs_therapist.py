#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys
import os
import json
import openai
import threading
import csv
import io
import datetime
from rfs_interfaces.srv import TTSService
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from ament_index_python.packages import get_package_share_directory

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

# OpenAI Client
if not os.environ.get("OPENAI_API_KEY"):
    print("Please set the OPENAI_API_KEY environment variable.")
    sys.exit(1)
client = openai.OpenAI(api_key=os.environ.get("OPENAI_API_KEY"))

class TTSClient:
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
    def __init__(self):
        super().__init__('rfs_therapist')
        self.role = "therapist"
        self.tts = TTSClient(node_name=self.role)
        
        self.TRAJECTORY_FILE = os.path.join(DB_DIR, "evaluation_trajectory.json")
        self.OMEGA_1 = 1.0
        self.OMEGA_2 = 1.0
        self.OMEGA_3 = 2.0 # Increased from 0.5 to prioritize center targeting
        self.LEARNING_RATE_SCALING = 0.25
        self.family_config = []
        
        self.member_results = {} # {step_id: {role: results}}
        self.processed_steps = set()
        
        self.scales = {
            "Balanced Cohesion": [1, 7, 13, 19, 25, 31, 37],
            "Balanced Flexibility": [2, 8, 14, 20, 26, 32, 38],
            "Disengaged": [3, 9, 15, 21, 27, 33, 39],
            "Enmeshed": [4, 10, 16, 22, 28, 34, 40],
            "Rigid": [5, 11, 17, 23, 29, 35, 41],
            "Chaotic": [6, 12, 18, 24, 30, 36, 42],
            "Communication": list(range(43, 53)),
            "Satisfaction": list(range(53, 63))
        }

        self._load_config()

        self.create_subscription(String, 'rfs_family_actions', self.family_actions_callback, 10)
        self.create_subscription(String, 'rfs_user_intervention', self.user_intervention_callback, 10)
        self.create_subscription(String, 'rfs_trigger_evaluation', self.trigger_callback, 10)
        self.create_subscription(String, 'rfs_member_evaluation_results', self.member_evaluation_callback, 10)

        self.family_publisher = self.create_publisher(String, 'rfs_family_actions', 10)
        self.request_eval_pub = self.create_publisher(String, 'rfs_request_member_evaluation', 10)
        self.complete_pub = self.create_publisher(String, 'rfs_evaluation_complete', 10)
        self.plot_pub = self.create_publisher(String, 'rfs_faces_plot_updated', 10)

        # Reset viewer state and initial plot if trajectory exists
        self.init_plot()
        self.get_logger().info("RFS Therapist Node Started.")

    def init_plot(self):
        # Notify viewer to clear state first
        reset_msg = String()
        reset_msg.data = "RESET"
        self.plot_pub.publish(reset_msg)

        traj = []
        if os.path.exists(self.TRAJECTORY_FILE):
            try:
                with open(self.TRAJECTORY_FILE, 'r') as f: traj = json.load(f)
            except: pass
        if traj:
            last = traj[-1]
            x = last.get("result_x", last.get("x", 8.0))
            y = last.get("result_y", last.get("y", 8.0))
            self.generate_plot(x, y, 1.0, 1.0, 1.0, traj)
        else:
            # Generate a blank plot on startup
            self.generate_plot(None, None, 1.0, 1.0, 1.0, [])

    def _load_config(self):
        try:
            if os.path.exists(CONFIG_FILE):
                with open(CONFIG_FILE, 'r', encoding='utf-8') as f:
                    config = json.load(f)
                    self.OMEGA_1 = config.get("w1", 1.0)
                    self.OMEGA_2 = config.get("w2", 1.0)
                    self.OMEGA_3 = config.get("w3", 0.5)
                    self.family_config = config.get("family_config", [])
                    self.initial_coords = config.get("initial_coords", {"x": 8.0, "y": 8.0})
                    self.LEARNING_RATE_SCALING = config.get("learning_rate_scaling", 0.25)
        except: pass

    def trigger_callback(self, msg: String):
        step_id = msg.data
        if step_id in self.processed_steps:
            self.get_logger().info(f"Trigger ignored for {step_id} (Already processed).")
            return
        # Deduplicate: if evaluation is already pending or in progress for this step
        if step_id in self.member_results:
            self.get_logger().info(f"Trigger ignored for {step_id} (Already in progress).")
            return
            
        self.get_logger().info(f"Trigger received for {step_id}. Requesting evaluations...")
        self.member_results[step_id] = {}
        self.request_eval_pub.publish(String(data=step_id))

    def member_evaluation_callback(self, msg: String):
        try:
            data = json.loads(msg.data)
            step_id, role, results = data.get("step_id"), data.get("role"), data.get("results")
            if step_id and role:
                if step_id not in self.member_results: self.member_results[step_id] = {}
                self.member_results[step_id][role.lower()] = results
                
                received = len(self.member_results[step_id])
                total = len(self.family_config)
                self.get_logger().info(f"[Therapist] Received FACES IV: {received}/{total}")
                
                if received >= total:
                    if step_id not in self.processed_steps:
                        self.evaluate_aggregated(step_id)
                        self.processed_steps.add(step_id)
                    self.complete_pub.publish(String(data=step_id))
        except Exception as e:
            self.get_logger().error(f"Error in evaluation aggregation: {e}")

    def evaluate_aggregated(self, step_id):
        aggregated_results = self.member_results.get(step_id, {})
        ratings = {}
        for item_id in range(1, 63):
            item_scores = []
            for role, results in aggregated_results.items():
                score = results.get(str(item_id))
                if isinstance(score, dict): score = score.get("rating")
                if score is not None: item_scores.append(float(score))
            ratings[item_id] = sum(item_scores)/len(item_scores) if item_scores else 3.0
            
        self.calculate_scores(ratings, step_id, aggregated_results)

    def calculate_scores(self, ratings, step_id, aggregated_results):
        scores = {k: sum(ratings.get(i, 3.0) for i in items) for k, items in self.scales.items()}
        
        # Percentile Lookup
        def get_pct(scale, raw):
            raw = max(7, min(35, int(raw)))
            if scale.startswith("Balanced"):
                table = {
                    7:16, 8:18, 9:20, 10:22, 11:24, 12:25, 13:26, 14:27, 15:28, 16:30, 17:32,
                    18:35, 19:36, 20:38, 21:40, 22:45, 23:50, 24:55, 25:58, 26:60, 27:62,
                    28:65, 29:68, 30:70, 31:75, 32:80, 33:82, 34:84, 35:85
                }
                return table.get(raw, 50)
            else:
                table = {
                    7:10, 8:12, 9:13, 10:14, 11:15, 12:16, 13:18, 14:20, 15:24, 16:26,
                    17:30, 18:32, 19:34, 20:36, 21:40, 22:45, 23:50, 24:55, 25:60, 26:64,
                    27:68, 28:70, 29:75, 30:80, 31:85, 32:90, 33:95, 34:98, 35:99
                }
                return table.get(raw, 30)

        pcts = {k: get_pct(k, v) for k, v in scores.items()}
        # Communication special
        comm_raw = max(10, min(50, int(scores["Communication"])))
        comm_pct_table = {50:99, 40:70, 30:24, 20:10, 10:10} # Interpolated
        pcts["Communication"] = 10 + (comm_raw-10) * (99-10)/(50-10) # Simple linear interp

        # Ratios
        unb_coh = (pcts["Disengaged"] + pcts["Enmeshed"]) / 2.0
        coh_ratio = pcts["Balanced Cohesion"] / unb_coh if unb_coh > 0 else 1.0
        unb_flex = (pcts["Rigid"] + pcts["Chaotic"]) / 2.0
        flex_ratio = pcts["Balanced Flexibility"] / unb_flex if unb_flex > 0 else 1.0
        tot_ratio = (pcts["Balanced Cohesion"] + pcts["Balanced Flexibility"]) / (unb_coh + unb_flex) if (unb_coh + unb_flex) > 0 else 1.0

        # Dimension Scores (formal FACES IV calculation provided by user)
        # Formula: Balanced + (High_Unbalanced - Low_Unbalanced) / 2
        coh_dim = pcts["Balanced Cohesion"] + (pcts["Enmeshed"] - pcts["Disengaged"]) / 2.0
        flex_dim = pcts["Balanced Flexibility"] + (pcts["Chaotic"] - pcts["Rigid"]) / 2.0
        
        # Clamp Score: 5 (Score < 5), Score (5 <= Score <= 95), 95 (Score > 95)
        x = max(5.0, min(95.0, coh_dim))
        y = max(5.0, min(95.0, flex_dim))
        self._last_x, self._last_y = x, y

        # Gradient Descent for NEXT session target
        # Decouple: current state x,y is fixed as the outcome of the current turns
        # new_scores represents the targeted Percentile Scores for next session
        new_scores = self.calculate_gradient(pcts, x, y)
        
        # Target Dimension Scores for NEXT session (RED POINT)
        tx_next = new_scores["Balanced Cohesion"] + (new_scores["Enmeshed"] - new_scores["Disengaged"]) / 2.0
        ty_next = new_scores["Balanced Flexibility"] + (new_scores["Chaotic"] - new_scores["Rigid"]) / 2.0
        
        # DECISIVE STEERING BOOST
        # Ensure target is at least 15 points closer to 50 than the current result
        # and NEVER regresses towards the boundary compared to current result.
        def boost_target(curr, tgt):
            if curr < 50:
                # Must move towards 50. Ensure at least +15 displacement
                # or enough to cross the 15.0 boundary.
                lower_bound = max(curr + 15.0, 26.0) if curr <= 15.0 else curr + 10.0
                return max(tgt, lower_bound)
            elif curr > 50:
                # Must move towards 50. Ensure at least -15 displacement
                # or enough to cross the 85.0 boundary.
                upper_bound = min(curr - 15.0, 74.0) if curr >= 85.0 else curr - 10.0
                return min(tgt, upper_bound)
            return tgt

        tx = max(5.0, min(95.0, boost_target(x, tx_next)))
        ty = max(5.0, min(95.0, boost_target(y, ty_next)))

        self.update_history_with_targets(new_scores, tx, ty)

        # Trajectory
        traj = []
        if os.path.exists(self.TRAJECTORY_FILE):
            try:
                with open(self.TRAJECTORY_FILE, 'r') as f: traj = json.load(f)
            except: pass

        # GOAL MONOTONICITY: Ensure targets don't regress if family fails
        if traj:
            prev = traj[-1]
            ptx = prev.get("target_x")
            pty = prev.get("target_y")
            if ptx is not None:
                if x < 50 and ptx < 50: tx = max(tx, ptx)
                elif x > 50 and ptx > 50: tx = min(tx, ptx)
            if pty is not None:
                if y < 50 and pty < 50: ty = max(ty, pty)
                elif y > 50 and pty > 50: ty = min(ty, pty)

        if not traj:
            # Initialize with S0 target
            traj.append({
                "step": "S0",
                "target_x": self.initial_coords.get("x", 8.0),
                "target_y": self.initial_coords.get("y", 8.0)
            })

        traj.append({
            "step": step_id, 
            "result_x": x, "result_y": y,
            "target_x": tx, "target_y": ty
        })
        with open(self.TRAJECTORY_FILE, 'w') as f: json.dump(traj, f)
        
        # Phase 14: CSV Logging with Dimension Scores
        self.log_evaluation_to_csv(step_id, aggregated_results, ratings, pcts, x, y, tx, ty, coh_ratio, flex_ratio, tot_ratio, new_scores)

        self.get_logger().info(f"FACES IV Succeeded: Result({x:.1f}, {y:.1f}), Target({tx:.1f}, {ty:.1f})")
        self.generate_plot(x, y, coh_ratio, flex_ratio, tot_ratio, traj)

    def log_evaluation_to_csv(self, step_id, member_results, mean_ratings, current_pcts, x, y, tx, ty, coh_ratio, flex_ratio, tot_ratio, target_scores):
        csv_file = os.path.join(DB_DIR, "evaluation_history.csv")
        file_exists = os.path.exists(csv_file)
        
        try:
            with open(csv_file, 'a', encoding='utf-8', newline='') as f:
                writer = csv.writer(f)
                if not file_exists:
                    # Write Header
                    header = ["Timestamp", "StepID", "Cohesion_Dim", "Flexibility_Dim", "Target_X", "Target_Y", 
                             "Coh_Ratio", "Flex_Ratio", "Tot_Ratio"]
                    # Add summaries for target scores
                    for k in target_scores.keys():
                        header.append(f"Target_{k}")
                    # Add JSON blobs for details to keep CSV manageable but complete
                    header.extend(["Member_Raw_Scores_JSON", "Mean_Ratings_JSON"])
                    writer.writerow(header)
                
                row = [
                    datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
                    step_id,
                    round(x, 2), round(y, 2), round(tx, 2), round(ty, 2),
                    round(coh_ratio, 2), round(flex_ratio, 2), round(tot_ratio, 2)
                ]
                # Target Scores
                for k in target_scores.keys():
                    row.append(round(target_scores[k], 2))
                
                # Detailed JSON strings
                row.append(json.dumps(member_results, ensure_ascii=False))
                row.append(json.dumps(mean_ratings, ensure_ascii=False))
                
                writer.writerow(row)
            self.get_logger().info(f"Evaluation results logged to {csv_file}")
        except Exception as e:
            self.get_logger().error(f"Failed to log evaluation to CSV: {e}")

    def generate_plot(self, x, y, coh_ratio, flex_ratio, tot_ratio, trajectory=None, next_target=None):
        try:
            import matplotlib.pyplot as plt
            import matplotlib.patches as patches
        except ImportError:
            self.get_logger().error("Matplotlib is required for plotting. Please install it (pip install matplotlib).")
            return

        fig, ax = plt.subplots(figsize=(12, 12))
        # Fixed layout to prevent jitter between blinking frames
        fig.subplots_adjust(left=0.15, right=0.82, top=0.92, bottom=0.12)
        
        # Visual limits: 0 to 5 (5 blocks)
        ax.set_xlim(0, 5)
        ax.set_ylim(0, 5)
        
        # Draw 5x5 grid with gaps
        gap = 0.04
        rect_size = 1.0 - gap * 2
        
        for i in range(5): # X index (0-4)
            for j in range(5): # Y index (0-4)
                # Color determination
                if (i == 0 and j == 0) or (i == 0 and j == 4) or \
                   (i == 4 and j == 0) or (i == 4 and j == 4):
                    color = '#A9A9A9' # Dark corners
                elif 1 <= i <= 3 and 1 <= j <= 3:
                    color = 'white' # Center 3x3
                else:
                    color = '#D3D3D3' # Outer ring (Middle range)
                
                # Draw rectangles
                rect = patches.Rectangle(
                    (i + gap, j + gap), 
                    rect_size, 
                    rect_size, 
                    facecolor=color, 
                    edgecolor='black', # Optional cell boundary
                    linewidth=1
                )
                ax.add_patch(rect)

        # Labels
        # X-axis categories
        labels_x = [
            (0.5, "Disengaged"),
            (1.5, "Somewhat\nConnected"),
            (2.5, "Connected"),
            (3.5, "Very\nConnected"),
            (4.5, "Enmeshed")
        ]
        for pos, text in labels_x:
            ax.text(pos, -0.15, text, ha='center', va='top', fontsize=10, fontweight='bold')
        ax.set_xlabel("COHESION", fontsize=14, fontweight='bold', labelpad=30)

        # Y-axis categories
        labels_y = [
            (0.5, "Rigid"),
            (1.5, "Somewhat\nFlexible"),
            (2.5, "Flexible"),
            (3.5, "Very\nFlexible"),
            (4.5, "Chaotic")
        ]
        for pos, text in labels_y:
            ax.text(-0.15, pos, text, ha='right', va='center', rotation=0, fontsize=10, fontweight='bold')
        ax.set_ylabel("FLEXIBILITY", fontsize=14, fontweight='bold', labelpad=40)

        # Ticks
        # Base thresholds + requested additional ticks
        tick_vals = [0, 8, 15, 16, 25, 35, 36, 50, 65, 66, 75, 85, 86, 95, 100]
        tick_pos = [self.get_visual_coord(v) for v in tick_vals]
        tick_labels = [str(v) for v in tick_vals]
        
        ax.set_xticks(tick_pos)
        ax.set_xticklabels(tick_labels, fontsize=8, rotation=0)
        ax.set_yticks(tick_pos)
        ax.set_yticklabels(tick_labels, fontsize=8)

        # Title positioned absolutely within axes coordinates
        ax.text(0.5, 1.05, "FACES IV Circumplex Model", transform=ax.transAxes, 
                fontsize=16, fontweight='bold', ha='center', va='bottom')

        # Plot current position data label (always show in background to fix layout)
        if x is not None and y is not None:
            label_text = (
                f"Coord: ({x:.1f}, {y:.1f})\n"
                f"Cohesion Ratio: {coh_ratio:.2f}\n"
                f"Flexibility Ratio: {flex_ratio:.2f}\n"
                f"Total Ratio: {tot_ratio:.2f}"
            )
            ax.text(1.05, 0.95, label_text, transform=ax.transAxes, 
                    color='black', fontsize=10, fontweight='bold', 
                    va='top', ha='left',
                    bbox=dict(facecolor='white', alpha=0.9, edgecolor='black'))

        # Draw Trajectory
        if trajectory and len(trajectory) > 0:
            # 1. Plot S0 (Initial State)
            s0 = trajectory[0]
            s0_x = s0.get("target_x", s0.get("x", 8.0))
            s0_y = s0.get("target_y", s0.get("y", 8.0))
            v_s0_x = self.get_visual_coord(s0_x)
            v_s0_y = self.get_visual_coord(s0_y)
            
            # If S0 is the ONLY point, don't plot it in BG yet (we want it to blink)
            if len(trajectory) > 1:
                ax.plot(v_s0_x, v_s0_y, 'bo', markersize=10, label='Initial State (S0)', zorder=5)
            
            # Start tracking separately (Both start from S0)
            prev_rx, prev_ry = v_s0_x, v_s0_y
            prev_tx, prev_ty = v_s0_x, v_s0_y
            
            # 2. Iterate through subsequent steps
            if len(trajectory) > 1:
                for i in range(1, len(trajectory)):
                    step = trajectory[i]
                    # Actual Result (History)
                    rx = step.get("result_x", step.get("x"))
                    ry = step.get("result_y", step.get("y"))
                    # Target Point (History)
                    tx = step.get("target_x")
                    ty = step.get("target_y")

                    is_latest = (i == len(trajectory) - 1)

                    if not is_latest:
                        if rx is not None and ry is not None:
                            v_rx, v_ry = self.get_visual_coord(rx), self.get_visual_coord(ry)
                            ax.plot(v_rx, v_ry, 'bo', markersize=8, alpha=0.3, zorder=4)
                            ax.annotate("", xy=(v_rx, v_ry), xytext=(prev_rx, prev_ry),
                                        arrowprops=dict(arrowstyle="->", color='blue', linestyle=':', lw=2, alpha=0.3))
                            prev_rx, prev_ry = v_rx, v_ry
                    
                        if tx is not None and ty is not None:
                            v_tx, v_ty = self.get_visual_coord(tx), self.get_visual_coord(ty)
                            ax.plot(v_tx, v_ty, 'ro', markersize=8, alpha=0.3, zorder=5)
                            ax.annotate("", xy=(v_tx, v_ty), xytext=(prev_tx, prev_ty),
                                        arrowprops=dict(arrowstyle="->", color='red', lw=2, alpha=0.3))
                            prev_tx, prev_ty = v_tx, v_ty
                    else:
                        # LATEST STEP
                        # Current Result (BLUE) is now STATIC and FADED
                        if rx is not None and ry is not None:
                            v_rx, v_ry = self.get_visual_coord(rx), self.get_visual_coord(ry)
                            ax.plot(v_rx, v_ry, 'bo', markersize=14, alpha=0.4, markeredgecolor='black', label='Latest Evaluation', zorder=7)
                            ax.annotate("", xy=(v_rx, v_ry), xytext=(prev_rx, prev_ry),
                                        arrowprops=dict(arrowstyle="->", color='blue', linestyle=':', lw=2, alpha=0.4))

                        # Save BG (Contains everything EXCEPT the latest Red Target)
                        bg_save_path = os.path.join(DB_DIR, "evaluation_plot_bg.png")
                        plt.savefig(bg_save_path)

                        # Latest Target (RED) is now BLINKING
                        if tx is not None and ty is not None:
                            v_tx, v_ty = self.get_visual_coord(tx), self.get_visual_coord(ty)
                            ax.plot(v_tx, v_ty, 'ro', markersize=16, markeredgecolor='black', markeredgewidth=2, label='Therapeutic Target', zorder=8)
                            ax.annotate("", xy=(v_tx, v_ty), xytext=(prev_tx, prev_ty),
                                        arrowprops=dict(arrowstyle="->", color='red', lw=2, alpha=0.8))
            
            # Special case: If only S0 exists, it must blink in the foreground (BLUE)
            else:
                bg_save_path = os.path.join(DB_DIR, "evaluation_plot_bg.png")
                plt.savefig(bg_save_path)
                
                s0 = trajectory[0]
                sx = s0.get("target_x", s0.get("x", 8.0))
                sy = s0.get("target_y", s0.get("y", 8.0))
                v_sx, v_sy = self.get_visual_coord(sx), self.get_visual_coord(sy)
                ax.plot(v_sx, v_sy, 'bo', markersize=14, markeredgecolor='black', label='S0 Blinking', zorder=7)

        # If trajectory is empty, handle BG save for initialization
        if not trajectory:
            bg_save_path = os.path.join(DB_DIR, "evaluation_plot_bg.png")
            plt.savefig(bg_save_path)

        save_path = os.path.join(DB_DIR, "evaluation_plot.png")
        plt.savefig(save_path) # No tight bbox
        self.get_logger().info(f"Plot saved to: {save_path}")
        plt.close()

        # Notify plot viewer
        plot_msg = String()
        plot_msg.data = save_path
        self.plot_pub.publish(plot_msg)

    def get_visual_coord(self, score):
        # Map 0-100 scores to visual coordinates piece-wise linearly
        # Ensure specific mid-point values (8, 25, 50, 75, 95) align with visual centers (X.5).
        # Boundaries: [Start, Mid, End] -> [Start+Gap, X.5, End-Gap]
        
        gap = 0.04
        
        # Range definition: (BlockIndex, min_s, mid_s, max_s)
        ranges = [
            (0, 0, 8, 15),
            (1, 16, 25, 35),
            (2, 36, 50, 65),
            (3, 66, 75, 85),
            (4, 86, 95, 100)
        ]
        
        # Find which block the score belongs to
        target_range = None
        for r in ranges:
            if r[1] <= score <= r[3]:
                target_range = r
                break
        
        if target_range is None:
            # Fallback or clamp
            if score < 0: target_range = ranges[0]
            else: target_range = ranges[4]

        idx, min_s, mid_s, max_s = target_range
        
        # Visual coordinates for this block
        vis_start = idx + gap
        vis_mid = idx + 0.5
        vis_end = idx + 1.0 - gap
        
        if score <= mid_s:
            # Mapping [min_s, mid_s] -> [vis_start, vis_mid]
            if mid_s == min_s: return vis_start # Avoid division by zero
            norm = (score - min_s) / (mid_s - min_s)
            return vis_start + norm * (vis_mid - vis_start)
        else:
            # Mapping [mid_s, max_s] -> [vis_mid, vis_end]
            if max_s == mid_s: return vis_end
            norm = (score - mid_s) / (max_s - mid_s)
            return vis_mid + norm * (vis_end - vis_mid)

    def calculate_gradient(self, pcts, x, y):
        # Extract current values
        c_bal = pcts["Balanced Cohesion"]
        f_bal = pcts["Balanced Flexibility"]
        c_dis = pcts["Disengaged"]
        c_enm = pcts["Enmeshed"]
        f_rig = pcts["Rigid"]
        f_cha = pcts["Chaotic"]
        comm = pcts["Communication"]

        # Calculate helper variables
        B = c_bal + f_bal
        U = c_dis + c_enm + f_rig + f_cha
        
        # Learning rate eta (Ultra-aggressive for decisive steering)
        # Using a base floor of 0.4 and scaling by communication/target_scaling
        eta = max(0.4, comm / 100.0) * self.LEARNING_RATE_SCALING * 2.5
        
        # Boundary Escape Factor: If we are at the edge, double the steering force
        # to ensure targets are visibly different even from extreme unbalance.
        boundary_dist = min(abs(x - 5.0), abs(x - 95.0), abs(y - 5.0), abs(y - 95.0))
        if boundary_dist < 10.0:
            eta *= 2.0 
        
        # Calculate gradients 
        # Objective J = w1*(U/2B) - w2*Comm + w3*0.5*((x-50)^2 + (y-50)^2)
        # Update gradients to use the formal Dimension Score derivatives (1/2 coefficient for unbalanced)
        grad_bal_prefix = - (self.OMEGA_1 * U) / (2.0 * B**2)
        grad_c_bal = grad_bal_prefix + self.OMEGA_3 * (x - 50.0)
        grad_f_bal = grad_bal_prefix + self.OMEGA_3 * (y - 50.0)
        
        grad_unbal_prefix = self.OMEGA_1 / (2.0 * B)
        # grad_high_unbal -> (1/2), grad_low_unbal -> (-1/2) matching formula derivatives
        grad_c_enm = grad_unbal_prefix + (self.OMEGA_3 / 2.0) * (x - 50.0)
        grad_c_dis = grad_unbal_prefix - (self.OMEGA_3 / 2.0) * (x - 50.0)
        grad_f_cha = grad_unbal_prefix + (self.OMEGA_3 / 2.0) * (y - 50.0)
        grad_f_rig = grad_unbal_prefix - (self.OMEGA_3 / 2.0) * (y - 50.0)
        
        grad_comm = - self.OMEGA_2
        
        # Update amounts
        delta_c_bal = - eta * grad_c_bal
        delta_f_bal = - eta * grad_f_bal
        delta_c_dis = - eta * grad_c_dis
        delta_c_enm = - eta * grad_c_enm
        delta_f_rig = - eta * grad_f_rig
        delta_f_cha = - eta * grad_f_cha
        delta_comm = - eta * grad_comm

        # Adjacency constraint
        def get_cell_idx(v):
            if v <= 15: return 0
            if v <= 35: return 1
            if v <= 65: return 2
            if v <= 85: return 3
            return 4
        
        next_x_raw = (c_bal + delta_c_bal) + ((c_enm + delta_c_enm) - (c_dis + delta_c_dis)) / 2.0
        next_y_raw = (f_bal + delta_f_bal) + ((f_cha + delta_f_cha) - (f_rig + delta_f_rig)) / 2.0
        
        curr_i, curr_j = get_cell_idx(x), get_cell_idx(y)
        ranges = [(0, 15), (16, 35), (36, 65), (66, 85), (86, 100)]
        low_x, high_x = ranges[max(0, curr_i - 1)][0], ranges[min(4, curr_i + 1)][1]
        low_y, high_y = ranges[max(0, curr_j - 1)][0], ranges[min(4, curr_j + 1)][1]
        
        alpha = 1.0
        dx, dy = next_x_raw - x, next_y_raw - y
        if dx != 0:
            if x + dx > high_x: alpha = min(alpha, (high_x - x) / dx)
            if x + dx < low_x:  alpha = min(alpha, (low_x - x) / dx)
        if dy != 0:
            if y + dy > high_y: alpha = min(alpha, (high_y - y) / dy)
            if y + dy < low_y:  alpha = min(alpha, (low_y - y) / dy)
            
        if alpha < 1.0:
            delta_c_bal *= alpha; delta_f_bal *= alpha; delta_c_dis *= alpha
            delta_c_enm *= alpha; delta_f_rig *= alpha; delta_f_cha *= alpha; delta_comm *= alpha

        new_scores = {
            "Balanced Cohesion": max(5.0, min(99.0, c_bal + delta_c_bal)),
            "Balanced Flexibility": max(5.0, min(99.0, f_bal + delta_f_bal)),
            "Disengaged": max(5.0, min(99.0, c_dis + delta_c_dis)),
            "Enmeshed": max(5.0, min(99.0, c_enm + delta_c_enm)),
            "Rigid": max(5.0, min(99.0, f_rig + delta_f_rig)),
            "Chaotic": max(5.0, min(99.0, f_cha + delta_f_cha)),
            "Communication": max(5.0, min(99.0, comm + delta_comm))
        }

        # FINAL ADJUSTMENT: Decisive Category-Crossing
        # Ensure the resulting Dim Scores (next_x, next_y) are at least N units closer to 50
        # or cross into the next behavioral category if the current result is extreme.
        def apply_steering_boost(current_v, target_v):
            if current_v < 50:
                # If in extreme zone (0-15), force target to at least 25
                if current_v <= 15: return max(target_v, 25.0)
                # Otherwise ensure at least +10 displacement
                return max(target_v, current_v + 10.0)
            elif current_v > 50:
                # If in extreme zone (85-100), force target to at least 75
                if current_v >= 85: return min(target_v, 75.0)
                # Otherwise ensure at least -10 displacement
                return min(target_v, current_v - 10.0)
            return target_v

        # We can't directly adjust new_scores easily without recalculating,
        # so we will adjust the final tx, ty in calculate_scores if needed,
        # OR we just increase the displacement here by scaling the deltas.
        
        return new_scores

    def update_history_with_targets(self, scores, tx, ty):
        # Already calculated in calculate_scores with steering boost
        
        update = f"\n[THERAPIST_STALL_SESSION_ANALYSIS]\n"
        update += f"Current Result Position: ({self._last_x:.1f}, {self._last_y:.1f})\n" if hasattr(self, '_last_x') else ""
        update += f"Determined Therapeutic Target for Next Session: ({tx:.1f}, {ty:.1f})\n"
        update += "Targeted FACES IV Percentile Scores (Steering towards center):\n"
        for k, v in scores.items(): update += f"- {k}: {int(v)}\n"
        update += "(Strategic Objective: Aggressively maneuver family towards the 'Balanced' zone (50, 50))\n"
        with open(HISTORY_FILE, "a", encoding="utf-8") as f: f.write(update)

    def family_actions_callback(self, msg: String): pass
    def user_intervention_callback(self, msg: String): pass

def main():
    rclpy.init()
    node = RFSTherapist()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
