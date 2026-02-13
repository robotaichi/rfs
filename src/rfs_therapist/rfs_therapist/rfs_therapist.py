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
import time
import shutil
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
        self.archival_lock = threading.Lock()
        
        self.TRAJECTORY_FILE = os.path.join(DB_DIR, "evaluation_trajectory.json")
        self.OMEGA_1 = 0.1
        self.OMEGA_2 = 0.1
        self.OMEGA_3 = 0.05 # Suppress center pull
        self.LEARNING_RATE_SCALING = 0.005 # Increased for faster progression
        self.family_config = []
        self.faces_tables = {}
        
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
        self.faces_tables = self._load_faces_tables()

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
            tx = last.get("target_x")
            ty = last.get("target_y")
            self.generate_plot(x, y, 1.0, 1.0, 1.0, traj, tx=tx, ty=ty)
        else:
            # Show S0 from initial_coords
            s0_x = self.initial_coords.get("x", 8.0)
            s0_y = self.initial_coords.get("y", 8.0)
            self.generate_plot(s0_x, s0_y, 1.0, 1.0, 1.0, [{"target_x": s0_x, "target_y": s0_y}])

    def _load_config(self):
        try:
            if os.path.exists(CONFIG_FILE):
                with open(CONFIG_FILE, 'r', encoding='utf-8') as f:
                    config = json.load(f)
                    self.family_config = config.get("family_config", [])
                    self.initial_coords = config.get("initial_coords", {"x": 8.0, "y": 8.0})
        except: pass

    def _load_faces_tables(self):
        tables = {"cohesion": {}, "flexibility": {}, "communication": {}}
        path = os.path.join(DB_DIR, "faces_iv_tables.md")
        if not os.path.exists(path): return tables
        
        current_section = None
        try:
            with open(path, 'r', encoding='utf-8') as f:
                for line in f:
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

    def _get_detailed_behavior(self, x, y):
        def get_col_idx(val):
            if val <= 15: return 0
            if val <= 35: return 1
            if val <= 65: return 2
            if val <= 85: return 3
            return 4
        
        c_idx = get_col_idx(x)
        f_idx = get_col_idx(y)
        
        lines = []
        if "cohesion" in self.faces_tables:
            for cat, rows in self.faces_tables["cohesion"].items():
                if len(rows) > c_idx: lines.append(f"- {cat}: {rows[c_idx]}")
        if "flexibility" in self.faces_tables:
            for cat, rows in self.faces_tables["flexibility"].items():
                if len(rows) > f_idx: lines.append(f"- {cat}: {rows[f_idx]}")
        return "\n".join(lines)

    def trigger_callback(self, msg: String):
        step_id = msg.data
        if step_id in self.processed_steps: return
        if step_id in self.member_results: return
            
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
                
                if received >= total:
                    if step_id not in self.processed_steps:
                        # DELEGATE TO EVALUATOR
                        req = {"step_id": step_id, "results": self.member_results[step_id]}
                        self.evaluator_req_pub.publish(String(data=json.dumps(req)))
        except Exception as e:
            self.get_logger().error(f"Error in evaluation aggregation: {e}")

    def evaluator_result_callback(self, msg: String):
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
                if val <= 15: return labels[0]
                if val <= 35: return labels[1]
                if val <= 65: return labels[2]
                if val <= 85: return labels[3]
                return labels[4]
            
            coh_label = get_label(x, ["Disengaged", "Somewhat Connected", "Connected", "Very Connected", "Enmeshed"])
            flex_label = get_label(y, ["Rigid", "Somewhat Flexible", "Flexible", "Very Flexible", "Chaotic"])
            family_type = f"{coh_label}-{flex_label}"
            
            self.get_logger().info(f"[{self.role}] Current Family Type: {family_type} at ({x:.1f}, {y:.1f})")
            details = self._get_detailed_behavior(x, y)
            if details:
                self.get_logger().info(f"[{self.role}] Clinical Behavioral Descriptions:\n{details}")
            
            self.update_history_with_targets(target_scores, tx, ty)

            # Trajectory update
            traj = []
            if os.path.exists(self.TRAJECTORY_FILE):
                try:
                    with open(self.TRAJECTORY_FILE, 'r') as f: traj = json.load(f)
                except: pass

            if not traj:
                traj.append({"step": "S0", "target_x": self.initial_coords.get("x", 8.0), "target_y": self.initial_coords.get("y", 8.0)})

            traj.append({"step": step_id, "result_x": x, "result_y": y, "target_x": tx, "target_y": ty})
            with open(self.TRAJECTORY_FILE, 'w') as f: json.dump(traj, f)
            
            # Logging
            self.log_evaluation_to_csv(step_id, self.member_results[step_id], mean_ratings, pcts, x, y, tx, ty, coh_ratio, flex_ratio, tot_ratio, target_scores)
            
            self.get_logger().info(f"FACES IV Succeeded: Result({x:.1f}, {y:.1f}), Target({tx:.1f}, {ty:.1f})")
            self.generate_plot(x, y, coh_ratio, flex_ratio, tot_ratio, traj, tx=tx, ty=ty)
            
            # Finalize step
            self.complete_pub.publish(String(data=step_id))
            
        except Exception as e:
            self.get_logger().error(f"Error processing evaluator results: {e}")

    def log_evaluation_to_csv(self, step_id, member_results, mean_ratings, current_pcts, x, y, tx, ty, coh_ratio, flex_ratio, tot_ratio, target_scores):
        csv_file = os.path.join(DB_DIR, "evaluation_history.csv")
        file_exists = os.path.exists(csv_file)
        
        try:
            with open(csv_file, 'a', encoding='utf-8', newline='') as f:
                writer = csv.writer(f)
                if not file_exists:
                    header = ["Timestamp", "StepID", "Cohesion_Dim", "Flexibility_Dim", "Target_X", "Target_Y", "Coh_Ratio", "Flex_Ratio", "Tot_Ratio"]
                    for k in target_scores.keys(): header.append(f"Target_{k}")
                    header.extend(["Member_Raw_Scores_JSON", "Mean_Ratings_JSON"])
                    writer.writerow(header)
                
                row = [datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S"), step_id, round(x, 2), round(y, 2), round(tx, 2), round(ty, 2), round(coh_ratio, 2), round(flex_ratio, 2), round(tot_ratio, 2)]
                for k in target_scores.keys(): row.append(round(target_scores[k], 2))
                row.append(json.dumps(member_results, ensure_ascii=False))
                row.append(json.dumps(mean_ratings, ensure_ascii=False))
                writer.writerow(row)
        except Exception as e:
            self.get_logger().error(f"Failed to log evaluation to CSV: {e}")

    def generate_plot(self, x, y, coh_ratio, flex_ratio, tot_ratio, trajectory=None, tx=None, ty=None):
        try:
            import matplotlib.pyplot as plt
            import matplotlib.patches as patches
        except: return

        fig, ax = plt.subplots(figsize=(12, 12))
        fig.subplots_adjust(left=0.15, right=0.82, top=0.92, bottom=0.12)
        ax.set_xlim(0, 5); ax.set_ylim(0, 5)
        
        gap = 0.04
        rect_size = 1.0 - gap * 2
        for i in range(5):
            for j in range(5):
                if (i == 0 and j == 0) or (i == 0 and j == 4) or (i == 4 and j == 0) or (i == 4 and j == 4): color = '#A9A9A9'
                elif 1 <= i <= 3 and 1 <= j <= 3: color = 'white'
                else: color = '#D3D3D3'
                rect = patches.Rectangle((i + gap, j + gap), rect_size, rect_size, facecolor=color, edgecolor='black', linewidth=1)
                ax.add_patch(rect)

        labels_x = [(0.5, "Disengaged"), (1.5, "Somewhat\nConnected"), (2.5, "Connected"), (3.5, "Very\nConnected"), (4.5, "Enmeshed")]
        for pos, text in labels_x: ax.text(pos, -0.15, text, ha='center', va='top', fontsize=10, fontweight='bold')
        ax.set_xlabel("COHESION", fontsize=14, fontweight='bold', labelpad=30)

        labels_y = [(0.5, "Rigid"), (1.5, "Somewhat\nFlexible"), (2.5, "Flexible"), (3.5, "Very\nFlexible"), (4.5, "Chaotic")]
        for pos, text in labels_y: ax.text(-0.15, pos, text, ha='right', va='center', rotation=0, fontsize=10, fontweight='bold')
        ax.set_ylabel("FLEXIBILITY", fontsize=14, fontweight='bold', labelpad=40)

        tick_vals = [0, 8, 15, 16, 25, 35, 36, 50, 65, 66, 75, 85, 86, 95, 100]
        tick_pos = [self.get_visual_coord(v) for v in tick_vals]
        ax.set_xticks(tick_pos); ax.set_xticklabels([str(v) for v in tick_vals], fontsize=8); ax.set_yticks(tick_pos); ax.set_yticklabels([str(v) for v in tick_vals], fontsize=8)
        ax.text(0.5, 1.05, "FACES IV Circumplex Model", transform=ax.transAxes, fontsize=16, fontweight='bold', ha='center', va='bottom')

        if x is not None and y is not None:
            tx_val = tx if tx is not None else 0.0
            ty_val = ty if ty is not None else 0.0
            label_text = f"Result (Blue): ({x:.1f}, {y:.1f})\nNext Target (Red): ({tx_val:.1f}, {ty_val:.1f})\nCoh Ratio: {coh_ratio:.2f}\nFlex Ratio: {flex_ratio:.2f}\nTot Ratio: {tot_ratio:.2f}"
            ax.text(1.05, 0.95, label_text, transform=ax.transAxes, color='black', fontsize=10, fontweight='bold', va='top', ha='left', bbox=dict(facecolor='white', alpha=0.9, edgecolor='black'))

        if trajectory:
            s0 = trajectory[0]
            prev_tx = s0.get("target_x", 8.0); prev_ty = s0.get("target_y", 8.0)
            v_prev_tx, v_prev_ty = self.get_visual_coord(prev_tx), self.get_visual_coord(prev_ty)
            if len(trajectory) > 1: ax.plot(v_prev_tx, v_prev_ty, 'bo', markersize=8, alpha=0.6, zorder=5)

            for i in range(1, len(trajectory)):
                step = trajectory[i]
                rx, ry = step.get("result_x"), step.get("result_y")
                tax, tay = step.get("target_x"), step.get("target_y")
                if rx is not None and ry is not None:
                    v_rx, v_ry = self.get_visual_coord(rx), self.get_visual_coord(ry)
                    ax.annotate("", xy=(v_rx, v_ry), xytext=(v_prev_tx, v_prev_ty), arrowprops=dict(arrowstyle="->", linestyle=':', color='gray', lw=2, alpha=0.6))
                    if not (i == len(trajectory) - 1 and (tax is None or tay is None)): ax.plot(v_rx, v_ry, 'bo', markersize=10, markeredgecolor='black', zorder=7)
                    if tax is not None and tay is not None:
                        v_tax, v_tay = self.get_visual_coord(tax), self.get_visual_coord(tay)
                        ax.annotate("", xy=(v_tax, v_tay), xytext=(v_rx, v_ry), arrowprops=dict(arrowstyle="->", color='red', lw=2.5, alpha=0.8))
                        if i < len(trajectory) - 1: ax.plot(v_tax, v_tay, 'ro', markersize=8, markeredgecolor='black', zorder=8)
                        v_prev_tx, v_prev_ty = v_tax, v_tay

        from matplotlib.lines import Line2D
        legend_elements = [
            Line2D([0], [0], marker='o', color='w', label='Result (Blue)', markerfacecolor='b', markersize=10, markeredgecolor='black'),
            Line2D([0], [0], marker='o', color='w', label='Target (Red)', markerfacecolor='r', markersize=10, markeredgecolor='black'),
            Line2D([0], [0], color='gray', linestyle=':', label='Actual Move (Dotted)'),
            Line2D([0], [0], color='red', label='Therapeutic Goal (Solid)')
        ]
        ax.legend(handles=legend_elements, loc='lower left', bbox_to_anchor=(1.02, 0))

        bg_save_path = os.path.join(DB_DIR, "evaluation_plot_bg.png")
        plt.savefig(bg_save_path)
        if trajectory:
            last_step = trajectory[-1]
            if last_step.get("target_x") is not None:
                v_lx, v_ly = self.get_visual_coord(last_step["target_x"]), self.get_visual_coord(last_step["target_y"])
                ax.plot(v_lx, v_ly, 'ro', markersize=16, markeredgecolor='black', zorder=10)
            elif last_step.get("result_x") is not None:
                v_lx, v_ly = self.get_visual_coord(last_step["result_x"]), self.get_visual_coord(last_step["result_y"])
                ax.plot(v_lx, v_ly, 'bo', markersize=16, markeredgecolor='black', zorder=10)
        
        save_path = os.path.join(DB_DIR, "evaluation_plot.png")
        plt.savefig(save_path); plt.close()
        self.plot_pub.publish(String(data=save_path))

    def get_visual_coord(self, score):
        gap = 0.04
        ranges = [(0, 0, 8, 15), (1, 16, 25, 35), (2, 36, 50, 65), (3, 66, 75, 85), (4, 86, 95, 100)]
        target_range = None
        for r in ranges:
            if r[1] <= score <= r[3]: target_range = r; break
        if target_range is None: target_range = ranges[0] if score < 0 else ranges[4]
        idx, min_s, mid_s, max_s = target_range
        vis_start, vis_mid, vis_end = idx + gap, idx + 0.5, idx + 1.0 - gap
        if score <= mid_s:
            if mid_s == min_s: return vis_start
            return vis_start + (score - min_s) / (mid_s - min_s) * (vis_mid - vis_start)
        else:
            if max_s == mid_s: return vis_end
            return vis_mid + (score - mid_s) / (max_s - mid_s) * (vis_end - vis_mid)

    def update_history_with_targets(self, scores, tx, ty):
        update = f"\n[THERAPIST_STALL_SESSION_ANALYSIS]\n"
        update += f"Determined Therapeutic Target for Next Session: ({tx:.1f}, {ty:.1f})\n"
        update += "Targeted FACES IV Percentile Scores (Steering towards center):\n"
        for k, v in scores.items(): update += f"- {k}: {int(v)}\n"
        update += "(Strategic Objective: Aggressively maneuver family towards the 'Balanced' zone (50, 50))\n"
        with open(HISTORY_FILE, "a", encoding="utf-8") as f:
            f.write(update)
            f.flush()
            try: os.fsync(f.fileno())
            except: pass

    def family_actions_callback(self, msg: String): pass
    def user_intervention_callback(self, msg: String): pass

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
