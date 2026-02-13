#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import os

class RFSEvaluator(Node):
    def __init__(self):
        super().__init__('rfs_evaluator')
        
        self.OMEGA_1 = 0.1
        self.OMEGA_2 = 0.1
        self.OMEGA_3 = 0.05
        self.LEARNING_RATE_SCALING = 0.005
        
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

        self.create_subscription(String, 'rfs_evaluator_request', self.request_callback, 10)
        self.result_pub = self.create_publisher(String, 'rfs_evaluator_results', 10)
        self.optimizer_req_pub = self.create_publisher(String, 'rfs_optimizer_request', 10)
        
        # Load config to sync parameters
        self._load_config()
        self.get_logger().info(f"RFS Evaluator Node Started.")

    def _load_config(self):
        home = os.path.expanduser("~")
        config_file = os.path.join(home, "rfs/src/rfs_config/config/config.json")
        try:
            if os.path.exists(config_file):
                with open(config_file, 'r', encoding='utf-8') as f:
                    config = json.load(f)
                    # OMEGAs are now primarily for the Optimizer, but kept for consistency
        except: pass

    def request_callback(self, msg: String):
        try:
            data = json.loads(msg.data)
            step_id = data.get("step_id")
            aggregated_results = data.get("results")
            
            # 1. Average ratings
            ratings = {}
            for item_id in range(1, 63):
                item_scores = []
                for role, results in aggregated_results.items():
                    score = results.get(str(item_id))
                    if isinstance(score, dict): score = score.get("rating")
                    if score is not None: item_scores.append(float(score))
                ratings[item_id] = sum(item_scores)/len(item_scores) if item_scores else 3.0
            
            # 2. Calculate scores
            results = self.calculate_scores(ratings)
            results["step_id"] = step_id
            
            # 3. Delegate to OPTIMIZER for next target
            self.optimizer_req_pub.publish(String(data=json.dumps(results)))
            self.get_logger().info(f"Scores calculated for {step_id}: Result({results['x']:.1f}, {results['y']:.1f}). Delegating to Optimizer.")
            
        except Exception as e:
            self.get_logger().error(f"Error in evaluator request processing: {e}")

    def calculate_scores(self, ratings):
        scores = {k: sum(ratings.get(i, 3.0) for i in items) for k, items in self.scales.items()}
        
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
        comm_raw = max(10, min(50, int(scores["Communication"])))
        pcts["Communication"] = 10 + (comm_raw-10) * (99-10)/(50-10)

        # Ratios
        unb_coh = (pcts["Disengaged"] + pcts["Enmeshed"]) / 2.0
        coh_ratio = pcts["Balanced Cohesion"] / unb_coh if unb_coh > 0 else 1.0
        unb_flex = (pcts["Rigid"] + pcts["Chaotic"]) / 2.0
        flex_ratio = pcts["Balanced Flexibility"] / unb_flex if unb_flex > 0 else 1.0
        tot_ratio = (pcts["Balanced Cohesion"] + pcts["Balanced Flexibility"]) / (unb_coh + unb_flex) if (unb_coh + unb_flex) > 0 else 1.0

        # Dimension Scores
        coh_dim = pcts["Balanced Cohesion"] + (pcts["Enmeshed"] - pcts["Disengaged"]) / 2.0
        flex_dim = pcts["Balanced Flexibility"] + (pcts["Chaotic"] - pcts["Rigid"]) / 2.0
        
        x = max(5.0, min(95.0, coh_dim))
        y = max(5.0, min(95.0, flex_dim))

        return {
            "x": x, "y": y,
            "pcts": pcts,
            "coh_ratio": coh_ratio, "flex_ratio": flex_ratio, "tot_ratio": tot_ratio,
            "mean_ratings": ratings
        }

def main(args=None):
    rclpy.init(args=args)
    node = RFSEvaluator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
