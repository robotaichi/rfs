#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import os

class RFSOptimizer(Node):
    def __init__(self):
        super().__init__('rfs_optimizer')
        
        self.OMEGA_1 = 0.1
        self.OMEGA_2 = 0.1
        self.OMEGA_3 = 0.05
        self.LEARNING_RATE_SCALING = 0.005
        
        self.create_subscription(String, 'rfs_optimizer_request', self.request_callback, 10)
        self.result_pub = self.create_publisher(String, 'rfs_evaluator_results', 10)
        
        self._load_config()
        self.get_logger().info(f"RFS Optimizer Node Started. LR={self.LEARNING_RATE_SCALING}")

    def _load_config(self):
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
        except: pass

    def request_callback(self, msg: String):
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
            
            # Perform Gradient Descent
            target_scores = self.calculate_gradient(pcts, x, y)
            
            # Target Dimension Scores for NEXT session
            tx = target_scores["Balanced Cohesion"] + (target_scores["Enmeshed"] - target_scores["Disengaged"]) / 2.0
            ty = target_scores["Balanced Flexibility"] + (target_scores["Chaotic"] - target_scores["Rigid"]) / 2.0
            tx = max(5.0, min(95.0, tx))
            ty = max(5.0, min(95.0, ty))
            
            # Combine all for final result
            final_results = {
                "step_id": step_id,
                "x": x, "y": y,
                "tx": tx, "ty": ty,
                "pcts": pcts,
                "target_scores": target_scores,
                "coh_ratio": coh_ratio,
                "flex_ratio": flex_ratio,
                "tot_ratio": tot_ratio,
                "mean_ratings": mean_ratings
            }
            
            self.result_pub.publish(String(data=json.dumps(final_results)))
            self.get_logger().info(f"Optimization calculated for {step_id}: Target({tx:.1f}, {ty:.1f})")
            
        except Exception as e:
            self.get_logger().error(f"Error in optimizer request processing: {e}")

    def calculate_gradient(self, pcts, x, y):
        c_bal = pcts["Balanced Cohesion"]
        f_bal = pcts["Balanced Flexibility"]
        c_dis = pcts["Disengaged"]
        c_enm = pcts["Enmeshed"]
        f_rig = pcts["Rigid"]
        f_cha = pcts["Chaotic"]
        
        # Baseline Reset / Shift
        raw_x = c_bal + (c_enm - c_dis) / 2.0
        raw_y = f_bal + (f_cha - f_rig) / 2.0

        if raw_x < 5.0: c_bal += (5.0 - raw_x)
        elif raw_x > 95.0: c_bal += (95.0 - raw_x)

        if raw_y < 5.0: f_bal += (5.0 - raw_y)
        elif raw_y > 95.0: f_bal += (95.0 - raw_y)
        
        eta = self.LEARNING_RATE_SCALING
        U_coh = c_dis + c_enm
        U_flex = f_rig + f_cha
        
        grad_c_bal = - (self.OMEGA_1 * U_coh) / (2.0 * max(1.0, c_bal**2)) + self.OMEGA_3 * (x - 50.0)
        grad_f_bal = - (self.OMEGA_2 * U_flex) / (2.0 * max(1.0, f_bal**2)) + self.OMEGA_3 * (y - 50.0)
        
        grad_c_unbal_prefix = self.OMEGA_1 / (2.0 * max(1.0, c_bal))
        grad_f_unbal_prefix = self.OMEGA_2 / (2.0 * max(1.0, f_bal))

        grad_c_enm = grad_c_unbal_prefix + (self.OMEGA_3 / 2.0) * (x - 50.0)
        grad_c_dis = grad_c_unbal_prefix - (self.OMEGA_3 / 2.0) * (x - 50.0)
        grad_f_cha = grad_f_unbal_prefix + (self.OMEGA_3 / 2.0) * (y - 50.0)
        grad_f_rig = grad_f_unbal_prefix - (self.OMEGA_3 / 2.0) * (y - 50.0)
        
        def get_cell_idx(v):
            if v <= 15: return 0
            if v <= 35: return 1
            if v <= 65: return 2
            if v <= 85: return 3
            return 4
        
        delta_c_bal = - eta * grad_c_bal
        delta_f_bal = - eta * grad_f_bal
        delta_c_dis = - eta * grad_c_dis
        delta_c_enm = - eta * grad_c_enm
        delta_f_rig = - eta * grad_f_rig
        delta_f_cha = - eta * grad_f_cha
        
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
            delta_c_enm *= alpha; delta_f_rig *= alpha; delta_f_cha *= alpha

        return {
            "Balanced Cohesion": max(5.0, min(99.0, c_bal + delta_c_bal)),
            "Balanced Flexibility": max(5.0, min(99.0, f_bal + delta_f_bal)),
            "Disengaged": max(5.0, min(99.0, c_dis + delta_c_dis)),
            "Enmeshed": max(5.0, min(99.0, c_enm + delta_c_enm)),
            "Rigid": max(5.0, min(99.0, f_rig + delta_f_rig)),
            "Chaotic": max(5.0, min(99.0, f_cha + delta_f_cha)),
            "Communication": pcts["Communication"] + eta * self.OMEGA_2
        }

def main(args=None):
    rclpy.init(args=args)
    node = RFSOptimizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
