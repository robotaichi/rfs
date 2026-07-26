#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Logic layer for visualizing and logging the FACES IV circumplex model.

A plain Python class with no ROS2 dependency. Handles plot image generation using
matplotlib, loading the clinical behavior description tables, logging each session
to CSV, and appending targets to the history file. Called from the rfs_therapist node.
"""

import csv
import datetime
import os


class FacesReportGenerator:
    """Logic class that handles FACES IV visualization and logging (no rclpy dependency)."""

    def __init__(self, logger, db_dir: str):
        self.logger = logger
        self.db_dir = db_dir
        self.faces_tables = self._load_faces_tables()

    def _load_faces_tables(self) -> dict:
        """Load the clinical behavior description tables from `faces_iv_tables.md` (Markdown table format)."""
        tables = {"cohesion": {}, "flexibility": {}, "communication": {}}
        path = os.path.join(self.db_dir, "faces_iv_tables.md")
        if not os.path.exists(path):
            return tables

        current_section = None
        try:
            with open(path, 'r', encoding='utf-8') as f:
                for line in f:
                    line = line.strip()
                    if "## 1. Cohesion" in line:
                        current_section = "cohesion"
                    elif "## 2. Flexibility" in line:
                        current_section = "flexibility"
                    elif "## 3. Communication" in line:
                        current_section = "communication"

                    if line.startswith("|"):
                        parts = [p.strip() for p in line.split("|")]
                        if parts and not parts[0]:
                            parts.pop(0)
                        if parts and not parts[-1]:
                            parts.pop()

                        if len(parts) < 2 or "---" in parts[0] or "Level" in parts[1] or "Low" in parts[1]:
                            continue

                        cat = parts[0].replace("**", "")
                        if current_section:
                            tables[current_section][cat] = parts[1:]
        except Exception as e:
            self.logger.error(f"Failed to load FACES tables: {e}")
        return tables

    def get_detailed_behavior(self, x: float, y: float) -> str:
        """Return the clinical behavior description (cohesion/flexibility) text for coordinates (x, y)."""

        def get_col_idx(val):
            if val <= 15:
                return 0
            if val <= 35:
                return 1
            if val <= 65:
                return 2
            if val <= 85:
                return 3
            return 4

        c_idx = get_col_idx(x)
        f_idx = get_col_idx(y)

        lines = []
        if "cohesion" in self.faces_tables:
            for cat, rows in self.faces_tables["cohesion"].items():
                if len(rows) > c_idx:
                    lines.append(f"- {cat}: {rows[c_idx]}")
        if "flexibility" in self.faces_tables:
            for cat, rows in self.faces_tables["flexibility"].items():
                if len(rows) > f_idx:
                    lines.append(f"- {cat}: {rows[f_idx]}")
        return "\n".join(lines)

    def get_visual_coord(self, score: float) -> float:
        """Convert a FACES IV percentile value (0-100) into a coordinate (0-5) for the 5x5 grid drawing.

        Within each segment (bounded by 0/16/36/66/86), the value is spread smoothly from min to mid to max.
        """
        gap = 0.04
        ranges = [(0, 0, 8, 15), (1, 16, 25, 35), (2, 36, 50, 65), (3, 66, 75, 85), (4, 86, 95, 100)]
        target_range = None
        for r in ranges:
            if r[1] <= score <= r[3]:
                target_range = r
                break
        if target_range is None:
            target_range = ranges[0] if score < 0 else ranges[4]
        idx, min_s, mid_s, max_s = target_range
        vis_start, vis_mid, vis_end = idx + gap, idx + 0.5, idx + 1.0 - gap
        if score <= mid_s:
            if mid_s == min_s:
                return vis_start
            return vis_start + (score - min_s) / (mid_s - min_s) * (vis_mid - vis_start)
        else:
            if max_s == mid_s:
                return vis_end
            return vis_mid + (score - mid_s) / (max_s - mid_s) * (vis_end - vis_mid)

    def generate_plot(self, x, y, coh_ratio, flex_ratio, tot_ratio, trajectory=None, tx=None, ty=None):
        """Generate the FACES IV circumplex model plot image and return the saved file path.

        Returns None if matplotlib is unavailable (the caller should then skip publishing).
        """
        try:
            import matplotlib.pyplot as plt
            import matplotlib.patches as patches
        except Exception:
            return None

        fig, ax = plt.subplots(figsize=(12, 12))
        fig.subplots_adjust(left=0.15, right=0.82, top=0.92, bottom=0.12)
        ax.set_xlim(0, 5)
        ax.set_ylim(0, 5)

        # Color the 5x5 grid background (corners = extreme type gray, center 3x3 = balanced type white, rest = mid gray)
        gap = 0.04
        rect_size = 1.0 - gap * 2
        for i in range(5):
            for j in range(5):
                if (i == 0 and j == 0) or (i == 0 and j == 4) or (i == 4 and j == 0) or (i == 4 and j == 4):
                    color = '#A9A9A9'
                elif 1 <= i <= 3 and 1 <= j <= 3:
                    color = 'white'
                else:
                    color = '#D3D3D3'
                rect = patches.Rectangle((i + gap, j + gap), rect_size, rect_size, facecolor=color, edgecolor='black', linewidth=1)
                ax.add_patch(rect)

        labels_x = [(0.5, "Disengaged"), (1.5, "Somewhat\nConnected"), (2.5, "Connected"), (3.5, "Very\nConnected"), (4.5, "Enmeshed")]
        for pos, text in labels_x:
            ax.text(pos, -0.15, text, ha='center', va='top', fontsize=10, fontweight='bold')
        ax.set_xlabel("COHESION", fontsize=14, fontweight='bold', labelpad=30)

        labels_y = [(0.5, "Rigid"), (1.5, "Somewhat\nFlexible"), (2.5, "Flexible"), (3.5, "Very\nFlexible"), (4.5, "Chaotic")]
        for pos, text in labels_y:
            ax.text(-0.15, pos, text, ha='right', va='center', rotation=0, fontsize=10, fontweight='bold')
        ax.set_ylabel("FLEXIBILITY", fontsize=14, fontweight='bold', labelpad=40)

        tick_vals = [0, 8, 15, 16, 25, 35, 36, 50, 65, 66, 75, 85, 86, 95, 100]
        tick_pos = [self.get_visual_coord(v) for v in tick_vals]
        ax.set_xticks(tick_pos)
        ax.set_xticklabels([str(v) for v in tick_vals], fontsize=8)
        ax.set_yticks(tick_pos)
        ax.set_yticklabels([str(v) for v in tick_vals], fontsize=8)
        ax.text(0.5, 1.05, "FACES IV Circumplex Model", transform=ax.transAxes, fontsize=16, fontweight='bold', ha='center', va='bottom')

        if x is not None and y is not None:
            tx_val = tx if tx is not None else 0.0
            ty_val = ty if ty is not None else 0.0
            label_text = f"Result (Blue): ({x:.1f}, {y:.1f})\nNext Target (Red): ({tx_val:.1f}, {ty_val:.1f})\nCoh Ratio: {coh_ratio:.2f}\nFlex Ratio: {flex_ratio:.2f}\nTot Ratio: {tot_ratio:.2f}"
            ax.text(1.05, 0.95, label_text, transform=ax.transAxes, color='black', fontsize=10, fontweight='bold', va='top', ha='left', bbox=dict(facecolor='white', alpha=0.9, edgecolor='black'))

        if trajectory:
            s0 = trajectory[0]
            prev_tx = s0.get("target_x", 8.0)
            prev_ty = s0.get("target_y", 8.0)
            v_prev_tx, v_prev_ty = self.get_visual_coord(prev_tx), self.get_visual_coord(prev_ty)
            if len(trajectory) > 1:
                ax.plot(v_prev_tx, v_prev_ty, 'bo', markersize=8, alpha=0.6, zorder=5)

            for i in range(1, len(trajectory)):
                step = trajectory[i]
                rx, ry = step.get("result_x"), step.get("result_y")
                tax, tay = step.get("target_x"), step.get("target_y")
                if rx is not None and ry is not None:
                    v_rx, v_ry = self.get_visual_coord(rx), self.get_visual_coord(ry)
                    ax.annotate("", xy=(v_rx, v_ry), xytext=(v_prev_tx, v_prev_ty), arrowprops=dict(arrowstyle="->", linestyle=':', color='gray', lw=2, alpha=0.6))
                    if not (i == len(trajectory) - 1 and (tax is None or tay is None)):
                        ax.plot(v_rx, v_ry, 'bo', markersize=10, markeredgecolor='black', zorder=7)
                    if tax is not None and tay is not None:
                        v_tax, v_tay = self.get_visual_coord(tax), self.get_visual_coord(tay)
                        ax.annotate("", xy=(v_tax, v_tay), xytext=(v_rx, v_ry), arrowprops=dict(arrowstyle="->", color='red', lw=2.5, alpha=0.8))
                        if i < len(trajectory) - 1:
                            ax.plot(v_tax, v_tay, 'ro', markersize=8, markeredgecolor='black', zorder=8)
                        v_prev_tx, v_prev_ty = v_tax, v_tay

        from matplotlib.lines import Line2D
        legend_elements = [
            Line2D([0], [0], marker='o', color='w', label='Result (Blue)', markerfacecolor='b', markersize=10, markeredgecolor='black'),
            Line2D([0], [0], marker='o', color='w', label='Target (Red)', markerfacecolor='r', markersize=10, markeredgecolor='black'),
            Line2D([0], [0], color='gray', linestyle=':', label='Actual Move (Dotted)'),
            Line2D([0], [0], color='red', label='Therapeutic Goal (Solid)'),
        ]
        ax.legend(handles=legend_elements, loc='lower left', bbox_to_anchor=(1.02, 0))

        # Save the background image (marker-less board only, used for the blink effect) first
        bg_save_path = os.path.join(self.db_dir, "evaluation_plot_bg.png")
        plt.savefig(bg_save_path)
        if trajectory:
            last_step = trajectory[-1]
            if last_step.get("target_x") is not None:
                v_lx, v_ly = self.get_visual_coord(last_step["target_x"]), self.get_visual_coord(last_step["target_y"])
                ax.plot(v_lx, v_ly, 'ro', markersize=16, markeredgecolor='black', zorder=10)
            elif last_step.get("result_x") is not None:
                v_lx, v_ly = self.get_visual_coord(last_step["result_x"]), self.get_visual_coord(last_step["result_y"])
                ax.plot(v_lx, v_ly, 'bo', markersize=16, markeredgecolor='black', zorder=10)

        save_path = os.path.join(self.db_dir, "evaluation_plot.png")
        plt.savefig(save_path)
        plt.close()
        return save_path

    def log_evaluation_to_csv(self, step_id, member_results, mean_ratings, current_pcts, x, y, tx, ty,
                               coh_ratio, flex_ratio, tot_ratio, target_scores):
        """Append one step's evaluation result to `evaluation_history.csv`."""
        csv_file = os.path.join(self.db_dir, "evaluation_history.csv")
        file_exists = os.path.exists(csv_file)

        try:
            with open(csv_file, 'a', encoding='utf-8', newline='') as f:
                writer = csv.writer(f)
                if not file_exists:
                    header = ["Timestamp", "StepID", "Cohesion_Dim", "Flexibility_Dim", "Target_X", "Target_Y", "Coh_Ratio", "Flex_Ratio", "Tot_Ratio"]
                    for k in target_scores.keys():
                        header.append(f"Target_{k}")
                    header.extend(["Member_Raw_Scores_JSON", "Mean_Ratings_JSON"])
                    writer.writerow(header)

                import json
                row = [datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S"), step_id, round(x, 2), round(y, 2), round(tx, 2), round(ty, 2), round(coh_ratio, 2), round(flex_ratio, 2), round(tot_ratio, 2)]
                for k in target_scores.keys():
                    row.append(round(target_scores[k], 2))
                row.append(json.dumps(member_results, ensure_ascii=False))
                row.append(json.dumps(mean_ratings, ensure_ascii=False))
                writer.writerow(row)
        except Exception as e:
            self.logger.error(f"Failed to log evaluation to CSV: {e}")

    def update_history_with_targets(self, history_file: str, scores: dict, tx: float, ty: float):
        """Append the next session's therapeutic target to the conversation history file (used as LLM context)."""
        update = f"\n[THERAPIST_STALL_SESSION_ANALYSIS]\n"
        update += f"Determined Therapeutic Target for Next Session: ({tx:.1f}, {ty:.1f})\n"
        update += "Targeted FACES IV Percentile Scores (Steering towards center):\n"
        for k, v in scores.items():
            update += f"- {k}: {int(v)}\n"
        update += "(Strategic Objective: Aggressively maneuver family towards the 'Balanced' zone (50, 50))\n"
        with open(history_file, "a", encoding="utf-8") as f:
            f.write(update)
            f.flush()
            try:
                os.fsync(f.fileno())
            except Exception:
                pass
