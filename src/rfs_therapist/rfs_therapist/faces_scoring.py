#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Logic layer for FACES IV (Family Adaptability and Cohesion Evaluation Scale) scoring.

A plain Python class with no ROS2 dependency. Takes the 62-item subjective ratings
(1-5 scale) and converts them into subscale scores -> percentiles -> circumplex
model coordinates (x, y). Called from the rfs_evaluator node.
"""


class FacesScoreCalculator:
    """Logic class that performs FACES IV score calculation (no rclpy dependency)."""

    def __init__(self):
        # Maps each of the 62 items to one of 8 subscales (standard FACES IV definition)
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

    def average_ratings(self, aggregated_results: dict) -> dict:
        """Average the raw per-member scores {role: {item_id: rating}} per item.

        Any item missing a rating is filled in with a neutral value (3.0).
        """
        ratings = {}
        for item_id in range(1, 63):
            item_scores = []
            for role, results in aggregated_results.items():
                score = results.get(str(item_id))
                if isinstance(score, dict):
                    score = score.get("rating")
                if score is not None:
                    item_scores.append(float(score))
            ratings[item_id] = sum(item_scores) / len(item_scores) if item_scores else 3.0
        return ratings

    def calculate_scores(self, ratings: dict) -> dict:
        """Convert averaged item scores into circumplex coordinates (x, y) and percentiles."""
        # Step 1: add up the items for each subscale, then convert each sum to a percentile.
        scores = {k: sum(ratings.get(i, 3.0) for i in items) for k, items in self.scales.items()}
        pcts = {k: self._get_percentile(k, v) for k, v in scores.items()}

        # Communication uses a different conversion range from the other scales, so it's converted on its own
        communication_raw = max(10, min(50, int(scores["Communication"])))
        pcts["Communication"] = 10 + (communication_raw - 10) * (99 - 10) / (50 - 10)

        # Step 2: imbalance ratios (balanced type score vs. the two extreme-type scores).
        # A ratio near 1.0 means the family is about as "balanced" as it is "extreme".
        unbalanced_cohesion = (pcts["Disengaged"] + pcts["Enmeshed"]) / 2.0
        cohesion_ratio = pcts["Balanced Cohesion"] / unbalanced_cohesion if unbalanced_cohesion > 0 else 1.0
        unbalanced_flexibility = (pcts["Rigid"] + pcts["Chaotic"]) / 2.0
        flexibility_ratio = pcts["Balanced Flexibility"] / unbalanced_flexibility if unbalanced_flexibility > 0 else 1.0
        total_ratio = (
            (pcts["Balanced Cohesion"] + pcts["Balanced Flexibility"]) / (unbalanced_cohesion + unbalanced_flexibility)
            if (unbalanced_cohesion + unbalanced_flexibility) > 0
            else 1.0
        )

        # Step 3: final coordinates on the circumplex model (cohesion x, flexibility y)
        cohesion_dimension = pcts["Balanced Cohesion"] + (pcts["Enmeshed"] - pcts["Disengaged"]) / 2.0
        flexibility_dimension = pcts["Balanced Flexibility"] + (pcts["Chaotic"] - pcts["Rigid"]) / 2.0
        x = max(5.0, min(95.0, cohesion_dimension))
        y = max(5.0, min(95.0, flexibility_dimension))

        return {
            "x": x,
            "y": y,
            "pcts": pcts,
            "coh_ratio": cohesion_ratio,
            "flex_ratio": flexibility_ratio,
            "tot_ratio": total_ratio,
            "mean_ratings": ratings,
        }

    def _get_percentile(self, scale: str, raw_score: float) -> float:
        """Convert a subscale's raw score into a percentile using the official FACES IV lookup table."""
        raw_score = max(7, min(35, int(raw_score)))
        if scale.startswith("Balanced"):
            # Lookup table for the two "Balanced" subscales (Balanced Cohesion, Balanced Flexibility)
            table = {
                7: 16, 8: 18, 9: 20, 10: 22, 11: 24, 12: 25, 13: 26, 14: 27, 15: 28, 16: 30, 17: 32,
                18: 35, 19: 36, 20: 38, 21: 40, 22: 45, 23: 50, 24: 55, 25: 58, 26: 60, 27: 62,
                28: 65, 29: 68, 30: 70, 31: 75, 32: 80, 33: 82, 34: 84, 35: 85,
            }
            return table.get(raw_score, 50)
        else:
            # Lookup table for the four "extreme" subscales (Disengaged, Enmeshed, Rigid, Chaotic)
            table = {
                7: 10, 8: 12, 9: 13, 10: 14, 11: 15, 12: 16, 13: 18, 14: 20, 15: 24, 16: 26,
                17: 30, 18: 32, 19: 34, 20: 36, 21: 40, 22: 45, 23: 50, 24: 55, 25: 60, 26: 64,
                27: 68, 28: 70, 29: 75, 30: 80, 31: 85, 32: 90, 33: 95, 34: 98, 35: 99,
            }
            return table.get(raw_score, 30)
