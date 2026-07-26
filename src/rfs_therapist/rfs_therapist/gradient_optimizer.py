#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Logic layer for the gradient descent that pulls FACES IV coordinates toward the "balanced" type.

A plain Python class with no ROS2 dependency. Given the current evaluation result
(percentiles), it updates the target subscale scores for the next session by one
gradient-descent step. Called from the rfs_optimizer node.

Beginner's note: "gradient descent" here just means "take a small step in the
direction that reduces the family's imbalance score, then check we didn't step
too far." The six subscale percentiles (Balanced Cohesion, Balanced Flexibility,
Disengaged, Enmeshed, Rigid, Chaotic) are the six numbers this step adjusts.
"""


class GradientOptimizer:
    """Logic class that computes the next target scores using gradient descent (no rclpy dependency)."""

    def __init__(self, omega_1: float, omega_2: float, omega_3: float, learning_rate_scaling: float):
        # omega_1: weight of the "imbalance penalty" for cohesion
        # omega_2: weight of the "imbalance penalty" for flexibility
        # omega_3: weight pulling the state toward the circumplex model center (50, 50)
        self.omega_1 = omega_1
        self.omega_2 = omega_2
        self.omega_3 = omega_3
        self.learning_rate_scaling = learning_rate_scaling

    def calculate_gradient(self, pcts: dict, x: float, y: float) -> dict:
        """Compute the next target scores from the current percentiles (pcts) and coordinates (x, y)."""
        # Step 1: read the six subscale percentiles we are going to adjust.
        cohesion_balanced = pcts["Balanced Cohesion"]
        flexibility_balanced = pcts["Balanced Flexibility"]
        cohesion_disengaged = pcts["Disengaged"]
        cohesion_enmeshed = pcts["Enmeshed"]
        flexibility_rigid = pcts["Rigid"]
        flexibility_chaotic = pcts["Chaotic"]

        # Step 2: baseline correction. (x, y) should equal balanced +/- half the
        # difference of the two extreme subscales; if it doesn't (because of how
        # the raw scores were computed), nudge "balanced" so it lines back up.
        implied_x = cohesion_balanced + (cohesion_enmeshed - cohesion_disengaged) / 2.0
        implied_y = flexibility_balanced + (flexibility_chaotic - flexibility_rigid) / 2.0

        if implied_x < 5.0:
            cohesion_balanced += (5.0 - implied_x)
        elif implied_x > 95.0:
            cohesion_balanced += (95.0 - implied_x)

        if implied_y < 5.0:
            flexibility_balanced += (5.0 - implied_y)
        elif implied_y > 95.0:
            flexibility_balanced += (95.0 - implied_y)

        # Step 3: compute the gradient (the direction + size of the "penalty" for
        # each subscale). A bigger gradient means that subscale needs a bigger nudge.
        learning_rate = self.learning_rate_scaling
        unbalanced_cohesion_total = cohesion_disengaged + cohesion_enmeshed
        unbalanced_flexibility_total = flexibility_rigid + flexibility_chaotic

        gradient_cohesion_balanced = - (self.omega_1 * unbalanced_cohesion_total) / (2.0 * max(1.0, cohesion_balanced ** 2)) + self.omega_3 * (x - 50.0)
        gradient_flexibility_balanced = - (self.omega_2 * unbalanced_flexibility_total) / (2.0 * max(1.0, flexibility_balanced ** 2)) + self.omega_3 * (y - 50.0)

        # This shared term appears in both the "enmeshed" and "disengaged" gradients
        # (and similarly for "chaotic"/"rigid"), just with the omega_3 part flipped in sign.
        gradient_cohesion_unbalanced_shared = self.omega_1 / (2.0 * max(1.0, cohesion_balanced))
        gradient_flexibility_unbalanced_shared = self.omega_2 / (2.0 * max(1.0, flexibility_balanced))

        gradient_cohesion_enmeshed = gradient_cohesion_unbalanced_shared + (self.omega_3 / 2.0) * (x - 50.0)
        gradient_cohesion_disengaged = gradient_cohesion_unbalanced_shared - (self.omega_3 / 2.0) * (x - 50.0)
        gradient_flexibility_chaotic = gradient_flexibility_unbalanced_shared + (self.omega_3 / 2.0) * (y - 50.0)
        gradient_flexibility_rigid = gradient_flexibility_unbalanced_shared - (self.omega_3 / 2.0) * (y - 50.0)

        # Step 4: turn each gradient into an actual step (move opposite the gradient,
        # scaled by the learning rate — this is the "descent" part of gradient descent).
        step_cohesion_balanced = - learning_rate * gradient_cohesion_balanced
        step_flexibility_balanced = - learning_rate * gradient_flexibility_balanced
        step_cohesion_disengaged = - learning_rate * gradient_cohesion_disengaged
        step_cohesion_enmeshed = - learning_rate * gradient_cohesion_enmeshed
        step_flexibility_rigid = - learning_rate * gradient_flexibility_rigid
        step_flexibility_chaotic = - learning_rate * gradient_flexibility_chaotic

        next_x_before_limit = (cohesion_balanced + step_cohesion_balanced) + ((cohesion_enmeshed + step_cohesion_enmeshed) - (cohesion_disengaged + step_cohesion_disengaged)) / 2.0
        next_y_before_limit = (flexibility_balanced + step_flexibility_balanced) + ((flexibility_chaotic + step_flexibility_chaotic) - (flexibility_rigid + step_flexibility_rigid)) / 2.0

        # Step 5: don't let the step jump past the grid cell next to the current one,
        # on the circumplex model's 5x5 grid. `step_scale` shrinks all the steps
        # above by the same amount if needed, so the move stays proportional.
        current_cohesion_cell, current_flexibility_cell = self._get_cell_idx(x), self._get_cell_idx(y)
        cell_ranges = [(0, 15), (16, 35), (36, 65), (66, 85), (86, 100)]
        min_allowed_x, max_allowed_x = cell_ranges[max(0, current_cohesion_cell - 1)][0], cell_ranges[min(4, current_cohesion_cell + 1)][1]
        min_allowed_y, max_allowed_y = cell_ranges[max(0, current_flexibility_cell - 1)][0], cell_ranges[min(4, current_flexibility_cell + 1)][1]

        step_scale = 1.0
        x_change, y_change = next_x_before_limit - x, next_y_before_limit - y
        if x_change != 0:
            if x + x_change > max_allowed_x:
                step_scale = min(step_scale, (max_allowed_x - x) / x_change)
            if x + x_change < min_allowed_x:
                step_scale = min(step_scale, (min_allowed_x - x) / x_change)
        if y_change != 0:
            if y + y_change > max_allowed_y:
                step_scale = min(step_scale, (max_allowed_y - y) / y_change)
            if y + y_change < min_allowed_y:
                step_scale = min(step_scale, (min_allowed_y - y) / y_change)

        if step_scale < 1.0:
            step_cohesion_balanced *= step_scale
            step_flexibility_balanced *= step_scale
            step_cohesion_disengaged *= step_scale
            step_cohesion_enmeshed *= step_scale
            step_flexibility_rigid *= step_scale
            step_flexibility_chaotic *= step_scale

        # Step 6: apply the (possibly shrunk) steps and return the new target scores.
        return {
            "Balanced Cohesion": max(5.0, min(99.0, cohesion_balanced + step_cohesion_balanced)),
            "Balanced Flexibility": max(5.0, min(99.0, flexibility_balanced + step_flexibility_balanced)),
            "Disengaged": max(5.0, min(99.0, cohesion_disengaged + step_cohesion_disengaged)),
            "Enmeshed": max(5.0, min(99.0, cohesion_enmeshed + step_cohesion_enmeshed)),
            "Rigid": max(5.0, min(99.0, flexibility_rigid + step_flexibility_rigid)),
            "Chaotic": max(5.0, min(99.0, flexibility_chaotic + step_flexibility_chaotic)),
            "Communication": pcts["Communication"] + learning_rate * self.omega_2,
        }

    @staticmethod
    def _get_cell_idx(v: float) -> int:
        """Return the grid cell index (Disengaged..Enmeshed, etc.) for the 5-level circumplex model."""
        if v <= 15:
            return 0
        if v <= 35:
            return 1
        if v <= 65:
            return 2
        if v <= 85:
            return 3
        return 4
