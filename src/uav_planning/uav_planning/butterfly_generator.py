#!/usr/bin/env python3

import random
import math


class ButterflyGenerator:
    """
    Handles the core logic for generating a butterfly-inspired Levy flight path.
    This class is completely independent of ROS and can be used in any Python project.
    """

    def __init__(
        self,
        x_min=0.0,
        x_max=10.0,
        y_min=0.0,
        y_max=5.0,
        z_min=1.0,
        z_max=2.0,
        alpha=1.5,
        step_scale=1.0,
    ):
        """
        Initializes the generator with boundaries and algorithm parameters.
        """
        self.x_min, self.x_max = x_min, x_max
        self.y_min, self.y_max = y_min, y_max
        self.z_min, self.z_max = z_min, z_max
        self.alpha = alpha
        self.step_scale = step_scale

        # --- State ---
        # Start in the middle of the configured volume
        self.current_x = (self.x_max - self.x_min) / 2.0 + self.x_min
        self.current_y = (self.y_max - self.y_min) / 2.0 + self.y_min
        self.current_z = (self.z_max - self.z_min) / 2.0 + self.z_min

        print("ButterflyGenerator initialized.")
        print(
            f"  Bounds: X:[{self.x_min}, {self.x_max}], Y:[{self.y_min}, {self.y_max}], Z:[{self.z_min}, {self.z_max}]"
        )
        print(
            f"  Initial Position: ({self.current_x:.2f}, {self.current_y:.2f}, {self.current_z:.2f})"
        )

    def levy_step(self):
        """Generates a Levy-distributed step size and a random direction."""
        step = self.step_scale * (random.paretovariate(self.alpha))
        theta = random.uniform(0, 2 * math.pi)
        dx = step * math.cos(theta)
        dy = step * math.sin(theta)
        return dx, dy

    def clip(self, val, vmin, vmax):
        """Utility function to constrain a value between a min and max."""
        return max(min(val, vmax), vmin)

    def generate_next_waypoint(self):
        """
        Calculates the next waypoint in the sequence.

        Returns:
            A tuple (x, y, z) representing the new waypoint.
        """
        dx, dy = self.levy_step()
        self.current_x = self.clip(self.current_x + dx, self.x_min, self.x_max)
        self.current_y = self.clip(self.current_y + dy, self.y_min, self.y_max)

        # Add a small random vertical "flutter"
        dz = random.uniform(-0.2, 0.2)
        self.current_z = self.clip(self.current_z + dz, self.z_min, self.z_max)

        return (self.current_x, self.current_y, self.current_z)
