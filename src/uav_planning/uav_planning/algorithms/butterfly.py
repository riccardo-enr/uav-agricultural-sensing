import matplotlib.pyplot as plt
import math
import numpy as np


class TimeBasedButterflyExplorer:
    def __init__(
        self,
        x_min,
        x_max,
        y_min,
        y_max,
        z_min,
        z_max,
        alpha,
        visit_threshold,
        drone_speed,
    ):
        self.x_min, self.x_max = x_min, x_max
        self.y_min, self.y_max = y_min, y_max
        self.z_min, self.z_max = z_min, z_max
        self.alpha = alpha
        self.visit_threshold = visit_threshold
        self.drone_speed = drone_speed  # m/s

        # Define the 8 corners of the bounding box
        self.corners = np.array(
            [
                [x_min, y_min, z_min],
                [x_min, y_min, z_max],
                [x_min, y_max, z_min],
                [x_min, y_max, z_max],
                [x_max, y_min, z_min],
                [x_max, y_min, z_max],
                [x_max, y_max, z_min],
                [x_max, y_max, z_max],
            ]
        )

        # Start position at center of the volume
        self.current_pos = np.array(
            [(x_min + x_max) / 2, (y_min + y_max) / 2, (z_min + z_max) / 2]
        )
        self.initial_pos = self.current_pos.copy()

        # Track which corners have been visited
        self.total_corners = len(self.corners)
        self.visited_corners = [False] * self.total_corners
        self.corner_visit_times = {}  # Track when each corner was visited

        # Calculate maximum possible distance in the bounding box for normalization
        self.max_distance = np.linalg.norm(
            [x_max - x_min, y_max - y_min, z_max - z_min]
        )

        # Enhanced adaptive parameters for guaranteed corner visits
        self.base_bias_weight = 0.2
        self.max_bias_weight = 2.2
        self.urgency_threshold = 0.8

        # Track exploration history for adaptive behavior
        self.steps_since_last_corner = 0
        self.exploration_history = []

        # Time-based tracking
        self.current_time = 0.0
        self.total_distance_traveled = 0.0

    def levy_step(self):
        """Generate a step following Lévy flight distribution"""
        sigma = (
            math.gamma(1 + self.alpha)
            * np.sin(np.pi * self.alpha / 2)
            / (
                math.gamma((1 + self.alpha) / 2)
                * self.alpha
                * 2 ** ((self.alpha - 1) / 2)
            )
        ) ** (1 / self.alpha)
        u = np.random.normal(0, sigma, 3)
        v = np.random.normal(0, 1, 3)
        step = u / (np.abs(v) ** (1 / self.alpha))
        return step

    def clip_with_reflection(self, pos):
        """Improved boundary handling with reflection to avoid getting stuck"""
        new_pos = pos.copy()

        # Reflect off boundaries instead of just clipping
        if new_pos[0] < self.x_min:
            new_pos[0] = self.x_min + (self.x_min - new_pos[0])
        elif new_pos[0] > self.x_max:
            new_pos[0] = self.x_max - (new_pos[0] - self.x_max)

        if new_pos[1] < self.y_min:
            new_pos[1] = self.y_min + (self.y_min - new_pos[1])
        elif new_pos[1] > self.y_max:
            new_pos[1] = self.y_max - (new_pos[1] - self.y_max)

        if new_pos[2] < self.z_min:
            new_pos[2] = self.z_min + (self.z_min - new_pos[2])
        elif new_pos[2] > self.z_max:
            new_pos[2] = self.z_max - (new_pos[2] - self.z_max)

        # Final clipping as safety net
        new_pos[0] = max(min(new_pos[0], self.x_max), self.x_min)
        new_pos[1] = max(min(new_pos[1], self.y_max), self.y_min)
        new_pos[2] = max(min(new_pos[2], self.z_max), self.z_min)

        return new_pos

    def get_unvisited_corners(self):
        """Get list of unvisited corner indices"""
        return [
            i for i in range(self.total_corners) if not self.visited_corners[i]
        ]

    def calculate_adaptive_bias_weight(
        self, distance_to_corner, exploration_ratio, time_pressure
    ):
        """Calculate adaptive bias weight based on multiple factors including time pressure"""
        # Progress factor: exponentially increase bias as more corners are visited
        progress_factor = (
            1.0 - (len(self.get_unvisited_corners()) / self.total_corners)
        ) ** 2

        # Distance factor: stronger bias when closer to corner
        normalized_distance = distance_to_corner / self.max_distance
        distance_factor = (1.0 - normalized_distance) ** 2

        if exploration_ratio <= self.urgency_threshold:
            urgency_factor = 1.0
        else:
            # Linearly ramp from 1.0 at threshold → max_urgency at ratio=1.0
            max_urgency = 3.0
            t = (exploration_ratio - self.urgency_threshold) / (
                1.0 - self.urgency_threshold
            )
            urgency_factor = 1.0 + t * (max_urgency - 1.0)

        # Combine factors with time pressure
        combined_factor = (
            0.4 * progress_factor  # Progress weight
            + 0.2 * distance_factor  # Distance weight
            + 0.2 * urgency_factor  # Urgency weight
            + 0.2 * time_pressure  # Time pressure weight
        )

        # Scale between base and max bias weight
        adaptive_weight = (
            self.base_bias_weight
            + (self.max_bias_weight - self.base_bias_weight) * combined_factor
        )

        return min(adaptive_weight, self.max_bias_weight)

    def bias_towards(self, target, distance_to_target, time_pressure):
        """Calculate bias vector toward target corner"""
        exploration_ratio = (
            self.total_corners - len(self.get_unvisited_corners())
        ) / self.total_corners

        # Calculate adaptive bias weight
        bias_weight = self.calculate_adaptive_bias_weight(
            distance_to_target, exploration_ratio, time_pressure
        )

        # Vector from current position to target
        direction = target - self.current_pos
        direction_magnitude = np.linalg.norm(direction)

        if direction_magnitude > 0:
            # Normalize direction
            direction_normalized = direction / direction_magnitude

            # Apply adaptive bias with distance scaling
            distance_scale = max(
                0.2, 1.0 - (distance_to_target / self.max_distance)
            )

            # In urgent situations or time pressure, bias becomes more direct
            if (
                exploration_ratio > self.urgency_threshold
                or time_pressure > 0.7
            ):
                # Direct movement toward corner with reduced randomness
                final_bias = (
                    direction_normalized
                    * bias_weight
                    * distance_scale
                    * direction_magnitude
                    * 0.5
                )
            else:
                final_bias = direction * bias_weight * distance_scale
        else:
            final_bias = np.zeros(3)

        return final_bias

    def generate_next_waypoint(self, time_remaining, max_time):
        """Generate next waypoint using Lévy flight + adaptive biasing with time constraints"""
        self.steps_since_last_corner += 1

        # Calculate time pressure (0 to 1, where 1 is high pressure)
        time_pressure = 1.0 - (time_remaining / max_time)

        unvisited_indices = self.get_unvisited_corners()

        if not unvisited_indices:
            # All corners visited, continue with pure Lévy flight
            step = self.levy_step()
            new_pos = self.current_pos + step
            new_pos = self.clip_with_reflection(new_pos)
        else:
            # Find nearest unvisited corner
            unvisited_corners = self.corners[unvisited_indices]
            distances = np.linalg.norm(
                unvisited_corners - self.current_pos, axis=1
            )
            nearest_idx_in_unvisited = np.argmin(distances)
            nearest_corner_idx = unvisited_indices[nearest_idx_in_unvisited]
            nearest_corner = self.corners[nearest_corner_idx]
            distance_to_nearest = distances[nearest_idx_in_unvisited]

            # Calculate exploration progress
            exploration_ratio = (
                self.total_corners - len(unvisited_indices)
            ) / self.total_corners

            # Adaptive Lévy step based on exploration stage and time pressure
            if (
                exploration_ratio > 0.8
                or self.steps_since_last_corner > 10
                or time_pressure > 0.7
            ):
                # Late stage, stuck, or time pressure: reduce randomness, increase bias
                step = self.levy_step() * (
                    0.5 - 0.3 * time_pressure
                )  # Reduce randomness with time pressure
            else:
                step = self.levy_step()

            # Get adaptive bias with time pressure
            bias = self.bias_towards(
                nearest_corner, distance_to_nearest, time_pressure
            )
            new_pos = self.current_pos + step + bias
            new_pos = self.clip_with_reflection(new_pos)

            # Enhanced corner detection with adaptive threshold
            adaptive_threshold = self.visit_threshold
            if self.steps_since_last_corner > 10 or time_pressure > 0.8:
                adaptive_threshold = (
                    self.visit_threshold * 1.2
                )  # Increase threshold if struggling or time pressure

            # Check all unvisited corners for visits
            for i, corner_idx in enumerate(unvisited_indices):
                corner = self.corners[corner_idx]
                if np.linalg.norm(new_pos - corner) < adaptive_threshold:
                    # Mark corner as visited
                    self.visited_corners[corner_idx] = True
                    self.corner_visit_times[corner_idx] = self.current_time
                    self.steps_since_last_corner = 0
                    print(
                        f"Corner {corner_idx} visited at t={self.current_time:.1f}s! {len(self.get_unvisited_corners())} corners remaining."
                    )
                    break

        return new_pos

    def get_exploration_stats(self):
        """Get exploration statistics"""
        visited_count = sum(self.visited_corners)
        return {
            "corners_visited": visited_count,
            "total_time": self.current_time,
            "total_distance": self.total_distance_traveled,
            "completion_rate": visited_count / self.total_corners,
            "avg_speed": self.total_distance_traveled
            / max(self.current_time, 0.001),
            "corner_visit_times": self.corner_visit_times.copy(),
        }


def run_time_based_exploration(max_time, drone_speed):
    """Run time-based exploration for specified duration"""

    explorer = TimeBasedButterflyExplorer(
        x_min=-8,
        x_max=8,
        y_min=-4,
        y_max=4,
        z_min=2,
        z_max=6,
        alpha=1.5,
        visit_threshold=2.0,
        drone_speed=drone_speed,
    )

    print(f"Starting time-based exploration...")
    print(f"Duration: {max_time}s, Drone speed: {drone_speed} m/s")
    print(
        f"Target: Visit all {explorer.total_corners} corners of the bounding box"
    )
    print("-" * 60)

    waypoints = []
    current_time = 0.0

    # Add initial position as first waypoint
    waypoints.append(
        {
            "x": explorer.current_pos[0],
            "y": explorer.current_pos[1],
            "z": explorer.current_pos[2],
            "t": current_time,
            "travel_time": 0.0,
        }
    )

    while current_time < max_time:
        # Generate next waypoint
        time_remaining = max_time - current_time
        next_pos = explorer.generate_next_waypoint(time_remaining, max_time)

        # Calculate travel time to next waypoint
        distance_to_next = np.linalg.norm(next_pos - explorer.current_pos)
        travel_time = distance_to_next / drone_speed

        # Check if we have enough time to reach the next waypoint
        if current_time + travel_time > max_time:
            print(
                f"Not enough time to reach next waypoint. Stopping at t={current_time:.1f}s"
            )
            break

        # Update position and time
        explorer.current_pos = next_pos
        current_time += travel_time
        explorer.current_time = current_time
        explorer.total_distance_traveled += distance_to_next

        # Store waypoint with timing information
        waypoints.append(
            {
                "x": next_pos[0],
                "y": next_pos[1],
                "z": next_pos[2],
                "t": current_time,
                "travel_time": travel_time,
            }
        )

        # Print progress every 100 seconds
        if len(waypoints) % 20 == 0:
            unvisited = len(explorer.get_unvisited_corners())
            print(
                f"t={current_time:.1f}s: {explorer.total_corners - unvisited}/{explorer.total_corners} corners visited"
            )

    # Display final statistics
    stats = explorer.get_exploration_stats()
    print("\n" + "=" * 60)
    print("EXPLORATION COMPLETED")
    print("=" * 60)
    print(f"Total time: {stats['total_time']:.1f}s / {max_time}s")
    print(f"Total distance: {stats['total_distance']:.1f}m")
    print(f"Average speed: {stats['avg_speed']:.2f} m/s")
    print(
        f"Corners visited: {stats['corners_visited']}/{explorer.total_corners}"
    )
    print(f"Completion rate: {stats['completion_rate']:.1%}")
    print(f"Total waypoints: {len(waypoints)}")
    print(
        f"All corners explored: {'YES' if len(explorer.get_unvisited_corners()) == 0 else 'NO'}"
    )

    if stats["corner_visit_times"]:
        print("\nCorner visit times:")
        for corner_idx, visit_time in sorted(
            stats["corner_visit_times"].items()
        ):
            print(f"  Corner {corner_idx}: {visit_time:.1f}s")

    return waypoints, explorer, stats


def visualize_exploration(waypoints, explorer, stats):
    """Visualize the exploration results"""
    if not waypoints:
        print("No waypoints to visualize!")
        return

    x_vals = [wp["x"] for wp in waypoints]
    y_vals = [wp["y"] for wp in waypoints]
    z_vals = [wp["z"] for wp in waypoints]
    times = [wp["t"] for wp in waypoints]

    # Create subplots
    fig = plt.figure(figsize=(16, 12))

    # 3D trajectory plot
    ax1 = fig.add_subplot(221, projection="3d")

    # Color-code path by time
    colors = plt.cm.viridis(np.array(times) / max(times))
    scatter = ax1.scatter(
        x_vals, y_vals, z_vals, c=times, cmap="viridis", s=20, alpha=0.7
    )
    ax1.plot(x_vals, y_vals, z_vals, c="steelblue", alpha=0.4, linewidth=1)

    # Mark start position
    ax1.scatter(
        explorer.initial_pos[0],
        explorer.initial_pos[1],
        explorer.initial_pos[2],
        color="red",
        s=100,
        marker="o",
        label="Start Position",
    )

    # Mark corners
    visited_indices = [
        i for i in range(explorer.total_corners) if explorer.visited_corners[i]
    ]
    unvisited_indices = [
        i
        for i in range(explorer.total_corners)
        if not explorer.visited_corners[i]
    ]

    if visited_indices:
        visited_corners = explorer.corners[visited_indices]
        ax1.scatter(
            visited_corners[:, 0],
            visited_corners[:, 1],
            visited_corners[:, 2],
            color="green",
            s=100,
            marker="^",
            label="Visited Corners",
        )

    if unvisited_indices:
        unvisited_corners = explorer.corners[unvisited_indices]
        ax1.scatter(
            unvisited_corners[:, 0],
            unvisited_corners[:, 1],
            unvisited_corners[:, 2],
            color="red",
            s=100,
            marker="^",
            label="Unvisited Corners",
        )

    ax1.set_xlabel("X (m)")
    ax1.set_ylabel("Y (m)")
    ax1.set_zlabel("Z (m)")
    ax1.set_title(
        f"3D Trajectory - {stats['corners_visited']}/{explorer.total_corners} Corners"
    )
    ax1.legend()
    plt.colorbar(scatter, ax=ax1, label="Time (s)")

    # Time vs distance plot
    ax2 = fig.add_subplot(222)
    cumulative_distance = np.cumsum(
        [0]
        + [
            wp.get("travel_time", 0) * explorer.drone_speed
            for wp in waypoints[1:]
        ]
    )
    ax2.plot(times, cumulative_distance, "b-", linewidth=2)
    ax2.set_xlabel("Time (s)")
    ax2.set_ylabel("Cumulative Distance (m)")
    ax2.set_title("Distance Traveled Over Time")
    ax2.grid(True)

    # Corner visits over time
    ax3 = fig.add_subplot(223)
    corner_visit_times = list(stats["corner_visit_times"].values())
    corner_numbers = list(range(1, len(corner_visit_times) + 1))
    if corner_visit_times:
        ax3.step(
            corner_visit_times, corner_numbers, "g-", linewidth=2, where="post"
        )
        ax3.scatter(
            corner_visit_times, corner_numbers, color="green", s=50, zorder=5
        )
    ax3.set_xlabel("Time (s)")
    ax3.set_ylabel("Corners Visited")
    ax3.set_title("Corner Discovery Over Time")
    ax3.grid(True)

    # Speed over time
    ax4 = fig.add_subplot(224)
    travel_times = [wp.get("travel_time", 0) for wp in waypoints[1:]]
    if travel_times:
        segment_times = np.array(times[1:])
        ax4.plot(
            segment_times,
            [explorer.drone_speed] * len(segment_times),
            "r--",
            linewidth=2,
            label=f"Target Speed ({explorer.drone_speed} m/s)",
        )
        ax4.axhline(y=explorer.drone_speed, color="red", alpha=0.3)
    ax4.set_xlabel("Time (s)")
    ax4.set_ylabel("Speed (m/s)")
    ax4.set_title("Flight Speed")
    ax4.legend()
    ax4.grid(True)

    plt.tight_layout()
    plt.show()


def main():
    """Main execution function"""
    # Run the exploration
    waypoints, explorer, stats = run_time_based_exploration(
        max_time=600, drone_speed=0.2
    )

    # Visualize results
    visualize_exploration(waypoints, explorer, stats)

    return waypoints, explorer, stats


if __name__ == "__main__":
    waypoints, explorer, stats = main()
