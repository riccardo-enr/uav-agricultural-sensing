import matplotlib.pyplot as plt
import numpy as np
import math
from typing import List, Dict, Tuple
import time
from dataclasses import dataclass
from mpl_toolkits.mplot3d import Axes3D


@dataclass
class ExplorationMetrics:
    """Metrics for comparing exploration algorithms"""

    vertices_visited: int
    total_time: float
    total_distance: float
    completion_rate: float
    time_to_first_corner: float
    time_to_last_corner: float
    coverage_efficiency: float  # vertices per unit distance
    time_efficiency: float  # vertices per unit time
    path_smoothness: float  # measure of path continuity
    search_variance: float  # spatial distribution variance


class BaseExplorer:
    """Base class for exploration algorithms"""

    def __init__(
        self,
        x_min,
        x_max,
        y_min,
        y_max,
        z_min,
        z_max,
        visit_threshold,
        drone_speed,
    ):
        self.x_min, self.x_max = x_min, x_max
        self.y_min, self.y_max = y_min, y_max
        self.z_min, self.z_max = z_min, z_max
        self.visit_threshold = visit_threshold
        self.drone_speed = drone_speed

        # Define vertices
        self.vertices = np.array(
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

        # Initialize state
        self.current_pos = np.array(
            [(x_min + x_max) / 2, (y_min + y_max) / 2, (z_min + z_max) / 2]
        )
        self.visited_vertices = [False] * len(self.vertices)
        self.corner_visit_times = {}
        self.current_time = 0.0
        self.total_distance = 0.0
        self.waypoints = []
        self.total_vertices = len(self.vertices)

    def clip_position(self, pos):
        """Clip position to bounds"""
        return np.clip(
            pos,
            [self.x_min, self.y_min, self.z_min],
            [self.x_max, self.y_max, self.z_max],
        )

    def clip_with_reflection(self, pos):
        """Clip position to bounds with reflection"""
        new_pos = pos.copy()

        # Reflect off boundaries
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

        # Final clipping to ensure within bounds
        return self.clip_position(new_pos)

    def get_unvisited_vertices(self):
        """Get indices of unvisited vertices"""
        return [
            i for i, visited in enumerate(self.visited_vertices) if not visited
        ]

    def check_corner_visits(self, pos):
        """Check if any vertices are visited"""
        for i, corner in enumerate(self.vertices):
            if (
                not self.visited_vertices[i]
                and np.linalg.norm(pos - corner) < self.visit_threshold
            ):
                self.visited_vertices[i] = True
                self.corner_visit_times[i] = self.current_time
                return True
        return False

    def generate_next_waypoint(self, time_remaining, max_time=None):
        """Generate next waypoint - to be implemented by subclasses"""
        raise NotImplementedError(
            "Subclasses must implement generate_next_waypoint"
        )

    def get_metrics(self) -> ExplorationMetrics:
        """Calculate exploration metrics"""
        visited_count = sum(self.visited_vertices)
        completion_rate = visited_count / len(self.vertices)

        # Time metrics
        visit_times = list(self.corner_visit_times.values())
        time_to_first = min(visit_times) if visit_times else float("inf")
        time_to_last = max(visit_times) if visit_times else float("inf")

        # Efficiency metrics
        coverage_efficiency = visited_count / max(self.total_distance, 0.001)
        time_efficiency = visited_count / max(self.current_time, 0.001)

        # Path smoothness (average change in direction)
        path_smoothness = self.calculate_path_smoothness()

        # Search variance (spatial distribution)
        search_variance = self.calculate_search_variance()

        return ExplorationMetrics(
            vertices_visited=visited_count,
            total_time=self.current_time,
            total_distance=self.total_distance,
            completion_rate=completion_rate,
            time_to_first_corner=time_to_first,
            time_to_last_corner=time_to_last,
            coverage_efficiency=coverage_efficiency,
            time_efficiency=time_efficiency,
            path_smoothness=path_smoothness,
            search_variance=search_variance,
        )

    def calculate_path_smoothness(self):
        """Calculate path smoothness metric"""
        if len(self.waypoints) < 3:
            return 0.0

        direction_changes = []
        for i in range(1, len(self.waypoints) - 1):
            p1 = np.array(
                [
                    self.waypoints[i - 1]["x"],
                    self.waypoints[i - 1]["y"],
                    self.waypoints[i - 1]["z"],
                ]
            )
            p2 = np.array(
                [
                    self.waypoints[i]["x"],
                    self.waypoints[i]["y"],
                    self.waypoints[i]["z"],
                ]
            )
            p3 = np.array(
                [
                    self.waypoints[i + 1]["x"],
                    self.waypoints[i + 1]["y"],
                    self.waypoints[i + 1]["z"],
                ]
            )

            v1 = p2 - p1
            v2 = p3 - p2

            if np.linalg.norm(v1) > 0 and np.linalg.norm(v2) > 0:
                cos_angle = np.dot(v1, v2) / (
                    np.linalg.norm(v1) * np.linalg.norm(v2)
                )
                cos_angle = np.clip(cos_angle, -1, 1)
                angle_change = np.arccos(cos_angle)
                direction_changes.append(angle_change)

        return np.mean(direction_changes) if direction_changes else 0.0

    def calculate_search_variance(self):
        """Calculate spatial distribution variance"""
        if len(self.waypoints) < 2:
            return 0.0

        positions = np.array(
            [[wp["x"], wp["y"], wp["z"]] for wp in self.waypoints]
        )
        return np.mean(np.var(positions, axis=0))


class LevyFlightExplorer(BaseExplorer):
    """Bio-Inspired Lévy flight explorer"""

    def __init__(
        self,
        x_min,
        x_max,
        y_min,
        y_max,
        z_min,
        z_max,
        visit_threshold,
        drone_speed,
        alpha=1.5,
    ):
        super().__init__(
            x_min,
            x_max,
            y_min,
            y_max,
            z_min,
            z_max,
            visit_threshold,
            drone_speed,
        )
        self.alpha = alpha
        self.max_distance = np.linalg.norm(
            [x_max - x_min, y_max - y_min, z_max - z_min]
        )
        self.steps_since_last_corner = 0

    def levy_step(self):
        """Generate Lévy flight step"""
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

    def bias_towards(self, target, distance, time_pressure):
        """Calculate bias vector towards target"""
        if distance == 0:
            return np.zeros(3)

        direction = (target - self.current_pos) / distance

        # Adaptive bias strength based on distance, exploration progress, and time pressure
        base_bias = 0.3
        distance_factor = min(1.0, distance / self.max_distance)
        time_factor = (
            1.0 + 2.0 * time_pressure
        )  # Increase bias with time pressure

        bias_strength = base_bias * distance_factor * time_factor

        return direction * bias_strength * distance

    def generate_next_waypoint(self, time_remaining, max_time=None):
        """Generate next waypoint using Lévy flight + adaptive biasing with time constraints"""
        self.steps_since_last_corner += 1

        # Calculate time pressure (0 to 1, where 1 is high pressure)
        time_pressure = 1.0 - (time_remaining / max_time)

        unvisited_indices = self.get_unvisited_vertices()

        if not unvisited_indices:
            # All vertices visited, continue with pure Lévy flight
            step = self.levy_step()
            new_pos = self.current_pos + step
            new_pos = self.clip_with_reflection(new_pos)
        else:
            # Find nearest unvisited corner
            unvisited_vertices = self.vertices[unvisited_indices]
            distances = np.linalg.norm(
                unvisited_vertices - self.current_pos, axis=1
            )
            nearest_idx_in_unvisited = np.argmin(distances)
            nearest_corner_idx = unvisited_indices[nearest_idx_in_unvisited]
            nearest_corner = self.vertices[nearest_corner_idx]
            distance_to_nearest = distances[nearest_idx_in_unvisited]

            # Calculate exploration progress
            exploration_ratio = (
                self.total_vertices - len(unvisited_indices)
            ) / self.total_vertices

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

            # Check all unvisited vertices for visits
            for i, corner_idx in enumerate(unvisited_indices):
                corner = self.vertices[corner_idx]
                if np.linalg.norm(new_pos - corner) < adaptive_threshold:
                    # Mark corner as visited
                    self.visited_vertices[corner_idx] = True
                    self.corner_visit_times[corner_idx] = self.current_time
                    self.steps_since_last_corner = 0
                    print(
                        f"Corner {corner_idx} visited at t={self.current_time:.1f}s! {len(self.get_unvisited_vertices())} vertices remaining."
                    )
                    break

        return new_pos


class GridSearchExplorer(BaseExplorer):
    """Systematic grid search explorer with variable grid points and sweep directions"""

    def __init__(
        self,
        x_min,
        x_max,
        y_min,
        y_max,
        z_min,
        z_max,
        visit_threshold,
        drone_speed,
        grid_points=15,
        sweep_direction="xyz",
    ):
        super().__init__(
            x_min,
            x_max,
            y_min,
            y_max,
            z_min,
            z_max,
            visit_threshold,
            drone_speed,
        )
        self.grid_points = grid_points
        self.sweep_direction = sweep_direction
        self.grid_index = 0

        # Create grid based on sweep direction
        self.grid = self._create_grid()

    def _create_grid(self):
        """Create grid with specified sweep direction"""
        x = np.linspace(self.x_min, self.x_max, self.grid_points)
        y = np.linspace(self.y_min, self.y_max, self.grid_points)
        z = np.linspace(self.z_min, self.z_max, self.grid_points)

        if self.sweep_direction == "xyz":
            # Sweep X first, then Y, then Z
            grid = []
            for zi in z:
                for yi in y:
                    for xi in x:
                        grid.append([xi, yi, zi])
        elif self.sweep_direction == "xzy":
            # Sweep X first, then Z, then Y
            grid = []
            for yi in y:
                for zi in z:
                    for xi in x:
                        grid.append([xi, yi, zi])
        elif self.sweep_direction == "yxz":
            # Sweep Y first, then X, then Z
            grid = []
            for zi in z:
                for xi in x:
                    for yi in y:
                        grid.append([xi, yi, zi])
        elif self.sweep_direction == "yzx":
            # Sweep Y first, then Z, then X
            grid = []
            for xi in x:
                for zi in z:
                    for yi in y:
                        grid.append([xi, yi, zi])
        elif self.sweep_direction == "zxy":
            # Sweep Z first, then X, then Y
            grid = []
            for yi in y:
                for xi in x:
                    for zi in z:
                        grid.append([xi, yi, zi])
        elif self.sweep_direction == "zyx":
            # Sweep Z first, then Y, then X
            grid = []
            for xi in x:
                for yi in y:
                    for zi in z:
                        grid.append([xi, yi, zi])
        else:
            # Default to xyz
            grid = []
            for zi in z:
                for yi in y:
                    for xi in x:
                        grid.append([xi, yi, zi])

        return np.array(grid)

    def generate_next_waypoint(self, time_remaining, max_time=None):
        """Generate next waypoint following grid pattern"""
        if self.grid_index >= len(self.grid):
            # If grid is exhausted, return current position
            return self.current_pos

        next_pos = self.grid[self.grid_index]
        self.grid_index += 1
        return next_pos


def plot_trajectories(explorers_data, bounds, title):
    """
    Plot 3D trajectories for comparison between different exploration algorithms

    Args:
        explorers_data: Dictionary with explorer names as keys and explorer objects as values
        bounds: Tuple of (x_min, x_max, y_min, y_max, z_min, z_max)
        title: Title for the plot
    """
    fig = plt.figure(figsize=(20, 6))

    x_min, x_max, y_min, y_max, z_min, z_max = bounds

    # Create subplots for each algorithm
    num_explorers = len(explorers_data)

    for i, (name, explorer) in enumerate(explorers_data.items()):
        ax = fig.add_subplot(1, num_explorers, i + 1, projection="3d")

        # Set up grid and appearance
        ax.grid(True, color="red", linestyle="-", linewidth=0.5, alpha=0.5)

        # Remove tick lines for cleaner look
        ax.tick_params(axis="x", which="both", length=0)
        ax.tick_params(axis="y", which="both", length=0)
        ax.tick_params(axis="z", which="both", length=0)

        # Extract trajectory data
        if len(explorer.waypoints) > 0:
            x_traj = [wp["x"] for wp in explorer.waypoints]
            y_traj = [wp["y"] for wp in explorer.waypoints]
            z_traj = [wp["z"] for wp in explorer.waypoints]
            times = [wp["t"] for wp in explorer.waypoints]

            # Plot trajectory
            ax.plot(
                x_traj,
                y_traj,
                z_traj,
                color="#1f77b4",
                alpha=0.5,
                linewidth=0.6,
            )

            # Plot trajectory points
            scatter = ax.scatter(
                x_traj, y_traj, z_traj, c=times, cmap="viridis", s=20, alpha=0.7
            )

            # Add colorbar for time
            plt.colorbar(
                scatter,
                ax=ax,
                label="Time (s)",
                shrink=0.5,
                location="left",
                pad=0.001,
            )

        # Plot corner points
        vertices = explorer.vertices
        visited_vertices = explorer.visited_vertices

        # Plot unvisited vertices in red
        unvisited_vertices = vertices[
            np.array([not v for v in visited_vertices])
        ]
        if len(unvisited_vertices) > 0:
            ax.scatter(
                unvisited_vertices[:, 0],
                unvisited_vertices[:, 1],
                unvisited_vertices[:, 2],
                c="red",
                s=100,
                marker="s",
                alpha=0.8,
                label="Unvisited vertices",
            )

        # Plot visited vertices in green
        visited_vertices_pos = vertices[np.array(visited_vertices)]
        if len(visited_vertices_pos) > 0:
            ax.scatter(
                visited_vertices_pos[:, 0],
                visited_vertices_pos[:, 1],
                visited_vertices_pos[:, 2],
                c="green",
                s=100,
                marker="s",
                alpha=0.8,
                label="Visited vertices",
            )

        # Plot starting position
        start_pos = (
            explorer.waypoints[0]
            if explorer.waypoints
            else {"x": 0, "y": 0, "z": 0}
        )
        ax.scatter(
            [start_pos["x"]],
            [start_pos["y"]],
            [start_pos["z"]],
            c="black",
            s=150,
            marker="*",
            alpha=1.0,
            label="Start",
        )

        # Plot current position
        if len(explorer.waypoints) > 0:
            current_pos = explorer.waypoints[-1]
            ax.scatter(
                [current_pos["x"]],
                [current_pos["y"]],
                [current_pos["z"]],
                c="black",
                s=100,
                marker="o",
                alpha=1.0,
                label="Final",
            )

        # Draw bounding box
        # Bottom face
        ax.plot(
            [x_min, x_max, x_max, x_min, x_min],
            [y_min, y_min, y_max, y_max, y_min],
            [z_min, z_min, z_min, z_min, z_min],
            color="gray",
            linestyle="--",
            alpha=0.3,
        )
        # Top face
        ax.plot(
            [x_min, x_max, x_max, x_min, x_min],
            [y_min, y_min, y_max, y_max, y_min],
            [z_max, z_max, z_max, z_max, z_max],
            color="black",
            linestyle="--",
            alpha=0.5,
        )

        # Vertical edges
        for corner in vertices:
            ax.plot(
                [corner[0], corner[0]],
                [corner[1], corner[1]],
                [z_min, z_max],
                color="gray",
                linestyle="--",
                alpha=0.2,
            )

        # Set labels and title
        ax.set_xlabel("X (m)")
        ax.set_ylabel("Y (m)")
        ax.set_zlabel("Z (m)")
        ax.set_title(name)
        ax.legend(loc="upper right", fontsize=8)

        # Set equal aspect ratio
        ax.set_xlim(x_min, x_max)
        ax.set_ylim(y_min, y_max)
        ax.set_zlim(z_min, z_max)

    plt.tight_layout()
    plt.show()


def run_trajectory_comparison(max_time=600, drone_speed=0.2):
    """
    Run a single comparison trial and plot trajectories

    Args:
        max_time: Maximum simulation time
        drone_speed: Drone speed in m/s

    Returns:
        Dictionary of explorer objects with their trajectories
    """
    # Define exploration volume
    bounds = (-8, 8, -4, 4, 2, 6)
    visit_threshold = 2.0

    # Initialize explorers with example parameters
    explorers = {
        "Lévy Flight (Bioinspired)": LevyFlightExplorer(
            *bounds, visit_threshold, drone_speed, alpha=1.5
        ),
        "Grid Search (18pts, XYZ)": GridSearchExplorer(
            *bounds,
            visit_threshold,
            drone_speed,
            grid_points=18,
            sweep_direction="xyz",
        ),
    }

    print("Running trajectory comparison...")
    print(f"Max time: {max_time}s, Drone speed: {drone_speed} m/s")
    print("-" * 50)

    # Run simulation for each explorer
    for name, explorer in explorers.items():
        print(f"Running {name}...")

        # Initialize trajectory
        current_time = 0.0
        explorer.waypoints.append(
            {
                "x": explorer.current_pos[0],
                "y": explorer.current_pos[1],
                "z": explorer.current_pos[2],
                "t": current_time,
            }
        )

        # Run exploration
        while current_time < max_time:
            time_remaining = max_time - current_time
            next_pos = explorer.generate_next_waypoint(time_remaining, max_time)

            # Calculate travel time
            distance = np.linalg.norm(next_pos - explorer.current_pos)
            travel_time = distance / drone_speed

            if current_time + travel_time > max_time:
                break

            # Update state
            explorer.current_pos = next_pos
            current_time += travel_time
            explorer.current_time = current_time
            explorer.total_distance += distance

            # Check corner visits
            explorer.check_corner_visits(next_pos)

            # Record waypoint
            explorer.waypoints.append(
                {
                    "x": next_pos[0],
                    "y": next_pos[1],
                    "z": next_pos[2],
                    "t": current_time,
                }
            )

        # Print results
        visited = sum(explorer.visited_vertices)
        metrics = explorer.get_metrics()
        print(
            f"  {name}: {visited}/8 vertices ({visited / 8 * 100:.1f}%) in {current_time:.1f}s"
        )
        print(f"    Distance: {explorer.total_distance:.1f}m")
        print(f"    Efficiency: {metrics.coverage_efficiency:.4f} vertices/m")

    return explorers, bounds


def run_comparison_study(
    max_time=600, drone_speed=0.2, num_trials=30, grid_points_range=(15, 20)
):
    """Run comparison study with randomly varying grid parameters"""

    # Define exploration volume
    bounds = (-8, 8, -4, 4, 2, 6)
    visit_threshold = 2.0

    # Define sweep directions
    sweep_directions = ["xyz", "xzy", "yxz", "yzx", "zxy", "zyx"]

    results = {"Lévy Flight (Bioinspired)": [], "Grid Search": []}
    grid_params = []  # Store grid parameters for each trial

    print(
        "Running exploration comparison study with randomly selected grid parameters..."
    )
    print(
        f"Grid points range: {grid_points_range[0]}-{grid_points_range[1]} (randomly selected)"
    )
    print(f"Sweep directions: {sweep_directions} (randomly selected)")
    print(
        f"Total trials: {num_trials}, Max time: {max_time}s, Drone speed: {drone_speed} m/s"
    )
    print("-" * 80)

    for trial in range(num_trials):
        # Randomly select grid points from range
        grid_pts = np.random.randint(
            grid_points_range[0], grid_points_range[1] + 1
        )

        # Randomly select sweep direction
        sweep_dir = np.random.choice(sweep_directions)

        print(
            f"\nTrial {trial + 1}/{num_trials} - Grid: {grid_pts} pts, Sweep: {sweep_dir}"
        )
        grid_params.append((grid_pts, sweep_dir))

        # Create fresh explorer instances for each trial
        explorers = {
            "Lévy Flight (Bioinspired)": LevyFlightExplorer(
                *bounds, visit_threshold, drone_speed, alpha=1.5
            ),
            "Grid Search": GridSearchExplorer(
                *bounds,
                visit_threshold,
                drone_speed,
                grid_points=grid_pts,
                sweep_direction=sweep_dir,
            ),
        }

        for name, explorer in explorers.items():
            # Run exploration
            current_time = 0.0
            explorer.waypoints.append(
                {
                    "x": explorer.current_pos[0],
                    "y": explorer.current_pos[1],
                    "z": explorer.current_pos[2],
                    "t": current_time,
                }
            )

            while current_time < max_time:
                time_remaining = max_time - current_time
                next_pos = explorer.generate_next_waypoint(
                    time_remaining, max_time
                )

                # Calculate travel time
                distance = np.linalg.norm(next_pos - explorer.current_pos)
                travel_time = distance / drone_speed

                if current_time + travel_time > max_time:
                    break

                # Update state
                explorer.current_pos = next_pos
                current_time += travel_time
                explorer.current_time = current_time
                explorer.total_distance += distance

                # Check corner visits
                explorer.check_corner_visits(next_pos)

                # Record waypoint
                explorer.waypoints.append(
                    {
                        "x": next_pos[0],
                        "y": next_pos[1],
                        "z": next_pos[2],
                        "t": current_time,
                    }
                )

            # Store metrics
            metrics = explorer.get_metrics()
            results[name].append(metrics)

            visited = sum(explorer.visited_vertices)
            print(f"  {name}: {visited}/8 vertices ({visited / 8 * 100:.1f}%)")

    return results, grid_params


def analyze_results(results, grid_params=None):
    """Analyze and visualize comparison results"""
    algorithms = list(results.keys())
    metrics_names = [
        "vertices_visited",
        "completion_rate",
        "coverage_efficiency",
        "time_efficiency",
        "path_smoothness",
        "search_variance",
    ]

    # Calculate statistics
    stats = {}
    for alg in algorithms:
        stats[alg] = {}
        for metric in metrics_names:
            values = [getattr(result, metric) for result in results[alg]]
            stats[alg][metric] = {
                "mean": np.mean(values),
                "std": np.std(values),
                "min": np.min(values),
                "max": np.max(values),
            }

    # Create comparison plots
    fig, axes = plt.subplots(2, 3, figsize=(18, 12))
    axes = axes.flatten()

    metric_labels = {
        "vertices_visited": "Vertices Visited",
        "completion_rate": "Completion Rate (%)",
        "coverage_efficiency": "Coverage Efficiency (vertices/m)",
        "time_efficiency": "Time Efficiency (vertices/s)",
        "path_smoothness": "Path Smoothness (rad)",
        "search_variance": "Search Variance",
    }

    colors = ["#1f77b4", "#ee9b1e"]

    for i, metric in enumerate(metrics_names):
        ax = axes[i]

        means = [stats[alg][metric]["mean"] for alg in algorithms]
        stds = [stats[alg][metric]["std"] for alg in algorithms]

        bars = ax.bar(
            algorithms, means, yerr=stds, capsize=5, color=colors, alpha=0.7
        )
        ax.set_title(metric_labels[metric])
        ax.set_ylabel("Value")

        # Rotate x-axis labels for better readability
        plt.setp(ax.get_xticklabels(), rotation=45, ha="right")

        # Add value labels on bars
        for bar, mean in zip(bars, means):
            height = bar.get_height()
            ax.text(
                bar.get_x() + bar.get_width() / 2.0,
                height,
                f"{mean:.3f}",
                ha="center",
                va="bottom",
            )

    plt.tight_layout()
    plt.show()

    # Analyze grid search parameter effects if available
    if grid_params and "Grid Search" in results:
        analyze_grid_parameter_effects(results["Grid Search"], grid_params)

    # Print detailed statistics
    print("\n" + "=" * 80)
    print("DETAILED COMPARISON RESULTS")
    print("=" * 80)

    for metric in metrics_names:
        print(f"\n{metric_labels[metric]}:")
        print("-" * 40)
        for alg in algorithms:
            mean = stats[alg][metric]["mean"]
            std = stats[alg][metric]["std"]
            print(f"{alg:20s}: {mean:.4f} ± {std:.4f}")

    # Statistical significance test (basic)
    print("\n" + "=" * 80)
    print("PERFORMANCE RANKING")
    print("=" * 80)

    # Rank algorithms by key metrics
    key_metrics = [
        "vertices_visited",
        "completion_rate",
        "coverage_efficiency",
        "time_efficiency",
    ]

    for metric in key_metrics:
        print(f"\n{metric_labels[metric]} (best to worst):")
        ranked = sorted(
            algorithms, key=lambda x: stats[x][metric]["mean"], reverse=True
        )
        for i, alg in enumerate(ranked):
            mean = stats[alg][metric]["mean"]
            print(f"  {i + 1}. {alg}: {mean:.4f}")

    return stats


def analyze_grid_parameter_effects(grid_results, grid_params):
    """Analyze the effect of randomly selected grid parameters on performance"""

    print("\n" + "=" * 80)
    print("GRID SEARCH PARAMETER ANALYSIS (Randomly Selected)")
    print("=" * 80)

    # Group results by grid points
    grid_points_performance = {}
    sweep_direction_performance = {}

    for i, (grid_pts, sweep_dir) in enumerate(grid_params):
        result = grid_results[i]

        # Group by grid points
        if grid_pts not in grid_points_performance:
            grid_points_performance[grid_pts] = []
        grid_points_performance[grid_pts].append(result.completion_rate)

        # Group by sweep direction
        if sweep_dir not in sweep_direction_performance:
            sweep_direction_performance[sweep_dir] = []
        sweep_direction_performance[sweep_dir].append(result.completion_rate)

    # Analyze grid points effect
    print("\nEffect of Grid Points on Completion Rate (Random Sampling):")
    print("-" * 60)
    for grid_pts in sorted(grid_points_performance.keys()):
        rates = grid_points_performance[grid_pts]
        mean_rate = np.mean(rates)
        std_rate = np.std(rates)
        print(
            f"  {grid_pts:2d} points: {mean_rate:.3f} ± {std_rate:.3f} ({len(rates)} trials)"
        )

    # Analyze sweep direction effect
    print("\nEffect of Sweep Direction on Completion Rate (Random Sampling):")
    print("-" * 60)
    for sweep_dir in sorted(sweep_direction_performance.keys()):
        rates = sweep_direction_performance[sweep_dir]
        mean_rate = np.mean(rates)
        std_rate = np.std(rates)
        print(
            f"  {sweep_dir:3s}: {mean_rate:.3f} ± {std_rate:.3f} ({len(rates)} trials)"
        )

    # Show parameter distribution
    print(f"\nParameter Distribution Across {len(grid_params)} Trials:")
    print("-" * 60)
    grid_counts = {}
    sweep_counts = {}

    for grid_pts, sweep_dir in grid_params:
        grid_counts[grid_pts] = grid_counts.get(grid_pts, 0) + 1
        sweep_counts[sweep_dir] = sweep_counts.get(sweep_dir, 0) + 1

    print("Grid Points Distribution:")
    for grid_pts in sorted(grid_counts.keys()):
        print(f"  {grid_pts:2d} points: {grid_counts[grid_pts]:2d} trials")

    print("Sweep Direction Distribution:")
    for sweep_dir in sorted(sweep_counts.keys()):
        print(f"  {sweep_dir:3s}: {sweep_counts[sweep_dir]:2d} trials")

    # Create visualization - only plot if we have sufficient data
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(15, 5))

    # Plot grid points effect (only if we have data for multiple grid points)
    if len(grid_points_performance) > 1:
        grid_pts_sorted = sorted(grid_points_performance.keys())
        means = [
            np.mean(grid_points_performance[pts]) for pts in grid_pts_sorted
        ]
        stds = [
            np.std(grid_points_performance[pts])
            if len(grid_points_performance[pts]) > 1
            else 0
            for pts in grid_pts_sorted
        ]

        ax1.errorbar(grid_pts_sorted, means, yerr=stds, marker="o", capsize=5)
        ax1.set_xlabel("Grid Points")
        ax1.set_ylabel("Completion Rate")
        ax1.set_title(
            "Effect of Grid Points on Performance\n(Randomly Sampled)"
        )
        ax1.grid(True, alpha=0.3)
    else:
        ax1.text(
            0.5,
            0.5,
            "Insufficient data\nfor grid points analysis",
            ha="center",
            va="center",
            transform=ax1.transAxes,
        )
        ax1.set_title("Grid Points Effect")

    # Plot sweep direction effect (only if we have data for multiple directions)
    if len(sweep_direction_performance) > 1:
        sweep_dirs = sorted(sweep_direction_performance.keys())
        means = [np.mean(sweep_direction_performance[d]) for d in sweep_dirs]
        stds = [
            np.std(sweep_direction_performance[d])
            if len(sweep_direction_performance[d]) > 1
            else 0
            for d in sweep_dirs
        ]

        ax2.bar(sweep_dirs, means, yerr=stds, capsize=5, alpha=0.7)
        ax2.set_xlabel("Sweep Direction")
        ax2.set_ylabel("Completion Rate")
        ax2.set_title(
            "Effect of Sweep Direction on Performance\n(Randomly Sampled)"
        )
        ax2.tick_params(axis="x", rotation=45)
    else:
        ax2.text(
            0.5,
            0.5,
            "Insufficient data\nfor sweep direction analysis",
            ha="center",
            va="center",
            transform=ax2.transAxes,
        )
        ax2.set_title("Sweep Direction Effect")

    plt.tight_layout()
    plt.show()


def main():
    """Main function to run the comparison study with trajectory visualization"""
    print("Bioinspired vs Traditional Exploration Algorithms")
    print("=" * 60)

    # Run trajectory comparison first
    print("\n1. TRAJECTORY COMPARISON")
    print("=" * 30)
    explorers_data, bounds = run_trajectory_comparison(
        max_time=600, drone_speed=0.5
    )

    # Plot trajectories
    plot_trajectories(explorers_data, bounds, "3D Exploration Trajectories")

    # Run full comparison study with randomly selected grid parameters
    print("\n2. STATISTICAL COMPARISON WITH RANDOMLY SELECTED GRID PARAMETERS")
    print("=" * 60)
    results, grid_params = run_comparison_study(
        max_time=300,
        drone_speed=0.5,
        num_trials=500,
        grid_points_range=(15, 20),
    )
    stats = analyze_results(results, grid_params)
    return results, stats, grid_params


if __name__ == "__main__":
    results, stats, grid_params = main()
