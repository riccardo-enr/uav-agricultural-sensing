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
    corners_visited: int
    total_time: float
    total_distance: float
    completion_rate: float
    time_to_first_corner: float
    time_to_last_corner: float
    coverage_efficiency: float  # corners per unit distance
    time_efficiency: float      # corners per unit time
    path_smoothness: float      # measure of path continuity
    search_variance: float      # spatial distribution variance

class BaseExplorer:
    """Base class for exploration algorithms"""
    def __init__(self, x_min, x_max, y_min, y_max, z_min, z_max, visit_threshold, drone_speed):
        self.x_min, self.x_max = x_min, x_max
        self.y_min, self.y_max = y_min, y_max
        self.z_min, self.z_max = z_min, z_max
        self.visit_threshold = visit_threshold
        self.drone_speed = drone_speed
        
        # Define corners
        self.corners = np.array([
            [x_min, y_min, z_min], [x_min, y_min, z_max],
            [x_min, y_max, z_min], [x_min, y_max, z_max],
            [x_max, y_min, z_min], [x_max, y_min, z_max],
            [x_max, y_max, z_min], [x_max, y_max, z_max],
        ])
        
        # Initialize state
        self.current_pos = np.array([(x_min + x_max) / 2, (y_min + y_max) / 2, (z_min + z_max) / 2])
        self.visited_corners = [False] * len(self.corners)
        self.corner_visit_times = {}
        self.current_time = 0.0
        self.total_distance = 0.0
        self.waypoints = []
        
    def clip_position(self, pos):
        """Clip position to bounds"""
        return np.clip(pos, [self.x_min, self.y_min, self.z_min], [self.x_max, self.y_max, self.z_max])
    
    def check_corner_visits(self, pos):
        """Check if any corners are visited"""
        for i, corner in enumerate(self.corners):
            if not self.visited_corners[i] and np.linalg.norm(pos - corner) < self.visit_threshold:
                self.visited_corners[i] = True
                self.corner_visit_times[i] = self.current_time
                return True
        return False
    
    def get_metrics(self) -> ExplorationMetrics:
        """Calculate exploration metrics"""
        visited_count = sum(self.visited_corners)
        completion_rate = visited_count / len(self.corners)
        
        # Time metrics
        visit_times = list(self.corner_visit_times.values())
        time_to_first = min(visit_times) if visit_times else float('inf')
        time_to_last = max(visit_times) if visit_times else float('inf')
        
        # Efficiency metrics
        coverage_efficiency = visited_count / max(self.total_distance, 0.001)
        time_efficiency = visited_count / max(self.current_time, 0.001)
        
        # Path smoothness (average change in direction)
        path_smoothness = self.calculate_path_smoothness()
        
        # Search variance (spatial distribution)
        search_variance = self.calculate_search_variance()
        
        return ExplorationMetrics(
            corners_visited=visited_count,
            total_time=self.current_time,
            total_distance=self.total_distance,
            completion_rate=completion_rate,
            time_to_first_corner=time_to_first,
            time_to_last_corner=time_to_last,
            coverage_efficiency=coverage_efficiency,
            time_efficiency=time_efficiency,
            path_smoothness=path_smoothness,
            search_variance=search_variance
        )
    
    def calculate_path_smoothness(self):
        """Calculate path smoothness metric"""
        if len(self.waypoints) < 3:
            return 0.0
        
        direction_changes = []
        for i in range(1, len(self.waypoints) - 1):
            p1 = np.array([self.waypoints[i-1]['x'], self.waypoints[i-1]['y'], self.waypoints[i-1]['z']])
            p2 = np.array([self.waypoints[i]['x'], self.waypoints[i]['y'], self.waypoints[i]['z']])
            p3 = np.array([self.waypoints[i+1]['x'], self.waypoints[i+1]['y'], self.waypoints[i+1]['z']])
            
            v1 = p2 - p1
            v2 = p3 - p2
            
            if np.linalg.norm(v1) > 0 and np.linalg.norm(v2) > 0:
                cos_angle = np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2))
                cos_angle = np.clip(cos_angle, -1, 1)
                angle_change = np.arccos(cos_angle)
                direction_changes.append(angle_change)
        
        return np.mean(direction_changes) if direction_changes else 0.0
    
    def calculate_search_variance(self):
        """Calculate spatial distribution variance"""
        if len(self.waypoints) < 2:
            return 0.0
        
        positions = np.array([[wp['x'], wp['y'], wp['z']] for wp in self.waypoints])
        return np.mean(np.var(positions, axis=0))


class LevyFlightExplorer(BaseExplorer):
    """Your bioinspired Lévy flight explorer"""
    def __init__(self, x_min, x_max, y_min, y_max, z_min, z_max, visit_threshold, drone_speed, alpha=1.5):
        super().__init__(x_min, x_max, y_min, y_max, z_min, z_max, visit_threshold, drone_speed)
        self.alpha = alpha
        self.max_distance = np.linalg.norm([x_max - x_min, y_max - y_min, z_max - z_min])
        
    def levy_step(self):
        """Generate Lévy flight step"""
        sigma = (math.gamma(1 + self.alpha) * np.sin(np.pi * self.alpha / 2) / 
                (math.gamma((1 + self.alpha) / 2) * self.alpha * 2 ** ((self.alpha - 1) / 2))) ** (1 / self.alpha)
        u = np.random.normal(0, sigma, 3)
        v = np.random.normal(0, 1, 3)
        step = u / (np.abs(v) ** (1 / self.alpha))
        return step
    
    def generate_next_waypoint(self, time_remaining):
        """Generate next waypoint using Lévy flight with corner biasing"""
        unvisited_indices = [i for i in range(len(self.corners)) if not self.visited_corners[i]]
        
        if unvisited_indices:
            # Find nearest unvisited corner
            distances = [np.linalg.norm(self.corners[i] - self.current_pos) for i in unvisited_indices]
            nearest_idx = unvisited_indices[np.argmin(distances)]
            nearest_corner = self.corners[nearest_idx]
            
            # Adaptive bias based on progress
            progress = (len(self.corners) - len(unvisited_indices)) / len(self.corners)
            bias_weight = 0.2 + 1.8 * progress  # Increase bias as more corners are found
            
            # Lévy step with bias
            step = self.levy_step()
            bias = (nearest_corner - self.current_pos) * bias_weight * 0.1
            new_pos = self.current_pos + step + bias
        else:
            # Pure Lévy flight after all corners found
            step = self.levy_step()
            new_pos = self.current_pos + step
        
        return self.clip_position(new_pos)


class GridSearchExplorer(BaseExplorer):
    """Systematic grid search explorer"""
    def __init__(self, x_min, x_max, y_min, y_max, z_min, z_max, visit_threshold, drone_speed, grid_points=10):
        super().__init__(x_min, x_max, y_min, y_max, z_min, z_max, visit_threshold, drone_speed)
        self.grid_points = grid_points
        self.grid_index = 0
        
        # Create grid
        x = np.linspace(x_min, x_max, grid_points)
        y = np.linspace(y_min, y_max, grid_points)
        z = np.linspace(z_min, z_max, grid_points)
        self.grid = np.array(np.meshgrid(x, y, z)).T.reshape(-1, 3)
        
    def generate_next_waypoint(self, time_remaining):
        """Generate next waypoint following grid pattern"""
        if self.grid_index >= len(self.grid):
            # If grid is exhausted, return current position
            return self.current_pos
        
        next_pos = self.grid[self.grid_index]
        self.grid_index += 1
        return next_pos


def plot_trajectories(explorers_data, bounds):
    """
    Plot 3D trajectories for comparison between different exploration algorithms

    Args:
        explorers_data: Dictionary with explorer names as keys and explorer objects as values
        bounds: Tuple of (x_min, x_max, y_min, y_max, z_min, z_max)
        title: Title for the plot
    """
    fig = plt.figure(figsize=(20, 6))
    
    x_min, x_max, y_min, y_max, z_min, z_max = bounds
    colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd', '#8c564b'] 
    
    # Create subplots for each algorithm
    num_explorers = len(explorers_data)
    
    for i, (name, explorer) in enumerate(explorers_data.items()):
        ax = fig.add_subplot(1, num_explorers, i+1, projection='3d')
        
        # Extract trajectory data
        if len(explorer.waypoints) > 0:
            x_traj = [wp['x'] for wp in explorer.waypoints]
            y_traj = [wp['y'] for wp in explorer.waypoints]
            z_traj = [wp['z'] for wp in explorer.waypoints]
            times = [wp['t'] for wp in explorer.waypoints]
            
            # Plot trajectory with color gradient based on time
            if len(x_traj) > 1:
                # Create line segments with color gradient
                for j in range(len(x_traj) - 1):
                    ax.plot([x_traj[j], x_traj[j+1]], 
                           [y_traj[j], y_traj[j+1]], 
                           [z_traj[j], z_traj[j+1]], 
                           color=colors[i % len(colors)], 
                           alpha=0.5, linewidth=0.6)
            
            # Plot trajectory points
            scatter = ax.scatter(x_traj, y_traj, z_traj, 
                               c=times, cmap='viridis', s=20, alpha=0.7)
            
            # Add colorbar for time
            plt.colorbar(scatter, ax=ax, label='Time (s)', 
                       shrink=0.5, location='left', pad=0.001)
        
        # Plot corner points
        corners = explorer.corners
        visited_corners = explorer.visited_corners
        
        # Plot unvisited corners in red
        unvisited_corners = corners[np.array([not v for v in visited_corners])]
        if len(unvisited_corners) > 0:
            ax.scatter(unvisited_corners[:, 0], unvisited_corners[:, 1], unvisited_corners[:, 2], 
                      c='red', s=100, marker='s', alpha=0.8, label='Unvisited corners')
        
        # Plot visited corners in green
        visited_corners_pos = corners[np.array(visited_corners)]
        if len(visited_corners_pos) > 0:
            ax.scatter(visited_corners_pos[:, 0], visited_corners_pos[:, 1], visited_corners_pos[:, 2], 
                      c='green', s=100, marker='s', alpha=0.8, label='Visited corners')
        
        # Plot starting position
        start_pos = explorer.waypoints[0] if explorer.waypoints else {'x': 0, 'y': 0, 'z': 0}
        ax.scatter([start_pos['x']], [start_pos['y']], [start_pos['z']], 
                  c='black', s=150, marker='*', alpha=1.0, label='Start')
        
        # Plot current position
        if len(explorer.waypoints) > 0:
            current_pos = explorer.waypoints[-1]
            ax.scatter([current_pos['x']], [current_pos['y']], [current_pos['z']], 
                      c='black', s=100, marker='o', alpha=1.0, label='Final')
        
        # Draw bounding box
        
        # Bottom face
        ax.plot([x_min, x_max, x_max, x_min, x_min], 
                [y_min, y_min, y_max, y_max, y_min], 
                [z_min, z_min, z_min, z_min, z_min], 'k--', alpha=0.3)
        # Top face
        ax.plot([x_min, x_max, x_max, x_min, x_min], 
                [y_min, y_min, y_max, y_max, y_min], 
                [z_max, z_max, z_max, z_max, z_max], 'k--', alpha=0.3)
        # Vertical edges
        for corner in corners:
            ax.plot([corner[0], corner[0]], [corner[1], corner[1]], 
                   [z_min, z_max], 'k--', alpha=0.3)
        
        # Set labels and title
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_zlabel('Z (m)')
        ax.legend(loc='upper right', fontsize=8)
        
        # Set equal aspect ratio
        ax.set_xlim(x_min, x_max)
        ax.set_ylim(y_min, y_max)
        ax.set_zlim(z_min, z_max)
    
    plt.tight_layout()
    plt.show()


def plot_trajectory_comparison_2d(explorers_data, bounds, title="2D Trajectory Projections"):
    """
    Plot 2D projections of trajectories for easier comparison
    
    Args:
        explorers_data: Dictionary with explorer names as keys and explorer objects as values
        bounds: Tuple of (x_min, x_max, y_min, y_max, z_min, z_max)
        title: Title for the plot
    """
    fig, axes = plt.subplots(2, 2, figsize=(15, 12))
    
    x_min, x_max, y_min, y_max, z_min, z_max = bounds
    colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd', '#8c564b']
    
    projections = [
        (0, 1, 'XY Plane', 'X (m)', 'Y (m)'),
        (0, 2, 'XZ Plane', 'X (m)', 'Z (m)'),
        (1, 2, 'YZ Plane', 'Y (m)', 'Z (m)'),
        (None, None, 'Coverage Over Time', 'Time (s)', 'Corners Visited')
    ]
    
    for proj_idx, (dim1, dim2, proj_name, xlabel, ylabel) in enumerate(projections):
        ax = axes[proj_idx // 2, proj_idx % 2]
        
        if proj_name == 'Coverage Over Time':
            # Plot corners visited over time
            for i, (name, explorer) in enumerate(explorers_data.items()):
                if len(explorer.waypoints) > 0:
                    times = [wp['t'] for wp in explorer.waypoints]
                    corners_found = []
                    count = 0
                    for wp in explorer.waypoints:
                        # Check how many corners were found at this time
                        pos = np.array([wp['x'], wp['y'], wp['z']])
                        for corner in explorer.corners:
                            if np.linalg.norm(pos - corner) < explorer.visit_threshold:
                                count += 1
                                break
                        corners_found.append(min(count, 8))
                    
                    # Use actual corner visit times for more accurate plot
                    visit_times = sorted(explorer.corner_visit_times.values())
                    cumulative_corners = list(range(1, len(visit_times) + 1))
                    
                    if visit_times:
                        ax.plot([0] + visit_times, [0] + cumulative_corners, 
                               color=colors[i % len(colors)], marker='o', 
                               linewidth=2, markersize=4, label=name)
                    
            ax.set_ylim(0, 8)
            ax.set_ylabel(ylabel)
            ax.grid(True, alpha=0.3)
            ax.legend()
            
        else:
            # Plot 2D projections
            for i, (name, explorer) in enumerate(explorers_data.items()):
                if len(explorer.waypoints) > 0:
                    positions = np.array([[wp['x'], wp['y'], wp['z']] for wp in explorer.waypoints])
                    
                    # Plot trajectory
                    ax.plot(positions[:, dim1], positions[:, dim2], 
                           color=colors[i % len(colors)], alpha=0.7, 
                           linewidth=1, label=name)
                    
                    # Plot start and end points
                    ax.scatter(positions[0, dim1], positions[0, dim2], 
                              color=colors[i % len(colors)], s=100, 
                              marker='*', edgecolors='black', linewidth=1, 
                              zorder=5)
                    ax.scatter(positions[-1, dim1], positions[-1, dim2], 
                              color=colors[i % len(colors)], s=80, 
                              marker='s', edgecolors='black', linewidth=1, 
                              zorder=5)
                
                # Plot corners
                corners = explorer.corners
                visited_corners = explorer.visited_corners
                
                # Unvisited corners
                unvisited = corners[np.array([not v for v in visited_corners])]
                if len(unvisited) > 0:
                    ax.scatter(unvisited[:, dim1], unvisited[:, dim2], 
                              c='red', s=60, marker='s', alpha=0.8, 
                              edgecolors='darkred', linewidth=1)
                
                # Visited corners
                visited = corners[np.array(visited_corners)]
                if len(visited) > 0:
                    ax.scatter(visited[:, dim1], visited[:, dim2], 
                              c='green', s=60, marker='s', alpha=0.8, 
                              edgecolors='darkgreen', linewidth=1)
            
            # Set bounds for projections
            bounds_map = {0: (x_min, x_max), 1: (y_min, y_max), 2: (z_min, z_max)}
            ax.set_xlim(bounds_map[dim1])
            ax.set_ylim(bounds_map[dim2])
            ax.grid(True, alpha=0.3)
            ax.legend()
        
        ax.set_title(proj_name)
        ax.set_xlabel(xlabel)
        ax.set_ylabel(ylabel)
    
    plt.tight_layout()
    plt.suptitle(title, fontsize=16, y=1.02)
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
    
    # Initialize explorers
    explorers = {
        'Lévy Flight (Bioinspired)': LevyFlightExplorer(*bounds, visit_threshold, drone_speed, alpha=1.5),
        'Grid Search': GridSearchExplorer(*bounds, visit_threshold, drone_speed, grid_points=8),
    }
    
    print("Running trajectory comparison...")
    print(f"Max time: {max_time}s, Drone speed: {drone_speed} m/s")
    print("-" * 50)
    
    # Run simulation for each explorer
    for name, explorer in explorers.items():
        print(f"Running {name}...")
        
        # Initialize trajectory
        current_time = 0.0
        explorer.waypoints.append({'x': explorer.current_pos[0], 'y': explorer.current_pos[1], 
                                 'z': explorer.current_pos[2], 't': current_time})
        
        # Run exploration
        while current_time < max_time:
            time_remaining = max_time - current_time
            next_pos = explorer.generate_next_waypoint(time_remaining)
            
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
            explorer.waypoints.append({'x': next_pos[0], 'y': next_pos[1], 'z': next_pos[2], 't': current_time})
        
        # Print results
        visited = sum(explorer.visited_corners)
        metrics = explorer.get_metrics()
        print(f"  {name}: {visited}/8 corners ({visited/8*100:.1f}%) in {current_time:.1f}s")
        print(f"    Distance: {explorer.total_distance:.1f}m")
        print(f"    Efficiency: {metrics.coverage_efficiency:.4f} corners/m")
    
    return explorers, bounds


def run_comparison_study(max_time=600, drone_speed=0.2, num_trials=10):
    """Run comparison study between different exploration algorithms"""
    
    # Define exploration volume
    bounds = (-8, 8, -4, 4, 2, 6)
    visit_threshold = 2.0
    
    # Initialize explorers
    explorers = {
        'Lévy Flight (Bioinspired)': LevyFlightExplorer(*bounds, visit_threshold, drone_speed, alpha=1.5),
        'Grid Search': GridSearchExplorer(*bounds, visit_threshold, drone_speed, grid_points=8),
    }
    
    results = {name: [] for name in explorers.keys()}
    
    print("Running exploration comparison study...")
    print(f"Trials: {num_trials}, Max time: {max_time}s, Drone speed: {drone_speed} m/s")
    print("-" * 70)
    
    for trial in range(num_trials):
        print(f"\nTrial {trial + 1}/{num_trials}")
        
        for name, explorer_class in explorers.items():
            # Reset explorer for each trial
            explorer = explorer_class.__class__(*bounds, visit_threshold, drone_speed)
            if hasattr(explorer, 'alpha'):
                explorer.alpha = 1.5
            
            # Run exploration
            current_time = 0.0
            explorer.waypoints.append({'x': explorer.current_pos[0], 'y': explorer.current_pos[1], 
                                     'z': explorer.current_pos[2], 't': current_time})
            
            while current_time < max_time:
                time_remaining = max_time - current_time
                next_pos = explorer.generate_next_waypoint(time_remaining)
                
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
                explorer.waypoints.append({'x': next_pos[0], 'y': next_pos[1], 'z': next_pos[2], 't': current_time})
            
            # Store metrics
            metrics = explorer.get_metrics()
            results[name].append(metrics)
            
            visited = sum(explorer.visited_corners)
            print(f"  {name}: {visited}/8 corners ({visited/8*100:.1f}%)")
    
    return results


def analyze_results(results):
    """Analyze and visualize comparison results"""
    algorithms = list(results.keys())
    metrics_names = ['corners_visited', 'completion_rate', 'coverage_efficiency', 
                     'time_efficiency', 'path_smoothness', 'search_variance']
    
    # Calculate statistics
    stats = {}
    for alg in algorithms:
        stats[alg] = {}
        for metric in metrics_names:
            values = [getattr(result, metric) for result in results[alg]]
            stats[alg][metric] = {
                'mean': np.mean(values),
                'std': np.std(values),
                'min': np.min(values),
                'max': np.max(values)
            }
    
    # Create comparison plots
    fig, axes = plt.subplots(2, 3, figsize=(18, 12))
    axes = axes.flatten()
    
    metric_labels = {
        'corners_visited': 'Corners Visited',
        'completion_rate': 'Completion Rate (%)',
        'coverage_efficiency': 'Coverage Efficiency (corners/m)',
        'time_efficiency': 'Time Efficiency (corners/s)',
        'path_smoothness': 'Path Smoothness (rad)',
        'search_variance': 'Search Variance'
    }
    
    colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728']
    
    for i, metric in enumerate(metrics_names):
        ax = axes[i]
        
        means = [stats[alg][metric]['mean'] for alg in algorithms]
        stds = [stats[alg][metric]['std'] for alg in algorithms]
        
        bars = ax.bar(algorithms, means, yerr=stds, capsize=5, color=colors, alpha=0.7)
        ax.set_title(metric_labels[metric])
        ax.set_ylabel('Value')
        
        # Rotate x-axis labels for better readability
        plt.setp(ax.get_xticklabels(), rotation=45, ha='right')
        
        # Add value labels on bars
        for bar, mean in zip(bars, means):
            height = bar.get_height()
            ax.text(bar.get_x() + bar.get_width()/2., height,
                   f'{mean:.3f}', ha='center', va='bottom')
    
    plt.tight_layout()
    plt.show()
    
    # Print detailed statistics
    print("\n" + "="*80)
    print("DETAILED COMPARISON RESULTS")
    print("="*80)
    
    for metric in metrics_names:
        print(f"\n{metric_labels[metric]}:")
        print("-" * 40)
        for alg in algorithms:
            mean = stats[alg][metric]['mean']
            std = stats[alg][metric]['std']
            print(f"{alg:20s}: {mean:.4f} ± {std:.4f}")
    
    # Statistical significance test (basic)
    print("\n" + "="*80)
    print("PERFORMANCE RANKING")
    print("="*80)
    
    # Rank algorithms by key metrics
    key_metrics = ['corners_visited', 'completion_rate', 'coverage_efficiency', 'time_efficiency']
    
    for metric in key_metrics:
        print(f"\n{metric_labels[metric]} (best to worst):")
        ranked = sorted(algorithms, key=lambda x: stats[x][metric]['mean'], reverse=True)
        for i, alg in enumerate(ranked):
            mean = stats[alg][metric]['mean']
            print(f"  {i+1}. {alg}: {mean:.4f}")
    
    return stats


def main():
    """Main function to run the comparison study with trajectory visualization"""
    print("Bioinspired vs Traditional Exploration Algorithms")
    print("=" * 60)
    
    # Run trajectory comparison first
    print("\n1. TRAJECTORY COMPARISON")
    print("=" * 30)
    explorers_data, bounds = run_trajectory_comparison(max_time=1000, drone_speed=0.5)
    
    # Plot trajectories
    plot_trajectories(explorers_data, bounds)
    plot_trajectory_comparison_2d(explorers_data, bounds, "2D Trajectory Projections")
    

    results = run_comparison_study(max_time=600, drone_speed=0.2, num_trials=100)
    stats = analyze_results(results)
    return results, stats


if __name__ == "__main__":
    results, stats = main()