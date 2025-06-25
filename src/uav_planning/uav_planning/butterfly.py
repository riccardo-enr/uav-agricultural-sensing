import matplotlib.pyplot as plt # type: ignore
import math
import numpy as np

class ButterflyExplorer:
    def __init__(self, x_min, x_max, y_min, y_max, z_min, z_max, alpha, visit_threshold):
        self.x_min, self.x_max = x_min, x_max
        self.y_min, self.y_max = y_min, y_max
        self.z_min, self.z_max = z_min, z_max
        self.alpha = alpha
        self.visit_threshold = visit_threshold

        # Define the 8 corners of the bounding box
        self.corners = np.array([
            [x_min, y_min, z_min],
            [x_min, y_min, z_max],
            [x_min, y_max, z_min],
            [x_min, y_max, z_max],
            [x_max, y_min, z_min],
            [x_max, y_min, z_max],
            [x_max, y_max, z_min],
            [x_max, y_max, z_max],
        ])

        # Start position at center of the volume 
        self.current_pos = np.array([(x_min + x_max) / 2, (y_min + y_max) / 2, (z_min + z_max) / 2])
        self.initial_pos = self.current_pos.copy()

        # Track which corners have been visited
        self.total_corners = len(self.corners)
        self.unvisited_corners = set(range(self.total_corners))
        
        
        # Calculate maximum possible distance in the bounding box for normalization
        self.max_distance = np.linalg.norm([x_max - x_min, y_max - y_min, z_max - z_min])
        
        # Enhanced adaptive parameters for guaranteed corner visits
        self.base_bias_weight = 0.2
        self.max_bias_weight = 1.5  # Increased for stronger corner attraction
        self.urgency_threshold = 0.8  # When to become more aggressive
        
        # Track exploration history for adaptive behavior
        self.steps_since_last_corner = 0
        self.exploration_history = []

    def levy_step(self):

        sigma = (math.gamma(1 + self.alpha) * np.sin(np.pi * self.alpha / 2) / (math.gamma((1 + self.alpha) / 2) * self.alpha * 2 ** ((self.alpha - 1) / 2))) ** (1 / self.alpha)
        u = np.random.normal(0, sigma, 1)
        v = np.random.normal(0, 1, 1)
        step = u / (abs(v) ** (1 / self.alpha))
        return step


    def clip(self, val, valmin, valmax):
        return max(min(val, valmax), valmin)

    def calculate_adaptive_bias_weight(self, distance_to_corner, exploration_ratio):
    
        # Progress factor: exponentially increase bias as more corners are visited
        progress_factor = (1.0 - (len(self.unvisited_corners) / self.total_corners)) ** 2
        
        # Distance factor: stronger bias when closer to corner
        normalized_distance = distance_to_corner / self.max_distance
        distance_factor = (1.0 - normalized_distance) ** 2  # Exponential for stronger effect
       
        if exploration_ratio <= self.urgency_threshold:
            urgency_factor = 1.0
        else:
            # linearly ramp from 1.0 at threshold → max_urgency at ratio=1.0
            max_urgency = 3.0
            t = (exploration_ratio - self.urgency_threshold) / (1.0 - self.urgency_threshold)
            urgency_factor = 1.0 + t * (max_urgency - 1.0)
        
        # Combine factors with adjusted weights
        combined_factor = (
            0.4 * progress_factor +      # Progress weight
            0.4 * distance_factor  +     # Distance weight  
            0.2 * urgency_factor         # Urgency weight
        )
        
        # Scale between base and max bias weight
        adaptive_weight = self.base_bias_weight + (self.max_bias_weight - self.base_bias_weight) * combined_factor
        
        return min(adaptive_weight, self.max_bias_weight)  # Cap at maximum

    def bias_towards(self, target, distance_to_target):

        exploration_ratio = (self.total_corners - len(self.unvisited_corners)) / self.total_corners
        
        # Calculate adaptive bias weight
        bias_weight = self.calculate_adaptive_bias_weight(distance_to_target, exploration_ratio)
        
        # Vector from current position to target
        direction = target - self.current_pos
        direction_magnitude = np.linalg.norm(direction)
        
        if direction_magnitude > 0:
            # Normalize direction
            direction_normalized = direction / direction_magnitude
            
            # Apply adaptive bias with distance scaling
            distance_scale = max(0.2, 1.0 - (distance_to_target / self.max_distance))
            
            # In urgent situations, bias becomes more direct
            if exploration_ratio > self.urgency_threshold:
                # Direct movement toward corner with reduced randomness
                final_bias = direction_normalized * bias_weight * distance_scale * direction_magnitude * 0.5
            else:
                final_bias = direction * bias_weight * distance_scale
        else:
            final_bias = np.zeros(3)
        
        # Store bias info for analysis
        self.exploration_history.append({
            'bias_weight': bias_weight,
            'distance_to_corner': distance_to_target,
            'corners_remaining': len(self.unvisited_corners),
            'steps_since_corner': self.steps_since_last_corner,
            'exploration_ratio': exploration_ratio
        })
        
        return final_bias

    def generate_next_waypoint(self):
        self.steps_since_last_corner += 1
        
        if not self.unvisited_corners:
            # All corners visited, continue with pure Levy flight
            step = self.levy_step()
            new_pos = self.current_pos + step
            
            # Clip to bounding box
            new_pos[0] = self.clip(new_pos[0], self.x_min, self.x_max)
            new_pos[1] = self.clip(new_pos[1], self.y_min, self.y_max)
            new_pos[2] = self.clip(new_pos[2], self.z_min, self.z_max)
        else:
            # Find nearest unvisited corner
            corners_left = np.array([self.corners[i] for i in self.unvisited_corners])
            dists = np.linalg.norm(corners_left - self.current_pos, axis=1)
            nearest_idx = np.argmin(dists)
            nearest_corner = corners_left[nearest_idx]
            distance_to_nearest = dists[nearest_idx]

            # Calculate exploration progress
            exploration_ratio = (self.total_corners - len(self.unvisited_corners)) / self.total_corners
            
            # Adaptive Levy step based on exploration stage
            if exploration_ratio > 0.8 or self.steps_since_last_corner > 15:
                # Late stage or stuck: reduce randomness, increase bias
                step = self.levy_step() * 0.5  # Smaller random steps
            else:
                step = self.levy_step()

            # Get adaptive bias
            bias = self.bias_towards(nearest_corner, distance_to_nearest)
            new_pos = self.current_pos + step + bias

            # Clip new position inside bounding box
            new_pos[0] = self.clip(new_pos[0], self.x_min, self.x_max)
            new_pos[1] = self.clip(new_pos[1], self.y_min, self.y_max)  
            new_pos[2] = self.clip(new_pos[2], self.z_min, self.z_max)

            # Enhanced corner detection with adaptive threshold
            adaptive_threshold = self.visit_threshold
            if self.steps_since_last_corner > 15:
                # Increase threshold if struggling to reach corner
                adaptive_threshold = self.visit_threshold * 1.5 
            
            if np.linalg.norm(new_pos - nearest_corner) < adaptive_threshold:
                # Mark corner as visited
                corner_global_idx = list(self.unvisited_corners)[nearest_idx]
                self.unvisited_corners.remove(corner_global_idx)
                self.steps_since_last_corner = 0  # Reset counter
                print(f"Corner {corner_global_idx} visited! {len(self.unvisited_corners)} corners remaining.")

        self.current_pos = new_pos
        return tuple(self.current_pos)

    def get_exploration_stats(self):
        if not self.exploration_history:
            return {}
        
        history = self.exploration_history
        return {
            'corners_visited': self.total_corners - len(self.unvisited_corners),
            'total_steps': len(history),
            'completion_rate': (self.total_corners - len(self.unvisited_corners)) / self.total_corners,
            'avg_bias_weight': np.mean([h['bias_weight'] for h in history]),
            'max_steps_between_corners': max([h['steps_since_corner'] for h in history]) if history else 0
        }

    def force_complete_exploration(self, max_additional_steps=10):
        additional_steps = 0
        while self.unvisited_corners and additional_steps < max_additional_steps:
            self.generate_next_waypoint()
            additional_steps += 1
        
        return len(self.unvisited_corners) == 0


# --- Usage Example with Guaranteed Corner Exploration ---
num_waypoints = 30
time_step_seconds = 30


explorer = ButterflyExplorer(
    x_min=-8, x_max=8,
    y_min=-4, y_max=4,
    z_min=2, z_max=6,
    alpha=3/2,
    visit_threshold= 1.2,  # Slightly reduced for easier corner detection
)

print("Starting guaranteed corner exploration...")
print(f"Target: Visit all {explorer.total_corners} corners of the bounding box")
print("-" * 50)

def main():
    waypoints = []
    timestamp = 0
    for i in range(num_waypoints):
        x, y, z = explorer.generate_next_waypoint()
        waypoints.append({"x": x, "y": y, "z": z, "t": timestamp})
        timestamp += time_step_seconds

    # Check if all corners were visited, if not, force completion
    if explorer.unvisited_corners:
        print(f"\nForcing completion of remaining {len(explorer.unvisited_corners)} corners...")
        extra_waypoints = []
        completion_successful = explorer.force_complete_exploration()
        
        # Add any additional waypoints generated during forced completion
        while len(waypoints) < num_waypoints + 50:  # Safety limit
            if not explorer.unvisited_corners:
                break
            x, y, z = explorer.generate_next_waypoint()
            extra_waypoints.append({"x": x, "y": y, "z": z, "t": timestamp})
            timestamp += time_step_seconds
        
        waypoints.extend(extra_waypoints)

    # Display final statistics
    stats = explorer.get_exploration_stats()
    print("\n" + "="*50)
    print("EXPLORATION COMPLETED")
    print("="*50)
    print(f"Corners visited: {stats['corners_visited']}/{explorer.total_corners}")
    print(f"Completion rate: {stats['completion_rate']:.1%}")
    print(f"Total waypoints: {len(waypoints)}")
    print(f"All corners explored: {'YES' if len(explorer.unvisited_corners) == 0 else 'NO'}")

    # Plot results
    x_vals = [wp["x"] for wp in waypoints]
    y_vals = [wp["y"] for wp in waypoints]
    z_vals = [wp["z"] for wp in waypoints]

    fig = plt.figure(figsize=(12, 8))
    ax = fig.add_subplot(111, projection='3d')

    # Plot initial starting position
    ax.scatter(explorer.initial_pos[0], explorer.initial_pos[1], explorer.initial_pos[2],
            color='red', s=20, marker='o', label='Start Position')

    # --- Highlight initial path to first waypoint ---
    first_wp = waypoints[0]
    ax.plot(
        [explorer.initial_pos[0], first_wp["x"]],
        [explorer.initial_pos[1], first_wp["y"]],
        [explorer.initial_pos[2], first_wp["z"]],
        c='steelblue', alpha=0.4, linewidth=1
    )

    # Color-code path by progress
    colors = plt.cm.viridis(np.linspace(0, 1, len(waypoints)))
    ax.scatter(x_vals, y_vals, z_vals, c=colors, s=15, alpha=0.7, label="Flight Path")
    ax.plot(x_vals, y_vals, z_vals, c='steelblue', alpha=0.4, linewidth=1)




    # Mark corners - visited vs unvisited
    visited_corners = [explorer.corners[i] for i in range(len(explorer.corners)) if i not in explorer.unvisited_corners]
    unvisited_corners = [explorer.corners[i] for i in explorer.unvisited_corners]

    if visited_corners:
        visited_corners = np.array(visited_corners)
        ax.scatter(visited_corners[:, 0], visited_corners[:, 1], visited_corners[:, 2], 
                color='green', s=100, marker='^', label='Visited Corners')

    if unvisited_corners:
        unvisited_corners = np.array(unvisited_corners)
        ax.scatter(unvisited_corners[:, 0], unvisited_corners[:, 1], unvisited_corners[:, 2], 
                color='red', s=100, marker='^', label='Unvisited Corners')

    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.set_title(f"Guaranteed Corner Exploration - {stats['corners_visited']}/{explorer.total_corners} Corners Visited")
    ax.legend()
    plt.show()

if __name__ == "__main__":
    main()
