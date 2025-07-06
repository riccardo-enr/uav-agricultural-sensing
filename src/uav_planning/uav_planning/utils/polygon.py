import matplotlib.pyplot as plt # type: ignore
from mpl_toolkits.mplot3d import Axes3D # type: ignore
import numpy as np

class DeterministicCornerExplorer:
    def __init__(self, x_min, x_max, y_min, y_max, z_min, z_max):
        self.x_min, self.x_max = x_min, x_max
        self.y_min, self.y_max = y_min, y_max
        self.z_min, self.z_max = z_min, z_max
        

        # Define the 8 corners of the bounding box
        self.corners = np.array([
            [x_min, y_min, z_min],  # 0: bottom corners
            [x_min, y_max, z_min],  # 1
            [x_max, y_max, z_min],  # 2
            [x_max, y_min, z_min],  # 3
            [x_min, y_min, z_max],  # 4: top corners
            [x_min, y_max, z_max],  # 5
            [x_max, y_max, z_max],  # 6
            [x_max, y_min, z_max],  # 7
        ],dtype=float)

        self.current_pos = np.array([(x_min + x_max) / 2, 
                             (y_min + y_max) / 2, 
                             (z_min + z_max) / 2], dtype=float)
        
        self.starting_pos = self.current_pos.copy()

        # Define the visiting order: bottom corners first, then diagonally to top
        self.visit_order = [
            0,  # Start: bottom front-left
            1,  # Move to: bottom back-left
            2,  # Move to: bottom back-right
            3,  # Move to: bottom front-right
            5,  # Diagonal to: top back-right
            4,  # Move to: top front-right
            7,  # Diagonal to: top front-left
            6,  # Move to: top back-left

        ]
       
        
        # Track current target
        self.current_target_index = 0
        self.visited_corners = []

    def get_current_target(self):
        if self.current_target_index < len(self.visit_order):
            corner_idx = self.visit_order[self.current_target_index]
            return self.corners[corner_idx], corner_idx
        return None, None

    def generate_next_waypoint(self, steps_per_segment=5):
        """Generate waypoint moving toward current target corner"""
        target_pos, target_idx = self.get_current_target()
        
        if target_pos is None:
            # All corners visited, stay at last position
            return tuple(self.current_pos)
        
        # Calculate direction to target
        direction = target_pos - self.current_pos
        distance = np.linalg.norm(direction)
        
        if distance < 0.1:  # Close enough to corner
            # Reached corner, move to next target
            self.visited_corners.append(target_idx)
            print(f"Reached corner {target_idx}: {target_pos}")
            self.current_target_index += 1
            self.current_pos = target_pos.copy()
        else:
            # Move toward target
            step_size = distance / steps_per_segment
            if distance > 0:
                direction_normalized = direction / distance
                self.current_pos += direction_normalized * step_size

        return tuple(self.current_pos)

    def generate_smooth_path(self, total_waypoints=50):
        waypoints = []
        timestamp = 0
        time_step_seconds = 30

        # First, visit all corners in order
        while self.current_target_index < len(self.visit_order):
            x, y, z = self.generate_next_waypoint(steps_per_segment=5)
            waypoints.append({"x": x, "y": y, "z": z, "t": timestamp})
            timestamp += time_step_seconds

        # Then, return to the starting position
        while np.linalg.norm(self.starting_pos - self.current_pos) > 0.2:
            direction = self.starting_pos - self.current_pos
            distance = np.linalg.norm(direction)
            step_size = distance / 5  # Smooth return
            if distance > 0:
                direction_normalized = direction / distance
                self.current_pos += direction_normalized * step_size
            x, y, z = self.current_pos
            waypoints.append({"x": x, "y": y, "z": z, "t": timestamp})
            timestamp += time_step_seconds

        return waypoints


    

    def get_exploration_info(self):
        """Return information about the exploration"""
        return {
            'visit_order': self.visit_order,
            'corners_visited': len(self.visited_corners),
            'total_corners': len(self.visit_order),
            'visited_corner_indices': self.visited_corners,
            'completion_rate': len(self.visited_corners) / len(self.visit_order)
        }


# --- Usage Example ---
explorer = DeterministicCornerExplorer(
    x_min=-10, x_max=10,
    y_min=-5, y_max=5,
    z_min=2, z_max=6
)

print("Starting deterministic corner exploration...")
print("Visit order: Bottom corners → Diagonal transitions to top corners")
print("-" * 60)

# Generate waypoints
waypoints = explorer.generate_smooth_path(total_waypoints=60)

# Display exploration info
info = explorer.get_exploration_info()
print(f"\nExploration completed!")
print(f"Corners visited: {info['corners_visited']}/{info['total_corners']}")
print(f"Visit order: {info['visit_order']}")
print(f"Visited corners: {info['visited_corner_indices']}")

# Plot results
x_vals = [wp["x"] for wp in waypoints]
y_vals = [wp["y"] for wp in waypoints]
z_vals = [wp["z"] for wp in waypoints]

fig = plt.figure(figsize=(12, 8))
ax = fig.add_subplot(111, projection='3d')

# Plot the path with color progression
colors = plt.cm.plasma(np.linspace(0, 1, len(waypoints)))
ax.scatter(x_vals, y_vals, z_vals, c=colors, s=20, alpha=0.7)
ax.plot(x_vals, y_vals, z_vals, alpha=0.6, linewidth=2, color='blue', label='Flight Path')

# Mark all corners with labels
for i, corner in enumerate(explorer.corners):
    color = 'green' if i in explorer.visited_corners else 'red'
    marker = '^' if corner[2] == explorer.z_min else 'v'  # Triangle up for bottom, down for top
    size = 100 if i in explorer.visited_corners else 60
    
    ax.scatter(corner[0], corner[1], corner[2], 
              color=color, s=size, marker=marker, 
              edgecolors='black', linewidth=1)
    
    # Add corner labels
    ax.text(corner[0], corner[1], corner[2] + 0.3, f'C{i}', 
           fontsize=10, ha='center', weight='bold')

# Add visit order annotations
visit_positions = [explorer.corners[i] for i in explorer.visit_order]
for i, pos in enumerate(visit_positions):
    ax.text(pos[0], pos[1], pos[2] - 0.5, f'{i+1}', 
           fontsize=12, ha='center', color='white', 
           bbox=dict(boxstyle='circle', facecolor='navy', alpha=0.8))

# Formatting
ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_zlabel("Z")
ax.set_title("Deterministic Corner Exploration\nBottom Corners → Diagonal to Top Corners")

# Create custom legend
from matplotlib.lines import Line2D
legend_elements = [
    Line2D([0], [0], color='blue', linewidth=2, label='Flight Path'),
    Line2D([0], [0], marker='^', color='green', linestyle='None', 
           markersize=10, label='Visited Corners'),
    Line2D([0], [0], marker='^', color='red', linestyle='None', 
           markersize=8, label='Unvisited Corners'),
    Line2D([0], [0], marker='o', color='navy', linestyle='None', 
           markersize=8, label='Visit Order')
]
ax.legend(handles=legend_elements, loc='upper right')

plt.tight_layout()
plt.show()

# Print corner coordinates for reference
print("\nCorner coordinates and visit order:")
print("Bottom corners (z=2):")
for i in [0, 1, 2, 3]:
    corner = explorer.corners[i]
    order_pos = explorer.visit_order.index(i) + 1
    print(f"  Corner {i} (visit #{order_pos}): ({corner[0]}, {corner[1]}, {corner[2]})")

print("Top corners (z=6):")
for i in [4, 5, 6, 7]:
    corner = explorer.corners[i]
    order_pos = explorer.visit_order.index(i) + 1
    print(f"  Corner {i} (visit #{order_pos}): ({corner[0]}, {corner[1]}, {corner[2]})")