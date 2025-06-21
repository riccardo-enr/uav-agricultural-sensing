#!/usr/bin/env python3

import sys
import os
import matplotlib.pyplot as plt
import numpy as np

# Add the uav_planning module to the path
sys.path.append(os.path.join(os.path.dirname(__file__), ".."))

from uav_planning.butterfly_generator import ButterflyGenerator


def test_basic_generation():
    """Test basic waypoint generation and plot the results."""
    print("Testing basic butterfly path generation...")

    # Create a butterfly generator with default parameters
    generator = ButterflyGenerator(
        x_min=0.0,
        x_max=20.0,
        y_min=0.0,
        y_max=15.0,
        z_min=1.0,
        z_max=5.0,
        alpha=1.5,
        step_scale=2.0,
    )

    # Generate waypoints
    num_waypoints = 200
    waypoints = []

    for i in range(num_waypoints):
        waypoint = generator.generate_next_waypoint()
        waypoints.append(waypoint)

    # Convert to numpy arrays for easier plotting
    waypoints = np.array(waypoints)
    x_coords = waypoints[:, 0]
    y_coords = waypoints[:, 1]
    z_coords = waypoints[:, 2]

    # Create 2D plot
    plt.figure(figsize=(15, 5))

    # 2D top-down view
    plt.subplot(1, 3, 1)
    plt.plot(x_coords, y_coords, "b-", alpha=0.7, linewidth=1)
    plt.scatter(
        x_coords[0], y_coords[0], color="green", s=100, label="Start", zorder=5
    )
    plt.scatter(
        x_coords[-1], y_coords[-1], color="red", s=100, label="End", zorder=5
    )
    plt.xlim(generator.x_min, generator.x_max)
    plt.ylim(generator.y_min, generator.y_max)
    plt.xlabel("X Coordinate")
    plt.ylabel("Y Coordinate")
    plt.title("Butterfly Path - Top View")
    plt.legend()
    plt.grid(True, alpha=0.3)

    # Altitude profile
    plt.subplot(1, 3, 2)
    plt.plot(range(len(z_coords)), z_coords, "g-", linewidth=1)
    plt.xlabel("Waypoint Index")
    plt.ylabel("Z Coordinate (Altitude)")
    plt.title("Altitude Profile")
    plt.grid(True, alpha=0.3)
    plt.ylim(generator.z_min, generator.z_max)

    # Step size analysis
    plt.subplot(1, 3, 3)
    step_sizes = []
    for i in range(1, len(waypoints)):
        dx = waypoints[i, 0] - waypoints[i - 1, 0]
        dy = waypoints[i, 1] - waypoints[i - 1, 1]
        step_size = np.sqrt(dx**2 + dy**2)
        step_sizes.append(step_size)

    plt.hist(step_sizes, bins=30, alpha=0.7, color="orange")
    plt.xlabel("Step Size")
    plt.ylabel("Frequency")
    plt.title("Distribution of Step Sizes")
    plt.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.show()

    # Create 3D plot
    fig = plt.figure(figsize=(12, 8))
    ax = fig.add_subplot(111, projection="3d")

    # Plot the path
    ax.plot(x_coords, y_coords, z_coords, "b-", alpha=0.7, linewidth=1)
    ax.scatter(
        x_coords[0],
        y_coords[0],
        z_coords[0],
        color="green",
        s=100,
        label="Start",
    )
    ax.scatter(
        x_coords[-1],
        y_coords[-1],
        z_coords[-1],
        color="red",
        s=100,
        label="End",
    )

    # Set labels and limits
    ax.set_xlabel("X Coordinate")
    ax.set_ylabel("Y Coordinate")
    ax.set_zlabel("Z Coordinate (Altitude)")
    ax.set_title("Butterfly Path - 3D View")
    ax.legend()

    # Set equal aspect ratio
    ax.set_xlim(generator.x_min, generator.x_max)
    ax.set_ylim(generator.y_min, generator.y_max)
    ax.set_zlim(generator.z_min, generator.z_max)

    plt.show()

    # Print statistics
    print("\nPath Statistics:")
    print(f"  Total waypoints: {len(waypoints)}")
    print(
        f"  Start position: ({x_coords[0]:.2f}, {y_coords[0]:.2f}, {z_coords[0]:.2f})"
    )
    print(
        f"  End position: ({x_coords[-1]:.2f}, {y_coords[-1]:.2f}, {z_coords[-1]:.2f})"
    )
    print(f"  Average step size: {np.mean(step_sizes):.2f}")
    print(f"  Max step size: {np.max(step_sizes):.2f}")
    print(f"  Min step size: {np.min(step_sizes):.2f}")

    total_distance = np.sum(step_sizes)
    print(f"  Total path length: {total_distance:.2f}")

    return waypoints


def test_multiple_configurations():
    """Test different configurations and compare them."""
    print("\n\nTesting multiple configurations...")

    configs = [
        {"alpha": 1.2, "step_scale": 1.0, "label": "Conservative (α=1.2)"},
        {"alpha": 1.5, "step_scale": 1.5, "label": "Moderate (α=1.5)"},
        {"alpha": 2.0, "step_scale": 2.0, "label": "Aggressive (α=2.0)"},
    ]

    plt.figure(figsize=(15, 5))
    colors = ["blue", "red", "green"]

    for i, config in enumerate(configs):
        generator = ButterflyGenerator(
            x_min=0.0,
            x_max=15.0,
            y_min=0.0,
            y_max=10.0,
            z_min=1.0,
            z_max=3.0,
            alpha=config["alpha"],
            step_scale=config["step_scale"],
        )

        # Generate waypoints
        waypoints = []
        for _ in range(150):
            waypoint = generator.generate_next_waypoint()
            waypoints.append(waypoint)

        waypoints = np.array(waypoints)

        plt.subplot(1, 3, i + 1)
        plt.plot(
            waypoints[:, 0],
            waypoints[:, 1],
            color=colors[i],
            alpha=0.7,
            linewidth=1,
        )
        plt.scatter(
            waypoints[0, 0], waypoints[0, 1], color="green", s=100, zorder=5
        )
        plt.scatter(
            waypoints[-1, 0], waypoints[-1, 1], color="red", s=100, zorder=5
        )
        plt.xlim(0, 15)
        plt.ylim(0, 10)
        plt.xlabel("X Coordinate")
        plt.ylabel("Y Coordinate")
        plt.title(config["label"])
        plt.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.show()


def test_boundary_behavior():
    """Test how the generator handles boundary constraints."""
    print("\n\nTesting boundary behavior...")

    # Create a generator with tight boundaries
    generator = ButterflyGenerator(
        x_min=0.0,
        x_max=5.0,
        y_min=0.0,
        y_max=3.0,
        z_min=1.0,
        z_max=2.0,
        alpha=1.5,
        step_scale=3.0,  # Large step scale to test boundary clipping
    )

    waypoints = []
    for _ in range(100):
        waypoint = generator.generate_next_waypoint()
        waypoints.append(waypoint)

    waypoints = np.array(waypoints)

    plt.figure(figsize=(10, 6))

    plt.subplot(1, 2, 1)
    plt.plot(waypoints[:, 0], waypoints[:, 1], "b-", alpha=0.7, linewidth=1)
    plt.scatter(
        waypoints[0, 0], waypoints[0, 1], color="green", s=100, label="Start"
    )
    plt.scatter(
        waypoints[-1, 0], waypoints[-1, 1], color="red", s=100, label="End"
    )

    # Draw boundary rectangle
    boundary_x = [0, 5, 5, 0, 0]
    boundary_y = [0, 0, 3, 3, 0]
    plt.plot(boundary_x, boundary_y, "r--", linewidth=2, label="Boundaries")

    plt.xlim(-0.5, 5.5)
    plt.ylim(-0.5, 3.5)
    plt.xlabel("X Coordinate")
    plt.ylabel("Y Coordinate")
    plt.title("Boundary Constraint Test")
    plt.legend()
    plt.grid(True, alpha=0.3)

    plt.subplot(1, 2, 2)
    plt.plot(range(len(waypoints)), waypoints[:, 2], "g-", linewidth=1)
    plt.axhline(y=1.0, color="r", linestyle="--", label="Z boundaries")
    plt.axhline(y=2.0, color="r", linestyle="--")
    plt.xlabel("Waypoint Index")
    plt.ylabel("Z Coordinate")
    plt.title("Altitude Constraint Test")
    plt.legend()
    plt.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.show()

    # Verify all waypoints are within boundaries
    within_bounds = (
        np.all(waypoints[:, 0] >= generator.x_min)
        and np.all(waypoints[:, 0] <= generator.x_max)
        and np.all(waypoints[:, 1] >= generator.y_min)
        and np.all(waypoints[:, 1] <= generator.y_max)
        and np.all(waypoints[:, 2] >= generator.z_min)
        and np.all(waypoints[:, 2] <= generator.z_max)
    )

    print(f"All waypoints within boundaries: {within_bounds}")
    if not within_bounds:
        print(
            "  X range: [{:.3f}, {:.3f}] (should be [0.0, 5.0])".format(
                np.min(waypoints[:, 0]), np.max(waypoints[:, 0])
            )
        )
        print(
            "  Y range: [{:.3f}, {:.3f}] (should be [0.0, 3.0])".format(
                np.min(waypoints[:, 1]), np.max(waypoints[:, 1])
            )
        )
        print(
            "  Z range: [{:.3f}, {:.3f}] (should be [1.0, 2.0])".format(
                np.min(waypoints[:, 2]), np.max(waypoints[:, 2])
            )
        )


def main():
    """Main function to run all tests."""
    print("Butterfly Generator Test Suite")
    print("=" * 50)

    try:
        # Test 1: Basic generation
        test_basic_generation()

        # Test 2: Multiple configurations
        test_multiple_configurations()

        # Test 3: Boundary behavior
        test_boundary_behavior()

        print("\n" + "=" * 50)
        print("All tests completed successfully!")

    except Exception as e:
        print(f"Error during testing: {e}")
        import traceback

        traceback.print_exc()


if __name__ == "__main__":
    main()
