#!/usr/bin/env python3
"""GUI monitoring application for the UAV system using PyQtGraph."""

import sys
import time
import os
import threading
from collections import deque
import numpy as np


import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    HistoryPolicy,
    DurabilityPolicy,
)
from geometry_msgs.msg import PoseStamped, TwistStamped
from std_msgs.msg import Bool

import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtWidgets
from PyQt5.QtWidgets import (
    QApplication,
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    QGroupBox,
    QGridLayout,
    QLabel,
    QSplitter,
)


# setup qt environment before any qt imports
def setup_qt_environment():
    """Setup Qt environment for headless/containerized environments."""
    # Set Qt platform plugin for headless environments
    if "DISPLAY" not in os.environ:
        os.environ["QT_QPA_PLATFORM"] = "offscreen"

    # Handle runtime directory permission issues
    if "XDG_RUNTIME_DIR" not in os.environ or not os.access(
        os.environ.get("XDG_RUNTIME_DIR", ""), os.W_OK
    ):
        # Create a temporary runtime directory
        import tempfile

        temp_dir = tempfile.mkdtemp(prefix="qt_runtime_")
        os.environ["XDG_RUNTIME_DIR"] = temp_dir
        os.chmod(temp_dir, 0o700)
        print(f"Created temporary runtime directory: {temp_dir}")


# Setup environment immediately
setup_qt_environment()


class UAVMonitorGUI(Node):
    """Main GUI window for UAV monitoring."""

    def __init__(self):
        super().__init__("uav_monitor_gui")

        self.max_history = 500  # Maximum number of data points to keep

        # Data storage for plotting
        self.time_data = deque(maxlen=self.max_history)
        self.position_data = {
            "x": deque(maxlen=self.max_history),
            "y": deque(maxlen=self.max_history),
            "z": deque(maxlen=self.max_history),
        }
        self.velocity_data = {
            "x": deque(maxlen=self.max_history),
            "y": deque(maxlen=self.max_history),
            "z": deque(maxlen=self.max_history),
            "magnitude": deque(maxlen=self.max_history),
        }

        # State variables
        self.current_position = None
        self.current_velocity = None
        self.current_waypoint = None
        self.armed_status = False

        # Timestamps for data freshness
        self.last_position_time = None
        self.last_velocity_time = None
        self.last_waypoint_time = None
        self.last_armed_time = None

        self.start_time = time.time()

        # Setup ROS subscribers
        self.setup_ros_subscribers()

        # Setup GUI
        self.setup_gui()

        # Setup update timer
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_display)
        self.timer.start(100)  # Update every 100ms

        self.get_logger().info("UAV Monitor GUI Node started")

    def setup_ros_subscribers(self):
        """Setup ROS subscribers."""
        # QoS profile for subscribers
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # Subscribers
        self.position_sub = self.create_subscription(
            PoseStamped,
            "/uav/current_pose",
            self.position_callback,
            qos_profile,
        )

        self.velocity_sub = self.create_subscription(
            TwistStamped,
            "/uav/current_velocity",
            self.velocity_callback,
            qos_profile,
        )

        self.waypoint_sub = self.create_subscription(
            PoseStamped, "/uav/waypoint", self.waypoint_callback, qos_profile
        )

        self.armed_sub = self.create_subscription(
            Bool, "/uav/armed", self.armed_callback, qos_profile
        )

    def setup_gui(self):
        """Setup the user interface."""
        # Main window
        self.main_widget = QtWidgets.QWidget()
        self.main_widget.setWindowTitle("UAV System Monitor")
        self.main_widget.resize(1400, 900)

        # Set dark theme
        self.main_widget.setStyleSheet("""
            QMainWindow { background-color: #2b2b2b; }
            QWidget { background-color: #2b2b2b; color: white; }
            QLabel { color: white; }
            QGroupBox { 
                color: white; 
                border: 1px solid #555555;
                border-radius: 5px;
                margin-top: 10px;
                font-weight: bold;
                padding-top: 15px;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 5px 0 5px;
            }
        """)

        main_layout = QtWidgets.QHBoxLayout(self.main_widget)

        splitter = QtWidgets.QSplitter(QtCore.Qt.Orientation.Horizontal)
        main_layout.addWidget(splitter)

        self.setup_status_pane(splitter)
        self.setup_plots_pane(splitter)

        splitter.setSizes([400, 1000])

        # Show the window only if we have a display
        if (
            "DISPLAY" in os.environ
            and os.environ.get("QT_QPA_PLATFORM") != "offscreen"
        ):
            self.main_widget.show()
            self.get_logger().info("UAV Monitor GUI displayed")
        else:
            self.get_logger().info(
                "Running in headless mode - GUI window not displayed"
            )

    def setup_status_pane(self, parent):
        """Setup the status information pane."""
        status_widget = QtWidgets.QWidget()
        status_layout = QtWidgets.QVBoxLayout(status_widget)

        # Status labels
        self.setup_status_group(status_layout)
        self.setup_waypoint_group(status_layout)
        self.setup_system_group(status_layout)

        status_layout.addStretch()
        parent.addWidget(status_widget)

    def setup_status_group(self, layout):
        """Setup status information group."""
        status_group = QtWidgets.QGroupBox("Current Status")
        status_layout = QtWidgets.QGridLayout(status_group)

        # Position labels
        status_layout.addWidget(QtWidgets.QLabel("Position:"), 0, 0)
        self.position_label = QtWidgets.QLabel("N/A")
        self.position_label.setFont(QtWidgets.QApplication.instance().font())
        status_layout.addWidget(self.position_label, 0, 1)

        # Velocity labels
        status_layout.addWidget(QtWidgets.QLabel("Velocity:"), 1, 0)
        self.velocity_label = QtWidgets.QLabel("N/A")
        self.velocity_label.setFont(QtWidgets.QApplication.instance().font())
        status_layout.addWidget(self.velocity_label, 1, 1)

        # Velocity magnitude
        status_layout.addWidget(QtWidgets.QLabel("Speed:"), 2, 0)
        self.speed_label = QtWidgets.QLabel("N/A")
        self.speed_label.setFont(QtWidgets.QApplication.instance().font())
        status_layout.addWidget(self.speed_label, 2, 1)

        layout.addWidget(status_group)

    def setup_waypoint_group(self, layout):
        """Setup waypoint information group."""
        waypoint_group = QtWidgets.QGroupBox("Waypoint Navigation")
        waypoint_layout = QtWidgets.QGridLayout(waypoint_group)

        # Current waypoint
        waypoint_layout.addWidget(QtWidgets.QLabel("Target:"), 0, 0)
        self.waypoint_label = QtWidgets.QLabel("N/A")
        self.waypoint_label.setFont(QtWidgets.QApplication.instance().font())
        waypoint_layout.addWidget(self.waypoint_label, 0, 1)

        # Distance to waypoint
        waypoint_layout.addWidget(QtWidgets.QLabel("Distance:"), 1, 0)
        self.distance_label = QtWidgets.QLabel("N/A")
        self.distance_label.setFont(QtWidgets.QApplication.instance().font())
        waypoint_layout.addWidget(self.distance_label, 1, 1)

        layout.addWidget(waypoint_group)

    def setup_system_group(self, layout):
        """Setup system status group."""
        system_group = QtWidgets.QGroupBox("System Status")
        system_layout = QtWidgets.QGridLayout(system_group)

        # Armed status
        system_layout.addWidget(QtWidgets.QLabel("Armed:"), 0, 0)
        self.armed_label = QtWidgets.QLabel("N/A")
        self.armed_label.setFont(QtWidgets.QApplication.instance().font())
        system_layout.addWidget(self.armed_label, 0, 1)

        # Data freshness indicators
        system_layout.addWidget(QtWidgets.QLabel("Data Status:"), 1, 0)
        self.data_status_label = QtWidgets.QLabel("N/A")
        self.data_status_label.setFont(QtWidgets.QApplication.instance().font())
        system_layout.addWidget(self.data_status_label, 1, 1)

        layout.addWidget(system_group)

    def setup_plots_pane(self, parent):
        """Setup the plots pane."""
        plots_widget = QtWidgets.QWidget()
        plots_layout = QtWidgets.QVBoxLayout(plots_widget)

        # Create plot widgets
        self.setup_position_plots(plots_layout)
        self.setup_velocity_plots(plots_layout)
        self.setup_3d_trajectory_plot(plots_layout)

        parent.addWidget(plots_widget)

    def setup_position_plots(self, layout):
        """Setup position plots."""
        pos_group = QtWidgets.QGroupBox("Position Over Time")
        pos_layout = QtWidgets.QVBoxLayout(pos_group)

        self.position_plot = pg.PlotWidget()
        self.position_plot.setBackground("k")
        self.position_plot.setLabel("left", "Position", units="m")
        self.position_plot.setLabel("bottom", "Time", units="s")
        self.position_plot.addLegend()

        # Create position curves
        self.pos_x_curve = self.position_plot.plot(
            pen=pg.mkPen("r", width=2), name="X"
        )
        self.pos_y_curve = self.position_plot.plot(
            pen=pg.mkPen("g", width=2), name="Y"
        )
        self.pos_z_curve = self.position_plot.plot(
            pen=pg.mkPen("b", width=2), name="Z"
        )

        pos_layout.addWidget(self.position_plot)
        layout.addWidget(pos_group)

    def setup_velocity_plots(self, layout):
        """Setup velocity plots."""
        vel_group = QtWidgets.QGroupBox("Velocity Over Time")
        vel_layout = QtWidgets.QVBoxLayout(vel_group)

        self.velocity_plot = pg.PlotWidget()
        self.velocity_plot.setBackground("k")
        self.velocity_plot.setLabel("left", "Velocity", units="m/s")
        self.velocity_plot.setLabel("bottom", "Time", units="s")
        self.velocity_plot.addLegend()

        # Create velocity curves
        self.vel_x_curve = self.velocity_plot.plot(
            pen=pg.mkPen("r", width=2), name="Vx"
        )
        self.vel_y_curve = self.velocity_plot.plot(
            pen=pg.mkPen("g", width=2), name="Vy"
        )
        self.vel_z_curve = self.velocity_plot.plot(
            pen=pg.mkPen("b", width=2), name="Vz"
        )
        self.vel_mag_curve = self.velocity_plot.plot(
            pen=pg.mkPen("y", width=3), name="|V|"
        )

        vel_layout.addWidget(self.velocity_plot)
        layout.addWidget(vel_group)

    def setup_3d_trajectory_plot(self, layout):
        """Setup 3D trajectory plot."""
        traj_group = QtWidgets.QGroupBox("3D Trajectory")
        traj_layout = QtWidgets.QVBoxLayout(traj_group)

        traj_splitter = QtWidgets.QSplitter(QtCore.Qt.Orientation.Horizontal)

        # XY plot
        self.xy_plot = pg.PlotWidget()
        self.xy_plot.setBackground("k")
        self.xy_plot.setLabel("left", "Y Position", units="m")
        self.xy_plot.setLabel("bottom", "X Position", units="m")
        self.xy_plot.setAspectLocked(True)
        self.xy_plot.setTitle("XY Plane")
        self.xy_trajectory = self.xy_plot.plot(
            pen=pg.mkPen("w", width=2), symbolBrush="r", symbolSize=5
        )
        self.xy_waypoint = self.xy_plot.plot(
            pen=None, symbolBrush="g", symbolSize=10, symbol="x"
        )
        traj_splitter.addWidget(self.xy_plot)

        # XZ plot
        self.xz_plot = pg.PlotWidget()
        self.xz_plot.setBackground("k")
        self.xz_plot.setLabel("left", "Z Position", units="m")
        self.xz_plot.setLabel("bottom", "X Position", units="m")
        self.xz_plot.setTitle("XZ Plane")
        self.xz_trajectory = self.xz_plot.plot(
            pen=pg.mkPen("w", width=2), symbolBrush="r", symbolSize=5
        )
        self.xz_waypoint = self.xz_plot.plot(
            pen=None, symbolBrush="g", symbolSize=10, symbol="x"
        )
        traj_splitter.addWidget(self.xz_plot)

        traj_layout.addWidget(traj_splitter)
        layout.addWidget(traj_group)

    def update_display(self):
        """Update all display elements."""
        current_time = time.time() - self.start_time

        # Update data if available
        if self.current_position is not None:
            self.time_data.append(current_time)
            for i, axis in enumerate(["x", "y", "z"]):
                self.position_data[axis].append(self.current_position[i])

        if self.current_velocity is not None:
            for i, axis in enumerate(["x", "y", "z"]):
                self.velocity_data[axis].append(self.current_velocity[i])

            # Calculate velocity magnitude
            vel_mag = np.sqrt(sum(v**2 for v in self.current_velocity))
            self.velocity_data["magnitude"].append(vel_mag)

        # Update status labels
        self.update_status_labels()

        # Update plots
        self.update_plots()

    def update_status_labels(self):
        """Update status text labels."""
        # Position
        if self.current_position:
            pos_text = (
                f"[{self.current_position[0]:.2f}, "
                f"{self.current_position[1]:.2f}, "
                f"{self.current_position[2]:.2f}]"
            )
            self.position_label.setText(pos_text)
        else:
            self.position_label.setText("N/A")

        # Velocity
        if self.current_velocity:
            vel_text = (
                f"[{self.current_velocity[0]:.2f}, "
                f"{self.current_velocity[1]:.2f}, "
                f"{self.current_velocity[2]:.2f}]"
            )
            self.velocity_label.setText(vel_text)

            # Speed
            speed = np.sqrt(sum(v**2 for v in self.current_velocity))
            self.speed_label.setText(f"{speed:.2f} m/s")
        else:
            self.velocity_label.setText("N/A")
            self.speed_label.setText("N/A")

        # Waypoint
        if self.current_waypoint:
            wp_text = (
                f"[{self.current_waypoint[0]:.2f}, "
                f"{self.current_waypoint[1]:.2f}, "
                f"{self.current_waypoint[2]:.2f}]"
            )
            self.waypoint_label.setText(wp_text)

            # Distance to waypoint
            if self.current_position:
                distance = np.sqrt(
                    sum(
                        (p - w) ** 2
                        for p, w in zip(
                            self.current_position,
                            self.current_waypoint,
                        )
                    )
                )
                self.distance_label.setText(f"{distance:.2f} m")
            else:
                self.distance_label.setText("N/A")
        else:
            self.waypoint_label.setText("N/A")
            self.distance_label.setText("N/A")

        # Armed status
        if self.last_armed_time:
            armed_text = "ARMED" if self.armed_status else "DISARMED"
            color = "#ff4444" if self.armed_status else "#44ff44"
            self.armed_label.setText(armed_text)
            self.armed_label.setStyleSheet(f"color: {color};")
        else:
            self.armed_label.setText("N/A")
            self.armed_label.setStyleSheet("color: white;")

        # Data freshness
        status_symbols = []
        for name, timestamp in [
            ("Pos", self.last_position_time),
            ("Vel", self.last_velocity_time),
            ("WP", self.last_waypoint_time),
            ("Arm", self.last_armed_time),
        ]:
            symbol = "✓" if self.is_data_fresh(timestamp) else "✗"
            status_symbols.append(f"{name}:{symbol}")

        self.data_status_label.setText(" | ".join(status_symbols))

    def update_plots(self):
        """Update all plots with current data."""
        if len(self.time_data) < 2:
            return

        time_array = np.array(self.time_data)

        # Update position plots
        if len(self.position_data["x"]) > 0:
            self.pos_x_curve.setData(
                time_array, np.array(self.position_data["x"])
            )
            self.pos_y_curve.setData(
                time_array, np.array(self.position_data["y"])
            )
            self.pos_z_curve.setData(
                time_array, np.array(self.position_data["z"])
            )

        # Update velocity plots
        if len(self.velocity_data["x"]) > 0:
            self.vel_x_curve.setData(
                time_array, np.array(self.velocity_data["x"])
            )
            self.vel_y_curve.setData(
                time_array, np.array(self.velocity_data["y"])
            )
            self.vel_z_curve.setData(
                time_array, np.array(self.velocity_data["z"])
            )
            self.vel_mag_curve.setData(
                time_array, np.array(self.velocity_data["magnitude"])
            )

        # Update trajectory plots
        self.update_trajectory_plots()

    def update_trajectory_plots(self):
        """Update 2D trajectory projection plots."""
        if len(self.position_data["x"]) < 2:
            return

        x_data = np.array(self.position_data["x"])
        y_data = np.array(self.position_data["y"])
        z_data = np.array(self.position_data["z"])

        # Update XY trajectory
        self.xy_trajectory.setData(x_data, y_data)

        # Update XZ trajectory
        self.xz_trajectory.setData(x_data, z_data)

        # Update waypoint markers
        if self.current_waypoint:
            wx, wy, wz = self.current_waypoint
            self.xy_waypoint.setData([wx], [wy])
            self.xz_waypoint.setData([wx], [wz])

    def position_callback(self, msg):
        """Update current position."""
        self.current_position = [
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z,
        ]
        self.last_position_time = time.time()

    def velocity_callback(self, msg):
        """Update current velocity."""
        self.current_velocity = [
            msg.twist.linear.x,
            msg.twist.linear.y,
            msg.twist.linear.z,
        ]
        self.last_velocity_time = time.time()

    def waypoint_callback(self, msg):
        """Update current waypoint target."""
        self.current_waypoint = [
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z,
        ]
        self.last_waypoint_time = time.time()

    def armed_callback(self, msg):
        """Update armed status."""
        self.armed_status = msg.data
        self.last_armed_time = time.time()

    def is_data_fresh(self, timestamp, max_age=3.0):
        """Check if data is fresh (received within max_age seconds)."""
        if timestamp is None:
            return False
        return (time.time() - timestamp) < max_age


def run_ros_spin(node):
    """Run ROS spinning in a separate thread."""
    try:
        rclpy.spin(node)
    except Exception as e:
        node.get_logger().error(f"Error in ROS spin: {e}")


def main(args=None):
    """Main function to run the GUI monitor."""
    try:
        # Initialize ROS 2
        rclpy.init(args=args)

        # Create PyQt application if it doesn't exist
        app = QtWidgets.QApplication.instance()
        if app is None:
            app = QtWidgets.QApplication(sys.argv)

        # Create monitor node
        node = UAVMonitorGUI()

        # Start ROS spinning in a separate thread
        ros_thread = threading.Thread(
            target=run_ros_spin, args=(node,), daemon=True
        )
        ros_thread.start()

        # Handle application exit
        def cleanup():
            node.get_logger().info("UAV Monitor GUI shutting down...")
            node.timer.stop()
            node.destroy_node()
            rclpy.shutdown()

        app.aboutToQuit.connect(cleanup)

        # Start Qt event loop
        sys.exit(app.exec_())

    except ImportError as e:
        print(f"Import error: {e}")
        print(
            "Please install required dependencies: pip install PyQt5 pyqtgraph"
        )
        return 1
    except Exception as e:
        print(f"Error starting UAV Monitor GUI: {e}")
        import traceback

        traceback.print_exc()
        return 1
    except KeyboardInterrupt:
        if "node" in locals():
            node.get_logger().info("UAV Monitor GUI shutting down...")
            node.timer.stop()
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
