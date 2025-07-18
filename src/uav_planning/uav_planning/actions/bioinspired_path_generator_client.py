#!/usr/bin/env python3
"""Bio-inspired path generator ROS 2 action client with PyQtGraph GUI - ENU Coordinate System."""

import sys
import os

# CRITICAL: Set Qt environment variables FIRST, before any Qt imports
os.environ["QT_QPA_PLATFORM"] = "xcb"
os.environ["QT_X11_NO_MITSHM"] = "1"

# Create QApplication IMMEDIATELY if this is the main module
_app = None
if __name__ == "__main__":
    # Import Qt modules
    from pyqtgraph.Qt import QtCore, QtWidgets
    import pyqtgraph as pg

    # Create QApplication before anything else
    _app = QtWidgets.QApplication(sys.argv)
    _app.setStyle("Fusion")
    print("QApplication created successfully")

# Now safe to import other modules
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from uav_interfaces.action import PathGeneration

import time
import threading
import numpy as np
from collections import deque

import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtWidgets

# Try to import OpenGL, fall back gracefully if not available
try:
    import pyqtgraph.opengl as gl

    OPENGL_AVAILABLE = True
    print("OpenGL module imported - 3D visualization may be available")
except Exception as e:
    OPENGL_AVAILABLE = False
    print(f"OpenGL not available ({e}) - using 2D visualization only")


class BioinspiredPathGeneratorGUIClient(QtWidgets.QMainWindow):
    """PyQtGraph GUI for the bioinspired path generator action client with ENU coordinate system."""

    # Qt signals for thread-safe GUI updates
    feedback_updated = QtCore.Signal(object)
    result_received = QtCore.Signal(bool, object)
    status_updated = QtCore.Signal(str)

    def __init__(self, app=None):
        # Ensure QApplication exists before calling super().__init__
        if app is None:
            app = QtWidgets.QApplication.instance()
            if app is None:
                print("ERROR: No QApplication found! Creating one...")
                app = QtWidgets.QApplication(sys.argv)

        print("Initializing GUI window...")
        super().__init__()

        # Initialize ROS 2 AFTER Qt initialization
        print("Initializing ROS 2...")
        if not rclpy.ok():
            rclpy.init()

        self.executor = MultiThreadedExecutor()
        self.client_node = PathGeneratorActionClient()
        self.executor.add_node(self.client_node)

        # Start ROS 2 executor in separate thread
        self.executor_thread = threading.Thread(
            target=self.executor.spin, daemon=True
        )
        self.executor_thread.start()
        print("ROS 2 executor started")

        # OpenGL availability will be tested when needed
        self.opengl_available = False
        self.opengl_tested = False

        # Data storage for visualization (ENU coordinates)
        self.trajectory_points = deque(maxlen=1000)
        self.trajectory_east = deque(maxlen=1000)  # East (X in ENU)
        self.trajectory_north = deque(maxlen=1000)  # North (Y in ENU)
        self.trajectory_up = deque(maxlen=1000)  # Up (Z in ENU)
        self.waypoint_history = deque(maxlen=100)
        self.time_data = deque(maxlen=100)
        self.completion_data = deque(maxlen=100)

        # Current waypoint data (ENU)
        self.current_waypoint_east = None
        self.current_waypoint_north = None
        self.current_waypoint_up = None

        # 3D visualization variables
        self.trajectory_data = np.empty((0, 3))
        self.last_trajectory_length = 0

        # View range tracking for stable visualization
        self.view_ranges_set = False
        self.initial_bounds = None

        # Connect signals
        self.feedback_updated.connect(self.update_feedback_display)
        self.result_received.connect(self.handle_result)
        self.status_updated.connect(self.update_status)

        # Set up GUI
        print("Setting up UI...")
        self.setup_ui()
        self.setup_plots()

        # Timer for updating plots
        self.plot_timer = QtCore.QTimer()
        self.plot_timer.timeout.connect(self.update_plots)
        self.plot_timer.start(100)  # Update at 10 Hz

        # Goal tracking
        self.goal_active = False
        self.start_time = None

        print("GUI initialization complete!")

    def test_opengl_context(self):
        """Test if OpenGL context can be created safely."""
        if not OPENGL_AVAILABLE or self.opengl_tested:
            return self.opengl_available

        self.opengl_tested = True

        try:
            # Test OpenGL context creation
            test_widget = gl.GLViewWidget()
            test_widget.setCameraPosition(distance=10)

            # Try to create a simple item to test the context
            test_item = gl.GLLinePlotItem(pos=np.array([[0, 0, 0], [1, 1, 1]]))
            test_widget.addItem(test_item)
            test_widget.removeItem(test_item)

            # Clean up
            test_widget.close()
            del test_widget
            del test_item

            print("OpenGL context test successful - 3D visualization enabled")
            self.opengl_available = True
            return True

        except Exception as e:
            print(
                f"OpenGL context test failed ({e}) - falling back to 2D visualization"
            )
            self.opengl_available = False
            return False

    def setup_ui(self):
        """Set up the main user interface."""
        self.setWindowTitle(
            "Bioinspired Path Generator - UAV Control GUI (ENU)"
        )
        self.setGeometry(100, 100, 1400, 900)

        # Central widget and main layout
        central_widget = QtWidgets.QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QtWidgets.QHBoxLayout(central_widget)

        # Left panel for controls
        control_panel = self.create_control_panel()
        main_layout.addWidget(control_panel, 1)

        # Right panel for plots
        plot_panel = self.create_plot_panel()
        main_layout.addWidget(plot_panel, 3)

        # Update UI after OpenGL testing
        self.add_opengl_status()
        self.update_window_title()

    def update_window_title(self):
        """Update window title based on OpenGL availability."""
        title = "Bioinspired Path Generator - UAV Control GUI (ENU)"
        if not self.opengl_available:
            title += " (2D Mode)"
        self.setWindowTitle(title)

    def create_control_panel(self):
        """Create the control panel with ENU coordinate parameters and status."""
        panel = QtWidgets.QWidget()
        panel.setMaximumWidth(380)
        layout = QtWidgets.QVBoxLayout(panel)

        # Title
        title = QtWidgets.QLabel("UAV Path Generator Control")
        title.setStyleSheet("font-size: 16px; font-weight: bold; margin: 10px;")
        layout.addWidget(title)

        # Coordinate system info
        coord_info = QtWidgets.QLabel(
            "🧭 Coordinate System: ENU (East-North-Up)"
        )
        coord_info.setStyleSheet(
            "color: #2196F3; font-weight: bold; margin: 5px;"
        )
        coord_info.setWordWrap(True)
        layout.addWidget(coord_info)

        # Parameters group
        params_group = QtWidgets.QGroupBox("Exploration Volume (ENU Frame)")
        params_layout = QtWidgets.QFormLayout(params_group)

        # East bounds (X in ENU)
        self.east_min_spin = QtWidgets.QDoubleSpinBox()
        self.east_min_spin.setRange(-100, 100)
        self.east_min_spin.setValue(-10.0)
        self.east_min_spin.setSuffix(" m")
        self.east_min_spin.setToolTip(
            "Minimum East coordinate (X in ENU frame)"
        )
        params_layout.addRow("East Min:", self.east_min_spin)

        self.east_max_spin = QtWidgets.QDoubleSpinBox()
        self.east_max_spin.setRange(-100, 100)
        self.east_max_spin.setValue(10.0)
        self.east_max_spin.setSuffix(" m")
        self.east_max_spin.setToolTip(
            "Maximum East coordinate (X in ENU frame)"
        )
        params_layout.addRow("East Max:", self.east_max_spin)

        # North bounds (Y in ENU)
        self.north_min_spin = QtWidgets.QDoubleSpinBox()
        self.north_min_spin.setRange(-100, 100)
        self.north_min_spin.setValue(-10.0)
        self.north_min_spin.setSuffix(" m")
        self.north_min_spin.setToolTip(
            "Minimum North coordinate (Y in ENU frame)"
        )
        params_layout.addRow("North Min:", self.north_min_spin)

        self.north_max_spin = QtWidgets.QDoubleSpinBox()
        self.north_max_spin.setRange(-100, 100)
        self.north_max_spin.setValue(10.0)
        self.north_max_spin.setSuffix(" m")
        self.north_max_spin.setToolTip(
            "Maximum North coordinate (Y in ENU frame)"
        )
        params_layout.addRow("North Max:", self.north_max_spin)

        # Up bounds (Z in ENU)
        self.up_min_spin = QtWidgets.QDoubleSpinBox()
        self.up_min_spin.setRange(-100, 100)
        self.up_min_spin.setValue(2.0)
        self.up_min_spin.setSuffix(" m")
        self.up_min_spin.setToolTip(
            "Minimum Up coordinate (Z in ENU frame) - negative for below ground"
        )
        params_layout.addRow("Up Min:", self.up_min_spin)

        self.up_max_spin = QtWidgets.QDoubleSpinBox()
        self.up_max_spin.setRange(-100, 100)
        self.up_max_spin.setValue(5.0)
        self.up_max_spin.setSuffix(" m")
        self.up_max_spin.setToolTip(
            "Maximum Up coordinate (Z in ENU frame) - negative for below ground"
        )
        params_layout.addRow("Up Max:", self.up_max_spin)

        # Algorithm parameters
        params_layout.addRow(QtWidgets.QLabel(""))  # Spacer

        self.alpha_spin = QtWidgets.QDoubleSpinBox()
        self.alpha_spin.setRange(0.1, 3.0)
        self.alpha_spin.setValue(1.5)
        self.alpha_spin.setDecimals(2)
        self.alpha_spin.setSingleStep(0.1)
        params_layout.addRow("Alpha (Levy):", self.alpha_spin)

        self.visit_threshold_spin = QtWidgets.QDoubleSpinBox()
        self.visit_threshold_spin.setRange(0.1, 5.0)
        self.visit_threshold_spin.setValue(1.2)
        self.visit_threshold_spin.setDecimals(1)
        self.visit_threshold_spin.setSuffix(" m")
        params_layout.addRow("Visit Threshold:", self.visit_threshold_spin)

        self.exploration_time_spin = QtWidgets.QDoubleSpinBox()
        self.exploration_time_spin.setRange(10, 3600)
        self.exploration_time_spin.setValue(300.0)
        self.exploration_time_spin.setSuffix(" s")
        params_layout.addRow("Max Time:", self.exploration_time_spin)

        layout.addWidget(params_group)

        # Control buttons
        button_layout = QtWidgets.QVBoxLayout()

        self.start_button = QtWidgets.QPushButton("Start Exploration")
        self.start_button.setStyleSheet(
            "QPushButton { background-color: #4CAF50; color: white; font-weight: bold; padding: 10px; }"
        )
        self.start_button.clicked.connect(self.start_exploration)
        button_layout.addWidget(self.start_button)

        self.cancel_button = QtWidgets.QPushButton("Cancel Mission")
        self.cancel_button.setStyleSheet(
            "QPushButton { background-color: #f44336; color: white; font-weight: bold; padding: 10px; }"
        )
        self.cancel_button.clicked.connect(self.cancel_exploration)
        self.cancel_button.setEnabled(False)
        button_layout.addWidget(self.cancel_button)

        self.clear_button = QtWidgets.QPushButton("Clear Visualization")
        self.clear_button.clicked.connect(self.clear_visualization)
        button_layout.addWidget(self.clear_button)

        layout.addLayout(button_layout)

        # Status group
        status_group = QtWidgets.QGroupBox("Mission Status")
        status_layout = QtWidgets.QVBoxLayout(status_group)

        self.status_label = QtWidgets.QLabel("Ready")
        self.status_label.setWordWrap(True)
        status_layout.addWidget(self.status_label)

        # Progress bars
        self.time_progress = QtWidgets.QProgressBar()
        self.time_progress.setFormat("Time: %p%")
        status_layout.addWidget(self.time_progress)

        self.completion_progress = QtWidgets.QProgressBar()
        self.completion_progress.setFormat("Completion: %p%")
        status_layout.addWidget(self.completion_progress)

        layout.addWidget(status_group)

        # Real-time data display (ENU coordinates)
        data_group = QtWidgets.QGroupBox("Real-time Data (ENU)")
        data_layout = QtWidgets.QFormLayout(data_group)

        self.position_label = QtWidgets.QLabel("N/A")
        data_layout.addRow("Position (E,N,U):", self.position_label)

        self.waypoint_label = QtWidgets.QLabel("N/A")
        data_layout.addRow("Waypoint (E,N,U):", self.waypoint_label)

        self.waypoints_count_label = QtWidgets.QLabel("0")
        data_layout.addRow("Waypoints:", self.waypoints_count_label)

        self.corners_label = QtWidgets.QLabel("0/0")
        data_layout.addRow("Corners:", self.corners_label)

        self.vehicle_status_label = QtWidgets.QLabel("Disarmed/Manual")
        data_layout.addRow("Vehicle:", self.vehicle_status_label)

        layout.addWidget(data_group)

        layout.addStretch()

        # Store layout reference for adding OpenGL status later
        self.control_layout = layout
        return panel

    def add_opengl_status(self):
        """Add OpenGL status to control panel after testing."""
        if not self.opengl_available:
            status_label = QtWidgets.QLabel(
                "⚠️ Running in 2D mode (OpenGL unavailable)"
            )
            status_label.setStyleSheet("color: orange; font-weight: bold;")
            # Insert after title, before parameters
            self.control_layout.insertWidget(2, status_label)

    def create_plot_panel(self):
        """Create the plot panel with ENU coordinate visualization."""
        panel = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(panel)

        # Tab widget for different views
        tab_widget = QtWidgets.QTabWidget()

        # Try to create 3D view with proper OpenGL context testing
        if self.test_opengl_context():
            try:
                self.gl_widget = gl.GLViewWidget()
                self.gl_widget.setCameraPosition(
                    distance=30, elevation=20, azimuth=45
                )
                self.gl_widget.setBackgroundColor("k")

                # Disable auto-scaling for stable view
                self.gl_widget.opts["center"] = pg.Vector(0, 0, 0)
                self.gl_widget.opts["distance"] = 30

                tab_widget.addTab(self.gl_widget, "3D Trajectory (ENU)")
                self.use_3d = True
                print("3D visualization successfully initialized")
            except Exception as e:
                print(f"Failed to create 3D view: {e}")
                self.use_3d = False
                self.opengl_available = False
        else:
            self.use_3d = False

        # 2D Trajectory tab (always available) - ENU projections
        trajectory_widget = pg.GraphicsLayoutWidget()

        # East-North trajectory plot (top-down view)
        self.en_plot = trajectory_widget.addPlot(
            title="East-North Trajectory (Top View)", row=0, col=0
        )
        self.en_plot.setLabel("left", "North Position (m)")
        self.en_plot.setLabel("bottom", "East Position (m)")
        self.en_plot.setAspectLocked(True)
        self.en_plot.showGrid(True, True)

        # Configure stable view settings
        self.en_plot.setMouseEnabled(x=True, y=True)
        self.en_plot.enableAutoRange(x=False, y=False)
        self.en_plot.getViewBox().setAutoPan(False)

        self.en_trajectory = self.en_plot.plot(
            pen=pg.mkPen("b", width=2), name="Trajectory"
        )
        self.en_current_pos = self.en_plot.plot(
            pen=None,
            symbol="o",
            symbolBrush="r",
            symbolSize=10,
            name="Current Position",
        )
        self.en_waypoint = self.en_plot.plot(
            pen=None,
            symbol="s",
            symbolBrush="g",
            symbolSize=8,
            name="Target Waypoint",
        )
        self.en_boundary = None

        # East-Up trajectory plot (side view from South)
        self.eu_plot = trajectory_widget.addPlot(
            title="East-Up Trajectory (Side View from South)", row=0, col=1
        )
        self.eu_plot.setLabel("left", "Up Position (m)")
        self.eu_plot.setLabel("bottom", "East Position (m)")
        self.eu_plot.showGrid(True, True)

        self.eu_plot.setMouseEnabled(x=True, y=True)
        self.eu_plot.enableAutoRange(x=False, y=False)
        self.eu_plot.getViewBox().setAutoPan(False)

        self.eu_trajectory = self.eu_plot.plot(
            pen=pg.mkPen("b", width=2), name="Trajectory"
        )
        self.eu_current_pos = self.eu_plot.plot(
            pen=None,
            symbol="o",
            symbolBrush="r",
            symbolSize=10,
            name="Current Position",
        )
        self.eu_waypoint = self.eu_plot.plot(
            pen=None,
            symbol="s",
            symbolBrush="g",
            symbolSize=8,
            name="Target Waypoint",
        )

        trajectory_widget.nextRow()

        # North-Up trajectory plot (side view from West)
        self.nu_plot = trajectory_widget.addPlot(
            title="North-Up Trajectory (Side View from West)", row=1, col=0
        )
        self.nu_plot.setLabel("left", "Up Position (m)")
        self.nu_plot.setLabel("bottom", "North Position (m)")
        self.nu_plot.showGrid(True, True)

        self.nu_plot.setMouseEnabled(x=True, y=True)
        self.nu_plot.enableAutoRange(x=False, y=False)
        self.nu_plot.getViewBox().setAutoPan(False)

        self.nu_trajectory = self.nu_plot.plot(
            pen=pg.mkPen("b", width=2), name="Trajectory"
        )
        self.nu_current_pos = self.nu_plot.plot(
            pen=None,
            symbol="o",
            symbolBrush="r",
            symbolSize=10,
            name="Current Position",
        )
        self.nu_waypoint = self.nu_plot.plot(
            pen=None,
            symbol="s",
            symbolBrush="g",
            symbolSize=8,
            name="Target Waypoint",
        )

        # Altitude profile over time
        self.alt_plot = trajectory_widget.addPlot(
            title="Altitude Profile (Up Coordinate)", row=1, col=1
        )
        self.alt_plot.setLabel("left", "Up Position (m)")
        self.alt_plot.setLabel("bottom", "Time (s)")
        self.alt_plot.showGrid(True, True)

        self.alt_plot.setMouseEnabled(x=True, y=True)
        self.alt_plot.enableAutoRange(x=False, y=False)
        self.alt_plot.getViewBox().setAutoPan(False)

        self.alt_curve = self.alt_plot.plot(
            pen=pg.mkPen("g", width=2), name="Altitude"
        )
        self.alt_target = self.alt_plot.plot(
            pen=pg.mkPen("r", style=QtCore.Qt.DashLine), name="Target Altitude"
        )

        tab_widget.addTab(trajectory_widget, "2D Trajectories (ENU)")

        # Progress plots tab
        progress_widget = pg.GraphicsLayoutWidget()

        # Progress over time plot
        self.progress_plot = progress_widget.addPlot(
            title="Mission Progress", row=0, col=0
        )
        self.progress_plot.setLabel("left", "Completion Rate (%)")
        self.progress_plot.setLabel("bottom", "Time (s)")
        self.progress_plot.showGrid(True, True)

        self.progress_plot.setMouseEnabled(x=True, y=True)
        self.progress_plot.enableAutoRange(x=False, y=False)
        self.progress_plot.getViewBox().setAutoPan(False)
        self.progress_plot.setYRange(0, 1000)

        self.progress_curve = self.progress_plot.plot(
            pen=pg.mkPen("g", width=2), name="Completion Rate"
        )

        # Waypoint generation rate
        self.waypoint_plot = progress_widget.addPlot(
            title="Waypoint Generation", row=0, col=1
        )
        self.waypoint_plot.setLabel("left", "Waypoints Generated")
        self.waypoint_plot.setLabel("bottom", "Time (s)")
        self.waypoint_plot.showGrid(True, True)

        self.waypoint_plot.setMouseEnabled(x=True, y=True)
        self.waypoint_plot.enableAutoRange(x=False, y=False)
        self.waypoint_plot.getViewBox().setAutoPan(False)

        self.waypoint_curve = self.waypoint_plot.plot(
            pen=pg.mkPen("b", width=2), name="Waypoints"
        )

        tab_widget.addTab(progress_widget, "Progress & Stats")

        layout.addWidget(tab_widget)
        return panel

    def setup_plots(self):
        """Set up plot elements."""
        # Initialize 3D plot items as None
        self.trajectory_plot = None
        self.current_position_plot = None
        self.current_waypoint_plot = None
        self.boundary_plot = None
        self.trail_plot = None

        if self.use_3d:
            try:
                # Create coordinate axes (ENU frame)
                self.create_enu_axes()
                # Initialize 3D grid for better depth perception
                self.create_3d_grid()
            except Exception as e:
                print(f"Error setting up 3D plots: {e}")
                self.use_3d = False

    def create_enu_axes(self):
        """Create ENU coordinate system axes for 3D view."""
        if not self.use_3d:
            return

        try:
            # East axis (red) - X direction
            east_axis = gl.GLLinePlotItem(
                pos=np.array([[0, 0, 0], [5, 0, 0]]),
                color=(1, 0, 0, 1),
                width=3,
                antialias=True,
            )
            self.gl_widget.addItem(east_axis)

            # North axis (green) - Y direction
            north_axis = gl.GLLinePlotItem(
                pos=np.array([[0, 0, 0], [0, 5, 0]]),
                color=(0, 1, 0, 1),
                width=3,
                antialias=True,
            )
            self.gl_widget.addItem(north_axis)

            # Up axis (blue) - Z direction
            up_axis = gl.GLLinePlotItem(
                pos=np.array([[0, 0, 0], [0, 0, 5]]),
                color=(0, 0, 1, 1),
                width=3,
                antialias=True,
            )
            self.gl_widget.addItem(up_axis)
        except Exception as e:
            print(f"Error creating ENU axes: {e}")
            self.use_3d = False

    def create_3d_grid(self):
        """Create a grid for better 3D depth perception."""
        if not self.use_3d:
            return

        try:
            grid_size = 20
            grid_spacing = 2
            grid_lines = []

            # Horizontal lines (East-West)
            for i in range(-grid_size // 2, grid_size // 2 + 1, grid_spacing):
                grid_lines.extend(
                    [[-grid_size // 2, i, 0], [grid_size // 2, i, 0]]
                )

            # Vertical lines (North-South)
            for i in range(-grid_size // 2, grid_size // 2 + 1, grid_spacing):
                grid_lines.extend(
                    [[i, -grid_size // 2, 0], [i, grid_size // 2, 0]]
                )

            if len(grid_lines) > 0:
                grid = gl.GLLinePlotItem(
                    pos=np.array(grid_lines),
                    color=(0.3, 0.3, 0.3, 0.5),
                    width=1,
                    antialias=True,
                )
                self.gl_widget.addItem(grid)
        except Exception as e:
            print(f"Error creating 3D grid: {e}")

    def set_plot_ranges(self):
        """Set fixed plot ranges based on ENU boundary parameters."""
        east_min, east_max = (
            self.east_min_spin.value(),
            self.east_max_spin.value(),
        )
        north_min, north_max = (
            self.north_min_spin.value(),
            self.north_max_spin.value(),
        )
        up_min, up_max = self.up_min_spin.value(), self.up_max_spin.value()

        # Add 10% padding to ranges
        east_padding = (east_max - east_min) * 0.1
        north_padding = (north_max - north_min) * 0.1
        up_padding = (up_max - up_min) * 0.1

        # Set ranges for 2D plots (ENU projections)
        self.en_plot.setXRange(east_min - east_padding, east_max + east_padding)
        self.en_plot.setYRange(
            north_min - north_padding, north_max + north_padding
        )

        self.eu_plot.setXRange(east_min - east_padding, east_max + east_padding)
        self.eu_plot.setYRange(up_min - up_padding, up_max + up_padding)

        self.nu_plot.setXRange(
            north_min - north_padding, north_max + north_padding
        )
        self.nu_plot.setYRange(up_min - up_padding, up_max + up_padding)

        # Mark that ranges have been set
        self.view_ranges_set = True

    def update_boundary_visualization(self):
        """Update the ENU boundary box visualization."""
        east_min, east_max = (
            self.east_min_spin.value(),
            self.east_max_spin.value(),
        )
        north_min, north_max = (
            self.north_min_spin.value(),
            self.north_max_spin.value(),
        )
        up_min, up_max = self.up_min_spin.value(), self.up_max_spin.value()

        # Set fixed plot ranges if not already set
        if not self.view_ranges_set:
            self.set_plot_ranges()

        # Update 2D boundaries
        try:
            # East-North boundary (top view)
            if hasattr(self, "en_boundary") and self.en_boundary:
                self.en_plot.removeItem(self.en_boundary)
            boundary_east = [east_min, east_max, east_max, east_min, east_min]
            boundary_north = [
                north_min,
                north_min,
                north_max,
                north_max,
                north_min,
            ]
            self.en_boundary = self.en_plot.plot(
                boundary_east,
                boundary_north,
                pen=pg.mkPen("r", width=2, style=QtCore.Qt.DashLine),
                name="Boundary",
            )

            # East-Up boundary (side view from South)
            if hasattr(self, "eu_boundary") and self.eu_boundary:
                self.eu_plot.removeItem(self.eu_boundary)
            boundary_east_eu = [
                east_min,
                east_max,
                east_max,
                east_min,
                east_min,
            ]
            boundary_up = [up_min, up_min, up_max, up_max, up_min]
            self.eu_boundary = self.eu_plot.plot(
                boundary_east_eu,
                boundary_up,
                pen=pg.mkPen("r", width=2, style=QtCore.Qt.DashLine),
                name="Boundary",
            )

            # North-Up boundary (side view from West)
            if hasattr(self, "nu_boundary") and self.nu_boundary:
                self.nu_plot.removeItem(self.nu_boundary)
            boundary_north_nu = [
                north_min,
                north_max,
                north_max,
                north_min,
                north_min,
            ]
            boundary_up_nu = [up_min, up_min, up_max, up_max, up_min]
            self.nu_boundary = self.nu_plot.plot(
                boundary_north_nu,
                boundary_up_nu,
                pen=pg.mkPen("r", width=2, style=QtCore.Qt.DashLine),
                name="Boundary",
            )
        except Exception as e:
            print(f"Error updating 2D boundaries: {e}")

        # Initialize boundary attributes if they don't exist
        if not hasattr(self, "eu_boundary"):
            self.eu_boundary = None
        if not hasattr(self, "nu_boundary"):
            self.nu_boundary = None

    def start_exploration(self):
        """Start the exploration mission."""
        if self.goal_active:
            return

        # Update boundary visualization
        self.update_boundary_visualization()

        # Clear previous data
        self.clear_data()

        # Prepare goal parameters (convert ENU to expected format)
        goal_params = {
            "x_min": self.east_min_spin.value(),  # East -> X
            "x_max": self.east_max_spin.value(),  # East -> X
            "y_min": self.north_min_spin.value(),  # North -> Y
            "y_max": self.north_max_spin.value(),  # North -> Y
            "z_min": self.up_min_spin.value(),  # Up -> Z
            "z_max": self.up_max_spin.value(),  # Up -> Z
            "alpha": self.alpha_spin.value(),
            "visit_threshold": self.visit_threshold_spin.value(),
            "exploration_time": self.exploration_time_spin.value(),
            "feedback_callback": self.feedback_callback,
        }

        # Send goal in separate thread
        self.goal_thread = threading.Thread(
            target=self.send_goal_thread, args=(goal_params,)
        )
        self.goal_thread.start()

        # Update UI state
        self.goal_active = True
        self.start_time = time.time()
        self.start_button.setEnabled(False)
        self.cancel_button.setEnabled(True)
        self.status_updated.emit("Connecting to action server...")

    def send_goal_thread(self, params):
        """Send goal in separate thread."""
        try:
            if not self.client_node.wait_for_server(timeout_sec=5.0):
                self.status_updated.emit("Error: Action server not available")
                self.reset_ui_state()
                return

            success, result = self.client_node.send_goal_and_wait(**params)
            self.result_received.emit(success, result)

        except Exception as e:
            self.status_updated.emit(f"Error: {str(e)}")
            self.reset_ui_state()

    def cancel_exploration(self):
        """Cancel the current exploration."""
        if self.goal_active:
            self.client_node.cancel_goal()
            self.status_updated.emit("Cancelling mission...")

    def clear_visualization(self):
        """Clear all visualization data."""
        self.clear_data()
        self.view_ranges_set = False
        self.update_plots()

    def clear_data(self):
        """Clear stored data."""
        self.trajectory_points.clear()
        self.trajectory_east.clear()
        self.trajectory_north.clear()
        self.trajectory_up.clear()
        self.waypoint_history.clear()
        self.time_data.clear()
        self.completion_data.clear()

        # Reset waypoint data
        self.current_waypoint_east = None
        self.current_waypoint_north = None
        self.current_waypoint_up = None

        # Reset 3D trajectory data
        self.trajectory_data = np.empty((0, 3))
        self.last_trajectory_length = 0

    def feedback_callback(self, feedback_msg):
        """Handle feedback from action server."""
        self.feedback_updated.emit(feedback_msg.feedback)

    def update_feedback_display(self, feedback):
        """Update GUI with feedback data (runs in main thread)."""
        if not self.goal_active:
            return

        current_time = time.time() - self.start_time if self.start_time else 0

        # Update progress bars
        max_time = self.exploration_time_spin.value()
        time_progress = min(100, (current_time / max_time) * 100)
        self.time_progress.setValue(int(time_progress))

        completion_rate = feedback.current_completion_rate * 100
        self.completion_progress.setValue(int(completion_rate))

        # Update data displays
        self.waypoints_count_label.setText(str(feedback.waypoints_generated))
        self.corners_label.setText(f"{feedback.corners_remaining}")

        # Vehicle status
        armed_status = "Armed" if feedback.vehicle_armed else "Disarmed"
        mode_status = "Offboard" if feedback.offboard_mode_active else "Manual"
        self.vehicle_status_label.setText(f"{armed_status}/{mode_status}")

        # Position and waypoint info (ENU coordinates)
        if hasattr(feedback, "current_position"):
            pos = feedback.current_position
            self.position_label.setText(
                f"[{pos.x:.2f}, {pos.y:.2f}, {pos.z:.2f}]"
            )

            # Add to trajectory data (treating x,y,z as East,North,Up)
            new_point = np.array([[pos.x, pos.y, pos.z]])
            self.trajectory_data = (
                np.vstack([self.trajectory_data, new_point])
                if self.trajectory_data.size > 0
                else new_point
            )

            # Also keep deque for compatibility with 2D plots
            self.trajectory_points.append([pos.x, pos.y, pos.z])
            self.trajectory_east.append(pos.x)  # East
            self.trajectory_north.append(pos.y)  # North
            self.trajectory_up.append(pos.z)  # Up

        if hasattr(feedback, "current_waypoint"):
            wp = feedback.current_waypoint
            self.waypoint_label.setText(f"[{wp.x:.2f}, {wp.y:.2f}, {wp.z:.2f}]")

            # Store current waypoint for visualization (ENU)
            self.current_waypoint_east = wp.x  # East
            self.current_waypoint_north = wp.y  # North
            self.current_waypoint_up = wp.z  # Up

        # Store data for plotting
        self.time_data.append(current_time)
        self.completion_data.append(completion_rate)
        self.waypoint_history.append(feedback.waypoints_generated)

        # Update time ranges for progress plots if needed
        if len(self.time_data) > 1:
            max_time_data = max(self.time_data)
            max_waypoint_data = (
                max(self.waypoint_history) if self.waypoint_history else 1
            )

            # Update time axis ranges
            self.progress_plot.setXRange(0, max_time_data * 1.1)
            self.waypoint_plot.setXRange(0, max_time_data * 1.1)
            self.waypoint_plot.setYRange(0, max_waypoint_data * 1.1)
            self.alt_plot.setXRange(0, max_time_data * 1.1)

        # Update status
        if hasattr(feedback, "status_message") and feedback.status_message:
            self.status_updated.emit(feedback.status_message)
        else:
            self.status_updated.emit(
                f"Mission active - {completion_rate:.1f}% complete"
            )

    def update_plots(self):
        """Update all plots with current ENU data."""
        if not self.time_data:
            return

        # Update progress plots
        if len(self.time_data) > 1:
            time_array = np.array(list(self.time_data))
            completion_array = np.array(list(self.completion_data))
            waypoint_array = np.array(list(self.waypoint_history))

            self.progress_curve.setData(time_array, completion_array)
            self.waypoint_curve.setData(time_array, waypoint_array)

        # Update 2D trajectory plots (ENU projections)
        if len(self.trajectory_east) > 1:
            # East-North trajectory (top view)
            self.en_trajectory.setData(
                list(self.trajectory_east), list(self.trajectory_north)
            )

            # East-Up trajectory (side view from South)
            self.eu_trajectory.setData(
                list(self.trajectory_east), list(self.trajectory_up)
            )

            # North-Up trajectory (side view from West)
            self.nu_trajectory.setData(
                list(self.trajectory_north), list(self.trajectory_up)
            )

            # Current position markers
            if self.trajectory_east:
                current_east = self.trajectory_east[-1]
                current_north = self.trajectory_north[-1]
                current_up = self.trajectory_up[-1]

                self.en_current_pos.setData([current_east], [current_north])
                self.eu_current_pos.setData([current_east], [current_up])
                self.nu_current_pos.setData([current_north], [current_up])

        # Update waypoint markers if we have waypoint data
        if (
            self.current_waypoint_east is not None
            and self.current_waypoint_north is not None
            and self.current_waypoint_up is not None
        ):
            self.en_waypoint.setData(
                [self.current_waypoint_east], [self.current_waypoint_north]
            )
            self.eu_waypoint.setData(
                [self.current_waypoint_east], [self.current_waypoint_up]
            )
            self.nu_waypoint.setData(
                [self.current_waypoint_north], [self.current_waypoint_up]
            )

        # Altitude plot over time
        if len(self.time_data) > 1:
            self.alt_curve.setData(
                list(self.time_data), list(self.trajectory_up)
            )

            # Show target altitude if we have waypoint data
            if self.current_waypoint_up is not None:
                target_altitudes = [self.current_waypoint_up] * len(
                    self.time_data
                )
                self.alt_target.setData(list(self.time_data), target_altitudes)

    def handle_result(self, success, result):
        """Handle final result from action server."""
        self.reset_ui_state()

        if success and result:
            status_msg = (
                f"Mission completed! "
                f"Waypoints: {result.total_waypoints_generated}, "
                f"Completion: {result.completion_rate:.1%}, "
                f"Time: {result.total_exploration_time:.1f}s"
            )
        else:
            status_msg = "Mission failed or was cancelled"

        self.status_updated.emit(status_msg)

    def update_status(self, message):
        """Update status label."""
        self.status_label.setText(message)

    def reset_ui_state(self):
        """Reset UI to ready state."""
        self.goal_active = False
        self.start_button.setEnabled(True)
        self.cancel_button.setEnabled(False)

    def closeEvent(self, event):
        """Handle application close."""
        if self.goal_active:
            self.cancel_exploration()

        self.client_node.destroy_node()
        self.executor.shutdown()
        rclpy.shutdown()
        event.accept()


class PathGeneratorActionClient(Node):
    """Simplified action client for GUI integration."""

    def __init__(self):
        super().__init__("bioinspired_path_generator_gui_client")
        self.callback_group = ReentrantCallbackGroup()

        self._action_client = ActionClient(
            self,
            PathGeneration,
            "/generate_path",
            callback_group=self.callback_group,
        )

        self.current_goal_handle = None

    def wait_for_server(self, timeout_sec=10.0):
        """Wait for action server."""
        return self._action_client.wait_for_server(timeout_sec=timeout_sec)

    def send_goal_and_wait(
        self,
        x_min,
        x_max,
        y_min,
        y_max,
        z_min,
        z_max,
        alpha,
        visit_threshold,
        exploration_time,
        feedback_callback,
    ):
        """Send goal and wait for completion."""
        if not self._action_client.server_is_ready():
            return False, None

        goal_msg = PathGeneration.Goal()
        goal_msg.x_min = float(x_min)
        goal_msg.x_max = float(x_max)
        goal_msg.y_min = float(y_min)
        goal_msg.y_max = float(y_max)
        goal_msg.z_min = float(z_min)
        goal_msg.z_max = float(z_max)
        goal_msg.alpha = float(alpha)
        goal_msg.visit_threshold = float(visit_threshold)
        goal_msg.exploration_time = float(exploration_time)

        send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=feedback_callback
        )

        rclpy.spin_until_future_complete(
            self, send_goal_future, timeout_sec=10.0
        )

        if not send_goal_future.done():
            return False, None

        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            return False, None

        self.current_goal_handle = goal_handle

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result = result_future.result()
        if result:
            return (result.status == 4), result.result  # SUCCEEDED
        return False, None

    def cancel_goal(self):
        """Cancel current goal."""
        if self.current_goal_handle is not None:
            self.current_goal_handle.cancel_goal_async()


def main(args=None):
    """Main function to run the GUI application."""
    global _app

    print("Starting Bioinspired Path Generator GUI Client (ENU)...")

    # Ensure QApplication exists
    if _app is None:
        from pyqtgraph.Qt import QtCore, QtWidgets

        _app = QtWidgets.QApplication(sys.argv)
        _app.setStyle("Fusion")
        print("QApplication created in main()")

    try:
        # Create and show the main window, passing the app reference
        print("Creating main window...")
        window = BioinspiredPathGeneratorGUIClient(app=_app)
        window.show()
        print("GUI window shown, starting event loop...")

        # Run the application
        exit_code = _app.exec_()
        print(f"Application exited with code: {exit_code}")
        return exit_code

    except Exception as e:
        print(f"Error in main: {e}")
        import traceback

        traceback.print_exc()
        return 1


if __name__ == "__main__":
    sys.exit(main())
