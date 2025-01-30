# main.py
import sys
import os
import numpy as np
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                           QHBoxLayout, QPushButton, QLabel, QSpinBox, 
                           QComboBox, QGroupBox, QTabWidget, QFileDialog,
                           QMessageBox, QProgressBar)
from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtGui import QPalette, QColor
import matplotlib.pyplot as plt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg
from matplotlib.figure import Figure
from matplotlib.patches import Patch
from matplotlib.colors import ListedColormap

from terrain import TerrainGenerator
from pathfinding import PathFinder
from rover import Rover, RoverStatus
from telemetry import RoverTelemetry

class MarsRoverGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Advanced Mars Rover Simulator")
        self.setGeometry(100, 100, 1400, 800)
        
        # Initialize components
        self.setup_ui()
        self.initialize_simulation()
        
        # Setup keyboard shortcuts
        self.setup_shortcuts()
        
    def setup_ui(self):
        """Setup the user interface"""
        # Set up dark theme
        self.setup_dark_theme()
        
        # Main widget and layout
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        main_layout = QHBoxLayout(main_widget)
        
        # Left panel with tabs
        left_panel = QTabWidget()
        left_panel.addTab(self.create_control_panel(), "Controls")
        left_panel.addTab(self.create_analytics_panel(), "Analytics")
        main_layout.addWidget(left_panel)
        
        # Right panel for visualization
        visual_layout = QVBoxLayout()
        
        # Main simulation view
        self.figure = Figure(figsize=(8, 8))
        self.canvas = FigureCanvasQTAgg(self.figure)
        visual_layout.addWidget(self.canvas)
        
        # Add telemetry plots
        self.telemetry_figure = Figure(figsize=(8, 3))
        self.telemetry_canvas = FigureCanvasQTAgg(self.telemetry_figure)
        visual_layout.addWidget(self.telemetry_canvas)
        
        right_widget = QWidget()
        right_widget.setLayout(visual_layout)
        main_layout.addWidget(right_widget)
        
    def setup_dark_theme(self):
        """Setup dark theme for the application"""
        self.setStyleSheet("""
            QMainWindow {
                background-color: #2b2b2b;
            }
            QWidget {
                background-color: #2b2b2b;
                color: #ffffff;
            }
            QPushButton {
                background-color: #404040;
                border: 2px solid #505050;
                border-radius: 4px;
                padding: 5px;
                min-width: 80px;
            }
            QPushButton:hover {
                background-color: #505050;
            }
            QProgressBar {
                border: 2px solid #505050;
                border-radius: 4px;
                text-align: center;
            }
            QProgressBar::chunk {
                background-color: #3daee9;
            }
            QGroupBox {
                border: 2px solid #505050;
                border-radius: 4px;
                margin-top: 0.5em;
                padding-top: 0.5em;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 3px;
            }
            QComboBox {
                background-color: #404040;
                border: 2px solid #505050;
                border-radius: 4px;
                padding: 5px;
            }
            QSpinBox {
                background-color: #404040;
                border: 2px solid #505050;
                border-radius: 4px;
                padding: 5px;
            }
        """)
        
    def create_control_panel(self):
        """Create the control panel with terrain information"""
        control_widget = QWidget()
        control_layout = QVBoxLayout(control_widget)
        
        # Terrain Info
        terrain_group = QGroupBox("Terrain Information")
        terrain_layout = QVBoxLayout()
        
        terrain_info = [
            ("Clear Path (Green)", "Energy Cost: 1x"),
            ("Sand (Orange)", "Energy Cost: 2x"),
            ("Rocks (Brown)", "Energy Cost: 3x"),
            ("Obstacle (Red)", "Cannot Pass"),
            ("Rover (Blue)", "Current Position"),
            ("Goal (Yellow)", "Target Position")
        ]
        
        for terrain, cost in terrain_info:
            label = QLabel(f"{terrain} - {cost}")
            terrain_layout.addWidget(label)
            
        terrain_group.setLayout(terrain_layout)
        control_layout.addWidget(terrain_group)
        
        # Simulation Parameters
        param_group = QGroupBox("Simulation Parameters")
        param_layout = QVBoxLayout()
        
        # Grid size
        grid_layout = QHBoxLayout()
        grid_layout.addWidget(QLabel("Grid Size:"))
        self.grid_size_spin = QSpinBox()
        self.grid_size_spin.setRange(10, 50)
        self.grid_size_spin.setValue(20)
        self.grid_size_spin.setToolTip("Set the size of the terrain grid")
        grid_layout.addWidget(self.grid_size_spin)
        param_layout.addLayout(grid_layout)
        
        # Initial energy
        energy_layout = QHBoxLayout()
        energy_layout.addWidget(QLabel("Initial Energy:"))
        self.energy_spin = QSpinBox()
        self.energy_spin.setRange(50, 200)
        self.energy_spin.setValue(100)
        self.energy_spin.setToolTip("Set rover's initial energy level")
        energy_layout.addWidget(self.energy_spin)
        param_layout.addLayout(energy_layout)
        
        # Algorithm selection
        algo_layout = QHBoxLayout()
        algo_layout.addWidget(QLabel("Algorithm:"))
        self.algo_combo = QComboBox()
        self.algo_combo.addItems(["A*", "Dijkstra", "Energy Efficient"])
        self.algo_combo.setToolTip("Select pathfinding algorithm")
        algo_layout.addWidget(self.algo_combo)
        param_layout.addLayout(algo_layout)
        
        param_group.setLayout(param_layout)
        control_layout.addWidget(param_group)
        
        # Controls
        button_group = QGroupBox("Controls")
        button_layout = QVBoxLayout()
        
        self.start_button = QPushButton("Start")
        self.start_button.clicked.connect(self.toggle_simulation)
        self.start_button.setToolTip("Start/Pause simulation (Space)")
        button_layout.addWidget(self.start_button)
        
        self.reset_button = QPushButton("Reset")
        self.reset_button.clicked.connect(self.reset_simulation)
        self.reset_button.setToolTip("Reset simulation (Ctrl+R)")
        button_layout.addWidget(self.reset_button)
        
        self.save_button = QPushButton("Save Data")
        self.save_button.clicked.connect(self.save_simulation_data)
        self.save_button.setToolTip("Save simulation data (Ctrl+S)")
        button_layout.addWidget(self.save_button)
        
        button_group.setLayout(button_layout)
        control_layout.addWidget(button_group)
        
        # Status
        status_group = QGroupBox("Status")
        status_layout = QVBoxLayout()
        
        self.energy_label = QLabel("Energy: 100")
        status_layout.addWidget(self.energy_label)
        
        self.position_label = QLabel("Position: (0, 0)")
        status_layout.addWidget(self.position_label)
        
        self.path_label = QLabel("Path: Not calculated")
        status_layout.addWidget(self.path_label)
        
        self.status_label = QLabel("Status: IDLE")
        status_layout.addWidget(self.status_label)
        
        self.progress_bar = QProgressBar()
        self.progress_bar.setRange(0, 100)
        status_layout.addWidget(self.progress_bar)
        
        status_group.setLayout(status_layout)
        control_layout.addWidget(status_group)
        
        control_layout.addStretch()
        return control_widget
        
    def create_analytics_panel(self):
        """Create the analytics panel"""
        analytics_widget = QWidget()
        layout = QVBoxLayout(analytics_widget)
        
        # Mission Statistics
        stats_group = QGroupBox("Mission Statistics")
        stats_layout = QVBoxLayout()
        
        self.mission_count_label = QLabel("Total Missions: 0")
        stats_layout.addWidget(self.mission_count_label)
        
        self.success_rate_label = QLabel("Success Rate: 0%")
        stats_layout.addWidget(self.success_rate_label)
        
        self.avg_energy_label = QLabel("Avg. Energy Usage: 0")
        stats_layout.addWidget(self.avg_energy_label)
        
        stats_group.setLayout(stats_layout)
        layout.addWidget(stats_group)
        
        # Performance Metrics
        metrics_group = QGroupBox("Current Mission")
        metrics_layout = QVBoxLayout()
        
        self.energy_efficiency_label = QLabel("Energy Efficiency: 0")
        metrics_layout.addWidget(self.energy_efficiency_label)
        
        self.terrain_distribution_label = QLabel("Terrain Distribution: -")
        metrics_layout.addWidget(self.terrain_distribution_label)
        
        metrics_group.setLayout(metrics_layout)
        layout.addWidget(metrics_group)
        
        layout.addStretch()
        return analytics_widget

    def initialize_simulation(self):
        """Initialize simulation components"""
        self.grid_size = self.grid_size_spin.value()
        self.terrain_generator = TerrainGenerator(self.grid_size)
        self.terrain = self.terrain_generator.generate_realistic_terrain()
        
        self.rover = Rover(initial_energy=self.energy_spin.value())
        self.pathfinder = PathFinder(self.terrain_generator)
        self.telemetry = RoverTelemetry()
        
        self.is_running = False
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_simulation)
        
        self.mission_id = 0
        self.current_path_index = 0
        
        # Calculate initial path
        self.calculate_path()
        
        # Update displays
        self.update_plot()
        self.update_telemetry_plots()
        
    def calculate_path(self):
        """Calculate path using selected algorithm"""
        start = tuple(self.rover.position)
        goal = (self.grid_size-1, self.grid_size-1)
        
        path, cost, comp_time = self.pathfinder.find_path(
            self.terrain,
            start,
            goal,
            self.algo_combo.currentText()
        )
        
        if path:
            self.rover.set_path(path)
            self.path_label.setText(f"Path: Found (Cost: {cost:.1f})")
        else:
            self.path_label.setText("Path: No path found!")
            
    def update_simulation(self):
        """Update simulation state"""
        if not self.is_running:
            return
            
        moved = self.rover.move(self.terrain, self.terrain_generator.get_terrain_cost)
        
        if moved:
            self.current_path_index = self.rover.current_path_index
            self.telemetry.record_movement(
                self.rover.position,
                self.rover.energy,
                self.terrain[tuple(self.rover.position)],
                self.rover.movement_speed
            )
            
            self.update_status_display()
            self.update_plot()
            self.update_telemetry_plots()
            
        if self.rover.status in [RoverStatus.REACHED_GOAL, RoverStatus.OUT_OF_ENERGY, RoverStatus.STUCK]:
            self.end_mission()
            
    def update_plot(self):
        """Update main plot with improved visualization"""
        self.figure.clear()
        ax = self.figure.add_subplot(111)
        
        # Custom colormap untuk terrain
        colors = ['#95FB51',    # Clear path - Hijau muda
                 '#FF0000',     # Obstacle - Merah
                 '#0000FF',     # Rover - Biru
                 '#FFD700',     # Goal - Kuning
                 '#FFA500',     # Sand - Orange
                 '#8B4513']     # Rocks - Coklat
        
        terrain_cmap = ListedColormap(colors)
        
        # Plot terrain
        im = ax.imshow(self.terrain, cmap=terrain_cmap, vmin=0, vmax=5)
        
        # Tambahkan legend untuk terrain types
        legend_elements = [
            Patch(facecolor=colors[0], label='Clear Path'),
            Patch(facecolor=colors[1], label='Obstacle'),
            Patch(facecolor=colors[2], label='Rover'),
            Patch(facecolor=colors[3], label='Goal'),
            Patch(facecolor=colors[4], label='Sand (2x Energy)'),
            Patch(facecolor=colors[5], label='Rocks (3x Energy)')
        ]
        ax.legend(handles=legend_elements, loc='center left', bbox_to_anchor=(1, 0.5))
        
        # Plot path jika ada
        if self.rover.path:
            path = np.array(self.rover.path)
            ax.plot(path[:, 1], path[:, 0], 'w--', linewidth=2, label='Planned Path')
            
            # Highlight current position
            if self.current_path_index < len(path):
                current_pos = path[self.current_path_index]
                ax.plot(current_pos[1], current_pos[0], 'w*', markersize=15, label='Current Position')
        
        # Tambahkan grid
        ax.grid(True, color='white', alpha=0.3)
        
        # Set title dengan informasi
        energy_cost = {
            'Clear Path': '1x',
            'Sand': '2x',
            'Rocks': '3x',
            'Obstacle': 'Blocked'
        }
        title = 'Mars Rover Simulation\nTerrain Energy Cost:\n' + \
                '\n'.join([f'{k}: {v}' for k, v in energy_cost.items()])
        ax.set_title(title)
        
        # Adjust layout untuk legend
        self.figure.tight_layout()
        self.canvas.draw()
        
    def update_telemetry_plots(self):
        """Update telemetry plots"""
        self.telemetry_figure.clear()
        self.telemetry.plot_energy_usage(self.telemetry_figure)
        self.telemetry_canvas.draw()
        
    def toggle_simulation(self):
        """Toggle simulation running state"""
        self.is_running = not self.is_running
        
        if self.is_running:
            if not self.rover.path:
                QMessageBox.warning(self, "Warning", "No valid path found!")
                self.is_running = False
                return
                
            self.start_button.setText("Pause")
            self.timer.start(500)  # Update every 500ms
            
            if self.rover.status == RoverStatus.IDLE:
                self.telemetry.start_mission(self.mission_id)
        else:
            self.start_button.setText("Start")
            self.timer.stop()
            
    def reset_simulation(self):
        """Reset simulation to initial state"""
        self.is_running = False
        self.timer.stop()
        self.start_button.setText("Start")
        
        # Re-initialize parameters
        self.grid_size = self.grid_size_spin.value()
        self.terrain = self.terrain_generator.generate_realistic_terrain()
        
        # Reset rover
        self.rover.reset()
        
        # Calculate new path
        self.calculate_path()
        
        # Reset telemetry
        self.telemetry = RoverTelemetry()
        self.current_path_index = 0
        
        # Reset displays
        self.energy_label.setText(f"Energy: {self.rover.energy}")
        self.position_label.setText("Position: (0, 0)")
        self.path_label.setText("Path: Not calculated")
        self.status_label.setText("Status: IDLE")
        self.progress_bar.setValue(0)
        
        # Update visuals
        self.update_plot()
        self.update_telemetry_plots()
        self.update_analytics_display()
        
    def end_mission(self):
        """Handle mission end"""
        self.is_running = False
        self.timer.stop()
        self.start_button.setText("Start")
        
        # Record mission data
        success = self.rover.status == RoverStatus.REACHED_GOAL
        mission_data = self.telemetry.end_mission(success)
        
        # Update analytics
        self.update_analytics_display()
        
        # Show mission result message
        result = "Success!" if success else "Failed!"
        message = f"""Mission {self.mission_id} {result}
Energy Remaining: {self.rover.energy:.1f}
Path Length: {len(self.rover.path)}
Status: {self.rover.status.value}"""
        
        QMessageBox.information(self, "Mission Complete", message)
        
        self.mission_id += 1
        
    def save_simulation_data(self):
        """Save simulation data to file"""
        filename, _ = QFileDialog.getSaveFileName(
            self,
            "Save Simulation Data",
            "",
            "JSON files (*.json)"
        )
        
        if filename:
            try:
                self.telemetry.save_data(filename)
                QMessageBox.information(self, "Success", "Data saved successfully!")
            except Exception as e:
                QMessageBox.critical(self, "Error", f"Failed to save data: {str(e)}")
                
    def update_analytics_display(self):
        """Update analytics display with current metrics"""
        # Get performance metrics
        metrics = self.telemetry.get_performance_metrics()
        
        if not metrics:
            return
            
        # Update mission statistics
        self.mission_count_label.setText(f"Total Missions: {metrics['total_missions']}")
        self.success_rate_label.setText(f"Success Rate: {metrics['success_rate']:.1f}%")
        self.avg_energy_label.setText(f"Avg. Energy Usage: {metrics['avg_energy_per_step']:.2f}")
        
        # Update current mission metrics
        if self.rover.path:
            rover_metrics = self.rover.get_efficiency_metrics()
            if rover_metrics:
                self.energy_efficiency_label.setText(
                    f"Energy Efficiency: {rover_metrics['energy_efficiency']:.2f}"
                )
                
                # Format terrain distribution
                terrain_dist = rover_metrics['terrain_distribution']
                terrain_text = ", ".join(
                    f"{k}: {v:.1%}" for k, v in terrain_dist.items()
                )
                self.terrain_distribution_label.setText(
                    f"Terrain Distribution: {terrain_text}"
                )
                
    def update_status_display(self):
        """Update status displays"""
        status = self.rover.get_status()
        self.energy_label.setText(f"Energy: {status['energy']:.1f}")
        self.position_label.setText(f"Position: {status['position']}")
        self.status_label.setText(f"Status: {status['status']}")
        self.progress_bar.setValue(int(status['progress']))
        
    def setup_shortcuts(self):
        """Setup keyboard shortcuts"""
        self.start_button.setShortcut(" ")  # Spacebar
        self.reset_button.setShortcut("Ctrl+R")
        self.save_button.setShortcut("Ctrl+S")

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = MarsRoverGUI()
    window.show()
    sys.exit(app.exec_())