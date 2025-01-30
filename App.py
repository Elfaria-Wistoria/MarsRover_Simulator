import sys
import time
import logging
from datetime import datetime
import numpy as np
import pandas as pd
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                           QHBoxLayout, QPushButton, QLabel, QSpinBox, 
                           QComboBox, QGroupBox, QTabWidget, QTextEdit,
                           QProgressBar, QFileDialog, QStyle, QStyleFactory)
from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtGui import QPalette, QColor
import matplotlib.pyplot as plt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg
from matplotlib.figure import Figure
from heapq import heappush, heappop
import json

# Initialize logging
logging.basicConfig(
    filename='rover_simulation.log',
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)

class TerrainGenerator:
    def __init__(self, size):
        self.size = size
        
    def generate_perlin_noise(self, shape, scale=100, octaves=6, persistence=0.5, lacunarity=2.0):
        def generate_octave(shape, scale):
            rand = np.random.rand(*shape)
            smoothed = np.zeros(shape)
            for i in range(1, shape[0]-1):
                for j in range(1, shape[1]-1):
                    smoothed[i,j] = (rand[i-1:i+2, j-1:j+2].mean() + rand[i,j]) / 2
            return smoothed
            
        noise = np.zeros(shape)
        amplitude = 1.0
        frequency = 1.0
        max_value = 0
        
        for _ in range(octaves):
            noise += amplitude * generate_octave(shape, scale/frequency)
            max_value += amplitude
            amplitude *= persistence
            frequency *= lacunarity
            
        noise = noise/max_value
        noise = (noise - noise.min()) / (noise.max() - noise.min())
        
        return noise
        
    def generate_realistic_terrain(self):
        # Generate base terrain 
        base_terrain = self.generate_perlin_noise((self.size, self.size))
        
        # Initialize terrain
        terrain = np.zeros((self.size, self.size))
        
        # Adjusted thresholds untuk mengurangi obstacles
        thresholds = {
            'clear': 0.4,    # Clear path below 0.4
            'sand': 0.6,     # Sand between 0.4 and 0.6
            'rocks': 0.85,   # Rocks between 0.6 and 0.85
            'obstacle': 1.0   # Obstacles above 0.85
        }
        
        # Assign terrain types based on new thresholds
        terrain[base_terrain < thresholds['clear']] = 0  # Clear path
        terrain[(base_terrain >= thresholds['clear']) & (base_terrain < thresholds['sand'])] = 4  # Sand
        terrain[(base_terrain >= thresholds['sand']) & (base_terrain < thresholds['rocks'])] = 5  # Rocks
        terrain[base_terrain >= thresholds['rocks']] = 1  # Obstacles
        
        # Create a central pathway
        center = self.size // 2
        pathway_width = max(2, self.size // 8)  # Ensure minimum width of 2
        
        # Calculate pathway boundaries
        path_start = center - pathway_width // 2
        path_end = center + pathway_width // 2
        
        # Create vertical pathway
        for i in range(path_start, path_end):
            if i >= 0 and i < self.size:
                terrain[i, :] = np.random.choice([0, 4], size=self.size, p=[0.7, 0.3])
        
        # Create horizontal pathway
        for j in range(path_start, path_end):
            if j >= 0 and j < self.size:
                terrain[:, j] = np.random.choice([0, 4], size=self.size, p=[0.7, 0.3])
        
        # Ensure start and goal positions and their surroundings are clear
        start_area = 2
        goal_area = 2
        
        # Clear start area
        terrain[:start_area, :start_area] = 0
        
        # Clear goal area
        terrain[-goal_area:, -goal_area:] = 0
        
        # Add some random clear paths
        random_clear = np.random.rand(self.size, self.size) < 0.15  # Reduced probability
        terrain[random_clear & (terrain == 1)] = 0
        
        # Ensure critical positions are clear
        terrain[0, 0] = 0  # Start
        terrain[-1, -1] = 0  # Goal
        
        return terrain

class PathPlanner:
    def __init__(self, terrain, terrain_costs):
        self.terrain = terrain
        self.terrain_costs = terrain_costs
        self.grid_size = len(terrain)
        
    def a_star(self, start, goal):
        if self.terrain[goal] == 1:
            return None, None, None
            
        frontier = []
        heappush(frontier, (0, start))
        came_from = {start: None}
        cost_so_far = {start: 0}
        
        start_time = time.time()
        
        while frontier:
            current = heappop(frontier)[1]
            
            if current == goal:
                break
                
            for next_pos in self.get_neighbors(current):
                if self.terrain[next_pos] == 1:
                    continue
                    
                new_cost = cost_so_far[current] + self.terrain_costs[self.terrain[next_pos]]
                if next_pos not in cost_so_far or new_cost < cost_so_far[next_pos]:
                    cost_so_far[next_pos] = new_cost
                    priority = new_cost + self.heuristic(goal, next_pos)
                    heappush(frontier, (priority, next_pos))
                    came_from[next_pos] = current
                    
        computation_time = time.time() - start_time
        
        if goal not in came_from:
            return None, None, computation_time
            
        return came_from, cost_so_far, computation_time
        
    def get_neighbors(self, pos):
        neighbors = []
        for dx, dy in [(0, 1), (1, 0), (0, -1), (-1, 0), (1, 1), (-1, -1), (1, -1), (-1, 1)]:
            new_pos = (pos[0] + dx, pos[1] + dy)
            if (0 <= new_pos[0] < self.grid_size and 
                0 <= new_pos[1] < self.grid_size):
                neighbors.append(new_pos)
        return neighbors
        
    def heuristic(self, a, b):
        return np.sqrt((b[0] - a[0])**2 + (b[1] - a[1])**2)  # Euclidean distance

class RoverAnalytics:
    def __init__(self):
        self.mission_data = []
        
    def collect_mission_data(self, mission_id, path_taken, energy_used, time_taken, success):
        self.mission_data.append({
            'mission_id': mission_id,
            'timestamp': datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
            'path_length': len(path_taken),
            'energy_efficiency': energy_used / len(path_taken) if path_taken else 0,
            'time_taken': time_taken,
            'success': success
        })
        
    def generate_mission_report(self):
        if not self.mission_data:
            return None
            
        df = pd.DataFrame(self.mission_data)
        report = {
            'success_rate': df['success'].mean() * 100,
            'avg_energy_efficiency': df['energy_efficiency'].mean(),
            'avg_path_length': df['path_length'].mean(),
            'total_missions': len(df),
            'total_successful': df['success'].sum()
        }
        
        return report
        
    def save_data(self, filename):
        with open(filename, 'w') as f:
            json.dump(self.mission_data, f)
            
    def load_data(self, filename):
        with open(filename, 'r') as f:
            self.mission_data = json.load(f)

class RoverSimGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Enhanced Mars Rover Simulator")
        self.setGeometry(100, 100, 1400, 800)
        
        # Initialize components
        self.setup_ui()
        self.initialize_simulation()
        
        # Analytics
        self.analytics = RoverAnalytics()
        self.mission_id = 0
        self.mission_start_time = None
        
    def setup_ui(self):
        # Set up dark theme
        self.setup_dark_theme()
        
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        main_layout = QHBoxLayout(main_widget)
        
        # Left panel with tabs
        left_panel = QTabWidget()
        left_panel.addTab(self.create_control_panel(), "Controls")
        left_panel.addTab(self.create_analytics_panel(), "Analytics")
        main_layout.addWidget(left_panel)
        
        # Right panel with visualization
        self.figure = Figure(figsize=(8, 8))
        self.canvas = FigureCanvasQTAgg(self.figure)
        main_layout.addWidget(self.canvas)
        
    def setup_dark_theme(self):
        self.setStyle(QStyleFactory.create("Fusion"))
        palette = QPalette()
        palette.setColor(QPalette.Window, QColor(53, 53, 53))
        palette.setColor(QPalette.WindowText, Qt.white)
        palette.setColor(QPalette.Base, QColor(25, 25, 25))
        palette.setColor(QPalette.AlternateBase, QColor(53, 53, 53))
        palette.setColor(QPalette.ToolTipBase, Qt.white)
        palette.setColor(QPalette.ToolTipText, Qt.white)
        palette.setColor(QPalette.Text, Qt.white)
        palette.setColor(QPalette.Button, QColor(53, 53, 53))
        palette.setColor(QPalette.ButtonText, Qt.white)
        palette.setColor(QPalette.BrightText, Qt.red)
        palette.setColor(QPalette.Link, QColor(42, 130, 218))
        palette.setColor(QPalette.Highlight, QColor(42, 130, 218))
        palette.setColor(QPalette.HighlightedText, Qt.black)
        self.setPalette(palette)

    def create_control_panel(self):
        control_widget = QWidget()
        control_layout = QVBoxLayout(control_widget)
        
        # Simulation Parameters
        param_group = QGroupBox("Simulation Parameters")
        param_layout = QVBoxLayout()
        
        # Grid size
        grid_layout = QHBoxLayout()
        grid_layout.addWidget(QLabel("Grid Size:"))
        self.grid_size_spin = QSpinBox()
        self.grid_size_spin.setRange(10, 50)
        self.grid_size_spin.setValue(20)
        grid_layout.addWidget(self.grid_size_spin)
        param_layout.addLayout(grid_layout)
        
        # Initial energy
        energy_layout = QHBoxLayout()
        energy_layout.addWidget(QLabel("Initial Energy:"))
        self.energy_spin = QSpinBox()
        self.energy_spin.setRange(50, 200)
        self.energy_spin.setValue(100)
        energy_layout.addWidget(self.energy_spin)
        param_layout.addLayout(energy_layout)
        
        # Algorithm selection
        algo_layout = QHBoxLayout()
        algo_layout.addWidget(QLabel("Algorithm:"))
        self.algo_combo = QComboBox()
        self.algo_combo.addItems(["A*", "Dijkstra", "RRT"])
        algo_layout.addWidget(self.algo_combo)
        param_layout.addLayout(algo_layout)
        
        param_group.setLayout(param_layout)
        control_layout.addWidget(param_group)
        
        # Controls
        button_group = QGroupBox("Controls")
        button_layout = QVBoxLayout()
        
        self.start_button = QPushButton("Start")
        self.start_button.clicked.connect(self.toggle_simulation)
        button_layout.addWidget(self.start_button)
        
        self.reset_button = QPushButton("Reset")
        self.reset_button.clicked.connect(self.reset_simulation)
        button_layout.addWidget(self.reset_button)
        
        self.save_button = QPushButton("Save Mission Data")
        self.save_button.clicked.connect(self.save_mission_data)
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
        
        self.path_label = QLabel("Path Found: -")
        status_layout.addWidget(self.path_label)
        
        self.progress_bar = QProgressBar()
        self.progress_bar.setRange(0, 100)
        status_layout.addWidget(self.progress_bar)
        
        status_group.setLayout(status_layout)
        control_layout.addWidget(status_group)
        
        control_layout.addStretch()
        return control_widget
        
    def create_analytics_panel(self):
        analytics_widget = QWidget()
        analytics_layout = QVBoxLayout(analytics_widget)
        
        self.analytics_text = QTextEdit()
        self.analytics_text.setReadOnly(True)
        analytics_layout.addWidget(self.analytics_text)
        
        return analytics_widget
        
    def initialize_simulation(self):
        self.grid_size = self.grid_size_spin.value()
        self.energy = self.energy_spin.value()
        self.terrain_costs = {
            0: 1,  # Clear path
            1: float('inf'),  # Obstacle
            2: 1,  # Rover
            3: 1,  # Goal
            4: 2,  # Sand
            5: 3   # Rocks
        }
        
        # Generate terrain
        terrain_gen = TerrainGenerator(self.grid_size)
        self.terrain = terrain_gen.generate_realistic_terrain()
        
        # Set rover and goal positions
        self.rover_pos = [0, 0]
        self.goal_pos = [self.grid_size-1, self.grid_size-1]
        self.terrain[tuple(self.rover_pos)] = 2
        self.terrain[tuple(self.goal_pos)] = 3
        
        # Initialize path planning
        self.path_planner = PathPlanner(self.terrain, self.terrain_costs)
        self.calculate_path()
        
        # Setup simulation variables
        self.is_running = False
        self.current_path_index = 0
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_simulation)
        
        self.update_plot()
        
    def calculate_path(self):
        came_from, cost_so_far, comp_time = self.path_planner.a_star(
            tuple(self.rover_pos), 
            tuple(self.goal_pos)
        )
        
        logging.info(f"Path calculation took {comp_time:.3f} seconds")
        
        if came_from:
            current = tuple(self.goal_pos)
            self.path = []
            while current != tuple(self.rover_pos):
                self.path.append(current)
                current = came_from[current]
            self.path.append(tuple(self.rover_pos))
            self.path.reverse()
            self.current_path_index = 0
            self.path_label.setText("Path Found: Yes")
            
            total_cost = cost_so_far[tuple(self.goal_pos)]
            logging.info(f"Path found with cost: {total_cost}")
        else:
            self.path = []
            self.path_label.setText("Path Found: No path possible!")
            logging.warning("No path found to goal")
            
    def update_plot(self):
        self.figure.clear()
        ax = self.figure.add_subplot(111)
        
        # Create custom colormap
        cmap = plt.cm.viridis.copy()
        cmap.set_bad('red')
        
        # Plot terrain
        im = ax.imshow(self.terrain, cmap=cmap)
        ax.grid(True)
        ax.set_title("Mars Rover Simulation")
        
        # Plot path if exists
        if hasattr(self, 'path') and self.path:
            path_x = [pos[0] for pos in self.path[:self.current_path_index + 1]]
            path_y = [pos[1] for pos in self.path[:self.current_path_index + 1]]
            ax.plot(path_y, path_x, 'r-', linewidth=2, label='Path taken')
            
            # Plot remaining path
            if self.current_path_index + 1 < len(self.path):
                future_x = [pos[0] for pos in self.path[self.current_path_index:]]
                future_y = [pos[1] for pos in self.path[self.current_path_index:]]
                ax.plot(future_y, future_x, 'r--', linewidth=2, label='Planned path')
        
        # Add colorbar
        cbar = self.figure.colorbar(im)
        cbar.set_label('Terrain Type (0:Clear, 1:Obstacle, 2:Rover, 3:Goal, 4:Sand, 5:Rocks)')
        
        # Add legend if path exists
        if hasattr(self, 'path') and self.path:
            ax.legend()
        
        self.canvas.draw()
        
    def toggle_simulation(self):
        if not hasattr(self, 'path') or not self.path:
            logging.warning("No valid path found!")
            return
            
        self.is_running = not self.is_running
        if self.is_running:
            self.start_button.setText("Pause")
            self.timer.start(500)
            if self.mission_start_time is None:
                self.mission_start_time = time.time()
        else:
            self.start_button.setText("Start")
            self.timer.stop()
    
    def update_simulation(self):
        if self.energy > 0 and self.current_path_index < len(self.path):
            # Clear current position
            self.terrain[tuple(self.rover_pos)] = 0
            
            # Get new position
            new_pos = self.path[self.current_path_index]
            
            # Check for obstacles
            if self.terrain[new_pos] == 1:
                logging.warning("Path blocked by obstacle!")
                self.toggle_simulation()
                return
            
            # Update rover position
            self.rover_pos = list(new_pos)
            self.terrain[tuple(self.rover_pos)] = 2
            
            # Calculate energy cost
            terrain_type = self.terrain[new_pos]
            energy_cost = self.terrain_costs.get(terrain_type, 1)
            self.energy -= energy_cost
            
            # Update progress
            self.current_path_index += 1
            progress = (self.current_path_index / len(self.path)) * 100
            self.progress_bar.setValue(int(progress))
            
            # Update labels
            self.energy_label.setText(f"Energy: {self.energy}")
            self.position_label.setText(f"Position: ({self.rover_pos[0]}, {self.rover_pos[1]})")
            
            # Update plot
            self.update_plot()
            
            # Check if goal reached
            if tuple(self.rover_pos) == tuple(self.goal_pos):
                self.mission_complete(True)
        else:
            if self.energy <= 0:
                logging.warning("Mission failed: Out of energy")
                self.mission_complete(False)
            self.toggle_simulation()
    
    def mission_complete(self, success):
        self.toggle_simulation()
        
        if success:
            logging.info("Goal reached successfully!")
        else:
            logging.warning("Mission failed!")
            
        # Calculate mission statistics
        mission_time = time.time() - self.mission_start_time if self.mission_start_time else 0
        
        # Record mission data
        self.analytics.collect_mission_data(
            self.mission_id,
            self.path[:self.current_path_index],
            100 - self.energy,  # energy used
            mission_time,
            success
        )
        
        # Update analytics display
        self.update_analytics_display()
        
        self.mission_id += 1
        self.mission_start_time = None
    
    def update_analytics_display(self):
        report = self.analytics.generate_mission_report()
        if report:
            report_text = f"""Mission Statistics:
            
Total Missions: {report['total_missions']}
Successful Missions: {report['total_successful']}
Success Rate: {report['success_rate']:.2f}%
Average Path Length: {report['avg_path_length']:.2f}
Average Energy Efficiency: {report['avg_energy_efficiency']:.2f}
"""
            self.analytics_text.setText(report_text)
    
    def reset_simulation(self):
        self.is_running = False
        self.start_button.setText("Start")
        self.timer.stop()
        
        # Update parameters
        self.initialize_simulation()
        
        # Reset progress
        self.progress_bar.setValue(0)
        
        # Reset labels
        self.energy_label.setText(f"Energy: {self.energy}")
        self.position_label.setText(f"Position: (0, 0)")
        self.path_label.setText("Path Found: -")
        
    def save_mission_data(self):
        filename, _ = QFileDialog.getSaveFileName(
            self,
            "Save Mission Data",
            "mission_data.json",
            "JSON files (*.json)"
        )
        
        if filename:
            self.analytics.save_data(filename)
            logging.info(f"Mission data saved to {filename}")

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = RoverSimGUI()
    window.show()
    sys.exit(app.exec_())