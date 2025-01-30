# telemetry.py
import json
import pandas as pd
from datetime import datetime
import matplotlib.pyplot as plt
import numpy as np

class RoverTelemetry:
    def __init__(self):
        self.mission_data = []
        self.current_mission = {
            'id': None,
            'start_time': None,
            'path': [],
            'energy_usage': [],
            'terrain_types': [],
            'speed': [],
            'timestamps': []
        }
        
    def start_mission(self, mission_id):
        """Initialize new mission"""
        self.current_mission = {
            'id': mission_id,
            'start_time': datetime.now(),
            'path': [],
            'energy_usage': [],
            'terrain_types': [],
            'speed': [],
            'timestamps': []
        }
        
    def record_movement(self, position, energy, terrain_type, speed):
        """Record rover movement data"""
        timestamp = datetime.now()
        
        self.current_mission['path'].append(position)
        self.current_mission['energy_usage'].append(energy)
        self.current_mission['terrain_types'].append(terrain_type)
        self.current_mission['speed'].append(speed)
        self.current_mission['timestamps'].append(timestamp)
        
    def end_mission(self, success):
        """End current mission and save data"""
        if not self.current_mission['path']:
            return None
            
        mission_data = {
            'mission_id': self.current_mission['id'],
            'start_time': self.current_mission['start_time'].strftime('%Y-%m-%d %H:%M:%S'),
            'end_time': datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
            'success': success,
            'total_distance': len(self.current_mission['path']),
            'energy_consumed': max(0, 100 - min(self.current_mission['energy_usage'])),
            'path': self.current_mission['path'],
            'terrain_distribution': self.calculate_terrain_distribution(),
            'average_speed': np.mean(self.current_mission['speed']) if self.current_mission['speed'] else 0
        }
        
        self.mission_data.append(mission_data)
        return mission_data
        
    def calculate_terrain_distribution(self):
        """Calculate distribution of terrain types traversed"""
        terrain_types = self.current_mission['terrain_types']
        if not terrain_types:
            return {}
            
        unique, counts = np.unique(terrain_types, return_counts=True)
        total = len(terrain_types)
        
        return {str(terrain): count/total for terrain, count in zip(unique, counts)}
        
    def generate_mission_report(self):
        """Generate comprehensive mission report"""
        if not self.mission_data:
            return None
            
        total_missions = len(self.mission_data)
        successful_missions = sum(1 for mission in self.mission_data if mission['success'])
        
        report = {
            'total_missions': total_missions,
            'success_rate': (successful_missions / total_missions) * 100 if total_missions > 0 else 0,
            'average_energy_consumption': np.mean([m['energy_consumed'] for m in self.mission_data]),
            'average_distance': np.mean([m['total_distance'] for m in self.mission_data]),
            'mission_history': self.mission_data
        }
        
        return report
        
    def plot_energy_usage(self, figure):
        """Plot energy usage over time"""
        if not self.current_mission['energy_usage']:
            return
            
        ax = figure.add_subplot(111)
        ax.plot(self.current_mission['energy_usage'], label='Energy Usage')
        ax.set_title('Energy Usage Over Time')
        ax.set_xlabel('Steps')
        ax.set_ylabel('Energy Level')
        ax.grid(True)
        ax.legend()
        
    def save_data(self, filename):
        """Save mission data to file"""
        with open(filename, 'w') as f:
            json.dump(self.mission_data, f, indent=4)
            
    def load_data(self, filename):
        """Load mission data from file"""
        with open(filename, 'r') as f:
            self.mission_data = json.load(f)
            
    def get_performance_metrics(self):
        """Calculate performance metrics"""
        if not self.mission_data:
            return {
                'success_rate': 0,
                'avg_energy_per_step': 0,
                'avg_mission_distance': 0,
                'total_missions': 0,
                'longest_mission': 0
            }
            
        df = pd.DataFrame(self.mission_data)
        
        # Avoid division by zero
        energy_per_step = df['energy_consumed'] / df['total_distance'].replace(0, 1)
        
        metrics = {
            'success_rate': df['success'].mean() * 100,
            'avg_energy_per_step': energy_per_step.mean(),
            'avg_mission_distance': df['total_distance'].mean(),
            'total_missions': len(df),
            'longest_mission': df['total_distance'].max()
        }
        
        # Find most efficient mission if possible
        if len(df) > 0:
            efficiency_index = energy_per_step.idxmin()
            if efficiency_index is not None:
                metrics['most_efficient_mission'] = df.iloc[efficiency_index]['mission_id']
                
        return metrics