# rover.py
import numpy as np
from enum import Enum

class RoverStatus(Enum):
    IDLE = "IDLE"
    MOVING = "MOVING"
    REACHED_GOAL = "REACHED_GOAL"
    OUT_OF_ENERGY = "OUT_OF_ENERGY"
    STUCK = "STUCK"

class Rover:
    def __init__(self, initial_energy=100, movement_speed=1.0):
        """Initialize rover with given energy and speed"""
        self.energy = initial_energy
        self.max_energy = initial_energy
        self.position = [0, 0]
        self.movement_speed = movement_speed
        self.status = RoverStatus.IDLE
        self.path = []
        self.current_path_index = 0
        self.total_distance = 0
        self.terrain_visited = []
        
    def reset(self):
        """Reset rover to initial state"""
        self.energy = self.max_energy
        self.position = [0, 0]
        self.status = RoverStatus.IDLE
        self.path = []
        self.current_path_index = 0
        self.total_distance = 0
        self.terrain_visited = []
        
    def set_path(self, path):
        """Set new path for rover to follow"""
        self.path = path
        self.current_path_index = 0
        if path:
            self.status = RoverStatus.IDLE
        
    def move(self, terrain, get_terrain_cost):
        """Move rover along path"""
        if self.status in [RoverStatus.REACHED_GOAL, RoverStatus.OUT_OF_ENERGY, RoverStatus.STUCK]:
            return False
            
        if not self.path or self.current_path_index >= len(self.path):
            return False
            
        # Get next position
        next_pos = self.path[self.current_path_index]
        
        # Check if movement is possible
        if terrain[next_pos] == 1:  # Obstacle
            self.status = RoverStatus.STUCK
            return False
            
        # Calculate energy cost
        terrain_type = terrain[next_pos]
        energy_cost = get_terrain_cost(terrain_type)
        
        # Check if enough energy
        if self.energy < energy_cost:
            self.status = RoverStatus.OUT_OF_ENERGY
            return False
            
        # Record previous position for terrain update
        prev_pos = tuple(self.position)
        
        # Update position and terrain
        terrain[prev_pos] = 0  # Clear previous position
        self.position = list(next_pos)
        terrain[tuple(self.position)] = 2  # Mark new position
        
        # Update rover state
        self.energy -= energy_cost
        self.current_path_index += 1
        self.total_distance += 1
        self.terrain_visited.append(terrain_type)
        self.status = RoverStatus.MOVING
        
        # Update movement speed based on terrain
        self.update_movement_speed(terrain_type)
        
        # Check if reached goal
        if self.current_path_index >= len(self.path):
            self.status = RoverStatus.REACHED_GOAL
            
        return True
        
    def get_status(self):
        """Get current rover status"""
        return {
            'position': tuple(self.position),
            'energy': self.energy,
            'status': self.status.value,
            'progress': (self.current_path_index / len(self.path) * 100) if self.path else 0,
            'distance': self.total_distance,
            'speed': self.movement_speed
        }
        
    def estimate_completion_time(self):
        """Estimate time to complete current path"""
        if not self.path or self.current_path_index >= len(self.path):
            return 0
            
        remaining_steps = len(self.path) - self.current_path_index
        return remaining_steps / self.movement_speed
        
    def can_complete_path(self, terrain, get_terrain_cost):
        """Check if rover has enough energy to complete path"""
        if not self.path or self.current_path_index >= len(self.path):
            return True
            
        remaining_path = self.path[self.current_path_index:]
        estimated_energy = sum(get_terrain_cost(terrain[pos]) for pos in remaining_path)
        
        return self.energy >= estimated_energy
        
    def get_efficiency_metrics(self):
        """Calculate efficiency metrics"""
        if not self.path:
            return None
            
        metrics = {
            'energy_efficiency': (self.max_energy - self.energy) / max(1, self.current_path_index),
            'progress_rate': self.current_path_index / len(self.path) if self.path else 0,
            'average_speed': self.total_distance / max(1, self.current_path_index),
            'terrain_distribution': self.get_terrain_distribution()
        }
        
        return metrics
        
    def get_terrain_distribution(self):
        """Calculate distribution of traversed terrain types"""
        if not self.terrain_visited:
            return {}
            
        unique, counts = np.unique(self.terrain_visited, return_counts=True)
        total = len(self.terrain_visited)
        
        return {terrain: count/total for terrain, count in zip(unique, counts)}
        
    def update_movement_speed(self, terrain_type):
        """Update movement speed based on terrain type"""
        terrain_speed_factors = {
            0: 1.0,  # Clear path - normal speed
            4: 0.7,  # Sand - slower
            5: 0.5   # Rocks - slowest
        }
        self.movement_speed = terrain_speed_factors.get(terrain_type, 1.0)
        
    def emergency_stop(self):
        """Emergency stop procedure"""
        self.status = RoverStatus.IDLE
        return {
            'position': tuple(self.position),
            'energy_remaining': self.energy,
            'path_progress': self.current_path_index / len(self.path) if self.path else 0,
            'final_status': self.status.value
        }
        
    def get_path_stats(self):
        """Get statistics about the current path"""
        if not self.path:
            return None
            
        return {
            'total_length': len(self.path),
            'completed': self.current_path_index,
            'remaining': len(self.path) - self.current_path_index,
            'estimated_time': self.estimate_completion_time()
        }
        
    def is_near_obstacle(self, terrain):
        """Check if rover is near any obstacles"""
        x, y = self.position
        size = len(terrain)
        
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                new_x, new_y = x + dx, y + dy
                if (0 <= new_x < size and 
                    0 <= new_y < size and 
                    terrain[new_x, new_y] == 1):
                    return True
        return False