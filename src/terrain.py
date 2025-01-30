# terrain.py
import numpy as np
from scipy.ndimage import gaussian_filter

class TerrainGenerator:
    """Advanced terrain generator with multiple generation strategies"""
    def __init__(self, size):
        self.size = size
        self.terrain_types = {
            'CLEAR': 0,
            'OBSTACLE': 1,
            'ROVER': 2,
            'GOAL': 3,
            'SAND': 4,
            'ROCKS': 5
        }
        
    def generate_perlin_noise(self, shape, scale=100, octaves=6, persistence=0.5, lacunarity=2.0):
        """Generate Perlin-like noise"""
        def generate_octave(shape, scale):
            rand = np.random.rand(*shape)
            smoothed = gaussian_filter(rand, sigma=scale/10)
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
        
    def generate_central_pathway(self, terrain):
        """Create accessible pathways through the center"""
        center = self.size // 2
        pathway_width = max(2, self.size // 8)
        
        path_start = center - pathway_width // 2
        path_end = center + pathway_width // 2
        
        # Create crossroads pattern
        for i in range(path_start, path_end):
            if 0 <= i < self.size:
                mask = terrain[i, :] == self.terrain_types['OBSTACLE']
                terrain[i, mask] = np.random.choice(
                    [self.terrain_types['CLEAR'], self.terrain_types['SAND']], 
                    size=np.sum(mask),
                    p=[0.7, 0.3]
                )
                
        for j in range(path_start, path_end):
            if 0 <= j < self.size:
                mask = terrain[:, j] == self.terrain_types['OBSTACLE']
                terrain[:, mask] = np.random.choice(
                    [self.terrain_types['CLEAR'], self.terrain_types['SAND']], 
                    size=np.sum(mask),
                    p=[0.7, 0.3]
                )
        
        return terrain
        
    def ensure_accessibility(self, terrain, start_pos, goal_pos):
        """Ensure the start and goal areas are accessible"""
        def clear_area(pos, radius):
            x, y = pos
            x_min, x_max = max(0, x-radius), min(self.size, x+radius+1)
            y_min, y_max = max(0, y-radius), min(self.size, y+radius+1)
            
            terrain[x_min:x_max, y_min:y_max] = self.terrain_types['CLEAR']
            
        # Clear start and goal areas
        clear_area(start_pos, 2)
        clear_area(goal_pos, 2)
        
        # Ensure positions are clear
        terrain[start_pos] = self.terrain_types['CLEAR']
        terrain[goal_pos] = self.terrain_types['CLEAR']
        
        return terrain
        
    def generate_realistic_terrain(self):
        """Generate realistic terrain with various features"""
        # Generate base terrain using Perlin noise
        base_terrain = self.generate_perlin_noise((self.size, self.size))
        
        # Initialize terrain
        terrain = np.zeros((self.size, self.size))
        
        # Define thresholds for terrain types
        thresholds = {
            'clear': 0.4,
            'sand': 0.6,
            'rocks': 0.85,
            'obstacle': 1.0
        }
        
        # Assign terrain types based on noise values
        terrain[base_terrain < thresholds['clear']] = self.terrain_types['CLEAR']
        terrain[(base_terrain >= thresholds['clear']) & 
               (base_terrain < thresholds['sand'])] = self.terrain_types['SAND']
        terrain[(base_terrain >= thresholds['sand']) & 
               (base_terrain < thresholds['rocks'])] = self.terrain_types['ROCKS']
        terrain[base_terrain >= thresholds['rocks']] = self.terrain_types['OBSTACLE']
        
        # Add central pathway
        terrain = self.generate_central_pathway(terrain)
        
        # Ensure accessibility from start to goal
        start_pos = (0, 0)
        goal_pos = (self.size-1, self.size-1)
        terrain = self.ensure_accessibility(terrain, start_pos, goal_pos)
        
        # Add some random clear paths for variety
        random_clear = np.random.rand(self.size, self.size) < 0.15
        terrain[random_clear & (terrain == self.terrain_types['OBSTACLE'])] = \
            self.terrain_types['CLEAR']
        
        return terrain.astype(int)
        
    def get_terrain_cost(self, terrain_type):
        """Get movement cost for each terrain type"""
        costs = {
            self.terrain_types['CLEAR']: 1,
            self.terrain_types['OBSTACLE']: float('inf'),
            self.terrain_types['ROVER']: 1,
            self.terrain_types['GOAL']: 1,
            self.terrain_types['SAND']: 2,
            self.terrain_types['ROCKS']: 3
        }
        return costs.get(terrain_type, float('inf'))
        
    def get_terrain_name(self, terrain_type):
        """Get readable name for terrain type"""
        names = {v: k for k, v in self.terrain_types.items()}
        return names.get(terrain_type, 'UNKNOWN')