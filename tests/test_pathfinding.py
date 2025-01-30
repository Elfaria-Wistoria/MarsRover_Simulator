# tests/test_pathfinding.py
import unittest
import numpy as np
import sys
import os

# Add the src directory to Python path
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src.terrain import TerrainGenerator
from src.pathfinding import PathFinder

class TestPathFinding(unittest.TestCase):
    def setUp(self):
        """Setup test environment"""
        self.grid_size = 10
        self.terrain_generator = TerrainGenerator(self.grid_size)
        self.pathfinder = PathFinder(self.terrain_generator)
        
    def test_path_exists_clear_terrain(self):
        """Test path finding on clear terrain"""
        # Create clear terrain
        terrain = np.zeros((self.grid_size, self.grid_size))
        start = (0, 0)
        goal = (9, 9)
        
        path, cost, _ = self.pathfinder.find_path(terrain, start, goal, "A*")
        
        self.assertIsNotNone(path, "Path should exist in clear terrain")
        self.assertTrue(len(path) > 0, "Path should contain points")
        self.assertEqual(path[0], start, "Path should start at start position")
        self.assertEqual(path[-1], goal, "Path should end at goal position")
        
    def test_no_path_with_wall(self):
        """Test when no path exists due to wall"""
        terrain = np.zeros((self.grid_size, self.grid_size))
        # Create wall
        terrain[4,:] = 1
        start = (0, 0)
        goal = (9, 9)
        
        path, cost, _ = self.pathfinder.find_path(terrain, start, goal, "A*")
        
        self.assertIsNone(path, "No path should exist with blocking wall")
        
    def test_energy_efficient_path(self):
        """Test energy efficient pathfinding"""
        terrain = np.zeros((self.grid_size, self.grid_size))
        # Add high cost terrain
        terrain[3:7, 3:7] = 5  # Rocks
        start = (0, 0)
        goal = (9, 9)
        
        # Compare normal A* with energy efficient
        _, normal_cost, _ = self.pathfinder.find_path(terrain, start, goal, "A*")
        _, efficient_cost, _ = self.pathfinder.find_path(terrain, start, goal, "Energy Efficient")
        
        self.assertLess(efficient_cost, normal_cost, 
                       "Energy efficient path should have lower cost")
        
    def test_diagonal_movement(self):
        """Test diagonal movement"""
        terrain = np.zeros((self.grid_size, self.grid_size))
        start = (0, 0)
        goal = (1, 1)
        
        path, cost, _ = self.pathfinder.find_path(terrain, start, goal, "A*")
        
        self.assertEqual(len(path), 2, "Diagonal path should be direct")
        self.assertAlmostEqual(cost, 1.4, 1, msg="Diagonal movement should cost √2")
        
    def test_obstacle_avoidance(self):
        """Test obstacle avoidance"""
        terrain = np.zeros((self.grid_size, self.grid_size))
        # Create obstacle
        terrain[1, 1] = 1
        start = (0, 0)
        goal = (2, 2)
        
        path, _, _ = self.pathfinder.find_path(terrain, start, goal, "A*")
        
        for point in path:
            self.assertNotEqual(terrain[point], 1, 
                              "Path should not contain obstacles")
                              
    def test_terrain_cost_consideration(self):
        """Test if pathfinder considers terrain costs"""
        terrain = np.zeros((self.grid_size, self.grid_size))
        # Create high cost area
        terrain[2:4, 2:4] = 5  # Rocks
        start = (0, 0)
        goal = (5, 5)
        
        path, _, _ = self.pathfinder.find_path(terrain, start, goal, "Energy Efficient")
        
        # Check if path avoids rocks when possible
        rocks_crossed = sum(1 for point in path if terrain[point] == 5)
        self.assertLess(rocks_crossed, 4, 
                       "Path should minimize crossing high-cost terrain")
                       
    def test_invalid_inputs(self):
        """Test handling of invalid inputs"""
        terrain = np.zeros((self.grid_size, self.grid_size))
        
        # Test invalid start position
        with self.assertRaises(IndexError):
            self.pathfinder.find_path(terrain, (-1, 0), (5, 5), "A*")
            
        # Test invalid goal position
        with self.assertRaises(IndexError):
            self.pathfinder.find_path(terrain, (0, 0), (100, 100), "A*")
            
        # Test invalid algorithm name
        with self.assertRaises(ValueError):
            self.pathfinder.find_path(terrain, (0, 0), (5, 5), "Invalid")

if __name__ == '__main__':
    unittest.main()