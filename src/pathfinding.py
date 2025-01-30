# pathfinding.py
import numpy as np
from heapq import heappush, heappop
import time

class PathFinder:
    def __init__(self, terrain_generator):
        self.terrain_generator = terrain_generator
        self.algorithms = {
            'A*': self.a_star,
            'Dijkstra': self.dijkstra,
            'Energy Efficient': self.energy_efficient_astar
        }
        
    def get_neighbors(self, pos, terrain):
        """Get valid neighboring positions with diagonal movement"""
        directions = [
            (0, 1), (1, 0), (0, -1), (-1, 0),  # Cardinal directions
            (1, 1), (-1, -1), (1, -1), (-1, 1)  # Diagonal directions
        ]
        
        neighbors = []
        size = len(terrain)
        
        for dx, dy in directions:
            new_pos = (pos[0] + dx, pos[1] + dy)
            if (0 <= new_pos[0] < size and 
                0 <= new_pos[1] < size and
                terrain[new_pos] != 1):  # Not an obstacle
                
                # Add diagonal penalty
                is_diagonal = dx != 0 and dy != 0
                neighbors.append((new_pos, 1.4 if is_diagonal else 1.0))
                
        return neighbors
        
    def heuristic(self, a, b, method='euclidean'):
        """Calculate heuristic distance"""
        if method == 'manhattan':
            return abs(b[0] - a[0]) + abs(b[1] - a[1])
        else:  # euclidean
            return np.sqrt((b[0] - a[0])**2 + (b[1] - a[1])**2)
            
    def a_star(self, terrain, start, goal):
        """A* pathfinding algorithm with terrain cost consideration"""
        if terrain[goal] == 1:  # Goal is obstacle
            return None, None, 0
            
        frontier = []
        heappush(frontier, (0, start))
        came_from = {start: None}
        cost_so_far = {start: 0}
        
        start_time = time.time()
        
        while frontier:
            current = heappop(frontier)[1]
            
            if current == goal:
                break
                
            for next_pos, move_cost in self.get_neighbors(current, terrain):
                # Calculate total movement cost including terrain
                terrain_cost = self.terrain_generator.get_terrain_cost(terrain[next_pos])
                new_cost = cost_so_far[current] + (move_cost * terrain_cost)
                
                if next_pos not in cost_so_far or new_cost < cost_so_far[next_pos]:
                    cost_so_far[next_pos] = new_cost
                    priority = new_cost + self.heuristic(goal, next_pos)
                    heappush(frontier, (priority, next_pos))
                    came_from[next_pos] = current
                    
        computation_time = time.time() - start_time
        
        if goal not in came_from:
            return None, None, computation_time
            
        return came_from, cost_so_far, computation_time
        
    def energy_efficient_astar(self, terrain, start, goal):
        """Energy-efficient variant of A* that heavily weights terrain costs"""
        def energy_heuristic(pos1, pos2):
            base_distance = self.heuristic(pos1, pos2)
            terrain_cost = self.terrain_generator.get_terrain_cost(terrain[pos2])
            return base_distance * terrain_cost
            
        if terrain[goal] == 1:
            return None, None, 0
            
        frontier = []
        heappush(frontier, (0, start))
        came_from = {start: None}
        cost_so_far = {start: 0}
        
        start_time = time.time()
        
        while frontier:
            current = heappop(frontier)[1]
            
            if current == goal:
                break
                
            for next_pos, move_cost in self.get_neighbors(current, terrain):
                terrain_cost = self.terrain_generator.get_terrain_cost(terrain[next_pos])
                # Penalize high terrain costs more in energy-efficient mode
                energy_cost = move_cost * (terrain_cost ** 2)  
                new_cost = cost_so_far[current] + energy_cost
                
                if next_pos not in cost_so_far or new_cost < cost_so_far[next_pos]:
                    cost_so_far[next_pos] = new_cost
                    priority = new_cost + energy_heuristic(next_pos, goal)
                    heappush(frontier, (priority, next_pos))
                    came_from[next_pos] = current
                    
        computation_time = time.time() - start_time
        
        if goal not in came_from:
            return None, None, computation_time
            
        return came_from, cost_so_far, computation_time
        
    def dijkstra(self, terrain, start, goal):
        """Dijkstra's algorithm - like A* but without heuristic"""
        came_from, cost_so_far, comp_time = self.a_star(terrain, start, goal)
        return came_from, cost_so_far, comp_time
        
    def reconstruct_path(self, came_from, start, goal):
        """Reconstruct path from came_from dictionary"""
        if not came_from or goal not in came_from:
            return []
            
        current = goal
        path = []
        
        while current != start:
            path.append(current)
            current = came_from[current]
        path.append(start)
        
        return path[::-1]  # Reverse to get start-to-goal order
        
    def find_path(self, terrain, start, goal, algorithm='A*'):
        """Find path using specified algorithm"""
        if algorithm not in self.algorithms:
            raise ValueError(f"Unknown algorithm: {algorithm}")
            
        came_from, cost_so_far, computation_time = self.algorithms[algorithm](
            terrain, start, goal
        )
        
        path = self.reconstruct_path(came_from, start, goal)
        total_cost = cost_so_far[goal] if cost_so_far and goal in cost_so_far else float('inf')
        
        return path, total_cost, computation_time