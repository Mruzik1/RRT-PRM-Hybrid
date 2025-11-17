"""Environment and obstacle handling for path planning"""
import random
import pygame


class Environment:
    
    def __init__(self, dimensions, obstacle_size, obstacle_count):
        self.height, self.width = dimensions
        self.obstacle_size = obstacle_size
        self.obstacle_count = obstacle_count
        self.obstacles = []
        self._collision_cache = {}

    def generate_obstacles(self, start, goal):
        self.obstacles = []
        self._collision_cache.clear()
        
        for _ in range(self.obstacle_count):
            while True:
                x = int(random.uniform(0, self.width - self.obstacle_size))
                y = int(random.uniform(0, self.height - self.obstacle_size))
                rect = pygame.Rect((x, y), (self.obstacle_size, self.obstacle_size))
                
                if not (rect.collidepoint(start) or rect.collidepoint(goal)):
                    self.obstacles.append(rect)
                    break
        return self.obstacles

    def is_collision_free(self, x, y):
        cache_key = (int(x), int(y))
        if cache_key in self._collision_cache:
            return self._collision_cache[cache_key]
        
        result = not any(obs.collidepoint(x, y) for obs in self.obstacles)
        
        if len(self._collision_cache) < 100000:
            self._collision_cache[cache_key] = result
        return result

    def is_path_clear(self, x1, y1, x2, y2, steps=50):
        min_x, max_x = min(x1, x2), max(x1, x2)
        min_y, max_y = min(y1, y2), max(y1, y2)
        
        relevant_obstacles = [obs for obs in self.obstacles 
                             if obs.right >= min_x and obs.left <= max_x 
                             and obs.bottom >= min_y and obs.top <= max_y]
        
        if not relevant_obstacles:
            return True
        
        for i in range(steps + 1):
            t = i / steps
            x, y = x1 + t * (x2 - x1), y1 + t * (y2 - y1)
            if any(obs.collidepoint(x, y) for obs in relevant_obstacles):
                return False
        return True

    def sample_free_space(self):
        return int(random.uniform(0, self.width)), int(random.uniform(0, self.height))
