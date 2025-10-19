"""Environment and obstacle handling for path planning"""
import random
import pygame


class Environment:
    """Handles environment setup including obstacles"""
    
    def __init__(self, dimensions, obstacle_size, obstacle_count):
        self.height, self.width = dimensions
        self.obstacle_size = obstacle_size
        self.obstacle_count = obstacle_count
        self.obstacles = []

    def generate_obstacles(self, start, goal):
        """Generate random obstacles avoiding start and goal positions"""
        self.obstacles = []
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
        """Check if point is free from obstacles"""
        for obstacle in self.obstacles:
            if obstacle.collidepoint(x, y):
                return False
        return True

    def is_path_clear(self, x1, y1, x2, y2, steps=100):
        """Check if path between two points is obstacle-free"""
        for obstacle in self.obstacles:
            for i in range(steps + 1):
                t = i / steps
                x = x1 + t * (x2 - x1)
                y = y1 + t * (y2 - y1)
                if obstacle.collidepoint(x, y):
                    return False
        return True

    def sample_free_space(self):
        """Sample a random point in the environment"""
        x = int(random.uniform(0, self.width))
        y = int(random.uniform(0, self.height))
        return x, y
