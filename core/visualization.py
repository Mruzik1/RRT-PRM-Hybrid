"""Visualization module for path planning algorithms"""
import pygame


class MapVisualizer:
    """Base visualization class for path planning algorithms"""
    
    def __init__(self, start, goal, dimensions, title="Path Planning"):
        self.start = start
        self.goal = goal
        self.height, self.width = dimensions
        
        pygame.display.set_caption(title)
        self.surface = pygame.display.set_mode((self.width, self.height))
        self.surface.fill((255, 255, 255))
        
        self.node_radius = 2
        self.edge_thickness = 1
        
        # Colors
        self.obstacle_color = (70, 70, 70)
        self.tree_color = (0, 0, 255)
        self.start_goal_color = (0, 255, 0)
        self.path_color = (255, 0, 0)

    def draw_initial(self, obstacles):
        """Draw start, goal, and obstacles"""
        pygame.draw.circle(self.surface, self.start_goal_color, self.start, self.node_radius + 5, 0)
        pygame.draw.circle(self.surface, self.start_goal_color, self.goal, self.node_radius + 20, 1)
        self._draw_obstacles(obstacles)

    def draw_path(self, path):
        """Draw the final path"""
        for i in range(len(path) - 1):
            pygame.draw.line(self.surface, self.path_color, path[i], path[i+1], 3)
        for node in path:
            pygame.draw.circle(self.surface, self.path_color, node, 4, 0)

    def draw_edge(self, node1, node2, color=None):
        """Draw an edge between two nodes"""
        color = color or self.tree_color
        pygame.draw.line(self.surface, color, node1, node2, self.edge_thickness)

    def draw_node(self, position, color=None, radius=None):
        """Draw a single node"""
        color = color or self.obstacle_color
        radius = radius or self.node_radius * 2
        pygame.draw.circle(self.surface, color, position, radius, 0)

    def _draw_obstacles(self, obstacles):
        """Draw all obstacles"""
        for obstacle in obstacles:
            pygame.draw.rect(self.surface, self.obstacle_color, obstacle)

    def update_display(self):
        """Update the pygame display"""
        pygame.display.update()

    def draw_text(self, text, position=(10, 10), font_size=25, clear_area=None):
        """Draw text on the surface"""
        font = pygame.font.Font(None, font_size)
        text_surface = font.render(text, True, (0, 0, 0))
        if clear_area:
            self.surface.fill((255, 255, 255), clear_area)
        self.surface.blit(text_surface, position)
