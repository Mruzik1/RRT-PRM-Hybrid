"""Graph data structure for path planning algorithms"""
import math
from abc import ABC, abstractmethod


class Graph(ABC):
    """Abstract base class for graph-based path planning algorithms"""
    
    def __init__(self, start, goal, dimensions):
        self.start = start
        self.goal = goal
        self.height, self.width = dimensions
        self.goal_reached = False
        
        x, y = start
        self.x = [x]
        self.y = [y]
        self.parent = [0]
        
        self.goal_index = None
        self.path = []

    def add_node(self, x, y):
        """Add a new node to the graph"""
        self.x.append(x)
        self.y.append(y)

    def remove_node(self, index):
        """Remove node at specified index"""
        self.x.pop(index)
        self.y.pop(index)

    def add_edge(self, parent_idx, child_idx):
        """Connect child node to parent node"""
        self.parent.append(parent_idx)

    def node_count(self):
        """Return total number of nodes"""
        return len(self.x)

    def distance(self, idx1, idx2):
        """Calculate Euclidean distance between two nodes"""
        return math.sqrt((self.x[idx1] - self.x[idx2])**2 + (self.y[idx1] - self.y[idx2])**2)

    def distance_to_point(self, idx, point):
        """Calculate distance from node to a point"""
        return math.sqrt((self.x[idx] - point[0])**2 + (self.y[idx] - point[1])**2)

    def nearest_node(self, target_idx):
        """Find nearest node to target"""
        dmin = self.distance(0, target_idx)
        nearest = 0
        for i in range(target_idx):
            d = self.distance(i, target_idx)
            if d < dmin:
                dmin = d
                nearest = i
        return nearest

    def get_path_coordinates(self):
        """Convert path indices to coordinate tuples"""
        return [(self.x[idx], self.y[idx]) for idx in self.path]

    def reconstruct_path(self):
        """Reconstruct path from goal to start"""
        if not self.goal_reached:
            return False
        
        self.path = []
        current = self.goal_index
        self.path.append(current)
        
        while current != 0:
            current = self.parent[current]
            self.path.append(current)
        
        return True

    @abstractmethod
    def expand(self):
        """Expand the search tree/graph (algorithm-specific)"""
        pass

    @abstractmethod
    def check_goal(self):
        """Check if goal has been reached (algorithm-specific)"""
        pass
