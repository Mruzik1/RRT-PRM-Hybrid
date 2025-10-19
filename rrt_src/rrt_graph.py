"""RRT (Rapidly-exploring Random Tree) algorithm implementation"""
import math
from core.graph import Graph


class RRTGraph(Graph):
    """RRT-specific graph implementation"""
    
    def __init__(self, start, goal, dimensions, environment, step_size=35):
        super().__init__(start, goal, dimensions)
        self.environment = environment
        self.step_size = step_size

    def expand(self):
        """Expand tree with a random node"""
        n = self.node_count()
        x, y = self.environment.sample_free_space()
        
        self.add_node(x, y)
        if not self.environment.is_collision_free(x, y):
            self.remove_node(n)
            return None
        
        nearest = self.nearest_node(n)
        self._limit_step(nearest, n)
        
        current_n = self.node_count() - 1
        if self._connect(nearest, current_n):
            return self.x, self.y, self.parent
        return None

    def bias_towards_goal(self):
        """Expand tree biased towards goal"""
        n = self.node_count()
        self.add_node(self.goal[0], self.goal[1])
        nearest = self.nearest_node(n)
        self._limit_step(nearest, n)
        current_n = self.node_count() - 1
        self._connect(nearest, current_n)
        return self.x, self.y, self.parent

    def _limit_step(self, near_idx, rand_idx):
        """Limit step size from nearest to random node"""
        d = self.distance(near_idx, rand_idx)
        if d > self.step_size:
            xnear, ynear = self.x[near_idx], self.y[near_idx]
            xrand, yrand = self.x[rand_idx], self.y[rand_idx]
            
            theta = math.atan2(yrand - ynear, xrand - xnear)
            x = int(xnear + self.step_size * math.cos(theta))
            y = int(ynear + self.step_size * math.sin(theta))
            
            self.remove_node(rand_idx)
            
            if abs(x - self.goal[0]) <= self.step_size and abs(y - self.goal[1]) <= self.step_size:
                self.add_node(self.goal[0], self.goal[1])
                self.goal_index = self.node_count() - 1
                self.goal_reached = True
            else:
                self.add_node(x, y)

    def _connect(self, idx1, idx2):
        """Connect two nodes if path is clear"""
        x1, y1 = self.x[idx1], self.y[idx1]
        x2, y2 = self.x[idx2], self.y[idx2]
        
        if not self.environment.is_path_clear(x1, y1, x2, y2):
            self.remove_node(idx2)
            return False
        
        self.add_edge(idx1, idx2)
        return True

    def check_goal(self):
        """Check if goal has been reached"""
        return self.reconstruct_path()
