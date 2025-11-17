"""PRM (Probabilistic Roadmap) algorithm"""
import heapq
import numpy as np
from scipy.spatial import cKDTree
from core.graph import Graph


class PRMGraph(Graph):
    
    def __init__(self, start, goal, dimensions, environment, num_samples=500, k_neighbors=10):
        super().__init__(start, goal, dimensions)
        self.environment = environment
        self.num_samples = num_samples
        self.k_neighbors = k_neighbors
        self.neighbors = []
        self.roadmap_built = False
        self._kdtree = None
        
        self.x = []
        self.y = []
        self.parent = []

    def build_roadmap(self):
        samples_added = 0
        attempts = 0
        max_attempts = self.num_samples * 10
        
        print(f"  Sampling {self.num_samples} points...", end='', flush=True)
        while samples_added < self.num_samples and attempts < max_attempts:
            x, y = self.environment.sample_free_space()
            if self.environment.is_collision_free(x, y):
                self.add_node(x, y)
                self.neighbors.append([])
                samples_added += 1
            attempts += 1
        print(f" {samples_added} sampled")
        
        if samples_added == 0:
            return 0
        
        points = np.column_stack((self.x, self.y))
        self._kdtree = cKDTree(points)
        
        connections = 0
        for i in range(self.node_count()):
            distances, indices = self._kdtree.query(
                [self.x[i], self.y[i]], 
                k=min(self.k_neighbors + 1, self.node_count())
            )
            
            for j_idx in range(1, len(indices)):
                j = indices[j_idx]
                if j > i and j not in self.neighbors[i]:
                    if self._can_connect(i, j):
                        self._add_bidirectional_edge(i, j)
                        connections += 1
        
        print(f"  Connected {connections} edges")
        self.roadmap_built = True
        return samples_added

    def add_start_goal(self):
        start_idx = self.node_count()
        self.add_node(self.start[0], self.start[1])
        self.neighbors.append([])
        
        goal_idx = self.node_count()
        self.add_node(self.goal[0], self.goal[1])
        self.neighbors.append([])
        self.goal_index = goal_idx
        
        self._connect_to_roadmap(start_idx)
        self._connect_to_roadmap(goal_idx)
        
        return start_idx, goal_idx

    def find_path(self):
        if not self.roadmap_built:
            return False
        
        start_idx = self.node_count() - 2
        goal_idx = self.goal_index
        
        open_set = [(0, start_idx)]
        came_from = {}
        g_score = {i: float('inf') for i in range(self.node_count())}
        g_score[start_idx] = 0
        f_score = {i: float('inf') for i in range(self.node_count())}
        f_score[start_idx] = self._heuristic(start_idx, goal_idx)
        
        visited = set()
        
        while open_set:
            _, current = heapq.heappop(open_set)
            
            if current in visited:
                continue
            visited.add(current)
            
            if current == goal_idx:
                self.path = []
                while current in came_from:
                    self.path.append(current)
                    current = came_from[current]
                self.path.append(start_idx)
                self.path.reverse()
                self.goal_reached = True
                return True
            
            for neighbor in self.neighbors[current]:
                if neighbor in visited:
                    continue
                    
                tentative_g = g_score[current] + self.distance(current, neighbor)
                
                if tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score[neighbor] = tentative_g + self._heuristic(neighbor, goal_idx)
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))
        
        return False

    def _find_k_nearest(self, node_idx):
        if self._kdtree is None:
            distances = [(self.distance(i, node_idx), i) for i in range(self.node_count()) if i != node_idx]
            distances.sort()
            return [idx for _, idx in distances[:self.k_neighbors]]
        
        distances, indices = self._kdtree.query(
            [self.x[node_idx], self.y[node_idx]], 
            k=min(self.k_neighbors + 1, self.node_count())
        )
        return [idx for idx in indices if idx != node_idx][:self.k_neighbors]

    def _can_connect(self, idx1, idx2):
        return self.environment.is_path_clear(self.x[idx1], self.y[idx1], self.x[idx2], self.y[idx2])

    def _add_bidirectional_edge(self, idx1, idx2):
        if idx2 not in self.neighbors[idx1]:
            self.neighbors[idx1].append(idx2)
        if idx1 not in self.neighbors[idx2]:
            self.neighbors[idx2].append(idx1)

    def _connect_to_roadmap(self, node_idx):
        nearest_indices = self._find_k_nearest(node_idx)
        for idx in nearest_indices:
            if self._can_connect(node_idx, idx):
                self._add_bidirectional_edge(node_idx, idx)

    def _heuristic(self, idx1, idx2):
        return self.distance(idx1, idx2)

    def expand(self):
        pass

    def check_goal(self):
        return self.goal_reached
