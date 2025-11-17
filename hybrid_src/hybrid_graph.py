"""Hybrid RRT-PRM algorithm"""
import heapq
import numpy as np
from scipy.spatial import ConvexHull, cKDTree
from shapely.geometry import Point, Polygon
from core.graph import Graph
from rrt_src.rrt_graph import RRTGraph
from prm_src.prm_graph import PRMGraph


class HybridGraph(Graph):
    
    def __init__(self, start, goal, dimensions, environment, 
                 rrt_step_size=35, prm_samples_initial=200, 
                 num_samples_convex=300, k_neighbors=10):
        super().__init__(start, goal, dimensions)
        self.environment = environment
        self.rrt_step_size = rrt_step_size
        self.prm_samples_initial = prm_samples_initial
        self.num_samples_convex = num_samples_convex
        self.k_neighbors = k_neighbors
        
        self.rrt_path = []
        self.prm_path = []
        self.convex_hull = None
        self.convex_hull_points = None
        self.convex_hull_polygon = None
        self.neighbors = []
        self.roadmap_built = False
        self._kdtree = None
        
        self.x = []
        self.y = []
        self.parent = []

    def run_initial_rrt(self):
        print("  RRT phase...", end='', flush=True)
        rrt = RRTGraph(self.start, self.goal, (self.height, self.width),
                      self.environment, self.rrt_step_size)
        
        iteration = 0
        max_iterations = 1000
        
        while not rrt.check_goal() and iteration < max_iterations:
            if np.random.random() < 0.15:
                rrt.bias_towards_goal()
            else:
                rrt.expand()
            iteration += 1
        
        if rrt.goal_reached:
            self.rrt_path = rrt.get_path_coordinates()
            print(f" path found ({len(self.rrt_path)} nodes)")
            return True
        print(" failed")
        return False

    def run_initial_prm(self):
        print("  PRM phase...", end='', flush=True)
        prm = PRMGraph(self.start, self.goal, (self.height, self.width),
                      self.environment, self.prm_samples_initial, self.k_neighbors)
        
        samples = 0
        attempts = 0
        max_attempts = self.prm_samples_initial * 10
        
        while samples < self.prm_samples_initial and attempts < max_attempts:
            x, y = self.environment.sample_free_space()
            if self.environment.is_collision_free(x, y):
                prm.add_node(x, y)
                prm.neighbors.append([])
                samples += 1
            attempts += 1
        
        for i in range(prm.node_count()):
            distances = [(prm.distance(i, j), j) for j in range(i + 1, prm.node_count())]
            if distances:
                distances.sort()
                for _, j in distances[:self.k_neighbors]:
                    if prm._can_connect(i, j):
                        prm._add_bidirectional_edge(i, j)
        
        prm.roadmap_built = True
        prm.add_start_goal()
        
        if prm.find_path():
            self.prm_path = prm.get_path_coordinates()
            print(f" path found ({len(self.prm_path)} nodes)")
            return True
        print(" failed")
        return False

    def compute_convex_hull(self):
        print("  Computing convex hull...", end='', flush=True)
        
        combined_points = list(self.rrt_path[1:-1]) + list(self.prm_path)
        unique_points = list(set(combined_points))
        
        if len(unique_points) < 3:
            print(" not enough points")
            return False
        
        self.convex_hull_points = np.array(unique_points)
        
        try:
            self.convex_hull = ConvexHull(self.convex_hull_points)
            hull_vertices = self.convex_hull_points[self.convex_hull.vertices]
            self.convex_hull_polygon = Polygon(hull_vertices)
            print(f" {len(hull_vertices)} vertices")
            return True
        except Exception as e:
            print(f" failed: {e}")
            return False

    def is_point_in_convex_hull(self, point):
        if self.convex_hull_polygon is None:
            return False
        return self.convex_hull_polygon.contains(Point(point))

    def sample_in_convex_hull(self):
        print(f"  Sampling {self.num_samples_convex} points in hull...", end='', flush=True)
        
        min_x, max_x = np.min(self.convex_hull_points[:, 0]), np.max(self.convex_hull_points[:, 0])
        min_y, max_y = np.min(self.convex_hull_points[:, 1]), np.max(self.convex_hull_points[:, 1])
        
        sampled_points = []
        attempts = 0
        max_attempts = self.num_samples_convex * 20
        
        while len(sampled_points) < self.num_samples_convex and attempts < max_attempts:
            x, y = np.random.uniform(min_x, max_x), np.random.uniform(min_y, max_y)
            if self.is_point_in_convex_hull((x, y)):
                if self.environment.is_collision_free(int(x), int(y)):
                    sampled_points.append((int(x), int(y)))
            attempts += 1
        
        print(f" {len(sampled_points)} sampled")
        return sampled_points

    def build_roadmap_in_convex_hull(self, sampled_points):
        print("  Building roadmap...", end='', flush=True)
        
        if not sampled_points:
            return 0
        
        for x, y in sampled_points:
            self.add_node(x, y)
            self.neighbors.append([])
        
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
        
        print(f" {connections} edges")
        self.roadmap_built = True
        return connections

    def add_start_goal_to_roadmap(self):
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

    def find_path_astar(self):
        print("  A* search...", end='', flush=True)
        
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
                print(f" path found ({len(self.path)} nodes)")
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
        
        print(" no path")
        return False

    def _can_connect(self, idx1, idx2):
        return self.environment.is_path_clear(self.x[idx1], self.y[idx1], self.x[idx2], self.y[idx2])

    def _add_bidirectional_edge(self, idx1, idx2):
        if idx2 not in self.neighbors[idx1]:
            self.neighbors[idx1].append(idx2)
        if idx1 not in self.neighbors[idx2]:
            self.neighbors[idx2].append(idx1)

    def _connect_to_roadmap(self, node_idx):
        if self._kdtree is None:
            distances = [(self.distance(i, node_idx), i) for i in range(self.node_count() - 2)]
            if distances:
                distances.sort()
                for _, idx in distances[:self.k_neighbors]:
                    if self._can_connect(node_idx, idx):
                        self._add_bidirectional_edge(node_idx, idx)
        else:
            distances, indices = self._kdtree.query(
                [self.x[node_idx], self.y[node_idx]], 
                k=min(self.k_neighbors, self.node_count() - 2)
            )
            for idx in indices:
                if idx < self.node_count() - 2:
                    if self._can_connect(node_idx, idx):
                        self._add_bidirectional_edge(node_idx, idx)

    def _heuristic(self, idx1, idx2):
        return self.distance(idx1, idx2)

    def get_convex_hull_vertices(self):
        if self.convex_hull is not None:
            return self.convex_hull_points[self.convex_hull.vertices]
        return None

    def expand(self):
        pass

    def check_goal(self):
        return self.goal_reached
