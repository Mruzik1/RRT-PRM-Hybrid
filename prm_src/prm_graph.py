"""PRM (Probabilistic Roadmap) algorithm implementation"""
import heapq
from core.graph import Graph


class PRMGraph(Graph):
    """PRM-specific graph implementation"""
    
    def __init__(self, start, goal, dimensions, environment, num_samples=500, k_neighbors=10):
        super().__init__(start, goal, dimensions)
        self.environment = environment
        self.num_samples = num_samples
        self.k_neighbors = k_neighbors
        self.neighbors = []  # List of lists: neighbors[i] contains indices of neighbors of node i
        self.roadmap_built = False
        
        # PRM doesn't start with any nodes (unlike RRT)
        self.x = []
        self.y = []
        self.parent = []

    def build_roadmap(self):
        """Phase 1: Sample points and build roadmap"""
        # Sample collision-free points
        samples_added = 0
        attempts = 0
        max_attempts = self.num_samples * 10
        
        print("  Sampling collision-free points...", end='', flush=True)
        while samples_added < self.num_samples and attempts < max_attempts:
            x, y = self.environment.sample_free_space()
            if self.environment.is_collision_free(x, y):
                self.add_node(x, y)
                self.neighbors.append([])
                samples_added += 1
                if samples_added % 100 == 0:
                    print(f"\r  Sampled {samples_added}/{self.num_samples} points...", end='', flush=True)
            attempts += 1
        print(f"\r  Sampled {samples_added} collision-free points")
        
        # Connect to k nearest neighbors
        print("  Connecting neighbors...", end='', flush=True)
        connections = 0
        for i in range(self.node_count()):
            # Find k nearest neighbors (only nodes not yet connected)
            distances = []
            for j in range(i + 1, self.node_count()):
                if j not in self.neighbors[i]:  # Skip if already connected
                    dist = self.distance(i, j)
                    distances.append((dist, j))
            
            if not distances:
                continue
                
            distances.sort()
            nearest = [idx for _, idx in distances[:self.k_neighbors]]
            
            # Try to connect to nearest neighbors
            for j in nearest:
                if self._can_connect(i, j):
                    self._add_bidirectional_edge(i, j)
                    connections += 1
            
            if (i + 1) % 50 == 0:
                print(f"\r  Processed {i + 1}/{self.node_count()} nodes, {connections} connections...", end='', flush=True)
        
        print(f"\r  Connected {connections} edge pairs")
        
        self.roadmap_built = True
        return samples_added

    def add_start_goal(self):
        """Phase 2: Add start and goal to roadmap"""
        # Add start node at the end (to avoid index shifting issues)
        start_idx = self.node_count()
        self.add_node(self.start[0], self.start[1])
        self.neighbors.append([])
        
        # Add goal node
        goal_idx = self.node_count()
        self.add_node(self.goal[0], self.goal[1])
        self.neighbors.append([])
        self.goal_index = goal_idx
        
        # Connect start and goal to nearby nodes
        self._connect_to_roadmap(start_idx)
        self._connect_to_roadmap(goal_idx)
        
        return start_idx, goal_idx

    def find_path(self):
        """Phase 3: Find path from start to goal using A*"""
        if not self.roadmap_built:
            return False
        
        # Start is now at node_count - 2, goal is at node_count - 1
        start_idx = self.node_count() - 2
        goal_idx = self.goal_index
        
        # A* algorithm
        open_set = [(0, start_idx)]  # (f_score, node)
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
                # Reconstruct path
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
        """Find k nearest neighbors to node_idx"""
        distances = []
        for i in range(self.node_count()):
            if i != node_idx:
                dist = self.distance(i, node_idx)
                distances.append((dist, i))
        
        distances.sort()
        return [idx for _, idx in distances[:self.k_neighbors]]

    def _can_connect(self, idx1, idx2):
        """Check if two nodes can be connected"""
        x1, y1 = self.x[idx1], self.y[idx1]
        x2, y2 = self.x[idx2], self.y[idx2]
        return self.environment.is_path_clear(x1, y1, x2, y2)

    def _add_bidirectional_edge(self, idx1, idx2):
        """Add edge in both directions"""
        if idx2 not in self.neighbors[idx1]:
            self.neighbors[idx1].append(idx2)
        if idx1 not in self.neighbors[idx2]:
            self.neighbors[idx2].append(idx1)

    def _connect_to_roadmap(self, node_idx):
        """Connect a node to nearby nodes in roadmap"""
        nearest_indices = self._find_k_nearest(node_idx)
        for idx in nearest_indices:
            if self._can_connect(node_idx, idx):
                self._add_bidirectional_edge(node_idx, idx)

    def _heuristic(self, idx1, idx2):
        """Heuristic for A* (Euclidean distance)"""
        return self.distance(idx1, idx2)

    def expand(self):
        """Not used in PRM (required by abstract base class)"""
        pass

    def check_goal(self):
        """Check if path to goal exists"""
        return self.goal_reached
