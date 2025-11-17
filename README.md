# RRT-PRM-Hybrid

TUKE - NMvR Assignment

Path planning algorithms implementation and comparison for mobile robotics.

## Structure

```
RRT-PRM-Hybrid/
├── core/                 # Generalized base classes for path planning
│   ├── graph.py          # Abstract graph structure
│   ├── environment.py    # Obstacle handling and collision detection
│   └── visualization.py  # Pygame-based visualization
├── rrt_src/              # RRT algorithm implementation
│   ├── rrt.py            # Main RRT execution
│   └── rrt_graph.py      # RRT-specific graph logic
├── prm_src/              # PRM algorithm implementation
│   ├── prm.py            # Main PRM execution
│   └── prm_graph.py      # PRM-specific graph logic
├── hybrid_src/           # Hybrid RRT-PRM implementation
│   ├── hybrid.py         # Main Hybrid execution
│   └── hybrid_graph.py   # Hybrid algorithm logic
└── figures/              # Output visualizations
```

## Algorithms

### RRT (Rapidly-exploring Random Tree)

**Algorithm Overview:**
- Incrementally builds a tree by randomly sampling the configuration space
- Connects new samples to the nearest existing node
- Uses goal biasing to improve convergence
- Continues until a path to the goal is found

**Implementation Structure:**
- `RRTGraph`: Extends `Graph` base class with RRT-specific tree expansion logic
- `expand()`: Random sampling and tree growth
- `bias_towards_goal()`: Goal-biased sampling for faster convergence
- Configurable step size and goal bias probability

**Usage:**
```python
python rrt_src/rrt.py
```

**Parameters:**
- `dimensions`: Environment size (height, width)
- `start`: Starting position (x, y)
- `goal`: Goal position (x, y)
- `obstacle_size`: Size of square obstacles
- `obstacle_count`: Number of random obstacles
- `step_size`: Maximum extension distance (default: 35)
- `goal_bias`: Probability of sampling toward goal (default: 0.1)

**Example Output:**

![RRT Output](figures/rrt_output.png)

*RRT incrementally builds a tree (blue) by random sampling, connecting new points to nearest neighbors, until reaching the goal. The final path (red) shows the route from start to goal.*

### PRM (Probabilistic Roadmap)

**Algorithm Overview:**
- Pre-computes a roadmap of collision-free configurations
- Three-phase approach: sampling, connection, and query
- Samples random points in free space and connects nearby neighbors
- Searches the roadmap using A* to find optimal path

**Implementation Structure:**
- `PRMGraph`: Extends `Graph` base class with roadmap construction and A* search
- `build_roadmap()`: Sample N collision-free points and connect to k-nearest neighbors
- `add_start_goal()`: Connect start and goal positions to the roadmap
- `find_path()`: A* search to find optimal path through roadmap

**Usage:**
```python
python prm_src/prm.py
```

**Parameters:**
- `dimensions`: Environment size (height, width)
- `start`: Starting position (x, y)
- `goal`: Goal position (x, y)
- `obstacle_size`: Size of square obstacles
- `obstacle_count`: Number of random obstacles
- `num_samples`: Number of roadmap samples (default: 500)
- `k_neighbors`: Neighbors to connect per sample (default: 10)
- `show_roadmap`: Visualize full roadmap (default: False, recommended for large roadmaps)

**Example Output:**

![PRM Output](figures/prm_output.png)

*PRM pre-computes a roadmap by sampling random points and connecting nearby neighbors (blue network). Start and goal are connected (green), then A* finds the optimal path (red) through the roadmap.*

### Hybrid RRT-PRM

**Algorithm Overview:**
- Novel multi-query sampling-based planner combining RRT and PRM strengths
- Reduces path length by focusing search in a convex hull region near start and goal
- Five-phase approach: RRT path, PRM path, convex hull computation, focused sampling, and A* search
- Roadmap can be reused for multiple start-goal queries

**Implementation Structure:**
- `HybridGraph`: Extends `Graph` base class with hybrid algorithm logic
- `run_initial_rrt()`: Generate first path using RRT (Phase 1a)
- `run_initial_prm()`: Generate second path using PRM (Phase 1b)
- `compute_convex_hull()`: Create convex hull from combined paths (Phase 2)
- `sample_in_convex_hull()`: Sample collision-free points in convex hull using linear programming (Phase 3)
- `build_roadmap_in_convex_hull()`: Connect sampled points into roadmap (Phase 3)
- `find_path_astar()`: Find optimal path through roadmap (Phase 5)

**Algorithm Phases:**

1. **Phase 1a - Initial RRT Path**: Run RRT algorithm to quickly find a first path from start to goal
2. **Phase 1b - Initial PRM Path**: Run PRM algorithm to find a second, potentially more optimal path
3. **Phase 2 - Convex Hull**: Compute convex hull from the combined RRT and PRM path nodes, creating a focused search region
4. **Phase 3 - Focused Sampling**: Sample points within the convex hull region using linear programming to verify point inclusion, filtering out obstacle collisions
5. **Phase 4 - Connect Start/Goal**: Add start and goal positions to the roadmap built in the convex hull
6. **Phase 5 - Optimal Path**: Use A* graph search to find the shortest path through the roadmap

**Usage:**
```python
python hybrid_src/hybrid.py
```

**Parameters:**
- `dimensions`: Environment size (height, width)
- `start`: Starting position (x, y)
- `goal`: Goal position (x, y)
- `obstacle_size`: Size of square obstacles
- `obstacle_count`: Number of random obstacles
- `rrt_step_size`: Maximum extension distance for RRT phase (default: 35)
- `prm_samples_initial`: Number of samples for initial PRM (default: 200)
- `num_samples_convex`: Number of samples within convex hull (default: 300)
- `k_neighbors`: Neighbors to connect per sample (default: 10)
- `show_roadmap`: Visualize full roadmap (default: True)

**Visualization Legend:**
- **Light Blue**: Initial RRT path
- **Cyan**: Initial PRM path
- **Orange**: Convex hull boundary
- **Green**: Roadmap built within convex hull
- **Yellow**: Start/goal connections to roadmap
- **Red**: Final optimal hybrid path

**Example Output:**

![Hybrid Output](figures/hybrid_output.png)

*Hybrid RRT-PRM combines initial RRT (light blue) and PRM (cyan) paths to compute a convex hull (orange boundary). Dense sampling within the hull creates a focused roadmap (green), enabling A\* to find an improved path (red) that's shorter than either initial path.*

## Algorithm Comparison

![Algorithm Comparison](figures/algorithm_comparison.png)

*Side-by-side comparison of RRT, PRM, and Hybrid algorithms solving the same problem. RRT (left) builds an incremental tree, PRM (middle) creates a dense pre-computed roadmap, and Hybrid (right) combines both approaches with focused sampling in a convex hull region for improved path quality.*

## Core Architecture

The `core` module provides reusable components for all path planning algorithms:

- **Graph**: Abstract base class defining tree/graph operations
- **Environment**: Obstacle generation and collision detection
- **MapVisualizer**: Pygame-based visualization for algorithm execution

## References

**Hybrid RRT-PRM Paper:**
- Jermyn, J. (2021). "A Comparison of the Effectiveness of the RRT, PRM, and Novel Hybrid RRT-PRM Path Planners." 
  International Journal for Research in Applied Science & Engineering Technology (IJRASET), 9(12), 600-611. 
  DOI: 10.22214/ijraset.2021.39297
