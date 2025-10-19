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
├── prm_src/              # PRM algorithm (to be implemented)
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

### PRM (Probabilistic Roadmap)
*To be implemented*

### Hybrid Algorithm
*To be implemented*

## Core Architecture

The `core` module provides reusable components for all path planning algorithms:

- **Graph**: Abstract base class defining tree/graph operations
- **Environment**: Obstacle generation and collision detection
- **MapVisualizer**: Pygame-based visualization for algorithm execution