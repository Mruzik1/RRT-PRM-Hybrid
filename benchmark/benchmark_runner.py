"""Benchmark runner for Hybrid RRT-PRM algorithm"""
import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import random
import time
import logging
from dataclasses import dataclass, field
from typing import List, Dict, Optional
import json
from datetime import datetime
from pathlib import Path

import pygame
from core.visualization import MapVisualizer
from core.environment import Environment
from hybrid_src.hybrid_graph import HybridGraph


@dataclass
class BenchmarkResult:
    """Results from a single benchmark run"""
    success: bool
    rrt_time: float = 0.0
    prm_time: float = 0.0
    hull_time: float = 0.0
    sample_time: float = 0.0
    build_time: float = 0.0
    search_time: float = 0.0
    total_time: float = 0.0
    
    hybrid_distance: float = 0.0
    hybrid_nodes: int = 0
    rrt_distance: float = 0.0
    rrt_nodes: int = 0
    prm_distance: float = 0.0
    prm_nodes: int = 0
    
    improvement_over_rrt: float = 0.0
    improvement_over_prm: float = 0.0
    
    roadmap_nodes: int = 0
    roadmap_connections: int = 0
    
    error_message: str = ""
    
    # Image paths
    image_rrt: str = ""
    image_prm: str = ""
    image_hull: str = ""
    image_roadmap: str = ""
    image_final: str = ""


@dataclass
class BenchmarkStats:
    """Aggregated statistics from multiple runs"""
    total_runs: int = 0
    successful_runs: int = 0
    failed_runs: int = 0
    success_rate: float = 0.0
    
    avg_total_time: float = 0.0
    avg_hybrid_distance: float = 0.0
    avg_improvement_rrt: float = 0.0
    avg_improvement_prm: float = 0.0
    
    min_distance: float = float('inf')
    max_distance: float = 0.0
    min_time: float = float('inf')
    max_time: float = 0.0
    
    results: List[BenchmarkResult] = field(default_factory=list)


class BenchmarkRunner:
    """Runs multiple iterations of the algorithm and collects statistics"""
    
    def __init__(self, config: Dict, headless: bool = False, save_images: bool = True, run_images_dir: Optional[Path] = None):
        self.config = config
        self.headless = headless
        self.save_images = save_images
        self.stats = BenchmarkStats()
        
        # Create images directory
        if save_images:
            if run_images_dir is not None:
                # Use provided directory (shared across iterations)
                self.run_images_dir = run_images_dir
            else:
                # Create new timestamped subdirectory
                self.images_dir = Path('images')
                self.images_dir.mkdir(exist_ok=True)
                
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                self.run_images_dir = self.images_dir / f"run_{timestamp}"
                self.run_images_dir.mkdir(exist_ok=True)
                logging.info(f"Images will be saved to: {self.run_images_dir}")
        
        if not headless:
            pygame.init()
    
    def run_single_iteration(self, seed: Optional[int] = None, save_images: bool = None) -> BenchmarkResult:
        """Run a single iteration of the algorithm"""
        if seed is not None:
            random.seed(seed)
        
        if save_images is None:
            save_images = self.save_images
        
        result = BenchmarkResult(success=False)
        
        try:
            # Initialize components
            visualizer = MapVisualizer(
                self.config['start'],
                self.config['goal'],
                self.config['dimensions'],
                f"Hybrid RRT-PRM (Seed: {seed})"
            )
            
            environment = Environment(
                self.config['dimensions'],
                self.config['obstacle_size'],
                self.config['obstacle_count']
            )
            
            graph = HybridGraph(
                self.config['start'],
                self.config['goal'],
                self.config['dimensions'],
                environment,
                rrt_step_size=self.config.get('rrt_step_size', 35),
                prm_samples_initial=self.config.get('prm_samples_initial', 200),
                num_samples_convex=self.config.get('num_samples_convex', 300),
                k_neighbors=self.config.get('k_neighbors', 10)
            )
            
            # Create iteration directory if saving images
            if save_images:
                iter_dir = self.run_images_dir / f"iteration_{seed:03d}"
                iter_dir.mkdir(exist_ok=True)
            
            # Generate obstacles
            obstacles = environment.generate_obstacles(self.config['start'], self.config['goal'])
            visualizer.draw_initial(obstacles)
            visualizer.update_display()
            
            # Phase 1a: RRT
            start_time = time.time()
            if not graph.run_initial_rrt():
                result.error_message = "RRT failed"
                return result
            result.rrt_time = time.time() - start_time
            result.rrt_nodes = len(graph.rrt_path)
            result.rrt_distance = self._calculate_path_length(graph.rrt_path)
            
            # Draw and save RRT - show all tree edges
            rrt_tree_color = (200, 220, 255)  # Light blue for tree
            rrt_path_color = (100, 150, 255)  # Dark blue for path
            
            # Draw all RRT tree edges
            if graph.rrt_graph:
                for i in range(1, graph.rrt_graph.node_count()):
                    parent_idx = graph.rrt_graph.parent[i]
                    if parent_idx >= 0:
                        node_pos = (graph.rrt_graph.x[i], graph.rrt_graph.y[i])
                        parent_pos = (graph.rrt_graph.x[parent_idx], graph.rrt_graph.y[parent_idx])
                        visualizer.draw_edge(parent_pos, node_pos, color=rrt_tree_color, width=1)
            
            # Draw RRT path on top
            for i in range(len(graph.rrt_path) - 1):
                visualizer.draw_edge(graph.rrt_path[i], graph.rrt_path[i+1], color=rrt_path_color, width=3)
            for point in graph.rrt_path:
                visualizer.draw_node(point, color=rrt_path_color, radius=4)
            
            text_y = self.config['dimensions'][0] - 40
            visualizer.draw_text("Phase 1a: RRT Path (light blue)", 
                               position=(10, text_y), clear_area=(10, text_y, 400, 30))
            visualizer.update_display()
            
            if save_images:
                result.image_rrt = str(iter_dir / "01_rrt.png")
                pygame.image.save(visualizer.surface, result.image_rrt)
            
            # Phase 1b: PRM
            start_time = time.time()
            if not graph.run_initial_prm():
                result.error_message = "PRM failed"
                return result
            result.prm_time = time.time() - start_time
            result.prm_nodes = len(graph.prm_path)
            result.prm_distance = self._calculate_path_length(graph.prm_path)
            
            # Clear screen for PRM (redraw obstacles and start/goal)
            visualizer.surface.fill((255, 255, 255))
            visualizer.draw_initial(obstacles)
            
            # Draw and save PRM - show all graph edges
            prm_graph_color = (220, 180, 255)  # Light purple for graph
            prm_path_color = (148, 0, 211)     # Dark purple for path
            
            # Draw all PRM graph edges
            if graph.prm_graph:
                for i in range(graph.prm_graph.node_count()):
                    for j in graph.prm_graph.neighbors[i]:
                        if i < j:  # Draw each edge only once
                            node_i = (graph.prm_graph.x[i], graph.prm_graph.y[i])
                            node_j = (graph.prm_graph.x[j], graph.prm_graph.y[j])
                            visualizer.draw_edge(node_i, node_j, color=prm_graph_color, width=1)
            
            # Draw PRM path on top
            for i in range(len(graph.prm_path) - 1):
                visualizer.draw_edge(graph.prm_path[i], graph.prm_path[i+1], color=prm_path_color, width=3)
            for point in graph.prm_path:
                visualizer.draw_node(point, color=prm_path_color, radius=4)
            
            visualizer.draw_text("Phase 1b: PRM Path (violet)", 
                               position=(10, text_y), clear_area=(10, text_y, 400, 30))
            visualizer.update_display()
            
            if save_images:
                result.image_prm = str(iter_dir / "02_prm.png")
                pygame.image.save(visualizer.surface, result.image_prm)
            
            # Phase 2: Convex hull
            start_time = time.time()
            if not graph.compute_convex_hull():
                result.error_message = "Convex hull computation failed"
                return result
            result.hull_time = time.time() - start_time
            
            # Clear screen and redraw for convex hull
            visualizer.surface.fill((255, 255, 255))
            visualizer.draw_initial(obstacles)
            
            # Draw convex hull
            hull_vertices = graph.get_convex_hull_vertices()
            if hull_vertices is not None:
                hull_color = (255, 165, 0)
                for i in range(len(hull_vertices)):
                    p1 = tuple(hull_vertices[i].astype(int))
                    p2 = tuple(hull_vertices[(i+1) % len(hull_vertices)].astype(int))
                    pygame.draw.line(visualizer.surface, hull_color, p1, p2, 3)
                
                # Draw only RRT and PRM paths (no graph edges)
                rrt_path_color = (100, 150, 255)
                for i in range(len(graph.rrt_path) - 1):
                    visualizer.draw_edge(graph.rrt_path[i], graph.rrt_path[i+1], color=rrt_path_color, width=3)
                for point in graph.rrt_path:
                    visualizer.draw_node(point, color=rrt_path_color, radius=4)
                
                prm_path_color = (148, 0, 211)
                for i in range(len(graph.prm_path) - 1):
                    visualizer.draw_edge(graph.prm_path[i], graph.prm_path[i+1], color=prm_path_color, width=3)
                for point in graph.prm_path:
                    visualizer.draw_node(point, color=prm_path_color, radius=4)
                
                visualizer.draw_text("Phase 2: Convex Hull (orange)", 
                                   position=(10, text_y), clear_area=(10, text_y, 400, 30))
                visualizer.update_display()
                
                if save_images:
                    result.image_hull = str(iter_dir / "03_hull.png")
                    pygame.image.save(visualizer.surface, result.image_hull)
            
            # Phase 3: Sample and build roadmap
            start_time = time.time()
            sampled_points = graph.sample_in_convex_hull()
            result.sample_time = time.time() - start_time
            
            if len(sampled_points) == 0:
                result.error_message = "No points sampled"
                return result
            
            start_time = time.time()
            connections = graph.build_roadmap_in_convex_hull(sampled_points)
            result.build_time = time.time() - start_time
            result.roadmap_nodes = len(sampled_points)
            result.roadmap_connections = connections
            
            # Draw and save roadmap
            roadmap_color = (144, 238, 144)
            for i in range(graph.node_count()):
                visualizer.draw_node((graph.x[i], graph.y[i]), color=roadmap_color, radius=1)
                for j in graph.neighbors[i]:
                    if i < j:
                        visualizer.draw_edge((graph.x[i], graph.y[i]), (graph.x[j], graph.y[j]), 
                                           color=roadmap_color)
            
            visualizer.draw_text("Phase 3: Hybrid Roadmap (light green)", 
                               position=(10, text_y), clear_area=(10, text_y, 400, 30))
            visualizer.update_display()
            
            if save_images:
                result.image_roadmap = str(iter_dir / "04_roadmap.png")
                pygame.image.save(visualizer.surface, result.image_roadmap)
            
            # Phase 4: Add start/goal
            start_idx, goal_idx = graph.add_start_goal_to_roadmap()
            
            # Draw start/goal connections
            connection_color = (255, 255, 0)
            for neighbor in graph.neighbors[start_idx]:
                visualizer.draw_edge((graph.x[start_idx], graph.y[start_idx]),
                                   (graph.x[neighbor], graph.y[neighbor]),
                                   color=connection_color)
            for neighbor in graph.neighbors[goal_idx]:
                visualizer.draw_edge((graph.x[goal_idx], graph.y[goal_idx]),
                                   (graph.x[neighbor], graph.y[neighbor]),
                                   color=connection_color)
            visualizer.update_display()
            
            # Phase 5: Find path
            start_time = time.time()
            path_found = graph.find_path_astar()
            result.search_time = time.time() - start_time
            
            if not path_found:
                result.error_message = "A* search failed"
                return result
            
            # Calculate final metrics
            path_coords = graph.get_path_coordinates()
            result.hybrid_distance = self._calculate_path_length(path_coords)
            result.hybrid_nodes = len(path_coords)
            
            result.improvement_over_rrt = ((result.rrt_distance - result.hybrid_distance) / result.rrt_distance * 100)
            result.improvement_over_prm = ((result.prm_distance - result.hybrid_distance) / result.prm_distance * 100)
            
            result.total_time = (result.rrt_time + result.prm_time + result.hull_time + 
                               result.sample_time + result.build_time + result.search_time)
            
            # Draw and save final path
            visualizer.draw_path(path_coords)
            
            text = f"Nodes: {len(path_coords)} | Distance: {result.hybrid_distance:.1f} | " \
                   f"Time: {result.total_time:.2f}s | Improvement: {result.improvement_over_rrt:.1f}%"
            visualizer.draw_text(text, position=(10, text_y), clear_area=(10, text_y, 800, 30))
            visualizer.update_display()
            
            if save_images:
                result.image_final = str(iter_dir / "05_final.png")
                pygame.image.save(visualizer.surface, result.image_final)
            
            result.success = True
            
        except Exception as e:
            result.error_message = str(e)
            logging.error(f"Error in iteration: {e}")
        
        finally:
            pygame.quit()
            if not self.headless:
                pygame.init()  # Reinitialize for next iteration
        
        return result
    
    def run_benchmark(self, num_iterations: int, start_seed: int = 0, 
                     save_images_for: str = 'all') -> BenchmarkStats:
        """
        Run multiple iterations and collect statistics
        
        Args:
            num_iterations: Number of iterations to run
            start_seed: Starting seed value
            save_images_for: 'all', 'successful', 'failed', or 'none'
        """
        logging.info("=" * 60)
        logging.info(f"RUNNING BENCHMARK: {num_iterations} iterations")
        logging.info(f"Saving images: {save_images_for}")
        logging.info("=" * 60)
        
        for i in range(num_iterations):
            seed = start_seed + i
            logging.info(f"\n[Iteration {i+1}/{num_iterations}] Seed: {seed}")
            
            # Determine if we should save images for this iteration
            save_images = self.save_images and (save_images_for == 'all')
            
            result = self.run_single_iteration(seed=seed, save_images=save_images)
            
            # Save images for failed/successful runs if requested
            if self.save_images and not save_images:
                if (save_images_for == 'failed' and not result.success) or \
                   (save_images_for == 'successful' and result.success):
                    result = self.run_single_iteration(seed=seed, save_images=True)
            
            self.stats.results.append(result)
            self.stats.total_runs += 1
            
            if result.success:
                self.stats.successful_runs += 1
                logging.info(f"  ✓ Success | Time: {result.total_time:.2f}s")
                logging.info(f"    Distances: Hybrid={result.hybrid_distance:.1f}, "
                           f"RRT={result.rrt_distance:.1f}, PRM={result.prm_distance:.1f}")
                logging.info(f"    Improvement: vs RRT={result.improvement_over_rrt:.1f}%, "
                           f"vs PRM={result.improvement_over_prm:.1f}%")
            else:
                self.stats.failed_runs += 1
                logging.info(f"  ✗ Failed: {result.error_message}")
        
        self._calculate_stats()
        return self.stats
    
    def _calculate_stats(self):
        """Calculate aggregate statistics"""
        successful = [r for r in self.stats.results if r.success]
        
        if len(successful) == 0:
            return
        
        self.stats.success_rate = (self.stats.successful_runs / self.stats.total_runs * 100)
        self.stats.avg_total_time = sum(r.total_time for r in successful) / len(successful)
        self.stats.avg_hybrid_distance = sum(r.hybrid_distance for r in successful) / len(successful)
        self.stats.avg_improvement_rrt = sum(r.improvement_over_rrt for r in successful) / len(successful)
        self.stats.avg_improvement_prm = sum(r.improvement_over_prm for r in successful) / len(successful)
        
        self.stats.min_distance = min(r.hybrid_distance for r in successful)
        self.stats.max_distance = max(r.hybrid_distance for r in successful)
        self.stats.min_time = min(r.total_time for r in successful)
        self.stats.max_time = max(r.total_time for r in successful)
    
    def print_summary(self):
        """Print benchmark summary"""
        logging.info("\n" + "=" * 60)
        logging.info("BENCHMARK SUMMARY")
        logging.info("=" * 60)
        logging.info(f"Total runs:       {self.stats.total_runs}")
        logging.info(f"Successful:       {self.stats.successful_runs} ({self.stats.success_rate:.1f}%)")
        logging.info(f"Failed:           {self.stats.failed_runs}")
        
        if self.stats.successful_runs > 0:
            successful = [r for r in self.stats.results if r.success]
            avg_rrt_distance = sum(r.rrt_distance for r in successful) / len(successful)
            avg_prm_distance = sum(r.prm_distance for r in successful) / len(successful)
            
            logging.info(f"\nAverage metrics:")
            logging.info(f"  Time:           {self.stats.avg_total_time:.2f}s")
            logging.info(f"  Distance:")
            logging.info(f"    Hybrid:       {self.stats.avg_hybrid_distance:.1f}")
            logging.info(f"    RRT:          {avg_rrt_distance:.1f}")
            logging.info(f"    PRM:          {avg_prm_distance:.1f}")
            logging.info(f"  Improvement:")
            logging.info(f"    vs RRT:       {self.stats.avg_improvement_rrt:.1f}%")
            logging.info(f"    vs PRM:       {self.stats.avg_improvement_prm:.1f}%")
            
            logging.info(f"\nRange:")
            logging.info(f"  Distance:       {self.stats.min_distance:.1f} - {self.stats.max_distance:.1f}")
            logging.info(f"  Time:           {self.stats.min_time:.2f}s - {self.stats.max_time:.2f}s")
        
        if self.save_images:
            logging.info(f"\nImages saved to: {self.run_images_dir}")
        
        logging.info("=" * 60)
    
    def save_results(self, filename: Optional[str] = None):
        """Save results to JSON file"""
        if filename is None:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"benchmark/benchmark_results_{timestamp}.json"
        
        data = {
            'config': self.config,
            'stats': {
                'total_runs': self.stats.total_runs,
                'successful_runs': self.stats.successful_runs,
                'failed_runs': self.stats.failed_runs,
                'success_rate': self.stats.success_rate,
                'avg_total_time': self.stats.avg_total_time,
                'avg_hybrid_distance': self.stats.avg_hybrid_distance,
                'avg_improvement_rrt': self.stats.avg_improvement_rrt,
                'avg_improvement_prm': self.stats.avg_improvement_prm,
                'min_distance': self.stats.min_distance,
                'max_distance': self.stats.max_distance,
                'min_time': self.stats.min_time,
                'max_time': self.stats.max_time,
            },
            'results': [
                {
                    'success': r.success,
                    'total_time': r.total_time,
                    'hybrid_distance': r.hybrid_distance,
                    'hybrid_nodes': r.hybrid_nodes,
                    'rrt_distance': r.rrt_distance,
                    'prm_distance': r.prm_distance,
                    'improvement_over_rrt': r.improvement_over_rrt,
                    'improvement_over_prm': r.improvement_over_prm,
                    'error_message': r.error_message,
                    'images': {
                        'rrt': r.image_rrt,
                        'prm': r.image_prm,
                        'hull': r.image_hull,
                        'roadmap': r.image_roadmap,
                        'final': r.image_final
                    }
                }
                for r in self.stats.results
            ]
        }
        
        with open(filename, 'w') as f:
            json.dump(data, f, indent=2)
        
        logging.info(f"\nResults saved to: {filename}")
    
    @staticmethod
    def _calculate_path_length(path: List[tuple]) -> float:
        """Calculate total path length"""
        return sum(((path[i+1][0]-path[i][0])**2 + (path[i+1][1]-path[i][1])**2)**0.5 
                  for i in range(len(path)-1))


def generate_random_config():
    """Generate a random configuration for testing"""
    import random as rand
    
    # Fixed screen dimensions
    main_dimension = (800, 1600)  # (height, width)
    main_dimension_1 = (1000, 1600)
    
    # Random start and goal positions within screen bounds
    margin = 100
    start = (
        rand.randint(margin, main_dimension[1] - margin),
        rand.randint(margin, main_dimension[0] - margin)
    )
    goal = (
        rand.randint(margin, main_dimension[1] - margin),
        rand.randint(margin, main_dimension[0] - margin)
    )
    
    # Make sure start and goal are not too close
    min_distance = 900
    while ((goal[0] - start[0])**2 + (goal[1] - start[1])**2)**0.5 < min_distance:
        goal = (
            rand.randint(margin, main_dimension[1] - margin),
            rand.randint(margin, main_dimension[0] - margin)
        )
    
    # Random parameters
    config = {
        'dimensions': main_dimension_1,
        'start': start,
        'goal': goal,
        'obstacle_size': rand.randint(40, 40),
        'obstacle_count': rand.randint(15, 15),
        'rrt_step_size': rand.randint(30, 40),
        'prm_samples_initial': rand.randint(80, 100),
        'num_samples_convex': rand.randint(800, 1000),
        'k_neighbors': rand.randint(40, 50),
    }
    
    return config


def generate_config_variations():
    """Generate different configuration variations for testing"""
    base_config = {
        'dimensions': (1000, 1600),
        'start': (100, 100),
        'goal': (700, 1400),
        'obstacle_size': 25,
    }
    
    variations = []
    
    # Vary obstacle count (density of environment)
    for obstacle_count in [250, 300, 350]:
        config = base_config.copy()
        config.update({
            'obstacle_count': obstacle_count,
            'rrt_step_size': 50,
            'prm_samples_initial': 100,
            'num_samples_convex': 800,
            'k_neighbors': 20,
        })
        variations.append(('obstacles', obstacle_count, config))
    
    # Vary sampling density in convex hull
    for samples in [600, 800, 1000]:
        config = base_config.copy()
        config.update({
            'obstacle_count': 200,
            'rrt_step_size': 50,
            'prm_samples_initial': 100,
            'num_samples_convex': samples,
            'k_neighbors': 20,
        })
        variations.append(('samples', samples, config))
    
    # Vary k_neighbors (roadmap connectivity)
    for k in [20, 25, 30]:
        config = base_config.copy()
        config.update({
            'obstacle_count': 200,
            'rrt_step_size': 50,
            'prm_samples_initial': 100,
            'num_samples_convex': 800,
            'k_neighbors': k,
        })
        variations.append(('k_neighbors', k, config))
    
    # Vary RRT step size
    for step_size in [25, 30, 35]:
        config = base_config.copy()
        config.update({
            'obstacle_count': 200,
            'rrt_step_size': step_size,
            'prm_samples_initial': 100,
            'num_samples_convex': 800,
            'k_neighbors': 20,
        })
        variations.append(('rrt_step', step_size, config))
    
    # Vary start and goal positions (path difficulty)
    start_goal_pairs = [
        ((25, 25), (1600, 800)),      # diagonal 
        ((25, 800), (1600, 0)),      # diagonal 
        ((25, 800), (1600, 800)),       # horizontal
    ]
    for idx, (start, goal) in enumerate(start_goal_pairs):
        config = base_config.copy()
        config.update({
            'start': start,
            'goal': goal,
            'obstacle_count': 200,
            'rrt_step_size': 50,
            'prm_samples_initial': 100,
            'num_samples_convex': 800,
            'k_neighbors': 20,
        })
        # Calculate distance for description
        distance = ((goal[0]-start[0])**2 + (goal[1]-start[1])**2)**0.5
        variations.append(('path_distance', int(distance), config))
    
    return variations


def main():
    """Main entry point for benchmark"""
    # Configuration
    MODE = 'single'  # 'single', 'variations', or 'random'
    ITERATIONS = 1
    SAVE_IMAGES = 'all'  # 'all', 'successful', 'failed', or 'none'
    START_SEED = None  # Use None for random seed, or set a number for reproducible results
    
    # Single config parameters (only used if MODE = 'single')
    START_POS = (100, 100)      # Start position (x, y)
    GOAL_POS = (1000, 1000)      # Goal position (x, y)
    OBSTACLES = 15
    SAMPLES = 200
    K_NEIGHBORS = 10
    RRT_STEP = 50
    PRM_STEPS = 100
    
    if MODE == 'single':
        # Run single configuration
        config = {
            'dimensions': (1200, 1200),
            'start': START_POS,
            'goal': GOAL_POS,
            'obstacle_size': 35,
            'obstacle_count': OBSTACLES,
            'rrt_step_size': RRT_STEP,
            'prm_samples_initial': PRM_STEPS,
            'num_samples_convex': SAMPLES,
            'k_neighbors': K_NEIGHBORS,
        }
        
        logging.info(f"Running single configuration benchmark")
        logging.info(f"Start: {START_POS}, Goal: {GOAL_POS}")
        logging.info(f"Config: obstacles={OBSTACLES}, samples={SAMPLES}, "
                    f"k_neighbors={K_NEIGHBORS}, rrt_step={RRT_STEP}")
        
        runner = BenchmarkRunner(config, headless=False, save_images=(SAVE_IMAGES != 'none'))
        
        # If START_SEED is None, use a random seed based on current time
        actual_seed = START_SEED if START_SEED is not None else int(time.time() * 1000) % 1000000
        logging.info(f"Using seed: {actual_seed}")
        
        runner.run_benchmark(num_iterations=ITERATIONS, start_seed=actual_seed, 
                            save_images_for=SAVE_IMAGES)
        runner.print_summary()
        runner.save_results()
        
    elif MODE == 'random':
        # Run random configurations
        logging.info(f"Running {ITERATIONS} iterations with random configurations")
        logging.info("=" * 60)
        
        # Create single shared directory for all iterations
        shared_images_dir = None
        if SAVE_IMAGES != 'none':
            images_dir = Path('images')
            images_dir.mkdir(exist_ok=True)
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            shared_images_dir = images_dir / f"run_{timestamp}_iter{ITERATIONS}"
            shared_images_dir.mkdir(exist_ok=True)
            logging.info(f"Images will be saved to: {shared_images_dir}")
        
        all_results = []
        
        for i in range(ITERATIONS):
            # Generate random config for each iteration
            config = generate_random_config()
            
            logging.info(f"\n{'='*60}")
            logging.info(f"ITERATION {i+1}/{ITERATIONS}")
            logging.info(f"{'='*60}")
            logging.info(f"Dimensions: {config['dimensions']}")
            logging.info(f"Start: {config['start']}, Goal: {config['goal']}")
            logging.info(f"Obstacles: {config['obstacle_count']}, Samples: {config['num_samples_convex']}")
            logging.info(f"k_neighbors: {config['k_neighbors']}, RRT step: {config['rrt_step_size']}")
            
            runner = BenchmarkRunner(config, headless=False, save_images=(SAVE_IMAGES != 'none'), run_images_dir=shared_images_dir)
            # Run single iteration with unique seed
            result = runner.run_single_iteration(seed=START_SEED + i, save_images=(SAVE_IMAGES != 'none'))
            
            if result.success:
                logging.info(f"  ✓ Success | Time: {result.total_time:.2f}s")
                logging.info(f"    Distances: Hybrid={result.hybrid_distance:.1f}, "
                           f"RRT={result.rrt_distance:.1f}, PRM={result.prm_distance:.1f}")
                logging.info(f"    Improvement: vs RRT={result.improvement_over_rrt:.1f}%, "
                           f"vs PRM={result.improvement_over_prm:.1f}%")
            else:
                logging.info(f"  ✗ Failed: {result.error_message}")
            
            # Collect results
            all_results.append({
                'iteration': i + 1,
                'config': config,
                'result': {
                    'success': result.success,
                    'total_time': result.total_time,
                    'hybrid_distance': result.hybrid_distance,
                    'hybrid_nodes': result.hybrid_nodes,
                    'rrt_distance': result.rrt_distance,
                    'prm_distance': result.prm_distance,
                    'improvement_over_rrt': result.improvement_over_rrt,
                    'improvement_over_prm': result.improvement_over_prm,
                    'error_message': result.error_message,
                }
            })
        
        # Print summary
        successful = [r for r in all_results if r['result']['success']]
        
        logging.info(f"\n{'='*60}")
        logging.info("RANDOM CONFIGURATIONS SUMMARY")
        logging.info(f"{'='*60}")
        logging.info(f"Total runs:       {len(all_results)}")
        logging.info(f"Successful:       {len(successful)} ({len(successful)/len(all_results)*100:.1f}%)")
        logging.info(f"Failed:           {len(all_results) - len(successful)}")
        
        if successful:
            avg_time = sum(r['result']['total_time'] for r in successful) / len(successful)
            avg_distance = sum(r['result']['hybrid_distance'] for r in successful) / len(successful)
            avg_rrt_distance = sum(r['result']['rrt_distance'] for r in successful) / len(successful)
            avg_prm_distance = sum(r['result']['prm_distance'] for r in successful) / len(successful)
            avg_improvement_rrt = sum(r['result']['improvement_over_rrt'] for r in successful) / len(successful)
            avg_improvement_prm = sum(r['result']['improvement_over_prm'] for r in successful) / len(successful)
            
            logging.info(f"\nAverage metrics:")
            logging.info(f"  Time:           {avg_time:.2f}s")
            logging.info(f"  Distance:")
            logging.info(f"    Hybrid:       {avg_distance:.1f}")
            logging.info(f"    RRT:          {avg_rrt_distance:.1f}")
            logging.info(f"    PRM:          {avg_prm_distance:.1f}")
            logging.info(f"  Improvement:")
            logging.info(f"    vs RRT:       {avg_improvement_rrt:.1f}%")
            logging.info(f"    vs PRM:       {avg_improvement_prm:.1f}%")
        
        # Save results
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        results_file = f"benchmark/random_results_{timestamp}.json"
        with open(results_file, 'w') as f:
            json.dump(all_results, f, indent=2)
        logging.info(f"\nResults saved to: {results_file}")
        logging.info(f"{'='*60}")
        
    else:
        # Run configuration variations
        variations = generate_config_variations()
        
        logging.info(f"Testing {len(variations)} configuration variations")
        logging.info(f"Iterations per config: {ITERATIONS}")
        logging.info("")
        
        all_results = []
        
        for param_name, param_value, config in variations:
            logging.info(f"\n{'='*60}")
            logging.info(f"TESTING: {param_name} = {param_value}")
            logging.info(f"{'='*60}")
            
            runner = BenchmarkRunner(config, headless=False, save_images=(SAVE_IMAGES != 'none'))
            runner.run_benchmark(num_iterations=ITERATIONS, start_seed=START_SEED, 
                                save_images_for=SAVE_IMAGES)
            runner.print_summary()
            
            # Save individual results
            result_filename = f"benchmark/results_{param_name}_{param_value}.json"
            runner.save_results(filename=result_filename)
            
            # Collect summary for comparison
            all_results.append({
                'param_name': param_name,
                'param_value': param_value,
                'config': config,
                'stats': {
                    'success_rate': runner.stats.success_rate,
                    'avg_time': runner.stats.avg_total_time,
                    'avg_distance': runner.stats.avg_hybrid_distance,
                    'avg_improvement_rrt': runner.stats.avg_improvement_rrt,
                    'avg_improvement_prm': runner.stats.avg_improvement_prm,
                }
            })
        
        # Print comparison summary
        logging.info(f"\n{'='*60}")
        logging.info("VARIATION COMPARISON SUMMARY")
        logging.info(f"{'='*60}")
        
        for result in all_results:
            logging.info(f"\n{result['param_name']} = {result['param_value']}:")
            logging.info(f"  Success rate:    {result['stats']['success_rate']:.1f}%")
            logging.info(f"  Avg time:        {result['stats']['avg_time']:.2f}s")
            logging.info(f"  Avg distances:")
            logging.info(f"    Hybrid:        {result['stats']['avg_distance']:.1f}")
            logging.info(f"    RRT:           {result['stats'].get('avg_rrt_distance', 0):.1f}")
            logging.info(f"    PRM:           {result['stats'].get('avg_prm_distance', 0):.1f}")
            logging.info(f"  Improvement:")
            logging.info(f"    vs RRT:        {result['stats']['avg_improvement_rrt']:.1f}%")
            logging.info(f"    vs PRM:        {result['stats']['avg_improvement_prm']:.1f}%")
        
        # Save comparison results
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        comparison_file = f"benchmark/comparison_{timestamp}.json"
        with open(comparison_file, 'w') as f:
            json.dump(all_results, f, indent=2)
        logging.info(f"\nComparison results saved to: {comparison_file}")
        logging.info(f"{'='*60}")



if __name__ == '__main__':
    logging.basicConfig(level=logging.INFO, format='%(message)s')
    main()