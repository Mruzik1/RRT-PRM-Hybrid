"""PRM path planning algorithm execution"""
import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import random
random.seed(42)

import pygame
import time
import logging
from core.visualization import MapVisualizer
from core.environment import Environment
from prm_src.prm_graph import PRMGraph

logging.basicConfig(level=logging.INFO, format='%(message)s')


def run_prm(config):
    """Execute PRM algorithm with given configuration"""
    pygame.init()
    
    visualizer = MapVisualizer(
        config['start'],
        config['goal'],
        config['dimensions'],
        "PRM Path Planning"
    )
    
    environment = Environment(
        config['dimensions'],
        config['obstacle_size'],
        config['obstacle_count']
    )
    
    graph = PRMGraph(
        config['start'],
        config['goal'],
        config['dimensions'],
        environment,
        config.get('num_samples', 500),
        config.get('k_neighbors', 10)
    )
    
    obstacles = environment.generate_obstacles(config['start'], config['goal'])
    visualizer.draw_initial(obstacles)
    visualizer.update_display()
    
    # Phase 1: Build roadmap
    logging.info("Building roadmap...")
    start_time = time.time()
    samples_added = graph.build_roadmap()
    build_time = time.time() - start_time
    logging.info(f"Roadmap built: {samples_added} samples in {build_time:.2f}s")
    
    # Draw roadmap
    show_roadmap = config.get('show_roadmap', False)
    if show_roadmap:
        logging.info("Drawing roadmap...")
        for i in range(graph.node_count()):
            visualizer.draw_node((graph.x[i], graph.y[i]), radius=1)
            for j in graph.neighbors[i]:
                if i < j:  # Draw each edge only once
                    visualizer.draw_edge((graph.x[i], graph.y[i]), (graph.x[j], graph.y[j]))
        
        text = f'Nodes: {graph.node_count()} Edges: {sum(len(n) for n in graph.neighbors)//2}'
        visualizer.draw_text(text, clear_area=(10, 10, 400, 40))
        visualizer.update_display()
    else:
        logging.info("Roadmap visualization skipped (show_roadmap=False)")
    
    # Phase 2: Add start and goal
    logging.info("Adding start and goal to roadmap...")
    start_idx, goal_idx = graph.add_start_goal()
    
    # Draw start and goal connections
    for neighbor in graph.neighbors[start_idx]:
        visualizer.draw_edge((graph.x[start_idx], graph.y[start_idx]), 
                           (graph.x[neighbor], graph.y[neighbor]), 
                           color=(0, 255, 0))
    for neighbor in graph.neighbors[goal_idx]:
        visualizer.draw_edge((graph.x[goal_idx], graph.y[goal_idx]), 
                           (graph.x[neighbor], graph.y[neighbor]), 
                           color=(0, 255, 0))
    
    visualizer.update_display()
    
    # Phase 3: Find path
    logging.info("Searching for path...")
    search_start = time.time()
    path_found = graph.find_path()
    search_time = time.time() - search_start
    
    if path_found:
        logging.info(f"Path found in {search_time:.2f}s with {len(graph.path)} nodes")
        visualizer.draw_path(graph.get_path_coordinates())
        
        # Calculate path length
        path_coords = graph.get_path_coordinates()
        total_distance = 0
        for i in range(len(path_coords) - 1):
            x1, y1 = path_coords[i]
            x2, y2 = path_coords[i+1]
            total_distance += ((x2-x1)**2 + (y2-y1)**2)**0.5
        
        text = f'Path: {len(graph.path)} nodes, Distance: {total_distance:.1f}'
        visualizer.draw_text(text, position=(10, 40), clear_area=(10, 40, 400, 30))
    else:
        logging.info("No path found!")
        text = 'No path found!'
        visualizer.draw_text(text, position=(10, 40), clear_area=(10, 40, 400, 30))
    
    visualizer.update_display()
    
    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT or (event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE):
                running = False
        time.sleep(0.01)
    
    pygame.quit()


def main():
    """Main entry point"""
    config = {
        'dimensions': (1000, 1600),
        'start': (100, 100),
        'goal': (1100, 800),
        'obstacle_size': 25,
        'obstacle_count': 100,
        'num_samples': 500,
        'k_neighbors': 10,
        'show_roadmap': False
    }
    
    try:
        run_prm(config)
    except Exception as e:
        logging.error(f"Error during execution: {e}")
        raise


if __name__ == '__main__':
    main()
