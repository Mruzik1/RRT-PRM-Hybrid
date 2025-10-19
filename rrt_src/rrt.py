import pygame
import time
import logging

import os, sys
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import random
random.seed(42)

from core.visualization import MapVisualizer
from core.environment import Environment
from rrt_src.rrt_graph import RRTGraph

logging.basicConfig(level=logging.INFO, format='%(message)s')


def run_rrt(config):
    """Execute RRT algorithm with given configuration"""
    pygame.init()
    
    visualizer = MapVisualizer(
        config['start'], 
        config['goal'], 
        config['dimensions'],
        "RRT Path Planning"
    )
    
    environment = Environment(
        config['dimensions'],
        config['obstacle_size'],
        config['obstacle_count']
    )
    
    graph = RRTGraph(
        config['start'],
        config['goal'],
        config['dimensions'],
        environment,
        config.get('step_size', 35)
    )
    
    obstacles = environment.generate_obstacles(config['start'], config['goal'])
    visualizer.draw_initial(obstacles)
    
    iteration = 0
    goal_bias = config.get('goal_bias', 0.1)
    goal_bias_threshold = config.get('goal_bias_threshold', 100)
    
    while not graph.check_goal():
        time.sleep(0.005)
        
        bias_prob = goal_bias if iteration < goal_bias_threshold else goal_bias * 4
        
        if random.random() < bias_prob:
            result = graph.bias_towards_goal()
        else:
            result = graph.expand()
        
        if result:
            X, Y, Parent = result
            visualizer.draw_node((X[-1], Y[-1]))
            visualizer.draw_edge((X[-1], Y[-1]), (X[Parent[-1]], Y[Parent[-1]]))
        
        if iteration % 5 == 0:
            text = f'Nodes: {graph.node_count()} Iter: {iteration}'
            visualizer.draw_text(text, clear_area=(10, 10, 300, 40))
            visualizer.update_display()
        
        iteration += 1
    
    logging.info(f"Goal reached after {iteration} iterations")
    visualizer.draw_path(graph.get_path_coordinates())
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
        'step_size': 35,
        'goal_bias': 0.1,
        'goal_bias_threshold': 100
    }
    
    try:
        run_rrt(config)
    except Exception as e:
        logging.error(f"Error during execution: {e}")
        raise


if __name__ == '__main__':
    main()
