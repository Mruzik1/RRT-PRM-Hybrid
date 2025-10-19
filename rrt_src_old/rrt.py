"""
RRT (Rapidly-exploring Random Tree) Path Planning Algorithm

This program implements the RRT algorithm to find a path from a start
position to a goal position while avoiding randomly generated obstacles.

The algorithm works by:
1. Randomly sampling points in the space
2. Building a tree from the start position
3. Biasing some samples towards the goal
4. Connecting nodes if the path is obstacle-free
5. Continuing until the goal is reached
"""

import pygame
from rrt_base import RRTGraph
from rrt_base import RRTMap
import time
import random
import logging
from rrt_base import draw_new_node

# Configure logging to display progress information
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

def main():
    """Main function to run the RRT path planning algorithm"""
    
    logging.info("Initializing map and graph")
    
    # Configuration parameters
    dimensions = (1000, 1600)  # Map size (height, width)
    start = (100, 100)         # Starting position (x, y)
    goal = (1100, 800)         # Goal position (x, y)
    obsdim = 25                # Obstacle size (square side length)
    obsnum = 100               # Number of obstacles
    iteration = 0              # Iteration counter

    # Initialize pygame and create map visualization
    pygame.init()
    map = RRTMap(start, goal, dimensions, obsdim, obsnum)
    graph = RRTGraph(start, goal, dimensions, obsdim, obsnum)

    # Generate random obstacles and draw initial map
    logging.info("Generating obstacles")
    obstacles = graph.makeobs()
    map.drawMap(obstacles)

    font = pygame.font.Font(None, 25)

    # Main RRT loop: expand tree until goal is reached
    while not graph.path_to_goal():
        time.sleep(0.005)  # Small delay for visualization

        # Goal biasing: increase probability of sampling towards goal after 100 iterations
        goal_bias_probability = 0.1 if iteration < 100 else 0.4
        
        if random.random() < goal_bias_probability:
            # Bias expansion towards goal
            logging.info(f"Iteration {iteration}: Biasing towards the goal")
            X, Y, Parent = graph.bias(goal)
            logging.info(f"Iteration {iteration}: Node {len(X) - 1} connected to {Parent[-1]} ({X[Parent[-1]]}, {Y[Parent[-1]]}). New node position: ({X[-1]}, {Y[-1]})")
            draw_new_node(map, graph, X, Y, Parent)

        else:
            # Random expansion
            logging.info(f"Iteration {iteration}: Random expansion")
            X, Y, Parent = graph.expand()
            logging.info(f"Iteration {iteration}: Node {len(X) - 1} connected to {Parent[-1]} ({X[Parent[-1]]}, {Y[Parent[-1]]}). New node position: ({X[-1]}, {Y[-1]})")
            draw_new_node(map, graph, X, Y, Parent)

        # Update screen display every 5 iterations
        if iteration % 5 == 0:
            logging.info(f"Iteration {iteration}: Updating screen. Nodes: {graph.number_of_nodes()}")
            text = font.render(f'Nodes: {graph.number_of_nodes()} Iter: {iteration}',
                               True, (0, 0, 0))
            map.map.fill((255, 255, 255), (10, 10, 300, 40))
            map.map.blit(text, (10, 10))
            pygame.display.update()
        iteration += 1

    # Goal reached - draw the final path
    logging.info("Goal reached. Drawing path.")
    map.drawPath(graph.getPathCoords())
    pygame.display.update()

    # Keep window open until user closes it
    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                logging.info("Exiting program via close button")
                running = False
            elif event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
                logging.info("Exiting program via ESC key")
                running = False
        time.sleep(0.01)
    pygame.quit()
    logging.info("Program terminated")

if __name__ == '__main__':
    # Retry loop: restart if any error occurs
    result = False
    while not result:
        try:
            main()
            result = True
        except:
            result = False


























