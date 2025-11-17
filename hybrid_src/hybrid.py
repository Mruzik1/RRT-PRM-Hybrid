"""Hybrid RRT-PRM path planning algorithm execution"""
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
from hybrid_src.hybrid_graph import HybridGraph

logging.basicConfig(level=logging.INFO, format='%(message)s')


def run_hybrid(config):
    """Execute Hybrid RRT-PRM algorithm with given configuration"""
    pygame.init()
    
    visualizer = MapVisualizer(
        config['start'],
        config['goal'],
        config['dimensions'],
        "Hybrid RRT-PRM Path Planning"
    )
    
    environment = Environment(
        config['dimensions'],
        config['obstacle_size'],
        config['obstacle_count']
    )
    
    graph = HybridGraph(
        config['start'],
        config['goal'],
        config['dimensions'],
        environment,
        rrt_step_size=config.get('rrt_step_size', 35),
        prm_samples_initial=config.get('prm_samples_initial', 200),
        num_samples_convex=config.get('num_samples_convex', 300),
        k_neighbors=config.get('k_neighbors', 10)
    )
    
    # Generate obstacles
    obstacles = environment.generate_obstacles(config['start'], config['goal'])
    visualizer.draw_initial(obstacles)
    visualizer.update_display()
    
    logging.info("=" * 60)
    logging.info("HYBRID RRT-PRM PATH PLANNING")
    logging.info("=" * 60)
    
    # Phase 1a: Run RRT for initial path
    logging.info("\n[PHASE 1a] Running RRT...")
    start_time = time.time()
    if not graph.run_initial_rrt():
        logging.error("Failed to find RRT path. Exiting.")
        pygame.quit()
        return
    rrt_time = time.time() - start_time
    logging.info(f"RRT completed in {rrt_time:.2f}s")
    
    # Draw RRT path in light blue
    rrt_color = (100, 150, 255)
    for i in range(len(graph.rrt_path) - 1):
        visualizer.draw_edge(graph.rrt_path[i], graph.rrt_path[i+1], color=rrt_color)
    for point in graph.rrt_path:
        visualizer.draw_node(point, color=rrt_color, radius=3)
    
    text_y = config['dimensions'][0] - 40
    visualizer.draw_text("Phase 1a: RRT Path (light blue)", position=(10, text_y), clear_area=(10, text_y, 400, 30))
    visualizer.update_display()
    time.sleep(0.5)
    
    # Phase 1b: Run PRM for second path
    logging.info("\n[PHASE 1b] Running PRM...")
    start_time = time.time()
    if not graph.run_initial_prm():
        logging.error("Failed to find PRM path. Exiting.")
        pygame.quit()
        return
    prm_time = time.time() - start_time
    logging.info(f"PRM completed in {prm_time:.2f}s")
    
    # Draw PRM path in cyan
    prm_color = (0, 200, 200)
    for i in range(len(graph.prm_path) - 1):
        visualizer.draw_edge(graph.prm_path[i], graph.prm_path[i+1], color=prm_color)
    for point in graph.prm_path:
        visualizer.draw_node(point, color=prm_color, radius=3)
    
    text_y = config['dimensions'][0] - 40
    visualizer.draw_text("Phase 1b: PRM Path (cyan)", position=(10, text_y), clear_area=(10, text_y, 400, 30))
    visualizer.update_display()
    time.sleep(0.5)
    
    # Phase 2: Compute convex hull
    logging.info("\n[PHASE 2] Computing convex hull...")
    start_time = time.time()
    if not graph.compute_convex_hull():
        logging.error("Failed to compute convex hull. Exiting.")
        pygame.quit()
        return
    hull_time = time.time() - start_time
    logging.info(f"Convex hull computed in {hull_time:.3f}s")
    
    # Draw convex hull boundary in orange
    hull_vertices = graph.get_convex_hull_vertices()
    if hull_vertices is not None:
        hull_color = (255, 165, 0)
        for i in range(len(hull_vertices)):
            p1 = tuple(hull_vertices[i].astype(int))
            p2 = tuple(hull_vertices[(i+1) % len(hull_vertices)].astype(int))
            pygame.draw.line(visualizer.surface, hull_color, p1, p2, 2)
        
        text_y = config['dimensions'][0] - 40
        visualizer.draw_text("Phase 2: Convex Hull (orange)", position=(10, text_y), clear_area=(10, text_y, 400, 30))
        visualizer.update_display()
        time.sleep(0.5)
    
    # Phase 3: Sample in convex hull and build roadmap
    logging.info("\n[PHASE 3] Sampling and building roadmap...")
    start_time = time.time()
    sampled_points = graph.sample_in_convex_hull()
    sample_time = time.time() - start_time
    
    if len(sampled_points) == 0:
        logging.error("No points sampled in convex hull. Exiting.")
        pygame.quit()
        return
    
    start_time = time.time()
    connections = graph.build_roadmap_in_convex_hull(sampled_points)
    build_time = time.time() - start_time
    logging.info(f"Roadmap built: {len(sampled_points)} nodes, {connections} connections in {sample_time + build_time:.2f}s")
    
    # Draw roadmap in green
    show_roadmap = config.get('show_roadmap', True)
    if show_roadmap:
        roadmap_color = (0, 255, 0)
        for i in range(graph.node_count()):
            visualizer.draw_node((graph.x[i], graph.y[i]), color=roadmap_color, radius=1)
            for j in graph.neighbors[i]:
                if i < j:
                    visualizer.draw_edge((graph.x[i], graph.y[i]), (graph.x[j], graph.y[j]), 
                                       color=roadmap_color)
        
        text_y = config['dimensions'][0] - 40
        visualizer.draw_text("Phase 3: Hybrid Roadmap (green)", position=(10, text_y), clear_area=(10, text_y, 400, 30))
        visualizer.update_display()
        time.sleep(0.5)
    
    # Phase 4: Add start and goal
    logging.info("\n[PHASE 4] Adding start and goal to roadmap...")
    start_idx, goal_idx = graph.add_start_goal_to_roadmap()
    
    # Draw start/goal connections in yellow
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
    
    # Phase 5: Find optimal path
    logging.info("\n[PHASE 5] Finding optimal path...")
    start_time = time.time()
    path_found = graph.find_path_astar()
    search_time = time.time() - start_time
    
    if path_found:
        # Calculate path length
        path_coords = graph.get_path_coordinates()
        total_distance = sum(((path_coords[i+1][0]-path_coords[i][0])**2 + 
                              (path_coords[i+1][1]-path_coords[i][1])**2)**0.5 
                            for i in range(len(path_coords)-1))
        
        # Calculate RRT path length
        rrt_distance = sum(((graph.rrt_path[i+1][0]-graph.rrt_path[i][0])**2 + 
                           (graph.rrt_path[i+1][1]-graph.rrt_path[i][1])**2)**0.5 
                          for i in range(len(graph.rrt_path)-1))
        
        # Calculate PRM path length
        prm_distance = sum(((graph.prm_path[i+1][0]-graph.prm_path[i][0])**2 + 
                           (graph.prm_path[i+1][1]-graph.prm_path[i][1])**2)**0.5 
                          for i in range(len(graph.prm_path)-1))
        
        logging.info(f"\nPath found in {search_time:.2f}s")
        logging.info(f"  Hybrid path: {len(graph.path)} nodes, distance: {total_distance:.1f}")
        logging.info(f"  RRT path:    {len(graph.rrt_path)} nodes, distance: {rrt_distance:.1f}")
        logging.info(f"  PRM path:    {len(graph.prm_path)} nodes, distance: {prm_distance:.1f}")
        
        improvement_rrt = ((rrt_distance - total_distance) / rrt_distance * 100)
        improvement_prm = ((prm_distance - total_distance) / prm_distance * 100)
        logging.info(f"\n  Improvement over RRT: {improvement_rrt:.1f}%")
        logging.info(f"  Improvement over PRM: {improvement_prm:.1f}%")
        
        # Draw final path in red (thick)
        visualizer.draw_path(path_coords)
        
        text_y = config['dimensions'][0] - 40
        total_time = rrt_time + prm_time + hull_time + sample_time + build_time + search_time
        text = f"Nodes: {len(graph.path)} | Distance: {total_distance:.1f} | Time: {total_time:.2f}s | Improvement: {improvement_rrt:.1f}%"
        visualizer.draw_text(text, position=(10, text_y), clear_area=(10, text_y, 700, 30))
    else:
        logging.error("No path found!")
        text_y = config['dimensions'][0] - 40
        visualizer.draw_text("No path found!", position=(10, text_y), clear_area=(10, text_y, 400, 30))
    
    visualizer.update_display()
    
    logging.info("\n" + "=" * 60)
    logging.info("Total time: {:.2f}s".format(rrt_time + prm_time + hull_time + sample_time + build_time + search_time))
    logging.info("=" * 60)
    logging.info("Press ESC or close window to exit")
    
    # Wait for user to close
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
        'rrt_step_size': 35,
        'prm_samples_initial': 200,
        'num_samples_convex': 300,
        'k_neighbors': 10,
        'show_roadmap': True
    }
    
    try:
        run_hybrid(config)
    except Exception as e:
        logging.error(f"Error during execution: {e}")
        import traceback
        traceback.print_exc()
        raise


if __name__ == '__main__':
    main()
