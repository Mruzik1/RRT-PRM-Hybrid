import random
import math
import pygame


class RRTMap:
    """Visualization class for RRT algorithm"""
    
    def __init__(self, start, goal, MapDimensions, obsdim, obsnum):
        """
        Initialize the map for RRT visualization
        
        Args:
            start: Tuple (x, y) for starting position
            goal: Tuple (x, y) for goal position
            MapDimensions: Tuple (height, width) of the map
            obsdim: Size of obstacles
            obsnum: Number of obstacles
        """
        self.start = start
        self.goal = goal
        self.MapDimensions = MapDimensions
        self.Maph, self.Mapw = self.MapDimensions

        # Window settings
        self.MapWindowName = 'RRT path planning'
        pygame.display.set_caption(self.MapWindowName)
        self.map = pygame.display.set_mode((self.Mapw, self.Maph))
        self.map.fill((255, 255, 255))
        
        # Drawing parameters
        self.nodeRad = 2
        self.nodeThickness = 0
        self.edgeThickness = 1

        # Obstacles
        self.obstacles = []
        self.obsdim = obsdim
        self.obsNumber = obsnum

        # Colors (RGB)
        self.grey = (70, 70, 70)      # Obstacles
        self.Blue = (0, 0, 255)       # Tree edges
        self.Green = (0, 255, 0)      # Start and goal
        self.Red = (255, 0, 0)        # Final path


    def drawMap(self, obstacles):
        """Draw the initial map with start, goal, and obstacles"""
        pygame.draw.circle(self.map, self.Green, self.start, self.nodeRad + 5, 0)
        pygame.draw.circle(self.map, self.Green, self.goal, self.nodeRad + 20, 1)
        self.drawObs(obstacles)

    def drawPath(self, path):
        """Draw the final path from start to goal"""
        # Draw path lines
        for i in range(len(path) - 1):
            pygame.draw.line(self.map, self.Red, path[i], path[i+1], 3)
        
        # Draw path nodes
        for node in path:
            pygame.draw.circle(self.map, self.Red, node, 4, 0)

    def drawObs(self, obstacles):
        """Draw all obstacles on the map"""
        for obstacle in obstacles:
            pygame.draw.rect(self.map, self.grey, obstacle)


class RRTGraph:
    """RRT algorithm implementation for path planning"""
    
    def __init__(self, start, goal, MapDimensions, obsdim, obsnum):
        """
        Initialize the RRT graph
        
        Args:
            start: Tuple (x, y) for starting position
            goal: Tuple (x, y) for goal position
            MapDimensions: Tuple (height, width) of the map
            obsdim: Size of obstacles
            obsnum: Number of obstacles
        """
        x, y = start
        self.start = start
        self.goal = goal
        self.goalFlag = False  # True when goal is reached
        self.maph, self.mapw = MapDimensions
        
        # Tree structure: lists of x, y coordinates and parent indices
        self.x = [x]
        self.y = [y]
        self.parent = [0]  # Index of parent node (root has itself as parent)
        
        # Obstacles
        self.obstacles = []
        self.obsDim = obsdim
        self.obsNum = obsnum
        
        # Path tracking
        self.goalstate = None  # Index of goal node when reached
        self.path = []  # List of node indices from goal to start

    def makeRandomRect(self):
        """
        Generate random position for obstacle
        
        Returns:
            Tuple (x, y) for upper-left corner of obstacle
        """
        uppercornerx = int(random.uniform(0, self.mapw - self.obsDim))
        uppercornery = int(random.uniform(0, self.maph - self.obsDim))
        return (uppercornerx, uppercornery)

    def makeobs(self):
        """
        Create random obstacles avoiding start and goal positions
        
        Returns:
            List of pygame.Rect objects representing obstacles
        """
        obs = []
        for i in range(self.obsNum):
            while True:
                upper = self.makeRandomRect()
                rectang = pygame.Rect(upper, (self.obsDim, self.obsDim))
                # Ensure obstacle doesn't overlap with start or goal
                if not (rectang.collidepoint(self.start) or rectang.collidepoint(self.goal)):
                    obs.append(rectang)
                    break
        self.obstacles = obs.copy()
        return obs


    def add_node(self, n, x, y):
        """Add a new node to the tree at index n"""
        self.x.insert(n, x)
        self.y.append(y)


    def remove_node(self, n):
        """Remove node at index n from the tree"""
        self.x.pop(n)
        self.y.pop(n)


    def add_edge(self, child, parent):
        """
        Connect child node to parent node
        
        Args:
            child: Index of child node
            parent: Index of parent node
        """
        self.parent.insert(parent, child)

    def number_of_nodes(self):
        """Return the total number of nodes in the tree"""
        return len(self.x)

    def distance(self, n1, n2):
        """
        Calculate Euclidean distance between two nodes
        
        Args:
            n1: Index of first node
            n2: Index of second node
            
        Returns:
            Distance as float
        """
        x1, y1 = self.x[n1], self.y[n1]
        x2, y2 = self.x[n2], self.y[n2]
        return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)

    def sample_envir(self):
        """
        Sample a random point in the environment
        
        Returns:
            Tuple (x, y) of random coordinates
        """
        x = int(random.uniform(0, self.mapw))
        y = int(random.uniform(0, self.maph))
        return x, y

    def nearest(self, n):
        """
        Find the nearest node in the tree to node n
        
        Args:
            n: Index of target node
            
        Returns:
            Index of nearest node
        """
        dmin = self.distance(0, n)
        nnear = 0
        for i in range(0, n):
            if self.distance(i, n) < dmin:
                dmin = self.distance(i, n)
                nnear = i
        return nnear

    def isFree(self):
        """
        Check if the last added node is free from obstacles
        
        Returns:
            True if node is free, False otherwise (node is removed if in obstacle)
        """
        n = self.number_of_nodes() - 1
        x, y = self.x[n], self.y[n]
        
        for obstacle in self.obstacles:
            if obstacle.collidepoint(x, y):
                self.remove_node(n)
                return False
        return True

    def crossObstacle(self, x1, x2, y1, y2):
        """
        Check if line segment from (x1,y1) to (x2,y2) crosses any obstacle
        
        Args:
            x1, y1: Start point coordinates
            x2, y2: End point coordinates
            
        Returns:
            True if path crosses obstacle, False otherwise
        """
        steps = 100
        for rect in self.obstacles:
            # Check points along the line segment
            for i in range(steps + 1):
                t = i / steps
                x = x1 + t * (x2 - x1)
                y = y1 + t * (y2 - y1)
                if rect.collidepoint(x, y):
                    return True
        return False

    def connect(self, n1, n2):
        """
        Try to connect two nodes if path is obstacle-free
        
        Args:
            n1: Index of first node
            n2: Index of second node
            
        Returns:
            True if connection successful, False if path blocked
        """
        x1, y1 = self.x[n1], self.y[n1]
        x2, y2 = self.x[n2], self.y[n2]
        
        if self.crossObstacle(x1, x2, y1, y2):
            self.remove_node(n2)
            return False
        else:
            self.add_edge(n1, n2)
            return True

    def step(self, nnear, nrand, dmax=35):
        """
        Limit the distance of new node from nearest node to dmax
        
        Args:
            nnear: Index of nearest node
            nrand: Index of random node
            dmax: Maximum step size
        """
        d = self.distance(nnear, nrand)
        if d > dmax:
            # Calculate new position at distance dmax from nnear towards nrand
            xnear, ynear = self.x[nnear], self.y[nnear]
            xrand, yrand = self.x[nrand], self.y[nrand]
            
            theta = math.atan2(yrand - ynear, xrand - xnear)
            x = int(xnear + dmax * math.cos(theta))
            y = int(ynear + dmax * math.sin(theta))
            
            self.remove_node(nrand)
            
            # Check if we're close enough to goal
            if abs(x - self.goal[0]) <= dmax and abs(y - self.goal[1]) <= dmax:
                self.add_node(nrand, self.goal[0], self.goal[1])
                self.goalstate = nrand
                self.goalFlag = True
            else:
                self.add_node(nrand, x, y)

    def bias(self, ngoal):
        """
        Add node biased towards goal
        
        Args:
            ngoal: Goal position tuple (x, y)
            
        Returns:
            Tuple (x, y, parent) - current tree state
        """
        n = self.number_of_nodes()
        self.add_node(n, ngoal[0], ngoal[1])
        nnear = self.nearest(n)
        self.step(nnear, n)
        self.connect(nnear, n)
        return self.x, self.y, self.parent

    def expand(self):
        """
        Add random node to tree (standard RRT expansion)
        
        Returns:
            Tuple (x, y, parent) - current tree state
        """
        while True:  
            n = self.number_of_nodes()
            x, y = self.sample_envir()
            self.add_node(n, x, y)
            if self.isFree():  
                xnearest = self.nearest(n)
                self.step(xnearest, n)
                self.connect(xnearest, n)
                return self.x, self.y, self.parent

    def path_to_goal(self):
        """
        Check if goal is reached and build path from start to goal
        
        Returns:
            True if goal reached, False otherwise
        """
        if self.goalFlag:
            self.path = []
            self.path.append(self.goalstate)
            newpos = self.parent[self.goalstate]
            # Trace back from goal to start
            while newpos != 0:
                self.path.append(newpos)
                newpos = self.parent[newpos]
            self.path.append(0)
        return self.goalFlag

    def getPathCoords(self):
        """
        Convert path node indices to coordinates
        
        Returns:
            List of (x, y) tuples representing path from goal to start
        """
        pathCoords = []
        for node in self.path:
            x, y = self.x[node], self.y[node]
            pathCoords.append((x, y))
        return pathCoords


def draw_new_node(map, graph, X, Y, Parent):
    """
    Draw newly added node and edge to visualization
    
    Args:
        map: RRTMap instance
        graph: RRTGraph instance
        X: List of x coordinates
        Y: List of y coordinates
        Parent: List of parent indices
    """
    pygame.draw.circle(map.map, map.grey, (X[-1], Y[-1]), map.nodeRad * 2, 0)
    pygame.draw.line(map.map, map.Blue, (X[-1], Y[-1]), 
                    (X[Parent[-1]], Y[Parent[-1]]), map.edgeThickness)



