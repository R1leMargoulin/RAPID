import math
import numpy as np
import heapq
import random
from dataclasses import dataclass
from scipy.ndimage import sobel
from sklearn.cluster import KMeans
from collections import deque

from .grid_variables import *



@dataclass(eq=False)
class Transform2d:
    """
    2D transform regrouping :
    - x : position of the entity on the x axis
    - y : position of the entity on the y axis
    - w : yaw -> rotation of the entity on the z axis
    """
    x: float = 0.0
    y: float = 0.0
    w: float = 0.0

DIRECTIONS = [(-1, 0), (1, 0), (0, -1), (0, 1)]

def dijkstra(occupancy_grid:np.ndarray, target_coord:tuple[int,int]):
    rows, cols = occupancy_grid.shape
    distances = np.full((rows, cols), np.inf)

    queue = deque([target_coord])
    distances[target_coord] = 0

    while queue:
        current = queue.popleft()
        current_distance = distances[current]

        for direction in DIRECTIONS:
            neighbor = (current[0] + direction[0], current[1] + direction[1])

            if 0 <= neighbor[0] < rows and 0 <= neighbor[1] < cols:
                if occupancy_grid[neighbor] == 0:
                    new_distance = current_distance + 1
                    if new_distance < distances[neighbor]:
                        distances[neighbor] = new_distance
                        queue.append(neighbor)

    return distances

djikstra = dijkstra

def find_frontier_cells(grid, traversable_types = (OG_FREE_CELL,)):
    """
    Find frontier cells: traversable cells with at least one unknown 4-neighbor.
    Returns an (n, 2) array in row-major order.
    """
    traversable = np.isin(grid, traversable_types)
    unknown = grid == OG_UNKNOWN_CELL
    next_to_unknown = np.zeros_like(unknown)
    next_to_unknown[1:, :] |= unknown[:-1, :]
    next_to_unknown[:-1, :] |= unknown[1:, :]
    next_to_unknown[:, 1:] |= unknown[:, :-1]
    next_to_unknown[:, :-1] |= unknown[:, 1:]
    return np.column_stack(np.where(traversable & next_to_unknown))

def sobel_frontier_detection(grid, traversable_types = (OG_FREE_CELL,)):
    explo_grid = grid==-1 #returns true if free and false if obstacle
    sobel_h = sobel(explo_grid, 0)  # horizontal gradient
    sobel_v = sobel(explo_grid, 1)  # vertical gradient
    magnitude = np.sqrt(sobel_h**2 + sobel_v**2)
    #magnitude *= 255.0 / np.max(magnitude)  # normalization

    frontiers_candidates = np.column_stack(np.where(magnitude>0))
    frontiers = []

    for x, y in frontiers_candidates:
        if not(grid[x,y] in traversable_types):
            continue
        else:
            frontiers.append((int(x),int(y)))

    return frontiers

def find_nearest_free(grid, target, neighborhood='moore', traversable_types = (OG_FREE_CELL,)):
    """
    Trouve la case libre la plus proche du point cible via BFS.
    
    Args:
        grid       : np.array 2D
        target     : tuple (x, y) — point cible
        neighborhood: 'moore' (8 voisins) ou 'vonneumann' (4 voisins)
        free_value : valeur d'une case libre (défaut 0)
    
    Returns:
        (x, y) de la case libre la plus proche, ou None si aucune trouvée
    """
    
    if neighborhood == 'moore':
        directions = [(-1,-1),(-1, 0),(-1, 1),
                      ( 0,-1),         ( 0, 1),
                      ( 1,-1),( 1, 0),( 1, 1)]
    else:  # von neumann
        directions = [(-1, 0),
                      ( 0,-1), (0, 1),
                      ( 1, 0)]

    rows, cols = grid.shape
    visited = set()
    queue = deque([target])
    visited.add(target)

    while queue:
        x, y = queue.popleft()

        # Si la case courante est libre (et ce n'est pas la cible elle-même)
        if grid[x, y] in traversable_types and (x, y) != target:
            return (x, y)

        # Sinon, on explore ses voisins
        for dx, dy in directions:
            nx, ny = x + dx, y + dy
            if (0 <= nx < rows and 0 <= ny < cols
                    and (nx, ny) not in visited):
                visited.add((nx, ny))
                queue.append((nx, ny))

    return None  # aucune case libre trouvée


def cluster_frontier_cells(grid, frontier_cells, vision_range, traversable_types = (OG_FREE_CELL,)):
    """
    Cluster frontier cells into groups considering walls and vision range.

    Parameters:
    - grid: 2D numpy array representing the grid.
    - frontier_cells: List of frontier cell coordinates.
    - vision_range: The vision range of the robot.

    Returns:
    - cluster_centers: List of cluster center coordinates.
    """
    cells = np.asarray(frontier_cells)
    traversable = np.isin(grid, traversable_types)

    def distances_from(index):
        return np.sqrt(((cells - cells[index]) ** 2).sum(1))

    def is_path_clear(start, end):
        """Check if there is a clear path between start and end cells."""
        num = max(abs(start[0]-end[0]), abs(start[1]-end[1])) + 1
        rows = np.linspace(start[0], end[0], num=num, dtype=int)
        cols = np.linspace(start[1], end[1], num=num, dtype=int)
        return traversable[rows, cols].all()

    def form_clusters():
        """Form clusters based on vision range and walls."""
        clusters = []
        visited = np.zeros(len(cells), dtype=bool)

        for first in range(len(cells)):
            if visited[first]:
                continue
            cluster = [first]
            visited[first] = True
            stack = [first]

            while stack:
                current = stack.pop()
                candidates = np.flatnonzero((distances_from(current) <= vision_range) & ~visited)
                for other in candidates:
                    if visited[other]:
                        continue
                    if is_path_clear(cells[current], cells[other]) and (distances_from(other)[cluster] <= vision_range).all():
                        cluster.append(other)
                        visited[other] = True
                        stack.append(other)

            clusters.append(cells[cluster])

        return clusters

    clusters = form_clusters()
    cluster_centers = []
    for i in range(len(clusters)) :
        cluster_center = np.round(np.mean(clusters[i], axis=0))
        if not(grid[int(cluster_center[0]), int(cluster_center[1])] in(traversable_types)): # barycenter inside an obstacle: use a random frontier cell instead
            index = random.randint(0, len(clusters[i])-1 )
            cluster_centers.append(clusters[i][index])
        else:
            cluster_centers.append(cluster_center)

    return np.round(cluster_centers)

def wavefront_propagation_algorithm(grid, self_position, robot_positions, frontier_clusters, weight_of_closer_robots = 10, traversable_types = (OG_FREE_CELL,)):
    """
    Perform wavefront propagation from frontiers clusters (also works with simple frontiers) to determine their score depending on it's distance and the robots closer to the one computing this algorithm.

    parameters:
    - grid: 2D numpy array representing the grid.
    - self_position:(int,int) = position xy of the robot computing this algorithm.
    - robot_positions: List of robot positions (row, col).
    - frontier_clusters: List of frontier cluster centers (row, col).
    - weight_of_closer_robots:int(default 10) = degree of penalty on a frontier score caused by closer robot on a frontier

    returns:
    - frontier_scores: Dictionary with frontier cluster centers as keys and scores as values.
    """
    def propagate(start_pos):
        """Propagate the wavefront from the start position."""
        width, height = grid.shape
        wavefront_map = np.full_like(grid, -1, dtype=int)  #this keeps tracks of explored cells by the WPA, in order to avoid multiple calculations for a cell.
        wavefront_map[start_pos] = 0
        next_queue = [start_pos]
        queue = []

        robots_touched = 0 #keeps tracks of the closer robots to the frontier
        frontier_distance_score = 0 #keeps track of the distance from the frontier to the robot doing this calculation
        reached_robot = False #propagation happens until a robot is reached

        # print(next_queue)

        while not reached_robot:
            
            if(len(queue) == 0):
                queue = next_queue
                next_queue = []
                if len(queue) == 0:
                    return np.inf, robots_touched
            #add 1 distance at each propagations
            frontier_distance_score += 1

            for i in queue:
                current = queue.pop(0)
                current_value = wavefront_map[current]
                neighbours = get_direct_neighbors(current, width, height)

                for n in neighbours:
                    if 0 <= n[0] < width and 0 <= n[1] < height: #verif that the neighbor is inbound
                        #If the neighbor is correct, we add the neighbors to the queue and we add 1 to the distance metric
                        if wavefront_map[n[0], n[1]] == -1:  # Unvisited cell on the wavefront map
                            if grid[n[0], n[1]] in traversable_types or grid[n[0], n[1]] == OG_UNKNOWN_CELL:  # Free cell in real env
                                wavefront_map[n[0], n[1]] = current_value + 1
                                next_queue.append((n[0], n[1])) #we append the correct neighbour to the next queue.
                            else:
                                wavefront_map[n[0], n[1]] = current_value + 1 #we update the wavefront map but not append the wall to the next_queue

                            #print(f"{(n[0], n[1])}//{self_position}") #TODO : trouver pourquoi ca y est jamais
                            if (n[0], n[1]) == self_position: # Stop if the wavefront reaches the "main" robot (the one doing the calculations)
                                # print("found")
                                reached_robot = True
                            #if a robot is touched by the propagation, we add it's coordinates to the list
                            elif (n[0], n[1]) in robot_positions:  # Robot cell
                                robots_touched += 1

        return frontier_distance_score, robots_touched

    frontier_scores = {}

        #
    

    for frontier in frontier_clusters:
        fx, fy = int(frontier[0]), int(frontier[1])
        frontier_distance_score, robots_touched = propagate((fx, fy))
        
        frontier_scores[(fx, fy)] = frontier_distance_score + robots_touched * weight_of_closer_robots

    return frontier_scores

def heuristic(a, b):
    """
    Manhattan distance
    This heuristic estimates the cost to reach the goal from a given point.
    """
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def a_star_search(grid, start, goal, traversable_types = (OG_FREE_CELL,)):
    """
    A star search algorithm\\
    parameters:
    - grid: 2D numpy array representing the occupancy grid.
    - start: (x,y) representing the starting cell.
    - goal: (x,y) representing the target cell.

    returns:
    - A list of tuples representing the path from the start to the goal, or None if no path is found.
    """
    rows, cols = grid.shape
    passable_types = set(traversable_types) | {OG_UNKNOWN_CELL}
    open_set = [(0, start)]
    came_from = {}
    g_score = {start: 0}

    while open_set:
        current = heapq.heappop(open_set)[1]

        if current == goal:
            return reconstruct_path(came_from, current)

        for neighbor in get_direct_neighbors(current, rows, cols):
            tentative_g_score = g_score[current] + 1

            if grid[neighbor] not in passable_types:
                continue

            if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                heapq.heappush(open_set, (tentative_g_score + heuristic(neighbor, goal), neighbor))

    return None


def a_star_cost(grid, start, goal, env_ease, traversable_types=(OG_FREE_CELL,)):
    """
    this function will calculate a a* cost from a goal point, and will then return the cost to go to this point
    """
    path = a_star_search(grid, start, goal, traversable_types)
    if path:
        costs = []
        for p in path:
            pvalue = int(grid[p])
            if pvalue != -1:
                current_cell_type_name = list(ENV_CELL_TYPES.keys())[list(ENV_CELL_TYPES.values()).index(pvalue)] #return the string name of the env type
                costs.append( 1/(env_ease[current_cell_type_name]+1e-8) ) #we make a cost for the cell only
            else:
                costs.append(1) # unknown cell costs 1, permitting exploration
        cost = np.sum(costs)
    else:
        cost = np.inf
    return cost
 

def get_direct_neighbors(cell, width, height):
    """
    Get the valid neighbors of a cell within the grid bounds.

    parameters:
    - cell: A tuple (row, col) representing the current cell.
    - width of the grid.
    - height of the grid.

    returns:
    - A list of tuples representing the valid neighbors cells.
    """
    neighbors = []

    for direction in DIRECTIONS:
        neighbor = (cell[0] + direction[0], cell[1] + direction[1])
        if 0 <= neighbor[0] < width and 0 <= neighbor[1] < height:
            neighbors.append(neighbor)

    return neighbors

def reconstruct_path(came_from, current):
    """
    Reconstruct the path from the goal to the start using the came_from dictionary.

    parameters:
    - came_from: dictionary mapping each cell to its predecessor.
    - current: tuple(x,y) representing the current cell (goal).

    returns:
    - A list of tuples representing the path from the start to the goal.
    """
    total_path = [current]
    while current in came_from:
        current = came_from[current]
        total_path.append(current)
    return total_path[::-1]  # Return the reversed path

def euclidian_distance(point1,point2):
    """
    give the euclidian distance between 2 points\\
    params:
    - point1:(float,float) : x cand y coordinates of point 1
    - point2:(float,float) : x cand y coordinates of point 2

    return : 
    - euclidian_distance:float
    """
    dx = point1[0]-point2[0]
    dy = point1[1]-point2[1]
    if isinstance(dx, np.ndarray) or isinstance(dy, np.ndarray):
        return np.sqrt(dx*dx+dy*dy)
    return math.sqrt(dx*dx+dy*dy)

manhathan_distance = heuristic


def heuristic_frontier_distance(start, goal, grid, traversable_types = (OG_FREE_CELL,)):
    """
    Calculate a heuristic distance by considering obstacles.

    Parameters:
    - start: Tuple (x, y) representing the start cell.
    - goal: Tuple (x, y) representing the goal cell.
    - grid: 2D numpy array representing the occupancy grid.

    Returns:
    - Float representing the heuristic distance.
    """
    # Calculate Euclidean distance
    euclidean_dist = euclidian_distance(start, goal)

    # Calculate a simple obstacle penalty
    line = np.linspace(start, goal, num=int(euclidean_dist) + 1)
    line = [(int(x), int(y)) for x, y in line]
    obstacle_penalty = sum(1 for x, y in line if not(grid[int(x), int(y)] in traversable_types))

    # Combine Euclidean distance and obstacle penalty
    return euclidean_dist + obstacle_penalty * 100  # Weight for obstacle penalty

def simple_clustering(coordinates, max_distance):
    """
    Give a Simple and fast non optimal clustering of given (x,y) coordinates list made with maximal distance

    parameters:
    - coordinates : list[(float, float)] = list of (x,y) of the points
    - max_distance : float = max size of the clusters in term of distance

    Return : Barycenter of the clusters
    """
    coords = np.array(coordinates)
    unvisited = set(range(len(coords)))
    clusters = []

    while unvisited:
        # Commencer avec un point non visité
        start_point = next(iter(unvisited))
        unvisited.remove(start_point)

        # Trouver tous les points à une distance inférieure ou égale à max_distance
        queue = deque([start_point])
        cluster = []

        while queue:
            point_idx = queue.popleft()
            cluster.append(point_idx)

            for unvisited_point in list(unvisited):
                if euclidian_distance(coords[point_idx], coords[unvisited_point]) <= max_distance:
                    unvisited.remove(unvisited_point)
                    queue.append(unvisited_point)

        clusters.append(cluster)

    # Calculer les barycentres pour chaque cluster
    barycenters = []
    for cluster in clusters:
        if cluster:
            barycenter = np.mean(coords[cluster], axis=0)
            barycenters.append(barycenter.tolist())

    return barycenters


def kmeans_cluster_frontiers(frontiers, robots):
    npfrontiers = np.array(frontiers)
    kmeans = KMeans(n_clusters=len(robots), random_state=0, n_init="auto").fit(npfrontiers)

    return kmeans.cluster_centers_, kmeans.labels_


def max_k(list, k):
    "gives the k-th max element of a list"
    if k <= len(list):
        partitioned = np.partition(list, -k)[-k:]
        xth_max = partitioned[0]

        return xth_max 
    else:
        return 0