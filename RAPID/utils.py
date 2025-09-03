import numpy as np
import heapq
import random

from .grid_variables import *



class Transform2d():
        """
        2D transform regrouping :
        - x : position of the entity on the x axis
        - y : position of the entity on the y axis
        - w : yaw -> rotation of the entity on the z axis
        """
        def __init__(self, x:float = 0.0, y:float = 0.0, w:float = 0.0):
                self.x = x
                self.y = y
                self.w = w

DIRECTIONS = [(-1, 0), (1, 0), (0, -1), (0, 1)]

def djikstra(occupancy_grid:np.ndarray, target_coord:tuple[int,int]):
    # Dimensions of the occupancy grid
    rows, cols = occupancy_grid.shape

    # Initialize the distance map with infinity (unreachable)
    djikstra = np.full((rows, cols), np.inf)

    # Queue for BFS, starting from the target coordinate
    queue = [target_coord]
    djikstra[target_coord] = 0

    while queue:
        current = queue.pop(0)  # Pop from the front of the list
        current_distance = djikstra[current]

        # Explore neighbors
        for direction in DIRECTIONS:
            neighbor = (current[0] + direction[0], current[1] + direction[1])

            # Check if the neighbor is within bounds and not an obstacle
            if 0 <= neighbor[0] < rows and 0 <= neighbor[1] < cols:
                if occupancy_grid[neighbor] == 0:
                    new_distance = current_distance + 1
                    if new_distance < djikstra[neighbor]:
                        djikstra[neighbor] = new_distance
                        queue.append(neighbor)

    return djikstra

def find_frontier_cells(grid, traversable_types = [OG_FREE_CELL]):
    """
    Find frontier cells in a (-1/0/1) array where:
    - -1 is an unknown cell
    - 0 is a free cell
    - 1 is an obstacle
    """
    # Get the dimensions of the grid
    width, height = grid.shape

    # Define shifts for neighbors
    shifts = [(-1, 0), (1, 0), (0, -1), (0, 1)]

    # Initialize a mask for frontier cells
    frontier_mask = np.zeros_like(grid, dtype=bool)

    # Iterate over each cell in the grid
    for r in range(width):
        for c in range(height):
            # Check if the current cell is known (0 or 1)
            #if grid[r, c] == 0 or grid[r, c] == 1:
            if grid[r, c] in traversable_types: #la case peut elle etre traversee?
                # Check the neighbors of the current cell
                for dr, dc in shifts:
                    nr, nc = r + dr, c + dc
                    # Check if the neighbor is within the grid bounds
                    if 0 <= nr < width and 0 <= nc < height:
                        # Check if the neighbor is unknown (-1)
                        if grid[nr, nc] == OG_UNKNOWN_CELL:
                            frontier_mask[r, c] = True
                            break  # No need to check other neighbors

    # Get the coordinates of frontier cells
    frontier_cells = np.column_stack(np.where(frontier_mask))

    return frontier_cells

def cluster_frontier_cells(grid, frontier_cells, vision_range, traversable_types = [OG_FREE_CELL]):
    """
    Cluster frontier cells into groups considering walls and vision range.

    Parameters:
    - grid: 2D numpy array representing the grid.
    - frontier_cells: List of frontier cell coordinates.
    - vision_range: The vision range of the robot.

    Returns:
    - cluster_centers: List of cluster center coordinates.
    """
    def is_within_range(cell1, cell2, vision_range):
        """Check if two cells are within the vision range."""
        return np.linalg.norm(np.array(cell1) - np.array(cell2)) <= vision_range

    def is_path_clear(grid, start, end):
        """Check if there is a clear path between start and end cells."""
        rr, cc = zip(start, end)
        cells_between = zip(np.linspace(rr[0], rr[1], num=max(abs(rr[0]-rr[1]), abs(cc[0]-cc[1]))+1, dtype=int),
                            np.linspace(cc[0], cc[1], num=max(abs(rr[0]-rr[1]), abs(cc[0]-cc[1]))+1, dtype=int))
        for cell in cells_between:
            if not (grid[cell] in traversable_types):
                return False
        return True

    def form_clusters(frontier_cells, vision_range):
        """Form clusters based on vision range and walls."""
        clusters = []
        visited = set()

        for cell in frontier_cells:
            if tuple(cell) not in visited:
                cluster = [cell]
                visited.add(tuple(cell))
                stack = [cell]

                while stack:
                    current_cell = stack.pop()
                    for other_cell in frontier_cells:
                        if tuple(other_cell) not in visited and is_within_range(current_cell, other_cell, vision_range):
                            # Check if there is a direct path not blocked by walls
                            if is_path_clear(grid, current_cell, other_cell):
                                # Ensure the new cell is within vision range of all cells in the cluster
                                if all(is_within_range(other_cell, c, vision_range) for c in cluster):
                                    cluster.append(other_cell)
                                    visited.add(tuple(other_cell))
                                    stack.append(other_cell)

                clusters.append(cluster)

        return clusters

    clusters = form_clusters(frontier_cells, vision_range)
    #cluster_centers = [np.mean(cluster, axis=0) for cluster in clusters]
    cluster_centers = []
    for i in range(len(clusters)) :
        cluster_center = np.round(np.mean(clusters[i], axis=0))
        if not(grid[int(cluster_center[0]), int(cluster_center[1])] in(traversable_types)): #si le baricentre est dans un obstacle, alors on prends juste une des case frontières.
            index = random.randint(0, len(clusters[i])-1 )
            cluster_centers.append(clusters[i][index])
        else:
            cluster_centers.append(cluster_center)

    return np.round(cluster_centers)

def wavefront_propagation_algorithm(grid, self_position, robot_positions, frontier_clusters, weight_of_closer_robots = 10, traversable_types = [OG_FREE_CELL]):
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

def a_star_search(grid, start, goal, traversable_types = [OG_FREE_CELL]):
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
    open_set = []

    # Push the start cell into the priority queue with a cost of 0
    heapq.heappush(open_set, (0, start))    
    came_from = {} # Dictionary to keep track of the path

    # Dictionary to store the cost of the cheapest path to each cell
    g_score = {start: 0}

    # Dictionary to store the estimated total cost to reach the goal from each cell
    f_score = {start: heuristic(start, goal)}

    while open_set:
        current = heapq.heappop(open_set)[1]  # Pop the cell with the lowest f_score value

        # If the goal is reached, reconstruct and return the path
        if current == goal:
            return reconstruct_path(came_from, current)

        # Iterate over the neighbors of the current cell
        for neighbor in get_direct_neighbors(current, rows, cols):
            # Calculate the tentative g_score for the neighbor
            tentative_g_score = g_score[current] + 1

            # Skip obstacle cells
            # if grid[neighbor] == 1 or grid[neighbor] == -1:
            if not (grid[neighbor] in traversable_types or grid[neighbor]==OG_UNKNOWN_CELL): #if it's an obstacle:
                continue

            # If the neighbor is not in g_score or the tentative g_score is lower, update the scores
            if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = tentative_g_score + heuristic(neighbor, goal)

                # Push the neighbor into the priority queue with the updated f_score
                heapq.heappush(open_set, (f_score[neighbor], neighbor))

    # If the open set is empty and the goal was not reached, return None
    return None


def a_star_cost(grid, start, goal, env_ease, traversable_types=[OG_FREE_CELL]):
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
                costs.append( 1/env_ease[current_cell_type_name] ) #we make a cost for the cell only
            else:
                p = costs.append(1) #p will be equal to 1 if we don't know it's value, permitting exploration
        #then we sum all the costs to get a final cost
        cost = np.sum(costs) #TODO check if that works
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
    directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]  # Up, Down, Left, Right

    for direction in directions:
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
    return np.sqrt((point1[0]-point2[0])**2+(point1[1]-point2[1])**2)

def heuristic_frontier_distance(start, goal, grid, traversable_types = [OG_FREE_CELL]):
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
    coords = np.array(coordinates)
    unvisited = set(range(len(coords)))
    clusters = []

    while unvisited:
        # Commencer avec un point non visité
        start_point = next(iter(unvisited))
        unvisited.remove(start_point)

        # Trouver tous les points à une distance inférieure ou égale à max_distance
        queue = [start_point]
        cluster = []

        while queue:
            point_idx = queue.pop(0)
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