import numpy as np
import heapq

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

def find_frontier_cells(grid):
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
            if grid[r, c] == OG_FREE_CELL: #test en enlevant les murs
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



def heuristic(a, b):
    """
    Manhattan distance
    This heuristic estimates the cost to reach the goal from a given point.
    """
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def a_star_search(grid, start, goal):
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
            if grid[neighbor] == OG_WALL:
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



def heuristic_frontier_distance(start, goal, grid):
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
    obstacle_penalty = sum(1 for x, y in line if grid[int(x), int(y)] == OG_WALL)

    # Combine Euclidean distance and obstacle penalty
    return euclidean_dist + obstacle_penalty * 100  # Weight for obstacle penalty