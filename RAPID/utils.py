import numpy as np



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



def djikstra(occupancy_grid:np.ndarray, target_coord:tuple[int,int]):
    # Dimensions of the occupancy grid
    rows, cols = occupancy_grid.shape

    # Initialize the distance map with infinity (unreachable)
    djikstra = np.full((rows, cols), np.inf)

    # Queue for BFS, starting from the target coordinate
    queue = [target_coord]
    djikstra[target_coord] = 0

    # Directions for wave propagation (4-connected grid)
    directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]

    while queue:
        current = queue.pop(0)  # Pop from the front of the list
        current_distance = djikstra[current]

        # Explore neighbors
        for direction in directions:
            neighbor = (current[0] + direction[0], current[1] + direction[1])

            # Check if the neighbor is within bounds and not an obstacle
            if 0 <= neighbor[0] < rows and 0 <= neighbor[1] < cols:
                if occupancy_grid[neighbor] == 0:
                    new_distance = current_distance + 1
                    if new_distance < djikstra[neighbor]:
                        djikstra[neighbor] = new_distance
                        queue.append(neighbor)

    return djikstra
            


