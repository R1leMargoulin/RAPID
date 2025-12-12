ENV_CELL_TYPES = {} #made for linking the env_cell_type name to it's value

#variables for occupancy grid
OG_FREE_CELL = 0
OG_FREE_CELL_GROUP_NAME = "free"
ENV_CELL_TYPES[OG_FREE_CELL_GROUP_NAME] = OG_FREE_CELL


OG_WALL = 1 #drone can pass -- black
OG_WALL_GROUP_NAME = "obstacles"
ENV_CELL_TYPES[OG_WALL_GROUP_NAME] = OG_WALL


OG_HIGH_WALL = 11 #drone cannot pass -- red
OG_HIGH_WALL_GROUP_NAME = "high_obstacles"
ENV_CELL_TYPES[OG_HIGH_WALL_GROUP_NAME] = OG_HIGH_WALL

OG_WATER = 2 #-- blue
OG_WATER_GROUP_NAME = "water"
ENV_CELL_TYPES[OG_WATER_GROUP_NAME] = OG_WATER

OG_SAND = 3 # -- yellow
OG_SAND_GROUP_NAME = "sand"
ENV_CELL_TYPES[OG_SAND_GROUP_NAME] = OG_SAND

OG_GRASS = 4 # -- green
OG_GRASS_GROUP_NAME = "grass"
ENV_CELL_TYPES[OG_GRASS_GROUP_NAME] = OG_GRASS


OG_TARGET_POINT = 100

OG_UNKNOWN_CELL = -1


BLOCKING_SENSOR_TYPES = [OG_WALL, OG_HIGH_WALL] #those kind of cell block sensors if it's an obstacle (for exemple, low wall won't block sensor for drone but will for ground robot.)