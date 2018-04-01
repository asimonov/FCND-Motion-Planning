import numpy as np
import time

filename = 'colliders.csv'
data = np.loadtxt(filename,delimiter=',',dtype='Float64',skiprows=2)

from planning_utils import a_star, heuristic, create_grid, Action

start_ne = (316, 445)
goal_ne = (539, 138)

drone_altitude = 5
safe_distance = 5

grid, north_offset, east_offset = create_grid(data, drone_altitude, safe_distance)


st = time.time()

path, cost = a_star(grid, heuristic, start_ne, goal_ne)

print(time.time()-st)
print(len(path))

