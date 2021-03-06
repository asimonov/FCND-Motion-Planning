from enum import Enum
from queue import PriorityQueue
import math

import numpy as np

from bresenham import bresenham

from scipy.spatial import Voronoi
import networkx as nx # need version 2.1
import numpy.linalg as LA


def create_grid(data, drone_altitude, safety_distance):
    """
    Returns a grid representation of a 2D configuration space
    based on given obstacle data, drone altitude and safety distance
    arguments.
    """

    # minimum and maximum north coordinates
    north_min = np.floor(np.min(data[:, 0] - data[:, 3]))
    north_max = np.ceil(np.max(data[:, 0] + data[:, 3]))

    # minimum and maximum east coordinates
    east_min = np.floor(np.min(data[:, 1] - data[:, 4]))
    east_max = np.ceil(np.max(data[:, 1] + data[:, 4]))

    # given the minimum and maximum coordinates we can
    # calculate the size of the grid.
    north_size = int(np.ceil(north_max - north_min))
    east_size = int(np.ceil(east_max - east_min))

    # Initialize an empty grid
    grid = np.zeros((north_size, east_size))

    # Populate the grid with obstacles
    for i in range(data.shape[0]):
        north, east, alt, d_north, d_east, d_alt = data[i, :]
        if alt + d_alt + safety_distance > drone_altitude:
            obstacle = [
                int(np.clip(north - d_north - safety_distance - north_min, 0, north_size-1)),
                int(np.clip(north + d_north + safety_distance - north_min, 0, north_size-1)),
                int(np.clip(east - d_east - safety_distance - east_min, 0, east_size-1)),
                int(np.clip(east + d_east + safety_distance - east_min, 0, east_size-1)),
            ]
            grid[obstacle[0]:obstacle[1]+1, obstacle[2]:obstacle[3]+1] = 1

    return grid, int(north_min), int(east_min)






####### A* on a grid ***************************

# Assume all actions cost the same.
class Action(Enum):
    """
    An action is represented by a 3 element tuple.

    The first 2 values are the delta of the action relative
    to the current grid position. The third and final value
    is the cost of performing the action.
    """

    WEST = (0, -1, 1.0)
    EAST = (0, 1, 1.0)
    NORTH = (-1, 0, 1.0)
    SOUTH = (1, 0, 1.0)
    NORTH_EAST = (1, 1, math.sqrt(2))
    SOUTH_EAST = (-1, 1, math.sqrt(2))
    SOUTH_WEST = (-1, -1, math.sqrt(2))
    NORTH_WEST = (1, -1, math.sqrt(2))

    #@property
    def cost(self):
        return self.value[2]

    #@property
    #def delta(self):
    #    return (self.value[0], self.value[1])

    def delta1(self):
        return self.value[0]
    def delta2(self):
        return self.value[1]


def valid_actions(grid, current_node):
    """
    Returns a list of valid actions given a grid and current node.
    """
    valid_actions = set(Action)
    n, m = grid.shape[0] - 1, grid.shape[1] - 1
    x, y = current_node

    # check if the node is off the grid or
    # it's an obstacle

    if x - 1 < 0 or grid[x - 1, y] == 1:
        valid_actions.discard(Action.NORTH)
        valid_actions.discard(Action.NORTH_EAST)
        valid_actions.discard(Action.NORTH_WEST)
    if x + 1 > n or grid[x + 1, y] == 1:
        valid_actions.discard(Action.SOUTH)
        valid_actions.discard(Action.SOUTH_EAST)
        valid_actions.discard(Action.SOUTH_WEST)
    if y - 1 < 0 or grid[x, y - 1] == 1:
        valid_actions.discard(Action.WEST)
        valid_actions.discard(Action.NORTH_WEST)
        valid_actions.discard(Action.SOUTH_WEST)
    if y + 1 > m or grid[x, y + 1] == 1:
        valid_actions.discard(Action.EAST)
        valid_actions.discard(Action.NORTH_EAST)
        valid_actions.discard(Action.SOUTH_EAST)

    return valid_actions


def a_star(grid, h, start, goal):
    """
    Given a grid and heuristic function returns
    the lowest cost path from start to goal.
    """

    path = []
    path_cost = 0.0
    queue = PriorityQueue()
    queue.put((0.0, start))
    visited = set(start)

    branch = {}
    found = False

    while not queue.empty():
        item = queue.get()
        current_cost = item[0]
        current_node = item[1]

        if current_node == goal:
            print('Found a path.')
            found = True
            break
        else:
            # Get the new vertexes connected to the current vertex
            for a in valid_actions(grid, current_node):
                next_node = (current_node[0] + a.delta1(), current_node[1] + a.delta2())
                new_cost = current_cost + a.cost() + h(next_node, goal)

                if next_node not in visited:
                    visited.add(next_node)
                    queue.put((new_cost, next_node))

                    branch[next_node] = (new_cost, current_node, a)

    if found:
        # retrace steps
        n = goal
        path_cost = branch[n][0]
        path.append(goal)
        while branch[n][1] != start:
            path.append(branch[n][1])
            n = branch[n][1]
        path.append(branch[n][1])
    else:
        print('**********************')
        print('Failed to find a path!')
        print('**********************') 
    return path[::-1], path_cost



def heuristic(position, goal_position):
    #h = np.linalg.norm(np.array(position) - np.array(goal_position))
    h = math.sqrt( (position[0]-goal_position[0])**2 + (position[1]-goal_position[1])**2 )
    return h





####### path pruning ***************************


def collinearity_2D(p1, p2, p3, epsilon=1e-6):
    collinear = False
    det = p1[0]*(p2[1] - p3[1]) + p2[0]*(p3[1] - p1[1]) + p3[0]*(p1[1] - p2[1])
    if abs(det) < epsilon:
        collinear = True
    return collinear


def prune_path(path):
    pruned_path = [p for p in path]
    i = 0
    while i < len(pruned_path) - 2:
        p1 = (pruned_path[i])
        p2 = (pruned_path[i + 1])
        p3 = (pruned_path[i + 2])

        # If the 3 points are in a line remove the 2nd point.
        # The 3rd point now becomes and 2nd point
        # and the check is redone with a new third point
        # on the next iteration.
        if collinearity_2D(p1, p2, p3):
            # Something subtle here but we can mutate
            # `pruned_path` freely because the length
            # of the list is check on every iteration.
            pruned_path.remove(pruned_path[i + 1])
        else:
            i += 1
    return pruned_path


def prune_path_bres(path, grid):
    """
    Prune grid path using Bresenham ray-tracing method.
    Requires grid to check if intermediate paths are obstacle-free

    :param path:
    :param grid:
    :return:
    """
    pruned_path = [path.pop(0)]
    p2 = path.pop(0)
    for i, p3 in enumerate(path):
        p1 = pruned_path[-1]
        for p in list(bresenham(p1[0], p1[1], p3[0], p3[1])):
            if grid[p[0], p[1]] > 0.0:
                # collision from p1 to p3
                # use p2 as last point working well
                pruned_path.append(p2)
                p2 = p
                break

        if i == len(path):
            pruned_path.append(p3)
            break

        # p1 to p3 has no collisions, so remove p2
        p2 = p3

    pruned_path.append(path[-1])

    return pruned_path







###### A* on a GRAPH ############################

def create_grid_and_edges(data, drone_altitude, safety_distance):
    """
    Returns a grid representation of a 2D configuration space
    along with Voronoi graph edges given obstacle data and the
    drone's altitude.
    """
    # minimum and maximum north coordinates
    north_min = np.floor(np.min(data[:, 0] - data[:, 3]))
    north_max = np.ceil(np.max(data[:, 0] + data[:, 3]))

    # minimum and maximum east coordinates
    east_min = np.floor(np.min(data[:, 1] - data[:, 4]))
    east_max = np.ceil(np.max(data[:, 1] + data[:, 4]))

    # given the minimum and maximum coordinates we can
    # calculate the size of the grid.
    north_size = int(np.ceil(north_max - north_min))
    east_size = int(np.ceil(east_max - east_min))

    # Initialize an empty grid
    grid = np.zeros((north_size, east_size))
    # Initialize an empty list for Voronoi points
    points = []
    # Populate the grid with obstacles
    for i in range(data.shape[0]):
        north, east, alt, d_north, d_east, d_alt = data[i, :]
        if alt + d_alt + safety_distance > drone_altitude:
            obstacle = [
                int(np.clip(north - d_north - safety_distance - north_min, 0, north_size - 1)),
                int(np.clip(north + d_north + safety_distance - north_min, 0, north_size - 1)),
                int(np.clip(east - d_east - safety_distance - east_min, 0, east_size - 1)),
                int(np.clip(east + d_east + safety_distance - east_min, 0, east_size - 1)),
            ]
            grid[obstacle[0]:obstacle[1] + 1, obstacle[2]:obstacle[3] + 1] = 1

            # add center of obstacles to points list
            points.append([north - north_min, east - east_min])

    # create a voronoi graph based on location of obstacle centres
    graph = Voronoi(points)
    print('voronoi stats: {} landmarks, {} ridge_vertices'.format(len(graph.points), len(graph.ridge_vertices)))

    # check each edge from graph.ridge_vertices for collision
    edges = []
    for v in graph.ridge_vertices:
        p1 = graph.vertices[v[0]]
        p2 = graph.vertices[v[1]]

        cells = list(bresenham(int(p1[0]), int(p1[1]), int(p2[0]), int(p2[1])))
        hit = False

        for c in cells:
            # First check if we're off the map
            if np.amin(c) < 0 or c[0] >= grid.shape[0] or c[1] >= grid.shape[1]:
                hit = True
                break
            # Next check if we're in collision
            if grid[c[0], c[1]] == 1:
                hit = True
                break

        # If the edge does not hit on obstacle
        # add it to the list
        if not hit:
            # array to tuple for future graph creation step)
            p1 = (p1[0], p1[1])
            p2 = (p2[0], p2[1])
            edges.append((p1, p2))

    print('voronoi stats: {} clear ridges'.format(len(edges)))

    return grid, edges

def closest_point(graph, current_point):
    """
    Compute the closest point in the `graph`
    to the `current_point`.
    """
    closest_point = None
    dist = 100000
    for p in graph.nodes:
        d = LA.norm(np.array(p) - np.array(current_point))
        if d < dist:
            closest_point = p
            dist = d
    return closest_point

def heuristic_npla(n1, n2):
    return LA.norm(np.array(n2) - np.array(n1))


def a_star_graph(map_data, drone_altitude, safety_distance, start, goal):
    """Modified A* to work with NetworkX graphs.
       Hardcoding metric norm heuristic
    """
    # convert data to grid and then find Voronoi edges assuming all colliders are points
    grid, edges = create_grid_and_edges(map_data, drone_altitude, safety_distance)
    # create the graph with the weight of the edges set to the Euclidean distance between the points
    graph = nx.Graph()
    for e in edges:
        p1 = e[0]
        p2 = e[1]
        dist = LA.norm(np.array(p2) - np.array(p1))
        graph.add_edge(p1, p2, weight=dist)

    start_ne_g = closest_point(graph, start)
    goal_ne_g = closest_point(graph, goal)
    print('graph start: {}'.format(start_ne_g))
    print('graph goal: {}'.format(goal))

    queue = PriorityQueue()
    queue.put((0, start_ne_g))
    visited = set(start_ne_g)

    branch = {}
    found = False

    while not queue.empty():
        item = queue.get()
        current_cost = item[0]
        current_node = item[1]

        if current_node == goal_ne_g:
            print('Found a path.')
            found = True
            break
        else:
            for next_node in graph[current_node]:
                cost = graph.edges[current_node, next_node]['weight']
                new_cost = current_cost + cost + heuristic_npla(next_node, goal_ne_g)

                if next_node not in visited:
                    visited.add(next_node)
                    queue.put((new_cost, next_node))

                    branch[next_node] = (new_cost, current_node)

    path = []
    path_cost = 0
    if found:
        # retrace steps
        path = []
        n = goal_ne_g
        path_cost = branch[n][0]
        path.append(goal_ne_g)
        while branch[n][1] != start_ne_g:
            path.append(branch[n][1])
            n = branch[n][1]
        path.append(branch[n][1])

    print('path length: {}'.format(len(path)))

    final_path = [start,] + path[::-1] + [goal,]
    return final_path, path_cost






###### RRT ############################


from RRT import RRT

def a_star_rrt(grid, start, goal, prob_goal=0.3, max_steps=10):
    """Modified A* on RRT.
       Shoots in goal direction with probability prob_goal.
       Otherwise chooses random place in the grid to go to.
       Then steps from currently closes RRT point in that direction max_steps or until hits obstacle.
       Stops when reached goal.
       Uses A* machinery to retrace path.
       Heuristic for shooting to goal and for 'nearest' tree branch: metric norm.
    """

    rrt = RRT(start)
    TIMEOUT = 10000000
    while TIMEOUT > 0:
        # get random state (goal with specified prob, otherwise totally random in free space)
        x_rand = rrt.sample_state(grid, goal, prob_goal=prob_goal)
        # find closest node in the tree to the one we sampled
        x_near = rrt.nearest_neighbor(x_rand)
        # step from closest node in the tree in direction of random state until we hit obstacle
        # or step MAX_STEPS steps
        cells = list(bresenham(int(x_near[0]), int(x_near[1]), int(x_rand[0]), int(x_rand[1])))
        hit = False
        i = 0
        for c in cells:
            # First check if we're off the map
            if np.amin(c) < 0 or c[0] >= grid.shape[0] or c[1] >= grid.shape[1]:
                hit = True
                break
            # Next check if we're in collision
            if grid[c[0], c[1]] == 1:
                hit = True
                break
            i += 1
            # Next check if we've reached the goal
            if c[0] == goal[0] and c[1] == goal[1]:
                break
            if i==max_steps:
                break

        x_next = cells[i-1]
        if i>0:
            rrt.add_vertex(x_next)
            rrt.add_edge(x_near, x_next)

        if x_next[0] == goal[0] and x_next[1] == goal[1]:
            break
        TIMEOUT -= 1
        if (TIMEOUT % 1000 == 0):
            print("+1000 expansions of RRT. number of RRT nodes: {}".format(len(rrt.tree.nodes)))

    if TIMEOUT == 0:
        raise Exception("timeout planning using RRT")


    graph = rrt.tree

    start_ne_g = closest_point(graph, start)
    goal_ne_g = closest_point(graph, goal)
    print('graph start: {}'.format(start_ne_g))
    print('graph goal: {}'.format(goal))

    queue = PriorityQueue()
    queue.put((0, start_ne_g))
    visited = set(start_ne_g)

    branch = {}
    found = False

    while not queue.empty():
        item = queue.get()
        current_cost = item[0]
        current_node = item[1]

        if current_node == goal_ne_g:
            print('Found a path.')
            found = True
            break
        else:
            for next_node in graph[current_node]:
                cost = graph.edges[current_node, next_node]['weight']
                new_cost = current_cost + cost + heuristic_npla(next_node, goal_ne_g)

                if next_node not in visited:
                    visited.add(next_node)
                    queue.put((new_cost, next_node))

                    branch[next_node] = (new_cost, current_node)

    path = []
    path_cost = 0
    if found:
        # retrace steps
        path = []
        n = goal_ne_g
        path_cost = branch[n][0]
        path.append(goal_ne_g)
        while branch[n][1] != start_ne_g:
            path.append(branch[n][1])
            n = branch[n][1]
        path.append(branch[n][1])

    print('path length: {}'.format(len(path)))

    final_path = [start,] + path[::-1] + [goal,]
    return final_path, path_cost





