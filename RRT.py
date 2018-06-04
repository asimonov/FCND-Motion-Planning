import numpy as np
import networkx as nx
import numpy.linalg as LA
import random

class RRT:
    def __init__(self, x_init):
        # A tree is a special case of a graph with
        # directed edges and only one path to any vertex.
        self.tree = nx.Graph()
        self.tree.add_node(tuple(x_init))

    def add_vertex(self, x_new):
        self.tree.add_node(tuple(x_new))

    def add_edge(self, x_near, x_new):
        dist = LA.norm(np.array(x_near) - np.array(x_new))
        self.tree.add_edge(x_near, x_new, weight=dist)
        #print("old node {}, new node {}, dist {}".format(x_near, x_new, dist))

    @property
    def vertices(self):
        return self.tree.nodes

    @property
    def edges(self):
        return self.tree.edges

    def nearest_neighbor(self, x_rand):
        closest_dist = 100000
        closest_vertex = None
        x_rand = np.array(x_rand)

        for v in self.tree.nodes:
            d = LA.norm(x_rand - np.array(v[:2]))
            if d < closest_dist:
                closest_dist = d
                closest_vertex = v
        return closest_vertex

    def sample_state(self, grid, goal, prob_goal=0.2):
        if random.uniform(0,1) < prob_goal:
            return goal
        else:
            # return random state
            # sample states until a free state is found
            x = -1
            y = -1
            while True:
                x = int(np.random.uniform(0, grid.shape[0]))
                y = int(np.random.uniform(0, grid.shape[1]))
                if grid[int(x), int(y)] == 0 and (x,y) not in self.tree.nodes:
                    break
            return (x, y)



