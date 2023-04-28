import math
import numpy as np
import random
from scipy.spatial import cKDTree
import matplotlib.pyplot as plt


# Class for each robot
class Robot:
    def __init__(self, rid):
        self.id = rid
        self.pos = []


# Class for each tree node
class Node:
    def __init__(self, row, col):
        self.row = row        # coordinate
        self.col = col        # coordinate
        self.parent = None    # parent node / edge
        self.cost = 0.0       # cost to parent / edge weight


# Class for RRT
class ODR:
    # Constructor
    def __init__(self, payload_verts, start, goal):
        self.payload = payload_verts          # array of tuples of payload vertices, 1->free, 0->obstacle

        self.start = Node(start[0], start[1]) # start node
        self.goal = Node(goal[0], goal[1])    # goal node
        self.vertices = []                    # list of nodes
        self.found = False                    # found flag


    def init_robots(self, n_robots):
        '''Initialize the robots before each search
        '''
        self.n_robots = n_robots
        self.robot = []
        for i in range(n_robots):
            self.robot.append(Robot(i))
            print('Robot', i, 'initialized')
        

    def init_map(self):
        '''Intialize the map before each search
        '''
        self.found = False
        self.vertices = []
        self.vertices.append(self.start)


    # def dis(self, node1, node2):
    #     '''Calculate the euclidean distance between two nodes
    #     '''
    #     # return np.sqrt((node1.row-node2.row)**2 + (node1.col-node2.col)**2)
    #     return np.sqrt((node1[0]-node2[0])**2 + (node1[1]-node2[1])**2).round(2)


    def get_nearest_node(self, point):
        '''Find the nearest node from the new point
        '''
        # Use kdtree to find k-nearest neighbors ***.COPY() SO WE DONT REMOVE FROM ORIGINAL LIST
        samples = self.payload.copy()
        # Remove this point from the list of samples
        samples.remove(point)

        kdtree = cKDTree(samples)
        coord, ind = kdtree.query(point, k=2) # k=2 means two nearest neighbors

        # If multiple nearest neighbors, choose randomly
        if isinstance(ind, np.ndarray):
            ind = random.choice(ind)
        return self.payload[ind]


    def run(self, n_robots):

        # Reset and initialize map
        self.init_map()

        # Initialize robots
        self.init_robots(n_robots)

        # Move robots just next to each other
        for i in range(self.n_robots):
            self.robot[i].pos = self.payload[i] #(self.start.row, self.start.col)
            print('Robot', i, 'initial position:', self.robot[i].pos)

        print('')           



        # print(self.vertices[0].row)

        visited = [self.start]
        try_these = [self.start]

        # Choose the next closest node
        next_node = self.get_nearest_node((visited[0].row, visited[0].col))
        # next_node = self.choose_next((visited[0].row, visited[0].col))
        # print(next_node)


if __name__ == '__main__':
    import subprocess
    subprocess.run(['python', 'main.py'])

