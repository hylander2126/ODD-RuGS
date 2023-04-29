import math
import numpy as np
import random
from scipy.spatial import cKDTree, ConvexHull
import matplotlib.pyplot as plt


# Class for each robot
class Robot:
    def __init__(self, rid):
        self.id = rid
        self.pos = []


# Class for each tree node
class Node:
    # def __init__(self, row, col):
    def __init__(self, configuration):
        self.config = configuration # List of (x,y) coords of robot distribution
        # self.row = row            # coordinate
        # self.col = col            # coordinate
        self.parent = None          # parent node / edge
        self.cost = 0.0             # cost to parent / edge weight


# Class for RRT
class ODR:
    # Constructor
    def __init__(self, payload_verts, start, goal):
        self.payload = payload_verts          # array of tuples of payload vertices

        self.start = Node(start) # start node
        self.goal = Node(goal)    # goal node
        self.all_nodes = []                    # list of nodes
        self.found = False                    # found flag


    def init_robots(self, n_robots):
        '''Initialize the robots before each search
        '''
        self.n_robots = n_robots
        self.robot = []
        for i in range(n_robots):
            self.robot.append(Robot(i))
            self.robot[i].pos = self.payload[i]
            print('Robot', i, 'initialized to position:', self.payload[i])


    def init_map(self):
        '''Intialize the map before each search
        '''
        self.found = False
        self.all_nodes = []
        new_config = np.array([self.robot[i].pos for i in range(self.n_robots)])
        self.all_nodes.append(Node(new_config))
        # self.all_nodes.append(self.start)


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

        # print(samples)

        kdtree = cKDTree(samples)
        coord, ind = kdtree.query(point, k=2) # k=2 means two nearest neighbors

        # If multiple nearest neighbors, choose randomly
        if isinstance(ind, np.ndarray):
            ind = random.choice(ind)
        return samples[ind]


    def get_cost(self):
        '''Determine the cost of the current configuration based on load distribution
        '''


    def check_support_polygon(self, tol=1e-12):
        '''Determines if CoM lies within support polygon of agents
        '''
        # Point to check (CoM of circle for now) TODO update with actual CoM or centroid of arbirtrary object
        point = np.array([0,0])
        # Array of coordinates of robot support points
        polygon = np.array([self.robot[i].pos for i in range(self.n_robots)])

        # Simple line check for two robots
        if self.n_robots < 3:

            v1 = polygon[1] - polygon[0]
            v2 = point - polygon[0]

            d = np.dot(v1, v2)
            v1_squared = np.dot(v1, v1)

            if d < 0 or d > v1_squared:
                print('CoM not supported by the robots!')
                return False
            else:
                dist = np.linalg.norm(np.cross(v1, v2)) / np.sqrt(v1_squared)
                return dist < tol

        else:
            hull = ConvexHull(polygon)
            return all(((np.dot(eq[:-1], point) + eq[-1]) <= tol) for eq in hull.equations)


    def move_robots(self):
        '''Randomly move robots to nearest attachment point
        '''
        for i in range(self.n_robots):
            next_node = self.get_nearest_node(self.robot[i].pos)
            self.robot[i].pos = next_node
            # print('Robot', i, 'moved to', next_node)


    def run(self, n_robots, n_iters):

        # Initialize robots
        self.init_robots(n_robots)

        # Reset and initialize map
        self.init_map()

        print('')

        # visited = [self.start]
        # try_these = [self.start]

        for i in range(n_iters):
            # Move each robot to nearest node/vertex (randomly choose between the two nearest)
            self.move_robots()

            # Add a node consisting of each robot's position
            new_config = np.array([self.robot[i].pos for i in range(n_robots)])

            # Check if this configuration has already been tried. Loop through all nodes added until now. TODO see if a 'visited' list would be faster
            visited = False
            for node in self.all_nodes:
                if np.array_equiv(np.sort(new_config,axis=0), np.sort(node.config,axis=0)):
                    print('New configuration already tried!!')
                    visited = True
                    break
            
            # If this configuration hasn't been tried, add a new node
            if not visited:
                new_node = Node(new_config)
                new_node.parent = self.all_nodes[-1]
                self.all_nodes.append(new_node)
            
            # next_node = self.get_nearest_node((visited[0].row, visited[0].col))
            # print(next_node)

            # Check support polygon condition
            # print(self.check_support_polygon())



if __name__ == '__main__':
    import subprocess
    subprocess.run(['python', 'main.py'])

