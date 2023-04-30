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
    def __init__(self, payload_verts, payload_shape, n_robots):
        self.payload = payload_verts    # array of tuples of payload vertices
        self.shape = payload_shape      # string describing shape of payload
        self.all_nodes = []             # list of nodes
        self.found = False              # found flag
        self.n_robots = n_robots        # Number of robots


    def init_robots(self):
        '''Initialize the robots before each search
        '''
        self.robot = []
        for i in range(self.n_robots):
            self.robot.append(Robot(i))
            self.robot[i].pos = self.payload[i]
            print('Robot', i, 'initialized to position:', self.payload[i])
        print('')


    def init_map(self):
        '''Intialize the map before each search
        '''
        self.found = False
        self.all_nodes = []
        # Generate initial configuration of robots around the payload and create the first Node 
        init_config = np.array([self.robot[i].pos for i in range(self.n_robots)])
        self.all_nodes.append(Node(init_config))


    def init_plot(self):
        '''Initialize the plot for live-plotting later
        '''
        self.fig, self.ax = plt.subplots()
        plt.xlim([-6,6])
        plt.ylim([-6,6])
        self.ax.set_aspect('equal', adjustable='box')
        
        if self.shape == 'circle':
            payl = plt.Circle((0,0), 4, color='r')
            self.ax.add_patch(payl)
        elif self.shape == 'square':
            payl = plt.Rectangle((-2,-2), 4, 4)
            self.ax.add_patch(payl)

        self.circles = [plt.Circle(self.payload[i],0.2, color='g') for i in range(self.n_robots)]
        for circ in self.circles:
            self.ax.add_patch(circ)


    def get_nearest_node(self, point):
        '''Find the nearest node from the new point
        '''
        # Use kdtree to find k-nearest neighbors ***.COPY() SO WE DONT REMOVE FROM ORIGINAL LIST
        samples = self.payload.copy()
        # Remove this point from the list of samples
        samples.remove(point)

        # print(samples)

        kdtree = cKDTree(samples)
        coord, index = kdtree.query(point, k=2) # k=2 means two nearest neighbors

        # If multiple nearest neighbors, choose randomly
        # if isinstance(ind, np.ndarray):
        #     ind = random.choice(ind)
        # return samples[ind]

        nearest_neighbors = [samples[i] for i in index]
        return nearest_neighbors


    def check_support_polygon(self, tol=1e-12):
        '''Determines if CoM lies within support polygon of agents
        '''
        # Point to check (CoM of circle for now) TODO update with actual CoM or centroid of arbirtrary object
        point = np.array([0,0])
        # Array of coordinates of robot support points
        polygon = np.array([self.robot[i].pos for i in range(self.n_robots)])

        # Simple distance check between robot midpoint and CoM
        if self.n_robots < 3:
            midpoint = np.add(self.robot[0].pos, self.robot[1].pos)/2
            diff = np.linalg.norm(midpoint)
            return diff < 1e-3
        # Use Scipy convex hull to check CoM in hull
        else:
            # If robots all on the same side, cannot create convex hull. In this case, return False
            try:
                hull = ConvexHull(polygon)
                return all(((np.dot(eq[:-1], point) + eq[-1]) <= tol) for eq in hull.equations)
            except:
                return False


    def move_robots(self):
        '''Move each robot to another grab point. Add a small chance for robot to not move.
        If multiple choices, choose randomly. Robot only moves if the space is unoccupied.
        '''
        # Create a set to store the positions of all robots
        occupied_nodes = set(robot.pos for robot in self.robot)
        # print('occupied nodes:', occupied_nodes)

        # Create a dictionary to store the nearest nodes for each robot
        nearest_nodes = {i: self.get_nearest_node(self.robot[i].pos) for i in range(self.n_robots)}
        # print(nearest_nodes)

        # Move each robot
        for i in range(self.n_robots):
            # Add a small chance to stay in this position
            if np.random.random() < 0.2:
                continue

            # Get this robot's candidate nodes
            candidate_nodes = nearest_nodes[i]
            # print('candidate nodes for', i, ':', candidate_nodes)
            # New list of only unoccupied nodes
            empty_nodes = [node for node in candidate_nodes if node not in occupied_nodes]

            # If all possibilities occupied, don't move
            if not empty_nodes:
                continue
            
            # If multiple choices exists, choose randomly
            next_node = random.choice(empty_nodes)
            # Record previous node for removal after updating robot
            previous_node = self.robot[i].pos

            # Move this robot to the new node
            self.robot[i].pos = next_node
            # Update list of occupied nodes
            occupied_nodes.remove(previous_node)
            occupied_nodes.add(next_node)


    def update_plot(self):
        '''Update robot positions each step
        '''
        for i in range(self.n_robots):
            new_center = self.robot[i].pos
            self.circles[i].center = new_center

        self.fig.canvas.draw()
        # plt.pause(0.001)
        plt.pause(0.01)


    def get_cost(self, is_supporting):
        '''Determine the cost of the current configuration based on load distribution
        '''
        # If the load is not supported, it is an invalid configuration
        if not is_supporting:
            return math.inf
        # Otherwise, calculate the goodness of this config based on load distribution
        else:
            
            return 0


    def run(self, n_iters=500):

        # Initialize robots
        self.init_robots()
        # Reset and initialize map
        self.init_map()
        # Initialize plot window
        self.init_plot()

        final_iters = 0
        is_supporting = False

        # Loop through all iterations
        for i in range(n_iters):
            # Move each robot to nearest unoccupied vertex
            self.move_robots()
            # Update plot
            self.update_plot()
            # Increment counter
            final_iters += 1

            # Record this configuration of robot positions
            new_config = np.array([self.robot[i].pos for i in range(self.n_robots)])

            # Check if configuration already tried, loop through all previously visited nodes. TODO see if a 'visited' list would be faster
            visited = False
            for node in self.all_nodes:
                if np.array_equiv(np.sort(new_config,axis=0), np.sort(node.config,axis=0)):
                    # print('New configuration already tried!!')
                    visited = True
                    break

            # If this configuration hasn't been tried, add a new node
            if not visited:
                new_node = Node(new_config)
                new_node.parent = self.all_nodes[-1]
                is_supporting = self.check_support_polygon()
                new_node.cost = self.get_cost(is_supporting)
                self.all_nodes.append(new_node)
            # else:
                # print('already visited')

            # FOR NOW, if valid configuratoin, STOP. TODO: Add IF ALL VISITED STOP CONDITION
            if is_supporting:
                print('All robots supporting CoM! Found in', final_iters, 'step(s).')
                print('Final robot positions:', [self.robot[i].pos for i in range(self.n_robots)])
                break

        plt.show()


if __name__ == '__main__':
    import subprocess
    subprocess.run(['python', 'main.py'])

