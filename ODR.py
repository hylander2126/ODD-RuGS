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
        self.best_cost = math.inf       # Best Cost


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
        # Generate initial configuration of robots around the payload
        init_config = np.array([self.robot[i].pos for i in range(self.n_robots)])
        # Get initial support polygon
        is_supporting, cost = self.check_support_polygon()
        # Create a new node and append to all nodes list
        init_node = Node(init_config)
        init_node.cost = cost
        
        self.all_nodes.append(init_node)


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

        kdtree = cKDTree(samples)
        coord, index = kdtree.query(point, k=2) # k=2 means two nearest neighbors

        return [samples[i] for i in index]


    def check_support_polygon(self, tol=1e-12):
        '''Determines if CoM lies within support polygon of agents
        '''
        # Array of coordinates of robot support points
        polygon = np.array([self.robot[i].pos for i in range(self.n_robots)])

        # Get centroid of support polygon
        cx = np.round(np.mean([pair[0] for pair in polygon]), 3)
        cy = np.round(np.mean([pair[1] for pair in polygon]), 3)
       
        # Get distance of polygon centroid to origin
        diff = np.linalg.norm((cx,cy))


        ## For two robots, simple line midpoint check
        if self.n_robots < 3:
            return (diff < 1e-3, diff)
        ## For n robots, scipy.convexhull to check if CoM in hull
        else:
            try:
                hull = ConvexHull(polygon) 
                
                #Get polygon centroid
                cx = np.mean(hull.points[hull.vertices,0])
                cy = np.mean(hull.points[hull.vertices,1])
                diff = np.linalg.norm((cx,cy))    
                return (all(((np.dot(eq[:-1], point) + eq[-1]) <= tol) for eq in hull.equations), diff)

            # IF scipy fails to make a convex hull, it means an invalid configuration
            except:
                return (False, diff)


    def move_robots(self):
        '''Move each robot to another grab point. Add a small chance for robot to not move. If multiple choices, choose randomly. 
        Robot only moves if the space is unoccupied.
        '''
        # Create a set to store the positions of all robots
        occupied_nodes = set(robot.pos for robot in self.robot)

        # Create a dictionary to store the nearest nodes for each robot
        candidate_nodes = {i: self.get_nearest_node(self.robot[i].pos) for i in range(self.n_robots)}

        # Move each robot
        for i in range(self.n_robots):
            # Add a small chance to stay in this position
            if np.random.random() < 0.2:
                continue

            # New list of only unoccupied nodes
            empty_nodes = [node for node in candidate_nodes[i] if node not in occupied_nodes]

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


    def extend(self):
        lowest_cost = math.inf
        best_node = []
        for node in self.all_nodes:
                if node.cost < lowest_cost:
                    lowest_cost = min(lowest_cost, node.cost)
                    best_node = node

        for i in range(self.n_robots):
                self.robot[i].pos = best_node.config[i]

        if lowest_cost < 0.1:
            self.found = True
            self.best_cost = lowest_cost


    def update_plot(self):
        '''Update robot positions each step
        '''
        for i in range(self.n_robots):
            new_center = self.robot[i].pos
            self.circles[i].center = new_center

        self.fig.canvas.draw()
        plt.pause(0.001)
        # plt.pause(0.5)


    def run(self, n_iters=500):

        # Initialize robots
        self.init_robots()
        # Reset and initialize map
        self.init_map()
        # Initialize plot window
        self.init_plot()

        final_iters = 0
        # Calculate number of tries to perform from each node to each subsequent node since we are doing a random walk
        n_tries = (self.n_robots**3) + self.n_robots**2

        # Loop through all iterations
        for i in range(n_iters):

            # Increment counter
            final_iters += 1
            list_of_visited = set()
            # lowest_cost = math.inf
            # best_node = []

            # print('\nMaking list of nodes ...')

            for kk in range(n_tries):
                # Move each robot to nearest unoccupied vertex
                self.move_robots()

                # Record this configuration of robot positions
                new_config = tuple([self.robot[i].pos for i in range(self.n_robots)])

                # If this configuration has been visited, skip it
                if new_config in list_of_visited:
                    continue
                else:
                    list_of_visited.add(new_config)
                    is_supporting, cost = self.check_support_polygon()
                    new_node = Node(new_config)
                    new_node.parent = self.all_nodes[-1]
                    new_node.cost = cost
                    self.all_nodes.append(new_node)


            # Now we have several new nodes from the current configuration. Let's choose the best node, i.e. lowest node cost
            self.extend()
            # print('Now finding the best node')
            # for node in self.all_nodes:
            #     if node.cost < lowest_cost:
            #         lowest_cost = min(lowest_cost, node.cost)
            #         best_node = node

            # Now we can erase all data from all nodes list
            # self.all_nodes = [self.all_nodes[0]]

            ## Technically, we should be able to compare this best cost with the last best cost and if its the same, then we have optimized
            ## HOWEVER, the random walk criteria makes it a non-guarantee that we have explored all nodes.

            # print('The best node is', best_node.config, 'with a cost of:', lowest_cost)
            # Now actually move all robots to the best configuration from the list
            # for i in range(self.n_robots):
            #     self.robot[i].pos = best_node.config[i]

            # Update plot
            self.update_plot()



            if self.found:
                print('\nAll robots supporting CoM! Found in', final_iters, 'step(s).')
                # print('Final robot positions:\n', np.round([self.robot[i].pos for i in range(self.n_robots)], 3))
                print('Final cost (polygon centroid dist to CoM):', round(self.best_cost,4))
                break

        plt.show()


if __name__ == '__main__':
    import subprocess
    subprocess.run(['python', 'main.py'])

