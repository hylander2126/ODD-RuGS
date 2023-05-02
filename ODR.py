import math
import numpy as np
import random
from scipy.spatial import cKDTree, ConvexHull
import matplotlib.pyplot as plt


# Class for each robot
class Robot:
    def __init__(self, rid):
        self.id = rid
        self.pos = ()


# Class for each tree node
class Node:
    # def __init__(self, row, col):
    def __init__(self, configuration):
        self.config = configuration  # List of (x,y) coords of robot distribution
        # self.row = row            # coordinate
        # self.col = col            # coordinate
        self.parent = None  # parent node / edge
        self.cost = 0.0  # cost to parent / edge weight


# Class for RRT
class ODR:
    # Constructor
    def __init__(self, payload_verts, payload_shape, n_robots):
        self.payload = payload_verts            # array of tuples of payload vertices
        self.kdtree = cKDTree(self.payload)     # kdtree for nearest neighbors search
        self.shape = payload_shape              # string describing shape of payload
        self.all_nodes = []                     # list of nodes
        self.found = False                      # found flag
        self.n_robots = n_robots                # Number of robots
        self.best_cost = math.inf               # Best Cost

    def init_robots(self):
        '''Initialize the robots before each search
        '''
        self.robot = []
        for i in range(self.n_robots):
            self.robot.append(Robot(i))
            self.robot[i].pos = tuple(self.payload[i])
            print('Robot', i, 'initialized to position:', self.payload[i])
        print('')

    def init_map(self):
        '''Initialize the map before each search
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
        plt.xlim([-6, 6])
        plt.ylim([-6, 6])
        self.ax.set_aspect('equal', adjustable='box')

        if self.shape == 'circle':
            payl = plt.Circle((0, 0), 4, color='r')
            self.ax.add_patch(payl)
        elif self.shape == 'square':
            payl = plt.Rectangle((-4, -4), 8, 8)
            self.ax.add_patch(payl)

        self.circles = [plt.Circle(self.payload[i], 0.2, color='g') for i in range(self.n_robots)]
        for circ in self.circles:
            self.ax.add_patch(circ)

    def get_nearest_node(self, query_point):
        '''Find the nearest node from the new point for each robot. Use kdtree to find k-nearest neighbors
        '''
        dist, idx = self.kdtree.query(query_point, k=3)  # k=3 nearest neighbors (including query_point !!!!)
        return [self.payload[i] for num, i in enumerate(idx) if dist[num] != 0]

    def check_support_polygon(self, tol=1e-12):
        '''Determines if CoM lies within support polygon of agents
        '''
        # Array of coordinates of robot support points
        polygon = np.array([self.robot[i].pos for i in range(self.n_robots)])

        # Get centroid of support polygon
        cx = np.round(np.mean([pair[0] for pair in polygon]), 3)
        cy = np.round(np.mean([pair[1] for pair in polygon]), 3)

        # Get distance of polygon centroid to origin
        diff = np.linalg.norm((cx, cy))

        # For two robots, simple line midpoint check
        if self.n_robots < 3:
            return diff < 1e-3, diff
        # For n robots, scipy.convexhull to check if CoM in hull
        else:
            try:
                hull = ConvexHull(polygon)

                # Get polygon centroid
                cx = np.mean(hull.points[hull.vertices, 0])
                cy = np.mean(hull.points[hull.vertices, 1])
                diff = np.linalg.norm((cx, cy))
                return all(((np.dot(eq[:-1], (0, 0)) + eq[-1]) <= tol) for eq in hull.equations), diff

            # IF scipy fails to make a convex hull, it means an invalid configuration
            except:
                return False, diff

    def move_robots(self):
        '''Move each robot to another grab point. Add a small chance for robot to not move. If multiple choices, choose randomly. 
        Robot only moves if the space is unoccupied.
        '''
        # Create a set to store the positions of all robots
        occupied_nodes = set(robot.pos for robot in self.robot)

        # Create a dictionary to store the nearest nodes for each robot
        # candidate_nodes = {i: self.get_nearest_node(self.robot[i].pos) for i in range(self.n_robots)}
        candidate_nodes = {robot.id: self.get_nearest_node(tuple(robot.pos)) for robot in self.robot}

        # Move each robot
        for i in range(self.n_robots):
            # Add a small chance to stay in this position
            if np.random.random() < 0.35:
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
            self.robot[i].pos = tuple(next_node)
            # Update list of occupied nodes
            occupied_nodes.remove(previous_node)
            occupied_nodes.add(next_node)

    def extend(self):
        '''From list of all nodes, choose the next configuration with the lowest cost.
        '''
        for node in self.all_nodes:
            # if node.cost <= self.best_cost:
            self.best_cost = min(self.best_cost, node.cost)
            self.best_node = node

        for i in range(self.n_robots):
            self.robot[i].pos = tuple(self.best_node.config[i])

        if self.best_cost < 0.01:
            self.found = True

    def update_plot(self):
        '''Update robot positions each step
        '''
        for i in range(self.n_robots):
            new_center = self.robot[i].pos
            self.circles[i].center = new_center

        self.fig.canvas.draw()
        plt.pause(0.01)
        # plt.pause(5)

    def run(self, n_iters=1000):

        # Initialize robots
        self.init_robots()
        # Reset and initialize map
        self.init_map()
        # Initialize plot window
        self.init_plot()

        # plt.pause(10)

        final_iters = n_iters
        # Calculate number of tries to perform from each node to each subsequent node since we are doing a random walk
        n_tries = (3 ** self.n_robots) + (self.n_robots ** 2)

        # List of 'errors' over time between centroid and Com
        # Moment at which polygon is supporting the CoM
        already_supported = False
        support_time = 0
        support_cost = 0

        visited_configs = set()

        # Loop through all iterations
        for i in range(n_iters):

            new_config = tuple(robot.pos for robot in self.robot)

            while new_config in visited_configs:
                self.move_robots()
                new_config = tuple(robot.pos for robot in self.robot)

            visited_configs.add(new_config)

            is_supporting, cost = self.check_support_polygon()
            new_node = Node(new_config)
            new_node.parent = self.all_nodes[-1]
            new_node.cost = cost
            self.all_nodes.append(new_node)

            if is_supporting and not already_supported:
                support_time = i
                support_cost = cost
                already_supported = True

            # Now we have several new nodes from the current configuration. Choose best, lowest cost node
            self.extend()
            # Update plot
            self.update_plot()

            # Technically, we should be able to compare this best cost with the last best cost and if it's the same,
            # then we have optimized, HOWEVER, random walk makes it a non-guarantee that we have explored all nodes.

            if self.found:
                final_iters = i
                break

        ## Final Plots and Values
        print('\nThis optimal configuration found in', final_iters, 'step(s).')
        print('Final cost (polygon centroid dist to CoM):', round(self.best_cost, 4))

        # Want to plot cost of visited nodes over time and the cost at the moment where the payload is supported
        # t_span = [i for i in range(final_iters+1)]
        list_of_costs = [node.cost for node in self.all_nodes]
        t_span = [i for i in range(np.size(self.all_nodes))]
        best_cost = np.array(np.size(self.all_nodes))
        best_cost.fill(self.best_cost)

        fig, axs = plt.subplots()
        axs.set_title('Distance between polygon centroid and CoM over time')
        axs.set_xlabel('Time Steps')
        axs.set_ylabel('Configuration Cost (error)')
        axs.plot(t_span, list_of_costs, zorder=-1)
        # axs.plot(t_span, )
        axs.scatter(support_time, support_cost, c='r', marker='*', s=300, zorder=1)

        plt.show()


if __name__ == '__main__':
    import subprocess

    subprocess.run(['python', 'main.py'])
