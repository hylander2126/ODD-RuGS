
import math
import numpy as np
from ODR import ODR
from ODR import Robot
import matplotlib.pyplot as plt


def create_payload(shape, n_vertices):
	''' Generate a list of tuples corresponding to vertices 
	of the payload (x,y)'''
	if shape=='circle':
		r = 4			# Radius
		n = n_vertices	# Num of desired vertices
		x = []
		y = []
		vertices = []
		for i in range(n):
			# x.append( round(r*math.cos(2*math.pi*i / n), 3) )
			# y.append( round(r*math.sin(2*math.pi*i / n), 3) )
			x = round(r*math.cos(2*math.pi*i / n), 3)
			y = round(r*math.sin(2*math.pi*i / n), 3)
			vertices.append((x,y))
		# vertices = np.array([x,y])

	return vertices


#___________________________________________________________
## Load the payload
n_vertices = 8
payload = create_payload('circle', n_vertices)
start = payload[0]
goal = payload[-1]

## Optimal dynamic distribution (ODR) class
n_robots = 2 # Define number of robots (id=0 is first robot)

ODR_Planner = ODR(payload, start, goal)
ODR_Planner.run(n_robots, 10)


## **** TEMP PLOT PAYLOAD ****
x = []
y = []
for i in payload:
	x.append(i[0])
	y.append(i[1])

plt.scatter(x,y)
# plt.show()


## Search with RRT and RRT*
# RRT_planner.RRT(n_pts=1000)
# RRT_planner.RRT_star(n_pts=2000)
# RRT_planner.informed_RRT_star(n_pts=2000)

