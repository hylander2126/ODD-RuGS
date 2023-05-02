
import math
from ODR import ODR
import numpy as np


def get_points_on_line(p1, p2, n):
	'''For square shape generation. Return n points on the line
	'''
	x1, y1 = p1
	x2, y2 = p2
	d = ((x2-x1)**2 + (y2-y1)**2)**0.5 # distance between p1 and p2
	dx = (x2-x1)/(n-1) # x increment
	dy = (y2-y1)/(n-1) # y increment
	points = [(x1+i*dx, y1+i*dy) for i in range(n)]
	return points


def create_payload(shape, n_vertices):
	''' Generate a list of tuples corresponding to vertices of the payload (x,y)
	'''
	vertices = []
	n = n_vertices			# Num of desired vertices
	r = 4					# Body Radius

	if shape == 'circle':
		for i in range(n):
			x = round(r*math.cos(2*math.pi*i / n), 3)
			y = round(r*math.sin(2*math.pi*i / n), 3)
			vertices.append((x,y))

	if shape == 'square':
		p1 = (r, -r)
		p2 = (r, r)
		p3 = (-r, r)
		p4 = (-r, -r)

		n = int(n/4)  # Divide by 4 so n/4 points for each line is generated

		points = get_points_on_line(p1, p2, n)
		points.extend(get_points_on_line(p2, p3, n))
		points.extend(get_points_on_line(p3, p4, n))
		points.extend(get_points_on_line(p4, p1, n))

		points = list(dict.fromkeys(points))
		vertices = points

	if shape == 'crescent':
		# define the outer and inner radii of the crescent
		outer_radius = r + 2
		inner_radius = r - 2

		# generate angles evenly spaced around a circle
		angles = np.linspace(0, 2 * np.pi, n)

		# calculate the x,y coordinates of the outer and inner circles
		outer_x = outer_radius * np.cos(angles)
		outer_y = outer_radius * np.sin(angles)
		inner_x = inner_radius * np.cos(angles)
		inner_y = inner_radius * np.sin(angles)

		# create a list of vertices for the crescent
		vertices = []
		for i in range(n):
			if i < n / 2:
				vertices.append((outer_x[i], outer_y[i]))
			else:
				vertices.append((inner_x[i], inner_y[i]))

	return vertices


#___________________________________________________________
## Define simulation parameters
n_vertices = 128		# Number of payload 'grab points' or vertices
n_robots = 3 			# Number of robots (id=0 is first robot)
shape = 'square'		# Shape to test. (only 'circle' for now)

## Load the payload object
payload = create_payload(shape, n_vertices)

# import matplotlib.pyplot as plt
# x = []
# y = []
# for thing in payload:
# 	x.append(thing[0])
# 	y.append(thing[1])
# plt.scatter(x,y)
# plt.show()
# print(payload)


## Optimal dynamic distribution (ODR) class
for _ in range(1):
	ODR_Planner = ODR(payload, shape, n_robots)
	ODR_Planner.run(n_iters=1000)

