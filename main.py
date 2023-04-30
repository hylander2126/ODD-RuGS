
import math
import numpy as np
from ODR import ODR
from ODR import Robot
import matplotlib.pyplot as plt


def get_points_on_line(p1, p2, n):
	'''For square shape generation. Return n points on line
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

	if shape=='circle':
		r = 4				# Radius
		for i in range(n):
			x = round(r*math.cos(2*math.pi*i / n), 3)
			y = round(r*math.sin(2*math.pi*i / n), 3)
			vertices.append((x,y))

	if shape=='square':
		r2 = 4			# Body Radius	
		r = r2/2

		p1 = (r,-r)
		p2 = (r,r)
		p3 = (-r,r)
		p4 = (-r,-r)

		n = int(n/4)	# Divide by 4 so n/4 points for each line is generated

		points = get_points_on_line(p1, p2, n)
		points.extend(get_points_on_line(p2, p3, n))
		points.extend(get_points_on_line(p3, p4, n))
		points.extend(get_points_on_line(p4, p1, n))

		points = list(dict.fromkeys(points))
		vertices = points

	return vertices


#___________________________________________________________
## Define simulation parameters
n_vertices = 36		# Number of payload 'grab points' or vertices
n_robots = 2 		# Number of robots (id=0 is first robot)
shape = 'square'	# Shape to test. (only 'circle' for now)

## Load the payload
payload = create_payload(shape, n_vertices)


## Optimal dynamic distribution (ODR) class
ODR_Planner = ODR(payload, shape, n_robots)
ODR_Planner.run(n_iters=100)
