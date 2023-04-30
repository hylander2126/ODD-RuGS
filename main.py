
import math
import numpy as np
from ODR import ODR
from ODR import Robot
import matplotlib.pyplot as plt


def create_payload(shape, n_vertices):
	''' Generate a list of tuples corresponding to vertices of the payload (x,y)
	'''
	if shape=='circle':
		r = 4				# Radius
		n = n_vertices		# Num of desired vertices
		x = []
		y = []
		vertices = []
		for i in range(n):
			# x.append( round(r*math.cos(2*math.pi*i / n), 3) )
			# y.append( round(r*math.sin(2*math.pi*i / n), 3) )
			x = round(r*math.cos(2*math.pi*i / n), 3)
			y = round(r*math.sin(2*math.pi*i / n), 3)
			vertices.append((x,y))

	return vertices



#___________________________________________________________
## Define simulation parameters
n_vertices = 36		# Number of payload 'grab points' or vertices
n_robots = 2 		# Number of robots (id=0 is first robot)
shape = 'circle'	# Shape to test. (only 'circle' for now)


## Load the payload
payload = create_payload(shape, n_vertices)

## Optimal dynamic distribution (ODR) class
ODR_Planner = ODR(payload, n_robots)
ODR_Planner.run(n_iters=10)
