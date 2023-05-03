
import math

import matplotlib.pyplot as plt

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


def create_payload(payload_shape, num_verts):
	''' Generate a list of tuples corresponding to vertices of the payload (x,y)
	'''
	vertices = []
	n = num_verts			# Num of desired vertices
	r = 4					# Body Radius

	if payload_shape == 'Circle':
		for i in range(n):
			x = round(r*math.cos(2*math.pi*i / n), 3)
			y = round(r*math.sin(2*math.pi*i / n), 3)
			vertices.append((x,y))

	if payload_shape == 'Box':
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

	# if shape == 'crescent':
	return vertices


#___________________________________________________________
## Define simulation parameters
n_vertices = 128		# Number of payload 'grab points' or vertices
n_robots = 3 			# Number of robots (id=0 is first robot)
shape = 'Circle'		# Shape to test. ('Box' or 'Circle'

## Load the payload object
payload = create_payload(shape, n_vertices)

## Visualize error
supp_time_list = []
supp_cost_list = []

fig, axs = plt.subplots()
axs.set_title('All Agents ' + str(shape) + ' Optimal Distribution Error')
axs.set_xlabel('Time Step')
axs.set_ylabel('Error')

## Optimal dynamic distribution (ODR) class
for n_robots in [3, 5, 10]:

	# Run for each desired n_robots
	ODR_Planner = ODR(payload, shape, n_robots)
	t, cost_list, supp_time, supp_cost = ODR_Planner.run(n_iters=1000)

	# If minimum viable solution has been found
	if supp_time != 0:
		supp_time_list.append(supp_time)
		supp_cost_list.append(supp_cost)

	# Plot this series (n_robots trial)
	axs.plot(t, cost_list, zorder=-1, label=str(n_robots)+' robots')

# Plot Viable solutions
axs.scatter(supp_time_list, supp_cost_list, c='r', marker='*', s=300, zorder=1, label='Viable Solution')
axs.legend()
plt.show()
