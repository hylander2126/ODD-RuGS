import numpy as np
from scipy.spatial import ConvexHull
import math

def calculate_weight_distribution(angle_a, angle_b, angle_c):
    # Coordinates of each robot
    coords_a = (1, 0)
    coords_b = (math.cos(angle_b), math.sin(angle_b))
    coords_c = (math.cos(angle_c), math.sin(angle_c))

    # Distance between each adjacent pair of robots
    dist_a_b = math.sqrt((coords_b[0] - coords_a[0]) ** 2 + (coords_b[1] - coords_a[1]) ** 2)
    dist_b_c = math.sqrt((coords_c[0] - coords_b[0]) ** 2 + (coords_c[1] - coords_b[1]) ** 2)
    dist_c_a = math.sqrt((coords_a[0] - coords_c[0]) ** 2 + (coords_a[1] - coords_c[1]) ** 2)

    # Angles between each adjacent pair of robots
    angle_a_b = angle_b - angle_a
    angle_b_c = angle_c - angle_b
    angle_c_a = 2 * math.pi - angle_c + angle_a

    # Weight distribution for each robot
    weight_a = (dist_b_c / 2 + dist_c_a) / (dist_a_b + dist_b_c + dist_c_a)
    weight_b = dist_a_b / (dist_a_b + dist_b_c + dist_c_a)
    weight_c = dist_b_c / 2 / (dist_a_b + dist_b_c + dist_c_a)

    # Normalize weight distribution to percentages
    total_weight = weight_a + weight_b + weight_c
    weight_a_pct = weight_a / total_weight * 100
    weight_b_pct = weight_b / total_weight * 100
    weight_c_pct = weight_c / total_weight * 100

    return [weight_a_pct, weight_b_pct, weight_c_pct]

def calculate_robot_weight(payload_weight, angles):
	"""
	Calculates the weight that each robot is carrying.

	Args:
	total_weight: The total weight of the payload.
	angles: A list of angles between each pair of robots.

	Returns:
	A list of weights, one for each robot.
	"""

	num_robots = len(angles)
	distance_matrix = np.zeros((num_robots, num_robots))

	# Calculate the distances between each pair of robots.
	for i in range(num_robots):
	    for j in range(i + 1, num_robots):
	        distance = np.sqrt(2 - 2 * np.cos(angles[i] - angles[j]))
	        distance_matrix[i][j] = distance
	        distance_matrix[j][i] = distance

	# Calculate the weight that each robot is carrying.
	robot_weights = np.zeros(num_robots)
	for i in range(num_robots):
	    j = (i + 1) % num_robots
	    k = (i + 2) % num_robots
	    s = (distance_matrix[i][j] + distance_matrix[j][k] + distance_matrix[k][i]) / 2
	    area = np.sqrt(s * (s - distance_matrix[i][j]) * (s - distance_matrix[j][k]) * (s - distance_matrix[k][i]))
	    robot_weights[i] = payload_weight * distance_matrix[j][k] * np.sin(angles[j] - angles[k]) / (2 * area)

	return robot_weights


def calculate_robot_weight2(payload_weight, angles):
	num_robots = len(angles)
	distance_matrix = np.zeros((num_robots, num_robots))

	# Calculate the distances between each pair of robots.
	for i in range(num_robots):
	    for j in range(i + 1, num_robots):
	        distance = np.sqrt(2 - 2 * np.cos(angles[i] - angles[j]))
	        distance_matrix[i][j] = distance
	        distance_matrix[j][i] = distance

	# Calculate the weight that each robot is carrying.
	robot_weights = np.zeros(num_robots)
	total_weight = 0
	for i in range(num_robots):
	    j = (i + 1) % num_robots
	    k = (i + 2) % num_robots
	    s = (distance_matrix[i][j] + distance_matrix[j][k] + distance_matrix[k][i]) / 2
	    area = np.sqrt(s * (s - distance_matrix[i][j]) * (s - distance_matrix[j][k]) * (s - distance_matrix[k][i]))
	    weight = payload_weight * distance_matrix[j][k] * np.sin(angles[j] - angles[k]) / (2 * area)
	    robot_weights[i] = weight
	    total_weight += weight

	# Adjust the weights so that they add up to the total weight of the payload.
	adjustment_factor = payload_weight / total_weight
	robot_weights *= adjustment_factor

	return robot_weights


th1 = math.radians(0)
th2 = math.radians(120+30)
th3 = math.radians(240-30)
th4 = math.radians(180)

th1 = math.radians(0)
th2 = math.radians(90)
th3 = math.radians(180)
th4 = math.radians(270)

ans = abs(calculate_robot_weight2(1, [th1, th2, th3, th4]))
ans = np.round(ans, 3)
ans = np.array(ans)
print(ans)
print(np.sum(ans))
