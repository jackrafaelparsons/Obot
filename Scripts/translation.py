import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np

def do_lines_intersect(p1, q1, p2, q2):
    def orientation(p, q, r):
        val = (q[1] - p[1]) * (r[0] - q[0]) - (q[0] - p[0]) * (r[1] - q[1])
        if val == 0: return 0  # Collinear
        return 1 if val > 0 else 2  # Clockwise or counterclockwise

    def on_segment(p, q, r):
        return (q[0] <= max(p[0], r[0]) and q[0] >= min(p[0], r[0]) and
                q[1] <= max(p[1], r[1]) and q[1] >= min(p[1], r[1]))

    o1 = orientation(p1, q1, p2)
    o2 = orientation(p1, q1, q2)
    o3 = orientation(p2, q2, p1)
    o4 = orientation(p2, q2, q1)

    if o1 != o2 and o3 != o4:
        return True

    if o1 == 0 and on_segment(p1, p2, q1): return True
    if o2 == 0 and on_segment(p1, q2, q1): return True
    if o3 == 0 and on_segment(p2, p1, q2): return True
    if o4 == 0 and on_segment(p2, q1, q2): return True

    return False

def calculate_intersection(coord1, coord2, coord3, coord4):

    x1, y1 = coord1
    x2, y2 = coord2
    x3, y3 = coord3
    x4, y4 = coord4

    # Calculate the vectors between the pairs of coordinates
    vector1 = (x2 - x1, y2 - y1)
    vector2 = (x4 - x3, y4 - y3)

    # Check if the vectors are parallel (cross product is zero)
    cross_product = vector1[0] * vector2[1] - vector1[1] * vector2[0]

    # Calculate the parameters for the intersection point
    t = ((x3 - x1) * vector2[1] - (y3 - y1) * vector2[0]) / cross_product

    # Calculate the intersection point
    intersection_x = x1 + t * vector1[0]
    intersection_y = y1 + t * vector1[1]

    intersection = (intersection_x, intersection_y)

    return intersection


def obstacle_expander(obstacle, clearance):
    # Calculate the center of the rectangle
    center = ((obstacle[0][0] + obstacle[2][0]) / 2, (obstacle[0][1] + obstacle[2][1]) / 2)

    # Calculate the new corner coordinates
    expanded_obstacle = []
    for corner in obstacle:
        if corner[0] < center[0]:
            new_corner_x = corner[0] - clearance
        else:
            new_corner_x = corner[0] + clearance
        if corner[1] < center[1]:
            new_corner_y = corner[1] - clearance
        else:
            new_corner_y = corner[1] + clearance
        new_corner = (new_corner_x, new_corner_y)
        expanded_obstacle.append(new_corner)
    
    return expanded_obstacle


def intersection_checker(robo_coords, station_coords, expanded_obstacle, relative_angle):
    # first check if path collides with obstacle
    # return alternative path
    min_dists = [(float('inf'), None), (float('inf'), None)]
    # If rel angle is north or south of robot
    if relative_angle < 45 or relative_angle > 315 or (relative_angle > 135 and relative_angle < 225):
        # choose two closest y coordinates from the obstacle
        for corner in expanded_obstacle:
            dist = abs(corner[1] - robo_coords[1])
            # Update the two closest distances and corners
            if dist < min_dists[0][0]:
                min_dists[1] = min_dists[0]
                min_dists[0] = (dist, corner)
            elif dist < min_dists[1][0]:
                min_dists[1] = (dist, corner)
    else:
        # choose two closest x coordinates from the obstacle
        for corner in expanded_obstacle:
            dist = abs(corner[0] - robo_coords[0])
            # Update the two closest distances and corners
            if dist < min_dists[0][0]:
                min_dists[1] = min_dists[0]
                min_dists[0] = (dist, corner)
            elif dist < min_dists[1][0]:
                min_dists[1] = (dist, corner)

    # Extract the two closest corners as intersection1 and intersection2
    intersection11 = min_dists[0][1]
    intersection12 = min_dists[1][1]

    # Find the remaining two corners
    remaining_corners = [corner for corner in expanded_obstacle if corner not in [intersection11, intersection12]]

    # If the relative angle is north/south, find the corner with an x coordinate closest to intersection21
    if relative_angle < 45 or relative_angle > 315 or (relative_angle > 135 and relative_angle < 225):
        remaining_corners.sort(key=lambda corner: abs(corner[0] - intersection12[0]))
    # If the relative angle is east/west, find the corner with a y coordinate closest to intersection21
    else:
        remaining_corners.sort(key=lambda corner: abs(corner[1] - intersection12[1]))

    intersection21 = remaining_corners[1]
    intersection22 = remaining_corners[0]

    intersection1 = calculate_intersection(robo_coords, intersection11, station_coords, intersection21)
    intersection2 = calculate_intersection(robo_coords, intersection12, station_coords, intersection22)

    return intersection1, intersection2



# Define the obstacle rectangle as a list of its four corners (x, y coordinates)
obstacle = [(370, -385), (390, -385), (388.7, -318.2), (371, -319)]


# define relative angle of station to robot
relative_angle = 270
robo_coords = (189, -354)

# Define the next station coordinates
station_coords = (570, -346)

# find clearnace from obstacle
clearance = 20 # Should be = to robot radius + safety margin
expanded_obstacle = obstacle_expander(obstacle, clearance)

path_line = [robo_coords, station_coords]
for i in range(4):
    corner1 = expanded_obstacle[i]
    corner2 = expanded_obstacle[(i + 1) % 4]

if do_lines_intersect(corner1, corner2, path_line[0], path_line[1]):
    intersection1, intersection2 = intersection_checker(robo_coords,station_coords, expanded_obstacle, relative_angle)



