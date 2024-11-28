import cv2
import numpy as np
import time
import math

# function to check if two line segments intersect
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

# Function to return two coordinates of the intersection point
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

# Function to expand obstacle for robot clearance
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

# function to calculate distance between two points
def calculate_distance(point1, point2):
    x1, y1 = point1
    x2, y2 = point2
    return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

# finds two optimum points around the obstacle
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

    total_dist1 = calculate_distance(robo_coords, intersection1) + calculate_distance(intersection1, station_coords)
    total_dist2 = calculate_distance(robo_coords, intersection2) + calculate_distance(intersection2, station_coords)

    if total_dist1 < total_dist2:
        optimal_intersection = intersection1
    else:
        optimal_intersection = intersection2
    
    return optimal_intersection

# function to transform pixels to real world coords
def pixel_to_real(x, y):

    # Pre-determined constants for the linear mapping
    x_pixel_min = 538
    x_pixel_max = 844.75
    x_real_min = 261.52
    x_real_max = 436.41
    y_pixel_min = 438.5
    y_pixel_max = 235.75
    y_real_min = -260.17
    y_real_max = -391.69

    # Calculate the x_real coordinate
    x_real = x_real_min + (x_real_max - x_real_min) * (x - x_pixel_min) / (x_pixel_max - x_pixel_min)
    # Calculate the y_real coordinate
    y_real = y_real_min + (y_real_max - y_real_min) * (y - y_pixel_min) / (y_pixel_max - y_pixel_min)

    return x_real, y_real


# Select the external camera (1) that is connected to the laptop
cap = cv2.VideoCapture(1)
width = 1280
height = 720
cap.set(3, width)  # 3 corresponds to CV_CAP_PROP_FRAME_WIDTH
cap.set(4, height)  # 4 corresponds to CV_CAP_PROP_FRAME_HEIGHT


# Robo coordinate to filter out edges
x_robo = 147.81
y_robo = -437.51

# Initialize the new list of points
filtered_points = []


# Execute this continuously
while(True):
    
    # Start the performance clock
    start = time.perf_counter()
    
    # Capture current frame from the camera
    ret, frame = cap.read()
    
    # Convert the image from the camera to Gray scale
    grey = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    grey_optimized = cv2.GaussianBlur(grey,(5,5),0)
    edges = cv2.Canny(grey_optimized,200,300)

    # Find contours in the edge image
    contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Iterate over detected contours
    for contour in contours:
        # Approximate the contour to reduce the number of points
        epsilon = 0.04 * cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, epsilon, True)

        # Check if the number of rows in the 'approx' array is not equal to 4
        if approx.shape[0] != 4:
            continue  # Skip this 'approx' array
            
        print(approx)
        # Create a list to store valid points
        valid_points = []

        thresh = 150
        # Extract and print the coordinates of the contour points
        for point in approx:
            for i in range(2):
                x, y = point[0]

                x_real, y_real = pixel_to_real(x, y)

                if not (x_real > x_robo - thresh and x_real < x_robo + thresh and y_real > y_robo - thresh and 
                        y_real < y_robo + thresh):
                    valid_points.append(point)
            
            
        valid_points = np.array(valid_points)
        print('hi')
        print(valid_points)

        # Draw the contours on the frame
        try:
            cv2.drawContours(frame, [valid_points], 0, (0, 255, 0), 2)
        except:
            pass

    
    cv2.imshow('high_higher',frame)

        # Stop the performance counter
    end = time.perf_counter()
    
    # Print to console the exucution time in FPS (frames per second)
    # print ('{:4.1f}'.format(1/(end - start)))

    # If the button q is pressed in one of the windows 
    if cv2.waitKey(20) & 0xFF == ord('q'):
        # Exit the While loop
        break
    
print(valid_points)

# When everything done, release the capture
cap.release()
# close all windows
cv2.destroyAllWindows()
# exit the kernel
exit(0)

