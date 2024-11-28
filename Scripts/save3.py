import cv2
import cv2.aruco as aruco
import numpy as np
import time
import math
import itertools
import paho.mqtt.client as mqtt

# Rotation Matrix function
# Calculates rotation matrix to euler angles
def matrix_to_euler_angles(R):
    sy = math.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)
    
    if sy < 1e-6:
        x = math.atan2(-R[1, 2], R[1, 1])
        y = math.atan2(-R[2, 0], sy)
        z = 0
    else:
        x = math.atan2(R[2, 1], R[2, 2])
        y = math.atan2(-R[2, 0], sy)
        z = math.atan2(R[1, 0], R[0, 0])
    
    return np.array([x, y, z])


# function to check if angle is within a range
def is_within_range(angle, target, margin):
    # Normalize both angles within the range 0 to 360 degrees
    angle = angle % 360
    target = target % 360

    # Calculate the lower and upper bounds of the range
    lower_bound = (target - margin) % 360
    upper_bound = (target + margin) % 360

    # Check if the angle falls within the range
    if lower_bound <= upper_bound:
        return lower_bound <= angle <= upper_bound
    else:
        # Check if the range crosses the 0/360-degree boundary
        return angle >= lower_bound or angle <= upper_bound


# function to calculate the shortest turn angle
def shortest_turn_angle(angle_robot, angle_station):
    # Calculate the absolute difference
    absolute_difference = abs(angle_station - angle_robot)

    # Ensure the angle is within the 0-360 degree range
    absolute_difference = absolute_difference % 360

    # Calculate the shortest turn, considering the maximum turn angle
    if absolute_difference <= 90:
        angle_2_motor = absolute_difference
    elif 360 - absolute_difference <= 90:
        angle_2_motor = 360 - absolute_difference
    else:
        angle_2_motor = 180 - absolute_difference

    return angle_2_motor
    

# function for fidning angle of station relative to robot
def station2robot_angle(x_current, y_current, x_value, y_value):
    
    angle_rad = math.atan2(x_value - x_current, y_value - y_current)

    # Convert the angle from radians to degrees
    relative_angle = math.degrees(angle_rad)

    if relative_angle < 0:
        relative_angle = 360 - abs(relative_angle)
    
    return relative_angle


# function to calculate the distance and angle to the next station
def current_station(x_current, y_current, x_value, y_value, current_angle):

    req_distance = np.sqrt((x_value - x_current)**2 + (y_value - y_current)**2)

    relative_angle = station2robot_angle(x_current, y_current, x_value, y_value)
    
    # Check if forward or backward motion is required
    result = is_within_range(current_angle, relative_angle, 90)
    
    # setting direction of motion
    if result == False:
        req_distance = -req_distance

    angle_2_motor = shortest_turn_angle(current_angle, relative_angle)

    if abs(relative_angle - current_angle) < 180:
        if relative_angle > current_angle:
            turn_direction = 'Counterclockwise'
        else:
            turn_direction = 'Clockwise'
    else:
        if relative_angle > current_angle:
            turn_direction = 'Clockwise'
        else:
            turn_direction = 'Counterclockwise'

    if turn_direction == 'Counterclockwise' and result == True:
        angle_2_motor = -angle_2_motor
    elif turn_direction == 'Clockwise' and result == False:
        angle_2_motor = -angle_2_motor

    print('Direction and Distance:', req_distance)
    print('Direction and angle:', angle_2_motor)

    return req_distance, angle_2_motor


# function to calculate positions of all stations
def initial_pos_finder(accumulated_tvecs):
    # Calculate the initial average tvecs
    initial_average_tvecs = {}
    for marker_id, tvecs_list in accumulated_tvecs.items():
        if tvecs_list:
            initial_average_tvecs[marker_id] = np.mean(tvecs_list, axis=0)
    
    return initial_average_tvecs
        

# function to calculate the optimum order of stations
def optimum_order(robot_id, ids, initial_average_tvecs):
    # Define the marker ids in the order we want to visit them
    start_id = robot_id
    stations = ids.flatten()
    stations = stations[stations != robot_id]

    # Generate all possible permutations of the remaining marker IDs
    permutations = itertools.permutations(stations)

    # Initialise variables
    best_distance = float('inf')

    # Loop over all permutations and calculate total distance
    for permutation in permutations:
        # Add start ID to permutation
        route = [start_id] + list(permutation)
        
        # Initialize a list to store the coordinates for each ID in the route
        coordinates = []

        # Extract the first two coordinates for each marker ID in the route
        for id in route:
            tvecs_array = initial_average_tvecs[id]
            x_value = tvecs_array[0][0][0]
            y_value = tvecs_array[0][0][1]
            coordinates.append((x_value, y_value))

        # Calculate the distance between each pair of coordinates
        distances = np.sqrt(np.sum(np.diff(coordinates, axis=0)**2, axis=1))

        # Calculate the total distance of the route
        total_distance = np.sum(distances)

        # Update best route and distance if current route is better
        if total_distance < best_distance:
            optimal_route = list(permutation)
            best_route = route
            best_distance = total_distance

    return optimal_route, best_distance, best_route


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


# Function to expand one obstacle for robot clearance
def expander(obstacle, clearance):
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


# Function to expand obstacles' array for robot clearance
def obstacle_expander(obstacle, clearance):
    real_world_data = []
    for array in obstacle:
        real_world_array = [[pixel_to_real(x, y) for x, y in corner] for corner in array]
        real_world_data.append(real_world_array)

    expanded_obstacles = []
    for real_world_array in real_world_data:
        # Use a list comprehension to extract the tuples and create a list of tuples
        flattened_obstacles = [item[0] for item in real_world_array]
        expanded_obstacle = expander(flattened_obstacles, clearance)
        expanded_obstacles.append(expanded_obstacle)
    
    return expanded_obstacles


# function to calculate distance between two points
def calculate_distance(point1, point2):
    x1, y1 = point1
    x2, y2 = point2
    return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)


# function to check if path from robot to station is clear
def obstructed(expanded_obstacles, x_current, y_current, x_value, y_value):
    intersection1 = None
    intersection2 = None
    optimal_intersection = None
    robo_coords = (x_current, y_current)
    station_coords = (x_value, y_value)
    for obstacle in expanded_obstacles:
        for i in range(4):
            corner1 = obstacle[i]
            corner2 = obstacle[(i + 1) % 4]

        if do_lines_intersect(corner1, corner2, robo_coords, station_coords):
            relative_angle = station2robot_angle(x_current, y_current, x_value, y_value)
            optimal_intersection = intersection_checker(robo_coords, station_coords, obstacle, relative_angle)
            break
    return optimal_intersection
    


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


# function to get coords of ArUco from its ID
def get_coords(id, initial_average_tvecs):
    tvecs_array = initial_average_tvecs[id]
    x_value = tvecs_array[0][0][0]
    y_value = tvecs_array[0][0][1]

    return x_value, y_value
    

# Function to filter and find obstacles
def obstacle_finder(all_accumulated_approx, route, tolerance=20):

    grouped_shapes = []
    # Iterate through all_accumulated_approx
    for approx in all_accumulated_approx:
        # Get the first row of coordinates as a tuple
        first_row = tuple(approx[0][0])

        # Check if the first_row is within tolerance of any existing group
        found_group = False

        for group in grouped_shapes:
            for existing_approx in group:
                x_diff = abs(first_row[0] - existing_approx[0][0][0])
                y_diff = abs(first_row[1] - existing_approx[0][0][1])
                if x_diff <= tolerance and y_diff <= tolerance:
                    group.append(approx)
                    found_group = True
                    break

        # If no matching group is found, create a new group
        if not found_group:
            grouped_shapes.append([approx])
    
    average_matrix = []
    for group in grouped_shapes:
        # Calculate the average across the 6 matrices
        average = np.mean(group, axis=0)
        average_matrix.append(average)

    average_matrix = np.array(average_matrix)
    # Convert the entire average_matrix to the integer data type
    average_matrix = average_matrix.astype(int)

    obstacles = []
    # Filter out 
    for point in average_matrix:
        score = 0
        for row in point:
            for x, y in row:
                for ids in route:
                    aruco_x, aruco_y = get_coords(ids, initial_average_tvecs)

                    x_real, y_real = pixel_to_real(x, y)

                    if not (x_real > aruco_x -64 and x_real < aruco_x + 64 and y_real > aruco_y -64 and y_real < aruco_y + 64):
                        score += 1            

        if score == 4* len(route):
            obstacles.append(point)


    obstacles = np.array(obstacles)

    return obstacles



# Select the external camera (1) that is connected to the laptop
cap = cv2.VideoCapture(1)
width = 1280
height = 720
cap.set(3, width)  # 3 corresponds to CV_CAP_PROP_FRAME_WIDTH
cap.set(4, height)  # 4 corresponds to CV_CAP_PROP_FRAME_HEIGHT

# Read and store the calibration information from Sample_Calibration
Camera=np.load('webcam_calib.npz') #Load the camera calibration values
CM=Camera['CM'] #camera matrix
dist_coef=Camera['dist_coef']# distortion coefficients from the camera

# Load the ArUco Dictionary Dictionary 4x4_50 and set the detection parameters
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
parameters = aruco.DetectorParameters()

# Initialise variables
robot_id = 46 # ArUco marker ID of the robot
aruco_size = 76 # Size of the ArUco marker in mm
prev_rot = None
prev_x = None
prev_y = None
rot_current = None
x_current = None
y_current = None
angle_2_motor = 9999
req_distance = float('inf')
robot_radius = 100 # mm
station_index = 0 # first station index


# Define the rotation and translation thresholds
rotation_threshold = 5.0  # Set to a suitable angle in degrees
translation_threshold = 2.0 # Set to a suitable length in mm


# Initialise necessary vectors/ arrays
accumulated_tvecs = {marker_id: [] for marker_id in range(1, 51)}  # Assuming marker IDs range from 1 to 50
all_accumulated_approx = []
average_tvecs = {}  # To store the average tvecs for each marker ID


# Define the number of frames to capture for averaging
num_frames_to_capture = 10  
frame_count = 0  


#Create one opencv named windows
cv2.namedWindow("frame-image", cv2.WINDOW_AUTOSIZE)
#Position the windows next to eachother
cv2.moveWindow("frame-image",0,100)


# Initialisation loop for ArUco marker detection
while frame_count < num_frames_to_capture:
    # Capture current frame from the camera
    ret, frame = cap.read()
    
    # Convert the image from the camera to Grey scale
    grey = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(grey, (5, 5), 0)
    edges = cv2.Canny(blur,200,300)
    
    # Detect the markers in the image
    corners, ids, rP = aruco.detectMarkers(blur, aruco_dict)

    if ids is not None:
        for marker_id in ids.flatten():
            index = list(ids).index(marker_id)
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners[index], 76, CM, dist_coef)
            accumulated_tvecs[marker_id].append(tvecs)

    frame_count += 1


# Initialisation loop for obstacle detection
frame_count = 0  # Re-initialize frame count
while frame_count < num_frames_to_capture:
    
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
            
        all_accumulated_approx.append(approx)

        # Stop the performance counter
    end = time.perf_counter()
    
    frame_count += 1


all_accumulated_approx = np.array(all_accumulated_approx)
initial_average_tvecs = initial_pos_finder(accumulated_tvecs)
# Find optimal route
optimal_route, best_distance, route = optimum_order(robot_id, ids, initial_average_tvecs)
# Find obstacles
obstacles = obstacle_finder(all_accumulated_approx, route)
# find clearnace from obstacle
clearance = 20 # Should be = to robot radius + safety margin
expanded_obstacles = obstacle_expander(obstacles, clearance)
print(expanded_obstacles)

print("Best route:", optimal_route)
print("Total distance:", best_distance)
print('Station coordinates found. Starting tracking...')


# Monitoring loop for ArUco marker detection
while(True):
    
    # Start the performance clock
    start = time.perf_counter()
    
    # Capture current frame from the cameraq
    ret, frame = cap.read()
    
    # Convert the image from the camera to Gray scale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    # Create Blurred image
    blur = cv2.GaussianBlur(gray,(5,5),0)

    # Detect the markers in the image
    corners, ids, rP = aruco.detectMarkers(blur, aruco_dict)

    if ids is not None:
        # Labeling station coordinates
        for marker_id in ids.flatten():
            index = list(ids).index(marker_id)
            if marker_id != 46:
                # Extract the corner positions of the current marker
                marker_corners = corners[index][0]
                # Calculate the center point of the marker
                center_x = int(sum(marker_corners[:, 0]) / 4)
                center_y = int(sum(marker_corners[:, 1]) / 4)

                x_value, y_value = get_coords(marker_id, initial_average_tvecs)

                label_text = f"ID: {marker_id} | X: {x_value:.2f} | Y: {y_value:.2f}"

                # Draw the label next to the marker's ID
                cv2.putText(frame, label_text, (center_x, center_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    # Update labels for robot with current coordinates and angle
    if ids is not None and 46 in ids:
        # Continually track ArUco marker ID 46
        index_robot = list(ids).index(robot_id)
        rvecs, tvecs, _objPoints = aruco.estimatePoseSingleMarkers(corners[index_robot], aruco_size, CM, dist_coef)
        x_current = tvecs[0, 0, 0]
        y_current = tvecs[0, 0, 1]
        rot = cv2.Rodrigues(np.asarray(rvecs))
        rot = rot[0]
        EulerAngle = matrix_to_euler_angles(rot)
        current_angle = abs(float(np.degrees(EulerAngle[2]))) # as robot is symmetrical
        angle4turn = float(np.degrees(EulerAngle[2]))

        if angle4turn < 0:
            angle4turn = float((180 - abs(current_angle)) + 180)

        angle4turn = (360 - angle4turn) % 360
        
        upper_bound = angle_2_motor + rotation_threshold
        lower_bound = angle_2_motor - rotation_threshold

        # Check if the robot has reached the station
        if abs(req_distance) <= robot_radius * 1.1:
            print('Hello world')
            # initiate station interaction sequence
            # station_index += 1
        
        x_value, y_value = get_coords(optimal_route[station_index], initial_average_tvecs)
        

        # Check if any obstacles between robot and station
        optimal_intersection = obstructed(expanded_obstacles, x_current, y_current, x_value, y_value)
        
        # Check if robot is off course to desried coordinate
        if current_angle < upper_bound or current_angle > lower_bound:
            # Check if obstacle has been detected
            if optimal_intersection is None:
                # Choose optimum intersection as next objective
                req_distance, angle_2_motor = current_station(x_current, y_current, x_value, y_value, current_angle)
            else:
                x_opt = optimal_intersection[0]
                y_opt = optimal_intersection[1]
                req_distance, angle_2_motor = current_station(x_current, y_current, x_opt, y_opt, current_angle)
        # client.publish("Obot/angle", str(angle_2_motor))


        # Check if robot has reached desired coordinate
        if optimal_intersection is None:
            if abs(req_distance) <= robot_radius * 1.1:
                # initiate station interaction sequence
                station_index += 1
                x_value, y_value = get_coords(optimal_route[station_index], initial_average_tvecs)
                req_distance, angle_2_motor = current_station(x_current, y_current, x_value, y_value, current_angle)
        else:
            # Check if robot has reached desired coordinate with tighter tolerance
            if abs(req_distance) <= 50: # mm (value to be decided)
                optimal_intersection = None
            print(optimal_intersection)
            # pass
        
            
        # Extract the corner positions of the current marker
        marker_corners = corners[index_robot][0]

        # Calculate the center point of the marker
        center_x = int(sum(marker_corners[:, 0]) / 4)
        center_y = int(sum(marker_corners[:, 1]) / 4)

        label_text = f"ID: {marker_id} | X: {x_current:.2f} | Y: {y_current:.2f} | Angle: {angle4turn:.2f}"

        # Draw the label next to the marker's ID
        cv2.putText(frame, label_text, (center_x, center_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

    # draw obstacles
    for shape in obstacles:
        # Draw the contours on the frame
        try:
            cv2.drawContours(frame, [shape], 0, (0, 255, 0), 2)
        except:
            pass
    
    # Display the original frame in a window
    cv2.imshow('frame-image',frame)

    # Stop the performance counter
    end = time.perf_counter()
    
    # Print to console the exucution time in FPS (frames per second)
    # print ('{:4.1f}'.format(1/(end - start)))

    # If the button q is pressed in one of the windows 
    if cv2.waitKey(20) & 0xFF == ord('q'):
        # Exit the While loop
        break


# When everything done, release the capture
cap.release()
# close all windows
cv2.destroyAllWindows()
# exit the kernel
exit(0)