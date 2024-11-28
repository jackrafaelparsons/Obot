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
def is_within_range(angle, target):

    # Normalize both angles within the range 0 to 360 degrees
    angle = angle % 360
    target = target % 360
    margin = 90

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
def current_station(x_current, y_current, x_value, y_value, current_angle, angle4turn):

    req_distance = np.sqrt((x_value - x_current)**2 + (y_value - y_current)**2)

    relative_angle = station2robot_angle(x_current, y_current, x_value, y_value)
    
    # Check if forward or backward motion is required
    result = is_within_range(angle4turn, relative_angle)
    
    # setting direction of motion
    if result == False:
        req_distance = -req_distance

    angle_2_motor = shortest_turn_angle(angle4turn, relative_angle)

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

    # print('Direction and Distance:', req_distance)
    # print('Direction and angle:', angle_2_motor)

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



# function to calculate distance between two points
def calculate_distance(point1, point2):
    x1, y1 = point1
    x2, y2 = point2
    return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
    


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
    

# Function to send angle and direction to robot
def send_angle(angle2motor):

    client.publish("Obot/angle", str(abs(angle_2_motor))) # Sends value of angle to motor

    if angle2motor < 0:
        direction = 4 # Counter clockwise
    else:
        direction = 3 # # Clockwise
    
    client.publish("Obot/movement", str(direction)) # Send value for movement
    return direction


# Function to send direction to robot
def send_direction(req_distance):
    if req_distance < 0:
        direction = 2
    else:
        direction = 1
    client.publish("Obot/movement", str(direction)) # Send value for movement
    return direction


# Function to send direction to robot
def send_at_station():
    client.publish("Obot/FeedbackLED", str(1)) # Turn LED on


# Create the mqtt client object
client = mqtt.Client()

# Set the username and password
client.username_pw_set("student",password="HousekeepingGlintsStreetwise")

# Connect to the server using a specific port with a timeout delay (in seconds)
client.connect("fesv-mqtt.bath.ac.uk",31415,60)

# Start the client to enable the above events to happen
client.loop_start()

# Shared variable to store the MQTT message
confirm_angle = None
direction = 0


# Select the external camera (1) that is connected to the laptop
cap = cv2.VideoCapture(1)
width = 1280
height = 720
cap.set(3, width)  # 3 corresponds to CV_CAP_PROP_FRAME_WIDTH
cap.set(4, height)  # 4 corresponds to CV_CAP_PROP_FRAME_HEIGHT
# cap.set(cv2.CAP_PROP_EXPOSURE, 0.15)  # Enable auto-exposure

# Read and store the calibration information from Sample_Calibration
Camera=np.load('gantry_calib.npz') #Load the camera calibration values
CM=Camera['CM'] #camera matrix
dist_coef=Camera['dist_coef']# distortion coefficients from the camera

# Load the ArUco Dictionary Dictionary 4x4_50 and set the detection parameters
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_250)
parameters = aruco.DetectorParameters()

# Initialise variables
robot_id = 190 # ArUco marker ID of the robot
aruco_size = 90 # Size of the ArUco marker in mm
x_current = None
y_current = None
angle_2_motor = 9999
req_distance = float('inf')
robot_radius = 200 # mm
ramp_threshold = 150 # mm
ramp_angle = 18 # degrees
station_index = 0 # first station index

# Define the rotation and translation thresholds
rotation_threshold = 25.0  # Set to a suitable angle in degrees

# Initialise necessary vectors/ arrays
accumulated_tvecs = {marker_id: [] for marker_id in range(1, 251)}  # Assuming marker IDs range from 1 to 50
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
    corners1, ids, rP = aruco.detectMarkers(blur, aruco_dict)

    if ids is not None:
        for marker_id in ids.flatten():
            index = list(ids).index(marker_id)
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners1[index], 76, CM, dist_coef)
            accumulated_tvecs[marker_id].append(tvecs)

    frame_count += 1

initial_average_tvecs = initial_pos_finder(accumulated_tvecs)
# Find optimal route
optimal_route, best_distance, route = optimum_order(robot_id, ids, initial_average_tvecs)


print("Best route:", optimal_route)
print("Total distance:", best_distance)
print('Station coordinates found. Starting tracking...')


# Enter main loop
start_time = time.time()
while(True):

    current_time = time.time()
    elapsed_time = current_time - start_time

    # Start the performance clock
    start = time.perf_counter()
    
    # Capture current frame from the cameraq
    ret, frame = cap.read()
    
    # Convert the image from the camera to Gray scale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    # Create Blurred image
    blur = cv2.GaussianBlur(gray,(5,5),0)

    # Apply adaptive thresholding
    adaptive_threshold = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_MEAN_C, 
                                            cv2.THRESH_BINARY, 37, 11)

    # Detect the markers in the image
    corners, id, rP = aruco.detectMarkers(adaptive_threshold, aruco_dict)

    # Labeling station coordinates
    for marker_id in ids.flatten():
        index = list(ids).index(marker_id)
        if marker_id != robot_id:
            # Extract the corner positions of the current marker
            marker_corners = corners1[index][0]
            # Calculate the center point of the marker
            center_x = int(sum(marker_corners[:, 0]) / 4)
            center_y = int(sum(marker_corners[:, 1]) / 4)

            x_value, y_value = get_coords(marker_id, initial_average_tvecs)

            label_text = f"ID: {marker_id} | X: {x_value:.2f} | Y: {y_value:.2f}"

            # Draw the label next to the marker's ID
            cv2.putText(frame, label_text, (center_x, center_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    # Update labels for robot with current coordinates and angle
    if id is not None and robot_id in id:
        # Continually track robot's ArUco marker ID 
        index_robot = list(id).index(robot_id)
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
        
        x_value, y_value = get_coords(optimal_route[station_index], initial_average_tvecs)

        # find coordinates of next point
        req_distance, angle_2_motor = current_station(x_current, y_current, x_value, y_value, current_angle, angle4turn)
        
        # Control logic for motor
        if direction == 0:
            if abs(req_distance) <= robot_radius * 1.1:
                    direction = 0
                    client.publish("Obot/movement", str(direction)) # Send value for movement
                    i = 0
                    for i in range(4):
                        client.publish("Obot/FeedbackLED", "1")
                        time.sleep(0.5)
                        i = i + 1
                    station_index += 1
                    optimal_intersection = None
            elif abs(angle_2_motor) <= rotation_threshold:
                direction = send_direction(req_distance)
            else:
                direction = send_angle(angle_2_motor)
        
        # Allow 4 seconds to pass from turning
        if direction == 3 or direction == 4:
            # time.sleep(5)
            if elapsed_time >= 5:
                start_time = current_time  # Reset the start time
                direction = 0
                client.publish("Obot/movement", str(direction))
        
        if direction == 1 or direction == 2:
            if abs(req_distance) <= robot_radius * 1.1:
                    direction = 0
                    client.publish("Obot/movement", str(direction)) # Send value for movement
                    i = 0
                    for i in range(4):
                        client.publish("Obot/FeedbackLED", "1")
                        time.sleep(0.5)
                        i = i + 1
                    station_index += 1

        # Extract the corner positions of the robot
        marker_corners = corners[index_robot][0]

        # Calculate the center point of the marker
        center_x = int(sum(marker_corners[:, 0]) / 4)
        center_y = int(sum(marker_corners[:, 1]) / 4)

        label_text = f"Obot | Dist: {req_distance:.2f} | Angle: {angle_2_motor:.2f}"

        # Draw the label next to the marker's ID
        cv2.putText(frame, label_text, (center_x, center_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
    
    current_markerID = optimal_route[station_index]
    next_station_corners = corners1[list(ids).index(current_markerID)][0]
    # Calculate the center point of the marker
    end_x = int(sum(next_station_corners[:, 0]) / 4)
    end_y = int(sum(next_station_corners[:, 1]) / 4)

    robo_coords_pixels = (center_x, center_y)
    goal_coords_pixels = (end_x, end_y)

    cv2.arrowedLine(frame, robo_coords_pixels, goal_coords_pixels, (0, 0, 255), thickness=2, tipLength=0.2)

    # Display the original frame in a window
    cv2.imshow('frame-image',frame)

    # Stop the performance counter
    end = time.perf_counter()
    
    # Display the original frame in a window
    cv2.imshow('frame-image',frame)

    # If the button q is pressed in one of the windows 
    if cv2.waitKey(20) & 0xFF == ord('q'):
        # Exit the While loop
        break


direction = 0
client.publish("Obot/movement", str(direction))

# When everything done, release the capture
cap.release()
# close all windows
cv2.destroyAllWindows()
# exit the kernel
exit(0)