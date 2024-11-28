import cv2
import cv2.aruco as aruco
import numpy as np
import time
import math
import itertools


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

def current_station(next_station, x_current, y_current, current_angle, angle4turn, initial_average_tvecs):

    tvecs_array = initial_average_tvecs[next_station]
    # Extract the first two numbers from the array
    x_value = tvecs_array[0][0][0]
    y_value = tvecs_array[0][0][1]

    req_distance = np.sqrt((x_value - x_current)**2 + (y_value - y_current)**2)

    # Calculate the angle (in radians) from the robot to the station
    angle_rad = math.atan2(x_value - x_current, y_value - y_current)

    # Convert the angle from radians to degrees
    relative_angle = math.degrees(angle_rad)

    if relative_angle < 0:
        relative_angle = 360 - abs(relative_angle)
    
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
prev_rot = None
prev_x = None
prev_y = None
rot_current = None
x_current = None
y_current = None
angle_2_motor = 9999
req_distance = float('inf')
robot_radius = 100 # actually 200mm
station_index = 0
rotation_threshold = 5.0  # Set to a suitable angle in degrees
translation_threshold = 2.0 # Set to a suitable length in mm

# Define the number of frames to capture for averaging
num_frames_to_capture = 10  # Change this as needed

# Initialize variables for accumulating pose information
accumulated_tvecs = {marker_id: [] for marker_id in range(1, 51)}  # Assuming marker IDs range from 1 to 50

frame_count = 0  # Initialize frame count
average_tvecs = {}  # To store the average tvecs for each marker ID


#Create one opencv named windows
cv2.namedWindow("frame-image", cv2.WINDOW_AUTOSIZE)

#Position the windows next to eachother
cv2.moveWindow("frame-image",0,100)


# Initialization loop
while frame_count < num_frames_to_capture:
    # Capture current frame from the camera
    ret, frame = cap.read()
    
    # Convert the image from the camera to Gray scale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    
    # Detect the markers in the image
    corners, ids, rP = aruco.detectMarkers(blur, aruco_dict)

    if ids is not None:
        for marker_id in ids.flatten():
            index = list(ids).index(marker_id)
            rvecs, tvecs, _objPoints = aruco.estimatePoseSingleMarkers(corners[index], 76, CM, dist_coef)
            accumulated_tvecs[marker_id].append(tvecs)

    frame_count += 1

# Calculate the initial average tvecs
initial_average_tvecs = {}
for marker_id, tvecs_list in accumulated_tvecs.items():
    if tvecs_list:
        initial_average_tvecs[marker_id] = np.mean(tvecs_list, axis=0)
        # print(f'{marker_id}:{initial_average_tvecs[marker_id]}')
        tvecs_array = initial_average_tvecs[marker_id]
        # Extract the first two numbers from the array
        x_value = tvecs_array[0][0][0]
        y_value = tvecs_array[0][0][1]

print(initial_average_tvecs)

# Define the marker ids in the order we want to visit them
start_id = 46
stations = ids.flatten()
stations = stations[stations != 46]

# Generate all possible permutations of the remaining marker IDs
permutations = itertools.permutations(stations)

# Initialize variables to store best route and distance
best_route = None
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
        best_route = route
        optimal_route = list(permutation)
        best_distance = total_distance


print("Best route:", optimal_route)
print("Total distance:", best_distance)

print('Station coordinates found. Starting tracking...')
# print(initial_average_tvecs)
    
# Execute this continuously
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
                tvecs_array = initial_average_tvecs[marker_id]
                # Extract the corner positions of the current marker
                marker_corners = corners[index][0]

                # Calculate the center point of the marker
                center_x = int(sum(marker_corners[:, 0]) / 4)
                center_y = int(sum(marker_corners[:, 1]) / 4)

                # Extract the first two numbers from the array
                x_value = tvecs_array[0][0][0]
                y_value = tvecs_array[0][0][1]

                label_text = f"ID: {marker_id} | X: {x_value:.2f} | Y: {y_value:.2f}"

                # Draw the label next to the marker's ID
                cv2.putText(frame, label_text, (center_x, center_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    # Update labels for robot with current coordinates and global angle
    if ids is not None and 46 in ids:
        # Continually track ArUco marker ID 46
        index_46 = list(ids).index(46)
        rvecs, tvecs, _objPoints = aruco.estimatePoseSingleMarkers(corners[index_46], 76, CM, dist_coef)
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

        if abs(req_distance) <= robot_radius * 1.1:
            print('Hello world')
            # initiate station interaction sequence
            # station_index += 1

        if current_angle < upper_bound or current_angle > lower_bound:

            if best_route[station_index] == 46:
                station_index += 1

            req_distance, angle_2_motor = current_station(best_route[station_index], x_current, y_current, 
                                                      current_angle, angle4turn, initial_average_tvecs)
        
        else:
            print('on course for collision')
            

        # Extract the corner positions of the current marker
        marker_corners = corners[index_46][0]

        # Calculate the center point of the marker
        center_x = int(sum(marker_corners[:, 0]) / 4)
        center_y = int(sum(marker_corners[:, 1]) / 4)

        label_text = f"ID: {marker_id} | X: {x_current:.2f} | Y: {y_current:.2f} | Angle: {current_angle:.2f}"

        # Draw the label next to the marker's ID
        cv2.putText(frame, label_text, (center_x, center_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)


    try:
        frame = cv2.drawFrameAxes(frame, CM, dist_coef, rvecs, tvecs, 10)
    except:
        frame = aruco.drawDetectedMarkers(frame, corners, ids)
    
    
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