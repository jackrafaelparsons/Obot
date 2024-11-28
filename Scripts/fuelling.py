import cv2
import cv2.aruco as aruco
import numpy as np
import time
import math
import itertools

# Select the external camera (1) that is connected to the laptop
cap = cv2.VideoCapture(1)
width = 1280
height = 720
cap.set(3, width)  # 3 corresponds to CV_CAP_PROP_FRAME_WIDTH
cap.set(4, height)  # 4 corresponds to CV_CAP_PROP_FRAME_HEIGHT

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

def current_station(next_station, x_current, y_current, current_angle, initial_average_tvecs):

    tvecs_array = initial_average_tvecs[next_station]
    # Extract the first two numbers from the array
    x_value = tvecs_array[0][0][0]
    y_value = tvecs_array[0][0][1]

    req_distance = np.sqrt((x_value - x_current)**2 + (y_value - y_current)**2)

    # Calculate the angle (in radians) from the robot to the station
    angle_rad = math.atan2(x_value - x_current, y_value - y_current)

    # Convert the angle from radians to degrees
    relative_angle = math.degrees(angle_rad)

    # Adjust the angle to ensure it's within the range [0, 360] degrees
    if relative_angle < 0:
        relative_angle += 360

    if relative_angle > 90:
        turn_diretion = 'Counter clockwise'
    else:
        turn_diretion = 'Clockwise'

    print(turn_diretion)
    
    # req_distance = req_distance - dimension of the robot 
    angle_2_motor = relative_angle - current_angle

    print(angle_2_motor)

    return req_angle, req_distance, angle_2_motor, turn_diretion


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
fuel_level = 100 #%
critical_fuel_level = 20 #%
fuel_station_id = 40 # ID of the fuel station
angle_2_motor = 9999
req_distance = float('inf')
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
        best_distance = total_distance

print("Best route:", best_route)
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

    # Update labels for ID 46
    if ids is not None and 46 in ids:
        # Continually track ArUco marker ID 46
        index_46 = list(ids).index(46)
        rvecs, tvecs, _objPoints = aruco.estimatePoseSingleMarkers(corners[index_46], 76, CM, dist_coef)
        x_current = tvecs[0, 0, 0]
        y_current = tvecs[0, 0, 1]
        rot = cv2.Rodrigues(np.asarray(rvecs))
        rot = rot[0]
        EulerAngle = matrix_to_euler_angles(rot)
        current_angle = float(np.degrees(EulerAngle[2]))

        if current_angle < 0:
            current_angle = float((180 - abs(current_angle)) + 180)

        if fuel_level <= critical_fuel_level:

            print('fuel level is critical, heading to fuel station')
            station_id = fuel_station_id # to set next station as fuel station
            angle_2_motor = 9999 # To interupt current station function
        
        else:
            station_id = best_route[station_index] # carry on as normal

        if angle_2_motor - rotation_threshold <= current_angle <= angle_2_motor + rotation_threshold:

            # Ensuring that robot is not next station 
            if best_route[station_index] == 46:
                station_index += 1

            req_angle, req_distance, angle_2_motor = current_station(station_id, x_current, y_current, 
                                                                     current_angle, initial_average_tvecs)
        
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