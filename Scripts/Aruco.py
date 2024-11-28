# This is the vision library OpenCV
import cv2
import cv2.aruco as aruco
# This is a library for mathematical functions for python (used later)
import numpy as np
# This is a library to get access to time-related functionalities
import time

# Read and store the calibration information from Sample_Calibration
Camera=np.load('webcam_calib.npz') #Load the camera calibration values
CM=Camera['CM'] #camera matrix
dist_coef=Camera['dist_coef']# distortion coefficients from the camera

# Load the ArUco Dictionary Dictionary 4x4_50 and set the detection parameters
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_250)
parameters = aruco.DetectorParameters()


# Select the first camera (0) that is connected to the machine
# in Laptops should be the build-in camera
cap = cv2.VideoCapture(1)

# Set the width and heigth of the camera to 640x480
cap.set(3, 1280)
cap.set(4, 720)

#Create one opencv named windows
cv2.namedWindow("frame-image", cv2.WINDOW_AUTOSIZE)

#Position the windows next to eachother
cv2.moveWindow("frame-image",0,100)

# Execute this continuously
while(True):
    
    # Start the performance clock
    start = time.perf_counter()
    
    # Capture current frame from the camera
    ret, frame = cap.read()
    
    # Convert the image from the camera to Gray scale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, rP = aruco.detectMarkers(gray, aruco_dict)
    rvecs,tvecs,_objPoints = aruco.estimatePoseSingleMarkers(corners,65,CM,dist_coef)

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