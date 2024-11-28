import cv2

# Initialize the webcam
cap = cv2.VideoCapture(1)  # 0 is the default camera (you can change this if you have multiple cameras)
# Disable autofocus (autofocus is enabled by default on most cameras)
# cap.set(cv2.CAP_PROP_AUTOFOCUS, 1)

width = 1280
height = 720
cap.set(3, width)  # 3 corresponds to CV_CAP_PROP_FRAME_WIDTH
cap.set(4, height)  # 4 corresponds to CV_CAP_PROP_FRAME_HEIGHT

# Initialize image counter
i = 0

while True:
    ret, frame = cap.read()  # Read a frame from the camera

    # Show the current frame in a window
    cv2.imshow('Webcam', frame)

    key = cv2.waitKey(1)  # Wait for a key event for 1 millisecond

    if key == 32:  # Code for spacebar
        filename = f'Scripts/board_{i}.jpg'
        cv2.imwrite(filename, frame)
        print(f'Saved {filename}')
        i += 1

    elif key == 27:  # 27 is the ASCII code for the 'Esc' key
        break

# Release the webcam and close all OpenCV windows
cap.release()
cv2.destroyAllWindows()
