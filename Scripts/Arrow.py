import cv2
import numpy as np

# Create an image (frame) to draw the arrowed line on
frame = np.zeros((500, 500, 3), dtype=np.uint8)

# Define the starting and ending points of the line
start_point = (100, 100)
end_point = (400, 400)

# Specify the color of the arrowed line (BGR format)
color = (0, 0, 255)  # Red

# Draw the arrowed line on the frame
cv2.arrowedLine(frame, start_point, end_point, color, thickness=2, tipLength=0.2)

# Display the frame with the arrowed line
cv2.imshow("Arrowed Line", frame)
cv2.waitKey(0)
cv2.destroyAllWindows()
