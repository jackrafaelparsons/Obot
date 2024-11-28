import numpy as np

# Constants for the linear mapping
x_pixel_min = 538
x_pixel_max = 844.75  # Assuming frame_width is the width of your image frame
x_real_min = 261.52
x_real_max = 436.41

# Constants for the linear mapping
y_pixel_min = 438.5
y_pixel_max = 235.75# Assuming frame_height is the height of your image frame
y_real_min = -260.17
y_real_max = -391.69

x = 761
y = 349

# Calculate the x_real coordinate
x_real = x_real_min + (x_real_max - x_real_min) * (x - x_pixel_min) / (x_pixel_max - x_pixel_min)
# Calculate the y_real coordinate
y_real = y_real_min + (y_real_max - y_real_min) * (y - y_pixel_min) / (y_pixel_max - y_pixel_min)


print('x', x_real,'y', y_real)

robot_pos = np.array([570, -346])
station_pos = np.array([189.64, -354])
obstacle_corners = np.array([[370, -385], [390, -385], [371, -319], [389, -318]])