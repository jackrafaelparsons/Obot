# function to transform pixels to real world coords
def pixel_to_real(x, y):

    # Pre-determined constants for the linear mapping
    x_real = (x * 0.939) - 330.3
    y_real = -720 + (y * 0.891) + 121.2

    return x_real, y_real