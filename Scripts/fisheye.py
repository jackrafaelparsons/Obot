import cv2
import numpy as np

def fisheye_effect(image_path, output_path, strength=0.5):
    # Read the input image
    img = cv2.imread(image_path)

    # Get image dimensions
    height, width = img.shape[:2]

    # Calculate the center of the image
    center_x, center_y = width // 2, height // 2

    # Generate a grid of coordinates
    y, x = np.ogrid[:height, :width]

    # Calculate the distance from the center for each pixel
    radius = np.sqrt((x - center_x) ** 2 + (y - center_y) ** 2)

    # Apply fisheye distortion
    theta = np.arctan2(y - center_y, x - center_x)
    radius = radius * np.exp(-strength * (radius / (width / 2))**2)

    # Map the distorted coordinates back to the image
    x_distorted = radius * np.cos(theta) + center_x
    y_distorted = radius * np.sin(theta) + center_y

    # Interpolate the pixel values from the distorted coordinates
    distorted_img = cv2.remap(img, x_distorted.astype(np.float32), y_distorted.astype(np.float32), interpolation=cv2.INTER_LINEAR)

    # Save the output image
    cv2.imwrite(output_path, distorted_img)

if __name__ == "__main__":
    input_image_path = "images/Picture5.png"  # Change this to the path of your input image
    output_image_path = "output_fisheye.jpg"  # Change this to the desired output path

    fisheye_effect(input_image_path, output_image_path, strength=0.25)
