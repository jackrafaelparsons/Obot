import matplotlib.path as mpath
import matplotlib.patches as mpatches

def is_shape_in_path(shape, path_line):
    # Extract the x and y coordinates from the shape
    shape_coords = [point[0] for point in shape]
    
    # Create a Path object for the path line
    path = mpath.Path(path_line)
    
    # Create a PathPatch for the shape
    shape_patch = mpatches.PathPatch(mpath.Path(shape_coords), fill=False)
    
    # Check if any point on the path line is within the shape
    for point in path_line:
        if shape_patch.contains_point(point, radius=0.0):
            return True
    
    return False

# Your coordinates and path line
robo_coords = (353.17, -484)
station_coords = (384, -184)
path_line = [robo_coords, station_coords]

# Your array of shapes
shapes = [
    [[[119, 681], [111, 573], [0, 631], [111, 573]],
     [[618, 130], [618, 234], [649, 235], [653, 132]]]
]

# Check if each shape is in the way of the path
for shape in shapes:
    if is_shape_in_path(shape, path_line):
        print("Shape is in the way")
    else:
        print("Shape is not in the way")
