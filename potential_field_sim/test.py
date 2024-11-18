import numpy as np
import matplotlib.pyplot as plt
from scipy.ndimage import rotate

def draw_rotated_rectangle(array, center, dimensions, angle):
    """
    Draws a rotated rectangle on a numpy array.
    
    Parameters:
        array (ndarray): The numpy array to draw on.
        center (tuple): (cx, cy) coordinates of the rectangle center.
        dimensions (tuple): (width, height) of the rectangle.
        angle (float): Rotation angle in degrees.
    """
    # Create a smaller array to represent the rectangle
    rect_width, rect_height = dimensions
    rect_array = np.zeros_like(array)

    # Compute the rectangle's vertices in its local coordinates
    rect_x = np.array([-rect_width / 2, rect_width / 2, rect_width / 2, -rect_width / 2])
    rect_y = np.array([-rect_height / 2, -rect_height / 2, rect_height / 2, rect_height / 2])
    
    # Compute rotation matrix
    rad_angle = np.deg2rad(angle)
    rotation_matrix = np.array([
        [np.cos(rad_angle), -np.sin(rad_angle)],
        [np.sin(rad_angle), np.cos(rad_angle)],
    ])
    
    # Apply rotation
    rotated_vertices = np.dot(rotation_matrix, np.vstack((rect_x, rect_y)))
    
    # Translate vertices to center
    rotated_vertices[0] += center[0]
    rotated_vertices[1] += center[1]
    
    # Draw the rectangle
    from skimage.draw import polygon
    rr, cc = polygon(rotated_vertices[1], rotated_vertices[0], array.shape)
    array[rr, cc] = 1

# Example usage
array_size = (200, 200)  # Size of the numpy array
rectangle_center = (50, 50)  # Rectangle center
rectangle_dimensions = (20, 10)  # Rectangle dimensions (width, height)
rectangle_angle = 30  # Rotation angle in degrees

array = np.zeros(array_size, dtype=np.int32)
draw_rotated_rectangle(array, rectangle_center, rectangle_dimensions, rectangle_angle)

# Visualize
plt.imshow(array, cmap='gray')
plt.title("Rotated Rectangle")
plt.show()
