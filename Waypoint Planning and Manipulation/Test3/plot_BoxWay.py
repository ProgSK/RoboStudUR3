import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import numpy as np

# Read waypoints from file
with open("waypoints.txt", "r") as file:
    lines = file.readlines()
    waypoints = [[float(num) for num in line.split()] for line in lines]

# Extract x, y, z coordinates
x = [point[0] for point in waypoints]
y = [point[1] for point in waypoints]
z = [point[2] for point in waypoints]

# Plot waypoints
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
waypoints_plot = ax.scatter(x, y, z, color='blue', label='Waypoints')

# Plot path
path_lines = []
for i in range(len(x) - 1):
    line, = ax.plot([x[i], x[i+1]], [y[i], y[i+1]], [z[i], z[i+1]], color='red', linestyle='--')
    path_lines.append(line)

# Plot box at each waypoint
for point in waypoints:
    x_box = np.array([point[0] - 0.25, point[0] + 0.25, point[0] + 0.25, point[0] - 0.25, point[0] - 0.25])
    y_box = np.array([point[1] - 0.25, point[1] - 0.25, point[1] + 0.25, point[1] + 0.25, point[1] - 0.25])
    z_box = np.array([point[2], point[2], point[2] + 0.5, point[2] + 0.5, point[2]])

    # Vertices for the box
    vertices = np.array([[x_box[0], y_box[0], z_box[0]],
                         [x_box[1], y_box[1], z_box[1]],
                         [x_box[2], y_box[2], z_box[2]],
                         [x_box[3], y_box[3], z_box[3]],
                         [x_box[0], y_box[0], z_box[0] + 0.5],
                         [x_box[1], y_box[1], z_box[1] + 0.5],
                         [x_box[2], y_box[2], z_box[2] + 0.5],
                         [x_box[3], y_box[3], z_box[3] + 0.5]])

    # Define the faces of the box
    faces = [[vertices[0], vertices[1], vertices[2], vertices[3]],
             [vertices[4], vertices[5], vertices[6], vertices[7]],
             [vertices[0], vertices[1], vertices[5], vertices[4]],
             [vertices[2], vertices[3], vertices[7], vertices[6]],
             [vertices[1], vertices[2], vertices[6], vertices[5]],
             [vertices[4], vertices[7], vertices[3], vertices[0]]]

    # Plot the box
    ax.add_collection3d(Poly3DCollection(faces, color='gray', alpha=0.5))

ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title('Waypoints, Path, and Box')

# Add legend for waypoints and pathway
plt.legend(handles=[waypoints_plot, path_lines[0]], labels=['Waypoints', 'Pathway'], loc='upper right', bbox_to_anchor=(1.2, 1))
plt.show()
