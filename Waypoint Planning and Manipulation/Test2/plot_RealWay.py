import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

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
ax.scatter(x, y, z, color='blue', label='Waypoints')

# Plot pathway
for i in range(len(x) - 1):
    ax.plot([x[i], x[i+1]], [y[i], y[i+1]], [z[i], z[i+1]], color='red', linestyle='--', label='Pathway')

ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title('Waypoints and Pathway')
ax.legend()
plt.show()
