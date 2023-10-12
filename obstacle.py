import numpy as np
import matplotlib.pyplot as plt

# grid size
grid_size = (100, 100, 100)
grid_resolution = 0.2  # Assuming each cell in the grid represents 0.2 meters

# Initialize grid
grid = np.zeros(grid_size)

# Obstacle info
walls = [
    {"pose": (3.9215, 9.98, 0), "size": (12.107, 0.15, 2.5)},
    # {"pose": (3.9215, 0, 1.25), "size": (11, 0.15, 2.5)},
    
    {"pose": (3.975, -0.87, 0), "size": (12, 0.15, 2.5)},
    # {"pose": (-2.057, 4.555, 0), "size": (11, 0.15, 2.5)},
    # {"pose":(-2.057, 4.555, 0), "size": (11, 0.15, 2.5)}
]

cylinders = [
    {"pose":(3.9343, 8.08708, 0.5), "radi":0.5, "length": 1},
    {"pose":(7.97738, 8.14621, 0.5), "radi":0.5, "length":1},
    {"pose":(-0.045174, 5.11343, 0.5), "radi":0.5, "length":1},
    {"pose":(3.96965, 5.10438, 0.5), "radi":0.5, "length":1},
    {"pose":(8.08343, 5.09331, 0.5), "radi":0.5, "length":1},
    {"pose":(-0.079143, 2.12, 0.5), "radi":0.5, "length":1},
    {"pose":(4.00467, 2.10866, 0.5), "radi":0.5, "length":1},
    {"pose":(8.12058, 2.03296, 0.5), "radi":0.5, "length":1}
]

# Helper function to convert world coordinates to grid indices
def world_to_grid(pose, grid_size, resolution):
    return (
        int((pose[0] + (grid_size[0] * resolution) / 2) // resolution),
        int((pose[1] + (grid_size[1] * resolution) / 2) // resolution),
        int((pose[2] + (grid_size[1] * resolution) / 2) // resolution)
    )

# Add obstacles to grid
def inside_cylinder(pose, center, radius, length):
    distance = np.sqrt((pose[0]-center[0])**2 + (pose[1]-center[1])**2)
    return distance <= radius and (center[2] - length/2) <= pose[2] <= (center[2] + length/2)

for cylinder in cylinders:
    center_x, center_y, center_z = world_to_grid(cylinder["pose"], grid_size, grid_resolution)
    radius = int(cylinder["radi"] / grid_resolution)
    length = int(cylinder["length"] / grid_resolution)

    # Adjust the z position to start from the ground
    start_z = center_z - length // 2

    for x in range(center_x-radius, center_x+radius+1):
        for y in range(center_y-radius, center_y+radius+1):
            for z in range(start_z, start_z+length):
                if inside_cylinder((x, y, z), (center_x, center_y, start_z + length // 2), radius, length):
                    grid[x, y, z] = 1

for wall in walls:
    center = world_to_grid(wall["pose"], grid_size, grid_resolution)
    size = (int(wall["size"][0] / (2 * grid_resolution)), int(wall["size"][1] / (2 * grid_resolution)), int(wall["size"][2] / (2 * grid_resolution)))
    grid[center[0]-size[0]:center[0]+size[0]+1, center[1]-size[1]:center[1]+size[1]+1, center[2]-size[2]:center[2]+size[2]+1] = 1

# 3D Visualization
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Find occupied cells
x, y, z = np.where(grid == 1)

ax.scatter(x, y, z, c='black', marker='o')
ax.set_title('3D Grid with Obstacles')

plt.show()