from transforms import Transformations

trans = Transformations()

# Test camera coordinates
camera_coords = [0.1, 0.0, 0.0]

# Convert camera coordinates to robot coordinates
robot_coords = trans.camera_to_robot(camera_coords)

print("Camera coordinates:", camera_coords)
print("Robot coordinates:", robot_coords)