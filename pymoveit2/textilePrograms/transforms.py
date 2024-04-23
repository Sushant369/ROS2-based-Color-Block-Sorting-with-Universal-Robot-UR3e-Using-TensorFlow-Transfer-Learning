import numpy as np
from math import radians

class Transformations:
    def __init__(self):
        self.tcp_offset_x = 0.0
        self.tcp_offset_y = -0.01
        self.tcp_offset_z = 0.082 # Offset from the end effector to the camera in meters
        # Manually measured distances between conveyor belt to robot base
        self.robot_distance_X = 0.70 - self.tcp_offset_x # Horizontal distance in meters
        self.robot_distance_Y = 0.088 - self.tcp_offset_y # Vertical distance in meters
        self.robot_distance_Z = 0.195 - self.tcp_offset_z # Depth or forward/backward distance in meters

        # Manually measured distances between conveyor belt to camera
        self.camera_distance_X = 0.0  # Horizontal distance in meters
        self.camera_distance_Y = -1.50  # Vertical distance in meters
        self.camera_distance_Z = 0.0  # Depth or forward/backward distance in meters

        # The z axis is known, this means it is always the same position
        self.pick_height = 0.01  # Height in meters

    
    def camera_to_conveyor(self, camera_coords):
        # Convert camera coordinates to conveyor belt coordinates
        conveyor_coords = [camera_coords[0] - self.camera_distance_X,
                        camera_coords[1] , #- self.camera_distance_Y
                        self.pick_height - self.camera_distance_Z]
        return conveyor_coords
    
    def camera_to_robot(self, camera_coords):
        # Convert camera coordinates to conveyor belt coordinates
        conveyor_coords = self.camera_to_conveyor(camera_coords)

        # Convert conveyor belt coordinates to robot coordinates
        robot_coords = [conveyor_coords[0] - self.robot_distance_X,
                        conveyor_coords[1] - self.robot_distance_Y,
                        conveyor_coords[2] - self.robot_distance_Z]
        return robot_coords
'''
from transforms import Transformations

trans = Transformations()

robot_translation, robot_rotation = trans.get_robot_to_host_transformation()
camera_translation, camera_rotation = trans.get_camera_to_host_transformation()

print("\nTransformation from robot base to host (conveyor belt):")
print("Translation:", robot_translation)
print("Rotation (quaternion):", robot_rotation)

print("\nTransformation from camera to host (conveyor belt):")
print("Translation:", camera_translation)
print("Rotation (quaternion):", camera_rotation)

'''