#!/usr/bin/env python3
"""
Example of moving to a pose goal.
- ros2 run pymoveit2 ex_pose_goal.py --ros-args -p position:="[0.25, 0.0, 1.0]" -p quat_xyzw:="[0.0, 0.0, 0.0, 1.0]" -p cartesian:=False
"""

from threading import Thread
import math
from time import sleep
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from io_port_toggle import toggle_gripper
from pymoveit2 import MoveIt2
from pymoveit2 import MoveIt2_sim
from pymoveit2.robots import ur5
from pymoveit2.robots import ur5_sim
from pyquaternion import Quaternion
from moveit_msgs.msg import CollisionObject
from geometry_msgs.msg import Pose
from moveit_msgs.msg import PlanningScene
from shape_msgs.msg import Plane
from transforms import Transformations




def move_to_place_cotton(standalone, sim):
        
        # TCP offsets
        Transforms = Transformations()
        tcp_x = Transforms.tcp_offset_x
        tcp_y = Transforms.tcp_offset_y
        tcp_z = Transforms.tcp_offset_z

        if standalone:
            rclpy.init()

        # Create node
        if sim:
            node = Node("moveit2_place_cotton", namespace="arm1")
        else:
            node = Node("moveit2_place_cotton")

        # Declare parameter for Cartesian or joint-space control       
        node.declare_parameter("cartesian", True)

        # Declare parameters for position and orientation
        node.declare_parameter("position1", [tcp_x + 0.3,  tcp_y + 0.45, tcp_z + 0.0])
        node.declare_parameter("position2", [tcp_x + 0.4, tcp_y + (-0.7), tcp_z +  (-0.1)])
        node.declare_parameter("position3", [tcp_x + 0.4, tcp_y + (-0.7), tcp_z +  (-0.2)])
        node.declare_parameter("PickPosition", [tcp_x -0.5, tcp_y + 0.0, tcp_z - 0.02])

        # Declare parameters for Euler angles (roll, pitch, yaw)
        node.declare_parameter("euler_xyz1", [math.radians(0), math.radians(0), math.radians(0)])  # roll, pitch, yaw for position1
        node.declare_parameter("euler_xyz2", [math.radians(0), math.radians(0), math.radians(0)])  # roll, pitch, yaw for position2
        node.declare_parameter("euler_xyz3", [math.radians(140), math.radians(0), math.radians(0)])  # roll, pitch, yaw for position3
        node.declare_parameter("euler_xyz4", [math.radians(140), math.radians(40), math.radians(-40)])  # roll, pitch, yaw for position3

        # Get parameters
        euler_xyz1 = node.get_parameter("euler_xyz1").get_parameter_value().double_array_value
        euler_xyz2 = node.get_parameter("euler_xyz2").get_parameter_value().double_array_value
        euler_xyz3 = node.get_parameter("euler_xyz3").get_parameter_value().double_array_value
        euler_xyz4 = node.get_parameter("euler_xyz4").get_parameter_value().double_array_value

        # Convert Euler angles to quaternion
        node.declare_parameter("quat_xyzw1", [0.0, 1.0, 0.0, 0.0])  # default value is a unit quaternion
        node.declare_parameter("quat_xyzw2", [0.0, 1.0, 0.0, 0.0])  # default value is a unit quaternion
        node.declare_parameter("quat_xyzw3", [0.0, 1.0, 0.0, 0.0])  # default value is a unit quaternion
        node.declare_parameter("quat_xyzw4", [0.0, 1.0, 0.0, 0.0])  # default value is a unit quaternion

        quat_xyzw1 = Quaternion(axis=[1, 0, 0], angle=euler_xyz1[0]) * Quaternion(axis=[0, 1, 0], angle=euler_xyz1[1]) * Quaternion(axis=[0, 0, 1], angle=euler_xyz1[2])
        quat_xyzw2 = Quaternion(axis=[1, 0, 0], angle=euler_xyz2[0]) * Quaternion(axis=[0, 1, 0], angle=euler_xyz2[1]) * Quaternion(axis=[0, 0, 1], angle=euler_xyz2[2])
        quat_xyzw3 = Quaternion(axis=[1, 0, 0], angle=euler_xyz3[0]) * Quaternion(axis=[0, 1, 0], angle=euler_xyz3[1]) * Quaternion(axis=[0, 0, 1], angle=euler_xyz3[2])
        quat_xyzw4 = Quaternion(axis=[1, 0, 0], angle=euler_xyz4[0]) * Quaternion(axis=[0, 1, 0], angle=euler_xyz4[1]) * Quaternion(axis=[0, 0, 1], angle=euler_xyz4[2])
        
        # Update the parameters in the node
        node.set_parameters([rclpy.parameter.Parameter("quat_xyzw1", rclpy.parameter.Parameter.Type.DOUBLE_ARRAY, list(quat_xyzw1.elements))])
        node.set_parameters([rclpy.parameter.Parameter("quat_xyzw2", rclpy.parameter.Parameter.Type.DOUBLE_ARRAY, list(quat_xyzw2.elements))])
        node.set_parameters([rclpy.parameter.Parameter("quat_xyzw3", rclpy.parameter.Parameter.Type.DOUBLE_ARRAY, list(quat_xyzw3.elements))])
        node.set_parameters([rclpy.parameter.Parameter("quat_xyzw4", rclpy.parameter.Parameter.Type.DOUBLE_ARRAY, list(quat_xyzw4.elements))])
        


        # Create callback group that allows execution of callbacks in parallel without restrictions
        callback_group = ReentrantCallbackGroup()

        if sim:
            # Create MoveIt 2 interface Robot
            moveit2 = MoveIt2_sim(
                node=node,
                joint_names=ur5_sim.joint_names(),
                base_link_name=ur5_sim.base_link_name(),
                end_effector_name=ur5_sim.end_effector_name(),
                group_name=ur5_sim.MOVE_GROUP_ARM,
                callback_group=callback_group,
            )
        else:
            
            # Create MoveIt 2 interface Simulation
            moveit2 = MoveIt2(
                node=node,
                joint_names=ur5.joint_names(),
                base_link_name=ur5.base_link_name(),
                end_effector_name=ur5.end_effector_name(),
                group_name=ur5.MOVE_GROUP_ARM,
                callback_group=callback_group,
            )

        # Spin the node in background thread(s) and wait a bit for initialization
        executor = rclpy.executors.MultiThreadedExecutor(2)
        executor.add_node(node)
        executor_thread = Thread(target=executor.spin, daemon=True, args=())
        executor_thread.start()
        node.create_rate(1.0).sleep()

        # Scale down velocity and acceleration of joints (percentage of maximum)
        moveit2.max_velocity = 0.5
        moveit2.max_acceleration = 0.5

        # Get parameters
        position1 = node.get_parameter("position1").get_parameter_value().double_array_value
        position2 = node.get_parameter("position2").get_parameter_value().double_array_value
        position3 = node.get_parameter("position3").get_parameter_value().double_array_value
        PickPosition = node.get_parameter("PickPosition").get_parameter_value().double_array_value
        quat_xyzw1 = node.get_parameter("quat_xyzw1").get_parameter_value().double_array_value
        quat_xyzw2 = node.get_parameter("quat_xyzw2").get_parameter_value().double_array_value
        quat_xyzw3 = node.get_parameter("quat_xyzw3").get_parameter_value().double_array_value
        quat_xyzw4 = node.get_parameter("quat_xyzw4").get_parameter_value().double_array_value
        cartesian = node.get_parameter("cartesian").get_parameter_value().bool_value


        # Move to pose
        node.get_logger().info(
            f"Moving to {{position: {list(position1)}, quat_xyzw: {list(quat_xyzw1)}}}"
        )

        node.get_logger().info(
            f"Moving to {{position: {list(position2)}, quat_xyzw: {list(quat_xyzw2)}}}"
        )
        
        node.get_logger().info(
            f"Moving to {{position: {list(position3)}, quat_xyzw: {list(quat_xyzw3)}}}"
        )
        
        node.get_logger().info(
            f"Moving to {{position: {list(position3)}, quat_xyzw: {list(quat_xyzw4)}}}"
        )

        node.get_logger().info(
            f"Moving to {{position: {list(position1)}, quat_xyzw: {list(quat_xyzw1)}}}"
        )

        node.get_logger().info(
            f"Moving to {{position: {list(PickPosition)}, quat_xyzw: {list(quat_xyzw1)}}}"
        )

        try:
            add_ground_plane(node)

            moveit2.move_to_pose(position=position1, quat_xyzw=quat_xyzw1, cartesian=cartesian)
            moveit2.wait_until_executed()
            
            moveit2.move_to_pose(position=position2, quat_xyzw=quat_xyzw2, cartesian=cartesian)
            moveit2.wait_until_executed()

            moveit2.move_to_pose(position=position3, quat_xyzw=quat_xyzw3, cartesian=cartesian)
            moveit2.wait_until_executed()

            moveit2.move_to_pose(position=position3, quat_xyzw=quat_xyzw4, cartesian=cartesian)
            moveit2.wait_until_executed()

            # Gripper
            toggle_gripper(1, simulation=sim)  # Call toggle_gripper with trigger = 0 to close the gripper

            sleep(1)

             # Gripper
            toggle_gripper(0, simulation=sim)  # Call toggle_gripper with trigger = 0 to close the gripper

            moveit2.move_to_pose(position=position3, quat_xyzw=quat_xyzw3, cartesian=cartesian)
            moveit2.wait_until_executed()

            moveit2.move_to_pose(position=position1, quat_xyzw=quat_xyzw1, cartesian=cartesian)
            moveit2.wait_until_executed()

            moveit2.move_to_pose(position=PickPosition, quat_xyzw=quat_xyzw1, cartesian=cartesian)
            moveit2.wait_until_executed()

        except Exception as e:
            print(f"An error occurred: {e}")
        finally:
            if standalone:
                rclpy.shutdown()
                print("Stand alone")

        node.get_logger().info(f'Movement completed')



def add_ground_plane(node):

    # Create a CollisionObject message
    collision_object = CollisionObject()
    collision_object.id = "ground_plane"
    collision_object.header.frame_id = "world"

    # Define the ground plane as a box shape
    ground_plane = Plane()
    ground_plane.coef = [0.0, 0.0, 1.0, 0.0]

    # Set the ground plane's pose
    ground_plane_pose = Pose()
    ground_plane_pose.position.z = -0.005  # Adjust the height of the ground plane

    collision_object.planes.append(ground_plane)
    collision_object.plane_poses.append(ground_plane_pose)

    # Create a PlanningScene message
    scene = PlanningScene()
    scene.world.collision_objects.append(collision_object)
    scene.is_diff = True
    
    publisher_ = node.create_publisher(PlanningScene, 'planning_scene', 10)
    publisher_.publish(scene)


if __name__ == "__main__":
    move_to_place_cotton(standalone=True, sim=True)
