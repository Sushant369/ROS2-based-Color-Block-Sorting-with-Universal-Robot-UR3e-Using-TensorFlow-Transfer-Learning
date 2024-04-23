#!/usr/bin/env python3
"""
Example of moving to a pose goal.
- ros2 run pymoveit2 ex_pose_goal.py --ros-args -p position:="[0.25, 0.0, 1.0]" -p quat_xyzw:="[0.0, 0.0, 0.0, 1.0]" -p cartesian:=False
"""

from threading import Thread

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from io_port_toggle import toggle_gripper
from pymoveit2 import MoveIt2
from pymoveit2.robots import ur5


def move_to_tcp_orientation(args=None):
    rclpy.init()

    # Create node for this example
    node = Node("moveit2_tcp_orientation")

    # Declare parameters for position and orientation
    node.declare_parameter("position1", [0.2, 0.3, 0.2])
    # node.declare_parameter("position2", [0.5, -0.4, -0.1])
    # node.declare_parameter("position3", [0.5, -0.4, -0.2])
    node.declare_parameter("quat_xyzw", [1.0, 0.0, 0.0, 0.0])
    node.declare_parameter("cartesian", True)


    # Create callback group that allows execution of callbacks in parallel without restrictions
    callback_group = ReentrantCallbackGroup()

    # Create MoveIt 2 interface
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
    # position2 = node.get_parameter("position2").get_parameter_value().double_array_value
    # position3 = node.get_parameter("position3").get_parameter_value().double_array_value
    quat_xyzw = node.get_parameter("quat_xyzw").get_parameter_value().double_array_value
    cartesian = node.get_parameter("cartesian").get_parameter_value().bool_value


    # Move to pose
    node.get_logger().info(
        f"Moving to {{position: {list(position1)}, quat_xyzw: {list(quat_xyzw)}}}"
    )
    moveit2.move_to_pose(position=position1, quat_xyzw=quat_xyzw, cartesian=cartesian)
    moveit2.wait_until_executed()

#     node.get_logger().info(
#         f"Moving to {{position: {list(position2)}, quat_xyzw: {list(quat_xyzw)}}}"
#     )
#     moveit2.move_to_pose(position=position2, quat_xyzw=quat_xyzw, cartesian=cartesian)
#     moveit2.wait_until_executed()

#     node.get_logger().info(
#         f"Moving to {{position: {list(position3)}, quat_xyzw: {list(quat_xyzw)}}}"
#     )
#     moveit2.move_to_pose(position=position3, quat_xyzw=quat_xyzw, cartesian=cartesian)
#     moveit2.wait_until_executed()

#   # Close the gripper after reaching the first position
#     toggle_gripper(1, moveit2)  # Call toggle_gripper with trigger = 0 to close the gripper

#     node.get_logger().info(
#         f"Moving to {{position: {list(position1)}, quat_xyzw: {list(quat_xyzw)}}}"
#     )
#     moveit2.move_to_pose(position=position1, quat_xyzw=quat_xyzw, cartesian=cartesian)
#     moveit2.wait_until_executed()

    rclpy.shutdown()
    executor_thread.join()


if __name__ == "__main__":
    move_to_tcp_orientation()
