#!/usr/bin/env python3
"""
Example of moving to a joint configuration.
- ros2 run pymoveit2 joint_goal.py --ros-args -p joint_positions:="[1.57, -1.57, 0.0, -1.57, 0.0, 1.57]"
"""

from threading import Thread
from sensor_msgs.msg import JointState

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

from pymoveit2 import MoveIt2
from pymoveit2.robots import ur5

# Global variable to store the current joint state
current_joint_state = None

def joint_state_callback(msg):
    global current_joint_state
    current_joint_state = msg.position

def main():
    rclpy.init()

    # Create node for this example
    node = Node("ex_joint_goal")

    # Declare parameter for joint positions
    node.declare_parameter(
        "joint_positions",
        [-1.8247373739825647,
          -1.906895939503805,
            -0.980743710194723,
              1.5707731246948242,
                1.379,
                  6.10673189163208],
    )

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

    # Create a subscription to the /joint_states topic
    joint_state_subscription = node.create_subscription(
        JointState,
        '/joint_states',
        joint_state_callback,
        10
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

    # Get parameter for joint positions
    joint_positions = (
        node.get_parameter("joint_positions").get_parameter_value().double_array_value
    )

    # Print the current joint state
    print(current_joint_state)

    # Set the planning time
    moveit2.planning_time = 10.0

    # Set the planner
    moveit2.planner = "RRTConnectkConfigDefault"

    # # Move to joint configuration
    # node.get_logger().info(f"Moving to {{joint_positions: {list(joint_positions)}}}")
    # moveit2.move_to_configuration(joint_positions)
    # moveit2.wait_until_executed()

    rclpy.shutdown()
    executor_thread.join()
    exit(0)


if __name__ == "__main__":
    main()