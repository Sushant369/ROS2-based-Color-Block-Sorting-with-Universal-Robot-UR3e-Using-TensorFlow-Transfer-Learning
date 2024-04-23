#!/usr/bin/env python3
"""
Example of moving to a pose goal.
- ros2 run pymoveit2 ex_pose_goal.py --ros-args -p position:="[0.25, 0.0, 1.0]" -p quat_xyzw:="[0.0, 0.0, 0.0, 1.0]" -p cartesian:=False
"""

from threading import Thread
from sensor_msgs.msg import JointState
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from io_port_toggle import toggle_gripper
from pymoveit2 import MoveIt2
from pymoveit2.robots import ur5
import rclpy


# Global variable to store the current joint state (joint)
current_joint_state = None

def joint_state_callback(msg):
    global current_joint_state
    current_joint_state = msg.position


def move_to_pose(pickPose, standalone=True):
    if standalone:
        rclpy.init()
    try:
        # Create node for this example
        node = Node("moveit2_pose_goal")

        # Declare parameter for joint positions (Joint)
        node.declare_parameter(
            "joint_positions1",
            [-1.8247373739825647,
            -1.906895939503805,
                -0.980743710194723,
                1.5707731246948242,
                    1.379,
                    6.10673189163208],
        )
        node.declare_parameter(
            "joint_positions2",
            [-1.824857536946432,
              -1.907266918812887,
                0.643309473991394,
                  1.5704615116119385,
                    -3.2108901182757776,
                      6.103066921234131],
        )

        # Declare parameters for position and orientation
        node.declare_parameter("position1", [-0.5, 0.2, 0.2])
        node.declare_parameter("position2", [pickPose, 0.2, -0.1])
        node.declare_parameter("position3", [pickPose, 0.6, -0.1])
        node.declare_parameter("quat_xyzw", [0.0, 1.0, 0.0, 0.0])
        node.declare_parameter("quat_rotate", [0.0, 1.0, 0.0, 0.0])
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

         # Create a subscription to the /joint_states topic (Joint)
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

        # Set the planning time (Joint)
        moveit2.planning_time = 10.0

        # Set the planner (Joint)
        moveit2.planner = "RRTConnectkConfigDefault"

        # Get parameters
        position1 = node.get_parameter("position1").get_parameter_value().double_array_value
        position2 = node.get_parameter("position2").get_parameter_value().double_array_value
        position3 = node.get_parameter("position3").get_parameter_value().double_array_value
        quat_xyzw = node.get_parameter("quat_xyzw").get_parameter_value().double_array_value
        quat_rotate = node.get_parameter("quat_rotate").get_parameter_value().double_array_value
        cartesian = node.get_parameter("cartesian").get_parameter_value().bool_value

        # Get parameter for joint positions (Joint)
        joint_positions1 = (
            node.get_parameter("joint_positions1").get_parameter_value().double_array_value
        )
        joint_positions2 = (
            node.get_parameter("joint_positions2").get_parameter_value().double_array_value
        )

        # Print the current joint state (Joint)
        print(current_joint_state)

        # Open the gripper before moving to the first position
        toggle_gripper(1, moveit2)  # Call toggle_gripper with trigger = 1 to open the gripper

        # Move to joint configuration1
        node.get_logger().info(f"Moving to {{joint_positions1: {list(joint_positions1)}}}")
        moveit2.move_to_configuration(joint_positions1)
        moveit2.wait_until_executed()

        # Move to joint configuration2
        node.get_logger().info(f"Moving to {{joint_positions2: {list(joint_positions2)}}}")
        moveit2.move_to_configuration(joint_positions2)
        moveit2.wait_until_executed()

        # Move to joint configuration1
        node.get_logger().info(f"Moving to {{joint_positions1: {list(joint_positions1)}}}")
        moveit2.move_to_configuration(joint_positions1)
        moveit2.wait_until_executed()

        # Move to pose
        # node.get_logger().info(
        #     f"Moving to {{position: {list(position1)}, quat_xyzw: {list(quat_xyzw)}}}"
        # )
        # moveit2.move_to_pose(position=position1, quat_xyzw=quat_xyzw, cartesian=cartesian)
        # moveit2.wait_until_executed()

        node.get_logger().info(
            f"Moving to {{position: {list(position1)}, quat_rotate: {list(quat_rotate)}}}"
        )
        moveit2.move_to_pose(position=position1, quat_xyzw=quat_rotate, cartesian=cartesian)
        moveit2.wait_until_executed()

        node.get_logger().info(
            f"Moving to {{position: {list(position2)}, quat_xyzw: {list(quat_xyzw)}}}"
        )
        moveit2.move_to_pose(position=position2, quat_xyzw=quat_xyzw, cartesian=cartesian)
        moveit2.wait_until_executed()

        node.get_logger().info(
            f"Moving to {{position: {list(position3)}, quat_xyzw: {list(quat_xyzw)}}}"
        )
        moveit2.move_to_pose(position=position3, quat_xyzw=quat_xyzw, cartesian=cartesian)
        moveit2.wait_until_executed()

    # Close the gripper after reaching the first position
        toggle_gripper(0, moveit2)  # Call toggle_gripper with trigger = 0 to close the gripper

        node.get_logger().info(
            f"Moving to {{position: {list(position1)}, quat_xyzw: {list(quat_xyzw)}}}"
        )
        moveit2.move_to_pose(position=position1, quat_xyzw=quat_xyzw, cartesian=cartesian)
        moveit2.wait_until_executed()

    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        if standalone:
            rclpy.shutdown()
            print("Stand alone")
            # executor_thread.join()
            

if __name__ == "__main__":
    move_to_pose(-0.5)  # Call move_to_pose with pickPose = -0.4 to -0.6 is the -x value of the pickPose


    # executor.shutdown()  # Explicitly stop the executor