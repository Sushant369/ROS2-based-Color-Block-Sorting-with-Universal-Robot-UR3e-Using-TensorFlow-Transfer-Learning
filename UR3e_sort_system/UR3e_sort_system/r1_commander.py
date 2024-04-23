#! /usr/bin/env python

from threading import Thread
import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup

from rclpy.node import Node
from rclpy.action import ActionServer

from pymoveit2 import MoveIt2
from pymoveit2.robots import ur3e as robot

from ur_msgs.msg import IOStates
from std_msgs.msg import String
import time

from sensor_msgs.msg import JointState
from my_robot_msgs.action import UR3eMove
from ur_msgs.srv import SetIO

# Maybe move gripper to a service server?
# from my_robot_msgs.srv import Gripper, GripperRequest



class ur3e_commander_class(Node):

    def __init__(self):
        super().__init__('ur3e_commander_node')
        self.node = Node("ex_pose_goal")
        self.grip_client = self.create_client(SetIO, '/io_and_status_controller/set_io')
        # while not self.grip_client.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info('/set_io service not available, waiting again...')
        self.grip = SetIO.Request()
        self.result = UR3eMove.Result()

        self.subscription_DI_states = self.create_subscription(
            IOStates,
            '/io_and_status_controller/io_states',
            self.digital_ouput_callback,
            10)

########################################## TO DO #################################################################################################
        # Declare parameter for joint positions (from JointState)


        cube = [<add waypoints>]
        HOME = [<add waypoints>]
        conveyor = [<add waypoints>]
        approach = [<add waypoints>]


        self.node.declare_parameter("HOME", HOME)
        self.node.declare_parameter("cube", cube)
        self.node.declare_parameter("conveyor", conveyor)
        self.node.declare_parameter("approach", approach)

############################################################################################################################################################

        self.reset_gripper()

        self.get_logger().info("Node Initialized")      
    
    def digital_ouput_callback(self, msg):

        if msg.digital_in_states[0].state == True:
            print("Executing pick up drop of block at Robot1")
            self.execute_pick_drop()
            time.sleep(1)



    def execute_pick_drop(self):

        callback_group = ReentrantCallbackGroup()

        # Create MoveIt 2 interface
        # Create MoveIt 2 interface
        moveit2 = MoveIt2(
            node=self.node,
            joint_names=robot.joint_names(),
            base_link_name=robot.base_link_name(),
            end_effector_name=robot.end_effector_name(),
            group_name=robot.MOVE_GROUP_ARM,
            callback_group=callback_group
        )

        # Spin the node in background thread(s)
        executor = rclpy.executors.MultiThreadedExecutor(2)
        executor.add_node(self.node)
        executor_thread = Thread(target=executor.spin, daemon=True, args=())
        executor_thread.start()

        # Scale down velocity and acceleration of joints (percentage of maximum)
        moveit2.max_velocity = 0.5
        moveit2.max_acceleration = 0.5

        # Set the planning time (Joint)
        moveit2.planning_time = 10.0

        # Set the planner (Joint)
        moveit2.planner = "RRTConnectkConfigDefault"


########################################## TO DO (update below line of code if you add additional waypoints)##########################################

        home = (self.node.get_parameter("HOME").get_parameter_value().double_array_value)
        cube = (self.node.get_parameter("cube").get_parameter_value().double_array_value)
        conveyor = (self.node.get_parameter("conveyor").get_parameter_value().double_array_value)
        approach = (self.node.get_parameter("approach").get_parameter_value().double_array_value)

##############################################################################################################################################################    
    
        # Set the planner (Joint)
        moveit2.planner = "RRTConnectkConfigDefault"



########################################## TO DO (update below sequence if you add additional waypoints)#######################################################

        self.get_logger().info(f"Picking Block")

        #Execute Home Waypoint
        self.get_logger().info(f"Moving to {{Home: {list(home)}}}")
        moveit2.move_to_configuration(home)
        moveit2.wait_until_executed()
        time.sleep(0.5)

        self.open_gripper() #Opens Gripper
        time.sleep(0.5)
        
        #Execute Approach Waypoint
        self.get_logger().info(f"Moving to {{Approach: {list(approach)}}}")
        moveit2.move_to_configuration(approach)
        moveit2.wait_until_executed()
        time.sleep(0.5)

        #Execute Cube Waypoint
        self.get_logger().info(f"Moving to {{Cube: {list(cube)}}}")
        moveit2.move_to_configuration(cube)
        moveit2.wait_until_executed()
        time.sleep(0.5)

        self.close_gripper()#Closes Gripper
        time.sleep(0.1)
        
        #Execute Home Waypoint
        self.get_logger().info(f"Moving to {{Home: {list(home)}}}")
        moveit2.move_to_configuration(home)
        moveit2.wait_until_executed()
        time.sleep(0.5)

        #Execute Conveyor Waypoint
        self.get_logger().info(f"Moving to {{Conveyor {list(conveyor)}}}")
        moveit2.move_to_configuration(conveyor)
        moveit2.wait_until_executed()
        time.sleep(0.5)

        self.open_gripper() #Opens Gripper
        time.sleep(0.5)
        
        #Execute Home Waypoint
        self.get_logger().info(f"Moving to {{Home: {list(home)}}}")
        moveit2.move_to_configuration(home)
        moveit2.wait_until_executed()
        time.sleep(0.5)

###############################################################################################################################################################

    def io_set_service(self, gripper_request):

        # CALL THE SERVICE
        self.future = self.grip_client.call_async(gripper_request)
        # rclpy.spin_until_future_complete(self, self.future)
        print(self.future.result())    # Print the result given by the service called

    def close_gripper(self):
        
        # SET DO1 to LO
        self.grip.fun = 1
        self.grip.pin = 1
        self.grip.state = 0.0
        # CALL THE SERVICE
        self.io_set_service(self.grip)
        
        # DELAY
        time.sleep(0.5)
       
        # THEN SET DO2 to LO
        self.grip.fun = 1
        self.grip.pin = 2
        self.grip.state = 1.0
        # CALL THE SERVICE
        self.io_set_service(self.grip)

    def open_gripper(self):

        # SET DO2 to LO
        self.grip.fun = 1
        self.grip.pin = 2
        self.grip.state = 0.0
        # CALL THE SERVICE
        self.io_set_service(self.grip)

        # DELAY
        time.sleep(0.5)

        # THEN SET DO1 to HI
        self.grip.fun = 1
        self.grip.pin = 1
        self.grip.state = 1.0
        # CALL THE SERVICE
        self.io_set_service(self.grip)

    def reset_gripper(self):
        self.grip.fun = 1
        self.grip.pin = 0
        self.grip.state = 1.0
        self.io_set_service(self.grip)

        # SET DO1 to LO 
        self.grip.fun = 1
        self.grip.pin = 1
        self.grip.state = 0.0
        self.io_set_service(self.grip)

        # DELAY
        time.sleep(0.5)
        
        # THEN SET DO2 to LO
        self.grip.fun = 1
        self.grip.pin = 2
        self.grip.state = 0.0
        self.io_set_service(self.grip)

def main(args=None):
    rclpy.init(args=args)

    ur3eMoveClass = ur3e_commander_class()

    rclpy.spin(ur3eMoveClass)
    
    ur3eMoveClass.destroy_node()
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()

    
