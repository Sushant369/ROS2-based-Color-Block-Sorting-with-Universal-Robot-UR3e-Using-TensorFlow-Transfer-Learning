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

# Maybe move gripper to a service server?==++
# from my_robot_msgs.srv import Gripper, GripperRequest



class ur3e_commander_class(Node):

    # INITIALIZE THE NODE
    
    def __init__(self):
        super().__init__('ur3e_commander_node')
        self.node = Node("ex_pose_goal")
        
        # SET IO SERVICE CLIENT
        self.grip_client = self.create_client(SetIO, '/io_and_status_controller/set_io')
        self.grip = SetIO.Request()


        # CREATE BLOCK_COLOR SUBSCRIPTION AND FLAGS        
                
        self.subscription = self.create_subscription(
            String,
            '/block_color',
            self.color_block_callback,
            10)
        
        # Initialize FLAGS
        self.check = True
        self.DI0 = False
            
        # CREATE DIGITAL INPUT SUBSCRIBER
        self.subscription_DI_states = self.create_subscription(
            IOStates,
            '/io_and_status_controller/io_states',
            self.digital_ouput_callback,
            10)

        # Call the subscribers
        self.subscription
        self.subscription_DI_states
        

        self.node.declare_parameter("Home", [-1.1530533593944092, -2.039259672164917, -3.122103830377096, -0.7799976507769983, -1.5435832182513636, -2.354912821446554])
        self.node.declare_parameter("MoveJ_1", [-1.7767912350096644, -2.691538095474243, -1.8474327526488246, -0.7395189444171351, -1.541957203541891, -2.3182433287249964])
        self.node.declare_parameter("MoveJ_2", [-2.001524110833639, -2.239919424057007, -2.0687858066954554, -0.9804046789752405, -1.549370590840475, -2.55831486383547])
        self.node.declare_parameter("MoveJ_ac", [-1.1542927783778687, -2.0393526554107666, -3.1221281490721644, -0.7800138632403772, -1.5435951391803187, -0.6639063994037073])

        # DECLARE FOR 'red_cube' TASK
        self.node.declare_parameter("MoveJ_6", [-1.1549317401698609, -2.039299726486206, -3.1221610508360804, -0.7800067106830042, -1.5435989538775843, 1.1388180255889893])
        self.node.declare_parameter("MoveJ_7", [-1.4921811309507866, -2.1673946380615234, -3.1644379101195277, -1.0599163214312952, -1.3268874327289026, 1.042476773262024])
        self.node.declare_parameter("MoveJ_8", [-1.1549317401698609, -2.039299726486206, -3.1221610508360804, -0.7800067106830042, -1.5435989538775843, 1.1388180255889893])


        # DECLARE FOR 'green_cube' TASK
        self.node.declare_parameter("MoveJ_9", [-1.1548482936671753, -2.0393221378326416, -3.122120042840475, -0.7800181547748011, -1.543542210255758, 0.5776258111000061])
        self.node.declare_parameter("MoveJ_10", [-1.228507713680603, -2.2540295124053955, -3.0917870006956996, -1.017245117818014, -1.4339044729815882, 0.48578473925590515])
        self.node.declare_parameter("MoveJ_11", [-1.1548482936671753, -2.0393221378326416, -3.122120042840475, -0.7800181547748011, -1.543542210255758, 0.5776258111000061])


        
        self.reset_gripper()

        self.get_logger().info("Node Initialized")      
    
###############################################################################################
#====================== TO DO: ADD THE STEPS TO BE PERFORMED =================================#
###############################################################################################
        
    def color_block_callback(self, msg):

        # Imports the /block_color message as msg
        
         
        if self.DI0 and self.check == True:
            
            # ADD IF STATEMENTS BASED ON BLOCK COLOR

            # Then, send the correct code to the robot            
            
            else:
                pass
            
            self.check = True
            self.DI0 = False

###############################################################################################
###############################################################################################

    def digital_ouput_callback(self, msg):

        if msg.digital_in_states[0].state == True:
            print("Checking if DI[0] is HIGH")
            self.DI0 = True
            time.sleep(1)


    ## Below is the callback that the ur3e_commander_class uses
    ## This is where the instance of pymoveit2 and the waypoints are stored


    def execute_goal(self, task):
        feedback_msg = UR3eMove.Feedback()
        feedback_msg.status = 'Starting...'


######################### Command to move the robot to the specified poses ##############################
###################### Think of this section as the program tree on PolyScope ########################### 


        def grab_cube():
            self.get_logger().info(f"RUNNING GRAB CUBE")
            self.get_logger().info(f"Moving to {{HOME: {list(home_pos)}}}")
            moveit2.move_to_configuration(home_pos)
            moveit2.wait_until_executed()

            # Set gripper state
            self.get_logger().info(f"Resetting GRIPPER")
            self.reset_gripper() #Resets Gripper


            # Move to approach
            self.get_logger().info(
                        f"Moving to {{APPROACH: {list(MoveJ_1)}}}"
            )
            moveit2.move_to_configuration(MoveJ_1)
            moveit2.wait_until_executed()
            time.sleep(0.1)

            # Move to conveyor
            self.get_logger().info(
                        f"Moving to {{CONVEYOR: {list(MoveJ_2)}}}"
            )
            moveit2.move_to_configuration(MoveJ_2)
            moveit2.wait_until_executed()
            time.sleep(0.5)

            # Close gripper
            self.get_logger().info(f"Closing GRIPPER")
            self.close_gripper()
            time.sleep(0.5)


            # Move to return_home
            self.get_logger().info(f"Moving to Home: {{HOME: {list(home_pos)}}}")
            moveit2.move_to_configuration(home_pos)
            moveit2.wait_until_executed()


            # Move to joint configuration1
            self.get_logger().info(f"Moving to {{HOME: {list(MoveJ_ac)}}}")
            moveit2.move_to_configuration(MoveJ_ac)
            moveit2.wait_until_executed()



    ###########################################################################


        callback_group = ReentrantCallbackGroup()

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


        
        home_pos = (self.node.get_parameter("HOME").get_parameter_value().double_array_value)
        MoveJ_1 = (self.node.get_parameter("MoveJ_1").get_parameter_value().double_array_value)
        MoveJ_2 = (self.node.get_parameter("MoveJ_2").get_parameter_value().double_array_value)
        MoveJ_6 = (self.node.get_parameter("MoveJ_6").get_parameter_value().double_array_value)
        MoveJ_7 = (self.node.get_parameter("MoveJ_7").get_parameter_value().double_array_value)
        MoveJ_8 = (self.node.get_parameter("MoveJ_8").get_parameter_value().double_array_value)
        MoveJ_9 = (self.node.get_parameter("MoveJ_9").get_parameter_value().double_array_value)
        MoveJ_10 = (self.node.get_parameter("MoveJ_10").get_parameter_value().double_array_value)
        MoveJ_11 = (self.node.get_parameter("MoveJ_11").get_parameter_value().double_array_value)
        
        MoveJ_ac = (self.node.get_parameter("MoveJ_ac").get_parameter_value().double_array_value)
        



######################### Command to move the robot to the specified poses ##############################
###################### Think of this section as the program tree on PolyScope ########################### 

        if task == 'red_cube':
            # CALL FUNCTION
            grab_cube()
            self.get_logger().info(f"MOVE TO RED CUBE BIN")


            self.get_logger().info(f"Moving to {{Red_Cube Bin: {list(MoveJ_6)}}}")
            moveit2.move_to_configuration(MoveJ_6)
            moveit2.wait_until_executed()
            time.sleep(0.2)

            self.get_logger().info(f"Moving to {{Red Cube Bin: {list(MoveJ_7)}}}")
            moveit2.move_to_configuration(MoveJ_7)
            moveit2.wait_until_executed()
            time.sleep(0.2)

            self.open_gripper() #Opens Gripper
            time.sleep(0.5)

            self.get_logger().info(f"Moving to {{Red Cube Bin: {list(MoveJ_8)}}}")
            moveit2.move_to_configuration(MoveJ_8)
            moveit2.wait_until_executed()
            time.sleep(0.2)

            self.get_logger().info(f"Moving to {{HOME: {list(home_pos)}}}")
            moveit2.move_to_configuration(home_pos)
            moveit2.wait_until_executed()
            time.sleep(0.1)


            result = UR3eMove.Result()
            result.completed = 'red_cube COMPLETE'
            return result


        if task == 'green_cube':
            grab_cube()
            self.get_logger().info(f"MOVE TO GREEN CUBE BIN")

            self.get_logger().info(f"Moving to {{Green Cube Bin: {list(MoveJ_9)}}}")
            moveit2.move_to_configuration(MoveJ_9)
            moveit2.wait_until_executed()
            time.sleep(0.5)

            self.get_logger().info(f"Moving to {{Green Cube Bin: {list(MoveJ_10)}}}")
            moveit2.move_to_configuration(MoveJ_10)
            moveit2.wait_until_executed()
            time.sleep(0.5)

            self.open_gripper() #Opens Gripper
            time.sleep(0.5)

            self.get_logger().info(f"Moving to {{Green Cube Bin: {list(MoveJ_11)}}}")
            moveit2.move_to_configuration(MoveJ_11)
            moveit2.wait_until_executed()
            time.sleep(0.5)


            self.get_logger().info(f"Moving to {{HOME: {list(home_pos)}}}")
            moveit2.move_to_configuration(home_pos)
            moveit2.wait_until_executed()
            time.sleep(0.5)


            



#############################################################################################################
#############################################################################################################



def main(args=None):
    rclpy.init(args=args)

    ur3eMoveClass = ur3e_commander_class()

    rclpy.spin(ur3eMoveClass)
    
    ur3eMoveClass.destroy_node()
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()

    

