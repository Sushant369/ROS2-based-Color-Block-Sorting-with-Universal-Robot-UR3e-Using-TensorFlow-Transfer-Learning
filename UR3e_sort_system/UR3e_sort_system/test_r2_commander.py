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

        self.subscription = self.create_subscription(
            String,
            '/block_color',
            self.color_block_callback,
            10)
        self.subscription_DI_states = self.create_subscription(
            IOStates,
            '/io_and_status_controller/io_states',
            self.digital_ouput_callback,
            10)

 
        self.subscription
        self.subscription_DI_states
        self.check = True
        self.DI0 = False

        self.node.declare_parameter("HOME", [-1.1530533593944092, -2.039259672164917, -3.122103830377096, -0.7799976507769983, -1.5435832182513636, -2.354912821446554])
        self.node.declare_parameter("MoveJ_1", [-1.7767912350096644, -2.691538095474243, -1.8474327526488246, -0.7395189444171351, -1.541957203541891, -2.3182433287249964])
        self.node.declare_parameter("MoveJ_2", [-2.001524110833639, -2.239919424057007, -2.0687858066954554, -0.9804046789752405, -1.549370590840475, -2.55831486383547])
        self.node.declare_parameter("MoveJ_ac", [-1.1542927783778687, -2.0393526554107666, -3.1221281490721644, -0.7800138632403772, -1.5435951391803187, -0.6639063994037073])

        # DECLARE FOR JOINT POSITIONS FOR 'place_cube' TASK
        self.node.declare_parameter("MoveJ_3", [])
        self.node.declare_parameter("MoveJ_4", [])
        self.node.declare_parameter("MoveJ_5", [])
        # THEN RETURN HOME

        # DECLARE FOR 'red_cube' TASK
        self.node.declare_parameter("MoveJ_6", [-1.1549317401698609, -2.039299726486206, -3.1221610508360804, -0.7800067106830042, -1.5435989538775843, 1.1388180255889893])
        self.node.declare_parameter("MoveJ_7", [-1.4921811309507866, -2.1673946380615234, -3.1644379101195277, -1.0599163214312952, -1.3268874327289026, 1.042476773262024])
        self.node.declare_parameter("MoveJ_8", [-1.1549317401698609, -2.039299726486206, -3.1221610508360804, -0.7800067106830042, -1.5435989538775843, 1.1388180255889893])


        # DECLARE FOR 'green_cube' TASK
        self.node.declare_parameter("MoveJ_9", [-1.1548482936671753, -2.0393221378326416, -3.122120042840475, -0.7800181547748011, -1.543542210255758, 0.5776258111000061])
        self.node.declare_parameter("MoveJ_10", [-1.228507713680603, -2.2540295124053955, -3.0917870006956996, -1.017245117818014, -1.4339044729815882, 0.48578473925590515])
        self.node.declare_parameter("MoveJ_11", [-1.1548482936671753, -2.0393221378326416, -3.122120042840475, -0.7800181547748011, -1.543542210255758, 0.5776258111000061])



        # self._action_server = ActionServer(self, UR3eMove, 'ur3e_action_move', execute_callback=self.goal_callback) 
        
        self.reset_gripper()

        self.get_logger().info("Node Initialized")      
    
    def color_block_callback(self, msg):
        if self.DI0 and self.check == True:
            self.check=False
            self.DI0 = False
            self.block_color = msg.data
            if self.block_color == "RedBlock":
                # self.execute_goal("grab_cube")
                self.execute_goal("red_cube")

                

                print("Executing Sequence 1")
                # sequence1.main()

            elif self.block_color == "GreenBlock":
                print("Executing Sequence 2")
                # self.execute_goal("grab_cube")
                self.execute_goal("green_cube")

            else:
                pass
            
            self.check = True

    def digital_ouput_callback(self, msg):

        if msg.digital_in_states[0].state == True:
            print("Checking if DI[0] is HIGH")
            self.DI0 = True
            time.sleep(1)


    def io_set_service(self, gripper_request):
        self.grip_client = self.create_client(SetIO, '/io_and_status_controller/set_io')
        # while not self.grip_client.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info('/set_io service not available, waiting again...')
        self.grip = SetIO.Request()
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
        time.sleep(0.1)
       
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
        time.sleep(0.1)

        # THEN SET DO1 to HI
        self.grip.fun = 1
        self.grip.pin = 1
        self.grip.state = 1.0
        # CALL THE SERVICE
        self.io_set_service(self.grip)

    def reset_gripper(self):
        print("Executing reset1")
        self.grip.fun = 1
        self.grip.pin = 0
        self.grip.state = 1.0
        self.io_set_service(self.grip)
        print("Executing reset1")

        # SET DO1 to LO 
        self.grip.fun = 1
        self.grip.pin = 1
        self.grip.state = 0.0
        self.io_set_service(self.grip)

        # DELAY
        time.sleep(0.1)
        
        # THEN SET DO2 to LO
        self.grip.fun = 1
        self.grip.pin = 2
        self.grip.state = 0.0
        self.io_set_service(self.grip)


    def execute_goal(self, task):
        feedback_msg = UR3eMove.Feedback()
        feedback_msg.status = 'Starting...'
    #########################################################################
        # Declare parameter for joint positions (from JointState)


        # DECLARE FOR JOINT POSITIONS FOR 'pick_cube' TASK
        # self.node.declare_parameter("HOME", [-0.952, -1.884, -3.472, -0.542, -1.554, 4.156])
        # self.node.declare_parameter("MoveJ_1", [-1.579, -2.280, -2.443, -0.887, -1.602, 3.815])
        # self.node.declare_parameter("MoveJ_2", [ -1.927,-2.412, -1.961, -0.917, -1.604, 3.785])

        def grab_cube():
            self.get_logger().info(f"RUNNING GRAB CUBE")
            self.get_logger().info(f"Moving to {{HOME: {list(moveJ_home)}}}")
            moveit2.move_to_configuration(moveJ_home)
            moveit2.wait_until_executed()

            # # Set gripper state
            self.get_logger().info(f"Resetting GRIPPER")
            self.reset_gripper() #Resets Gripper
            # self.open_gripper() #Opens Gripper


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

            # # Close gripper
            self.get_logger().info(f"Closing GRIPPER")
            self.close_gripper()
            time.sleep(0.5)


            # Move to return_home
            self.get_logger().info(f"Moving to goal_object.request.task{{HOME: {list(moveJ_home)}}}")
            moveit2.move_to_configuration(moveJ_home)
            moveit2.wait_until_executed()


            # Move to joint configuration1
            self.get_logger().info(f"Moving to {{HOME: {list(MoveJ_ac)}}}")
            moveit2.move_to_configuration(MoveJ_ac)
            moveit2.wait_until_executed()



    ###########################################################################

        # DECLARE A MOVEL POSITION

        # Units for [x, y, z] are in METERS !!!
        # self.declare_parameter("approach", [ ])

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


        
        moveJ_home = (self.node.get_parameter("HOME").get_parameter_value().double_array_value)
        MoveJ_1 = (self.node.get_parameter("MoveJ_1").get_parameter_value().double_array_value)
        MoveJ_2 = (self.node.get_parameter("MoveJ_2").get_parameter_value().double_array_value)
        MoveJ_3 = (self.node.get_parameter("MoveJ_3").get_parameter_value().double_array_value)
        MoveJ_4 = (self.node.get_parameter("MoveJ_4").get_parameter_value().double_array_value)
        MoveJ_5 = (self.node.get_parameter("MoveJ_5").get_parameter_value().double_array_value)
        MoveJ_6 = (self.node.get_parameter("MoveJ_6").get_parameter_value().double_array_value)
        MoveJ_7 = (self.node.get_parameter("MoveJ_7").get_parameter_value().double_array_value)
        MoveJ_8 = (self.node.get_parameter("MoveJ_8").get_parameter_value().double_array_value)
        MoveJ_9 = (self.node.get_parameter("MoveJ_9").get_parameter_value().double_array_value)
        MoveJ_10 = (self.node.get_parameter("MoveJ_10").get_parameter_value().double_array_value)
        MoveJ_11 = (self.node.get_parameter("MoveJ_11").get_parameter_value().double_array_value)
        
        MoveJ_ac = (self.node.get_parameter("MoveJ_ac").get_parameter_value().double_array_value)
        

########################################## TO DO ##########################################
        # move = goal_object.request.task

        if task == 'red_cube':
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

            # self.get_logger().info(f"Moving to {{HOME {list(MoveJ_ac)}}}")
            # moveit2.move_to_configuration(MoveJ_ac)
            # moveit2.wait_until_executed()
            # time.sleep(0.1)

            self.get_logger().info(f"Moving to {{HOME: {list(moveJ_home)}}}")
            moveit2.move_to_configuration(moveJ_home)
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


            # self.get_logger().info(f"Moving to {{HOME: {list(MoveJ_ac)}}}")
            # moveit2.move_to_configuration(MoveJ_ac)
            # moveit2.wait_until_executed()
            # time.sleep(0.5)

            self.get_logger().info(f"Moving to {{HOME: {list(moveJ_home)}}}")
            moveit2.move_to_configuration(moveJ_home)
            moveit2.wait_until_executed()
            time.sleep(0.5)

            # result = UR3eMove.Result()
            # result.completed = 'green COMPLETE'
            # return result


        # elif  move == 'place_cube':
        #     self.get_logger().info(f"RUNNING PLACE CUBE")
        #     # self.move_to_pose()

        #     # if self._as.is_preempt_requested():
        #     #     self.get_logger().info('The goal has been cancelled/preempted')
        #     #     self._as.set_preempted()
        #     # result = UR3eMove.Result()
        #     # result.completed = 'place COMPLETE'
        #     # return result

        #     #     self.arm.stop()

    
        # elif task == 'grab_cube':
        #     self.get_logger().info(f"RUNNING GRAB CUBE")
        #     ############################# Command to move the robot to the specified poses ##############################
        #     ########################## Think of this section as the program tree on PolyScope ########################### 

            # Move to joint configuration1

        #     result = UR3eMove.Result()
        #     result.completed = 'grab_cube COMPLETE'
        #     return result


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

    

