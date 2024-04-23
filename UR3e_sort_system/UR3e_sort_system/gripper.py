#!/usr/bin/env python3 

import rclpy
from rclpy.node import Node
import time
from ur_msgs.srv import SetIO


class GripperClient(Node):
    def __init__(self):
        super().__init__('gripper_client')
        self.grip_client = self.create_client(SetIO, '/io_and_status_controller/set_io')
        while not self.grip_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('/set_io service not available, waiting again...')
        self.grip = SetIO.Request()

    def io_set_service(self, gripper_request):
        # SOLUTION: create the client in the function
        # self.grip_client = self.create_client(SetIO, '/io_and_status_controller/set_io')

        # CALL THE SERVICE
        # SOLUTION: spin_until_future_complete() is a blocking function
        # rclpy.spin_until_future_complete(self, self.future)
        self.future = self.grip_client.call_async(gripper_request)
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