#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from std_msgs.msg import String
from my_robot_msgs.action import UR3eMove
from ur_msgs.msg import IOStates
import time

class BlockSubscriber(Node):
    def __init__(self):
        
        super().__init__('robot2_subscriber')
        self.subscription = self.create_subscription(
            String,
            '/block_color',
            self.color_block_callback,
            10)
        self.subscription_DO_states = self.create_subscription(
            IOStates,
            '/io_and_status_controller/io_states',
            self.digital_ouput_callback,
            10)

        self._action_client = ActionClient(self, UR3eMove, 'ur3e_action_move')
 
        self.subscription
        self.subscription_DO_states
        self.check = True
        self.D0 = False

        self.result = None

    def send_goal(self, task):
        goal_object = UR3eMove.Goal()
        goal_object.task = task
        self._action_client.wait_for_server()
        return self._action_client.send_goal_async(goal_object)


    def color_block_callback(self, msg):
        if self.D0 and self.check == True:
            self.check=False
            self.D0 = False
            self.block_color = msg.data
            if self.block_color == "RedBlock":
                self.send_goal("red_cube")
                print("Executing Sequence 1")
                # sequence1.main()

            elif self.block_color == "GreenBlock":
                print("Executing Sequence 2")
                self.send_goal("green_cube")

            else:
                pass
            
            self.check = True


    def digital_ouput_callback(self, msg):

        if msg.digital_in_states[0].state == True:
            print("setting D0 to high")
            self.D0 = True
            time.sleep(1)




def main(args=None):
    rclpy.init(args=args)
    block_subscriber = BlockSubscriber()
    rclpy.spin(block_subscriber)
    block_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
