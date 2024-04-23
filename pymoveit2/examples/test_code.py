#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from ur_msgs.msg import IOStates
from std_msgs.msg import String
import time
# import ex_pose_goal as sequence1



class MinimalSubscriber(Node):

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
        
        self.subscription
        self.subscription_DO_states
        self.check = True
        self.D0 = False


    def color_block_callback(self, msg):
        if self.D0 and self.check == True:
            self.check=False
            self.D0 = False
            self.block_color = msg.data
            if self.block_color == "RedBlock":
                print("Executing Sequence 1")
                # sequence1.main()

            elif self.block_color == "GreenBlock":
                print("Executing Sequence 2")

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
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()