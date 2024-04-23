#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ur_msgs.msg import IOStates
from std_msgs.msg import String
import time
import ex_pose_goal as sequence1

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
        self.subscription_DO_states  # prevent unused variable warning
        self.check = True
        self.block_color=""
        self.D0 = False

    
    # global check, Block_color
    
    def color_block_callback(self, msg):
        # self.get_logger().info('Block Color is : "%s"' % msg.data)
        # check=True
        if self.D0 and self.check == True:
            self.check=False
            self.D0 = False 
            self.block_color = msg.data
            if self.block_color == "RedBlock":
                print("Executing Sequence 1")
                sequence1.main()

            elif self.block_color == "GreenBlock":
                print("Executing Sequence 2")
            
            self.check = True


    def digital_ouput_callback(self, msg):
        # print(msg.digital_in_states[0].state)

        if msg.digital_in_states[0].state == True:
            print("setting D0 to high")
            self.D0 = True
            time.sleep(1)
            
        # else:
        #     self.D0 = False

        # self.subscription_DO_states.destroy()



def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector dest   roys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
