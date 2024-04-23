import rclpy
from rclpy.node import Node
from std_msgs.msg import String  # Import the String message type
from pick_goal import move_to_pose
from place_cotton import move_to_place_cotton
from place_mix import move_to_place_mix
from place_wool import move_to_place_wool
import threading  # Import the threading module
from geometry_msgs.msg import Point  # Import the Point message type
import argparse
import time
from std_msgs.msg import Float32MultiArray



class MySubscriber(Node):
    def __init__(self, z):
        super().__init__('my_subscriber')
        self.z = z
        self.x_received = False
        self.y_received = False
        self.lock = threading.Lock()  # Create a new lock


        self.subscription_object = self.create_subscription(
            Float32MultiArray,  # Specify the message type
            '/object_pose_topic',  # Specify the topic
            self.listener_callback_object,  # Specify the callback
            0)  # Specify the queue size
        
        self.subscription_material = self.create_subscription(
            Point,  # Specify the message type
            '/my_robot_topic',  # Specify the topic
            self.listener_callback_material,  # Specify the callback
            0)  # Specify the queue size

    def listener_callback_object(self, msg):
            with self.lock:  # Acquire the lock before executing the rest of the method
                if self.y_received:  # Only accept the x value if the y value has been received
                    self.x = msg.data[0:3]
                    self.x_received = True
                    if self.y_received:
                        self.load_program()
                        self.x_received = False
                        self.y_received = False
                else:
                    self.x_received = False  # If the y value has not been received, do not accept the x value

    def listener_callback_material(self, msg):
        self.y = msg.y
        self.y_received = True
        if self.x_received:
            with self.lock:  # Acquire the lock before executing the rest of the method
                self.load_program()
            self.x_received = False  # Reset both flags after calling load_program
            self.y_received = False

    def load_program(self):
        if self.y_received and self.x_received:
            if self.y == 0.0:
                print("No objecrt detected")

            if self.y == 1.0:
                if self.x[0] >= 0.1 and self.x[0] <= 0.37:
                    try:
                        # Start a new thread to run the move_to_pose function with the received x and y values
                        # threading.Thread(target=self.run_pick_cotton, args=(x, z)).start()
                        self.run_pick_cotton(self.x[0:3], self.z)
                    except Exception as e:
                        print(f"An error occurred: {e}")
                else:
                    print("Invalid x value")
                    print(self.x[0:3])


            if self.y == 2.0:
                if self.x[0] >= 0.1 and self.x[0] <= 0.37:
                    try:
                        # Start a new thread to run the move_to_pose function with the received x and y values
                        # threading.Thread(target=self.run_pick_wool, args=(x, z)).start()
                        self.run_pick_wool(self.x[0:3], self.z)
                    except Exception as e:
                        print(f"An error occurred: {e}")
                else:
                    print("Invalid x value")
                    print(self.x[0:3])

            if self.y == 3.0:
                if self.x[0] >= 0.1 and self.x[0] <= 0.37:
                    try:
                        # Start a new thread to run the move_to_pose function with the received x and y values
                        # threading.Thread(target=self.run_pick_mix, args=(x, z)).start()
                        self.run_pick_mix(self.x[0:3], self.z)
                    except Exception as e:
                        print(f"An error occurred: {e}")    
                else:
                    print("Invalid x value")
                    print(self.x[0:3])
                    
                # Reset the flags after using the x and y values
        self.x_received = False
        self.y_received = False

    def run_pick_cotton(self ,xyz, z):
        if z == 1.0:
            sim = True
        else:
            sim = False
        move_to_pose(xyz, False, sim)
        print("Pick finished")
        # move_to_place_cotton(False, sim)
        # print("Place finished")
        MySubscriber(z)     # Re-subscribe to the topic

        
    def run_pick_wool(self ,xyz, z):
        if z == 1.0:
            sim = True
        else:
            sim = False
        move_to_pose(xyz, False, sim)
        print("Pick finished")
        move_to_place_wool(False, sim)
        print("Place finished")
        MySubscriber(z)     # Re-subscribe to the topic

    def run_pick_mix(self ,xyz, z):
        if z == 1.0:
            sim = True
        else:
            sim = False
        move_to_pose(xyz, False, sim)
        print("Pick finished")
        move_to_place_mix(False, sim)
        print("Place finished")
        MySubscriber(z)     # Re-subscribe to the topic


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("z", type=str, help="Specify the z value as 'True' or 'False'")
    args = parser.parse_args()

    z = args.z.lower() == 'true'  # Convert the argument to a boolean

    rclpy.init()  # Initialize the default context
    try:
        node = MySubscriber(z)
        rclpy.spin(node)  # Spin the node so it can process callbacks
    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        if rclpy.ok():  # Check if the context is still valid
            node.destroy_node()
            rclpy.shutdown()  # Shutdown the default context

if __name__ == '__main__':
    main()