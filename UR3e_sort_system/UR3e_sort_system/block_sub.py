#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from std_msgs.msg import String
from my_robot_msgs.action import UR3eMove
from ur_msgs.msg import IOStates

class BlockSubscriber(Node):
    def __init__(self):
        super().__init__('block_subscriber')
        self.block_sub = self.create_subscription(String, '/block_color', self.listener_callback, 10)
        self.result = None

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)
        self.color = msg.data
        self.perform_actions_based_on_color(self.color)
        

    def perform_actions_based_on_color(self, color):
        if color == 'RedBlock':
            self.result = color
            print(self.result)
            self.get_logger().info('Red Block Detected, sending goals')
            self.destroy_node()
            return self.result


        elif color == 'GreenBlock':
            self.result = color
            self.get_logger().info('Green Block Detected, sending goals')
            self.destroy_node()
            return self.result

class IOStateSubscriber(Node):
    def __init__(self):
        super().__init__('io_state_subscriber')
        self.io_state_sub = self.create_subscription(IOStates, '/io_and_status_controller/io_states', self.digital_in_callback,10)
        
        # Waiting for Digital In 0 to be HIGH
        self.desired_state = 1
        self.reached_desired_state = False

    def digital_in_callback(self, msg):
        conveyor_state = msg.digital_in_states[0].state

        self.get_logger().info('Digital In: %s' % conveyor_state)
        if conveyor_state == self.desired_state:
            self.get_logger().info('Digital In 0 is HIGH: BLOCK DETECTED')
            self.reached_desired_state = True
            self.destroy_subscription(self.io_state_sub)
            #self.destroy_node()
            return self.reached_desired_state



class UR3eMoveClient(Node):
    def __init__(self):
        super().__init__('ur3e_move_client')
        self._action_client = ActionClient(self, UR3eMove, 'ur3e_action_move')

    def send_goal(self, task):
        goal_object = UR3eMove.Goal()
        goal_object.task = task
        self._action_client.wait_for_server()
        print("in send_goal")

        self._send_goal_future = self._action_client.send_goal_async(goal_object)

        self._send_goal_future.add_done_callback(self.goal_response_callback)
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.sequence))
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback


def main(args=None):
    rclpy.init(args=args)
    io_state_subscriber = IOStateSubscriber()
    block_subscriber = BlockSubscriber()
    move_ur3e_client = UR3eMoveClient()

    block_count = 6
    for i in range(6):
        while io_state_subscriber.reached_desired_state == False:
            rclpy.spin_once(io_state_subscriber)
    

        #rclpy.spin_once(io_state_subscriber)

        rclpy.spin_once(block_subscriber)

        if block_subscriber.result == 'RedBlock':
            task = 'red_block'
            print('Red Block Detected')
            
            future = move_ur3e_client.send_goal('pick_block')
            rclpy.spin_until_future_complete(move_ur3e_client, future)


            move_ur3e_client.send_goal(task)
            rclpy.spin_until_future_complete(move_ur3e_client, future)



        elif block_subscriber.result == 'GreenBlock':
            task = 'green_block'
            print('Green Block Detected')

            future = move_ur3e_client.send_goal('pick_block')
            rclpy.spin_until_future_complete(move_ur3e_client, future)


            future = move_ur3e_client.send_goal(task)
            rclpy.spin_until_future_complete(move_ur3e_client, future)


        else:
            task = 'no_cube in topic, but exited spin anyway?'
            print(block_subscriber.result)
            print('No cube detected')
        i =+ 1


    rclpy.shutdown()

if __name__ == '__main__':
    main()