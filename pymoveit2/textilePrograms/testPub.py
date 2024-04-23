import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point  # Import the Point message type

class MyPublisher(Node):
    def __init__(self):
        super().__init__('my_publisher')
        self.publisher_ = self.create_publisher(Point, 'my_robot_topic', 0)

    def publish_message(self, y):
        msg = Point()
        msg.y = y
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: y: %s"' % (msg.y))

def main(args=None):
    rclpy.init(args=args)
    node = MyPublisher()
    while rclpy.ok():

        y = float(input("Enter the second float to publish (or 'quit' to exit): "))
        if str(y).lower() == 'quit':
            break

        node.publish_message(y)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()