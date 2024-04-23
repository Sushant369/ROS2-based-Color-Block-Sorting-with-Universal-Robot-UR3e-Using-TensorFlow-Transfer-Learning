import rclpy
from rclpy.node import Node
from pymoveit2.msg import MaterialPose

class MaterialPosePublisher(Node):
    def __init__(self):
        super().__init__('material_pose_publisher')
        self.publisher_ = self.create_publisher(MaterialPose, 'material_pose_topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = MaterialPose()
        msg.material = 'cotton'
        msg.pose = -0.55
        self.publisher_.publish(msg)
        self.get_logger().info(f"Publishing: '{msg.material}', '{msg.pose}'")

def main(args=None):
    rclpy.init(args=args)

    material_pose_publisher = MaterialPosePublisher()

    rclpy.spin(material_pose_publisher)

    material_pose_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()