import rclpy
from rclpy.node import Node
from pymoveit2.msg import MaterialPose

from pick_goal import move_to_pose
from place_cotton import move_to_place_cotton
from place_mix import move_to_place_mix
from place_wool import move_to_place_wool

class MaterialPoseSubscriber(Node):
    def __init__(self):
        super().__init__('material_pose_subscriber')
        self.subscription = self.create_subscription(
            MaterialPose,
            'material_pose_topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
            material = msg.material
            pose = msg.pose
            try:
                move_to_pose(pose)
                if material == 'cotton':
                    move_to_place_cotton()
                elif material == 'mix':
                    move_to_place_mix()
                elif material == 'wool':
                    move_to_place_wool()
                else:
                    print(f"Unknown material: {material}")
            except Exception as e:
                print(f"An error occurred: {e}")

def main(args=None):
    rclpy.init(args=args)

    material_pose_subscriber = MaterialPoseSubscriber()

    rclpy.spin(material_pose_subscriber)

    material_pose_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()