import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from zed_msgs.msg import ObjectsStamped
class ZedObjectListener(Node):
    def __init__(self):
        super().__init__('zed_object_listener')
        self.subscription = self.create_subscription(
            ObjectsStamped, 
            '/zed/zed_node/obj_det/objects',
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        for obj in msg.objects:
            label = obj.label
            z_distance = obj.position[0]  # z값은 position의 0번째 요소
            self.get_logger().info(f"Label: {label}, Distance (z): {z_distance:.2f} meters")

def main(args=None):
    rclpy.init(args=args)
    node = ZedObjectListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node stopped by user.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
