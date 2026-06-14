import rclpy
from rclpy.node import Node

# Import requested message types
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

class MockPublisherNode(Node):
    def __init__(self):
        super().__init__('mock_generic_publisher')
        
        # Define the topics exactly as requested
        vision_topic: str = "/mavros/vision_pose/pose_cov"
        depth_topic: str = "/camera/camera/depth/image_rect_raw"
        
        # Create the publishers
        self.vision_pub = self.create_publisher(PoseStamped, vision_topic, 10)
        self.depth_pub = self.create_publisher(String, depth_topic, 10)
        
        # Create a timer to publish at 10 Hz (0.1 seconds)
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.publish_callbacks)
        
        self.get_logger().info(f"Generic Publisher started!")
        self.get_logger().info(f" -> Publishing PoseStamped to {vision_topic}")
        self.get_logger().info(f" -> Publishing String to {depth_topic}")

    def publish_callbacks(self):
        # 1. Populate and publish the PoseStamped message
        pose_msg = PoseStamped()
        
        # Fill in header with current time and a dummy frame
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "map"
        
        # Add some dummy position/orientation data
        pose_msg.pose.position.x = 1.0
        pose_msg.pose.position.y = 0.5
        pose_msg.pose.position.z = 2.0
        pose_msg.pose.orientation.w = 1.0  # Valid neutral quaternion
        
        self.vision_pub.publish(pose_msg)
        
        # 2. Populate and publish the String message
        str_msg = String()
        str_msg.data = "fake_depth_image_string_data"
        
        self.depth_pub.publish(str_msg)


def main(args=None):
    rclpy.init(args=args)
    node = MockPublisherNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt, shutting down publisher...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()