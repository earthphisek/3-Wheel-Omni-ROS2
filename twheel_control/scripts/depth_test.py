import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import numpy as np
import cv2
from cv_bridge import CvBridge

class DepthSubscriber(Node):
    def __init__(self):
        super().__init__('depth_subscriber')
        self.subscription = self.create_subscription(
            Image,
            '/camera/depth/image_rect_raw',
            self.listener_callback,
            10)
        self.bridge = CvBridge()

    def listener_callback(self, msg):
        depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        depth_image = np.array(depth_image, dtype=np.float32)
        
        # Convert depth values from millimeters to meters
        depth_image *= 0.001

        # Define near and far thresholds (e.g., 1 meter for near objects)
        near_threshold = 1.2
        far_threshold = 2.0

        # Calculate masks for near and far objects
        near_objects = depth_image < near_threshold
        far_objects = depth_image >= far_threshold

        # Calculate the percentage of near and far objects
        total_pixels = depth_image.size
        near_percentage = np.sum(near_objects) / total_pixels * 100
        far_percentage = np.sum(far_objects) / total_pixels * 100

        self.get_logger().info(f'Near objects: {near_percentage:.2f}%')
        self.get_logger().info(f'Far objects: {far_percentage:.2f}%')

        # Create a color map to visualize depth image
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

        # Highlight near objects in red and far objects in blue
        depth_colormap[near_objects] = [0, 0, 255]
        depth_colormap[far_objects] = [255, 0, 0]

        # Show the depth image with near and far objects highlighted
        cv2.imshow("Depth Image", depth_colormap)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    depth_subscriber = DepthSubscriber()
    rclpy.spin(depth_subscriber)
    depth_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
