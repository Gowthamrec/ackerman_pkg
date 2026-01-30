#!/usr/bin/env python3
import cv2
import rclpy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class ImageViewer:
    def __init__(self, topic_name):
        rclpy.init()
        self.node = rclpy.create_node('image_viewer')
        self.bridge = CvBridge()
        self.subscription = self.node.create_subscription(
            Image,
            topic_name,
            self.image_callback,
            10
        )
        self.topic_name = topic_name
        print(f"Listening to {topic_name}...")

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            cv2.imshow(f"Vision Output - {self.topic_name}", cv_image)
            cv2.waitKey(1)
        except Exception as e:
            self.node.get_logger().error(f'Failed to display image: {e}')

    def run(self):
        try:
            rclpy.spin(self.node)
        except KeyboardInterrupt:
            pass
        finally:
            cv2.destroyAllWindows()
            self.node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    import sys
    topic = sys.argv[1] if len(sys.argv) > 1 else '/object_detection/image'
    viewer = ImageViewer(topic)
    viewer.run()
