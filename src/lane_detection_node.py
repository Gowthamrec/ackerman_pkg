#!/usr/bin/env python3
"""
Lane Detection Node
Works with both Gazebo camera and real hardware cameras
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String, Float32
from cv_bridge import CvBridge
import cv2
import numpy as np

class LaneDetectionNode(Node):
    def __init__(self):
        super().__init__('lane_detection_node')
        
        # Declare parameters as strings (matching launch file types)
        self.declare_parameter('camera_topic', '/camera/image_raw')
        self.declare_parameter('enable_visualization', 'true')  # String, not bool
        self.declare_parameter('lower_hsv', [0, 0, 200])
        self.declare_parameter('upper_hsv', [180, 30, 255])
        
        # Get parameters
        self.camera_topic = self.get_parameter('camera_topic').value
        enable_vis = self.get_parameter('enable_visualization').value
        self.enable_visualization = enable_vis.lower() == 'true'
            
        lower_hsv = self.get_parameter('lower_hsv').value
        upper_hsv = self.get_parameter('upper_hsv').value
        
        self.lower_hsv = np.array(lower_hsv, dtype=np.uint8)
        self.upper_hsv = np.array(upper_hsv, dtype=np.uint8)
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # Subscribers
        self.image_sub = self.create_subscription(
            Image,
            self.camera_topic,
            self.image_callback,
            10
        )
        
        # Publishers
        self.lane_detected_pub = self.create_publisher(String, '/lane_status', 10)
        self.lane_offset_pub = self.create_publisher(Float32, '/lane_offset', 10)
        self.processed_image_pub = self.create_publisher(Image, '/lane_detection/image', 10)
        
        self.get_logger().info(f"Lane Detection Node initialized (Camera: {self.camera_topic})")
        
    def image_callback(self, msg):
        """Process incoming image and detect lanes"""
        try:
            # Convert ROS image to OpenCV format
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Convert to HSV
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            
            # Create mask for lane colors (white lanes)
            mask = cv2.inRange(hsv, self.lower_hsv, self.upper_hsv)
            
            # Apply morphological operations
            kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            
            # Find contours
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            h, w = frame.shape[:2]
            annotated_frame = frame.copy()
            
            # Analyze lanes
            lane_detected = False
            lane_offset = 0.0
            
            if contours:
                # Get the largest contour
                largest_contour = max(contours, key=cv2.contourArea)
                area = cv2.contourArea(largest_contour)
                
                if area > 500:  # Minimum area threshold
                    lane_detected = True
                    
                    # Fit line to lane
                    M = cv2.moments(largest_contour)
                    if M["m00"] != 0:
                        cx = int(M["m10"] / M["m00"])
                        lane_offset = (cx - w/2) / (w/2)  # Normalized offset
                    
                    if self.enable_visualization:
                        # Draw contour
                        cv2.drawContours(annotated_frame, [largest_contour], 0, (0, 255, 0), 2)
                        
                        # Draw center line
                        cv2.line(annotated_frame, (w//2, 0), (w//2, h), (255, 0, 0), 2)
                        
                        # Draw lane center if detected
                        if lane_offset != 0:
                            lane_x = w//2 + int(lane_offset * w/2)
                            cv2.line(annotated_frame, (lane_x, 0), (lane_x, h), (0, 255, 255), 2)
                            cv2.putText(annotated_frame, f"Offset: {lane_offset:.2f}", 
                                      (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            
            # Publish status
            status_msg = String()
            status_msg.data = "Lane detected" if lane_detected else "Lane not detected"
            self.lane_detected_pub.publish(status_msg)
            
            # Publish offset
            offset_msg = Float32()
            offset_msg.data = float(lane_offset)
            self.lane_offset_pub.publish(offset_msg)
            
            # Publish annotated image
            if self.enable_visualization:
                annotated_img_msg = self.bridge.cv2_to_imgmsg(annotated_frame, encoding='bgr8')
                annotated_img_msg.header.stamp = msg.header.stamp
                self.processed_image_pub.publish(annotated_img_msg)
            
        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = LaneDetectionNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
