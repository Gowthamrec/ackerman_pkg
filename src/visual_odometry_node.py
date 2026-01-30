#!/usr/bin/env python3
"""
Visual Odometry Node using feature matching
Works with both Gazebo camera and real hardware cameras
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovariance, TwistWithCovariance, Vector3, Quaternion
from cv_bridge import CvBridge
import cv2
import numpy as np
from tf_transformations import quaternion_from_matrix
import threading

class VisualOdometryNode(Node):
    def __init__(self):
        super().__init__('visual_odometry_node')
        
        # Declare parameters as strings (matching launch file types)
        self.declare_parameter('camera_topic', '/camera/image_raw')
        self.declare_parameter('publish_tf', 'false')  # String, not bool
        self.declare_parameter('camera_matrix', 
                             [[500.0, 0.0, 320.0],
                              [0.0, 500.0, 240.0],
                              [0.0, 0.0, 1.0]])
        
        self.camera_topic = self.get_parameter('camera_topic').value
        
        publish_tf = self.get_parameter('publish_tf').value
        self.publish_tf = publish_tf.lower() == 'true'
            
        camera_matrix = self.get_parameter('camera_matrix').value
        self.K = np.array(camera_matrix, dtype=np.float32)
        
        # Initialize ORB detector
        self.orb = cv2.ORB_create(nfeatures=500)
        self.bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=False)
        
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
        self.odometry_pub = self.create_publisher(Odometry, '/visual_odometry/odometry', 10)
        
        # Odometry state
        self.pose = np.eye(4)
        self.previous_frame = None
        self.previous_kp = None
        self.previous_des = None
        
        self.frame_count = 0
        self.lock = threading.Lock()
        
        self.get_logger().info(f"Visual Odometry Node initialized (Camera: {self.camera_topic})")
    
    def image_callback(self, msg):
        """Process incoming image and estimate odometry"""
        try:
            # Convert ROS image to OpenCV format
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='grayscale')
            
            with self.lock:
                if self.previous_frame is None:
                    # First frame
                    self.previous_frame = frame.copy()
                    self.previous_kp, self.previous_des = self.orb.detectAndCompute(frame, None)
                    return
                
                # Detect and compute features
                kp, des = self.orb.detectAndCompute(frame, None)
                
                if des is not None and self.previous_des is not None and len(kp) > 10:
                    # Match features
                    matches = self.bf.knnMatch(self.previous_des, des, k=2)
                    
                    # Apply Lowe's ratio test
                    good_matches = []
                    for match_pair in matches:
                        if len(match_pair) == 2:
                            m, n = match_pair
                            if m.distance < 0.75 * n.distance:
                                good_matches.append(m)
                    
                    if len(good_matches) > 10:
                        # Estimate motion
                        src_pts = np.float32([self.previous_kp[m.queryIdx].pt for m in good_matches])
                        dst_pts = np.float32([kp[m.trainIdx].pt for m in good_matches])
                        
                        # Calculate fundamental matrix
                        F, mask = cv2.findFundamentalMat(src_pts, dst_pts, cv2.FM_RANSAC, 1.0, 0.99)
                        
                        if F is not None:
                            # Calculate essential matrix
                            E = self.K.T @ F @ self.K
                            
                            # Recover pose
                            _, R, t, mask = cv2.recoverPose(E, src_pts, dst_pts, self.K, mask=mask)
                            
                            # Update pose (simple integration)
                            pose_change = np.eye(4)
                            pose_change[:3, :3] = R
                            pose_change[:3, 3] = t.flatten() * 0.1  # Scale factor for sim
                            
                            self.pose = self.pose @ pose_change
                            
                            # Publish odometry
                            self.publish_odometry(msg.header, R, t)
                
                # Update previous frame
                self.previous_frame = frame.copy()
                self.previous_kp = kp
                self.previous_des = des
                self.frame_count += 1
                
        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")
    
    def publish_odometry(self, header, R, t):
        """Publish odometry message"""
        try:
            # Create odometry message
            odom = Odometry()
            odom.header.stamp = header.stamp
            odom.header.frame_id = 'odom'
            odom.child_frame_id = 'base_link'
            
            # Extract position and rotation
            x, y, z = self.pose[0, 3], self.pose[1, 3], self.pose[2, 3]
            
            # Convert rotation matrix to quaternion
            R_full = self.pose[:3, :3]
            H = np.eye(4)
            H[:3, :3] = R_full
            quat = quaternion_from_matrix(H)
            
            # Set pose
            odom.pose.pose.position = Vector3(x=float(x), y=float(y), z=float(z))
            odom.pose.pose.orientation = Quaternion(
                x=float(quat[0]),
                y=float(quat[1]),
                z=float(quat[2]),
                w=float(quat[3])
            )
            
            # Set covariance
            odom.pose.covariance = [0.1, 0, 0, 0, 0, 0,
                                   0, 0.1, 0, 0, 0, 0,
                                   0, 0, 0.1, 0, 0, 0,
                                   0, 0, 0, 0.1, 0, 0,
                                   0, 0, 0, 0, 0.1, 0,
                                   0, 0, 0, 0, 0, 0.1]
            
            # Set twist (velocity)
            odom.twist.twist.linear = Vector3(x=0.0, y=0.0, z=0.0)
            odom.twist.twist.angular = Vector3(x=0.0, y=0.0, z=0.0)
            
            self.odometry_pub.publish(odom)
            
        except Exception as e:
            self.get_logger().error(f"Error publishing odometry: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = VisualOdometryNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
