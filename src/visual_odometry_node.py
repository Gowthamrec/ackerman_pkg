#!/usr/bin/env python3
"""
visual_odometry_node.py
-----------------------
Estimates robot motion using ORB feature matching between consecutive
camera frames. Publishes /odom and broadcasts odom → base_footprint TF.

Used as odometry source on REAL HARDWARE when no wheel encoders exist.
slam_toolbox scan-matching compensates for any drift.

Publishes:
  /odom                (nav_msgs/Odometry)
  TF: odom → base_footprint  (when publish_tf: true)

Subscribes:
  /<camera_topic>      (sensor_msgs/Image)

Parameters:
  camera_topic         : /image
  publish_tf           : true  (set false in simulation — Gazebo provides TF)
  camera_fx            : 500.0 (focal length x — calibrate for your camera)
  camera_fy            : 500.0 (focal length y)
  camera_cx            : 320.0 (principal point x — image_width / 2)
  camera_cy            : 240.0 (principal point y — image_height / 2)
  min_features         : 30    (min ORB matches to accept a motion estimate)
  max_depth            : 5.0   (assumed scene depth in meters for scale)
"""

import math
import threading
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion
import tf2_ros
from cv_bridge import CvBridge
import cv2


def euler_to_quaternion(roll: float, pitch: float, yaw: float) -> Quaternion:
    """Convert euler angles (radians) to ROS Quaternion."""
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = Quaternion()
    q.w = cr * cp * cy + sr * sp * sy
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy
    return q


class VisualOdometryNode(Node):
    def __init__(self):
        super().__init__('visual_odometry_node')

        # ── Parameters ──
        self.declare_parameter('camera_topic', '/image')
        self.declare_parameter('publish_tf', 'true')
        self.declare_parameter('camera_fx', '500.0')
        self.declare_parameter('camera_fy', '500.0')
        self.declare_parameter('camera_cx', '320.0')
        self.declare_parameter('camera_cy', '240.0')
        self.declare_parameter('min_features', '30')
        self.declare_parameter('camera_height', '0.15')  # metres — measure on your car
        self.declare_parameter('use_sim_time', 'false')

        self.camera_topic = self.get_parameter('camera_topic').value
        self.publish_tf   = self.get_parameter('publish_tf').value.lower() == 'true'
        fx = float(self.get_parameter('camera_fx').value)
        fy = float(self.get_parameter('camera_fy').value)
        cx = float(self.get_parameter('camera_cx').value)
        cy = float(self.get_parameter('camera_cy').value)
        self.min_features   = int(self.get_parameter('min_features').value)
        self.camera_height  = float(self.get_parameter('camera_height').value)
        # Scale factor: monocular VO gives unit-length translation.
        # For a ground robot we use camera height above ground to estimate
        # real-world scale: scale ≈ camera_height / focal_length_y
        self.vo_scale = self.camera_height / fy if fy > 0 else 0.01

        # Camera intrinsic matrix
        self.K = np.array([
            [fx,  0, cx],
            [ 0, fy, cy],
            [ 0,  0,  1]
        ], dtype=np.float64)

        # ── ORB detector and BF matcher ──
        self.orb     = cv2.ORB_create(nfeatures=1000)
        self.matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
        self.bridge  = CvBridge()

        # ── State ──
        self.prev_gray     = None
        self.prev_kp       = None
        self.prev_desc     = None
        self.lock          = threading.Lock()

        # Accumulated pose (2D: x, y, yaw)
        self.x   = 0.0
        self.y   = 0.0
        self.yaw = 0.0
        self.vx  = 0.0
        self.vyaw = 0.0
        self.last_stamp = None

        # ── TF broadcaster ──
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # ── Publishers ──
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

        # ── Subscriber ──
        self.create_subscription(Image, self.camera_topic, self.image_callback, 10)

        self.get_logger().info(
            f'Visual Odometry Node initialized\n'
            f'  Camera topic   : {self.camera_topic}\n'
            f'  Publish TF     : {self.publish_tf}\n'
            f'  Focal length   : fx={fx} fy={fy}\n'
            f'  Principal pt   : cx={cx} cy={cy}\n'
            f'  Camera height  : {self.camera_height} m  (above ground)\n'
            f'  VO scale factor: {self.vo_scale:.4f}\n'
            f'  Min features   : {self.min_features}\n'
            f'  NOTE: Set camera_fx/fy/cx/cy/camera_height to your real values!'
        )

    def image_callback(self, msg: Image):
        """Process each frame: detect ORB features, match, estimate motion."""
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            gray  = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        except Exception as e:
            self.get_logger().error(f'Image conversion error: {e}')
            return

        stamp = msg.header.stamp

        with self.lock:
            # First frame — just store
            if self.prev_gray is None:
                kp, desc = self.orb.detectAndCompute(gray, None)
                self.prev_gray = gray
                self.prev_kp   = kp
                self.prev_desc = desc
                self.last_stamp = stamp
                return

            # Detect features in current frame
            kp, desc = self.orb.detectAndCompute(gray, None)

            dx, dyaw = 0.0, 0.0
            motion_estimated = False

            if (desc is not None and self.prev_desc is not None and
                    len(kp) >= self.min_features and
                    len(self.prev_kp) >= self.min_features):
                try:
                    matches = self.matcher.match(self.prev_desc, desc)
                    # Keep best 70% of matches
                    matches = sorted(matches, key=lambda m: m.distance)
                    good = matches[:max(self.min_features, int(len(matches) * 0.7))]

                    if len(good) >= self.min_features:
                        src_pts = np.float32(
                            [self.prev_kp[m.queryIdx].pt for m in good]
                        ).reshape(-1, 1, 2)
                        dst_pts = np.float32(
                            [kp[m.trainIdx].pt for m in good]
                        ).reshape(-1, 1, 2)

                        # Essential matrix from matched points
                        E, mask = cv2.findEssentialMat(
                            src_pts, dst_pts, self.K,
                            method=cv2.RANSAC, prob=0.999, threshold=1.0
                        )

                        if E is not None and mask is not None:
                            inliers = int(mask.sum())
                            if inliers >= self.min_features:
                                _, R, t, _ = cv2.recoverPose(
                                    E, src_pts, dst_pts, self.K, mask=mask
                                )
                                # Scale translation using camera height above ground
                                # t[2] = forward motion in camera frame (z = optical axis)
                                # t[0] = lateral motion
                                dx   = float(t[2]) * self.vo_scale
                                dyaw = math.atan2(float(R[1, 0]), float(R[0, 0]))
                                motion_estimated = True

                except Exception as e:
                    self.get_logger().debug(f'Feature matching error: {e}')

            # Compute dt
            dt = 0.1  # fallback
            if self.last_stamp is not None:
                prev_ns = self.last_stamp.sec * 1e9 + self.last_stamp.nanosec
                curr_ns = stamp.sec * 1e9 + stamp.nanosec
                dt_computed = (curr_ns - prev_ns) / 1e9
                if 0.001 < dt_computed < 2.0:
                    dt = dt_computed

            if motion_estimated:
                # Update pose
                self.x   += dx * math.cos(self.yaw)
                self.y   += dx * math.sin(self.yaw)
                self.yaw += dyaw
                self.yaw  = math.atan2(math.sin(self.yaw), math.cos(self.yaw))  # normalize
                self.vx   = dx / dt if dt > 0 else 0.0
                self.vyaw = dyaw / dt if dt > 0 else 0.0

            # Always publish odom (even if no motion — keeps TF alive)
            self._publish_odom(stamp)

            # Store for next iteration
            self.prev_gray  = gray
            self.prev_kp    = kp
            self.prev_desc  = desc
            self.last_stamp = stamp

    def _publish_odom(self, stamp):
        """Publish Odometry message and TF."""
        q = euler_to_quaternion(0.0, 0.0, self.yaw)
        now = stamp

        # ── Odometry message ──
        odom = Odometry()
        odom.header.stamp    = now
        odom.header.frame_id = 'odom'
        odom.child_frame_id  = 'base_footprint'

        odom.pose.pose.position.x    = self.x
        odom.pose.pose.position.y    = self.y
        odom.pose.pose.position.z    = 0.0
        odom.pose.pose.orientation   = q

        odom.twist.twist.linear.x    = self.vx
        odom.twist.twist.angular.z   = self.vyaw

        # Covariance — higher values = less trust (visual odom is noisy)
        odom.pose.covariance[0]  = 0.1   # x
        odom.pose.covariance[7]  = 0.1   # y
        odom.pose.covariance[35] = 0.05  # yaw
        odom.twist.covariance[0]  = 0.1
        odom.twist.covariance[35] = 0.05

        self.odom_pub.publish(odom)

        # ── TF: odom → base_footprint ──
        if self.publish_tf:
            tf = TransformStamped()
            tf.header.stamp    = now
            tf.header.frame_id = 'odom'
            tf.child_frame_id  = 'base_footprint'

            tf.transform.translation.x = self.x
            tf.transform.translation.y = self.y
            tf.transform.translation.z = 0.0
            tf.transform.rotation      = q

            self.tf_broadcaster.sendTransform(tf)


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
