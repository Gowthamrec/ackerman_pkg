#!/usr/bin/env python3
"""
Risk-Aware Lane Follower

- Subscribes to Nav2 cmd_vel
- Applies steering correction from fused lane offset (from sensor_fusion_node)
- Scales speed using /speed_factor

Topics:
  /cmd_vel                (in)  Nav2 output
  /lane_offset_fused      (in)  Safe lane offset (0 if obstacle)
  /speed_factor           (in)  0-1 risk-based speed scaling
  /risk_level             (in)  NORMAL/CAUTION/SLOW/EMERGENCY (for logging)
  /cmd_vel_lane_corrected (out) Final command
"""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, String


class LaneFollowerNode(Node):
    def __init__(self):
        super().__init__('lane_follower_node')

        # Parameters
        self.declare_parameter('enable_lane_following', 'true')
        self.declare_parameter('lane_offset_gain', '0.5')
        self.declare_parameter('max_steering_angle', '0.5')
        self.declare_parameter('min_speed_factor', '0.05')

        self.enable_lane_following = self.get_parameter('enable_lane_following').value.lower() == 'true'
        self.lane_offset_gain = float(self.get_parameter('lane_offset_gain').value)
        self.max_steering_angle = float(self.get_parameter('max_steering_angle').value)
        self.min_speed_factor = float(self.get_parameter('min_speed_factor').value)

        # State
        self.current_cmd = Twist()
        self.lane_offset = 0.0
        self.lane_available = False
        self.speed_factor = 1.0
        self.risk_level = 'NORMAL'

        # Subscribers
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.create_subscription(Float32, '/lane_offset_fused', self.lane_offset_callback, 10)
        self.create_subscription(Float32, '/speed_factor', self.speed_factor_callback, 10)
        self.create_subscription(String, '/risk_level', self.risk_level_callback, 10)

        # Publisher
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel_lane_corrected', 10)

        # 10Hz stop timer: continuously hold zero when speed_factor is 0 (risk=EMERGENCY)
        # Overrides velocity_smoother which also publishes to /cmd_vel
        self.create_timer(0.1, self.stop_timer_callback)

        self.get_logger().info(
            'Lane Follower Node initialized\n'
            f'  Lane following: {self.enable_lane_following}\n'
            f'  Offset gain: {self.lane_offset_gain}\n'
            f'  Max steering: {self.max_steering_angle} rad/s\n'
            f'  Min speed factor: {self.min_speed_factor}'
        )

    # ── Callbacks ──
    def cmd_vel_callback(self, msg: Twist):
        self.current_cmd = msg
        out = Twist()

        # Do NOT forward zero/idle commands — only move when Nav2 explicitly commands it
        if msg.linear.x == 0.0 and msg.linear.y == 0.0 and msg.angular.z == 0.0:
            self.cmd_pub.publish(out)  # publish zero (stop)
            return

        # FULL STOP when risk = 1 (speed_factor = 0.0)
        if self.speed_factor == 0.0:
            self.cmd_pub.publish(Twist())
            return

        out.linear = msg.linear

        # Apply lane correction if enabled and available
        if self.enable_lane_following and self.lane_available:
            correction = -self.lane_offset * self.lane_offset_gain
            correction = max(-self.max_steering_angle, min(self.max_steering_angle, correction))
            combined = msg.angular.z + correction
            out.angular.z = max(-self.max_steering_angle, min(self.max_steering_angle, combined))
        else:
            out.angular = msg.angular

        # Apply risk-based speed scaling (0.0 = full stop, already handled above)
        scale = min(1.0, self.speed_factor)
        out.linear.x = msg.linear.x * scale
        out.linear.y = msg.linear.y * scale
        out.linear.z = msg.linear.z * scale

        # Log when speed is reduced significantly
        if scale < 0.99:
            self.get_logger().debug(
                f'Speed scaled: factor={scale:.2f} risk={self.risk_level} offset={self.lane_offset:.3f}')

        self.cmd_pub.publish(out)

    def stop_timer_callback(self):
        """At 10 Hz, keep publishing zero while risk is EMERGENCY (speed_factor=0.0).
        This overrides Nav2's velocity_smoother which also publishes to /cmd_vel."""
        if self.speed_factor == 0.0:
            self.cmd_pub.publish(Twist())

    def lane_offset_callback(self, msg: Float32):
        self.lane_offset = msg.data
        self.lane_available = abs(msg.data) > 0.0

    def speed_factor_callback(self, msg: Float32):
        self.speed_factor = msg.data

    def risk_level_callback(self, msg: String):
        self.risk_level = msg.data.strip().upper() if msg.data else 'NORMAL'


def main(args=None):
    rclpy.init(args=args)
    node = LaneFollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()#!/usr/bin/env python3
"""
Lane Following Controller Node
- Subscribes to /cmd_vel from Nav2
- Reads /lane_offset for steering correction
- Publishes corrected /cmd_vel with steering based on lane offset
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Float32, String
import math

class LaneFollowerNode(Node):
    def __init__(self):
        super().__init__('lane_follower_node')
        
        # Declare parameters
        self.declare_parameter('enable_lane_following', 'true')
        self.declare_parameter('lane_offset_gain', '0.5')  # Steering sensitivity
        self.declare_parameter('max_steering_angle', '0.5')  # Max angular velocity (rad/s)
        self.declare_parameter('confidence_threshold', '0.3')  # Min lane confidence to apply correction
        
        # Get parameters
        enable_lane = self.get_parameter('enable_lane_following').value
        self.enable_lane_following = enable_lane.lower() == 'true'
        
        self.lane_offset_gain = float(self.get_parameter('lane_offset_gain').value)
        self.max_steering_angle = float(self.get_parameter('max_steering_angle').value)
        self.confidence_threshold = float(self.get_parameter('confidence_threshold').value)
        
        # State variables
        self.current_cmd_vel = Twist()
        self.lane_offset = 0.0
        self.lane_detected = False
        self.lane_confidence = 0.0
        self.camera_obstacle = False  # Camera obstacle → stop robot
        self.goal_pose_received = False  # Only activate stop logic after a Nav2 goal is given
        
        # Subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        self.lane_offset_sub = self.create_subscription(
            Float32,
            '/lane_offset',
            self.lane_offset_callback,
            10
        )
        
        self.lane_status_sub = self.create_subscription(
            String,
            '/lane_status',
            self.lane_status_callback,
            10
        )

        # Subscribe to fused obstacles → stop robot when camera detects object
        self.obstacles_fused_sub = self.create_subscription(
            String,
            '/obstacles_fused',
            self.obstacles_fused_callback,
            10
        )

        # Subscribe to navigation goal → arm the stop procedure only when goal is given
        self.goal_pose_sub = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_pose_callback,
            10
        )
        
        # Publishers
        self.cmd_vel_corrected_pub = self.create_publisher(
            Twist,
            '/cmd_vel_lane_corrected',
            10
        )

        # 10Hz timer: continuously publish zero velocity while obstacle active
        # This overrides velocity_smoother which also publishes to /cmd_vel
        self.stop_timer = self.create_timer(0.1, self.stop_timer_callback)

        self.get_logger().info(
            f"Lane Follower Node initialized\n"
            f"  Lane Following: {self.enable_lane_following}\n"
            f"  Offset Gain: {self.lane_offset_gain}\n"
            f"  Max Steering: {self.max_steering_angle} rad/s"
        )
    
    def cmd_vel_callback(self, msg):
        """Receive velocity command from Nav2"""
        self.current_cmd_vel = msg

        # Only stop if a goal was given AND obstacle detected
        if self.camera_obstacle and self.goal_pose_received:
            self.cmd_vel_corrected_pub.publish(Twist())  # full stop
            return

        # Apply lane correction if lane is detected
        if self.enable_lane_following and self.lane_detected:
            corrected_cmd_vel = self.apply_lane_correction(msg)
            self.cmd_vel_corrected_pub.publish(corrected_cmd_vel)
        else:
            self.cmd_vel_corrected_pub.publish(msg)
    
    def obstacles_fused_callback(self, msg):
        """Stop the robot when obstacle detected and a goal has been given"""
        was_obstacle = self.camera_obstacle
        self.camera_obstacle = 'Fused obstacles:' in msg.data
        if self.camera_obstacle and not was_obstacle and self.goal_pose_received:
            self.get_logger().info('LANE FOLLOWER: Obstacle detected after goal given - holding robot STOPPED')
        elif not self.camera_obstacle and was_obstacle:
            self.get_logger().info('LANE FOLLOWER: Obstacle cleared - resuming movement')

    def goal_pose_callback(self, msg):
        """Arm the stop procedure when a navigation goal is received"""
        if not self.goal_pose_received:
            self.goal_pose_received = True
            self.get_logger().info('LANE FOLLOWER: Navigation goal received - stop procedure ARMED')

    def stop_timer_callback(self):
        """Continuously publish zero at 10Hz when goal given AND obstacle detected."""
        if self.camera_obstacle and self.goal_pose_received:
            self.cmd_vel_corrected_pub.publish(Twist())

    def lane_offset_callback(self, msg):
        """Receive lane offset from lane detection node"""
        self.lane_offset = msg.data
    
    def lane_status_callback(self, msg):
        """Receive lane detection status"""
        status_str = msg.data.lower()
        self.lane_detected = "detected" in status_str or "true" in status_str
        
        # Extract confidence if available (format: "Lane: detected, confidence: 0.85")
        try:
            if "confidence:" in status_str:
                conf_str = status_str.split("confidence:")[-1].strip()
                self.lane_confidence = float(conf_str)
        except:
            pass
    
    def apply_lane_correction(self, cmd_vel):
        """
        Apply steering correction based on lane offset
        
        lane_offset: -1 (left of center) to +1 (right of center)
        correction: positive angular velocity = turn left, negative = turn right
        """
        corrected = Twist()
        
        # Copy linear velocity (forward/backward)
        corrected.linear = cmd_vel.linear
        
        # Calculate steering correction
        # lane_offset is normalized -1 to 1
        # We want to steer BACK to center (negative offset = steer right = negative angular)
        steering_correction = -self.lane_offset * self.lane_offset_gain
        
        # Clamp to max steering angle
        steering_correction = max(-self.max_steering_angle, 
                                 min(self.max_steering_angle, steering_correction))
        
        # Combine with existing angular velocity (from Nav2 path following)
        # This allows BOTH lane following AND obstacle avoidance to work together
        combined_angular = cmd_vel.angular.z + steering_correction
        
        # Final clamp to max steering
        corrected.angular.z = max(-self.max_steering_angle,
                                  min(self.max_steering_angle, combined_angular))
        
        # Log when significant correction is applied
        if abs(steering_correction) > 0.05:
            self.get_logger().debug(
                f"Lane correction: offset={self.lane_offset:.3f}, "
                f"correction={steering_correction:.3f}, "
                f"final_angular={corrected.angular.z:.3f}"
            )
        
        return corrected


def main(args=None):
    rclpy.init(args=args)
    node = LaneFollowerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
