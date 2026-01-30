#!/usr/bin/env python3
"""
Lane Following Controller Node
- Subscribes to /cmd_vel from Nav2
- Reads /lane_offset for steering correction
- Publishes corrected /cmd_vel with steering based on lane offset
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
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
        self.lane_offset = 0.0  # Offset from lane center (-1 to 1, normalized)
        self.lane_detected = False
        self.lane_confidence = 0.0
        
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
        
        # Publishers
        self.cmd_vel_corrected_pub = self.create_publisher(
            Twist,
            '/cmd_vel_lane_corrected',
            10
        )
        
        self.get_logger().info(
            f"Lane Follower Node initialized\n"
            f"  Lane Following: {self.enable_lane_following}\n"
            f"  Offset Gain: {self.lane_offset_gain}\n"
            f"  Max Steering: {self.max_steering_angle} rad/s"
        )
    
    def cmd_vel_callback(self, msg):
        """Receive velocity command from Nav2"""
        self.current_cmd_vel = msg
        
        # Apply lane correction if lane is detected
        if self.enable_lane_following and self.lane_detected:
            corrected_cmd_vel = self.apply_lane_correction(msg)
            self.cmd_vel_corrected_pub.publish(corrected_cmd_vel)
        else:
            # Pass through original command if lane not detected
            self.cmd_vel_corrected_pub.publish(msg)
    
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
