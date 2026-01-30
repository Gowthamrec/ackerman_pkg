#!/usr/bin/env python3
"""
Sensor Fusion Node - Prioritizes LiDAR for Obstacle Detection
- LiDAR (HIGH PRIORITY): Reliable obstacle detection
- Camera (MEDIUM PRIORITY): Lane detection, object classification
- Combines both for robust navigation
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String, Float32
from geometry_msgs.msg import Twist
import numpy as np
import math

class SensorFusionNode(Node):
    def __init__(self):
        super().__init__('sensor_fusion_node')
        
        # Declare parameters
        self.declare_parameter('lidar_range_threshold', '0.5')  # meters (HIGH PRIORITY)
        self.declare_parameter('lidar_angle_range', '60.0')  # degrees (front detection range)
        self.declare_parameter('use_camera_obstacles', 'true')
        self.declare_parameter('use_lane_following', 'true')
        self.declare_parameter('lidar_priority', 'true')  # LiDAR overrides camera detections
        
        # Get parameters
        self.lidar_range_threshold = float(self.get_parameter('lidar_range_threshold').value)
        self.lidar_angle_range = float(self.get_parameter('lidar_angle_range').value)
        self.use_camera_obstacles = self.get_parameter('use_camera_obstacles').value.lower() == 'true'
        self.use_lane_following = self.get_parameter('use_lane_following').value.lower() == 'true'
        self.lidar_priority = self.get_parameter('lidar_priority').value.lower() == 'true'
        
        # State variables
        self.lidar_obstacle_detected = False
        self.lidar_distance = float('inf')
        self.lidar_direction = 0.0  # Direction to obstacle (radians)
        
        self.camera_obstacle_detected = False
        self.camera_obstacles = []
        
        self.lane_offset = 0.0
        self.lane_detected = False
        
        # Subscribers
        # 1. LiDAR Scan (HIGH PRIORITY)
        self.lidar_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            10
        )
        
        # 2. Camera Detections (MEDIUM PRIORITY)
        self.camera_obstacles_sub = self.create_subscription(
            String,
            '/obstacles_detected',
            self.camera_obstacles_callback,
            10
        )
        
        # 3. Lane Information
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
        # Fused obstacle detection (PRIMARY for navigation)
        self.fused_obstacles_pub = self.create_publisher(
            String,
            '/obstacles_fused',
            10
        )
        
        # Sensor fusion status (for debugging)
        self.fusion_status_pub = self.create_publisher(
            String,
            '/sensor_fusion_status',
            10
        )
        
        # Fused lane offset (only if lane is safe)
        self.fused_lane_offset_pub = self.create_publisher(
            Float32,
            '/lane_offset_fused',
            10
        )
        
        # Timer for publishing fused data
        self.timer = self.create_timer(0.1, self.publish_fused_data)
        
        self.get_logger().info(
            f"Sensor Fusion Node initialized\n"
            f"  LiDAR Priority: {self.lidar_priority}\n"
            f"  LiDAR Range Threshold: {self.lidar_range_threshold}m\n"
            f"  LiDAR Angle Range: {self.lidar_angle_range}°"
        )
    
    def lidar_callback(self, msg):
        """Process LiDAR scan (HIGH PRIORITY)"""
        try:
            # Get angle increment and min/max angle
            angle_increment = msg.angle_increment
            angle_min = msg.angle_min
            angle_max = msg.angle_max
            ranges = np.array(msg.ranges)
            
            # Calculate center angle and range (-30 to +30 degrees)
            center_angle_idx = len(ranges) // 2
            half_range_deg = self.lidar_angle_range / 2.0
            half_range_rad = math.radians(half_range_deg)
            
            # Extract front-facing scan (e.g., -30 to +30 degrees)
            start_angle = angle_min + (center_angle_idx * angle_increment)
            relevant_indices = []
            
            for i, angle in enumerate(np.arange(angle_min, angle_max, angle_increment)):
                if abs(angle) <= half_range_rad:
                    relevant_indices.append(i)
            
            if relevant_indices:
                relevant_ranges = ranges[relevant_indices]
                # Filter out invalid readings
                valid_ranges = relevant_ranges[(relevant_ranges > msg.range_min) & 
                                               (relevant_ranges < msg.range_max)]
                
                if len(valid_ranges) > 0:
                    min_distance = float(np.min(valid_ranges))
                    
                    # HIGH PRIORITY: Check for obstacles
                    if min_distance < self.lidar_range_threshold:
                        self.lidar_obstacle_detected = True
                        self.lidar_distance = min_distance
                        
                        # Find direction to closest obstacle
                        closest_idx = relevant_indices[np.argmin(relevant_ranges[relevant_indices])]
                        self.lidar_direction = angle_min + (closest_idx * angle_increment)
                    else:
                        self.lidar_obstacle_detected = False
                        self.lidar_distance = min_distance
        
        except Exception as e:
            self.get_logger().error(f"LiDAR processing error: {e}")
    
    def camera_obstacles_callback(self, msg):
        """Process camera detections (MEDIUM PRIORITY)"""
        try:
            obstacle_str = msg.data
            if "Obstacles detected:" in obstacle_str:
                obstacles_part = obstacle_str.split("Obstacles detected:")[-1]
                self.camera_obstacles = [obs.strip() for obs in obstacles_part.split(",")]
                self.camera_obstacle_detected = len(self.camera_obstacles) > 0
            else:
                self.camera_obstacle_detected = False
                self.camera_obstacles = []
        except Exception as e:
            self.get_logger().error(f"Camera obstacle parsing error: {e}")
    
    def lane_offset_callback(self, msg):
        """Receive lane offset"""
        self.lane_offset = msg.data
    
    def lane_status_callback(self, msg):
        """Receive lane status"""
        self.lane_detected = "detected" in msg.data.lower()
    
    def publish_fused_data(self):
        """Publish fused sensor data with prioritization"""
        try:
            # PRIMARY: LiDAR-based decision (HIGH PRIORITY)
            fused_obstacles = []
            
            if self.lidar_obstacle_detected:
                fused_obstacles.append(f"LiDAR obstacle at {self.lidar_distance:.2f}m")
                
                # If LiDAR detects obstacle, DON'T use lane following
                # (safety first - avoid hitting obstacle while following lane)
                lane_offset_to_pub = 0.0
            else:
                # LiDAR clear - can use lane following
                if self.camera_obstacle_detected:
                    fused_obstacles.extend(self.camera_obstacles)
                
                # Use lane offset only if no obstacles detected
                lane_offset_to_pub = self.lane_offset if self.lane_detected else 0.0
            
            # Publish fused obstacles
            fused_msg = String()
            if fused_obstacles:
                fused_msg.data = f"Fused obstacles: {', '.join(fused_obstacles)}"
            else:
                fused_msg.data = "No obstacles detected (LiDAR clear)"
            self.fused_obstacles_pub.publish(fused_msg)
            
            # Publish fused lane offset (safe only when no obstacles)
            lane_msg = Float32()
            lane_msg.data = lane_offset_to_pub
            self.fused_lane_offset_pub.publish(lane_msg)
            
            # Publish fusion status for debugging
            status_msg = String()
            status_msg.data = (
                f"LiDAR: {self.lidar_distance:.2f}m (obstacle={self.lidar_obstacle_detected}), "
                f"Camera: {self.camera_obstacles}, "
                f"Lane: {self.lane_offset:.2f} (detected={self.lane_detected})"
            )
            self.fusion_status_pub.publish(status_msg)
        
        except Exception as e:
            self.get_logger().error(f"Fusion publishing error: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = SensorFusionNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
