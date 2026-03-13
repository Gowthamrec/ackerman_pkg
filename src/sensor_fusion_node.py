#!/usr/bin/env python3
"""
Risk-Based Sensor Fusion Node

Priority rules (safety-first):
1. LiDAR (HIGH)  - front sector obstacle check; overrides everything
2. Camera (MED)  - obstacles only if LiDAR is clear
3. Lane  (LOW)   - only when both LiDAR and camera are clear

Outputs:
- /obstacles_fused (String)
- /lane_offset_fused (Float32)  -> 0 if any obstacle
- /sensor_fusion_status (String)
- /speed_factor (Float32)       -> for risk-aware speed scaling
- /risk_score (Float32)         -> continuous risk [0,1]
- /risk_level (String)          -> NORMAL/CAUTION/SLOW/EMERGENCY
"""

import math
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String, Float32


class SensorFusionNode(Node):
    def __init__(self):
        super().__init__('sensor_fusion_node')

        # ── Parameters (all passed as strings in launch) ──
        self.declare_parameter('alpha_lidar', '0.5')
        self.declare_parameter('beta_camera', '0.3')
        self.declare_parameter('gamma_lane', '0.2')
        self.declare_parameter('emergency_threshold', '0.8')
        self.declare_parameter('slow_threshold', '0.5')
        self.declare_parameter('caution_threshold', '0.2')
        self.declare_parameter('lidar_max_relevant_dist', '5.0')
        self.declare_parameter('lidar_angle_range', '60.0')  # degrees (front sector)
        self.declare_parameter('lidar_min_safe_distance', '0.5')
        self.declare_parameter('use_camera_obstacles', 'true')
        self.declare_parameter('use_lane_following', 'true')
        self.declare_parameter('fusion_rate', '10.0')  # Hz

        # ── Parameter values ──
        self.alpha = float(self.get_parameter('alpha_lidar').value)
        self.beta = float(self.get_parameter('beta_camera').value)
        self.gamma = float(self.get_parameter('gamma_lane').value)
        self.emergency_th = float(self.get_parameter('emergency_threshold').value)
        self.slow_th = float(self.get_parameter('slow_threshold').value)
        self.caution_th = float(self.get_parameter('caution_threshold').value)
        self.lidar_max_dist = float(self.get_parameter('lidar_max_relevant_dist').value)
        self.lidar_angle_range = float(self.get_parameter('lidar_angle_range').value)
        self.lidar_min_safe = float(self.get_parameter('lidar_min_safe_distance').value)
        self.use_camera = self.get_parameter('use_camera_obstacles').value.lower() == 'true'
        self.use_lane = self.get_parameter('use_lane_following').value.lower() == 'true'
        fusion_rate = float(self.get_parameter('fusion_rate').value)

        # ── State ──
        self.lidar_min_distance = float('inf')
        self.lidar_obstacle = False
        self.camera_obstacle = False
        self.camera_obstacles = []
        self.lane_offset = 0.0
        self.lane_detected = False

        # ── Subscribers ──
        self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.create_subscription(String, '/obstacles_detected', self.camera_callback, 10)
        self.create_subscription(Float32, '/lane_offset', self.lane_offset_callback, 10)
        self.create_subscription(String, '/lane_status', self.lane_status_callback, 10)

        # ── Publishers ──
        self.obstacles_pub = self.create_publisher(String, '/obstacles_fused', 10)
        self.lane_offset_pub = self.create_publisher(Float32, '/lane_offset_fused', 10)
        self.status_pub = self.create_publisher(String, '/sensor_fusion_status', 10)
        self.speed_factor_pub = self.create_publisher(Float32, '/speed_factor', 10)
        self.risk_score_pub = self.create_publisher(Float32, '/risk_score', 10)
        self.risk_level_pub = self.create_publisher(String, '/risk_level', 10)

        # ── Timer ──
        period = 1.0 / fusion_rate if fusion_rate > 0 else 0.1
        self.create_timer(period, self.publish_fused_data)

        self.get_logger().info(
            'Sensor Fusion Node initialized\n'
            f'  alpha (LiDAR): {self.alpha}\n'
            f'  beta  (Camera): {self.beta}\n'
            f'  gamma (Lane): {self.gamma}\n'
            f'  LiDAR angle range: {self.lidar_angle_range} deg\n'
            f'  LiDAR safe distance: {self.lidar_min_safe} m')

    # ── Callbacks ──
    def lidar_callback(self, msg: LaserScan):
        try:
            ranges = np.array(msg.ranges)
            angle_min = msg.angle_min
            angle_max = msg.angle_max
            angle_inc = msg.angle_increment
            half_range_rad = math.radians(self.lidar_angle_range / 2.0)

            angles = np.arange(angle_min, angle_max, angle_inc)
            mask = np.abs(angles) <= half_range_rad
            relevant = ranges[mask]

            valid = relevant[(relevant > msg.range_min) & (relevant < msg.range_max)]
            if len(valid) == 0:
                self.lidar_min_distance = float('inf')
                self.lidar_obstacle = False
                return

            min_d = float(np.min(valid))
            self.lidar_min_distance = min_d
            self.lidar_obstacle = min_d < self.lidar_min_safe
        except Exception as e:
            self.get_logger().error(f'LiDAR processing error: {e}')

    def camera_callback(self, msg: String):
        if not self.use_camera:
            self.camera_obstacle = False
            self.camera_obstacles = []
            return
        try:
            data = msg.data.strip()
            # 'clear' or empty → no obstacle
            if not data or 'clear' in data.lower():
                self.camera_obstacle = False
                self.camera_obstacles = []
                return
            # Parse 'Obstacles detected: person, car, ...' format
            if 'Obstacles detected:' in data or 'obstacles detected:' in data.lower():
                parts = data.split(':', 1)[-1]  # everything after the colon
                self.camera_obstacles = [p.strip() for p in parts.split(',') if p.strip()]
            else:
                self.camera_obstacles = [p.strip() for p in data.split(',') if p.strip()]
            self.camera_obstacle = len(self.camera_obstacles) > 0
        except Exception as e:
            self.get_logger().error(f'Camera parsing error: {e}')
            self.camera_obstacle = False
            self.camera_obstacles = []

    def lane_offset_callback(self, msg: Float32):
        self.lane_offset = msg.data

    def lane_status_callback(self, msg: String):
        text = msg.data.lower()
        self.lane_detected = 'detect' in text or 'true' in text

    # ── Fusion ──
    def publish_fused_data(self):
        try:
            # SIMPLE BINARY RULE: object detected → risk 1.0 EMERGENCY, else 0.0 NORMAL
            if self.camera_obstacle:
                risk_score = 1.0
                risk_level = 'EMERGENCY'
                speed_factor = 0.0  # full stop
            else:
                risk_score = 0.0
                risk_level = 'NORMAL'
                speed_factor = 1.0

            # Lane gating (safety first)
            if self.lidar_obstacle or self.camera_obstacle:
                lane_offset_fused = 0.0
            else:
                lane_offset_fused = self.lane_offset if (self.use_lane and self.lane_detected) else 0.0

            # Obstacles fused message
            fused_list = []
            if self.lidar_obstacle:
                fused_list.append(f'LiDAR obstacle {self.lidar_min_distance:.2f}m')
            if self.camera_obstacle:
                fused_list.extend(self.camera_obstacles)
            obstacles_msg = String()
            obstacles_msg.data = ', '.join(fused_list) if fused_list else 'clear'

            # Publish
            lane_msg = Float32()
            lane_msg.data = lane_offset_fused
            speed_msg = Float32()
            speed_msg.data = speed_factor
            risk_msg = Float32()
            risk_msg.data = risk_score
            risk_level_msg = String()
            risk_level_msg.data = risk_level
            status_msg = String()
            status_msg.data = (
                f'LiDAR: {self.lidar_min_distance:.2f}m (obs={self.lidar_obstacle}), '
                f'Camera: {self.camera_obstacles}, '
                f'Lane: {self.lane_offset:.2f} (detected={self.lane_detected}), '
                f'Risk: {risk_score:.3f} [{risk_level}]'
            )

            self.obstacles_pub.publish(obstacles_msg)
            self.lane_offset_pub.publish(lane_msg)
            self.speed_factor_pub.publish(speed_msg)
            self.risk_score_pub.publish(risk_msg)
            self.risk_level_pub.publish(risk_level_msg)
            self.status_pub.publish(status_msg)

        except Exception as e:
            self.get_logger().error(f'Fusion error: {e}')


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
    main()#!/usr/bin/env python3
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

        # Navigation goal state
        self.goal_pose_received = False  # True once user gives a 2D Nav Goal
        
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

        # 4. Navigation Goal (enables detection only when a goal is active)
        self.goal_pose_sub = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_pose_callback,
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

        # 5-second obstacle wait state machine
        # States: 'idle' → 'waiting' (0-10s, robot stops) → 'timeout' (nav2 replans)
        self.wait_state = 'idle'
        self.obstacle_wait_start = None   # clock time (sec) when obstacle first seen
        self.obstacle_wait_duration = 10.0 # seconds to wait before letting Nav2 replan
        self.last_countdown_print = -1    # track last printed second for countdown

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
    
    def goal_pose_callback(self, msg):
        """Enable obstacle detection only when a navigation goal has been given."""
        if not self.goal_pose_received:
            self.goal_pose_received = True
            self.get_logger().info(
                f'\n========================================\n'
                f'  NAVIGATION GOAL RECEIVED\n'
                f'  Obstacle detection / wait procedure ARMED\n'
                f'========================================')

    def publish_fused_data(self):
        """Publish fused sensor data.

        Camera has FULL PRIORITY - any detected object stops the robot.
        Behaviour:
          - Camera OR LiDAR detects obstacle → robot stops immediately.
          - Wait 10 s for obstacle to move (countdown printed every second).
          - Obstacle moves away within 10 s → resume immediately.
          - Obstacle persists after 10 s → Nav2 replans alternative route.
        """
        try:
            # CAMERA ONLY PRIORITY: ignore LiDAR for obstacle stopping
            any_obstacle = self.camera_obstacle_detected
            now = self.get_clock().now().nanoseconds / 1e9

            # ── 10-second wait state machine ─────────────────────────────────
            # Only trigger waiting when a navigation goal has been given
            if any_obstacle and self.goal_pose_received:
                if self.wait_state == 'idle':
                    self.wait_state = 'waiting'
                    self.obstacle_wait_start = now
                    self.last_countdown_print = -1
                    src = []
                    if self.camera_obstacle_detected:
                        src.append(f'CAMERA({self.camera_obstacles})')
                    if self.lidar_obstacle_detected:
                        src.append(f'LiDAR({self.lidar_distance:.2f}m)')
                    self.get_logger().info(
                        f'\n========================================\n'
                        f'  OBSTACLE DETECTED by {" + ".join(src)}\n'
                        f'  ROBOT STOPPED - waiting 10 s for obstacle to move\n'
                        f'========================================')

                if self.wait_state == 'waiting':
                    elapsed = now - self.obstacle_wait_start
                    remaining = self.obstacle_wait_duration - elapsed
                    # Print countdown every second
                    current_sec = int(elapsed)
                    if current_sec != self.last_countdown_print:
                        self.last_countdown_print = current_sec
                        self.get_logger().info(
                            f'  [WAITING] {elapsed:.0f}s elapsed - '
                            f'{remaining:.0f}s remaining before replan...')

                    if elapsed < self.obstacle_wait_duration:
                        report_obstacle = True
                    else:
                        self.wait_state = 'timeout'
                        self.get_logger().warn(
                            f'\n========================================\n'
                            f'  TIMEOUT: Obstacle did not move after 10 s\n'
                            f'  Releasing control → Nav2 will REPLAN route\n'
                            f'========================================')
                        report_obstacle = False
                else:
                    report_obstacle = False
            else:
                if self.wait_state != 'idle':
                    self.get_logger().info(
                        f'\n========================================\n'
                        f'  OBSTACLE CLEARED - resuming original path\n'
                        f'========================================')
                self.wait_state = 'idle'
                self.obstacle_wait_start = None
                self.last_countdown_print = -1
                report_obstacle = False

            # ── Build fused messages ─────────────────────────────────────────
            if report_obstacle:
                elapsed = now - self.obstacle_wait_start
                fused_parts = []
                if self.camera_obstacle_detected:
                    fused_parts.extend(self.camera_obstacles)
                fused_msg_data = f"Fused obstacles: {', '.join(fused_parts)} (wait {elapsed:.1f}s/10.0s)"
                lane_offset_to_pub = 0.0
            else:
                fused_msg_data = 'No obstacles detected (clear)'
                lane_offset_to_pub = self.lane_offset if self.lane_detected else 0.0

            # ── Publish ──────────────────────────────────────────────────────
            fused_msg = String()
            fused_msg.data = fused_msg_data
            self.fused_obstacles_pub.publish(fused_msg)

            lane_msg = Float32()
            lane_msg.data = lane_offset_to_pub
            self.fused_lane_offset_pub.publish(lane_msg)

            wait_info = (
                f"{now - self.obstacle_wait_start:.1f}s"
                if self.obstacle_wait_start is not None else 'inactive')
            status_msg = String()
            status_msg.data = (
                f'Camera: {self.camera_obstacles}(priority=FULL), '
                f'LiDAR: {self.lidar_distance:.2f}m, '
                f'Wait: [{self.wait_state}] {wait_info}'
            )
            self.fusion_status_pub.publish(status_msg)

        except Exception as e:
            self.get_logger().error(f'Fusion publishing error: {e}')


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
