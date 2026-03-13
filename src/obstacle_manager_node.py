#!/usr/bin/env python3
"""
Smart Obstacle Manager Node
-----------------------------
Monitors /scan for critical-range obstacles and implements:

  STATE MACHINE:
  ┌─────────────┐   obstacle < critical_range    ┌─────────────────┐
  │  MONITORING │ ──────────────────────────────► │  CRITICAL_STOP  │
  └─────────────┘                                 │  (send zeros,   │
        ▲                                         │   record scan)  │
        │                                         └────────┬────────┘
        │                                                  │ 5 seconds elapsed
        │                                         ┌────────▼────────┐
        │                                         │    CHECKING     │
        │                                         │ compare LiDAR   │
        │                                         │ before vs after │
        │                                         └────────┬────────┘
        │                              moved?              │
        │            ┌──────────────────────────┬──────────┘
        │            │ YES (dynamic object)      │ NO (static object)
        │   ┌────────▼─────────┐      ┌─────────▼──────────┐
        └───│  RESUMING        │      │  REPLANNING         │
            │  Release control │      │  Cancel Nav2 goal,  │
            │  Nav2 takes over │      │  resubmit goal →    │
            └──────────────────┘      │  Nav2 replans path  │
                                      └─────────────────────┘

TOPICS:
  Subscribes:
    /scan              (LaserScan)  — obstacle monitoring
    /cmd_vel           (Twist)      — from lane_follower, pass through or zero
    /risk_level        (String)     — current risk level
    /current_goal      (PoseStamped)— optional: current Nav2 goal for replan

  Publishes:
    /cmd_vel_safe      (Twist)      — to arduino_bridge (zeroed when stopping)
    /obstacle_status   (String)     — current state machine state
    /replan_trigger    (Bool)       — signals Nav2 to replan

PARAMETERS:
    critical_range_m      : 0.30   Distance (m) triggering emergency stop
    stop_duration_sec     : 5.0   Seconds to wait before checking
    movement_threshold_m  : 0.15   Min change in scan to consider object moved
    check_angle_deg       : 60.0   Front sector angle to watch
    pass_rate_hz          : 20.0   Timer rate for cmd_vel passthrough
"""

import math
import time
import numpy as np
import threading

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import String, Bool
from nav2_msgs.action import NavigateToPose


class ObstacleManagerNode(Node):

    # State machine states
    MONITORING = 'MONITORING'
    CRITICAL_STOP = 'CRITICAL_STOP'
    CHECKING = 'CHECKING'
    RESUMING = 'RESUMING'
    REPLANNING = 'REPLANNING'

    def __init__(self):
        super().__init__('obstacle_manager_node')

        # ── Parameters ──
        self.declare_parameter('critical_range_m', '0.30')
        self.declare_parameter('stop_duration_sec', '5.0')
        self.declare_parameter('movement_threshold_m', '0.15')
        self.declare_parameter('check_angle_deg', '60.0')
        self.declare_parameter('pass_rate_hz', '20.0')

        self.critical_range = float(self.get_parameter('critical_range_m').value)
        self.stop_duration = float(self.get_parameter('stop_duration_sec').value)
        self.move_thresh = float(self.get_parameter('movement_threshold_m').value)
        self.check_angle = float(self.get_parameter('check_angle_deg').value)
        pass_rate = float(self.get_parameter('pass_rate_hz').value)

        # ── State ──
        self.state = self.MONITORING
        self.state_lock = threading.Lock()
        self.latest_cmd = Twist()           # Latest /cmd_vel from lane_follower
        self.current_scan = None            # Latest LaserScan
        self.scan_at_stop = None            # LiDAR reading when stop triggered
        self.stop_start_time = None         # When the stop began
        self.current_goal = None            # Current Nav2 goal (for replanning)
        self.risk_level = 'NORMAL'

        # ── Subscribers ──
        self.create_subscription(LaserScan, '/scan', self._scan_callback, 10)
        self.create_subscription(Twist, '/cmd_vel', self._cmd_vel_callback, 10)
        self.create_subscription(String, '/risk_level', self._risk_callback, 10)
        self.create_subscription(PoseStamped, '/current_goal', self._goal_callback, 10)

        # ── Publishers ──
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel_safe', 10)
        self.status_pub = self.create_publisher(String, '/obstacle_status', 10)
        self.replan_pub = self.create_publisher(Bool, '/replan_trigger', 10)

        # ── Nav2 Action Client (for replanning) ──
        self._nav2_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # ── Timers ──
        self.create_timer(1.0 / pass_rate, self._control_loop)
        self.create_timer(0.5, self._publish_status)

        self.get_logger().info(
            f'Obstacle Manager initialized\n'
            f'  Critical range : {self.critical_range} m\n'
            f'  Stop duration  : {self.stop_duration} s\n'
            f'  Move threshold : {self.move_thresh} m\n'
            f'  Check angle    : {self.check_angle}°'
        )

    # ──────────────────────────────────────────────────────────────
    # Callbacks
    # ──────────────────────────────────────────────────────────────
    def _scan_callback(self, msg: LaserScan):
        self.current_scan = msg

    def _cmd_vel_callback(self, msg: Twist):
        self.latest_cmd = msg

    def _risk_callback(self, msg: String):
        self.risk_level = msg.data.strip().upper() if msg.data else 'NORMAL'

    def _goal_callback(self, msg: PoseStamped):
        self.current_goal = msg

    # ──────────────────────────────────────────────────────────────
    # LiDAR Utilities
    # ──────────────────────────────────────────────────────────────
    def _get_front_distances(self, scan: LaserScan) -> np.ndarray:
        """Extract valid distances in front sector (±half_angle)."""
        if scan is None:
            return np.array([])
        ranges = np.array(scan.ranges, dtype=np.float32)
        angle_inc = scan.angle_increment
        angle_min = scan.angle_min
        half_rad = math.radians(self.check_angle / 2.0)

        angles = angle_min + np.arange(len(ranges)) * angle_inc
        mask = np.abs(angles) <= half_rad
        front = ranges[mask]
        valid = front[(front > scan.range_min) & (front < scan.range_max)
                      & np.isfinite(front)]
        return valid

    def _min_front_distance(self, scan: LaserScan) -> float:
        """Return minimum distance in front sector. inf if no obstacles."""
        valid = self._get_front_distances(scan)
        return float(np.min(valid)) if len(valid) > 0 else float('inf')

    def _obstacle_moved(self, scan_before: LaserScan, scan_after: LaserScan) -> bool:
        """
        Compare front scans to detect if obstacle moved.
        Returns True  → obstacle moved (dynamic object, safe to resume)
        Returns False → obstacle still there (static, need replan)
        """
        if scan_before is None or scan_after is None:
            return False

        before = self._get_front_distances(scan_before)
        after = self._get_front_distances(scan_after)

        # If front is now clear → object definitely moved
        if len(after) == 0 or np.min(after) > self.critical_range * 2.0:
            self.get_logger().info('✅ Front clear after wait → dynamic obstacle')
            return True

        # If readings changed significantly → object moved
        if len(before) > 0 and len(after) > 0:
            mean_change = abs(np.median(before) - np.median(after))
            self.get_logger().info(
                f'Obstacle check: before_median={np.median(before):.2f}m '
                f'after_median={np.median(after):.2f}m '
                f'change={mean_change:.2f}m (threshold={self.move_thresh}m)'
            )
            if mean_change > self.move_thresh:
                return True

        return False

    # ──────────────────────────────────────────────────────────────
    # Nav2 Replanning
    # ──────────────────────────────────────────────────────────────
    def _trigger_replan(self):
        """
        Cancel current goal and resubmit it.
        Nav2 will compute a new path around the static obstacle
        (which is now marked in the costmap from LiDAR).
        """
        replan_msg = Bool()
        replan_msg.data = True
        self.replan_pub.publish(replan_msg)

        if self.current_goal is None:
            self.get_logger().warn(
                'Replan triggered but no /current_goal received. '
                'Nav2 will auto-replan via behavior tree.'
            )
            return

        if not self._nav2_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error('Nav2 action server not available for replan')
            return

        self.get_logger().info('🔄 Sending new NavigateToPose goal for replan...')
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self.current_goal
        self._nav2_client.send_goal_async(goal_msg)

    # ──────────────────────────────────────────────────────────────
    # Main Control Loop (State Machine)
    # ──────────────────────────────────────────────────────────────
    def _control_loop(self):
        with self.state_lock:
            self._run_state_machine()

    def _run_state_machine(self):
        scan = self.current_scan
        min_dist = self._min_front_distance(scan)

        # ── MONITORING: normal pass-through ────────────────────────
        if self.state == self.MONITORING:
            # Check for critical obstacle
            if min_dist < self.critical_range:
                self.get_logger().warn(
                    f'🚨 CRITICAL obstacle at {min_dist:.2f}m '
                    f'(threshold: {self.critical_range}m) → STOPPING'
                )
                self.scan_at_stop = scan       # Record scan before stop
                self.stop_start_time = time.time()
                self.state = self.CRITICAL_STOP
                self._publish_zero()
            else:
                # Normal: pass latest cmd_vel through
                self.cmd_pub.publish(self.latest_cmd)

        # ── CRITICAL_STOP: send zeros for stop_duration ─────────────
        elif self.state == self.CRITICAL_STOP:
            self._publish_zero()
            elapsed = time.time() - self.stop_start_time
            remaining = self.stop_duration - elapsed

            if elapsed >= self.stop_duration:
                self.get_logger().info(
                    f'⏱ {self.stop_duration}s wait complete → CHECKING if obstacle moved'
                )
                self.state = self.CHECKING
            else:
                self.get_logger().info(
                    f'🛑 STOPPED — checking in {remaining:.1f}s '
                    f'(obstacle at {min_dist:.2f}m)',
                    throttle_duration_sec=1.0
                )

        # ── CHECKING: compare scan before and after ─────────────────
        elif self.state == self.CHECKING:
            moved = self._obstacle_moved(self.scan_at_stop, scan)
            if moved:
                self.get_logger().info(
                    '✅ Dynamic obstacle — resuming original path'
                )
                self.state = self.RESUMING
            else:
                self.get_logger().warn(
                    '⚠ Static obstacle — triggering Nav2 replan'
                )
                self.state = self.REPLANNING
                self._trigger_replan()

        # ── RESUMING: release control, back to monitoring ────────────
        elif self.state == self.RESUMING:
            # Pass command through; return to monitoring
            self.cmd_pub.publish(self.latest_cmd)
            # Only return to monitoring if obstacle cleared
            if min_dist >= self.critical_range:
                self.get_logger().info('↩ Path clear — back to MONITORING')
                self.state = self.MONITORING

        # ── REPLANNING: hold stop while Nav2 computes new path ───────
        elif self.state == self.REPLANNING:
            self._publish_zero()
            # Once obstacle clears (Nav2 found new path and robot moves away)
            if min_dist >= self.critical_range * 1.5:
                self.get_logger().info('↩ New path clear — back to MONITORING')
                self.state = self.MONITORING

    def _publish_zero(self):
        """Publish zero velocity — complete stop."""
        stop = Twist()
        stop.linear.x = 0.0
        stop.angular.z = 0.0
        self.cmd_pub.publish(stop)

    def _publish_status(self):
        scan = self.current_scan
        min_dist = self._min_front_distance(scan)
        msg = String()
        msg.data = (
            f'state={self.state} '
            f'min_dist={min_dist:.2f}m '
            f'risk={self.risk_level}'
        )
        self.status_pub.publish(msg)
        self.get_logger().debug(msg.data)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
