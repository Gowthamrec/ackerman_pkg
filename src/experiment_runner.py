#!/usr/bin/env python3
"""
Experiment Runner for Ackermann robot (Nav2 NavigateToPose)

Features:
- Runs N trials sequentially
- Resets robot in Gazebo via /set_entity_state (needs gazebo_ros_state plugin)
- Publishes /initialpose to AMCL
- Records metrics per run and writes results/<mode>_<timestamp>.json

Metrics collected:
- success flag, timeout flag, collisions (LaserScan threshold)
- completion time, path length, goal error
- avg/max speed, avg/max risk score, avg lane deviation

Parameters (all strings for launch compatibility):
- mode: baseline | lane_only | proposed
- num_runs: number of trials
- goal_x, goal_y, goal_yaw
- start_x, start_y, start_yaw
- timeout_sec
- collision_threshold (m)
- goal_tolerance (m)
- settle_time (s) wait after reset
- robot_name: Gazebo entity name (default: my_bot)
"""

import json
import math
import os
import time
from datetime import datetime
from threading import Lock

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32, String


class ExperimentRunner(Node):
    def __init__(self):
        super().__init__('experiment_runner')

        # Parameters
        self.declare_parameter('mode', 'proposed')
        self.declare_parameter('num_runs', '20')
        self.declare_parameter('goal_x', '5.0')
        self.declare_parameter('goal_y', '3.0')
        self.declare_parameter('goal_yaw', '0.0')
        self.declare_parameter('start_x', '0.0')
        self.declare_parameter('start_y', '0.0')
        self.declare_parameter('start_yaw', '0.0')
        self.declare_parameter('timeout_sec', '120.0')
        self.declare_parameter('collision_threshold', '0.18')
        self.declare_parameter('goal_tolerance', '0.5')
        self.declare_parameter('settle_time', '3.0')
        self.declare_parameter('robot_name', 'my_bot')
        self.declare_parameter('vary_positions', 'true')

        # Parameter values
        self.mode = self.get_parameter('mode').value
        self.num_runs = int(self.get_parameter('num_runs').value)
        self.goal_x = float(self.get_parameter('goal_x').value)
        self.goal_y = float(self.get_parameter('goal_y').value)
        self.goal_yaw = float(self.get_parameter('goal_yaw').value)
        self.start_x = float(self.get_parameter('start_x').value)
        self.start_y = float(self.get_parameter('start_y').value)
        self.start_yaw = float(self.get_parameter('start_yaw').value)
        self.timeout_sec = float(self.get_parameter('timeout_sec').value)
        self.collision_threshold = float(self.get_parameter('collision_threshold').value)
        self.goal_tolerance = float(self.get_parameter('goal_tolerance').value)
        self.settle_time = float(self.get_parameter('settle_time').value)
        self.robot_name = self.get_parameter('robot_name').value
        self.vary_positions = self.get_parameter('vary_positions').value.lower() == 'true'

        self.mode_descriptions = {
            'baseline': 'Nav2 only',
            'lane_only': 'Nav2 + lane follower',
            'proposed': 'Nav2 + risk-based fusion + lane follower',
            'weight_variant1': 'Weight tuning: α=0.6, β=0.3, γ=0.1',
            'weight_variant2': 'Weight tuning: α=0.5, β=0.3, γ=0.2 (baseline)',
            'weight_variant3': 'Weight tuning: α=0.4, β=0.4, γ=0.2',
        }

        # Callback group
        self.cb_group = ReentrantCallbackGroup()

        # Nav2 action client
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose', callback_group=self.cb_group)

        # Gazebo reset service (Gazebo Classic)
        self.reset_client = self.create_client(SetModelState, '/gazebo/set_model_state', callback_group=self.cb_group)

        # Initial pose publisher
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)

        # Subscribers
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10, callback_group=self.cb_group)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10, callback_group=self.cb_group)
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10, callback_group=self.cb_group)
        self.create_subscription(Float32, '/risk_score', self.risk_score_callback, 10, callback_group=self.cb_group)
        self.create_subscription(String, '/risk_level', self.risk_level_callback, 10, callback_group=self.cb_group)
        self.create_subscription(Float32, '/lane_offset', self.lane_offset_callback, 10, callback_group=self.cb_group)

        # State
        self.lock = Lock()
        self._reset_accumulators()
        self.running = False
        self.current_run = 0
        self.nav_goal_handle = None
        self.nav_succeeded = None

        # Results
        # Results paths (per-run files + summary)
        base_results_dir = os.path.join('/home/ros2/car_project_ws', 'results', self.mode)
        os.makedirs(base_results_dir, exist_ok=True)
        ts = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.run_file_template = os.path.join(base_results_dir, 'run_{:02d}.json')
        self.output_file = os.path.join(base_results_dir, f'{self.mode}_summary_{ts}.json')
        self.all_runs = []

        self.get_logger().info('=' * 70)
        self.get_logger().info('EXPERIMENT RUNNER')
        self.get_logger().info(f'Mode: {self.mode} — {self.mode_descriptions.get(self.mode, "?")}')
        self.get_logger().info(f'Runs: {self.num_runs}')
        self.get_logger().info(f'Goal: ({self.goal_x}, {self.goal_y}, yaw={self.goal_yaw})')
        self.get_logger().info(f'Start: ({self.start_x}, {self.start_y}, yaw={self.start_yaw})')
        self.get_logger().info(f'Output (summary): {self.output_file}')
        self.get_logger().info(f'Per-run files: {self.run_file_template.format(1)} ...')
        self.get_logger().info('=' * 70)

        # Experiments will start when main() calls run_experiments()
        self.experiments_started = False

    # ────────────────────────
    # Callbacks
    # ────────────────────────
    def odom_callback(self, msg: Odometry):
        if not self.running:
            return
        with self.lock:
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            self.current_x = x
            self.current_y = y
            if self.prev_odom is not None:
                dx = x - self.prev_odom[0]
                dy = y - self.prev_odom[1]
                d = math.sqrt(dx * dx + dy * dy)
                if d < 2.0:  # ignore teleports
                    self.path_length += d
            self.prev_odom = (x, y)

    def scan_callback(self, msg: LaserScan):
        if not self.running:
            return
        ranges = np.array(msg.ranges)
        valid = ranges[(ranges > msg.range_min) & (ranges < msg.range_max)]
        if len(valid) == 0:
            return
        min_d = float(np.min(valid))
        with self.lock:
            if min_d < self.min_obstacle_dist:
                self.min_obstacle_dist = min_d
            if min_d < self.collision_threshold:
                # basic collision hit counter with cooldown
                now = time.time()
                if now - self.collision_cooldown > 2.0:
                    self.collision_count += 1
                    self.collision_cooldown = now

    def cmd_vel_callback(self, msg: Twist):
        if not self.running:
            return
        speed = math.sqrt(msg.linear.x ** 2 + msg.linear.y ** 2)
        with self.lock:
            self.speed_samples.append(speed)

    def risk_score_callback(self, msg: Float32):
        if not self.running:
            return
        with self.lock:
            self.risk_samples.append(msg.data)

    def risk_level_callback(self, msg: String):
        if not self.running:
            return
        level = msg.data.strip().upper()
        with self.lock:
            self.risk_level_counts[level] = self.risk_level_counts.get(level, 0) + 1

    def lane_offset_callback(self, msg: Float32):
        if not self.running:
            return
        with self.lock:
            self.lane_offset_samples.append(abs(msg.data))

    # ────────────────────────
    # Experiment flow
    # ────────────────────────
    def run_experiments(self):
        """Public method to start experiments (called from main)"""
        if self.experiments_started:
            return
        self.experiments_started = True
        self._run_all()

    def _run_all(self):
        self.get_logger().info('Waiting for Nav2 action server...')
        self.get_logger().info('(Make sure Nav2 is running in a separate terminal!)')
        if not self.nav_client.wait_for_server(timeout_sec=120.0):
            self.get_logger().error('❌ Nav2 action server NOT AVAILABLE after 120 seconds!')
            self.get_logger().error('   Please ensure Nav2 is running:')
            self.get_logger().error('   ros2 launch ackerman_pkg nav2.launch.py')
            return
        self.get_logger().info('✓ Nav2 action server connected.')

        for run_id in range(1, self.num_runs + 1):
            self.current_run = run_id
            
            # Vary positions if enabled (avoids reset service)
            if self.vary_positions:
                start_x, start_y, goal_x, goal_y = self._get_varied_positions(run_id)
            else:
                start_x, start_y = self.start_x, self.start_y
                goal_x, goal_y = self.goal_x, self.goal_y
            
            self.get_logger().info('\n' + '=' * 50)
            self.get_logger().info(f'  RUN {run_id}/{self.num_runs}')
            self.get_logger().info(f'  Start: ({start_x:.1f}, {start_y:.1f}) → Goal: ({goal_x:.1f}, {goal_y:.1f})')
            self.get_logger().info('=' * 50)

            # 1) Reset robot (skip if varying positions - AMCL will localize)
            if not self.vary_positions:
                self._reset_robot()
                self._sleep(self.settle_time)

            # 2) Publish initial pose
            self._publish_initial_pose(start_x, start_y, self.start_yaw)
            self._sleep(2.0 if not self.vary_positions else 0.5)

            # 3) Reset accumulators
            with self.lock:
                self._reset_accumulators()
                self.run_start_time = time.time()
                self.run_start_iso = datetime.now().isoformat()

            # 4) Send goal
            self.running = True
            self.nav_succeeded = None
            self._send_goal(goal_x, goal_y, self.goal_yaw)

            # 5) Wait for completion or timeout
            deadline = time.time() + self.timeout_sec
            while self.nav_succeeded is None and time.time() < deadline:
                rclpy.spin_once(self, timeout_sec=0.1)

            self.running = False
            elapsed = time.time() - self.run_start_time

            # 6) Evaluate
            with self.lock:
                dist_to_goal = math.sqrt((self.current_x - goal_x) ** 2 + (self.current_y - goal_y) ** 2)

            timed_out = self.nav_succeeded is None
            nav_ok = self.nav_succeeded is True
            reached = dist_to_goal < self.goal_tolerance

            # Reject instant successes (likely already at goal)
            too_fast = (elapsed < 1.0 and self.path_length < 0.5)
            if too_fast:
                self.get_logger().warn(
                    f'  ⚠ Rejecting instant success (elapsed={elapsed:.1f}s path={self.path_length:.2f}m)')

            # Success if: reached goal within tolerance, moved reasonable distance, no collisions
            # Allow success even if Nav2 timed out, as long as robot physically reached goal
            success = (reached and not too_fast and self.collision_count == 0 and self.path_length > 1.0)

            # Record
            with self.lock:
                record = self._build_record(run_id, success, reached, elapsed, dist_to_goal, timed_out)
            self.all_runs.append(record)

            # Save per-run file immediately
            self._save_run_file(run_id, record)

            status = '✓ SUCCESS' if success else '✗ FAIL'
            reason = ''
            if too_fast:
                reason = ' (invalid: start at goal)'
            elif not reached:
                reason = f' (dist={dist_to_goal:.2f}m > {self.goal_tolerance}m)'
            elif self.collision_count > 0:
                reason = f' (collisions={self.collision_count})'
            elif timed_out and reached:
                reason = ' (slow but reached goal)'

            self.get_logger().info(
                f'  {status}{reason} | time={elapsed:.1f}s | path={self.path_length:.2f}m | '
                f'collisions={self.collision_count}')

            # Cancel goal if still active
            if self.nav_goal_handle is not None:
                try:
                    self.nav_goal_handle.cancel_goal_async()
                except Exception:
                    pass
                self.nav_goal_handle = None

            # Save incremental results
            self._save_results()

        self._print_summary()
        self.get_logger().info(f'Results saved to: {self.output_file}')
        
        # Signal completion - main() will handle shutdown
        self.get_logger().info('Experiment complete.')

    # ────────────────────────
    # Helpers
    # ────────────────────────
    def _reset_accumulators(self):
        self.prev_odom = None
        self.path_length = 0.0
        self.current_x = 0.0
        self.current_y = 0.0
        self.speed_samples = []
        self.risk_samples = []
        self.risk_level_counts = {}
        self.lane_offset_samples = []
        self.min_obstacle_dist = float('inf')
        self.collision_count = 0
        self.collision_cooldown = 0.0
        self.run_start_time = None
        self.run_start_iso = None

    def _reset_robot(self):
        # Wait longer for Gazebo service (can be slow to advertise)
        if not self.reset_client.wait_for_service(timeout_sec=15.0):
            self.get_logger().error('  ✗ Gazebo /gazebo/set_model_state NOT available! Robot will NOT be reset. Results invalid.')
            return

        req = SetModelState.Request()
        req.model_state = ModelState()
        req.model_state.model_name = self.robot_name
        req.model_state.pose.position.x = self.start_x
        req.model_state.pose.position.y = self.start_y
        req.model_state.pose.position.z = 0.25
        req.model_state.pose.orientation = self._yaw_to_quat(self.start_yaw)
        req.model_state.twist.linear.x = 0.0
        req.model_state.twist.linear.y = 0.0
        req.model_state.twist.angular.z = 0.0
        req.model_state.reference_frame = 'world'

        fut = self.reset_client.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        if fut.done() and fut.result() and fut.result().success:
            self.get_logger().info('  Robot reset to start')
        else:
            self.get_logger().warn('  ⚠ Reset service call failed')

    def _get_varied_positions(self, run_id):
        """Alternate: origin→goal, goal→origin. No reset needed."""
        # Goal positions for odd runs
        goal_positions = [
            (5.0, 3.0), (4.0, 2.0), (6.0, 3.5), (5.5, 2.5),
            (4.5, 3.5), (5.0, 2.0), (6.0, 2.5), (4.0, 3.0),
        ]
        
        # Determine which goal to use (cycles through list)
        goal_idx = ((run_id - 1) // 2) % len(goal_positions)
        goal_x, goal_y = goal_positions[goal_idx]
        
        # Odd runs: origin → goal, Even runs: goal → origin
        if run_id % 2 == 1:  # Odd: go to goal
            start_x, start_y = 0.0, 0.0
            end_x, end_y = goal_x, goal_y
        else:  # Even: return to origin
            start_x, start_y = goal_x, goal_y
            end_x, end_y = 0.0, 0.0
        
        return start_x, start_y, end_x, end_y
    
    def _publish_initial_pose(self, x=None, y=None, yaw=None):
        if x is None:
            x, y, yaw = self.start_x, self.start_y, self.start_yaw
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.orientation = self._yaw_to_quat(yaw)
        msg.pose.covariance = [0.0] * 36
        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.07
        self.initial_pose_pub.publish(msg)
        self.get_logger().info('  Initial pose published')

    def _send_goal(self, goal_x=None, goal_y=None, goal_yaw=None):
        if goal_x is None:
            goal_x, goal_y, goal_yaw = self.goal_x, self.goal_y, self.goal_yaw
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = goal_x
        goal.pose.pose.position.y = goal_y
        goal.pose.pose.orientation = self._yaw_to_quat(goal_yaw)

        self.get_logger().info(f'  Sending goal -> ({goal_x}, {goal_y})')
        fut = self.nav_client.send_goal_async(goal)
        fut.add_done_callback(self._goal_response_cb)

    def _goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('  Goal rejected by Nav2')
            self.nav_succeeded = False
            return
        self.nav_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._nav_result_cb)

    def _nav_result_cb(self, future):
        try:
            res = future.result()
            self.nav_succeeded = (res.status == 4)
        except Exception:
            self.nav_succeeded = False

    def _build_record(self, run_id, success, goal_reached, elapsed, dist_to_goal, timed_out):
        avg_speed = float(np.mean(self.speed_samples)) if self.speed_samples else 0.0
        max_speed = float(np.max(self.speed_samples)) if self.speed_samples else 0.0
        avg_risk = float(np.mean(self.risk_samples)) if self.risk_samples else 0.0
        max_risk = float(np.max(self.risk_samples)) if self.risk_samples else 0.0
        std_risk = float(np.std(self.risk_samples)) if self.risk_samples else 0.0
        avg_lane = float(np.mean(self.lane_offset_samples)) if self.lane_offset_samples else 0.0

        total_levels = sum(self.risk_level_counts.values())
        level_dist = {k: (v / total_levels if total_levels else 0.0) for k, v in self.risk_level_counts.items()}

        return {
            'run_id': run_id,
            'start_time': self.run_start_iso,
            'success': success,
            'goal_reached': goal_reached,
            'timed_out': timed_out,
            'collision_count': self.collision_count,
            'completion_time_sec': round(elapsed, 2),
            'path_length_m': round(self.path_length, 3),
            'goal_distance_error_m': round(dist_to_goal, 3),
            'avg_speed_mps': round(avg_speed, 4),
            'max_speed_mps': round(max_speed, 4),
            'avg_risk_score': round(avg_risk, 4),
            'max_risk_score': round(max_risk, 4),
            'std_risk_score': round(std_risk, 4),
            'risk_level_distribution': level_dist,
            'min_obstacle_dist_m': round(self.min_obstacle_dist, 3) if self.min_obstacle_dist != float('inf') else None,
            'avg_lane_deviation': round(avg_lane, 4),
            'speed_samples_count': len(self.speed_samples),
            'risk_samples_count': len(self.risk_samples),
        }

    def _save_results(self):
        data = {
            'experiment_config': {
                'mode': self.mode,
                'mode_description': self.mode_descriptions.get(self.mode, 'unknown'),
                'num_runs': self.num_runs,
                'start_pose': {'x': self.start_x, 'y': self.start_y, 'yaw': self.start_yaw},
                'goal_pose': {'x': self.goal_x, 'y': self.goal_y, 'yaw': self.goal_yaw},
                'timeout_sec': self.timeout_sec,
                'collision_threshold_m': self.collision_threshold,
                'goal_tolerance_m': self.goal_tolerance,
                'timestamp': datetime.now().isoformat(),
            },
            'runs': self.all_runs,
            'summary': self._summary(),
        }
        with open(self.output_file, 'w') as f:
            json.dump(data, f, indent=2)

    def _save_run_file(self, run_id, record):
        run_path = self.run_file_template.format(run_id)
        payload = {
            'run_id': run_id,
            'mode': self.mode,
            'mode_description': self.mode_descriptions.get(self.mode, 'unknown'),
            'start_pose': {'x': self.start_x, 'y': self.start_y, 'yaw': self.start_yaw},
            'goal_pose': {'x': self.goal_x, 'y': self.goal_y, 'yaw': self.goal_yaw},
            'timeout_sec': self.timeout_sec,
            'collision_threshold_m': self.collision_threshold,
            'goal_tolerance_m': self.goal_tolerance,
            'record': record,
            'logged_topics': ['/odom', '/cmd_vel', '/risk_score', '/risk_level', '/scan'],
        }
        with open(run_path, 'w') as f:
            json.dump(payload, f, indent=2)

    def _summary(self):
        if not self.all_runs:
            return {}
        runs = self.all_runs
        successes = [r for r in runs if r['success']]
        times = [r['completion_time_sec'] for r in runs]
        paths = [r['path_length_m'] for r in runs]
        speeds = [r['avg_speed_mps'] for r in runs]
        risks = [r['avg_risk_score'] for r in runs]
        lanes = [r['avg_lane_deviation'] for r in runs]
        collisions = [r['collision_count'] for r in runs]

        return {
            'total_runs': len(runs),
            'successful_runs': len(successes),
            'success_rate': round(len(successes) / len(runs), 4),
            'total_collisions': sum(collisions),
            'avg_collisions_per_run': round(float(np.mean(collisions)), 3),
            'avg_completion_time_sec': round(float(np.mean(times)), 2),
            'std_completion_time_sec': round(float(np.std(times)), 2),
            'avg_path_length_m': round(float(np.mean(paths)), 3),
            'std_path_length_m': round(float(np.std(paths)), 3),
            'avg_speed_mps': round(float(np.mean(speeds)), 4),
            'std_speed_mps': round(float(np.std(speeds)), 4),
            'avg_risk_score': round(float(np.mean(risks)), 4),
            'std_risk_score': round(float(np.std(risks)), 4),
            'avg_lane_deviation': round(float(np.mean(lanes)), 4),
            'std_lane_deviation': round(float(np.std(lanes)), 4),
        }

    def _print_summary(self):
        s = self._summary()
        if not s:
            return
        self.get_logger().info('\n' + '=' * 70)
        self.get_logger().info(f'  EXPERIMENT COMPLETE — Mode: {self.mode}')
        self.get_logger().info('=' * 70)
        self.get_logger().info(
            f'  Success Rate: {s["successful_runs"]}/{s["total_runs"]} ({s["success_rate"]*100:.1f}%)')
        self.get_logger().info(f'  Total Collisions: {s["total_collisions"]}')
        self.get_logger().info(
            f'  Avg Time: {s["avg_completion_time_sec"]:.1f}s (±{s["std_completion_time_sec"]:.1f})')
        self.get_logger().info(
            f'  Avg Path Length: {s["avg_path_length_m"]:.2f}m (±{s["std_path_length_m"]:.2f})')
        self.get_logger().info(
            f'  Avg Speed: {s["avg_speed_mps"]:.3f} m/s (±{s["std_speed_mps"]:.3f})')
        self.get_logger().info(
            f'  Avg Risk Score: {s["avg_risk_score"]:.3f} (±{s["std_risk_score"]:.3f})')
        self.get_logger().info(
            f'  Avg Lane Dev: {s["avg_lane_deviation"]:.3f} (±{s["std_lane_deviation"]:.3f})')
        self.get_logger().info('=' * 70)

    @staticmethod
    def _yaw_to_quat(yaw):
        from geometry_msgs.msg import Quaternion
        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(yaw / 2.0)
        q.w = math.cos(yaw / 2.0)
        return q

    def _sleep(self, seconds: float):
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)


def main(args=None):
    rclpy.init(args=args)
    node = ExperimentRunner()
    try:
        # Give time for subscribers to connect
        time.sleep(2.0)
        # Run experiments (blocking until all runs complete)
        node.run_experiments()
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted — saving results...')
        node._save_results()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
