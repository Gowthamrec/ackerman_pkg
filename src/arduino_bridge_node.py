#!/usr/bin/env python3
"""
Arduino Mega Serial Bridge Node
---------------------------------
Converts ROS2 /cmd_vel_safe (Twist) → Serial commands → Arduino Mega
The Arduino then drives motors and steering servo.

SERIAL PROTOCOL (sent to Arduino):
    Format:  S<speed_pwm>,T<steer_pwm>\n
    speed_pwm : -255 to +255  (negative = reverse)
    steer_pwm :    0 to  180  (degrees, 90 = straight)
    Example:  S128,T90\n   → half-forward, straight
    Example:  S0,T70\n     → stopped, turn left
    Example:  S-80,T110\n  → reverse slightly, slight right

ARDUINO EXPECTS (paste this in Arduino IDE):
    See comment block at bottom of this file.

WIRING (Arduino Mega):
    Pin 2  → Motor Driver PWM (speed)
    Pin 3  → Motor Driver DIR (direction)
    Pin 9  → Steering Servo Signal
    GND    → Common ground with motor driver
    USB    → Connected to PC running ROS2

USAGE:
    ros2 run ackerman_pkg arduino_bridge_node.py
    OR included in autonomous_complete.launch.py

PARAMETERS:
    serial_port    : /dev/ttyUSB0  (or /dev/ttyACM0 for Mega)
    baud_rate      : 115200
    max_speed_mps  : max m/s from robot (used for scaling)
    max_steer_rad  : max steering rad (maps to servo range)
    steer_center   : servo center degree (default 90)
    steer_range    : degrees each side of center (default 45)
    speed_max_pwm  : max PWM value for motor (default 200 out of 255)
    cmd_timeout    : seconds before sending stop if no cmd_vel received
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Bool
import serial
import serial.tools.list_ports
import threading
import time


class ArduinoBridgeNode(Node):
    def __init__(self):
        super().__init__('arduino_bridge_node')

        # ── Parameters ──
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', '115200')
        self.declare_parameter('max_speed_mps', '1.0')
        self.declare_parameter('max_steer_rad', '0.6')
        self.declare_parameter('steer_center', '90')
        self.declare_parameter('steer_range', '45')
        self.declare_parameter('speed_max_pwm', '200')
        self.declare_parameter('cmd_timeout', '0.5')

        self.serial_port = self.get_parameter('serial_port').value
        self.baud_rate = int(self.get_parameter('baud_rate').value)
        self.max_speed_mps = float(self.get_parameter('max_speed_mps').value)
        self.max_steer_rad = float(self.get_parameter('max_steer_rad').value)
        self.steer_center = int(self.get_parameter('steer_center').value)
        self.steer_range = int(self.get_parameter('steer_range').value)
        self.speed_max_pwm = int(self.get_parameter('speed_max_pwm').value)
        self.cmd_timeout = float(self.get_parameter('cmd_timeout').value)

        # ── State ──
        self.serial_conn = None
        self.last_cmd_time = self.get_clock().now()
        self.connected = False
        self.last_speed_pwm = 0
        self.last_steer_pwm = self.steer_center

        # ── Subscribers ──
        # Reads from obstacle_manager output (safest velocity)
        self.create_subscription(Twist, '/cmd_vel_safe', self.cmd_vel_callback, 10)

        # ── Publishers ──
        self.status_pub = self.create_publisher(String, '/arduino_status', 10)
        self.connected_pub = self.create_publisher(Bool, '/arduino_connected', 10)

        # ── Connect to Arduino ──
        self._connect_serial()

        # ── Safety watchdog: stop if no cmd received ──
        self.create_timer(0.1, self._watchdog_callback)

        # ── Status publisher ──
        self.create_timer(1.0, self._publish_status)

        self.get_logger().info(
            f'Arduino Bridge Node initialized\n'
            f'  Port: {self.serial_port} @ {self.baud_rate} baud\n'
            f'  Speed max PWM: {self.speed_max_pwm}\n'
            f'  Steer center: {self.steer_center}° ± {self.steer_range}°\n'
            f'  Cmd timeout (watchdog): {self.cmd_timeout}s'
        )

    # ──────────────────────────────────────────────────────────────
    # Serial Connection
    # ──────────────────────────────────────────────────────────────
    def _connect_serial(self):
        """Connect to Arduino Mega serial port."""
        try:
            self.serial_conn = serial.Serial(
                port=self.serial_port,
                baudrate=self.baud_rate,
                timeout=1.0
            )
            time.sleep(2.0)  # Wait for Arduino to reset after connection
            self.connected = True
            self.get_logger().info(f'✅ Arduino connected on {self.serial_port}')
        except serial.SerialException as e:
            self.connected = False
            self.get_logger().error(
                f'❌ Cannot open {self.serial_port}: {e}\n'
                f'  Available ports: {self._list_ports()}\n'
                f'  Try: ls /dev/ttyACM* /dev/ttyUSB*'
            )

    def _list_ports(self):
        """List available serial ports for diagnostics."""
        ports = serial.tools.list_ports.comports()
        return [p.device for p in ports] if ports else ['none found']

    # ──────────────────────────────────────────────────────────────
    # Velocity → PWM Conversion
    # ──────────────────────────────────────────────────────────────
    def _velocity_to_speed_pwm(self, linear_x: float) -> int:
        """
        Convert linear velocity (m/s) to motor PWM.
        linear_x positive = forward, negative = reverse
        Returns: -speed_max_pwm to +speed_max_pwm
        """
        if abs(linear_x) < 0.01:
            return 0
        ratio = linear_x / self.max_speed_mps
        ratio = max(-1.0, min(1.0, ratio))
        return int(ratio * self.speed_max_pwm)

    def _angular_to_steer_pwm(self, angular_z: float) -> int:
        """
        Convert angular velocity (rad/s) to steering servo degrees.
        angular_z positive = turn left (CCW), negative = turn right (CW)
        Returns: steer_center ± steer_range degrees
        """
        ratio = angular_z / self.max_steer_rad
        ratio = max(-1.0, min(1.0, ratio))
        # Invert: positive angular_z (left turn) → servo below center
        steer = self.steer_center - int(ratio * self.steer_range)
        return max(self.steer_center - self.steer_range,
                   min(self.steer_center + self.steer_range, steer))

    # ──────────────────────────────────────────────────────────────
    # Send to Arduino
    # ──────────────────────────────────────────────────────────────
    def _send_command(self, speed_pwm: int, steer_pwm: int):
        """
        Send command to Arduino over serial.
        Format: S<speed>,T<steer>\n
        """
        if not self.connected or self.serial_conn is None:
            return

        try:
            cmd = f'S{speed_pwm},T{steer_pwm}\n'
            self.serial_conn.write(cmd.encode('utf-8'))
            self.last_speed_pwm = speed_pwm
            self.last_steer_pwm = steer_pwm
            self.get_logger().debug(f'→ Arduino: {cmd.strip()}')
        except serial.SerialException as e:
            self.get_logger().error(f'Serial write error: {e}')
            self.connected = False

    def _send_stop(self):
        """Send immediate stop command."""
        self._send_command(0, self.steer_center)

    # ──────────────────────────────────────────────────────────────
    # Callbacks
    # ──────────────────────────────────────────────────────────────
    def cmd_vel_callback(self, msg: Twist):
        """Receive /cmd_vel_safe and send to Arduino."""
        self.last_cmd_time = self.get_clock().now()

        speed_pwm = self._velocity_to_speed_pwm(msg.linear.x)
        steer_pwm = self._angular_to_steer_pwm(msg.angular.z)
        self._send_command(speed_pwm, steer_pwm)

    def _watchdog_callback(self):
        """Stop motors if no cmd_vel received within timeout."""
        now = self.get_clock().now()
        elapsed = (now - self.last_cmd_time).nanoseconds / 1e9
        if elapsed > self.cmd_timeout:
            self._send_stop()
            self.get_logger().warn(
                f'Watchdog: no cmd_vel for {elapsed:.1f}s → STOP', throttle_duration_sec=2.0)

        # Try reconnect if disconnected
        if not self.connected:
            self._connect_serial()

    def _publish_status(self):
        """Publish connection status."""
        status = String()
        status.data = (
            f'connected={self.connected} port={self.serial_port} '
            f'speed={self.last_speed_pwm} steer={self.last_steer_pwm}'
        )
        self.status_pub.publish(status)

        connected_msg = Bool()
        connected_msg.data = self.connected
        self.connected_pub.publish(connected_msg)

    def destroy_node(self):
        """Clean shutdown: stop motors before exit."""
        self._send_stop()
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ArduinoBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


# ═══════════════════════════════════════════════════════════════════════
# ARDUINO MEGA CODE — Upload this to your Arduino Mega
# ═══════════════════════════════════════════════════════════════════════
#
# Paste the following in Arduino IDE and upload to Arduino Mega:
#
# -----------------------------------------------------------------------
# #include <Servo.h>
#
# // ── Pin definitions ──
# const int MOTOR_PWM_PIN = 2;   // Motor driver ENA / PWM
# const int MOTOR_DIR_PIN = 3;   // Motor driver IN1 (direction)
# const int STEER_PIN     = 9;   // Steering servo signal
#
# // ── Limits ──
# const int STEER_MIN  = 45;    // Minimum servo angle
# const int STEER_MAX  = 135;   // Maximum servo angle
# const int STEER_MID  = 90;    // Straight ahead
# const int SPEED_MAX  = 200;   // Max PWM (out of 255)
# const unsigned long TIMEOUT_MS = 500; // Stop if no data for 500ms
#
# Servo steerServo;
# unsigned long lastReceived = 0;
#
# void setup() {
#   Serial.begin(115200);
#   pinMode(MOTOR_PWM_PIN, OUTPUT);
#   pinMode(MOTOR_DIR_PIN, OUTPUT);
#   steerServo.attach(STEER_PIN);
#   steerServo.write(STEER_MID);
#   stopMotors();
#   Serial.println("READY");
# }
#
# void loop() {
#   // ── Read serial command ──
#   if (Serial.available() > 0) {
#     String cmd = Serial.readStringUntil('\n');
#     cmd.trim();
#     if (cmd.length() > 0) {
#       parseCommand(cmd);
#       lastReceived = millis();
#     }
#   }
#
#   // ── Watchdog: stop if no command for TIMEOUT_MS ──
#   if (millis() - lastReceived > TIMEOUT_MS) {
#     stopMotors();
#     steerServo.write(STEER_MID);
#   }
# }
#
# void parseCommand(String cmd) {
#   // Expected format: S<speed>,T<steer>
#   // Example: S128,T90
#   int sIdx = cmd.indexOf('S');
#   int tIdx = cmd.indexOf(',');
#
#   if (sIdx < 0 || tIdx < 0) return;  // invalid
#
#   int speedVal = cmd.substring(sIdx + 1, tIdx).toInt();
#   int steerVal = cmd.substring(tIdx + 2).toInt();  // skip 'T'
#
#   // ── Apply steering ──
#   steerVal = constrain(steerVal, STEER_MIN, STEER_MAX);
#   steerServo.write(steerVal);
#
#   // ── Apply speed ──
#   speedVal = constrain(speedVal, -SPEED_MAX, SPEED_MAX);
#   if (speedVal == 0) {
#     stopMotors();
#   } else if (speedVal > 0) {
#     digitalWrite(MOTOR_DIR_PIN, HIGH);   // Forward
#     analogWrite(MOTOR_PWM_PIN, speedVal);
#   } else {
#     digitalWrite(MOTOR_DIR_PIN, LOW);    // Reverse
#     analogWrite(MOTOR_PWM_PIN, -speedVal);
#   }
# }
#
# void stopMotors() {
#   analogWrite(MOTOR_PWM_PIN, 0);
#   digitalWrite(MOTOR_DIR_PIN, LOW);
# }
# -----------------------------------------------------------------------
