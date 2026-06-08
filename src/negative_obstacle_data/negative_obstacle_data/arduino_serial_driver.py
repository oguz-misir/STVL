"""
ROS 2 node: Arduino Mega 4WD serial driver.

Subscribes  /cmd_vel (Twist) → computes differential PWM → sends "L R\n" over serial
            at <400 ms intervals (Arduino freshness requirement).
Publishes   /wheel_odom  (nav_msgs/Odometry) from JS telemetry lines
            /robot_mode  (std_msgs/String)    from MODE telemetry lines
            /wheel_rpm   (std_msgs/Float32MultiArray) raw rpm: [FL FR RL RR]
Subscribes  /wheel_test_command (std_msgs/String) for manual wheel tests:
            "FL 100", "FR -80", "STOP", ...

Serial protocol (115200 8N1):
  Commands accepted by the Arduino sketch:
    "L R\n", "PWM L R\n", "L,R\n", "V\n", "STOP\n", "FL rpm\n", ...
  This ROS driver intentionally sends the pair-PWM form only:
    "L R\n"  e.g. "120 120\n" forward, "120 -120\n" right pivot,
    "-120 120\n" left pivot, "0 0\n" stop.
  Telemetry lines parsed:
    JS seq t_us FLrpm10 FRrpm10 RLrpm10 RRrpm10 pwmL pwmR FLcmps10 FRcmps10 RLcmps10 RRcmps10 RobotCmps10
    MODE [RC_KUMANDA_AKTIF | SERI_PWM_AKTIF | ...]
    RC   ch1_us ch2_us ch3_us ch1_ok ch2_ok ch3_ok

Requirements:  pyserial (pip install pyserial)
               CH2 must be passive for serial commands to take effect.
"""

from __future__ import annotations

import math
import threading
import time
import re
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Float32MultiArray
from geometry_msgs.msg import TransformStamped
import tf2_ros

try:
    import serial
    SERIAL_AVAILABLE = True
except ImportError:
    SERIAL_AVAILABLE = False


# Wheel geometry (from 4WD_USE_manual.tex)
WHEEL_RADIUS_M = 0.190          # 19.0 cm
WHEEL_CIRCUMFERENCE_M = 2.0 * math.pi * WHEEL_RADIUS_M   # ~1.194 m
TRACK_WIDTH_M = 0.34            # left-right axle separation, adjust per robot

# PWM limits
PWM_MAX = 200                   # absolute max PWM sent to Arduino
PWM_FORWARD_LIMIT = int(PWM_MAX * 0.60)   # 60% limit per manual

# Safety
ARDUINO_FRESHNESS_S = 0.4       # manual: pair PWM must be refreshed in ~400 ms
CMD_TIMEOUT_S = 0.35            # stop before Arduino freshness window expires
SEND_PERIOD_S = 0.15            # re-send interval (< 400 ms freshness)


class ArduinoSerialDriver(Node):
    def __init__(self) -> None:
        super().__init__("arduino_serial_driver")

        # Parameters
        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baud", 115200)
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("publish_tf", True)
        self.declare_parameter("max_linear_speed", 0.5)    # m/s → maps to PWM_FORWARD_LIMIT
        self.declare_parameter("max_angular_speed", 1.5)   # rad/s → maps to differential

        port  = self.get_parameter("port").get_parameter_value().string_value
        baud  = self.get_parameter("baud").get_parameter_value().integer_value
        self._base_frame  = self.get_parameter("base_frame").get_parameter_value().string_value
        self._odom_frame  = self.get_parameter("odom_frame").get_parameter_value().string_value
        self._publish_tf  = self.get_parameter("publish_tf").get_parameter_value().bool_value
        self._max_lin     = self.get_parameter("max_linear_speed").get_parameter_value().double_value
        self._max_ang     = self.get_parameter("max_angular_speed").get_parameter_value().double_value

        # Serial port
        self._ser: Optional[serial.Serial] = None
        if SERIAL_AVAILABLE:
            try:
                self._ser = serial.Serial(port, baud, timeout=0.05)
                time.sleep(2.0)  # let Arduino reboot after DTR reset
                self.get_logger().info(f"Serial opened: {port} @ {baud}")
            except Exception as exc:
                self.get_logger().error(f"Cannot open serial {port}: {exc}")
        else:
            self.get_logger().warn("pyserial not installed — serial I/O disabled (simulation mode)")

        # State
        self._pwm_l: int = 0
        self._pwm_r: int = 0
        self._last_cmd_time: float = time.monotonic()
        self._robot_mode: str = "UNKNOWN"
        self._wheel_test_active: bool = False
        self._lock = threading.Lock()

        # Odometry accumulator
        self._x: float = 0.0
        self._y: float = 0.0
        self._yaw: float = 0.0
        self._last_js_us: Optional[int] = None

        # Publishers
        reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self._odom_pub  = self.create_publisher(Odometry, "/wheel_odom", reliable)
        self._mode_pub  = self.create_publisher(String, "/robot_mode", reliable)
        self._rpm_pub   = self.create_publisher(Float32MultiArray, "/wheel_rpm", reliable)

        if self._publish_tf:
            self._tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Subscribers
        self.create_subscription(Twist, "/cmd_vel", self._cmd_vel_cb, 10)
        self.create_subscription(String, "/wheel_test_command", self._wheel_test_cb, 10)

        # Timers
        self.create_timer(SEND_PERIOD_S, self._send_timer_cb)

        # Serial read thread
        self._running = True
        self._read_thread = threading.Thread(target=self._serial_read_loop, daemon=True)
        self._read_thread.start()

        self.get_logger().info("ArduinoSerialDriver ready.")

    # ------------------------------------------------------------------
    # cmd_vel callback
    # ------------------------------------------------------------------
    def _cmd_vel_cb(self, msg: Twist) -> None:
        with self._lock:
            if self._wheel_test_active:
                return

        lin = msg.linear.x    # m/s
        ang = msg.angular.z   # rad/s

        # Normalize to [-1, 1]
        lin_norm = max(-1.0, min(1.0, lin / self._max_lin))
        ang_norm = max(-1.0, min(1.0, ang / self._max_ang))

        # Differential drive mixing.
        # ROS convention: +angular.z is a left/CCW turn. The Arduino manual
        # defines "-L +R" as left pivot and "+L -R" as right pivot.
        l_norm = lin_norm - ang_norm
        r_norm = lin_norm + ang_norm

        # Clamp to [-1, 1] after mixing
        l_norm = max(-1.0, min(1.0, l_norm))
        r_norm = max(-1.0, min(1.0, r_norm))

        pwm_l = int(l_norm * PWM_FORWARD_LIMIT)
        pwm_r = int(r_norm * PWM_FORWARD_LIMIT)

        with self._lock:
            self._pwm_l = pwm_l
            self._pwm_r = pwm_r
            self._last_cmd_time = time.monotonic()

    # ------------------------------------------------------------------
    # Manual wheel-test command callback
    # ------------------------------------------------------------------
    def _wheel_test_cb(self, msg: String) -> None:
        command = self._normalize_wheel_command(msg.data)
        if command is None:
            self.get_logger().warn(f"Invalid wheel test command ignored: {msg.data!r}")
            return

        with self._lock:
            if command == "STOP":
                self._wheel_test_active = False
                self._pwm_l = 0
                self._pwm_r = 0
                self._last_cmd_time = time.monotonic()
            else:
                self._wheel_test_active = True

        self._send_serial_line(command)

    @staticmethod
    def _normalize_wheel_command(text: str) -> str | None:
        value = text.strip().upper()
        if value in {"STOP", "0", "0 0", "PWM 0 0"}:
            return "STOP"

        match = re.fullmatch(r"(FL|FR|RL|RR)\s+(-?\d+)", value)
        if not match:
            return None

        wheel, rpm_text = match.groups()
        rpm = max(-300, min(300, int(rpm_text)))
        return f"{wheel} {rpm}"

    # ------------------------------------------------------------------
    # Periodic send timer (≤ 400 ms freshness)
    # ------------------------------------------------------------------
    def _send_timer_cb(self) -> None:
        now = time.monotonic()
        with self._lock:
            if self._wheel_test_active:
                return
            elapsed = now - self._last_cmd_time
            if elapsed > CMD_TIMEOUT_S:
                pwm_l, pwm_r = 0, 0
            else:
                pwm_l, pwm_r = self._pwm_l, self._pwm_r

        self._send_pwm(pwm_l, pwm_r)

    def _send_pwm(self, pwm_l: int, pwm_r: int) -> None:
        self._send_serial_line(f"{pwm_l} {pwm_r}")

    def _send_serial_line(self, line: str) -> None:
        if self._ser is None or not self._ser.is_open:
            return
        cmd = f"{line.strip()}\n".encode()
        try:
            self._ser.write(cmd)
        except Exception as exc:
            self.get_logger().warn(f"Serial write error: {exc}")

    # ------------------------------------------------------------------
    # Serial read loop (background thread)
    # ------------------------------------------------------------------
    def _serial_read_loop(self) -> None:
        while self._running:
            if self._ser is None or not self._ser.is_open:
                time.sleep(0.1)
                continue
            try:
                raw = self._ser.readline()
                if not raw:
                    continue
                line = raw.decode("ascii", errors="ignore").strip()
                if not line:
                    continue
                self._parse_line(line)
            except Exception as exc:
                self.get_logger().debug(f"Serial read error: {exc}")

    # ------------------------------------------------------------------
    # Telemetry line parser
    # ------------------------------------------------------------------
    def _parse_line(self, line: str) -> None:
        parts = line.split()
        if not parts:
            return

        tag = parts[0]

        if tag == "JS":
            self._parse_js(parts)
        elif tag == "MODE":
            self._parse_mode(parts)
        # RC lines are ignored (not needed for navigation)

    def _parse_js(self, parts: list[str]) -> None:
        # JS seq t_us FLrpm10 FRrpm10 RLrpm10 RRrpm10 pwmL pwmR FLcmps10 FRcmps10 RLcmps10 RRcmps10 RobotCmps10
        if len(parts) < 14:
            return
        try:
            t_us    = int(parts[2])
            fl_rpm  = int(parts[3]) / 10.0
            fr_rpm  = int(parts[4]) / 10.0
            rl_rpm  = int(parts[5]) / 10.0
            rr_rpm  = int(parts[6]) / 10.0
            fl_cmps = int(parts[9])  / 10.0   # cm/s
            fr_cmps = int(parts[10]) / 10.0
            rl_cmps = int(parts[11]) / 10.0
            rr_cmps = int(parts[12]) / 10.0
        except (ValueError, IndexError):
            return

        now = self.get_clock().now()

        # Publish raw RPM
        rpm_msg = Float32MultiArray()
        rpm_msg.data = [fl_rpm, fr_rpm, rl_rpm, rr_rpm]
        self._rpm_pub.publish(rpm_msg)

        # Convert cm/s → m/s, average left/right
        left_v  = (fl_cmps + rl_cmps) / 2.0 / 100.0   # m/s
        right_v = (fr_cmps + rr_cmps) / 2.0 / 100.0

        # Differential drive kinematics
        linear_v  = (left_v + right_v) / 2.0
        angular_v = (right_v - left_v) / TRACK_WIDTH_M

        # Integrate odometry using t_us delta
        if self._last_js_us is not None:
            dt_s = (t_us - self._last_js_us) / 1e6
            if 0.0 < dt_s < 2.0:      # guard against wrap or jitter (>2s = stale)
                self._yaw += angular_v * dt_s
                self._x   += linear_v * math.cos(self._yaw) * dt_s
                self._y   += linear_v * math.sin(self._yaw) * dt_s
        self._last_js_us = t_us

        # Build Odometry message
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self._odom_frame
        odom.child_frame_id  = self._base_frame

        odom.pose.pose.position.x = self._x
        odom.pose.pose.position.y = self._y
        odom.pose.pose.position.z = 0.0

        cy = math.cos(self._yaw * 0.5)
        sy = math.sin(self._yaw * 0.5)
        odom.pose.pose.orientation.w = cy
        odom.pose.pose.orientation.z = sy
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0

        odom.twist.twist.linear.x  = linear_v
        odom.twist.twist.angular.z = angular_v

        self._odom_pub.publish(odom)

        if self._publish_tf:
            self._broadcast_tf(now, odom)

    def _broadcast_tf(self, stamp, odom: Odometry) -> None:
        t = TransformStamped()
        t.header.stamp    = stamp.to_msg()
        t.header.frame_id = self._odom_frame
        t.child_frame_id  = self._base_frame
        t.transform.translation.x = odom.pose.pose.position.x
        t.transform.translation.y = odom.pose.pose.position.y
        t.transform.translation.z = 0.0
        t.transform.rotation      = odom.pose.pose.orientation
        self._tf_broadcaster.sendTransform(t)

    def _parse_mode(self, parts: list[str]) -> None:
        # MODE [RC_KUMANDA_AKTIF | SERI_PWM_AKTIF | ...]
        mode = parts[1] if len(parts) > 1 else "UNKNOWN"
        if mode != self._robot_mode:
            self._robot_mode = mode
            self.get_logger().info(f"Robot mode: {mode}")
        msg = String()
        msg.data = mode
        self._mode_pub.publish(msg)

    # ------------------------------------------------------------------
    # Cleanup
    # ------------------------------------------------------------------
    def destroy_node(self) -> None:
        self._running = False
        if self._ser and self._ser.is_open:
            try:
                self._ser.write(b"0 0\n")   # send stop before closing
                self._ser.close()
            except Exception:
                pass
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ArduinoSerialDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
