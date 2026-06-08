"""Unit tests for arduino_serial_driver telemetry parser (no hardware, no ROS)."""

import math
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

# Minimal stubs so the module imports without rclpy / serial
import types

# stub rclpy
rclpy_stub = types.ModuleType("rclpy")
rclpy_stub.node = types.ModuleType("rclpy.node")

class _Node:
    def declare_parameter(self, *a, **kw): pass
    def get_parameter(self, name):
        class _P:
            def get_parameter_value(self_):
                class _V:
                    string_value = "/dev/ttyUSB0"
                    integer_value = 115200
                    bool_value = True
                    double_value = 0.5
                return _V()
        return _P()
    def create_publisher(self, *a, **kw): return None
    def create_subscription(self, *a, **kw): pass
    def create_timer(self, *a, **kw): pass
    def get_logger(self):
        class _L:
            info = warn = error = debug = staticmethod(lambda *a, **kw: None)
        return _L()
    def get_clock(self):
        class _C:
            def now(self_):
                class _T:
                    def to_msg(self__): return None
                return _T()
        return _C()

rclpy_stub.node.Node = _Node
sys.modules["rclpy"] = rclpy_stub
sys.modules["rclpy.node"] = rclpy_stub.node

for mod in ["rclpy.qos", "rclpy.qos", "geometry_msgs", "geometry_msgs.msg",
            "nav_msgs", "nav_msgs.msg", "std_msgs", "std_msgs.msg",
            "tf2_ros"]:
    sys.modules[mod] = types.ModuleType(mod)

# stub QoS classes used at module-level in arduino_serial_driver
import rclpy.qos as _qos
_qos.QoSProfile = lambda **kw: None
_qos.ReliabilityPolicy = type("RP", (), {"RELIABLE": 1})()
_qos.HistoryPolicy = type("HP", (), {"KEEP_LAST": 1})()

# stub message types
import geometry_msgs.msg as _gm
import nav_msgs.msg as _nm
import std_msgs.msg as _sm

class _Twist: linear = type("L", (), {"x": 0.0})(); angular = type("Z", (), {"z": 0.0})()
class _Odometry:
    header = type("H", (), {"stamp": None, "frame_id": ""})()
    child_frame_id = ""
    pose = type("P", (), {"pose": type("P2", (), {
        "position": type("Pos", (), {"x": 0.0, "y": 0.0, "z": 0.0})(),
        "orientation": type("Q", (), {"w": 1.0, "x": 0.0, "y": 0.0, "z": 0.0})(),
    })()})()
    twist = type("T", (), {"twist": type("T2", (), {
        "linear": type("L", (), {"x": 0.0})(),
        "angular": type("Z", (), {"z": 0.0})(),
    })()})()

class _String: data = ""
class _Float32MultiArray: data = []
class _TransformStamped:
    header = type("H", (), {"stamp": None, "frame_id": ""})()
    child_frame_id = ""
    transform = type("T", (), {
        "translation": type("V", (), {"x": 0.0, "y": 0.0, "z": 0.0})(),
        "rotation":    type("Q", (), {"w": 1.0, "x": 0.0, "y": 0.0, "z": 0.0})(),
    })()

_gm.Twist = _Twist
_nm.Odometry = _Odometry
_sm.String = _String
_sm.Float32MultiArray = _Float32MultiArray
_gm.TransformStamped = _TransformStamped

sys.modules["tf2_ros"].TransformBroadcaster = lambda *a, **kw: type("TFB", (), {"sendTransform": lambda *a, **kw: None})()

# Now import the driver
from negative_obstacle_data.arduino_serial_driver import (
    ArduinoSerialDriver, PWM_FORWARD_LIMIT, CMD_TIMEOUT_S, WHEEL_RADIUS_M, TRACK_WIDTH_M,
    ARDUINO_FRESHNESS_S, SEND_PERIOD_S,
)


# -------------------------------------------------------------------------
# Helpers — instantiate driver bypassing serial
# -------------------------------------------------------------------------
def make_driver() -> ArduinoSerialDriver:
    d = object.__new__(ArduinoSerialDriver)
    d._ser = None
    d._pwm_l = 0
    d._pwm_r = 0
    d._last_cmd_time = 0.0
    d._robot_mode = "UNKNOWN"
    import threading
    d._lock = threading.Lock()
    d._x = 0.0
    d._y = 0.0
    d._yaw = 0.0
    d._last_js_us = None
    d._base_frame = "base_link"
    d._odom_frame = "odom"
    d._publish_tf = False
    d._max_lin = 0.5
    d._max_ang = 1.5
    d._running = False
    d._wheel_test_active = False

    # stub publishers
    class _Pub:
        def __init__(self): self.last = None
        def publish(self, msg): self.last = msg
    d._odom_pub = _Pub()
    d._mode_pub = _Pub()
    d._rpm_pub  = _Pub()
    return d


# -------------------------------------------------------------------------
# Tests
# -------------------------------------------------------------------------
def test_pwm_forward():
    d = make_driver()
    msg = _Twist()
    msg.linear.x = 0.25
    msg.angular.z = 0.0
    import time; d._last_cmd_time = time.monotonic()
    d._cmd_vel_cb(msg)
    assert d._pwm_l == d._pwm_r
    assert d._pwm_l > 0


def test_pwm_stop():
    d = make_driver()
    msg = _Twist()
    msg.linear.x = 0.0
    msg.angular.z = 0.0
    d._cmd_vel_cb(msg)
    assert d._pwm_l == 0
    assert d._pwm_r == 0


def test_pwm_rotate_left():
    d = make_driver()
    msg = _Twist()
    msg.linear.x = 0.0
    msg.angular.z = 1.0
    d._cmd_vel_cb(msg)
    assert d._pwm_l < 0      # Arduino manual: -L +R = left pivot
    assert d._pwm_r > 0


def test_pwm_rotate_right():
    d = make_driver()
    msg = _Twist()
    msg.linear.x = 0.0
    msg.angular.z = -1.0
    d._cmd_vel_cb(msg)
    assert d._pwm_l > 0      # Arduino manual: +L -R = right pivot
    assert d._pwm_r < 0


def test_pwm_clamped():
    d = make_driver()
    msg = _Twist()
    msg.linear.x = 999.0    # way over max
    msg.angular.z = 0.0
    d._cmd_vel_cb(msg)
    assert abs(d._pwm_l) <= PWM_FORWARD_LIMIT
    assert abs(d._pwm_r) <= PWM_FORWARD_LIMIT


def test_js_parse_forward():
    """Forward motion: both sides equal positive speed."""
    d = make_driver()
    # JS seq t_us FLrpm10 FRrpm10 RLrpm10 RRrpm10 pwmL pwmR FLcmps10 FRcmps10 RLcmps10 RRcmps10 RobotCmps10
    #        FL=FR=RL=RR=200 rpm, cmps=300 (30.0 cm/s each)
    parts = "JS 1 1000000 2000 2000 2000 2000 120 120 300 300 300 300 300".split()
    d._parse_js(parts)
    assert d._odom_pub.last is not None
    odom = d._odom_pub.last
    assert odom.twist.twist.linear.x > 0.0
    assert abs(odom.twist.twist.angular.z) < 1e-6


def test_js_parse_stationary():
    d = make_driver()
    parts = "JS 1 1000000 0 0 0 0 0 0 0 0 0 0 0".split()
    d._parse_js(parts)
    odom = d._odom_pub.last
    assert abs(odom.twist.twist.linear.x)  < 1e-9
    assert abs(odom.twist.twist.angular.z) < 1e-9


def test_js_odometry_accumulation():
    """Two consecutive JS lines should accumulate position."""
    d = make_driver()
    # t=0: set baseline
    p1 = "JS 1 0 2000 2000 2000 2000 120 120 300 300 300 300 300".split()
    d._parse_js(p1)
    x_before = d._x
    # t=1s later, still moving forward
    p2 = "JS 2 1000000 2000 2000 2000 2000 120 120 300 300 300 300 300".split()
    d._parse_js(p2)
    assert d._x > x_before    # moved forward


def test_js_bad_line_ignored():
    d = make_driver()
    d._parse_js("JS 1 abc".split())
    assert d._odom_pub.last is None


def test_js_short_line_ignored():
    d = make_driver()
    d._parse_js("JS 1".split())
    assert d._odom_pub.last is None


def test_rpm_published():
    d = make_driver()
    parts = "JS 1 1000000 500 600 500 600 120 120 300 360 300 360 330".split()
    d._parse_js(parts)
    rpm = d._rpm_pub.last
    assert rpm is not None
    assert abs(rpm.data[0] - 50.0) < 1e-6   # FLrpm10=500 → 50.0 RPM
    assert abs(rpm.data[1] - 60.0) < 1e-6


def test_mode_parse_and_publish():
    d = make_driver()
    d._parse_mode("MODE SERI_PWM_AKTIF".split())
    assert d._robot_mode == "SERI_PWM_AKTIF"
    assert d._mode_pub.last.data == "SERI_PWM_AKTIF"


def test_mode_parse_unknown():
    d = make_driver()
    d._parse_mode(["MODE"])   # no second token
    assert d._robot_mode == "UNKNOWN"


def test_parse_line_dispatch():
    d = make_driver()
    d._parse_line("MODE RC_KUMANDA_AKTIF")
    assert d._robot_mode == "RC_KUMANDA_AKTIF"
    d._parse_line("JS 1 2000000 0 0 0 0 0 0 0 0 0 0 0")
    assert d._odom_pub.last is not None


def test_parse_line_empty():
    d = make_driver()
    d._parse_line("")   # should not crash
    d._parse_line("   ")


def test_wheel_constants():
    assert abs(WHEEL_RADIUS_M - 0.19) < 1e-6
    assert TRACK_WIDTH_M > 0.0
    assert 0 < PWM_FORWARD_LIMIT <= 200
    assert 0.0 < CMD_TIMEOUT_S < ARDUINO_FRESHNESS_S
    assert 0.0 < SEND_PERIOD_S < ARDUINO_FRESHNESS_S


def test_normalize_wheel_command_accepts_manual_format():
    assert ArduinoSerialDriver._normalize_wheel_command("FL 100") == "FL 100"
    assert ArduinoSerialDriver._normalize_wheel_command("rr -80") == "RR -80"
    assert ArduinoSerialDriver._normalize_wheel_command("STOP") == "STOP"


def test_normalize_wheel_command_clamps_rpm():
    assert ArduinoSerialDriver._normalize_wheel_command("FR 999") == "FR 300"
    assert ArduinoSerialDriver._normalize_wheel_command("RL -999") == "RL -300"


def test_normalize_wheel_command_rejects_bad_format():
    assert ArduinoSerialDriver._normalize_wheel_command("LEFT 100") is None
    assert ArduinoSerialDriver._normalize_wheel_command("FL fast") is None
