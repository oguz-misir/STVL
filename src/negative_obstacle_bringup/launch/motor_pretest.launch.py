"""Manual 4WD motor pre-test launch.

Starts only the Arduino serial bridge and the Tkinter motor GUI. This launch is
intended for bench/field motor checks before running the full navigation stack.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    port_arg = DeclareLaunchArgument("port", default_value="/dev/ttyACM0")
    baud_arg = DeclareLaunchArgument("baud", default_value="115200")
    publish_tf_arg = DeclareLaunchArgument("publish_tf", default_value="false")

    serial_driver = Node(
        package="negative_obstacle_data",
        executable="arduino_serial_driver",
        name="arduino_serial_driver",
        output="screen",
        parameters=[{
            "port": LaunchConfiguration("port"),
            "baud": LaunchConfiguration("baud"),
            "publish_tf": LaunchConfiguration("publish_tf"),
            "base_frame": "base_link",
            "odom_frame": "odom",
            "max_linear_speed": 0.5,
            "max_angular_speed": 1.5,
            "use_sim_time": False,
        }],
    )

    motor_gui = Node(
        package="negative_obstacle_data",
        executable="motor_control_gui",
        name="motor_control_gui",
        output="screen",
        parameters=[{
            "max_linear_speed": 0.5,
            "max_angular_speed": 1.5,
            "default_speed_ratio": 0.25,
            "default_wheel_rpm": 60,
        }],
    )

    return LaunchDescription([
        port_arg,
        baud_arg,
        publish_tf_arg,
        serial_driver,
        motor_gui,
    ])
