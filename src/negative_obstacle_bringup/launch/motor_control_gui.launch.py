"""Launch only the 4WD manual motor control GUI."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    max_linear_arg = DeclareLaunchArgument("max_linear_speed", default_value="0.5")
    max_angular_arg = DeclareLaunchArgument("max_angular_speed", default_value="1.5")
    speed_ratio_arg = DeclareLaunchArgument("default_speed_ratio", default_value="0.35")
    wheel_rpm_arg = DeclareLaunchArgument("default_wheel_rpm", default_value="80")

    gui = Node(
        package="negative_obstacle_data",
        executable="motor_control_gui",
        name="motor_control_gui",
        output="screen",
        parameters=[{
            "max_linear_speed": LaunchConfiguration("max_linear_speed"),
            "max_angular_speed": LaunchConfiguration("max_angular_speed"),
            "default_speed_ratio": LaunchConfiguration("default_speed_ratio"),
            "default_wheel_rpm": LaunchConfiguration("default_wheel_rpm"),
        }],
    )

    return LaunchDescription([
        max_linear_arg,
        max_angular_arg,
        speed_ratio_arg,
        wheel_rpm_arg,
        gui,
    ])
