"""Launch: real robot serial driver + semantic drop publisher."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    port_arg    = DeclareLaunchArgument("port",    default_value="/dev/ttyUSB0")
    baud_arg    = DeclareLaunchArgument("baud",    default_value="115200")
    cam_arg     = DeclareLaunchArgument("camera",  default_value="/camera/depth/image_raw")

    serial_driver = Node(
        package="negative_obstacle_data",
        executable="arduino_serial_driver",
        name="arduino_serial_driver",
        output="screen",
        parameters=[{
            "port":               LaunchConfiguration("port"),
            "baud":               LaunchConfiguration("baud"),
            "base_frame":         "base_link",
            "odom_frame":         "odom",
            "publish_tf":         True,
            "max_linear_speed":   0.5,
            "max_angular_speed":  1.5,
        }],
    )

    semantic_drop = Node(
        package="negative_obstacle_stvl",
        executable="semantic_drop_publisher",
        name="semantic_drop_publisher",
        output="screen",
        parameters=[{
            "depth_topic": LaunchConfiguration("camera"),
        }],
    )

    return LaunchDescription([
        port_arg, baud_arg, cam_arg,
        serial_driver,
        semantic_drop,
    ])
