"""
SemanticDropPublisher launch dosyası.

Kullanım:
    ros2 launch negative_obstacle_bringup semantic_drop_publisher.launch.py
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    bringup_dir = Path(get_package_share_directory("negative_obstacle_bringup"))
    params_file = str(bringup_dir / "config" / "semantic_drop_publisher_params.yaml")

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Gazebo simülasyon saati kullan",
    )
    depth_topic_arg = DeclareLaunchArgument(
        "depth_topic",
        default_value="/camera/depth/image_raw",
        description="Depth image topic",
    )
    camera_info_topic_arg = DeclareLaunchArgument(
        "camera_info_topic",
        default_value="/camera/depth/camera_info",
        description="CameraInfo topic",
    )

    semantic_drop_node = Node(
        package="negative_obstacle_stvl",
        executable="semantic_drop_publisher",
        name="semantic_drop_publisher",
        parameters=[
            params_file,
            {
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "depth_topic": LaunchConfiguration("depth_topic"),
                "camera_info_topic": LaunchConfiguration("camera_info_topic"),
            },
        ],
        output="screen",
    )

    return LaunchDescription([
        use_sim_time_arg,
        depth_topic_arg,
        camera_info_topic_arg,
        semantic_drop_node,
    ])
