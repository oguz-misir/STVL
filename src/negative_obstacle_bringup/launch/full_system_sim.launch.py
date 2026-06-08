"""
Full-system SIMÜLASYON launch — Gazebo + Nav2 + STVL + RViz

Başlatılanlar:
  1. robot_state_publisher  (URDF → TF)
  2. Gazebo Harmonic        (pit_scene veya platform_edge)
  3. ros_gz_bridge          (depth image + camera_info + clock)
  4. semantic_drop_publisher (/camera/depth/image_raw → /semantic_drop_points)
  5. nav2_stvl              (tam Nav2 stack: planner/controller/BT/costmap/STVL)
  6. rviz2                  (negative_obstacle.rviz)

Kullanım:
  # Temel (haritasız):
  ros2 launch negative_obstacle_bringup full_system_sim.launch.py

  # Farklı world:
  ros2 launch negative_obstacle_bringup full_system_sim.launch.py world:=platform_edge

  # RViz kapalı:
  ros2 launch negative_obstacle_bringup full_system_sim.launch.py rviz:=false

  # Haritalı mod (harita önceden kaydedilmişse):
  ros2 launch negative_obstacle_bringup full_system_sim.launch.py map_mode:=true map:=/path/map.yaml

ÖNEMLİ (WSL2):
  Gazebo'yu önce ayrı terminalde başlatın:
    DISPLAY=:0 gz sim pit_scene.sdf -r -s
  Sonra bu launch dosyasını çalıştırın (world:=none yaparak Gazebo başlatmayı atlayın).
  Veya direkt burada başlatmak için DISPLAY=:0 olduğundan emin olun.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    bringup_dir = Path(get_package_share_directory("negative_obstacle_bringup"))

    # ------------------------------------------------------------------ args
    world_arg    = DeclareLaunchArgument("world",      default_value="pit_scene")
    rviz_arg     = DeclareLaunchArgument("rviz",       default_value="true")
    map_mode_arg = DeclareLaunchArgument("map_mode",   default_value="false")
    map_arg      = DeclareLaunchArgument("map",        default_value="")
    gz_arg       = DeclareLaunchArgument(
        "start_gazebo", default_value="true",
        description="false → Gazebo dışarıda başlatılıyor (WSL2 önerilen)")

    world_sdf = PathJoinSubstitution([
        FindPackageShare("negative_obstacle_bringup"), "worlds",
        [LaunchConfiguration("world"), ".sdf"],
    ])

    # ------------------------------------------------------------------ robot description
    robot_desc = Command([
        "xacro ", str(bringup_dir / "urdf" / "robot.urdf.xacro"),
    ])

    robot_state_pub = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[{"robot_description": robot_desc, "use_sim_time": True}],
        output="screen",
    )

    # ------------------------------------------------------------------ Gazebo
    gazebo = ExecuteProcess(
        cmd=["gz", "sim", world_sdf, "-r", "-s"],
        output="screen",
        condition=IfCondition(LaunchConfiguration("start_gazebo")),
    )

    # ------------------------------------------------------------------ ros_gz_bridge
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="ros_gz_bridge",
        arguments=[
            "/camera/depth@sensor_msgs/msg/Image[gz.msgs.Image",
            "/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
            "/camera/depth/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
        ],
        remappings=[
            ("/camera/depth", "/camera/depth/image_raw"),
        ],
        parameters=[{"use_sim_time": True}],
        output="screen",
    )

    # ------------------------------------------------------------------ perception
    semantic_drop = Node(
        package="negative_obstacle_stvl",
        executable="semantic_drop_publisher",
        name="semantic_drop_publisher",
        parameters=[
            str(bringup_dir / "config" / "semantic_drop_publisher_params.yaml"),
            {"use_sim_time": True},
        ],
        output="screen",
    )

    # ------------------------------------------------------------------ Nav2 full stack
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(bringup_dir / "launch" / "nav2_stvl.launch.py")
        ),
        launch_arguments={
            "use_sim_time": "true",
            "map_mode":     LaunchConfiguration("map_mode"),
            "map":          LaunchConfiguration("map"),
        }.items(),
    )

    # ------------------------------------------------------------------ RViz
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", str(bringup_dir / "rviz" / "negative_obstacle.rviz")],
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(LaunchConfiguration("rviz")),
        output="screen",
    )

    # Zaman gecikmeli başlatma: Gazebo → bridge → perception → Nav2 → RViz
    return LaunchDescription([
        world_arg, rviz_arg, map_mode_arg, map_arg, gz_arg,
        robot_state_pub,
        gazebo,
        TimerAction(period=3.0, actions=[bridge]),
        TimerAction(period=4.5, actions=[semantic_drop]),
        TimerAction(period=5.0, actions=[nav2_launch]),
        TimerAction(period=6.0, actions=[rviz_node]),
    ])
