"""
Tam Nav2 stack + STVL local costmap launch.

Modlar:
  map_mode=true  → AMCL + map_server + global static layer (harita gerekli)
  map_mode=false → odometry-only, rolling global costmap (haritasız)
  gps_mode=true  → GPS-EKF lokalizasyon, map çerçevesi, rolling global costmap

Kullanım:
  # Simülasyon, haritasız:
  ros2 launch negative_obstacle_bringup nav2_stvl.launch.py use_sim_time:=true

  # Simülasyon, haritalı:
  ros2 launch negative_obstacle_bringup nav2_stvl.launch.py use_sim_time:=true map_mode:=true map:=/path/to/map.yaml

  # Gerçek robot, haritasız:
  ros2 launch negative_obstacle_bringup nav2_stvl.launch.py

  # Gerçek robot, haritalı:
  ros2 launch negative_obstacle_bringup nav2_stvl.launch.py map_mode:=true map:=/path/to/map.yaml

  # Gerçek robot, GPS modu (dış ortam, haritasız GPS navigasyon):
  ros2 launch negative_obstacle_bringup nav2_stvl.launch.py gps_mode:=true
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    bringup_dir   = Path(get_package_share_directory("negative_obstacle_bringup"))
    use_sim_time  = LaunchConfiguration("use_sim_time")
    map_mode      = LaunchConfiguration("map_mode")
    gps_mode      = LaunchConfiguration("gps_mode")
    map_yaml      = LaunchConfiguration("map")

    params_no_map = str(bringup_dir / "config" / "nav2_params_no_map.yaml")
    params_map    = str(bringup_dir / "config" / "nav2_params.yaml")
    params_gps    = str(bringup_dir / "config" / "nav2_params_gps.yaml")

    sim_param  = {"use_sim_time": use_sim_time}

    # ------------------------------------------------------------------
    # Nav2 core — mod önceliği: gps_mode > map_mode > no_map
    # ------------------------------------------------------------------
    is_gps = context.perform_substitution(gps_mode) == "true"
    is_map = context.perform_substitution(map_mode) == "true"
    if is_gps:
        active_params = params_gps
    elif is_map:
        active_params = params_map
    else:
        active_params = params_no_map

    nav_nodes = [
        Node(
            package="nav2_planner",
            executable="planner_server",
            name="planner_server",
            output="screen",
            parameters=[active_params, sim_param],
            remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
        ),
        Node(
            package="nav2_controller",
            executable="controller_server",
            name="controller_server",
            output="screen",
            parameters=[active_params, sim_param],
            remappings=[
                ("/tf", "tf"),
                ("/tf_static", "tf_static"),
                ("cmd_vel", "cmd_vel_nav"),
            ],
        ),
        Node(
            package="nav2_smoother",
            executable="smoother_server",
            name="smoother_server",
            output="screen",
            parameters=[active_params, sim_param],
            remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
        ),
        Node(
            package="nav2_behaviors",
            executable="behavior_server",
            name="behavior_server",
            output="screen",
            parameters=[active_params, sim_param],
            remappings=[
                ("/tf", "tf"),
                ("/tf_static", "tf_static"),
                ("cmd_vel", "cmd_vel_nav"),
            ],
        ),
        Node(
            package="nav2_bt_navigator",
            executable="bt_navigator",
            name="bt_navigator",
            output="screen",
            parameters=[active_params, sim_param],
            remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
        ),
        Node(
            package="nav2_waypoint_follower",
            executable="waypoint_follower",
            name="waypoint_follower",
            output="screen",
            parameters=[active_params, sim_param],
            remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
        ),
        Node(
            package="nav2_velocity_smoother",
            executable="velocity_smoother",
            name="velocity_smoother",
            output="screen",
            parameters=[active_params, sim_param],
            remappings=[
                ("/tf", "tf"),
                ("/tf_static", "tf_static"),
                ("cmd_vel", "cmd_vel_nav"),
                ("smoothed_cmd_vel", "cmd_vel"),
            ],
        ),
    ]

    # ------------------------------------------------------------------
    # Lifecycle manager — haritasız mod
    # NOT: local_costmap ve global_costmap ayrı node DEĞİLDİR.
    # Bunlar controller_server (local) ve planner_server (global) içinde
    # yaşar; params yaml'dan okunur.
    # ------------------------------------------------------------------
    lifecycle_no_map = [
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_navigation",
            output="screen",
            parameters=[{
                "autostart": True,
                "bond_timeout": 4.0,
                "node_names": [
                    "controller_server",
                    "smoother_server",
                    "planner_server",
                    "behavior_server",
                    "bt_navigator",
                    "waypoint_follower",
                    "velocity_smoother",
                ],
            }],
            condition=UnlessCondition(map_mode),
        ),
    ]

    # ------------------------------------------------------------------
    # AMCL + map_server + lifecycle managers — haritalı mod
    # ------------------------------------------------------------------
    lifecycle_map = [
        Node(
            package="nav2_map_server",
            executable="map_server",
            name="map_server",
            output="screen",
            parameters=[params_map, sim_param, {"yaml_filename": map_yaml}],
            condition=IfCondition(map_mode),
        ),
        Node(
            package="nav2_amcl",
            executable="amcl",
            name="amcl",
            output="screen",
            parameters=[params_map, sim_param],
            condition=IfCondition(map_mode),
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_localization",
            output="screen",
            parameters=[{
                "autostart": True,
                "bond_timeout": 4.0,
                "node_names": ["map_server", "amcl"],
            }],
            condition=IfCondition(map_mode),
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_navigation",
            output="screen",
            parameters=[{
                "autostart": True,
                "bond_timeout": 4.0,
                "node_names": [
                    "controller_server",
                    "smoother_server",
                    "planner_server",
                    "behavior_server",
                    "bt_navigator",
                    "waypoint_follower",
                    "velocity_smoother",
                ],
            }],
            condition=IfCondition(map_mode),
        ),
    ]

    return nav_nodes + lifecycle_no_map + lifecycle_map


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("map_mode",     default_value="false",
                              description="true → AMCL + map_server; false → odometry-only"),
        DeclareLaunchArgument("gps_mode",     default_value="false",
                              description="true → GPS-EKF lokalizasyon, map çerçevesi, haritasız"),
        DeclareLaunchArgument("map",          default_value="",
                              description="Harita YAML yolu (map_mode:=true için)"),
        OpaqueFunction(function=launch_setup),
    ])
