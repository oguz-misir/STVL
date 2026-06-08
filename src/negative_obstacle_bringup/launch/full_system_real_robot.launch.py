"""
Full-system GERÇEK ROBOT launch.

Başlatılanlar:
  1. robot_state_publisher  (URDF → TF)
  2. arduino_serial_driver  (/cmd_vel → serial PWM, JS → /wheel_odom)
  3. semantic_drop_publisher (/camera/depth/image_raw → /semantic_drop_points)
  4. gps_localization        (GPS modu ise: NMEA + IMU + dual-EKF + navsat)
  5. nav2_stvl              (tam Nav2 stack: planner/controller/BT/costmap/STVL)
  6. rviz2                  (negative_obstacle.rviz)

Ön koşullar:
  - Arduino CH2 kanalı pasif (aksi halde serial komutlar işlenmez)
  - RGB-D kamera /camera/depth/image_raw ve /camera/depth/points yayınlıyor
  - /dev/ttyUSB0 erişilebilir (yoksa port:=/dev/ttyACM0)
  - GPS modu: /dev/ttyUSB1 (GPS) ve /dev/ttyUSB2 (IMU) bağlı olmalı

Kullanım:
  # Haritasız iç ortam (ilk test önerilen):
  ros2 launch negative_obstacle_bringup full_system_real_robot.launch.py

  # Farklı seri port:
  ros2 launch negative_obstacle_bringup full_system_real_robot.launch.py port:=/dev/ttyACM0

  # Haritalı mod:
  ros2 launch negative_obstacle_bringup full_system_real_robot.launch.py map_mode:=true map:=/path/map.yaml

  # GPS modu (dış ortam, haritasız GPS navigasyon):
  ros2 launch negative_obstacle_bringup full_system_real_robot.launch.py gps_mode:=true

  # GPS modu, özel portlar:
  ros2 launch negative_obstacle_bringup full_system_real_robot.launch.py gps_mode:=true gps_port:=/dev/ttyUSB1 imu_port:=/dev/ttyUSB2

  # RViz kapalı (headless):
  ros2 launch negative_obstacle_bringup full_system_real_robot.launch.py rviz:=false

  # Manuel motor kontrol GUI açık:
  ros2 launch negative_obstacle_bringup full_system_real_robot.launch.py motor_gui:=true

  # Sensör sürücülerini de başlat:
  ros2 launch negative_obstacle_bringup full_system_real_robot.launch.py start_realsense:=true start_lidar:=true
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
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    bringup_dir = Path(get_package_share_directory("negative_obstacle_bringup"))

    # ------------------------------------------------------------------ args
    port_arg     = DeclareLaunchArgument("port",      default_value="/dev/ttyUSB0")
    baud_arg     = DeclareLaunchArgument("baud",      default_value="115200")
    rviz_arg     = DeclareLaunchArgument("rviz",      default_value="true")
    motor_gui_arg = DeclareLaunchArgument("motor_gui", default_value="false")
    start_realsense_arg = DeclareLaunchArgument("start_realsense", default_value="false")
    start_lidar_arg = DeclareLaunchArgument("start_lidar", default_value="false")
    camera_depth_topic_arg = DeclareLaunchArgument("camera_depth_topic", default_value="/camera/depth/image_raw")
    camera_info_topic_arg = DeclareLaunchArgument("camera_info_topic", default_value="/camera/depth/camera_info")
    lidar_package_arg = DeclareLaunchArgument("lidar_package", default_value="rplidar_ros")
    lidar_launch_arg = DeclareLaunchArgument("lidar_launch", default_value="view_rplidar_launch.py")
    map_mode_arg = DeclareLaunchArgument("map_mode",  default_value="false")
    map_arg      = DeclareLaunchArgument("map",       default_value="")
    gps_mode_arg = DeclareLaunchArgument(
        "gps_mode", default_value="false",
        description="true → GPS+IMU+EKF lokalizasyon, dış ortam navigasyonu"
    )
    gps_port_arg = DeclareLaunchArgument(
        "gps_port", default_value="/dev/ttyUSB1",
        description="GPS serial port"
    )
    gps_baud_arg = DeclareLaunchArgument(
        "gps_baud", default_value="9600",
        description="GPS baud rate"
    )
    imu_port_arg = DeclareLaunchArgument(
        "imu_port", default_value="/dev/ttyUSB2",
        description="Witmotion IMU serial port"
    )
    imu_baud_arg = DeclareLaunchArgument(
        "imu_baud", default_value="9600",
        description="Witmotion IMU baud rate"
    )

    # ------------------------------------------------------------------ optional sensor drivers
    realsense_driver = Node(
        package="realsense2_camera",
        executable="realsense2_camera_node",
        name="realsense2_camera",
        output="screen",
        parameters=[{
            "enable_depth": True,
            "enable_color": True,
            "pointcloud.enable": True,
            "align_depth.enable": True,
            "depth_module.profile": "640x480x30",
            "rgb_camera.profile": "640x480x30",
        }],
        remappings=[
            ("/camera/camera/depth/image_rect_raw", "/camera/depth/image_raw"),
            ("/camera/camera/depth/camera_info", "/camera/depth/camera_info"),
            ("/camera/camera/depth/color/points", "/camera/depth/points"),
            ("/camera/camera/color/image_raw", "/camera/color/image_raw"),
            ("/camera/camera/color/camera_info", "/camera/color/camera_info"),
        ],
        condition=IfCondition(LaunchConfiguration("start_realsense")),
    )

    lidar_driver = ExecuteProcess(
        cmd=[
            "ros2", "launch",
            LaunchConfiguration("lidar_package"),
            LaunchConfiguration("lidar_launch"),
        ],
        output="screen",
        condition=IfCondition(LaunchConfiguration("start_lidar")),
    )

    # ------------------------------------------------------------------ robot description
    robot_desc = ParameterValue(Command([
        "xacro ", str(bringup_dir / "urdf" / "robot.urdf.xacro"),
    ]), value_type=str)

    robot_state_pub = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[{"robot_description": robot_desc, "use_sim_time": False}],
        output="screen",
    )

    # ------------------------------------------------------------------ serial driver
    # GPS modunda odom→base_link TF'yi EKF yayınlar; arduino sürücüsü yayınlamamalı.
    # İki ayrı Node tanımı: gps_mode=true ise publish_tf=False, aksi halde True.
    serial_driver_base_params = {
        "port":               LaunchConfiguration("port"),
        "baud":               LaunchConfiguration("baud"),
        "base_frame":         "base_link",
        "odom_frame":         "odom",
        "max_linear_speed":   0.5,
        "max_angular_speed":  1.5,
        "use_sim_time":       False,
    }

    serial_driver_no_gps = Node(
        package="negative_obstacle_data",
        executable="arduino_serial_driver",
        name="arduino_serial_driver",
        output="screen",
        parameters=[{**serial_driver_base_params, "publish_tf": True}],
        condition=UnlessCondition(LaunchConfiguration("gps_mode")),
    )

    serial_driver_gps = Node(
        package="negative_obstacle_data",
        executable="arduino_serial_driver",
        name="arduino_serial_driver",
        output="screen",
        parameters=[{**serial_driver_base_params, "publish_tf": False}],
        condition=IfCondition(LaunchConfiguration("gps_mode")),
    )

    # ------------------------------------------------------------------ GPS lokalizasyon (isteğe bağlı)
    gps_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(bringup_dir / "launch" / "gps_localization.launch.py")
        ),
        launch_arguments={
            "gps_port": LaunchConfiguration("gps_port"),
            "gps_baud": LaunchConfiguration("gps_baud"),
            "imu_port": LaunchConfiguration("imu_port"),
            "imu_baud": LaunchConfiguration("imu_baud"),
        }.items(),
        condition=IfCondition(LaunchConfiguration("gps_mode")),
    )

    # ------------------------------------------------------------------ perception
    semantic_drop = Node(
        package="negative_obstacle_stvl",
        executable="semantic_drop_publisher",
        name="semantic_drop_publisher",
        parameters=[
            str(bringup_dir / "config" / "semantic_drop_publisher_params.yaml"),
            {
                "use_sim_time": False,
                "depth_topic": LaunchConfiguration("camera_depth_topic"),
                "camera_info_topic": LaunchConfiguration("camera_info_topic"),
            },
        ],
        output="screen",
    )

    # ------------------------------------------------------------------ Nav2 full stack
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(bringup_dir / "launch" / "nav2_stvl.launch.py")
        ),
        launch_arguments={
            "use_sim_time": "false",
            "map_mode":     LaunchConfiguration("map_mode"),
            "gps_mode":     LaunchConfiguration("gps_mode"),
            "map":          LaunchConfiguration("map"),
        }.items(),
    )

    # ------------------------------------------------------------------ RViz
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", str(bringup_dir / "rviz" / "negative_obstacle.rviz")],
        parameters=[{"use_sim_time": False}],
        condition=IfCondition(LaunchConfiguration("rviz")),
        output="screen",
    )

    motor_gui_node = Node(
        package="negative_obstacle_data",
        executable="motor_control_gui",
        name="motor_control_gui",
        output="screen",
        parameters=[{
            "max_linear_speed": 0.5,
            "max_angular_speed": 1.5,
            "default_speed_ratio": 0.35,
            "default_wheel_rpm": 80,
        }],
        condition=IfCondition(LaunchConfiguration("motor_gui")),
    )

    return LaunchDescription([
        port_arg, baud_arg, rviz_arg, map_mode_arg, map_arg,
        gps_mode_arg, gps_port_arg, gps_baud_arg, imu_port_arg, imu_baud_arg,
        motor_gui_arg, start_realsense_arg, start_lidar_arg,
        camera_depth_topic_arg, camera_info_topic_arg,
        lidar_package_arg, lidar_launch_arg,
        robot_state_pub,
        realsense_driver,
        lidar_driver,
        serial_driver_no_gps,
        serial_driver_gps,
        semantic_drop,
        gps_localization,
        nav2_launch,
        TimerAction(period=3.0, actions=[rviz_node]),
        motor_gui_node,
    ])
