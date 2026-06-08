"""
GPS + IMU + EKF Lokalizasyon Launch (Alt-launch).

Başlatılanlar:
  1. nmea_serial_driver       — GPS → /fix
  2. witmotion_imu_driver     — Witmotion HWT905 → /imu/data
  3. ekf_filter_node_odom     — wheel_odom + IMU → /odometry/filtered + odom→base_link TF
  4. navsat_transform_node    — /fix → /odometry/gps (ENU)
  5. ekf_filter_node_map      — /odometry/gps + wheel_odom + IMU → map→odom TF

TF ağacı:
  map (GPS-ENU) → odom (EKF local) → base_link → camera_link
                                               → imu_link
                                               → gps_link

Kullanım (doğrudan):
  ros2 launch negative_obstacle_bringup gps_localization.launch.py

Kullanım (full_system ile):
  ros2 launch negative_obstacle_bringup full_system_real_robot.launch.py gps_mode:=true
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    bringup_dir = Path(get_package_share_directory("negative_obstacle_bringup"))

    # ── Argümanlar ────────────────────────────────────────────────────────
    gps_port_arg = DeclareLaunchArgument(
        "gps_port", default_value="/dev/ttyUSB1",
        description="GPS serial port"
    )
    gps_baud_arg = DeclareLaunchArgument(
        "gps_baud", default_value="9600",
        description="GPS baud rate (çoğu NMEA GPS: 9600, u-blox: 115200)"
    )
    imu_port_arg = DeclareLaunchArgument(
        "imu_port", default_value="/dev/ttyUSB2",
        description="Witmotion IMU serial port"
    )
    imu_baud_arg = DeclareLaunchArgument(
        "imu_baud", default_value="9600",
        description="Witmotion IMU baud rate"
    )

    # ── GPS Sürücüsü (NMEA) ───────────────────────────────────────────────
    gps_driver = Node(
        package="nmea_navsat_driver",
        executable="nmea_serial_driver",
        name="nmea_gps_driver",
        output="screen",
        parameters=[{
            "port":      LaunchConfiguration("gps_port"),
            "baud":      LaunchConfiguration("gps_baud"),
            "frame_id":  "gps_link",
            "time_ref_source": "gps",
            "useRMC":    False,        # GGA+GSA+GSV kullan (daha fazla bilgi)
        }],
    )

    # ── IMU Sürücüsü (Witmotion HWT905) ──────────────────────────────────
    imu_driver = Node(
        package="witmotion_hwt905_driver",
        executable="witmotion_hwt905_driver",
        name="witmotion_imu",
        output="screen",
        parameters=[{
            "port":       LaunchConfiguration("imu_port"),
            "baud_rate":  LaunchConfiguration("imu_baud"),
            "frame_id":   "imu_link",
        }],
        remappings=[
            ("imu", "/imu/data"),
        ],
    )

    # ── EKF Local (odom → base_link TF) ──────────────────────────────────
    ekf_local = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node_odom",
        output="screen",
        parameters=[str(bringup_dir / "config" / "ekf_local_params.yaml")],
        remappings=[
            ("odometry/filtered", "/odometry/filtered"),
        ],
    )

    # ── navsat_transform_node (GPS fix → ENU odometry) ───────────────────
    navsat = Node(
        package="robot_localization",
        executable="navsat_transform_node",
        name="navsat_transform_node",
        output="screen",
        parameters=[str(bringup_dir / "config" / "navsat_params.yaml")],
        remappings=[
            ("imu/data",         "/imu/data"),
            ("gps/fix",          "/fix"),
            ("odometry/filtered", "/odometry/filtered"),
            ("odometry/gps",     "/odometry/gps"),
        ],
    )

    # ── EKF Global (map → odom TF, GPS-düzeltmeli) ───────────────────────
    ekf_global = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node_map",
        output="screen",
        parameters=[str(bringup_dir / "config" / "ekf_global_params.yaml")],
        remappings=[
            ("odometry/filtered", "/odometry/map_filtered"),
        ],
    )

    return LaunchDescription([
        gps_port_arg, gps_baud_arg, imu_port_arg, imu_baud_arg,
        gps_driver,
        imu_driver,
        ekf_local,
        navsat,
        ekf_global,
    ])
