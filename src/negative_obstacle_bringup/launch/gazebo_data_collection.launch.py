"""
Gazebo veri toplama launch dosyası.

Başlatılanlar:
  1. ros_gz_bridge — depth image ve camera_info köprüsü
  2. gazebo_data_collector — derinlik karelerini diske kaydeder

Kullanım:
  # Önce Gazebo'yu WSLg display ile server modunda başlat:
  DISPLAY=:0 gz sim /path/to/pit_scene.sdf -r -s

  # Sonra bu launch dosyasını çalıştır:
  ros2 launch negative_obstacle_bringup gazebo_data_collection.launch.py \\
      output_dir:=/tmp/gazebo_depth \\
      scene_name:=pit_scene \\
      max_frames:=60

  # Gerçek Gazebo topic isimleri (SDF topic="camera/depth"):
  #   /camera/depth       → sensor_msgs/msg/Image (depth, float32)
  #   /camera/camera_info → sensor_msgs/msg/CameraInfo

Parametreler:
  output_dir  : Kaydedilecek klasör
  scene_name  : Sahne adı (dosya öneki)
  max_frames  : Kaydedilecek maksimum kare sayısı
  frame_skip  : Her N karede bir kaydet
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    output_dir  = LaunchConfiguration("output_dir")
    scene_name  = LaunchConfiguration("scene_name")
    max_frames  = LaunchConfiguration("max_frames")
    frame_skip  = LaunchConfiguration("frame_skip")

    return LaunchDescription([
        DeclareLaunchArgument("output_dir",  default_value="/tmp/gazebo_depth"),
        DeclareLaunchArgument("scene_name",  default_value="pit_scene"),
        DeclareLaunchArgument("max_frames",  default_value="60"),
        DeclareLaunchArgument("frame_skip",  default_value="3"),

        LogInfo(msg=["Gazebo veri toplama başlatılıyor: sahne=", scene_name]),

        # ros_gz_bridge: depth image (Gazebo → ROS 2)
        # Gazebo gerçek topic'ler: /camera/depth (image), /camera/camera_info
        # Yön: [  = GZ→ROS tek yönlü
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="gz_depth_bridge",
            arguments=[
                "/camera/depth@sensor_msgs/msg/Image[gz.msgs.Image",
                "/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
            ],
            output="screen",
        ),

        # Veri toplayıcı node
        Node(
            package="negative_obstacle_data",
            executable="gazebo_data_collector",
            name="gazebo_data_collector",
            parameters=[{
                "output_dir":  output_dir,
                "scene_name":  scene_name,
                "max_frames":  max_frames,
                "frame_skip":  frame_skip,
                "depth_topic": "/camera/depth",
                "info_topic":  "/camera/camera_info",
            }],
            output="screen",
        ),
    ])
