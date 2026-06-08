"""
Gazebo derinlik veri toplayıcı — ROS 2 node.

/camera/depth/image_raw topic'ini dinler ve her N karede bir
derinlik görüntüsünü .npy olarak kaydeder.

Parametreler (ROS 2):
  output_dir    : str  — kaydedilecek klasör (varsayılan: /tmp/gazebo_depth)
  scene_name    : str  — çıktı dosya adı öneki (varsayılan: pit_scene)
  max_frames    : int  — bu kadar kare sonra dur, -1=sonsuz (varsayılan: 100)
  frame_skip    : int  — her N karede bir kaydet (varsayılan: 5)
  depth_topic   : str  — depth image topic (varsayılan: /camera/depth/image_raw)

Kullanım:
  ros2 run negative_obstacle_data gazebo_data_collector \\
      --ros-args \\
      -p output_dir:=/tmp/gazebo_depth \\
      -p scene_name:=pit_scene \\
      -p max_frames:=60 \\
      -p frame_skip:=3
"""

from __future__ import annotations

import json
import struct
import sys
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image


class GazeboDataCollector(Node):
    """Gazebo depth topic'inden ham derinlik kareleri kaydeder."""

    def __init__(self) -> None:
        super().__init__("gazebo_data_collector")

        self.declare_parameter("output_dir",  "/tmp/gazebo_depth")
        self.declare_parameter("scene_name",  "pit_scene")
        self.declare_parameter("max_frames",  100)
        self.declare_parameter("frame_skip",  5)
        self.declare_parameter("depth_topic", "/camera/depth")
        self.declare_parameter("info_topic",  "/camera/camera_info")

        self._output_dir  = Path(self.get_parameter("output_dir").value)
        self._scene_name  = self.get_parameter("scene_name").value
        self._max_frames  = self.get_parameter("max_frames").value
        self._frame_skip  = self.get_parameter("frame_skip").value
        depth_topic       = self.get_parameter("depth_topic").value
        info_topic        = self.get_parameter("info_topic").value

        self._output_dir.mkdir(parents=True, exist_ok=True)

        self._frame_count   = 0   # toplam gelen kare
        self._saved_count   = 0   # kaydedilen kare
        self._camera_info: dict | None = None

        self.create_subscription(CameraInfo, info_topic,  self._info_cb,  10)
        self.create_subscription(Image,      depth_topic, self._depth_cb, 10)

        self.get_logger().info(
            f"GazeboDataCollector başladı — output={self._output_dir} "
            f"scene={self._scene_name} max={self._max_frames} skip={self._frame_skip}"
        )

    # ------------------------------------------------------------------

    def _info_cb(self, msg: CameraInfo) -> None:
        if self._camera_info is not None:
            return
        self._camera_info = {
            "width":  msg.width,
            "height": msg.height,
            "fx": msg.k[0],
            "fy": msg.k[4],
            "cx": msg.k[2],
            "cy": msg.k[5],
        }
        info_path = self._output_dir / f"{self._scene_name}_camera_info.json"
        with open(info_path, "w") as f:
            json.dump(self._camera_info, f, indent=2)
        self.get_logger().info(f"CameraInfo kaydedildi: {info_path}")

    def _depth_cb(self, msg: Image) -> None:
        if self._max_frames > 0 and self._saved_count >= self._max_frames:
            self.get_logger().info(
                f"Hedef ulaşıldı: {self._saved_count} kare kaydedildi. Node durduruluyor."
            )
            rclpy.shutdown()
            return

        self._frame_count += 1
        if self._frame_count % self._frame_skip != 0:
            return

        depth = self._decode_depth(msg)
        if depth is None:
            self.get_logger().warn(f"Bilinmeyen encoding: {msg.encoding}")
            return

        stem = f"{self._scene_name}_{self._saved_count:04d}"
        np.save(str(self._output_dir / f"{stem}_depth.npy"), depth)
        self._saved_count += 1

        if self._saved_count % 10 == 0:
            self.get_logger().info(
                f"Kaydedilen: {self._saved_count} / "
                f"{self._max_frames if self._max_frames > 0 else '∞'}"
            )

    def _decode_depth(self, msg: Image) -> np.ndarray | None:
        """Image mesajını float32 metre derinlik dizisine çevirir."""
        h, w = msg.height, msg.width

        if msg.encoding in ("32FC1", "32FC"):
            arr = np.frombuffer(msg.data, dtype=np.float32).reshape(h, w).copy()
            return arr

        if msg.encoding in ("16UC1", "16UC", "mono16"):
            arr = np.frombuffer(msg.data, dtype=np.uint16).reshape(h, w)
            return arr.astype(np.float32) / 1000.0  # mm → m

        return None


# ---------------------------------------------------------------------------
# main
# ---------------------------------------------------------------------------

def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = GazeboDataCollector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
