"""
SemanticDropPublisher — ROS 2 node.

Kameradan gelen depth görüntüsünü işler:
  1. GeometricRiskAnalyzer ile belirsizlik-duyarlı risk alanı üretir.
  2. Edge piksellerini 3B'ye projekte eder.
  3. TemporalPointAccumulator ile noktaları biriktirir.
  4. /semantic_drop_points topic'ine PointCloud2 yayınlar.
  5. /risk/drop_probability ve /risk/cost_image topic'lerini yayınlar.

STVL bu topic'i ikinci observation source olarak kullanır.
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import rclpy
from rclpy._rclpy_pybind11 import RCLError
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Image, PointCloud2, PointField, CameraInfo
from std_msgs.msg import Header

# Paket yolları (colcon install edilmişse gerek yoktur)
_repo_root = Path(__file__).parent.parent.parent.parent.parent
sys.path.insert(0, str(_repo_root / "src" / "negative_obstacle_common"))
sys.path.insert(0, str(_repo_root / "src" / "negative_obstacle_perception"))

from negative_obstacle_common.camera_utils import CameraIntrinsics
from negative_obstacle_perception.geometric_risk_analyzer import GeometricRiskAnalyzer
from .edge_to_pointcloud import edge_mask_to_3d_points, transform_points_camera_to_base
from .temporal_persistence import TemporalPointAccumulator
from .pointcloud2_builder import XYZ_FIELDS, POINT_STEP, points_to_binary


class SemanticDropPublisher(Node):
    """
    Drop-edge noktalarını /semantic_drop_points olarak yayınlayan ROS 2 node.
    """

    def __init__(self) -> None:
        super().__init__("semantic_drop_publisher")

        # Parametreler
        self.declare_parameter("image_width",          320)
        self.declare_parameter("image_height",         240)
        self.declare_parameter("fov_h_deg",            70.0)
        self.declare_parameter("camera_height_m",       0.45)
        self.declare_parameter("camera_pitch_deg",     -20.0)
        self.declare_parameter("jump_threshold_m",       0.12)
        self.declare_parameter("drop_threshold_m",       0.08)
        self.declare_parameter("depth_dilation_m",       0.05)
        self.declare_parameter("temporal_window_sec",    1.0)
        self.declare_parameter("max_points",           3000)
        self.declare_parameter("voxel_size",             0.05)
        self.declare_parameter("use_ground_estimator",  True)
        self.declare_parameter("publish_frame_id",     "base_link")
        self.declare_parameter("use_camera_info",      False)
        self.declare_parameter("depth_topic",          "/camera/depth/image_raw")
        self.declare_parameter("camera_info_topic",    "/camera/depth/camera_info")
        self.declare_parameter("drop_points_topic",    "/semantic_drop_points")
        self.declare_parameter("drop_probability_topic", "/risk/drop_probability")
        self.declare_parameter("risk_cost_topic",      "/risk/cost_image")

        w   = self.get_parameter("image_width").value
        h   = self.get_parameter("image_height").value
        fov = self.get_parameter("fov_h_deg").value

        self._camera_height_m  = self.get_parameter("camera_height_m").value
        self._camera_pitch_deg = self.get_parameter("camera_pitch_deg").value
        self._depth_dilation_m = self.get_parameter("depth_dilation_m").value
        self._publish_frame    = self.get_parameter("publish_frame_id").value
        self._use_camera_info  = self.get_parameter("use_camera_info").value
        self._depth_topic      = self.get_parameter("depth_topic").value
        self._camera_info_topic = self.get_parameter("camera_info_topic").value
        self._drop_points_topic = self.get_parameter("drop_points_topic").value
        self._drop_probability_topic = self.get_parameter("drop_probability_topic").value
        self._risk_cost_topic = self.get_parameter("risk_cost_topic").value

        self._intrinsics = CameraIntrinsics.from_fov(w, h, fov_h_deg=fov)

        self._analyzer = GeometricRiskAnalyzer(
            intrinsics=self._intrinsics,
            jump_threshold_m=self.get_parameter("jump_threshold_m").value,
            drop_threshold_m=self.get_parameter("drop_threshold_m").value,
            use_ground_estimator=self.get_parameter("use_ground_estimator").value,
        )

        self._accumulator = TemporalPointAccumulator(
            window_sec=self.get_parameter("temporal_window_sec").value,
            max_points=self.get_parameter("max_points").value,
            voxel_size=self.get_parameter("voxel_size").value,
        )

        self._frame_count = 0

        # QoS — STVL için reliable tercih edilir
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=5,
        )

        # Subscriber'lar
        self._depth_sub = self.create_subscription(
            Image,
            self._depth_topic,
            self._depth_callback,
            sensor_qos,
        )

        if self._use_camera_info:
            self._info_sub = self.create_subscription(
                CameraInfo,
                self._camera_info_topic,
                self._camera_info_callback,
                sensor_qos,
            )

        # Publisher
        self._drop_pub = self.create_publisher(
            PointCloud2,
            self._drop_points_topic,
            sensor_qos,
        )
        self._drop_prob_pub = self.create_publisher(
            Image,
            self._drop_probability_topic,
            sensor_qos,
        )
        self._risk_cost_pub = self.create_publisher(
            Image,
            self._risk_cost_topic,
            sensor_qos,
        )

        self.get_logger().info(
            f"SemanticDropPublisher baslatildi. "
            f"Kamera: {w}x{h}, FOV={fov}°, "
            f"depth_topic={self._depth_topic}, frame_id={self._publish_frame}"
        )

    def _camera_info_callback(self, msg: CameraInfo) -> None:
        """CameraInfo mesajından intrinsics güncelle."""
        self._intrinsics = CameraIntrinsics(
            fx=msg.k[0], fy=msg.k[4],
            cx=msg.k[2], cy=msg.k[5],
            width=msg.width, height=msg.height,
        )
        self._analyzer = GeometricRiskAnalyzer(
            intrinsics=self._intrinsics,
            jump_threshold_m=self.get_parameter("jump_threshold_m").value,
            drop_threshold_m=self.get_parameter("drop_threshold_m").value,
            use_ground_estimator=self.get_parameter("use_ground_estimator").value,
        )

    def _depth_callback(self, msg: Image) -> None:
        """Depth Image mesajı alındığında işle ve yayınla."""
        depth = self._image_to_depth(msg)
        if depth is None:
            return

        # Risk alanı ve edge mask üret
        if self.get_parameter("use_ground_estimator").value:
            result = self._analyzer.analyze(depth, rng_seed=self._frame_count)
        else:
            result = self._analyzer.analyze_edge_only(depth)
        edge_mask = result["edge_mask"]

        # Edge piksellerini 3B'ye projekte et
        pts_cam = edge_mask_to_3d_points(
            edge_mask=edge_mask,
            depth=depth,
            intrinsics=self._intrinsics,
            depth_dilation_m=self._depth_dilation_m,
        )

        # Base_link çerçevesine dönüştür
        pts_base = transform_points_camera_to_base(
            pts_cam,
            camera_height_m=self._camera_height_m,
            camera_pitch_deg=self._camera_pitch_deg,
        )

        # Biriktir
        self._accumulator.add(pts_base)
        accumulated = self._accumulator.get_accumulated()

        # PointCloud2 yayınla
        cloud_msg = self._build_cloud_msg(accumulated, msg.header.stamp)
        self._drop_pub.publish(cloud_msg)
        self._drop_prob_pub.publish(
            self._build_image_msg(
                result["drop_probability"].astype(np.float32),
                msg.header.stamp,
                encoding="32FC1",
            )
        )
        self._risk_cost_pub.publish(
            self._build_image_msg(
                result["risk_cost"].astype(np.uint8),
                msg.header.stamp,
                encoding="mono8",
            )
        )

        self._frame_count += 1
        if self._frame_count % 30 == 0:
            self.get_logger().info(
                f"Frame {self._frame_count}: edge={len(pts_cam)} nokta, "
                f"biriktirilmis={len(accumulated)} nokta, "
                f"max_risk={float(np.max(result['risk_field']['risk_score'])):.2f}"
            )

    def _image_to_depth(self, msg: Image) -> np.ndarray | None:
        """ROS Image mesajını numpy float32 depth dizisine çevir."""
        encoding = msg.encoding
        data = np.frombuffer(msg.data, dtype=np.uint8)

        try:
            if encoding == "32FC1":
                depth = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width)
            elif encoding in ("16UC1", "mono16"):
                depth_u16 = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width)
                depth = depth_u16.astype(np.float32) / 1000.0  # mm → m
            else:
                self.get_logger().warn(f"Desteklenmeyen depth encoding: {encoding}")
                return None
        except Exception as e:
            self.get_logger().error(f"Depth donusturme hatasi: {e}")
            return None

        # 0 ve inf değerlerini NaN'a çevir
        depth[~np.isfinite(depth)] = np.nan
        depth[depth <= 0] = np.nan
        return depth

    def _build_cloud_msg(self, points: np.ndarray, stamp) -> PointCloud2:
        """(N, 3) float32 dizisinden sensor_msgs/PointCloud2 mesajı üret."""
        header = Header()
        header.stamp = stamp
        header.frame_id = self._publish_frame

        fields = [
            PointField(name="x", offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8,  datatype=PointField.FLOAT32, count=1),
        ]

        n = len(points)
        msg = PointCloud2()
        msg.header = header
        msg.height = 1
        msg.width = n
        msg.fields = fields
        msg.is_bigendian = False
        msg.point_step = POINT_STEP
        msg.row_step = POINT_STEP * n
        msg.data = list(points_to_binary(points))
        msg.is_dense = True
        return msg

    def _build_image_msg(self, array: np.ndarray, stamp, encoding: str) -> Image:
        """Numpy dizisinden sensor_msgs/Image mesajı üret."""
        header = Header()
        header.stamp = stamp
        header.frame_id = self._publish_frame

        contiguous = np.ascontiguousarray(array)
        msg = Image()
        msg.header = header
        msg.height = int(contiguous.shape[0])
        msg.width = int(contiguous.shape[1])
        msg.encoding = encoding
        msg.is_bigendian = False
        msg.step = int(contiguous.strides[0])
        msg.data = contiguous.tobytes()
        return msg


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SemanticDropPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except RCLError:
            pass


if __name__ == "__main__":
    main()
