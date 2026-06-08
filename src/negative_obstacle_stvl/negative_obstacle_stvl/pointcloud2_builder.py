"""
sensor_msgs/PointCloud2 mesaj yapıcı.

ROS 2 mesaj altyapısına bağımlı olmadan PointCloud2 ikili verisini üretir.
ROS node içinde kullanılırken rclpy/sensor_msgs ile entegre edilir.
"""

from __future__ import annotations

import struct
import time
from dataclasses import dataclass, field

import numpy as np


# PointField veri tipleri (sensor_msgs/PointField.msg)
FLOAT32 = 7
UINT32 = 6


@dataclass
class PointField:
    name: str
    offset: int
    datatype: int
    count: int = 1


# XYZ nokta bulutu için standart alan tanımları
XYZ_FIELDS: list[PointField] = [
    PointField("x", 0,  FLOAT32, 1),
    PointField("y", 4,  FLOAT32, 1),
    PointField("z", 8,  FLOAT32, 1),
]

POINT_STEP = 12  # 3 * 4 byte (float32)


def points_to_binary(points: np.ndarray) -> bytes:
    """
    (N, 3) float32 nokta dizisini PointCloud2 ikili formatına çevir.

    Her nokta: x (float32) + y (float32) + z (float32) = 12 byte.
    """
    if len(points) == 0:
        return b""
    pts = np.asarray(points, dtype=np.float32)
    return pts.tobytes()


def build_pointcloud2_dict(
    points: np.ndarray,
    frame_id: str = "camera_depth_optical_frame",
    stamp_sec: int | None = None,
    stamp_nanosec: int | None = None,
) -> dict:
    """
    (N, 3) float32 nokta dizisinden PointCloud2 mesaj sözlüğü üret.

    ROS 2 node içinde bu sözlük sensor_msgs.msg.PointCloud2'ye dönüştürülür.
    Sözlük formatı ROS mesaj alanlarıyla birebir uyumludur.

    Args:
        points: (N, 3) float32 XYZ noktaları.
        frame_id: Yayınlanacak TF frame.
        stamp_sec: Zaman damgası saniye kısmı (None → sistem saati).
        stamp_nanosec: Zaman damgası nanosaniye kısmı.

    Döndürür:
        ROS 2 PointCloud2 alanlarına karşılık gelen dict.
    """
    if stamp_sec is None:
        t = time.time()
        stamp_sec = int(t)
        stamp_nanosec = int((t - stamp_sec) * 1e9)

    n_points = len(points)
    data = points_to_binary(points)

    return {
        "header": {
            "stamp": {"sec": stamp_sec, "nanosec": stamp_nanosec or 0},
            "frame_id": frame_id,
        },
        "height": 1,
        "width": n_points,
        "fields": [
            {"name": f.name, "offset": f.offset, "datatype": f.datatype, "count": f.count}
            for f in XYZ_FIELDS
        ],
        "is_bigendian": False,
        "point_step": POINT_STEP,
        "row_step": POINT_STEP * n_points,
        "data": data,
        "is_dense": True,
    }


def binary_to_points(data: bytes, n_points: int) -> np.ndarray:
    """
    PointCloud2 ikili verisini (N, 3) float32 dizisine geri dönüştür.

    Test ve doğrulama için kullanılır.
    """
    if n_points == 0:
        return np.empty((0, 3), dtype=np.float32)
    arr = np.frombuffer(data, dtype=np.float32)
    return arr.reshape(n_points, 3)
