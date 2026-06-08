"""
Edge mask piksellerini 3B nokta bulutuna dönüştürücü.

ROS bağımlılığı yoktur; saf numpy ile çalışır.
GeometricRiskAnalyzer çıktısını STVL'nin anlayacağı PointCloud2'ye
hazır hâle getirir.
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parent.parent.parent.parent / "negative_obstacle_common"))
from negative_obstacle_common.camera_utils import CameraIntrinsics


def edge_mask_to_3d_points(
    edge_mask: np.ndarray,
    depth: np.ndarray,
    intrinsics: CameraIntrinsics,
    depth_dilation_m: float = 0.05,
    min_depth: float = 0.10,
    max_depth: float = 8.0,
) -> np.ndarray:
    """
    Edge mask piksellerini kamera çerçevesinde 3B noktalara dönüştür.

    Edge pikseli başına, o pikselin depth değeri kullanılarak bir 3B nokta
    üretilir. Geçersiz (NaN/sıfır) depth değeri olan edge pikseller atlanır.
    Gerçek edge'in hemen ötesini işaretlemek için depth'e depth_dilation_m eklenir.

    Args:
        edge_mask: (H, W) uint8 — 255: edge pikseli, 0: değil.
        depth: (H, W) float32, metre.
        intrinsics: Kamera iç parametreleri.
        depth_dilation_m: Edge noktalarını biraz daha ileriye (drop tarafına)
            kaydırmak için eklenen mesafe (metre).
        min_depth / max_depth: Kabul edilebilir derinlik aralığı.

    Döndürür:
        (N, 3) float32 — kamera çerçevesinde XYZ noktaları.
        N = geçerli edge piksel sayısı.
    """
    edge_bin = edge_mask > 127
    rows, cols = np.where(edge_bin)

    if len(rows) == 0:
        return np.empty((0, 3), dtype=np.float32)

    z = depth[rows, cols].astype(np.float32)

    # Geçersiz depth değerlerini filtrele
    valid = np.isfinite(z) & (z >= min_depth) & (z <= max_depth)
    rows, cols, z = rows[valid], cols[valid], z[valid]

    if len(rows) == 0:
        return np.empty((0, 3), dtype=np.float32)

    # Drop tarafına kaydır
    z = z + depth_dilation_m

    x = (cols - intrinsics.cx) / intrinsics.fx * z
    y = (rows - intrinsics.cy) / intrinsics.fy * z

    return np.stack([x, y, z], axis=1).astype(np.float32)


def transform_points_camera_to_base(
    points: np.ndarray,
    camera_height_m: float = 0.45,
    camera_pitch_deg: float = -20.0,
) -> np.ndarray:
    """
    Kamera çerçevesindeki noktaları basit robot base_link çerçevesine dönüştür.

    Gerçek TF transform yokken offline veya test için kullanılır.
    ROS ortamında tf2 ile dönüşüm yapılmalıdır.

    Args:
        points: (N, 3) kamera çerçevesi XYZ.
        camera_height_m: Kameranın yerden yüksekliği (metre).
        camera_pitch_deg: Kameranın eğim açısı (negatif = aşağı).

    Döndürür:
        (N, 3) float32 — base_link çerçevesinde XYZ.
    """
    import math

    if len(points) == 0:
        return points.copy()

    pitch = math.radians(camera_pitch_deg)
    cos_p, sin_p = math.cos(pitch), math.sin(pitch)

    # Kamera X → base_link X (sağ)
    # Kamera Y → base_link Z (aşağı) — pitch rotasyonuyla
    # Kamera Z → base_link X (ileri) — pitch rotasyonuyla
    x_cam = points[:, 0]
    y_cam = points[:, 1]
    z_cam = points[:, 2]

    x_base = z_cam * cos_p - y_cam * sin_p
    y_base = x_cam
    z_base = camera_height_m - (z_cam * sin_p + y_cam * cos_p)

    return np.stack([x_base, y_base, z_base], axis=1).astype(np.float32)


def filter_ground_points(
    points: np.ndarray,
    min_z: float = -0.05,
    max_z: float = 0.30,
) -> np.ndarray:
    """
    Base_link çerçevesindeki noktalardan zemin seviyesine yakın olanları al.

    Costmap'e verilecek noktalar yalnızca zemin düzeyinde olmalıdır;
    havadaki veya çok aşağıdaki noktalar STVL'yi yanıltır.

    Args:
        points: (N, 3) base_link çerçevesi XYZ.
        min_z: Minimum Z eşiği (zemin altı).
        max_z: Maksimum Z eşiği (zemin üstü).

    Döndürür:
        (M, 3) float32 — filtrelenmiş noktalar.
    """
    if len(points) == 0:
        return points
    z = points[:, 2]
    mask = (z >= min_z) & (z <= max_z)
    return points[mask]
