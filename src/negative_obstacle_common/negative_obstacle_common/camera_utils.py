"""
Kamera intrinsics ve projeksiyon yardımcıları.

Tüm fonksiyonlar numpy vektörizasyonu kullanır; döngü içinde çağrılabilir.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class CameraIntrinsics:
    fx: float
    fy: float
    cx: float
    cy: float
    width: int
    height: int

    @classmethod
    def from_fov(cls, width: int, height: int, fov_h_deg: float = 70.0) -> "CameraIntrinsics":
        """Yatay FOV açısından intrinsics hesapla."""
        fov_h = math.radians(fov_h_deg)
        fx = width / (2.0 * math.tan(fov_h / 2.0))
        fy = fx
        cx = width / 2.0
        cy = height / 2.0
        return cls(fx=fx, fy=fy, cx=cx, cy=cy, width=width, height=height)

    @classmethod
    def from_dict(cls, d: dict) -> "CameraIntrinsics":
        return cls(
            fx=float(d["fx"]),
            fy=float(d["fy"]),
            cx=float(d["cx"]),
            cy=float(d["cy"]),
            width=int(d["width"]),
            height=int(d["height"]),
        )

    def to_dict(self) -> dict:
        return {
            "fx": self.fx, "fy": self.fy,
            "cx": self.cx, "cy": self.cy,
            "width": self.width, "height": self.height,
        }


def depth_to_pointcloud(
    depth: np.ndarray,
    intrinsics: CameraIntrinsics,
    depth_scale: float = 1.0,
) -> np.ndarray:
    """
    Depth görüntüsünü kamera çerçevesinde 3D nokta bulutuna dönüştür.

    Args:
        depth: (H, W) float array, metre cinsinden. NaN = geçersiz.
        intrinsics: Kamera iç parametreleri.
        depth_scale: depth değerlerini metreye çeviren çarpan.

    Returns:
        (N, 3) float32 array — geçerli piksellerden üretilen XYZ noktaları.
        Kamera çerçevesi: X sağ, Y aşağı, Z ileri.
    """
    h, w = depth.shape
    rows, cols = np.meshgrid(np.arange(h), np.arange(w), indexing="ij")

    z = depth * depth_scale
    valid = np.isfinite(z) & (z > 0)

    z_v = z[valid]
    x_v = (cols[valid] - intrinsics.cx) / intrinsics.fx * z_v
    y_v = (rows[valid] - intrinsics.cy) / intrinsics.fy * z_v

    return np.stack([x_v, y_v, z_v], axis=1).astype(np.float32)


def pixel_to_ray(
    row: int | np.ndarray,
    col: int | np.ndarray,
    intrinsics: CameraIntrinsics,
) -> np.ndarray:
    """
    Piksel koordinatlarını normalize edilmiş kamera ışını yönüne çevir.

    Returns:
        (..., 3) float32 — (rx, ry, rz) normalize ışın vektörü.
    """
    rx = (np.asarray(col, dtype=np.float32) - intrinsics.cx) / intrinsics.fx
    ry = (np.asarray(row, dtype=np.float32) - intrinsics.cy) / intrinsics.fy
    rz = np.ones_like(rx)
    rays = np.stack([rx, ry, rz], axis=-1)
    norms = np.linalg.norm(rays, axis=-1, keepdims=True)
    return (rays / np.where(norms > 0, norms, 1.0)).astype(np.float32)


def project_points_to_image(
    points: np.ndarray,
    intrinsics: CameraIntrinsics,
) -> tuple[np.ndarray, np.ndarray]:
    """
    Kamera çerçevesindeki 3D noktaları piksel koordinatlarına projekte et.

    Args:
        points: (N, 3) XYZ kamera çerçevesinde.
        intrinsics: Kamera iç parametreleri.

    Returns:
        (cols, rows): (N,) int32 piksel koordinatları.
        Geçersiz (z<=0 veya görüntü dışı) noktalar -1 ile işaretlenir.
    """
    z = points[:, 2]
    valid = z > 0

    cols = np.full(len(points), -1, dtype=np.int32)
    rows = np.full(len(points), -1, dtype=np.int32)

    u = (points[valid, 0] / z[valid] * intrinsics.fx + intrinsics.cx).astype(np.int32)
    v = (points[valid, 1] / z[valid] * intrinsics.fy + intrinsics.cy).astype(np.int32)

    in_frame = (u >= 0) & (u < intrinsics.width) & (v >= 0) & (v < intrinsics.height)
    idx = np.where(valid)[0][in_frame]
    cols[idx] = u[in_frame]
    rows[idx] = v[in_frame]

    return cols, rows
