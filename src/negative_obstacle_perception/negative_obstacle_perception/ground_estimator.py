"""
Zemin düzlemi tahmincisi.

Depth görüntüsünden RANSAC ile zemin düzlemini tahmin eder.
Zemin dışı pikselleri ve zemin altı (drop) bölgeleri belirler.
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parent.parent.parent.parent / "negative_obstacle_common"))
from negative_obstacle_common.camera_utils import CameraIntrinsics
from negative_obstacle_common.depth_utils import valid_mask


def _fit_plane_svd(points: np.ndarray) -> np.ndarray:
    """
    Nokta bulutuna en küçük kareler ile düzlem uydur.

    Döndürür:
        (4,) float — [a, b, c, d] katsayıları: ax + by + cz = d,
        normalize edilmiş normal vektörü ile.
    """
    centroid = points.mean(axis=0)
    centered = points - centroid
    _, _, Vt = np.linalg.svd(centered, full_matrices=False)
    normal = Vt[-1]  # en küçük tekil değere karşılık gelen vektör
    d = float(np.dot(normal, centroid))
    return np.append(normal, d).astype(np.float32)


def ransac_ground_plane(
    points: np.ndarray,
    n_iterations: int = 80,
    distance_threshold: float = 0.04,
    min_inlier_ratio: float = 0.30,
    rng_seed: int = 0,
) -> tuple[np.ndarray | None, np.ndarray]:
    """
    RANSAC ile zemin düzlemini tahmin et.

    Args:
        points: (N, 3) float32 — kamera çerçevesinde nokta bulutu.
        n_iterations: RANSAC iterasyon sayısı.
        distance_threshold: Inlier kabul mesafesi (metre).
        min_inlier_ratio: Minimum inlier oranı; altında None döner.
        rng_seed: Tekrar üretilebilirlik için seed.

    Returns:
        (plane, inlier_mask): plane (4,) veya None; inlier_mask (N,) bool.
    """
    if len(points) < 3:
        return None, np.zeros(len(points), dtype=bool)

    rng = np.random.default_rng(rng_seed)
    n = len(points)
    best_plane: np.ndarray | None = None
    best_inliers = np.zeros(n, dtype=bool)
    best_count = 0

    for _ in range(n_iterations):
        idx = rng.choice(n, size=3, replace=False)
        sample = points[idx]

        # 3 noktadan düzlem
        v1 = sample[1] - sample[0]
        v2 = sample[2] - sample[0]
        normal = np.cross(v1, v2)
        norm_len = np.linalg.norm(normal)
        if norm_len < 1e-8:
            continue
        normal = normal / norm_len
        d = float(np.dot(normal, sample[0]))
        plane = np.append(normal, d).astype(np.float32)

        # Inlier kontrolü
        distances = np.abs(points @ plane[:3] - plane[3])
        inliers = distances < distance_threshold
        count = int(inliers.sum())

        if count > best_count:
            best_count = count
            best_inliers = inliers
            best_plane = plane

    if best_plane is None or best_count / n < min_inlier_ratio:
        return None, best_inliers

    # SVD ile inlier'lar üzerinde rafine et
    refined_plane = _fit_plane_svd(points[best_inliers])
    distances = np.abs(points @ refined_plane[:3] - refined_plane[3])
    final_inliers = distances < distance_threshold

    return refined_plane, final_inliers


def orient_plane_toward_camera(plane: np.ndarray) -> np.ndarray:
    """
    Düzlem normalini kameraya bakan yönde normalize et.

    Kamera çerçevesinde Z ekseni sahneye doğru (+z) işaret eder.
    Normal -z yönünde (kameraya bakacak şekilde) olursa, imzalı mesafe
    zemin altındaki noktalar (çukur) için negatif olur — drop tespitinde
    tutarlı işaret sağlar.
    """
    if plane[2] > 0:
        return -plane
    return plane.copy()


def signed_distance_to_plane(
    points: np.ndarray,
    plane: np.ndarray,
) -> np.ndarray:
    """
    Her noktanın düzleme imzalı uzaklığını hesapla.

    Normal kameraya bakacak şekilde orient_plane_toward_camera ile
    düzenlendikten sonra:
      Pozitif = zemin üstü (engel veya zemin inlier)
      Negatif = zemin altı (drop / çukur bölgesi)
    """
    return (points @ plane[:3] - plane[3]).astype(np.float32)


class GroundEstimator:
    """
    Tek görüntü veya akış için zemin tahmini sarmalayıcısı.

    Kullanım:
        estimator = GroundEstimator(intrinsics)
        result = estimator.estimate(depth)
    """

    def __init__(
        self,
        intrinsics: CameraIntrinsics,
        ransac_iterations: int = 80,
        distance_threshold_m: float = 0.04,
        drop_threshold_m: float = 0.08,
        min_inlier_ratio: float = 0.25,
    ) -> None:
        self.intrinsics = intrinsics
        self.ransac_iterations = ransac_iterations
        self.distance_threshold_m = distance_threshold_m
        self.drop_threshold_m = drop_threshold_m
        self.min_inlier_ratio = min_inlier_ratio

    def estimate(
        self,
        depth: np.ndarray,
        rng_seed: int = 0,
    ) -> dict:
        """
        Depth görüntüsünden zemin düzlemini tahmin et.

        Döndürür:
            dict ile şu alanlar:
            - plane: (4,) float veya None
            - ground_mask: (H, W) bool — zemin inlier pikseller
            - drop_mask: (H, W) bool — zemin altında kalan pikseller
            - height_map: (H, W) float — her pikselin zemine göre yüksekliği
            - success: bool
        """
        h, w = depth.shape
        vm = valid_mask(depth)

        result = {
            "plane": None,
            "ground_mask": np.zeros((h, w), dtype=bool),
            "drop_mask": np.zeros((h, w), dtype=bool),
            "height_map": np.full((h, w), np.nan, dtype=np.float32),
            "success": False,
        }

        # Piksel indekslerini ve nokta bulutunu aynı vm maskesinden üret
        valid_rows = np.argwhere(vm)[:, 0]
        valid_cols = np.argwhere(vm)[:, 1]
        z_vals = depth[vm].astype(np.float32)
        x_vals = (valid_cols - self.intrinsics.cx) / self.intrinsics.fx * z_vals
        y_vals = (valid_rows - self.intrinsics.cy) / self.intrinsics.fy * z_vals
        pts = np.stack([x_vals, y_vals, z_vals], axis=1)

        if len(pts) < 10:
            return result

        plane, inlier_mask_pts = ransac_ground_plane(
            pts,
            n_iterations=self.ransac_iterations,
            distance_threshold=self.distance_threshold_m,
            min_inlier_ratio=self.min_inlier_ratio,
            rng_seed=rng_seed,
        )

        if plane is None:
            return result

        # Normal yönünü kameraya bakacak şekilde sabitle
        plane = orient_plane_toward_camera(plane)

        # Her geçerli pikselin zemine imzalı uzaklığı
        signed_dist = signed_distance_to_plane(pts, plane)

        height_map = result["height_map"]
        height_map[valid_rows, valid_cols] = signed_dist

        # Zemin maskesi: |dist| < threshold
        ground_flat = np.abs(signed_dist) < self.distance_threshold_m
        ground_mask = np.zeros((h, w), dtype=bool)
        ground_mask[valid_rows, valid_cols] = ground_flat

        # Drop maskesi: zemin seviyesinin önemli ölçüde altında
        drop_flat = signed_dist < -self.drop_threshold_m
        drop_mask = np.zeros((h, w), dtype=bool)
        drop_mask[valid_rows, valid_cols] = drop_flat

        result.update({
            "plane": plane,
            "ground_mask": ground_mask,
            "drop_mask": drop_mask,
            "height_map": height_map,
            "success": True,
        })
        return result
