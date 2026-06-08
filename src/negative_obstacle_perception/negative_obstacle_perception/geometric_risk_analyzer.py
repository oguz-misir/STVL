"""
Geometrik ve belirsizlik-duyarlı risk üreticisi.

GroundEstimator ve DepthDiscontinuityDetector çıktılarını birleştirerek
hem geriye uyumlu risk maskesi (SAFE/UNCERTAIN/UNSAFE_DROP) hem de
dereceli negatif engel risk alanı üretir.
ROS node değildir; offline ve online her ikisinde kullanılabilir.
"""

from __future__ import annotations

import cv2
import numpy as np

from negative_obstacle_common.camera_utils import CameraIntrinsics
from negative_obstacle_common.risk_labels import SAFE, UNCERTAIN, UNSAFE_DROP

from .ground_estimator import GroundEstimator
from .depth_discontinuity_detector import DepthDiscontinuityDetector
from .risk_field import RiskFieldConfig, UncertaintyAwareRiskField


class GeometricRiskAnalyzer:
    """
    Tek RGB-D görüntüden geometrik risk maskesi üretir.

    Birleştirme mantığı (öncelik sırası):
    1. UNSAFE_DROP  — ground drop + edge bölgelerinin kesişimi veya birleşimi
    2. UNCERTAIN    — missing depth bölgeleri
    3. SAFE         — geri kalan geçerli pikseller
    """

    def __init__(
        self,
        intrinsics: CameraIntrinsics,
        jump_threshold_m: float = 0.15,
        drop_threshold_m: float = 0.08,
        dilation_drop_px: int = 4,
        dilation_edge_px: int = 2,
        min_missing_area_px: int = 50,
        ransac_iterations: int = 80,
        use_ground_estimator: bool = True,
        risk_field_config: RiskFieldConfig | None = None,
    ) -> None:
        self.intrinsics = intrinsics
        self.dilation_drop_px = dilation_drop_px
        self.use_ground_estimator = use_ground_estimator

        self._ground_est = GroundEstimator(
            intrinsics=intrinsics,
            ransac_iterations=ransac_iterations,
            drop_threshold_m=drop_threshold_m,
        )
        self._disc_det = DepthDiscontinuityDetector(
            jump_threshold_m=jump_threshold_m,
            dilation_px=dilation_edge_px,
            min_missing_area_px=min_missing_area_px,
        )
        self._risk_field = UncertaintyAwareRiskField(
            risk_field_config
            or RiskFieldConfig(
                jump_threshold_m=jump_threshold_m,
                drop_threshold_m=drop_threshold_m,
            )
        )

    def analyze(self, depth: np.ndarray, rng_seed: int = 0) -> dict:
        """
        Depth görüntüsünü analiz et ve risk maskesi üret.

        Args:
            depth: (H, W) float32, metre. NaN = geçersiz piksel.
            rng_seed: RANSAC tekrar üretilebilirliği için.

        Döndürür:
            dict ile:
            - risk_mask: (H, W) uint8 — SAFE=0, UNCERTAIN=1, UNSAFE_DROP=3
            - edge_mask: (H, W) uint8 — drop-edge pikseller (255)
            - drop_mask: (H, W) bool — ham drop bölgesi
            - risk_field: dict — p_drop, p_unknown, confidence,
              temporal_stability, risk_score, cost
            - risk_cost: (H, W) uint8 — Nav2 uyumlu dereceli maliyet
            - ground_plane: (4,) float veya None
            - ground_success: bool
        """
        h, w = depth.shape
        risk_mask = np.full((h, w), SAFE, dtype=np.uint8)

        # Süreksizlik tespiti
        disc = self._disc_det.detect(depth)
        edge_mask: np.ndarray = disc["edge_mask"]
        missing_mask: np.ndarray = disc["missing_mask"]

        # Missing bölgeler → UNCERTAIN
        risk_mask[missing_mask > 127] = UNCERTAIN

        # Ground tahmini
        ground_plane = None
        ground_success = False
        drop_mask = np.zeros((h, w), dtype=bool)
        ground_mask = np.zeros((h, w), dtype=bool)
        height_map = np.full((h, w), np.nan, dtype=np.float32)

        if self.use_ground_estimator:
            ground_result = self._ground_est.estimate(depth, rng_seed=rng_seed)
            ground_success = ground_result["success"]
            ground_plane = ground_result["plane"]
            drop_mask = ground_result["drop_mask"]
            ground_mask = ground_result["ground_mask"]
            height_map = ground_result["height_map"]

        # Drop bölgesini genişlet ve edge ile birleştir
        drop_u8 = drop_mask.astype(np.uint8)

        if self.dilation_drop_px > 0:
            kernel = cv2.getStructuringElement(
                cv2.MORPH_ELLIPSE,
                (2 * self.dilation_drop_px + 1, 2 * self.dilation_drop_px + 1),
            )
            drop_dilated = cv2.dilate(drop_u8, kernel).astype(bool)
        else:
            drop_dilated = drop_mask

        # Edge maskesini drop bölgesiyle birleştir
        edge_bin = edge_mask > 127
        unsafe_region = drop_dilated | edge_bin

        risk_mask[unsafe_region] = UNSAFE_DROP

        # UNCERTAIN drop bölgesini ezip UNSAFE_DROP olarak işaretle
        # (drop her zaman uncertain'dan önceliklidir)
        risk_mask[drop_mask] = UNSAFE_DROP

        risk_field = self._risk_field.compute(
            depth=depth,
            jump_map=disc["jump_map"],
            missing_mask=missing_mask,
            edge_mask=edge_mask,
            height_map=height_map,
            ground_mask=ground_mask,
            ground_success=ground_success,
        )

        return {
            "risk_mask": risk_mask,
            "edge_mask": edge_mask,
            "drop_mask": drop_mask,
            "risk_field": risk_field,
            "risk_cost": risk_field["cost"],
            "drop_probability": risk_field["p_drop"],
            "unknown_probability": risk_field["p_unknown"],
            "confidence": risk_field["confidence"],
            "temporal_stability": risk_field["temporal_stability"],
            "ground_plane": ground_plane,
            "ground_success": ground_success,
        }

    def analyze_edge_only(self, depth: np.ndarray) -> dict:
        """
        Yalnızca süreksizlik tabanlı hafif analiz (RANSAC olmadan).

        Ground tahmini yapılmaz; yalnızca depth jump'lardan edge ve risk maskesi üretir.
        Düşük gecikmeli, Raspberry Pi 5 uyumlu çalışma modu.
        """
        h, w = depth.shape
        risk_mask = np.full((h, w), SAFE, dtype=np.uint8)

        disc = self._disc_det.detect(depth)
        edge_mask: np.ndarray = disc["edge_mask"]
        missing_mask: np.ndarray = disc["missing_mask"]

        risk_mask[missing_mask > 127] = UNCERTAIN
        risk_mask[edge_mask > 127] = UNSAFE_DROP

        risk_field = self._risk_field.compute(
            depth=depth,
            jump_map=disc["jump_map"],
            missing_mask=missing_mask,
            edge_mask=edge_mask,
            height_map=None,
            ground_mask=None,
            ground_success=False,
        )

        return {
            "risk_mask": risk_mask,
            "edge_mask": edge_mask,
            "drop_mask": np.zeros((h, w), dtype=bool),
            "risk_field": risk_field,
            "risk_cost": risk_field["cost"],
            "drop_probability": risk_field["p_drop"],
            "unknown_probability": risk_field["p_unknown"],
            "confidence": risk_field["confidence"],
            "temporal_stability": risk_field["temporal_stability"],
            "ground_plane": None,
            "ground_success": False,
        }
