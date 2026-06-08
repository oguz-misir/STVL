"""
negative_obstacle_perception için temel pytest testleri.

Gerçek sentetik veri setinden bağımsız; küçük sentetik depth dizileriyle çalışır.
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).parent.parent.parent / "negative_obstacle_common"))
sys.path.insert(0, str(Path(__file__).parent.parent))

from negative_obstacle_common.camera_utils import CameraIntrinsics
from negative_obstacle_common.risk_labels import SAFE, UNCERTAIN, UNSAFE_DROP

from negative_obstacle_perception.ground_estimator import (
    GroundEstimator, ransac_ground_plane,
)
from negative_obstacle_perception.depth_discontinuity_detector import (
    DepthDiscontinuityDetector, compute_depth_jump_map, detect_drop_edges,
)
from negative_obstacle_perception.geometric_risk_analyzer import GeometricRiskAnalyzer


# -------------------------------------------------------------------------
# Yardımcı sabitler
# -------------------------------------------------------------------------
W, H = 80, 60
INTR = CameraIntrinsics.from_fov(W, H, fov_h_deg=70.0)


def _flat_depth(val: float = 1.5) -> np.ndarray:
    """Tamamen düz, sabit derinlikli görüntü."""
    return np.full((H, W), val, dtype=np.float32)


def _pit_depth(
    pit_val: float = 2.0,
    ground_val: float = 1.5,
    row_start: int = 25,
    row_end: int = 40,
    col_start: int = 20,
    col_end: int = 60,
) -> np.ndarray:
    """Ortada dikdörtgen bir çukur içeren depth görüntüsü."""
    depth = np.full((H, W), ground_val, dtype=np.float32)
    depth[row_start:row_end, col_start:col_end] = pit_val
    return depth


# -------------------------------------------------------------------------
# GroundEstimator testleri
# -------------------------------------------------------------------------
class TestGroundEstimator:
    def test_flat_ground_success(self) -> None:
        depth = _flat_depth(1.5)
        est = GroundEstimator(INTR, ransac_iterations=30)
        result = est.estimate(depth, rng_seed=0)
        assert result["success"] is True
        assert result["plane"] is not None
        assert result["plane"].shape == (4,)

    def test_flat_ground_no_drop(self) -> None:
        depth = _flat_depth(1.5)
        est = GroundEstimator(INTR, ransac_iterations=30, drop_threshold_m=0.08)
        result = est.estimate(depth, rng_seed=0)
        assert not np.any(result["drop_mask"]), "Düz zeminde drop bölgesi olmamalı"

    def test_pit_produces_drop_mask(self) -> None:
        depth = _pit_depth(pit_val=2.0, ground_val=1.5)
        est = GroundEstimator(INTR, ransac_iterations=50, drop_threshold_m=0.08)
        result = est.estimate(depth, rng_seed=42)
        assert result["success"] is True
        # Çukur bölgesinde en az bir drop pikseli olmalı
        drop = result["drop_mask"]
        assert np.any(drop[25:40, 20:60]), "Çukur bölgesinde drop maskesi bekleniyor"

    def test_height_map_shape(self) -> None:
        depth = _flat_depth(1.5)
        est = GroundEstimator(INTR)
        result = est.estimate(depth)
        assert result["height_map"].shape == (H, W)

    def test_ransac_plane_shape(self) -> None:
        pts = np.random.default_rng(0).uniform(0, 2, (200, 3)).astype(np.float32)
        pts[:, 1] = 0.0 + np.random.default_rng(1).normal(0, 0.01, 200)
        plane, inliers = ransac_ground_plane(pts, n_iterations=50)
        assert plane is not None
        assert plane.shape == (4,)
        assert inliers.shape == (200,)

    def test_insufficient_points_returns_failure(self) -> None:
        depth = np.full((H, W), np.nan, dtype=np.float32)
        est = GroundEstimator(INTR)
        result = est.estimate(depth)
        assert result["success"] is False


# -------------------------------------------------------------------------
# DepthDiscontinuityDetector testleri
# -------------------------------------------------------------------------
class TestDepthDiscontinuityDetector:
    def test_flat_depth_no_edges(self) -> None:
        depth = _flat_depth(1.5)
        det = DepthDiscontinuityDetector(jump_threshold_m=0.10)
        result = det.detect(depth)
        # Düz zeminde kenar olmamalı (sınır pikseller hariç)
        interior = result["edge_mask"][2:-2, 2:-2]
        assert np.all(interior == 0), "Düz derinlikte iç piksellerde edge olmamalı"

    def test_pit_produces_edge(self) -> None:
        depth = _pit_depth(pit_val=2.0, ground_val=1.5)
        det = DepthDiscontinuityDetector(jump_threshold_m=0.10, dilation_px=1)
        result = det.detect(depth)
        assert np.any(result["edge_mask"] > 0), "Çukur kenarında edge bekleniyor"

    def test_jump_map_shape(self) -> None:
        depth = _flat_depth(1.5)
        jmap = compute_depth_jump_map(depth)
        assert jmap.shape == (H, W)
        assert jmap.dtype == np.float32

    def test_jump_map_zero_flat(self) -> None:
        depth = _flat_depth(1.5)
        jmap = compute_depth_jump_map(depth)
        assert float(jmap[H // 2, W // 2]) == pytest.approx(0.0, abs=1e-4)

    def test_missing_mask_nan_regions(self) -> None:
        depth = _flat_depth(1.5)
        # Büyük bir NaN bölgesi yarat
        depth[20:40, 20:60] = np.nan
        det = DepthDiscontinuityDetector(min_missing_area_px=10)
        result = det.detect(depth)
        missing = result["missing_mask"]
        # NaN bölgede en az bir missing pikseli olmalı
        assert np.any(missing[20:40, 20:60] > 0)

    def test_detect_drop_edges_returns_uint8(self) -> None:
        depth = _pit_depth()
        edges = detect_drop_edges(depth, jump_threshold=0.10)
        assert edges.dtype == np.uint8
        assert set(np.unique(edges).tolist()).issubset({0, 255})


# -------------------------------------------------------------------------
# GeometricRiskAnalyzer testleri
# -------------------------------------------------------------------------
class TestGeometricRiskAnalyzer:
    def test_flat_depth_all_safe(self) -> None:
        depth = _flat_depth(1.5)
        analyzer = GeometricRiskAnalyzer(INTR, use_ground_estimator=False)
        result = analyzer.analyze_edge_only(depth)
        risk = result["risk_mask"]
        interior = risk[2:-2, 2:-2]
        assert np.all(interior == SAFE), "Düz derinlikte iç pikseller SAFE olmalı"

    def test_pit_produces_unsafe_drop(self) -> None:
        depth = _pit_depth(pit_val=2.2, ground_val=1.5)
        analyzer = GeometricRiskAnalyzer(
            INTR,
            jump_threshold_m=0.10,
            drop_threshold_m=0.08,
            use_ground_estimator=True,
            ransac_iterations=40,
        )
        result = analyzer.analyze(depth, rng_seed=0)
        risk = result["risk_mask"]
        assert np.any(risk == UNSAFE_DROP), "Çukur bölgesinde UNSAFE_DROP bekleniyor"

    def test_nan_region_becomes_uncertain(self) -> None:
        depth = _flat_depth(1.5)
        depth[20:40, 20:60] = np.nan
        analyzer = GeometricRiskAnalyzer(INTR, use_ground_estimator=False)
        result = analyzer.analyze_edge_only(depth)
        risk = result["risk_mask"]
        assert np.any(risk[22:38, 22:58] == UNCERTAIN), "NaN bölge UNCERTAIN olmalı"

    def test_risk_mask_shape(self) -> None:
        depth = _flat_depth(1.5)
        analyzer = GeometricRiskAnalyzer(INTR, use_ground_estimator=False)
        result = analyzer.analyze_edge_only(depth)
        assert result["risk_mask"].shape == (H, W)

    def test_risk_mask_values_in_range(self) -> None:
        depth = _pit_depth()
        analyzer = GeometricRiskAnalyzer(INTR, use_ground_estimator=False)
        result = analyzer.analyze_edge_only(depth)
        unique = set(np.unique(result["risk_mask"]).tolist())
        assert unique.issubset({0, 1, 2, 3}), f"Beklenmeyen label değerleri: {unique}"

    def test_edge_mask_shape_and_dtype(self) -> None:
        depth = _pit_depth()
        analyzer = GeometricRiskAnalyzer(INTR, use_ground_estimator=False)
        result = analyzer.analyze_edge_only(depth)
        assert result["edge_mask"].shape == (H, W)
        assert result["edge_mask"].dtype == np.uint8

    def test_edge_only_no_ground_plane(self) -> None:
        depth = _flat_depth(1.5)
        analyzer = GeometricRiskAnalyzer(INTR, use_ground_estimator=True)
        result = analyzer.analyze_edge_only(depth)
        assert result["ground_plane"] is None
        assert result["ground_success"] is False

    def test_risk_field_channels_shape_and_range(self) -> None:
        depth = _pit_depth(pit_val=2.2, ground_val=1.5)
        analyzer = GeometricRiskAnalyzer(
            INTR,
            jump_threshold_m=0.10,
            drop_threshold_m=0.08,
            use_ground_estimator=True,
            ransac_iterations=40,
        )
        result = analyzer.analyze(depth, rng_seed=0)
        field = result["risk_field"]

        for key in ["p_drop", "p_unknown", "confidence", "temporal_stability", "risk_score"]:
            assert field[key].shape == (H, W)
            assert field[key].dtype == np.float32
            assert float(np.min(field[key])) >= 0.0
            assert float(np.max(field[key])) <= 1.0

        assert field["cost"].shape == (H, W)
        assert field["cost"].dtype == np.uint8

    def test_pit_has_higher_drop_probability_than_flat_ground(self) -> None:
        flat = _flat_depth(1.5)
        pit = _pit_depth(pit_val=2.2, ground_val=1.5)
        analyzer = GeometricRiskAnalyzer(
            INTR,
            jump_threshold_m=0.10,
            drop_threshold_m=0.08,
            use_ground_estimator=True,
            ransac_iterations=40,
        )
        flat_result = analyzer.analyze(flat, rng_seed=0)
        pit_result = analyzer.analyze(pit, rng_seed=0)

        flat_score = float(np.mean(flat_result["drop_probability"][20:45, 15:65]))
        pit_score = float(np.mean(pit_result["drop_probability"][25:40, 20:60]))
        assert pit_score > flat_score
