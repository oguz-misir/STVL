"""
negative_obstacle_common için temel pytest testleri.
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).parent.parent))

from negative_obstacle_common.risk_labels import (
    RiskLabel, SAFE, UNCERTAIN, UNSAFE_SOLID, UNSAFE_DROP,
    is_hard_negative, label_name,
)
from negative_obstacle_common.camera_utils import (
    CameraIntrinsics, depth_to_pointcloud, pixel_to_ray, project_points_to_image,
)
from negative_obstacle_common.depth_utils import (
    valid_mask, normalize_depth, depth_gradient_magnitude, depth_stats,
)
from negative_obstacle_common.metrics import (
    drop_recall, false_safe_rate, drop_precision, edge_f1, hard_negative_fpr,
)
from negative_obstacle_common.mask_colorizer import (
    colorize_risk_mask, colorize_edge_mask, label_color_bgr,
)


# -------------------------------------------------------------------------
# risk_labels
# -------------------------------------------------------------------------
class TestRiskLabels:
    def test_label_values(self) -> None:
        assert SAFE == 0
        assert UNCERTAIN == 1
        assert UNSAFE_SOLID == 2
        assert UNSAFE_DROP == 3

    def test_is_hard_negative(self) -> None:
        assert is_hard_negative("ramp_hard_negative")
        assert not is_hard_negative("rectangular_pit")

    def test_label_name(self) -> None:
        assert label_name(0) == "SAFE"
        assert label_name(3) == "UNSAFE_DROP"
        assert "UNKNOWN" in label_name(99)


# -------------------------------------------------------------------------
# camera_utils
# -------------------------------------------------------------------------
class TestCameraUtils:
    def setup_method(self) -> None:
        self.intr = CameraIntrinsics.from_fov(320, 240, fov_h_deg=70.0)

    def test_intrinsics_from_fov(self) -> None:
        assert self.intr.width == 320
        assert self.intr.height == 240
        assert self.intr.fx > 0

    def test_depth_to_pointcloud_shape(self) -> None:
        depth = np.ones((240, 320), dtype=np.float32) * 1.5
        pts = depth_to_pointcloud(depth, self.intr)
        assert pts.shape == (240 * 320, 3)

    def test_depth_to_pointcloud_ignores_nan(self) -> None:
        depth = np.ones((240, 320), dtype=np.float32) * 1.0
        depth[0, 0] = np.nan
        pts = depth_to_pointcloud(depth, self.intr)
        assert pts.shape[0] == 240 * 320 - 1

    def test_pixel_to_ray_normalized(self) -> None:
        ray = pixel_to_ray(120, 160, self.intr)
        assert abs(np.linalg.norm(ray) - 1.0) < 1e-5

    def test_project_roundtrip(self) -> None:
        depth = np.ones((240, 320), dtype=np.float32) * 2.0
        pts = depth_to_pointcloud(depth, self.intr)
        cols, rows = project_points_to_image(pts, self.intr)
        assert np.all(cols >= 0)
        assert np.all(rows >= 0)

    def test_intrinsics_to_dict_roundtrip(self) -> None:
        d = self.intr.to_dict()
        restored = CameraIntrinsics.from_dict(d)
        assert restored.fx == pytest.approx(self.intr.fx)


# -------------------------------------------------------------------------
# depth_utils
# -------------------------------------------------------------------------
class TestDepthUtils:
    def test_valid_mask_basic(self) -> None:
        depth = np.array([[1.0, np.nan], [0.0, 5.0]], dtype=np.float32)
        vm = valid_mask(depth, min_depth=0.05, max_depth=10.0)
        assert vm[0, 0] is np.bool_(True)
        assert vm[0, 1] is np.bool_(False)  # NaN
        assert vm[1, 0] is np.bool_(False)  # < min_depth

    def test_normalize_depth_range(self) -> None:
        depth = np.linspace(1.0, 5.0, 240 * 320, dtype=np.float32).reshape(240, 320)
        norm = normalize_depth(depth)
        assert float(norm.min()) == pytest.approx(0.0, abs=1e-5)
        assert float(norm.max()) == pytest.approx(1.0, abs=1e-5)

    def test_depth_gradient_shape(self) -> None:
        depth = np.ones((240, 320), dtype=np.float32)
        grad = depth_gradient_magnitude(depth)
        assert grad.shape == (240, 320)

    def test_depth_stats_keys(self) -> None:
        depth = np.ones((240, 320), dtype=np.float32) * 2.0
        stats = depth_stats(depth)
        for key in ("count", "mean", "std", "min", "max", "valid_ratio"):
            assert key in stats

    def test_depth_stats_all_nan(self) -> None:
        depth = np.full((10, 10), np.nan, dtype=np.float32)
        stats = depth_stats(depth)
        assert stats["count"] == 0


# -------------------------------------------------------------------------
# metrics
# -------------------------------------------------------------------------
class TestMetrics:
    def _perfect_pred(self) -> tuple[np.ndarray, np.ndarray]:
        gt = np.zeros((100, 100), dtype=np.uint8)
        gt[40:60, 40:60] = UNSAFE_DROP
        return gt.copy(), gt

    def test_drop_recall_perfect(self) -> None:
        pred, gt = self._perfect_pred()
        assert drop_recall(pred, gt) == pytest.approx(1.0)

    def test_false_safe_rate_perfect(self) -> None:
        pred, gt = self._perfect_pred()
        assert false_safe_rate(pred, gt) == pytest.approx(0.0)

    def test_false_safe_rate_worst(self) -> None:
        gt = np.zeros((100, 100), dtype=np.uint8)
        gt[40:60, 40:60] = UNSAFE_DROP
        pred = np.zeros_like(gt)  # hiç drop tahmin yok
        assert false_safe_rate(pred, gt) == pytest.approx(1.0)

    def test_drop_precision_perfect(self) -> None:
        pred, gt = self._perfect_pred()
        assert drop_precision(pred, gt) == pytest.approx(1.0)

    def test_edge_f1_perfect(self) -> None:
        edge = np.zeros((100, 100), dtype=np.uint8)
        edge[50, :] = 255
        assert edge_f1(edge, edge) == pytest.approx(1.0)

    def test_hard_negative_fpr_no_drop(self) -> None:
        gt = np.zeros((100, 100), dtype=np.uint8)   # drop yok
        pred = np.zeros_like(gt)
        assert hard_negative_fpr(pred, gt) == pytest.approx(0.0)

    def test_hard_negative_fpr_returns_nan_when_drop_exists(self) -> None:
        gt = np.zeros((100, 100), dtype=np.uint8)
        gt[50, 50] = UNSAFE_DROP
        pred = np.zeros_like(gt)
        result = hard_negative_fpr(pred, gt)
        assert result != result  # NaN kontrolü


# -------------------------------------------------------------------------
# mask_colorizer
# -------------------------------------------------------------------------
class TestMaskColorizer:
    def test_colorize_output_shape(self) -> None:
        mask = np.zeros((240, 320), dtype=np.uint8)
        mask[100:140, 100:220] = UNSAFE_DROP
        out = colorize_risk_mask(mask)
        assert out.shape == (240, 320, 3)

    def test_label_color_bgr_known(self) -> None:
        color = label_color_bgr(UNSAFE_DROP)
        assert len(color) == 3

    def test_edge_mask_colorize(self) -> None:
        edge = np.zeros((240, 320), dtype=np.uint8)
        edge[120, :] = 255
        out = colorize_edge_mask(edge)
        assert out.shape == (240, 320, 3)
        assert tuple(out[120, 160]) == (0, 255, 255)  # varsayılan sarı
