"""
negative_obstacle_stvl yardımcı modülleri için pytest testleri.

ROS 2 gerektirmez; saf numpy ile çalışır.
"""

from __future__ import annotations

import sys
import time
from pathlib import Path

import numpy as np
import pytest

_src = Path(__file__).parent.parent.parent.parent
sys.path.insert(0, str(_src / "negative_obstacle_common"))
sys.path.insert(0, str(_src / "negative_obstacle_stvl"))
sys.path.insert(0, str(Path(__file__).parent.parent))

from negative_obstacle_common.camera_utils import CameraIntrinsics

from negative_obstacle_stvl.edge_to_pointcloud import (
    edge_mask_to_3d_points,
    transform_points_camera_to_base,
    filter_ground_points,
)
from negative_obstacle_stvl.pointcloud2_builder import (
    build_pointcloud2_dict,
    binary_to_points,
    points_to_binary,
    POINT_STEP,
)
from negative_obstacle_stvl.temporal_persistence import (
    TemporalPointAccumulator,
    _voxel_downsample,
)


W, H = 320, 240
INTR = CameraIntrinsics.from_fov(W, H, fov_h_deg=70.0)


# -------------------------------------------------------------------------
# edge_to_pointcloud testleri
# -------------------------------------------------------------------------
class TestEdgeToPointcloud:
    def test_empty_edge_mask_returns_empty(self) -> None:
        edge = np.zeros((H, W), dtype=np.uint8)
        depth = np.ones((H, W), dtype=np.float32) * 1.5
        pts = edge_mask_to_3d_points(edge, depth, INTR)
        assert pts.shape == (0, 3)

    def test_full_edge_mask_returns_all_valid(self) -> None:
        edge = np.full((H, W), 255, dtype=np.uint8)
        depth = np.ones((H, W), dtype=np.float32) * 1.5
        pts = edge_mask_to_3d_points(edge, depth, INTR)
        assert pts.shape == (H * W, 3)
        assert pts.dtype == np.float32

    def test_nan_depth_excluded(self) -> None:
        edge = np.full((H, W), 255, dtype=np.uint8)
        depth = np.ones((H, W), dtype=np.float32) * 1.5
        depth[0, :] = np.nan   # ilk satır NaN
        pts = edge_mask_to_3d_points(edge, depth, INTR)
        assert pts.shape[0] == H * W - W

    def test_depth_dilation_shifts_z(self) -> None:
        edge = np.zeros((H, W), dtype=np.uint8)
        edge[H // 2, W // 2] = 255
        depth = np.ones((H, W), dtype=np.float32) * 2.0

        pts_no_dil  = edge_mask_to_3d_points(edge, depth, INTR, depth_dilation_m=0.0)
        pts_with_dil = edge_mask_to_3d_points(edge, depth, INTR, depth_dilation_m=0.10)

        assert pts_with_dil[0, 2] == pytest.approx(pts_no_dil[0, 2] + 0.10, abs=1e-4)

    def test_output_shape_single_point(self) -> None:
        edge = np.zeros((H, W), dtype=np.uint8)
        edge[100, 150] = 255
        depth = np.ones((H, W), dtype=np.float32) * 1.0
        pts = edge_mask_to_3d_points(edge, depth, INTR)
        assert pts.shape == (1, 3)

    def test_transform_to_base_shape(self) -> None:
        pts_cam = np.random.default_rng(0).uniform(0, 2, (50, 3)).astype(np.float32)
        pts_base = transform_points_camera_to_base(pts_cam)
        assert pts_base.shape == (50, 3)
        assert pts_base.dtype == np.float32

    def test_transform_empty_input(self) -> None:
        pts = np.empty((0, 3), dtype=np.float32)
        result = transform_points_camera_to_base(pts)
        assert result.shape == (0, 3)

    def test_filter_ground_points(self) -> None:
        pts = np.array([
            [1.0, 0.0,  0.10],   # zemin seviyesi — geçerli
            [1.0, 0.0, -0.20],   # çok aşağı — filtrelenir
            [1.0, 0.0,  0.50],   # çok yukarı — filtrelenir
        ], dtype=np.float32)
        filtered = filter_ground_points(pts, min_z=-0.05, max_z=0.30)
        assert filtered.shape == (1, 3)


# -------------------------------------------------------------------------
# pointcloud2_builder testleri
# -------------------------------------------------------------------------
class TestPointcloud2Builder:
    def test_build_dict_keys(self) -> None:
        pts = np.ones((10, 3), dtype=np.float32)
        d = build_pointcloud2_dict(pts, frame_id="base_link")
        for key in ("header", "height", "width", "fields", "data",
                    "point_step", "row_step", "is_dense"):
            assert key in d

    def test_point_step_correct(self) -> None:
        pts = np.ones((5, 3), dtype=np.float32)
        d = build_pointcloud2_dict(pts)
        assert d["point_step"] == POINT_STEP
        assert d["row_step"] == POINT_STEP * 5

    def test_binary_roundtrip(self) -> None:
        pts = np.random.default_rng(7).uniform(-1, 1, (20, 3)).astype(np.float32)
        binary = points_to_binary(pts)
        restored = binary_to_points(binary, 20)
        np.testing.assert_array_almost_equal(pts, restored)

    def test_empty_points(self) -> None:
        pts = np.empty((0, 3), dtype=np.float32)
        d = build_pointcloud2_dict(pts)
        assert d["width"] == 0
        assert d["data"] == b""

    def test_frame_id_set(self) -> None:
        pts = np.ones((3, 3), dtype=np.float32)
        d = build_pointcloud2_dict(pts, frame_id="odom")
        assert d["header"]["frame_id"] == "odom"

    def test_three_fields(self) -> None:
        pts = np.ones((3, 3), dtype=np.float32)
        d = build_pointcloud2_dict(pts)
        names = [f["name"] for f in d["fields"]]
        assert names == ["x", "y", "z"]


# -------------------------------------------------------------------------
# temporal_persistence testleri
# -------------------------------------------------------------------------
class TestTemporalPersistence:
    def test_empty_accumulator(self) -> None:
        acc = TemporalPointAccumulator(window_sec=1.0)
        result = acc.get_accumulated()
        assert result.shape == (0, 3)

    def test_add_and_retrieve(self) -> None:
        acc = TemporalPointAccumulator(window_sec=10.0, voxel_size=0.0)
        pts = np.ones((50, 3), dtype=np.float32)
        acc.add(pts)
        result = acc.get_accumulated()
        assert result.shape == (50, 3)

    def test_accumulates_multiple_frames(self) -> None:
        acc = TemporalPointAccumulator(window_sec=10.0, voxel_size=0.0)
        for _ in range(5):
            acc.add(np.ones((10, 3), dtype=np.float32))
        result = acc.get_accumulated()
        assert result.shape[0] == 50

    def test_max_points_limit(self) -> None:
        acc = TemporalPointAccumulator(window_sec=10.0, max_points=20, voxel_size=0.0)
        acc.add(np.ones((100, 3), dtype=np.float32))
        result = acc.get_accumulated()
        assert result.shape[0] <= 20

    def test_frame_count(self) -> None:
        acc = TemporalPointAccumulator(window_sec=10.0)
        acc.add(np.ones((5, 3), dtype=np.float32))
        acc.add(np.ones((5, 3), dtype=np.float32))
        assert acc.frame_count == 2

    def test_clear(self) -> None:
        acc = TemporalPointAccumulator(window_sec=10.0)
        acc.add(np.ones((10, 3), dtype=np.float32))
        acc.clear()
        assert acc.get_accumulated().shape == (0, 3)

    def test_add_empty_points_ignored(self) -> None:
        acc = TemporalPointAccumulator(window_sec=10.0)
        acc.add(np.empty((0, 3), dtype=np.float32))
        assert acc.frame_count == 0

    def test_voxel_downsample_reduces_count(self) -> None:
        # 1000 nokta yakın konumda → voxel sonrası çok daha az
        pts = np.random.default_rng(0).uniform(0, 0.1, (1000, 3)).astype(np.float32)
        downsampled = _voxel_downsample(pts, voxel_size=0.05)
        assert downsampled.shape[0] < 1000

    def test_voxel_downsample_preserves_spread(self) -> None:
        # Geniş alana yayılmış noktalar → voxel sonrası çok azalmamalı
        pts = np.random.default_rng(1).uniform(0, 5.0, (100, 3)).astype(np.float32)
        downsampled = _voxel_downsample(pts, voxel_size=0.05)
        # Her nokta kendi voxelinde → yaklaşık aynı sayıda kalmalı
        assert downsampled.shape[0] >= 80
