"""
gazebo_label_generator.py için birim testler.

Gazebo gerektirmez — sentetik derinlik dizileri oluşturarak
kamera modelini ve etiketleme mantığını doğrular.

Çalıştırma:
  PYTHONPATH=src/negative_obstacle_data \
    python3 -m pytest src/negative_obstacle_data/test/test_gazebo_label_generator.py -v
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np
import pytest

# PYTHONPATH yönetimi
_root = Path(__file__).parent.parent.parent.parent
sys.path.insert(0, str(_root / "src" / "negative_obstacle_data"))

from negative_obstacle_data.gazebo_label_generator import (
    build_cam_to_world,
    depth_to_world_points,
    generate_labels,
    process_sample,
)

# ---------------------------------------------------------------------------
# Sabitler — pit_scene.sdf ile uyumlu
# ---------------------------------------------------------------------------
CAM_POS   = [0.15, 0.0, 0.45]
PITCH_DEG = 20.0
PITCH_RAD = math.radians(PITCH_DEG)
W, H      = 320, 240
FOV_H_DEG = 70.0
FX        = (W / 2.0) / math.tan(math.radians(FOV_H_DEG) / 2.0)
CX, CY    = W / 2.0, H / 2.0


# ---------------------------------------------------------------------------
# Yardımcı fonksiyonlar
# ---------------------------------------------------------------------------

def make_scene_cfg(pitch_deg: float = PITCH_DEG, drop_thr: float = 0.15) -> dict:
    return {
        "scene_type": "rectangular_pit",
        "camera": {
            "position_xyz": CAM_POS,
            "pitch_sdf_deg": pitch_deg,
            "fov_h_deg": FOV_H_DEG,
        },
        "ground_z": 0.0,
        "drop_detection_threshold_m": drop_thr,
    }


def ground_depth_at_center(cam_h: float = 0.45, pitch_deg: float = PITCH_DEG) -> float:
    """Orta piksel için beklenen zemin derinliğini hesaplar."""
    psi = math.radians(pitch_deg)
    cos_p = math.cos(psi)
    sin_p = math.sin(psi)
    # Optik eksen yönü: [cos_p, 0, -sin_p]
    # z_world = 0 için: cam_pos[2] + t * (-sin_p) = 0 → t = cam_h / sin_p
    # Derinlik = t * 1 (ray direction magnitude ≈ 1 for center pixel normalized)
    t = cam_h / sin_p
    return float(t)


# ---------------------------------------------------------------------------
# Testler — kamera modeli
# ---------------------------------------------------------------------------

class TestBuildCamToWorld:
    def test_returns_4x4(self):
        T = build_cam_to_world(CAM_POS, PITCH_RAD)
        assert T.shape == (4, 4)

    def test_translation_column(self):
        T = build_cam_to_world(CAM_POS, PITCH_RAD)
        np.testing.assert_allclose(T[:3, 3], CAM_POS, atol=1e-9)

    def test_rotation_is_proper(self):
        """Döndürme matrisi determinantı +1 olmalı."""
        T = build_cam_to_world(CAM_POS, PITCH_RAD)
        R = T[:3, :3]
        np.testing.assert_allclose(np.linalg.det(R), 1.0, atol=1e-9)

    def test_rotation_is_orthogonal(self):
        T = build_cam_to_world(CAM_POS, PITCH_RAD)
        R = T[:3, :3]
        np.testing.assert_allclose(R @ R.T, np.eye(3), atol=1e-9)

    def test_optical_axis_direction(self):
        """Optik eksen (kamera +Z) dünyada ileri+aşağı göstermeli."""
        T = build_cam_to_world(CAM_POS, PITCH_RAD)
        R = T[:3, :3]
        camera_z_in_world = R[:, 2]
        # x > 0 (ileri), y = 0, z < 0 (aşağı)
        assert camera_z_in_world[0] > 0.5, "Optik eksen öne bakmalı"
        assert camera_z_in_world[2] < -0.1, "Optik eksen aşağı eğimli olmalı"
        assert abs(camera_z_in_world[1]) < 1e-9, "Y bileşeni sıfır olmalı"

    def test_level_camera_zero_pitch(self):
        """ψ=0 → optik eksen düz ileriye (+X) bakmalı."""
        T = build_cam_to_world([0, 0, 0], 0.0)
        R = T[:3, :3]
        cam_z = R[:, 2]
        np.testing.assert_allclose(cam_z, [1.0, 0.0, 0.0], atol=1e-9)


# ---------------------------------------------------------------------------
# Testler — kamera→dünya projeksiyon
# ---------------------------------------------------------------------------

class TestDepthToWorldPoints:
    def _cam_to_world(self):
        return build_cam_to_world(CAM_POS, PITCH_RAD)

    def test_output_shape(self):
        depth = np.ones((H, W), dtype=np.float32)
        pts = depth_to_world_points(depth, FX, FX, CX, CY, self._cam_to_world())
        assert pts.shape == (H, W, 3)

    def test_center_pixel_z_negative(self):
        """Orta pikselin dünya z'si, kameradan daha düşük olmalı (aşağı bakıyor)."""
        depth_val = ground_depth_at_center()
        depth = np.full((H, W), depth_val, dtype=np.float32)
        pts = depth_to_world_points(depth, FX, FX, CX, CY, self._cam_to_world())
        z_center = pts[H // 2, W // 2, 2]
        assert z_center < 0.1, f"Orta piksel z={z_center:.3f} zemin seviyesinde olmalı"

    def test_center_pixel_hits_ground(self):
        """Orta pikselin beklenen derinliğinde zemin (z≈0) görülmeli."""
        depth_val = ground_depth_at_center()
        depth = np.full((H, W), depth_val, dtype=np.float32)
        pts = depth_to_world_points(depth, FX, FX, CX, CY, self._cam_to_world())
        z_center = pts[H // 2, W // 2, 2]
        np.testing.assert_allclose(z_center, 0.0, atol=0.05)

    def test_invalid_depth_gives_nan(self):
        depth = np.zeros((H, W), dtype=np.float32)
        pts = depth_to_world_points(depth, FX, FX, CX, CY, self._cam_to_world())
        assert np.all(np.isnan(pts[:, :, 0])), "Sıfır derinlik → NaN olmalı"

    def test_bottom_pixel_closer_on_ground(self):
        """
        Alt piksel, orta pikselden zemine daha yakın bir noktayı görür.
        (x_world küçük olmalı)
        """
        d_center = ground_depth_at_center()
        depth = np.full((H, W), d_center, dtype=np.float32)
        T = self._cam_to_world()
        pts = depth_to_world_points(depth, FX, FX, CX, CY, T)
        x_center = pts[H // 2, W // 2, 0]
        x_bottom = pts[H - 1, W // 2, 0]
        assert x_bottom < x_center, "Alt piksel zemine daha yakın noktayı görür"


# ---------------------------------------------------------------------------
# Testler — etiket üretimi
# ---------------------------------------------------------------------------

class TestGenerateLabels:
    def _cfg(self):
        return make_scene_cfg()

    def _ground_depth(self) -> np.ndarray:
        """Tüm pikseller için yaklaşık zemin derinliği."""
        T = build_cam_to_world(CAM_POS, PITCH_RAD)
        R = T[:3, :3]
        rows, cols = np.mgrid[0:H, 0:W]
        fx = FX
        rays_cam = np.stack([
            (cols - CX) / fx,
            (rows - CY) / fx,
            np.ones((H, W)),
        ], axis=-1).astype(np.float64)
        # Dünya yönü
        rays_world = (R @ rays_cam.reshape(-1, 3).T).T.reshape(H, W, 3)
        # t: z=0 çarpışması
        z_dir = rays_world[:, :, 2]
        z_dir_safe = np.where(np.abs(z_dir) < 1e-6, np.nan, z_dir)
        t = -CAM_POS[2] / z_dir_safe
        depth = np.where(t > 0, t, np.nan).astype(np.float32)
        depth = np.nan_to_num(depth, nan=0.0)
        return depth

    def test_pure_ground_gives_safe(self):
        """Düz zemin gören derinlik → tüm pikseller SAFE."""
        depth = self._ground_depth()
        risk, edge = generate_labels(depth, self._cfg())
        n_drop = int(np.sum(risk == 3))
        assert n_drop == 0, f"Düz zeminde {n_drop} UNSAFE_DROP piksel var"

    def test_deeper_depth_gives_drop(self):
        """Beklenen zemin derinliğinin %50 fazlası → drop bölgesi oluşur."""
        depth = self._ground_depth() * 1.5
        depth[depth == 0] = 0  # geçersizleri koru
        risk, edge = generate_labels(depth, self._cfg())
        n_drop = int(np.sum(risk == 3))
        assert n_drop > 0, "Derin derinlik drop piksel üretmeli"

    def test_zero_depth_gives_uncertain(self):
        """Sıfır derinlik → UNCERTAIN."""
        depth = np.zeros((H, W), dtype=np.float32)
        risk, edge = generate_labels(depth, self._cfg())
        n_uncertain = int(np.sum(risk == 1))
        assert n_uncertain == H * W

    def test_output_shapes(self):
        depth = self._ground_depth()
        risk, edge = generate_labels(depth, self._cfg())
        assert risk.shape == (H, W)
        assert edge.shape == (H, W)

    def test_risk_mask_values(self):
        """risk_mask yalnızca {0, 1, 2, 3} içermeli."""
        depth = self._ground_depth() * 1.3
        risk, _ = generate_labels(depth, self._cfg())
        unique = set(np.unique(risk).tolist())
        assert unique.issubset({0, 1, 2, 3})

    def test_edge_mask_binary(self):
        """edge_mask yalnızca 0 veya 255 içermeli."""
        depth = self._ground_depth() * 1.5
        _, edge = generate_labels(depth, self._cfg())
        unique = set(np.unique(edge).tolist())
        assert unique.issubset({0, 255})

    def test_pit_scene_depth_produces_drop(self):
        """
        Pit alanında (2m önde) pit tabanı derinliği → UNSAFE_DROP.
        Pit tabanı: z=-0.5m. Kamera z=0.45m.
        Beklenen derinlik zeminden ~0.5m daha derin.
        """
        T = build_cam_to_world(CAM_POS, PITCH_RAD)
        R = T[:3, :3]
        # Orta piksel için pit tabanı derinliği hesapla
        # Optik eksen: R[:,2]
        opt_axis = R[:, 2]
        # t: z = -0.5 → cam_z + t*opt_z = -0.5
        opt_z = opt_axis[2]
        if abs(opt_z) < 1e-6:
            pytest.skip("Optik eksen zemine paralel")
        t_pit = (-0.5 - CAM_POS[2]) / opt_z
        if t_pit <= 0:
            pytest.skip("Pit kameranın arkasında")

        depth = np.full((H, W), float(t_pit), dtype=np.float32)
        risk, _ = generate_labels(depth, self._cfg())
        n_drop = int(np.sum(risk == 3))
        assert n_drop > 0, f"Pit derinliğinde drop üretilmeli, ama {n_drop} piksel"


# ---------------------------------------------------------------------------
# Testler — process_sample (disk IO)
# ---------------------------------------------------------------------------

class TestProcessSample:
    def test_creates_output_files(self, tmp_path):
        """process_sample risk_mask.png, edge_mask.png ve meta.json üretmeli."""
        depth = np.full((H, W), 1.5, dtype=np.float32)
        depth_path = tmp_path / "test_0000_depth.npy"
        np.save(str(depth_path), depth)

        cfg = make_scene_cfg()
        meta = process_sample(depth_path, cfg, tmp_path / "out", "test_0000")

        assert (tmp_path / "out" / "labels" / "test_0000_risk_mask.png").exists()
        assert (tmp_path / "out" / "labels" / "test_0000_edge_mask.png").exists()
        assert (tmp_path / "out" / "raw"    / "test_0000_meta.json").exists()
        assert (tmp_path / "out" / "raw"    / "test_0000_depth.npy").exists()

    def test_meta_fields(self, tmp_path):
        depth = np.ones((H, W), dtype=np.float32)
        depth_path = tmp_path / "s_depth.npy"
        np.save(str(depth_path), depth)

        cfg = make_scene_cfg()
        meta = process_sample(depth_path, cfg, tmp_path / "out", "s")

        assert meta["source"] == "gazebo"
        assert meta["image_width"] == W
        assert meta["image_height"] == H
        assert "drop_pixel_ratio" in meta
        assert 0.0 <= meta["drop_pixel_ratio"] <= 1.0
