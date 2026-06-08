"""
synthetic_depth_generator için temel pytest testleri.
Geçici klasör kullanır; gerçek data/ klasörüne yazmaz.
"""

import json
import sys
from pathlib import Path

import cv2
import numpy as np
import pytest

# Paketi import edebilmek için src yolunu ekle
sys.path.insert(0, str(Path(__file__).parent.parent))

from negative_obstacle_data.synthetic_depth_generator import (
    UNSAFE_DROP,
    generate_sample,
)


@pytest.fixture()
def tmp_data(tmp_path: Path) -> Path:
    return tmp_path


def _gen(scene_type: str, tmp_data: Path, seed: int = 42) -> tuple[Path, Path, Path, Path]:
    """Tek bir örnek üret; (depth_path, risk_mask_path, edge_mask_path, meta_path) döndür."""
    stem = generate_sample(
        scene_type=scene_type,
        sample_index=0,
        output_dir=tmp_data,
        seed=seed,
        width=320,
        height=240,
    )
    raw_dir = tmp_data / "raw"
    labels_dir = tmp_data / "labels"
    return (
        raw_dir / f"{stem}_depth.npy",
        labels_dir / f"{stem}_risk_mask.png",
        labels_dir / f"{stem}_edge_mask.png",
        raw_dir / f"{stem}_meta.json",
    )


class TestRectangularPit:
    def test_unsafe_drop_label_exists(self, tmp_data: Path) -> None:
        _, risk_mask_path, _, _ = _gen("rectangular_pit", tmp_data)
        mask = cv2.imread(str(risk_mask_path), cv2.IMREAD_GRAYSCALE)
        assert mask is not None, "risk_mask.png okunamadi"
        assert np.any(mask == UNSAFE_DROP), "rectangular_pit icin UNSAFE_DROP etiketi bekleniyor"

    def test_depth_shape(self, tmp_data: Path) -> None:
        depth_path, _, _, _ = _gen("rectangular_pit", tmp_data)
        depth = np.load(str(depth_path))
        assert depth.shape == (240, 320), f"Beklenmeyen depth shape: {depth.shape}"

    def test_meta_required_fields(self, tmp_data: Path) -> None:
        _, _, _, meta_path = _gen("rectangular_pit", tmp_data)
        with open(meta_path) as f:
            meta = json.load(f)
        for field in ("scene_id", "scene_type", "has_negative_obstacle"):
            assert field in meta, f"meta.json'da eksik alan: {field}"
        assert meta["has_negative_obstacle"] is True
        assert meta["scene_type"] == "rectangular_pit"


class TestCircularPit:
    def test_edge_mask_not_empty(self, tmp_data: Path) -> None:
        _, _, edge_mask_path, _ = _gen("circular_pit", tmp_data)
        edge = cv2.imread(str(edge_mask_path), cv2.IMREAD_GRAYSCALE)
        assert edge is not None, "edge_mask.png okunamadi"
        assert np.any(edge > 0), "circular_pit icin edge_mask bos olmamali"


class TestHardNegative:
    @pytest.mark.parametrize("scene_type", [
        "ramp_hard_negative",
        "shadow_hard_negative",
        "dark_floor_hard_negative",
        "reflective_floor_hard_negative",
        "uneven_ground_hard_negative",
        "sloped_ground_hard_negative",
    ])
    def test_no_unsafe_drop_label(self, scene_type: str, tmp_data: Path) -> None:
        _, risk_mask_path, _, _ = _gen(scene_type, tmp_data)
        mask = cv2.imread(str(risk_mask_path), cv2.IMREAD_GRAYSCALE)
        assert mask is not None
        assert not np.any(mask == UNSAFE_DROP), (
            f"{scene_type} hard-negative sahnede UNSAFE_DROP etiketi olmamali"
        )

    def test_ramp_meta_has_negative_obstacle_false(self, tmp_data: Path) -> None:
        _, _, _, meta_path = _gen("ramp_hard_negative", tmp_data)
        with open(meta_path) as f:
            meta = json.load(f)
        assert meta["has_negative_obstacle"] is False


class TestOutputFiles:
    def test_all_files_created(self, tmp_data: Path) -> None:
        stem = generate_sample(
            scene_type="trench",
            sample_index=0,
            output_dir=tmp_data,
            seed=77,
            width=320,
            height=240,
        )
        raw_dir = tmp_data / "raw"
        labels_dir = tmp_data / "labels"
        processed_dir = tmp_data / "processed"

        assert (raw_dir / f"{stem}_depth.npy").exists()
        assert (raw_dir / f"{stem}_meta.json").exists()
        assert (labels_dir / f"{stem}_risk_mask.png").exists()
        assert (labels_dir / f"{stem}_edge_mask.png").exists()
        assert (processed_dir / f"{stem}_overlay.png").exists()

    def test_depth_dtype_float32(self, tmp_data: Path) -> None:
        depth_path, _, _, _ = _gen("platform_edge", tmp_data)
        depth = np.load(str(depth_path))
        assert depth.dtype == np.float32, f"Beklenmeyen dtype: {depth.dtype}"

    def test_risk_mask_values_in_range(self, tmp_data: Path) -> None:
        _, risk_mask_path, _, _ = _gen("platform_edge", tmp_data)
        mask = cv2.imread(str(risk_mask_path), cv2.IMREAD_GRAYSCALE)
        unique_vals = set(np.unique(mask).tolist())
        valid_vals = {0, 1, 2, 3}
        assert unique_vals.issubset(valid_vals), f"Beklenmeyen label degerleri: {unique_vals}"
