"""
Gazebo'dan bağımsız analitik sentetik depth image üreticisi.

Her sahne tipi için depth.npy, risk_mask.png, edge_mask.png,
meta.json ve overlay.png üretir.
"""

from __future__ import annotations

import argparse
import json
import math
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Optional

import cv2
import numpy as np


# ---------------------------------------------------------------------------
# Risk label constants
# ---------------------------------------------------------------------------
SAFE = 0
UNCERTAIN = 1
UNSAFE_SOLID = 2
UNSAFE_DROP = 3

RISK_COLORS_BGR = {
    SAFE:         (0,   200,   0),   # green
    UNCERTAIN:    (0,   165, 255),   # orange
    UNSAFE_SOLID: (0,     0, 255),   # red
    UNSAFE_DROP:  (255,   0, 128),   # magenta
}

HARD_NEGATIVE_SCENE_TYPES = {
    "ramp_hard_negative",
    "shadow_hard_negative",
    "dark_floor_hard_negative",
    "reflective_floor_hard_negative",
    "uneven_ground_hard_negative",
    "sloped_ground_hard_negative",
}

ALL_SCENE_TYPES = [
    "rectangular_pit",
    "circular_pit",
    "platform_edge",
    "trench",
    "stairs_descent",
    "curb_drop",
    "ramp_hard_negative",
    "shadow_hard_negative",
    "dark_floor_hard_negative",
    "reflective_floor_hard_negative",
    "uneven_ground_hard_negative",
    "sloped_ground_hard_negative",
]


# ---------------------------------------------------------------------------
# Scene parameter dataclass
# ---------------------------------------------------------------------------
@dataclass
class SceneParams:
    scene_type: str
    image_width: int
    image_height: int
    camera_height_m: float
    camera_pitch_deg: float
    ground_slope_deg: float
    drop_depth_m: float
    drop_width_m: float
    noise_std: float
    missing_depth_ratio: float
    seed: int
    scene_id: str
    sample_id: str

    def to_meta_dict(self) -> dict:
        d = asdict(self)
        d["has_negative_obstacle"] = self.scene_type not in HARD_NEGATIVE_SCENE_TYPES
        d["risk_labels"] = {
            "SAFE": SAFE,
            "UNCERTAIN": UNCERTAIN,
            "UNSAFE_SOLID": UNSAFE_SOLID,
            "UNSAFE_DROP": UNSAFE_DROP,
        }
        return d


# ---------------------------------------------------------------------------
# Depth generation helpers
# ---------------------------------------------------------------------------

def _flat_ground_depth(
    height: int,
    width: int,
    camera_height_m: float,
    camera_pitch_deg: float,
    ground_slope_deg: float,
    fx: float,
    fy: float,
    cx: float,
    cy: float,
) -> np.ndarray:
    """Return per-pixel depth for a flat (possibly sloped) ground plane."""
    pitch_rad = math.radians(camera_pitch_deg)
    slope_rad = math.radians(ground_slope_deg)

    rows, cols = np.meshgrid(np.arange(height), np.arange(width), indexing="ij")
    # Ray direction in camera frame
    ray_y = (rows - cy) / fy
    ray_z = np.ones((height, width), dtype=np.float32)

    # Camera optical axis tilted by pitch_rad downward
    # Ground normal (world up = [0,1,0]) in camera frame after pitch rotation
    # depth = camera_height_m / (sin(pitch) + ray_y * cos(pitch))  (simplified)
    denom = math.sin(-pitch_rad) + ray_y * math.cos(-pitch_rad)
    denom = np.where(np.abs(denom) < 1e-6, 1e-6, denom)
    depth = camera_height_m / denom
    depth = np.where(depth > 0, depth, np.nan)

    # Apply mild forward slope: closer rows get slightly different depth
    x_world = (cols - cx) / fx * depth
    slope_correction = np.tan(slope_rad) * x_world
    depth = depth + slope_correction.astype(np.float32)

    return depth.astype(np.float32)


def _camera_intrinsics(width: int, height: int) -> tuple[float, float, float, float]:
    """Return (fx, fy, cx, cy) for a typical RGB-D camera FOV."""
    fov_h = math.radians(70.0)
    fx = width / (2 * math.tan(fov_h / 2))
    fy = fx
    cx = width / 2.0
    cy = height / 2.0
    return fx, fy, cx, cy


def _add_noise(
    depth: np.ndarray,
    rng: np.random.Generator,
    noise_std: float,
    missing_ratio: float,
) -> np.ndarray:
    """Add Gaussian noise and random missing pixels."""
    noisy = depth + rng.normal(0, noise_std, depth.shape).astype(np.float32)
    mask_missing = rng.random(depth.shape) < missing_ratio
    noisy[mask_missing] = np.nan
    return noisy


def _depth_to_uint16(depth: np.ndarray, max_depth: float = 5.0) -> np.ndarray:
    """Convert float depth [m] to uint16 [0..65535]."""
    d = np.clip(depth, 0, max_depth)
    d = np.nan_to_num(d, nan=0.0)
    return (d / max_depth * 65535).astype(np.uint16)


# ---------------------------------------------------------------------------
# Risk mask helpers
# ---------------------------------------------------------------------------

def _edge_from_drop_mask(drop_mask: np.ndarray, dilation: int = 2) -> np.ndarray:
    """Return binary edge mask (boundary of drop region)."""
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * dilation + 1, 2 * dilation + 1))
    dilated = cv2.dilate(drop_mask.astype(np.uint8), kernel)
    eroded = cv2.erode(drop_mask.astype(np.uint8), kernel)
    edge = (dilated - eroded).astype(np.uint8)
    return edge


# ---------------------------------------------------------------------------
# Scene generators
# ---------------------------------------------------------------------------

def _generate_rectangular_pit(params: SceneParams, rng: np.random.Generator) -> tuple:
    h, w = params.image_height, params.image_width
    fx, fy, cx, cy = _camera_intrinsics(w, h)

    base_depth = _flat_ground_depth(h, w, params.camera_height_m, params.camera_pitch_deg,
                                    params.ground_slope_deg, fx, fy, cx, cy)

    # Pit region: centered rectangle in lower half of image
    pit_row_start = int(h * 0.35)
    pit_row_end = int(h * 0.65)
    pit_col_start = int(w * (0.5 - params.drop_width_m / 2.4))
    pit_col_end = int(w * (0.5 + params.drop_width_m / 2.4))
    pit_col_start = max(0, pit_col_start)
    pit_col_end = min(w, pit_col_end)

    depth = base_depth.copy()
    depth[pit_row_start:pit_row_end, pit_col_start:pit_col_end] += params.drop_depth_m

    risk_mask = np.full((h, w), SAFE, dtype=np.uint8)
    drop_mask = np.zeros((h, w), dtype=bool)
    drop_mask[pit_row_start:pit_row_end, pit_col_start:pit_col_end] = True
    risk_mask[drop_mask] = UNSAFE_DROP

    # Uncertain band around pit
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
    uncertain_zone = cv2.dilate(drop_mask.astype(np.uint8), kernel).astype(bool)
    uncertain_only = uncertain_zone & ~drop_mask
    risk_mask[uncertain_only] = UNCERTAIN

    edge_mask = _edge_from_drop_mask(drop_mask)
    depth = _add_noise(depth, rng, params.noise_std, params.missing_depth_ratio)
    return depth, risk_mask, edge_mask


def _generate_circular_pit(params: SceneParams, rng: np.random.Generator) -> tuple:
    h, w = params.image_height, params.image_width
    fx, fy, cx, cy = _camera_intrinsics(w, h)

    base_depth = _flat_ground_depth(h, w, params.camera_height_m, params.camera_pitch_deg,
                                    params.ground_slope_deg, fx, fy, cx, cy)

    center_r = int(h * 0.50)
    center_c = int(w * 0.50)
    radius_px = int(min(h, w) * params.drop_width_m / 3.0)

    rows, cols = np.meshgrid(np.arange(h), np.arange(w), indexing="ij")
    dist = np.sqrt((rows - center_r) ** 2 + (cols - center_c) ** 2)
    drop_mask = dist < radius_px

    depth = base_depth.copy()
    depth[drop_mask] += params.drop_depth_m

    risk_mask = np.full((h, w), SAFE, dtype=np.uint8)
    risk_mask[drop_mask] = UNSAFE_DROP

    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
    uncertain_zone = cv2.dilate(drop_mask.astype(np.uint8), kernel).astype(bool)
    risk_mask[uncertain_zone & ~drop_mask] = UNCERTAIN

    edge_mask = _edge_from_drop_mask(drop_mask)
    depth = _add_noise(depth, rng, params.noise_std, params.missing_depth_ratio)
    return depth, risk_mask, edge_mask


def _generate_platform_edge(params: SceneParams, rng: np.random.Generator) -> tuple:
    h, w = params.image_height, params.image_width
    fx, fy, cx, cy = _camera_intrinsics(w, h)

    base_depth = _flat_ground_depth(h, w, params.camera_height_m, params.camera_pitch_deg,
                                    params.ground_slope_deg, fx, fy, cx, cy)

    edge_row = int(h * 0.45)
    depth = base_depth.copy()
    depth[edge_row:, :] += params.drop_depth_m

    risk_mask = np.full((h, w), SAFE, dtype=np.uint8)
    drop_mask = np.zeros((h, w), dtype=bool)
    drop_mask[edge_row:, :] = True
    risk_mask[drop_mask] = UNSAFE_DROP

    edge_mask = _edge_from_drop_mask(drop_mask)
    depth = _add_noise(depth, rng, params.noise_std, params.missing_depth_ratio)
    return depth, risk_mask, edge_mask


def _generate_trench(params: SceneParams, rng: np.random.Generator) -> tuple:
    h, w = params.image_height, params.image_width
    fx, fy, cx, cy = _camera_intrinsics(w, h)

    base_depth = _flat_ground_depth(h, w, params.camera_height_m, params.camera_pitch_deg,
                                    params.ground_slope_deg, fx, fy, cx, cy)

    trench_col_start = int(w * (0.5 - params.drop_width_m / 3.0))
    trench_col_end = int(w * (0.5 + params.drop_width_m / 3.0))
    trench_col_start = max(0, trench_col_start)
    trench_col_end = min(w, trench_col_end)

    depth = base_depth.copy()
    drop_mask = np.zeros((h, w), dtype=bool)
    drop_mask[:, trench_col_start:trench_col_end] = True
    depth[drop_mask] += params.drop_depth_m

    risk_mask = np.full((h, w), SAFE, dtype=np.uint8)
    risk_mask[drop_mask] = UNSAFE_DROP

    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
    uncertain_zone = cv2.dilate(drop_mask.astype(np.uint8), kernel).astype(bool)
    risk_mask[uncertain_zone & ~drop_mask] = UNCERTAIN

    edge_mask = _edge_from_drop_mask(drop_mask)
    depth = _add_noise(depth, rng, params.noise_std, params.missing_depth_ratio)
    return depth, risk_mask, edge_mask


def _generate_stairs_descent(params: SceneParams, rng: np.random.Generator) -> tuple:
    h, w = params.image_height, params.image_width
    fx, fy, cx, cy = _camera_intrinsics(w, h)

    base_depth = _flat_ground_depth(h, w, params.camera_height_m, params.camera_pitch_deg,
                                    params.ground_slope_deg, fx, fy, cx, cy)

    depth = base_depth.copy()
    risk_mask = np.full((h, w), SAFE, dtype=np.uint8)
    drop_mask = np.zeros((h, w), dtype=bool)

    n_steps = 4
    step_height = params.drop_depth_m / n_steps
    step_row_span = h // (n_steps + 2)

    for i in range(n_steps):
        row_start = int(h * 0.3) + i * step_row_span
        row_end = row_start + step_row_span
        row_end = min(h, row_end)
        depth[row_start:row_end, :] += step_height * (i + 1)
        drop_mask[row_start:row_end, :] = True

    risk_mask[drop_mask] = UNSAFE_DROP

    edge_mask = _edge_from_drop_mask(drop_mask)
    depth = _add_noise(depth, rng, params.noise_std, params.missing_depth_ratio)
    return depth, risk_mask, edge_mask


def _generate_curb_drop(params: SceneParams, rng: np.random.Generator) -> tuple:
    h, w = params.image_height, params.image_width
    fx, fy, cx, cy = _camera_intrinsics(w, h)

    base_depth = _flat_ground_depth(h, w, params.camera_height_m, params.camera_pitch_deg,
                                    params.ground_slope_deg, fx, fy, cx, cy)

    curb_row = int(h * 0.50)
    col_start = int(w * 0.20)
    col_end = int(w * 0.80)

    depth = base_depth.copy()
    drop_mask = np.zeros((h, w), dtype=bool)
    drop_mask[curb_row:, col_start:col_end] = True
    depth[drop_mask] += params.drop_depth_m

    risk_mask = np.full((h, w), SAFE, dtype=np.uint8)
    risk_mask[drop_mask] = UNSAFE_DROP

    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    uncertain_zone = cv2.dilate(drop_mask.astype(np.uint8), kernel).astype(bool)
    risk_mask[uncertain_zone & ~drop_mask] = UNCERTAIN

    edge_mask = _edge_from_drop_mask(drop_mask)
    depth = _add_noise(depth, rng, params.noise_std, params.missing_depth_ratio)
    return depth, risk_mask, edge_mask


# ---------------------------------------------------------------------------
# Hard-negative scene generators (no UNSAFE_DROP label)
# ---------------------------------------------------------------------------

def _generate_ramp_hard_negative(params: SceneParams, rng: np.random.Generator) -> tuple:
    h, w = params.image_height, params.image_width
    fx, fy, cx, cy = _camera_intrinsics(w, h)

    # Ramp: gradual depth increase — no real drop
    base_depth = _flat_ground_depth(h, w, params.camera_height_m, params.camera_pitch_deg,
                                    0.0, fx, fy, cx, cy)
    rows = np.arange(h).reshape(h, 1)
    ramp_gradient = (rows / h) * 0.40
    depth = base_depth + ramp_gradient.astype(np.float32)
    depth = _add_noise(depth, rng, params.noise_std, params.missing_depth_ratio)

    risk_mask = np.full((h, w), SAFE, dtype=np.uint8)
    edge_mask = np.zeros((h, w), dtype=np.uint8)
    return depth, risk_mask, edge_mask


def _generate_shadow_hard_negative(params: SceneParams, rng: np.random.Generator) -> tuple:
    h, w = params.image_height, params.image_width
    fx, fy, cx, cy = _camera_intrinsics(w, h)

    base_depth = _flat_ground_depth(h, w, params.camera_height_m, params.camera_pitch_deg,
                                    params.ground_slope_deg, fx, fy, cx, cy)
    # Shadow = NaN patch (missing depth), not a real drop
    shadow_r0, shadow_r1 = int(h * 0.4), int(h * 0.6)
    shadow_c0, shadow_c1 = int(w * 0.3), int(w * 0.7)
    depth = _add_noise(base_depth, rng, params.noise_std, params.missing_depth_ratio)
    depth[shadow_r0:shadow_r1, shadow_c0:shadow_c1] = np.nan

    risk_mask = np.full((h, w), SAFE, dtype=np.uint8)
    risk_mask[shadow_r0:shadow_r1, shadow_c0:shadow_c1] = UNCERTAIN
    edge_mask = np.zeros((h, w), dtype=np.uint8)
    return depth, risk_mask, edge_mask


def _generate_dark_floor_hard_negative(params: SceneParams, rng: np.random.Generator) -> tuple:
    h, w = params.image_height, params.image_width
    fx, fy, cx, cy = _camera_intrinsics(w, h)

    base_depth = _flat_ground_depth(h, w, params.camera_height_m, params.camera_pitch_deg,
                                    params.ground_slope_deg, fx, fy, cx, cy)
    # Dark floor: higher noise + missing ratio in a patch
    depth = _add_noise(base_depth, rng, params.noise_std * 3, params.missing_depth_ratio * 4)

    risk_mask = np.full((h, w), SAFE, dtype=np.uint8)
    edge_mask = np.zeros((h, w), dtype=np.uint8)
    return depth, risk_mask, edge_mask


def _generate_reflective_floor_hard_negative(params: SceneParams, rng: np.random.Generator) -> tuple:
    h, w = params.image_height, params.image_width
    fx, fy, cx, cy = _camera_intrinsics(w, h)

    base_depth = _flat_ground_depth(h, w, params.camera_height_m, params.camera_pitch_deg,
                                    params.ground_slope_deg, fx, fy, cx, cy)
    # Reflective: random depth spikes in patches
    depth = _add_noise(base_depth, rng, params.noise_std, params.missing_depth_ratio)
    spike_mask = rng.random((h, w)) < 0.05
    depth[spike_mask] = depth[spike_mask] + rng.uniform(0.1, 0.5, size=spike_mask.sum()).astype(np.float32)

    risk_mask = np.full((h, w), SAFE, dtype=np.uint8)
    edge_mask = np.zeros((h, w), dtype=np.uint8)
    return depth, risk_mask, edge_mask


def _generate_uneven_ground_hard_negative(params: SceneParams, rng: np.random.Generator) -> tuple:
    h, w = params.image_height, params.image_width
    fx, fy, cx, cy = _camera_intrinsics(w, h)

    base_depth = _flat_ground_depth(h, w, params.camera_height_m, params.camera_pitch_deg,
                                    params.ground_slope_deg, fx, fy, cx, cy)
    # Low-frequency bumps using sine waves
    rows, cols = np.meshgrid(np.arange(h), np.arange(w), indexing="ij")
    bumps = (
        0.04 * np.sin(2 * np.pi * rows / (h * 0.3))
        + 0.03 * np.sin(2 * np.pi * cols / (w * 0.4))
    ).astype(np.float32)
    depth = _add_noise(base_depth + bumps, rng, params.noise_std, params.missing_depth_ratio)

    risk_mask = np.full((h, w), SAFE, dtype=np.uint8)
    edge_mask = np.zeros((h, w), dtype=np.uint8)
    return depth, risk_mask, edge_mask


def _generate_sloped_ground_hard_negative(params: SceneParams, rng: np.random.Generator) -> tuple:
    h, w = params.image_height, params.image_width
    fx, fy, cx, cy = _camera_intrinsics(w, h)

    slope_deg = rng.uniform(8.0, 20.0)
    base_depth = _flat_ground_depth(h, w, params.camera_height_m, params.camera_pitch_deg,
                                    float(slope_deg), fx, fy, cx, cy)
    depth = _add_noise(base_depth, rng, params.noise_std, params.missing_depth_ratio)

    risk_mask = np.full((h, w), SAFE, dtype=np.uint8)
    edge_mask = np.zeros((h, w), dtype=np.uint8)
    return depth, risk_mask, edge_mask


# ---------------------------------------------------------------------------
# Scene dispatcher
# ---------------------------------------------------------------------------
_SCENE_GENERATORS = {
    "rectangular_pit":            _generate_rectangular_pit,
    "circular_pit":               _generate_circular_pit,
    "platform_edge":              _generate_platform_edge,
    "trench":                     _generate_trench,
    "stairs_descent":             _generate_stairs_descent,
    "curb_drop":                  _generate_curb_drop,
    "ramp_hard_negative":         _generate_ramp_hard_negative,
    "shadow_hard_negative":       _generate_shadow_hard_negative,
    "dark_floor_hard_negative":   _generate_dark_floor_hard_negative,
    "reflective_floor_hard_negative": _generate_reflective_floor_hard_negative,
    "uneven_ground_hard_negative":    _generate_uneven_ground_hard_negative,
    "sloped_ground_hard_negative":    _generate_sloped_ground_hard_negative,
}


# ---------------------------------------------------------------------------
# Overlay / QC image
# ---------------------------------------------------------------------------

def _make_overlay(depth: np.ndarray, risk_mask: np.ndarray) -> np.ndarray:
    """Create a color overlay of risk_mask on top of a depth-grayscale background."""
    depth_valid = np.nan_to_num(depth, nan=0.0)
    dmax = depth_valid.max()
    dmin = depth_valid.min()
    if dmax > dmin:
        depth_norm = ((depth_valid - dmin) / (dmax - dmin) * 255).astype(np.uint8)
    else:
        depth_norm = np.zeros_like(depth_valid, dtype=np.uint8)

    bg = cv2.cvtColor(depth_norm, cv2.COLOR_GRAY2BGR)
    overlay = bg.copy()

    for label, color_bgr in RISK_COLORS_BGR.items():
        mask = risk_mask == label
        if np.any(mask) and label != SAFE:
            overlay[mask] = color_bgr

    blended = cv2.addWeighted(bg, 0.5, overlay, 0.5, 0)
    return blended


# ---------------------------------------------------------------------------
# Main sample generation
# ---------------------------------------------------------------------------

def _randomize_params(
    scene_type: str,
    width: int,
    height: int,
    rng: np.random.Generator,
    seed: int,
    scene_id: str,
    sample_id: str,
) -> SceneParams:
    return SceneParams(
        scene_type=scene_type,
        image_width=width,
        image_height=height,
        camera_height_m=float(rng.uniform(0.35, 0.60)),
        camera_pitch_deg=float(rng.uniform(-35.0, -15.0)),
        ground_slope_deg=float(rng.uniform(-3.0, 3.0)),
        drop_depth_m=float(rng.uniform(0.20, 0.80)),
        drop_width_m=float(rng.uniform(0.30, 1.20)),
        noise_std=float(rng.uniform(0.005, 0.025)),
        missing_depth_ratio=float(rng.uniform(0.01, 0.05)),
        seed=seed,
        scene_id=scene_id,
        sample_id=sample_id,
    )


def generate_sample(
    scene_type: str,
    sample_index: int,
    output_dir: Path,
    seed: int,
    width: int,
    height: int,
    scene_seed_offset: int = 0,
) -> Path:
    """Generate one synthetic sample and write all output files.

    Returns the stem path (scene_id_sample_id) used for all files.
    """
    sample_seed = seed + scene_seed_offset + sample_index
    rng = np.random.default_rng(sample_seed)

    scene_id = f"{scene_type}_seed{seed:04d}_{sample_index:04d}"
    sample_id = f"{sample_index:04d}"

    params = _randomize_params(
        scene_type=scene_type,
        width=width,
        height=height,
        rng=rng,
        seed=sample_seed,
        scene_id=scene_id,
        sample_id=sample_id,
    )

    generator_fn = _SCENE_GENERATORS[scene_type]
    depth, risk_mask, edge_mask = generator_fn(params, rng)

    # Output directories
    raw_dir = output_dir / "raw"
    labels_dir = output_dir / "labels"
    processed_dir = output_dir / "processed"
    for d in (raw_dir, labels_dir, processed_dir):
        d.mkdir(parents=True, exist_ok=True)

    stem = scene_id

    # Save depth as npy
    np.save(str(raw_dir / f"{stem}_depth.npy"), depth)

    # Save risk_mask as PNG (single-channel, label values 0-3)
    cv2.imwrite(str(labels_dir / f"{stem}_risk_mask.png"), risk_mask)

    # Save edge_mask as PNG
    cv2.imwrite(str(labels_dir / f"{stem}_edge_mask.png"), edge_mask * 255)

    # Save meta.json
    meta = params.to_meta_dict()
    with open(raw_dir / f"{stem}_meta.json", "w") as f:
        json.dump(meta, f, indent=2)

    # Save overlay
    overlay = _make_overlay(depth, risk_mask)
    cv2.imwrite(str(processed_dir / f"{stem}_overlay.png"), overlay)

    return stem


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Gazebo'dan bağımsız analitik sentetik depth dataset üreticisi."
    )
    parser.add_argument(
        "--scene-type",
        required=True,
        choices=ALL_SCENE_TYPES,
        help="Üretilecek sahne tipi.",
    )
    parser.add_argument(
        "--count",
        type=int,
        default=20,
        help="Üretilecek örnek sayısı.",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("data"),
        help="Çıktı kök klasörü (varsayılan: data).",
    )
    parser.add_argument(
        "--seed",
        type=int,
        default=42,
        help="Rastgele sayı üreteci seed değeri.",
    )
    parser.add_argument(
        "--width",
        type=int,
        default=320,
        help="Görüntü genişliği (piksel).",
    )
    parser.add_argument(
        "--height",
        type=int,
        default=240,
        help="Görüntü yüksekliği (piksel).",
    )
    return parser.parse_args()


def main() -> None:
    args = _parse_args()

    print(f"Sahne tipi : {args.scene_type}")
    print(f"Örnek sayısı: {args.count}")
    print(f"Çıktı klasörü: {args.output_dir}")
    print(f"Seed: {args.seed}")
    print(f"Boyut: {args.width}x{args.height}")
    print("-" * 40)

    for i in range(args.count):
        stem = generate_sample(
            scene_type=args.scene_type,
            sample_index=i,
            output_dir=args.output_dir,
            seed=args.seed,
            width=args.width,
            height=args.height,
        )
        print(f"  [{i+1:3d}/{args.count}] {stem}")

    print("-" * 40)
    print(f"Tamamlandi. {args.count} ornek uretildi.")
    print(f"  data/raw/       -> depth.npy, meta.json")
    print(f"  data/labels/    -> risk_mask.png, edge_mask.png")
    print(f"  data/processed/ -> overlay.png")


if __name__ == "__main__":
    main()
