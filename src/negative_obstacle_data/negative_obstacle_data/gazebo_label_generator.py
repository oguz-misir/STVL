"""
Gazebo derinlik görüntüleri için geometri tabanlı offline etiket üretici.

Kamera pozu ve sahne geometrisi (YAML) kullanılarak kaydedilmiş .npy
derinlik dosyalarına risk_mask ve edge_mask etiketleri üretir.

Kamera modeli:
  - Kamera optik ekseni: SDF link'inin +X yönü
  - SDF pitch ψ: pozitif ψ → link +X = [cos ψ, 0, -sin ψ] (ileri+aşağı) ✓
  - Dünya koordinatları: X ileri, Y sol, Z yukarı (Gazebo)

Kullanım:
  python3 gazebo_label_generator.py \\
      --input-dir /tmp/gazebo_depth \\
      --output-dir data \\
      --scene-config scene_geometry_config.yaml \\
      --scene-name pit_scene
"""

from __future__ import annotations

import argparse
import json
import math
import shutil
import sys
from pathlib import Path

import cv2
import numpy as np
import yaml


# ---------------------------------------------------------------------------
# Kamera modeli
# ---------------------------------------------------------------------------

def build_cam_to_world(
    cam_pos: list[float],
    pitch_sdf_rad: float,
) -> np.ndarray:
    """
    4×4 kamera→dünya dönüşüm matrisi döndürür.

    Kamera çerçevesi: Z ileri (optik), X sağ, Y aşağı (OpenCV)
    Dünya çerçevesi: X ileri, Y sol, Z yukarı (Gazebo)
    Kamera SDF link'inin +X yönüne bakıyor.

    SDF pitch ψ > 0 → link +X = [cos ψ, 0, -sin ψ] → ileri+aşağı ✓
    """
    psi = pitch_sdf_rad
    cos_p = math.cos(psi)
    sin_p = math.sin(psi)

    # Sütunlar: kamera eksenlerinin dünya çerçevesindeki karşılıkları
    # camera +X (resim sağı)  → dünya -Y  = [0, -1, 0]
    # camera +Y (resim aşağı) → [-sin ψ, 0, -cos ψ]
    # camera +Z (optik ileri) → [cos ψ,  0, -sin ψ]
    R = np.array([
        [0.0,    -sin_p,   cos_p ],
        [-1.0,    0.0,     0.0   ],
        [0.0,    -cos_p,  -sin_p ],
    ], dtype=np.float64)

    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = R
    T[:3, 3] = cam_pos
    return T


def depth_to_world_points(
    depth: np.ndarray,
    fx: float,
    fy: float,
    cx: float,
    cy: float,
    cam_to_world: np.ndarray,
) -> np.ndarray:
    """
    (H, W) derinlik dizisini (H, W, 3) dünya koordinatlarına dönüştürür.

    Geçersiz pikseller (depth <= 0 veya nan) [nan, nan, nan] olur.
    """
    H, W = depth.shape
    rows, cols = np.mgrid[0:H, 0:W]

    # Kamera çerçevesi — vektörize
    z = depth.astype(np.float64)
    x_cam = (cols - cx) * z / fx
    y_cam = (rows - cy) * z / fy

    # Homojen [x, y, z, 1]
    ones = np.ones_like(z)
    pts_cam = np.stack([x_cam, y_cam, z, ones], axis=-1)  # (H, W, 4)

    # Dünya çerçevesi
    pts_flat = pts_cam.reshape(-1, 4)
    pts_world = (cam_to_world @ pts_flat.T).T  # (N, 4)
    pts_world = pts_world[:, :3].reshape(H, W, 3)

    # Geçersiz pikselleri NaN yap
    invalid = (depth <= 0.0) | ~np.isfinite(depth)
    pts_world[invalid] = np.nan

    return pts_world


# ---------------------------------------------------------------------------
# Etiket üretimi
# ---------------------------------------------------------------------------

def generate_labels(
    depth: np.ndarray,
    scene_cfg: dict,
) -> tuple[np.ndarray, np.ndarray]:
    """
    Derinlik görüntüsü + sahne konfigürasyonundan risk_mask ve edge_mask üretir.

    Dönüş:
        risk_mask : uint8 (H, W)  — 0=SAFE, 1=UNCERTAIN, 3=UNSAFE_DROP
        edge_mask : uint8 (H, W)  — 0 veya 255
    """
    H, W = depth.shape
    cam_cfg = scene_cfg["camera"]

    cam_pos = cam_cfg["position_xyz"]
    pitch_rad = math.radians(cam_cfg["pitch_sdf_deg"])
    fov_h_deg = cam_cfg.get("fov_h_deg", 70.0)
    fov_h_rad = math.radians(fov_h_deg)

    fx = (W / 2.0) / math.tan(fov_h_rad / 2.0)
    fy = fx
    cx = W / 2.0
    cy = H / 2.0

    cam_to_world = build_cam_to_world(cam_pos, pitch_rad)
    world_pts = depth_to_world_points(depth, fx, fy, cx, cy, cam_to_world)

    ground_z: float = scene_cfg.get("ground_z", 0.0)
    drop_thr: float = scene_cfg.get("drop_detection_threshold_m", 0.15)

    # UNSAFE_DROP: z_dünya < zemin - eşik
    z_world = world_pts[:, :, 2]
    drop_region = np.isfinite(z_world) & (z_world < ground_z - drop_thr)

    # UNCERTAIN: ölçüm yok
    invalid = (depth <= 0.0) | ~np.isfinite(depth)

    risk_mask = np.zeros((H, W), dtype=np.uint8)
    risk_mask[drop_region] = 3   # UNSAFE_DROP
    risk_mask[invalid] = 1       # UNCERTAIN

    # Edge mask: drop bölgesinin sınırı
    drop_uint8 = (risk_mask == 3).astype(np.uint8) * 255
    k3 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    k5 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    dilated = cv2.dilate(drop_uint8, k5)
    eroded = cv2.erode(drop_uint8, k3)
    edge_mask = cv2.bitwise_and(dilated, cv2.bitwise_not(eroded))

    return risk_mask, edge_mask


# ---------------------------------------------------------------------------
# Dosya işleme
# ---------------------------------------------------------------------------

def process_sample(
    depth_path: Path,
    scene_cfg: dict,
    output_dir: Path,
    stem: str,
) -> dict:
    """Tek bir .npy derinlik dosyasını etiketler ve sonuçları kaydeder."""
    depth = np.load(str(depth_path)).astype(np.float32)

    risk_mask, edge_mask = generate_labels(depth, scene_cfg)

    labels_dir = output_dir / "labels"
    labels_dir.mkdir(parents=True, exist_ok=True)
    raw_dir = output_dir / "raw"
    raw_dir.mkdir(parents=True, exist_ok=True)

    cv2.imwrite(str(labels_dir / f"{stem}_risk_mask.png"), risk_mask)
    cv2.imwrite(str(labels_dir / f"{stem}_edge_mask.png"), edge_mask)

    # Derinlik dosyasını raw klasörüne kopyala (henüz orada değilse)
    dest_depth = raw_dir / f"{stem}_depth.npy"
    if not dest_depth.exists():
        shutil.copy(str(depth_path), str(dest_depth))

    cam = scene_cfg["camera"]
    meta = {
        "stem": stem,
        "scene_type": scene_cfg.get("scene_type", "unknown"),
        "source": "gazebo",
        "camera_height_m": cam["position_xyz"][2],
        "camera_pitch_sdf_deg": cam["pitch_sdf_deg"],
        "image_width": int(depth.shape[1]),
        "image_height": int(depth.shape[0]),
        "drop_pixel_count": int(np.sum(risk_mask == 3)),
        "safe_pixel_count": int(np.sum(risk_mask == 0)),
        "uncertain_pixel_count": int(np.sum(risk_mask == 1)),
        "drop_pixel_ratio": round(float(np.sum(risk_mask == 3)) / risk_mask.size, 4),
    }

    with open(raw_dir / f"{stem}_meta.json", "w") as f:
        json.dump(meta, f, indent=2, ensure_ascii=False)

    return meta


def collect_depth_files(input_dir: Path) -> list[Path]:
    return sorted(input_dir.glob("*_depth.npy"))


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def _parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Gazebo derinlik görüntülerini etiketler.")
    p.add_argument("--input-dir",    type=Path, required=True,
                   help="Kaydedilmiş *_depth.npy dosyalarının bulunduğu klasör")
    p.add_argument("--output-dir",   type=Path, default=Path("data"),
                   help="Çıktı kök klasörü (data/raw, data/labels oluşturulur)")
    p.add_argument("--scene-config", type=Path, required=True,
                   help="scene_geometry_config.yaml yolu")
    p.add_argument("--scene-name",   type=str,  required=True,
                   help="Konfigürasyondaki sahne adı (ör. pit_scene)")
    p.add_argument("--max-samples",  type=int,  default=1000)
    return p.parse_args()


def main() -> None:
    args = _parse_args()

    with open(args.scene_config) as f:
        cfg_all = yaml.safe_load(f)

    if args.scene_name not in cfg_all.get("scenes", {}):
        print(f"HATA: '{args.scene_name}' sahnesi konfigürasyonda bulunamadı.", file=sys.stderr)
        sys.exit(1)

    scene_cfg = cfg_all["scenes"][args.scene_name]
    depth_files = collect_depth_files(args.input_dir)[: args.max_samples]

    if not depth_files:
        print(f"UYARI: {args.input_dir} içinde *_depth.npy dosyası bulunamadı.")
        return

    print(f"Etiketleniyor: {len(depth_files)} dosya, sahne={args.scene_name}")
    print("-" * 50)

    results = []
    for dp in depth_files:
        stem = dp.name.replace("_depth.npy", "")
        meta = process_sample(dp, scene_cfg, args.output_dir, stem)
        drop_pct = meta["drop_pixel_ratio"] * 100
        print(f"  {stem}: drop={drop_pct:.1f}% safe={meta['safe_pixel_count']} pix")
        results.append(meta)

    print("-" * 50)
    avg_drop = float(np.mean([r["drop_pixel_ratio"] for r in results])) * 100
    print(f"Tamamlandı. Ortalama drop oranı: {avg_drop:.1f}%")
    print(f"Çıktı: {args.output_dir}/raw/ ve {args.output_dir}/labels/")


if __name__ == "__main__":
    main()
