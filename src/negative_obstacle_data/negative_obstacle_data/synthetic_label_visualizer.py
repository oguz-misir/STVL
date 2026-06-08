"""
Sentetik depth dataset QC görselleştirici.

data/raw/ ve data/labels/ altındaki örnekleri okuyarak
data/processed/ altına depth_viz.png, risk_overlay.png ve edge_overlay.png üretir.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import cv2
import numpy as np

# Risk label renkleri (BGR)
RISK_COLORS_BGR = {
    0: (0,   200,   0),   # SAFE     — yeşil
    1: (0,   165, 255),   # UNCERTAIN — turuncu
    2: (0,     0, 255),   # UNSAFE_SOLID — kırmızı
    3: (255,   0, 128),   # UNSAFE_DROP  — magenta
}

RISK_LABEL_NAMES = {0: "SAFE", 1: "UNCERTAIN", 2: "UNSAFE_SOLID", 3: "UNSAFE_DROP"}


def _normalize_depth_to_uint8(depth: np.ndarray) -> np.ndarray:
    """Float32 depth arrayi 0-255 uint8'e normalize et; NaN = 0."""
    valid = np.nan_to_num(depth, nan=0.0)
    dmin, dmax = valid.min(), valid.max()
    if dmax > dmin:
        norm = ((valid - dmin) / (dmax - dmin) * 255).astype(np.uint8)
    else:
        norm = np.zeros_like(valid, dtype=np.uint8)
    return norm


def _apply_colormap_overlay(
    gray_bg: np.ndarray,
    mask: np.ndarray,
    color_map: dict[int, tuple[int, int, int]],
    alpha: float = 0.6,
    skip_label: int | None = None,
) -> np.ndarray:
    """Gray background üzerine mask renklerini alpha blend ile uygula."""
    bg_bgr = cv2.cvtColor(gray_bg, cv2.COLOR_GRAY2BGR)
    overlay = bg_bgr.copy()

    for label, color in color_map.items():
        if label == skip_label:
            continue
        where = mask == label
        if np.any(where):
            overlay[where] = color

    blended = cv2.addWeighted(bg_bgr, 1 - alpha, overlay, alpha, 0)
    return blended


def _add_legend(img: np.ndarray, color_map: dict[int, tuple[int, int, int]]) -> np.ndarray:
    """Görüntünün sağ üst köşesine küçük bir risk label efsanesi ekle."""
    out = img.copy()
    x0, y0 = img.shape[1] - 160, 8
    for label, color in color_map.items():
        text = RISK_LABEL_NAMES.get(label, str(label))
        cv2.rectangle(out, (x0, y0), (x0 + 14, y0 + 14), color, -1)
        cv2.putText(out, text, (x0 + 18, y0 + 12),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.38, (230, 230, 230), 1, cv2.LINE_AA)
        y0 += 18
    return out


def visualize_sample(
    stem: str,
    raw_dir: Path,
    labels_dir: Path,
    processed_dir: Path,
) -> bool:
    """Bir örnek için üç QC görseli üret.

    Returns:
        True başarılıysa, False gerekli dosya eksikse.
    """
    depth_path = raw_dir / f"{stem}_depth.npy"
    risk_path = labels_dir / f"{stem}_risk_mask.png"
    edge_path = labels_dir / f"{stem}_edge_mask.png"

    if not depth_path.exists() or not risk_path.exists() or not edge_path.exists():
        return False

    depth = np.load(str(depth_path))
    risk_mask = cv2.imread(str(risk_path), cv2.IMREAD_GRAYSCALE)
    edge_raw = cv2.imread(str(edge_path), cv2.IMREAD_GRAYSCALE)

    if risk_mask is None or edge_raw is None:
        return False

    gray = _normalize_depth_to_uint8(depth)

    # 1) Depth görselleştirmesi (jet colormap)
    depth_viz = cv2.applyColorMap(gray, cv2.COLORMAP_JET)
    depth_viz_path = processed_dir / f"{stem}_depth_viz.png"
    cv2.imwrite(str(depth_viz_path), depth_viz)

    # 2) Risk mask overlay
    risk_overlay = _apply_colormap_overlay(gray, risk_mask, RISK_COLORS_BGR, alpha=0.55, skip_label=0)
    risk_overlay = _add_legend(risk_overlay, RISK_COLORS_BGR)
    cv2.imwrite(str(processed_dir / f"{stem}_risk_overlay.png"), risk_overlay)

    # 3) Edge mask overlay (beyaz kontur)
    edge_bin = (edge_raw > 127).astype(np.uint8)
    edge_overlay = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
    edge_overlay[edge_bin == 1] = (0, 255, 255)  # sarı kontur
    cv2.imwrite(str(processed_dir / f"{stem}_edge_overlay.png"), edge_overlay)

    return True


def _collect_stems(raw_dir: Path) -> list[str]:
    """raw_dir içindeki tüm *_depth.npy dosyalarının stem listesini döndür."""
    stems = []
    for f in sorted(raw_dir.glob("*_depth.npy")):
        stem = f.name.replace("_depth.npy", "")
        stems.append(stem)
    return stems


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Sentetik depth dataset QC görselleştirici."
    )
    parser.add_argument(
        "--data-dir",
        type=Path,
        default=Path("data"),
        help="Veri kök klasörü (varsayılan: data).",
    )
    parser.add_argument(
        "--max-samples",
        type=int,
        default=40,
        help="İşlenecek maksimum örnek sayısı.",
    )
    return parser.parse_args()


def main() -> None:
    args = _parse_args()

    raw_dir = args.data_dir / "raw"
    labels_dir = args.data_dir / "labels"
    processed_dir = args.data_dir / "processed"
    processed_dir.mkdir(parents=True, exist_ok=True)

    stems = _collect_stems(raw_dir)
    if not stems:
        print(f"UYARI: {raw_dir} altinda hic depth.npy bulunamadi.")
        return

    stems = stems[: args.max_samples]
    ok = 0
    skip = 0

    print(f"Gorsellestirilecek ornek sayisi: {len(stems)}")
    print("-" * 40)

    for stem in stems:
        success = visualize_sample(stem, raw_dir, labels_dir, processed_dir)
        if success:
            ok += 1
            print(f"  OK  {stem}")
        else:
            skip += 1
            print(f"  SKIP {stem}  (eksik dosya)")

    print("-" * 40)
    print(f"Tamamlandi. OK={ok}  SKIP={skip}")
    print(f"Gorseller: {processed_dir}")


if __name__ == "__main__":
    main()
