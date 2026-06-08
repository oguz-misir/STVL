"""
Depth görüntüsü işleme yardımcıları.

NaN filtreleme, normalizasyon, geçerlilik maskesi ve temel istatistikler.
"""

from __future__ import annotations

import numpy as np


def valid_mask(depth: np.ndarray, min_depth: float = 0.05, max_depth: float = 10.0) -> np.ndarray:
    """Geçerli piksel maskesi döndür (float, bool array).

    Args:
        depth: (H, W) float array, metre.
        min_depth: Minimum geçerli derinlik (m).
        max_depth: Maksimum geçerli derinlik (m).

    Returns:
        (H, W) bool — True = geçerli piksel.
    """
    return np.isfinite(depth) & (depth >= min_depth) & (depth <= max_depth)


def normalize_depth(
    depth: np.ndarray,
    min_depth: float | None = None,
    max_depth: float | None = None,
) -> np.ndarray:
    """Depth'i [0, 1] aralığına normalize et; NaN pikseller 0 olur.

    Args:
        depth: (H, W) float array.
        min_depth: None ise geçerli piksel minimumunu kullan.
        max_depth: None ise geçerli piksel maksimumunu kullan.

    Returns:
        (H, W) float32 [0, 1].
    """
    valid = valid_mask(depth)
    filled = np.where(valid, depth, np.nan)

    d_min = min_depth if min_depth is not None else float(np.nanmin(filled)) if np.any(valid) else 0.0
    d_max = max_depth if max_depth is not None else float(np.nanmax(filled)) if np.any(valid) else 1.0

    if d_max <= d_min:
        return np.zeros_like(depth, dtype=np.float32)

    norm = np.where(valid, (depth - d_min) / (d_max - d_min), 0.0)
    return np.clip(norm, 0.0, 1.0).astype(np.float32)


def fill_nans_nearest(depth: np.ndarray) -> np.ndarray:
    """NaN pikselleri en yakın geçerli piksel değeriyle doldur.

    Küçük missing-depth yamalarını gidermek için kullanılır.
    """
    import cv2

    valid = valid_mask(depth)
    if np.all(valid):
        return depth.copy()

    # OpenCV inpaint için uint16'ya dönüştür
    filled = np.nan_to_num(depth, nan=0.0)
    mask_u8 = (~valid).astype(np.uint8)
    d_max = float(np.nanmax(depth)) if np.any(valid) else 1.0
    scale = 65535.0 / max(d_max, 1e-6)
    d_u16 = np.clip(filled * scale, 0, 65535).astype(np.uint16)
    inpainted = cv2.inpaint(d_u16, mask_u8, inpaintRadius=3, flags=cv2.INPAINT_NS)
    return (inpainted.astype(np.float32) / scale)


def depth_gradient_magnitude(depth: np.ndarray) -> np.ndarray:
    """Depth görüntüsünün gradyan büyüklüğünü hesapla (Sobel).

    NaN pikseller 0 kabul edilir.
    Returns:
        (H, W) float32 — gradyan büyüklüğü.
    """
    import cv2

    filled = np.nan_to_num(depth, nan=0.0).astype(np.float32)
    gx = cv2.Sobel(filled, cv2.CV_32F, 1, 0, ksize=3)
    gy = cv2.Sobel(filled, cv2.CV_32F, 0, 1, ksize=3)
    return np.sqrt(gx ** 2 + gy ** 2)


def depth_stats(depth: np.ndarray) -> dict[str, float]:
    """Geçerli pikseller üzerinde temel istatistikler döndür."""
    valid = valid_mask(depth)
    if not np.any(valid):
        return {"count": 0, "mean": float("nan"), "std": float("nan"),
                "min": float("nan"), "max": float("nan"), "valid_ratio": 0.0}
    vals = depth[valid]
    return {
        "count": int(vals.size),
        "mean": float(np.mean(vals)),
        "std": float(np.std(vals)),
        "min": float(np.min(vals)),
        "max": float(np.max(vals)),
        "valid_ratio": float(vals.size) / depth.size,
    }
