"""
Risk mask görselleştirme renk haritası.

Tüm paketlerde tutarlı renk kullanımı için merkezi tanım.
"""

from __future__ import annotations

import numpy as np

from .risk_labels import RiskLabel

# BGR renk sözlüğü (OpenCV formatı)
RISK_COLOR_BGR: dict[int, tuple[int, int, int]] = {
    RiskLabel.SAFE:         (0,   200,   0),   # yeşil
    RiskLabel.UNCERTAIN:    (0,   165, 255),   # turuncu
    RiskLabel.UNSAFE_SOLID: (0,     0, 255),   # kırmızı
    RiskLabel.UNSAFE_DROP:  (255,   0, 128),   # magenta
}

# RGB (matplotlib / PIL için)
RISK_COLOR_RGB: dict[int, tuple[int, int, int]] = {
    label: (bgr[2], bgr[1], bgr[0])
    for label, bgr in RISK_COLOR_BGR.items()
}


def colorize_risk_mask(
    risk_mask: np.ndarray,
    alpha: float = 1.0,
    background: np.ndarray | None = None,
) -> np.ndarray:
    """
    Risk mask'i renkli BGR görüntüye dönüştür.

    Args:
        risk_mask: (H, W) uint8 — label değerleri 0-3.
        alpha: Overlay şeffaflığı (0=arka plan, 1=tam renkli).
        background: (H, W, 3) uint8 BGR arka plan; None ise siyah.

    Returns:
        (H, W, 3) uint8 BGR görüntü.
    """
    h, w = risk_mask.shape
    out = np.zeros((h, w, 3), dtype=np.uint8) if background is None else background.copy()

    for label, color in RISK_COLOR_BGR.items():
        where = risk_mask == label
        if np.any(where):
            if alpha < 1.0 and background is not None:
                colored = np.array(color, dtype=np.uint8)
                out[where] = (
                    alpha * colored + (1 - alpha) * out[where]
                ).astype(np.uint8)
            else:
                out[where] = color

    return out


def colorize_edge_mask(
    edge_mask: np.ndarray,
    color_bgr: tuple[int, int, int] = (0, 255, 255),
    background: np.ndarray | None = None,
) -> np.ndarray:
    """
    Edge mask'i verilen renkle BGR görüntüye dönüştür.

    Args:
        edge_mask: (H, W) uint8 — 0 veya 255.
        color_bgr: Edge pikseli rengi (varsayılan: sarı).
        background: (H, W, 3) uint8 BGR arka plan.

    Returns:
        (H, W, 3) uint8 BGR.
    """
    h, w = edge_mask.shape
    out = np.zeros((h, w, 3), dtype=np.uint8) if background is None else background.copy()
    out[edge_mask > 127] = color_bgr
    return out


def label_color_bgr(label: int) -> tuple[int, int, int]:
    """Tek bir label için BGR renk döndür."""
    return RISK_COLOR_BGR.get(label, (128, 128, 128))


def label_color_rgb(label: int) -> tuple[int, int, int]:
    """Tek bir label için RGB renk döndür."""
    return RISK_COLOR_RGB.get(label, (128, 128, 128))
