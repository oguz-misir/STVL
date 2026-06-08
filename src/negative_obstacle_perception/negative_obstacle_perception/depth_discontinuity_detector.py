"""
Depth süreksizlik (discontinuity) ve drop-edge tespiti.

Komşu pikseller arasındaki ani derinlik değişimlerini negatif engel
sınırı olarak işaretler. Ground estimator sonucuyla birleştirilir.
"""

from __future__ import annotations

import cv2
import numpy as np

from negative_obstacle_common.depth_utils import valid_mask


def compute_depth_jump_map(
    depth: np.ndarray,
    min_depth: float = 0.05,
    max_depth: float = 10.0,
) -> np.ndarray:
    """
    Her piksel için komşularıyla maksimum derinlik farkını hesapla.

    8-komşuluk kullanılır; NaN pikseller 0 kabul edilir.

    Döndürür:
        (H, W) float32 — piksel başına maksimum komşu depth farkı (metre).
    """
    vm = valid_mask(depth, min_depth, max_depth).astype(np.float32)
    filled = np.where(np.isfinite(depth), depth, 0.0).astype(np.float32)

    # Mutlak fark: her yönde Sobel-benzeri shift
    shifts = [
        np.roll(filled,  1, axis=0), np.roll(filled, -1, axis=0),
        np.roll(filled,  1, axis=1), np.roll(filled, -1, axis=1),
        np.roll(np.roll(filled,  1, axis=0),  1, axis=1),
        np.roll(np.roll(filled,  1, axis=0), -1, axis=1),
        np.roll(np.roll(filled, -1, axis=0),  1, axis=1),
        np.roll(np.roll(filled, -1, axis=0), -1, axis=1),
    ]
    diffs = np.stack([np.abs(filled - s) for s in shifts], axis=0)
    jump_map = diffs.max(axis=0).astype(np.float32)

    # Geçersiz pikselleri sıfırla
    jump_map *= vm
    return jump_map


def detect_drop_edges(
    depth: np.ndarray,
    jump_threshold: float = 0.15,
    drop_direction_only: bool = True,
    dilation_px: int = 1,
    min_depth: float = 0.05,
    max_depth: float = 10.0,
) -> np.ndarray:
    """
    Depth süreksizliklerinden drop-edge maskesi üret.

    Args:
        depth: (H, W) float32, metre.
        jump_threshold: Bu eşiğin üstündeki farklar edge kabul edilir (metre).
        drop_direction_only: True ise yalnızca derinlik artan (uzaklaşan)
            yöndeki geçişler işaretlenir — gerçek drop sınırı.
        dilation_px: Kenar genişletme yarıçapı.
        min_depth / max_depth: Geçerli derinlik aralığı.

    Döndürür:
        (H, W) uint8 — 255: edge piksel, 0: edge değil.
    """
    jump_map = compute_depth_jump_map(depth, min_depth, max_depth)
    edge_raw = (jump_map > jump_threshold).astype(np.uint8)

    if drop_direction_only:
        # Yalnızca ileri (z artıyor = uzaklaşıyor) yönde geçişi tut
        # Merkez pikselden komşuya derinlik artıyorsa gerçek drop kenarı
        filled = np.where(np.isfinite(depth), depth, 0.0).astype(np.float32)
        below = np.roll(filled, -1, axis=0)   # bir satır aşağı
        drop_dir = (below - filled) > jump_threshold
        edge_raw = (edge_raw.astype(bool) & drop_dir).astype(np.uint8)

    if dilation_px > 0:
        kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (2 * dilation_px + 1, 2 * dilation_px + 1)
        )
        edge_raw = cv2.dilate(edge_raw, kernel)

    return (edge_raw * 255).astype(np.uint8)


def detect_missing_depth_regions(
    depth: np.ndarray,
    min_area_px: int = 50,
    min_depth: float = 0.05,
    max_depth: float = 10.0,
) -> np.ndarray:
    """
    Geçersiz (NaN/sıfır) derinlik bölgelerini bul.

    Gölge ve parlak yüzey gibi sensör başarısızlıkları bu bölgeler olarak
    döner; negatif engel tespitinde UNCERTAIN etiketi için kullanılır.

    Döndürür:
        (H, W) uint8 — 255: missing bölge, 0: geçerli.
    """
    vm = valid_mask(depth, min_depth, max_depth)
    missing = (~vm).astype(np.uint8) * 255

    # Küçük gürültü noktalarını temizle
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    missing = cv2.morphologyEx(missing, cv2.MORPH_OPEN, kernel)

    # Çok küçük bölgeleri at
    n_labels, labels, stats, _ = cv2.connectedComponentsWithStats(missing, connectivity=8)
    filtered = np.zeros_like(missing)
    for lbl in range(1, n_labels):
        if stats[lbl, cv2.CC_STAT_AREA] >= min_area_px:
            filtered[labels == lbl] = 255

    return filtered


class DepthDiscontinuityDetector:
    """
    Tek ayar noktasından süreksizlik tespiti sarmalayıcısı.
    """

    def __init__(
        self,
        jump_threshold_m: float = 0.15,
        dilation_px: int = 2,
        min_missing_area_px: int = 50,
        min_depth: float = 0.05,
        max_depth: float = 10.0,
    ) -> None:
        self.jump_threshold_m = jump_threshold_m
        self.dilation_px = dilation_px
        self.min_missing_area_px = min_missing_area_px
        self.min_depth = min_depth
        self.max_depth = max_depth

    def detect(self, depth: np.ndarray) -> dict:
        """
        Depth görüntüsünden süreksizlik ve missing bölgelerini tespit et.

        Döndürür:
            dict ile:
            - edge_mask: (H, W) uint8 — drop-edge pikseller (255)
            - missing_mask: (H, W) uint8 — missing derinlik bölgeler (255)
            - jump_map: (H, W) float32 — komşu derinlik fark haritası
        """
        jump_map = compute_depth_jump_map(depth, self.min_depth, self.max_depth)

        edge_mask = detect_drop_edges(
            depth,
            jump_threshold=self.jump_threshold_m,
            drop_direction_only=True,
            dilation_px=self.dilation_px,
            min_depth=self.min_depth,
            max_depth=self.max_depth,
        )

        missing_mask = detect_missing_depth_regions(
            depth,
            min_area_px=self.min_missing_area_px,
            min_depth=self.min_depth,
            max_depth=self.max_depth,
        )

        return {
            "edge_mask": edge_mask,
            "missing_mask": missing_mask,
            "jump_map": jump_map,
        }
