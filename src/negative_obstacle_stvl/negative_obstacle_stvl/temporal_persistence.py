"""
Temporal nokta bulutu biriktiricisi.

Drop-edge noktalarını zaman içinde biriktirerek STVL'ye tutarlı
ve gürültüye dayanıklı bir marking source sağlar.
"""

from __future__ import annotations

import time
from collections import deque
from dataclasses import dataclass, field

import numpy as np


@dataclass
class StampedPoints:
    points: np.ndarray       # (N, 3) float32
    timestamp: float = field(default_factory=time.monotonic)


class TemporalPointAccumulator:
    """
    Zaman pencereli nokta bulutu biriktiricisi.

    Her frame'den gelen noktaları bir pencere içinde tutar.
    Eski noktalar otomatik olarak atılır.
    STVL'ye gönderilecek birleşik nokta bulutu döndürülür.
    """

    def __init__(
        self,
        window_sec: float = 1.0,
        max_points: int = 5000,
        voxel_size: float = 0.05,
    ) -> None:
        """
        Args:
            window_sec: Noktaların saklandığı zaman penceresi (saniye).
            max_points: Döndürülecek maksimum nokta sayısı.
            voxel_size: Voxel filtreleme çözünürlüğü (metre). 0 = devre dışı.
        """
        self.window_sec = window_sec
        self.max_points = max_points
        self.voxel_size = voxel_size
        self._buffer: deque[StampedPoints] = deque()

    def add(self, points: np.ndarray) -> None:
        """
        Yeni frame noktalarını ekle.

        Args:
            points: (N, 3) float32 — bu frame'in drop-edge noktaları.
        """
        if len(points) == 0:
            return
        self._buffer.append(StampedPoints(points=points.astype(np.float32)))
        self._evict_old()

    def _evict_old(self) -> None:
        """Zaman penceresinin dışındaki eski noktaları sil."""
        cutoff = time.monotonic() - self.window_sec
        while self._buffer and self._buffer[0].timestamp < cutoff:
            self._buffer.popleft()

    def get_accumulated(self) -> np.ndarray:
        """
        Penceredeki tüm noktaları birleştir ve döndür.

        Voxel filtreleme uygulanır; max_points kadar örneklenir.

        Döndürür:
            (M, 3) float32 — biriktirilmiş nokta bulutu.
        """
        self._evict_old()

        if not self._buffer:
            return np.empty((0, 3), dtype=np.float32)

        all_pts = np.vstack([sp.points for sp in self._buffer])

        if self.voxel_size > 0:
            all_pts = _voxel_downsample(all_pts, self.voxel_size)

        if len(all_pts) > self.max_points:
            idx = np.random.default_rng(0).choice(len(all_pts), self.max_points, replace=False)
            all_pts = all_pts[idx]

        return all_pts.astype(np.float32)

    def clear(self) -> None:
        """Tamponu temizle."""
        self._buffer.clear()

    @property
    def frame_count(self) -> int:
        """Tamponda bekleyen frame sayısı."""
        return len(self._buffer)

    @property
    def point_count(self) -> int:
        """Tamponda bekleyen toplam nokta sayısı (voxel öncesi)."""
        return sum(len(sp.points) for sp in self._buffer)


def _voxel_downsample(points: np.ndarray, voxel_size: float) -> np.ndarray:
    """
    Basit voxel grid downsampling.

    Her voxel içindeki noktaların merkezini döndürür.
    """
    if len(points) == 0:
        return points

    # Her noktanın hangi voxel'e ait olduğunu bul
    mins = points.min(axis=0)
    voxel_ids = np.floor((points - mins) / voxel_size).astype(np.int32)

    # Benzersiz voxel'lerin merkezini hesapla
    unique_voxels, inverse = np.unique(voxel_ids, axis=0, return_inverse=True)
    centroids = np.zeros((len(unique_voxels), 3), dtype=np.float64)
    counts = np.zeros(len(unique_voxels), dtype=np.int32)

    np.add.at(centroids, inverse, points)
    np.add.at(counts, inverse, 1)
    centroids /= counts[:, np.newaxis]

    return centroids.astype(np.float32)
