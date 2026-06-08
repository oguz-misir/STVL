"""
Ablation deney grupları tanımları.

Her grup farklı bir algoritmik konfigürasyon temsil eder.
Makale Tablo 1'deki ablation karşılaştırması için kullanılır.
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class AblationConfig:
    name: str
    description: str
    use_ground_estimator: bool
    use_temporal_persistence: bool
    temporal_window_sec: float
    jump_threshold_m: float
    drop_threshold_m: float
    dilation_drop_px: int
    dilation_edge_px: int


ABLATION_GROUPS: list[AblationConfig] = [
    AblationConfig(
        name="A1_baseline_edge_only",
        description="Sadece depth discontinuity tespiti; ground tahmini yok, temporal yok.",
        use_ground_estimator=False,
        use_temporal_persistence=False,
        temporal_window_sec=0.0,
        jump_threshold_m=0.12,
        drop_threshold_m=0.08,
        dilation_drop_px=0,
        dilation_edge_px=2,
    ),
    AblationConfig(
        name="A2_ground_plus_edge",
        description="RANSAC zemin tahmini + depth discontinuity birleşimi.",
        use_ground_estimator=True,
        use_temporal_persistence=False,
        temporal_window_sec=0.0,
        jump_threshold_m=0.12,
        drop_threshold_m=0.08,
        dilation_drop_px=3,
        dilation_edge_px=2,
    ),
    AblationConfig(
        name="A3_ground_plus_edge_temporal",
        description="A2 + 1.5 saniyelik temporal nokta biriktiricisi.",
        use_ground_estimator=True,
        use_temporal_persistence=True,
        temporal_window_sec=1.5,
        jump_threshold_m=0.12,
        drop_threshold_m=0.08,
        dilation_drop_px=3,
        dilation_edge_px=2,
    ),
    AblationConfig(
        name="A4_aggressive_thresholds",
        description="Daha agresif eşikler — yüksek recall, potansiyel false positive artışı.",
        use_ground_estimator=True,
        use_temporal_persistence=True,
        temporal_window_sec=1.5,
        jump_threshold_m=0.08,
        drop_threshold_m=0.05,
        dilation_drop_px=5,
        dilation_edge_px=3,
    ),
    AblationConfig(
        name="A5_conservative_thresholds",
        description="Muhafazakâr eşikler — düşük false positive, potansiyel recall kaybı.",
        use_ground_estimator=True,
        use_temporal_persistence=False,
        temporal_window_sec=0.0,
        jump_threshold_m=0.18,
        drop_threshold_m=0.12,
        dilation_drop_px=2,
        dilation_edge_px=1,
    ),
]
