"""
Risk label sabitleri ve yardımcı fonksiyonlar.

Tüm paketler bu modülden import eder; magic number kullanılmaz.
"""

from __future__ import annotations

from enum import IntEnum


class RiskLabel(IntEnum):
    SAFE         = 0
    UNCERTAIN    = 1
    UNSAFE_SOLID = 2
    UNSAFE_DROP  = 3


SAFE         = RiskLabel.SAFE
UNCERTAIN    = RiskLabel.UNCERTAIN
UNSAFE_SOLID = RiskLabel.UNSAFE_SOLID
UNSAFE_DROP  = RiskLabel.UNSAFE_DROP

HARD_NEGATIVE_SCENE_TYPES: frozenset[str] = frozenset({
    "ramp_hard_negative",
    "shadow_hard_negative",
    "dark_floor_hard_negative",
    "reflective_floor_hard_negative",
    "uneven_ground_hard_negative",
    "sloped_ground_hard_negative",
})

LABEL_NAMES: dict[int, str] = {
    RiskLabel.SAFE:         "SAFE",
    RiskLabel.UNCERTAIN:    "UNCERTAIN",
    RiskLabel.UNSAFE_SOLID: "UNSAFE_SOLID",
    RiskLabel.UNSAFE_DROP:  "UNSAFE_DROP",
}


def is_hard_negative(scene_type: str) -> bool:
    """Sahne tipi hard-negative mi?"""
    return scene_type in HARD_NEGATIVE_SCENE_TYPES


def label_name(label: int) -> str:
    """Label integer'ını okunabilir isme çevir."""
    return LABEL_NAMES.get(label, f"UNKNOWN({label})")


def all_labels() -> list[RiskLabel]:
    return list(RiskLabel)
