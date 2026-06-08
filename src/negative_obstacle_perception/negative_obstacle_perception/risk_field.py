"""
Uncertainty-aware negative obstacle risk field.

This module turns classical geometric cues into graded risk channels:
drop probability, unknown-support probability, confidence, temporal
stability, and navigation cost. It intentionally stays lightweight so the
pipeline can run on embedded CPUs and remain compatible with the existing
binary risk-mask output.
"""

from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np

from negative_obstacle_common.depth_utils import valid_mask


def _sigmoid(x: np.ndarray) -> np.ndarray:
    return (1.0 / (1.0 + np.exp(-np.clip(x, -30.0, 30.0)))).astype(np.float32)


def _normalize_positive(values: np.ndarray, scale: float) -> np.ndarray:
    if scale <= 0:
        return np.zeros_like(values, dtype=np.float32)
    return np.clip(values / scale, 0.0, 1.0).astype(np.float32)


@dataclass
class RiskFieldConfig:
    jump_threshold_m: float = 0.12
    drop_threshold_m: float = 0.08
    missing_support_weight: float = 0.65
    drop_weight: float = 0.70
    confidence_weight: float = 0.20
    temporal_alpha: float = 0.65
    lethal_cost_threshold: float = 0.82
    high_cost_threshold: float = 0.55


class UncertaintyAwareRiskField:
    """
    Converts depth/ground/discontinuity evidence into a graded risk field.

    The output is not a classifier only. It keeps separate channels for
    geometric drop evidence, support uncertainty, confidence, temporal
    stability, and Nav2-compatible cost. This is the representation-level
    contribution used by the paper narrative.
    """

    def __init__(self, config: RiskFieldConfig | None = None) -> None:
        self.config = config or RiskFieldConfig()
        self._prev_risk: np.ndarray | None = None

    def reset(self) -> None:
        self._prev_risk = None

    def compute(
        self,
        depth: np.ndarray,
        jump_map: np.ndarray,
        missing_mask: np.ndarray,
        edge_mask: np.ndarray,
        height_map: np.ndarray | None = None,
        ground_mask: np.ndarray | None = None,
        ground_success: bool = False,
    ) -> dict[str, np.ndarray]:
        cfg = self.config
        finite = valid_mask(depth).astype(np.float32)
        missing = (missing_mask > 127).astype(np.float32)
        edge = (edge_mask > 127).astype(np.float32)

        jump_evidence = _normalize_positive(jump_map, cfg.jump_threshold_m * 2.0)

        if height_map is None:
            drop_height = np.zeros_like(depth, dtype=np.float32)
        else:
            drop_height = np.where(np.isfinite(height_map), -height_map, 0.0).astype(np.float32)
            drop_height = np.maximum(drop_height, 0.0)
        height_evidence = _normalize_positive(drop_height, cfg.drop_threshold_m * 2.0)

        # A smooth ramp tends to have a broad first-order gradient but weak
        # edge evidence. Penalize that case to reduce hard-negative alarms.
        filled = np.where(np.isfinite(depth), depth, 0.0).astype(np.float32)
        grad_x = cv2.Sobel(filled, cv2.CV_32F, 1, 0, ksize=3)
        grad_y = cv2.Sobel(filled, cv2.CV_32F, 0, 1, ksize=3)
        smooth_slope = _normalize_positive(np.sqrt(grad_x * grad_x + grad_y * grad_y), cfg.jump_threshold_m)
        ramp_consistency = np.clip(smooth_slope * (1.0 - edge), 0.0, 1.0)

        drop_logit = (
            3.0 * jump_evidence
            + 2.4 * height_evidence
            + 1.5 * edge
            - 1.8 * ramp_consistency
            - 2.0
        )
        p_drop = _sigmoid(drop_logit) * finite

        # Unknown support is different from confirmed drop: missing or invalid
        # depth near traversable ground is risky, but should not always be lethal.
        near_edge = cv2.dilate(edge.astype(np.uint8), np.ones((5, 5), np.uint8)).astype(np.float32)
        p_unknown = np.clip(0.75 * missing + 0.35 * missing * near_edge, 0.0, 1.0)

        if ground_mask is None:
            plane_conf = np.full_like(depth, 0.55 if ground_success else 0.35, dtype=np.float32)
        else:
            inlier_ratio = float(np.mean(ground_mask)) if ground_mask.size else 0.0
            plane_conf = np.full_like(depth, np.clip(0.35 + 1.4 * inlier_ratio, 0.35, 0.95), dtype=np.float32)

        # Local depth quality lowers confidence around invalid or noisy regions.
        local_valid = cv2.blur(finite, (5, 5)).astype(np.float32)
        local_noise = cv2.blur(np.abs(filled - cv2.blur(filled, (5, 5))), (5, 5))
        noise_penalty = _normalize_positive(local_noise, cfg.jump_threshold_m * 1.5)
        confidence = np.clip(plane_conf * local_valid * (1.0 - 0.45 * noise_penalty), 0.05, 1.0)

        instantaneous = np.clip(
            cfg.drop_weight * p_drop
            + cfg.missing_support_weight * p_unknown
            + cfg.confidence_weight * (1.0 - confidence) * np.maximum(p_drop, p_unknown),
            0.0,
            1.0,
        ).astype(np.float32)

        if self._prev_risk is None or self._prev_risk.shape != instantaneous.shape:
            temporal_stability = instantaneous
        else:
            temporal_stability = (
                cfg.temporal_alpha * self._prev_risk
                + (1.0 - cfg.temporal_alpha) * instantaneous
            ).astype(np.float32)
        self._prev_risk = temporal_stability

        risk_score = np.clip(0.65 * instantaneous + 0.35 * temporal_stability, 0.0, 1.0)
        cost = np.clip(np.round(254.0 * risk_score), 0, 254).astype(np.uint8)

        # Keep high-confidence drop edges lethal for compatibility with STVL
        # marking behavior while preserving graded costs elsewhere.
        lethal = risk_score >= cfg.lethal_cost_threshold
        cost[lethal] = 254

        high = (risk_score >= cfg.high_cost_threshold) & ~lethal
        cost[high] = np.maximum(cost[high], 180).astype(np.uint8)

        return {
            "p_drop": p_drop.astype(np.float32),
            "p_unknown": p_unknown.astype(np.float32),
            "confidence": confidence.astype(np.float32),
            "temporal_stability": temporal_stability.astype(np.float32),
            "risk_score": risk_score.astype(np.float32),
            "cost": cost,
        }
