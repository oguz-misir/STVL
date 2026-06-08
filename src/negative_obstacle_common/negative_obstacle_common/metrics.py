"""
Değerlendirme metrikleri.

Deney ve makale için Drop Recall, False Safe Rate, Edge F1,
Drop Precision ve Hard-Negative FPR hesaplar.
"""

from __future__ import annotations

import numpy as np

from .risk_labels import UNSAFE_DROP


def _binary(mask: np.ndarray, positive_label: int) -> np.ndarray:
    """Mask'i verilen label için binary'ye çevir."""
    return (mask == positive_label).astype(bool)


def drop_recall(pred_mask: np.ndarray, gt_mask: np.ndarray) -> float:
    """
    Drop Recall = TP_drop / (TP_drop + FN_drop)

    Gerçek drop piksellerinin kaçının yakalandığını ölçer.
    En kritik metrik: düşük recall → robot düşer.
    """
    gt_drop = _binary(gt_mask, UNSAFE_DROP)
    pred_drop = _binary(pred_mask, UNSAFE_DROP)

    tp = int(np.sum(gt_drop & pred_drop))
    fn = int(np.sum(gt_drop & ~pred_drop))
    denom = tp + fn
    return tp / denom if denom > 0 else float("nan")


def false_safe_rate(pred_mask: np.ndarray, gt_mask: np.ndarray) -> float:
    """
    False Safe Rate = FN_drop / (TP_drop + FN_drop)

    Gerçek drop bölgesinin güvenli işaretlenme oranı.
    En tehlikeli hata türü: makale için birincil metrik.
    """
    r = drop_recall(pred_mask, gt_mask)
    if r != r:  # NaN kontrolü
        return float("nan")
    return 1.0 - r


def drop_precision(pred_mask: np.ndarray, gt_mask: np.ndarray) -> float:
    """
    Drop Precision = TP_drop / (TP_drop + FP_drop)

    Tahmin edilen drop piksellerinin ne kadarının gerçekten drop olduğu.
    """
    gt_drop = _binary(gt_mask, UNSAFE_DROP)
    pred_drop = _binary(pred_mask, UNSAFE_DROP)

    tp = int(np.sum(gt_drop & pred_drop))
    fp = int(np.sum(~gt_drop & pred_drop))
    denom = tp + fp
    return tp / denom if denom > 0 else float("nan")


def edge_f1(
    pred_edge: np.ndarray,
    gt_edge: np.ndarray,
    tolerance_px: int = 2,
) -> float:
    """
    Edge F1 skoru — toleranslı piksel bazlı.

    pred_edge ve gt_edge: binary (0/255 veya bool) edge mask.
    tolerance_px: GT edge etrafında genişletme (dilation) yarıçapı.
    """
    import cv2

    pred_bin = (pred_edge > 127).astype(np.uint8)
    gt_bin = (gt_edge > 127).astype(np.uint8)

    if tolerance_px > 0:
        kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (2 * tolerance_px + 1, 2 * tolerance_px + 1)
        )
        gt_dilated = cv2.dilate(gt_bin, kernel)
    else:
        gt_dilated = gt_bin

    tp = int(np.sum((pred_bin == 1) & (gt_dilated == 1)))
    fp = int(np.sum((pred_bin == 1) & (gt_dilated == 0)))
    fn = int(np.sum((gt_bin == 1) & (pred_bin == 0)))

    precision = tp / (tp + fp) if (tp + fp) > 0 else 0.0
    recall = tp / (tp + fn) if (tp + fn) > 0 else 0.0
    denom = precision + recall
    return 2 * precision * recall / denom if denom > 0 else 0.0


def hard_negative_fpr(
    pred_mask: np.ndarray,
    gt_mask: np.ndarray,
) -> float:
    """
    Hard-Negative False Positive Rate.

    GT'de hiç UNSAFE_DROP yokken pred'de UNSAFE_DROP piksel oranı.
    Yalnızca hard-negative örneklerde çağrılmalıdır.
    """
    gt_drop = _binary(gt_mask, UNSAFE_DROP)
    if np.any(gt_drop):
        # Bu fonksiyon hard-negative (drop olmayan) örnekler içindir
        return float("nan")
    pred_drop = _binary(pred_mask, UNSAFE_DROP)
    return float(np.sum(pred_drop)) / pred_drop.size


def compute_all_metrics(
    pred_mask: np.ndarray,
    gt_mask: np.ndarray,
    pred_edge: np.ndarray | None = None,
    gt_edge: np.ndarray | None = None,
    is_hard_negative: bool = False,
) -> dict[str, float]:
    """Tüm metrikleri tek sözlükte döndür."""
    result: dict[str, float] = {}

    if is_hard_negative:
        result["hard_negative_fpr"] = hard_negative_fpr(pred_mask, gt_mask)
    else:
        result["drop_recall"]    = drop_recall(pred_mask, gt_mask)
        result["false_safe_rate"] = false_safe_rate(pred_mask, gt_mask)
        result["drop_precision"] = drop_precision(pred_mask, gt_mask)

    if pred_edge is not None and gt_edge is not None:
        result["edge_f1"] = edge_f1(pred_edge, gt_edge)

    return result
