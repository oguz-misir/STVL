"""
Offline geometrik risk algoritması değerlendirme scripti.

Gazebo veya ROS node çalıştırmadan, sentetik depth dataset üzerinde
geometrik risk algoritmasının metriklerini hesaplar.

Kullanım:
    python3 experiments/ablation/test_synthetic_geometric_metrics.py \
        --data-dir data \
        --output-json experiments/results/synthetic_geometric_metrics.json
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import cv2
import numpy as np

# Paket yolları
REPO_ROOT = Path(__file__).parent.parent.parent
sys.path.insert(0, str(REPO_ROOT / "src" / "negative_obstacle_common"))
sys.path.insert(0, str(REPO_ROOT / "src" / "negative_obstacle_perception"))

from negative_obstacle_common.camera_utils import CameraIntrinsics
from negative_obstacle_common.risk_labels import UNSAFE_DROP, HARD_NEGATIVE_SCENE_TYPES
from negative_obstacle_common.metrics import compute_all_metrics

from negative_obstacle_perception.geometric_risk_analyzer import GeometricRiskAnalyzer


def _load_sample(
    stem: str,
    raw_dir: Path,
    labels_dir: Path,
) -> tuple[np.ndarray | None, np.ndarray | None, np.ndarray | None, dict | None]:
    """Tek örnek için depth, risk_mask, edge_mask ve meta yükle."""
    depth_path = raw_dir / f"{stem}_depth.npy"
    risk_path = labels_dir / f"{stem}_risk_mask.png"
    edge_path = labels_dir / f"{stem}_edge_mask.png"
    meta_path = raw_dir / f"{stem}_meta.json"

    if not all(p.exists() for p in [depth_path, risk_path, edge_path, meta_path]):
        return None, None, None, None

    depth = np.load(str(depth_path))
    risk_mask = cv2.imread(str(risk_path), cv2.IMREAD_GRAYSCALE)
    edge_mask = cv2.imread(str(edge_path), cv2.IMREAD_GRAYSCALE)

    with open(meta_path) as f:
        meta = json.load(f)

    if risk_mask is None or edge_mask is None:
        return None, None, None, None

    return depth, risk_mask, edge_mask, meta


def _collect_stems(raw_dir: Path, max_samples: int) -> list[str]:
    stems = []
    for f in sorted(raw_dir.glob("*_depth.npy")):
        stems.append(f.name.replace("_depth.npy", ""))
    return stems[:max_samples]


def evaluate_dataset(
    data_dir: Path,
    max_samples: int = 500,
    use_ground_estimator: bool = True,
    width: int = 320,
    height: int = 240,
) -> dict:
    """
    Tüm dataset üzerinde geometrik risk algoritmasını çalıştır ve metrik hesapla.
    """
    raw_dir = data_dir / "raw"
    labels_dir = data_dir / "labels"

    intrinsics = CameraIntrinsics.from_fov(width, height, fov_h_deg=70.0)
    analyzer = GeometricRiskAnalyzer(
        intrinsics=intrinsics,
        jump_threshold_m=0.12,
        drop_threshold_m=0.08,
        dilation_drop_px=3,
        dilation_edge_px=2,
        use_ground_estimator=use_ground_estimator,
    )

    stems = _collect_stems(raw_dir, max_samples)
    if not stems:
        return {"error": "Hiç örnek bulunamadı", "total": 0}

    # Metrik toplayıcılar — pozitif sahneler
    pos_recalls: list[float] = []
    pos_fsrates: list[float] = []
    pos_precisions: list[float] = []
    pos_edge_f1s: list[float] = []

    # Hard-negative sahneler
    hn_fprs: list[float] = []

    skipped = 0
    processed = 0

    for stem in stems:
        depth, gt_risk, gt_edge, meta = _load_sample(stem, raw_dir, labels_dir)
        if depth is None:
            skipped += 1
            continue

        scene_type = meta.get("scene_type", "unknown")
        is_hn = scene_type in HARD_NEGATIVE_SCENE_TYPES
        h_actual, w_actual = depth.shape

        # Görüntü boyutuna uygun intrinsics
        intr = CameraIntrinsics.from_fov(w_actual, h_actual, fov_h_deg=70.0)
        if intr.width != intrinsics.width or intr.height != intrinsics.height:
            local_analyzer = GeometricRiskAnalyzer(
                intrinsics=intr,
                jump_threshold_m=0.12,
                drop_threshold_m=0.08,
                dilation_drop_px=3,
                dilation_edge_px=2,
                use_ground_estimator=use_ground_estimator,
            )
        else:
            local_analyzer = analyzer

        # Risk maskesi üret
        if use_ground_estimator:
            result = local_analyzer.analyze(depth, rng_seed=0)
        else:
            result = local_analyzer.analyze_edge_only(depth)

        pred_risk = result["risk_mask"]
        pred_edge = result["edge_mask"]

        # Metrikleri hesapla
        metrics = compute_all_metrics(
            pred_mask=pred_risk,
            gt_mask=gt_risk,
            pred_edge=pred_edge,
            gt_edge=gt_edge,
            is_hard_negative=is_hn,
        )

        if is_hn:
            fpr = metrics.get("hard_negative_fpr", float("nan"))
            if fpr == fpr:  # NaN değil
                hn_fprs.append(fpr)
        else:
            r = metrics.get("drop_recall", float("nan"))
            f = metrics.get("false_safe_rate", float("nan"))
            p = metrics.get("drop_precision", float("nan"))
            e = metrics.get("edge_f1", float("nan"))
            if r == r:
                pos_recalls.append(r)
            if f == f:
                pos_fsrates.append(f)
            if p == p:
                pos_precisions.append(p)
            if e == e:
                pos_edge_f1s.append(e)

        processed += 1

    def _mean(lst: list[float]) -> float:
        return float(np.mean(lst)) if lst else float("nan")

    report = {
        "total_evaluated": processed,
        "total_skipped": skipped,
        "positive_scenes": {
            "count": len(pos_recalls),
            "mean_drop_recall":    round(_mean(pos_recalls), 4),
            "mean_false_safe_rate": round(_mean(pos_fsrates), 4),
            "mean_drop_precision": round(_mean(pos_precisions), 4),
            "mean_edge_f1":        round(_mean(pos_edge_f1s), 4),
        },
        "hard_negative_scenes": {
            "count": len(hn_fprs),
            "mean_false_positive_rate": round(_mean(hn_fprs), 4),
        },
        "settings": {
            "use_ground_estimator": use_ground_estimator,
            "jump_threshold_m": 0.12,
            "drop_threshold_m": 0.08,
        },
    }
    return report


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Sentetik dataset üzerinde offline geometrik risk metrikleri."
    )
    parser.add_argument("--data-dir", type=Path, default=Path("data"))
    parser.add_argument(
        "--output-json",
        type=Path,
        default=Path("experiments/results/synthetic_geometric_metrics.json"),
    )
    parser.add_argument("--max-samples", type=int, default=500)
    parser.add_argument("--no-ground-estimator", action="store_true",
                        help="RANSAC ground tahmini olmadan yalnızca edge tespiti kullan.")
    return parser.parse_args()


def main() -> None:
    args = _parse_args()

    print("Geometrik risk metrikleri hesaplaniyor...")
    print(f"  Veri klasoru : {args.data_dir}")
    print(f"  Ground estimator: {'Hayir (edge-only)' if args.no_ground_estimator else 'Evet'}")
    print(f"  Max ornek   : {args.max_samples}")
    print("-" * 50)

    report = evaluate_dataset(
        data_dir=args.data_dir,
        max_samples=args.max_samples,
        use_ground_estimator=not args.no_ground_estimator,
    )

    args.output_json.parent.mkdir(parents=True, exist_ok=True)
    with open(args.output_json, "w") as f:
        json.dump(report, f, indent=2, ensure_ascii=False)

    print(f"Degerlendirilen ornek : {report['total_evaluated']}")
    pos = report["positive_scenes"]
    hn = report["hard_negative_scenes"]
    print(f"\nPozitif sahneler ({pos['count']} ornek):")
    print(f"  Drop Recall         : {pos['mean_drop_recall']:.4f}")
    print(f"  False Safe Rate     : {pos['mean_false_safe_rate']:.4f}  <- kritik metrik")
    print(f"  Drop Precision      : {pos['mean_drop_precision']:.4f}")
    print(f"  Edge F1             : {pos['mean_edge_f1']:.4f}")
    print(f"\nHard-negative sahneler ({hn['count']} ornek):")
    print(f"  False Positive Rate : {hn['mean_false_positive_rate']:.4f}")
    print(f"\nRapor: {args.output_json}")


if __name__ == "__main__":
    main()
