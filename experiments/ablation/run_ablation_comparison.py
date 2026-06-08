"""
WP5 Ablation karşılaştırma scripti.

Tüm ablation gruplarını sentetik dataset üzerinde çalıştırır;
JSON ve Markdown tablo olarak sonuçları yazar.

Kullanım:
    python3 experiments/ablation/run_ablation_comparison.py \
        --data-dir data \
        --output-dir experiments/results
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from time import perf_counter

import cv2
import numpy as np

_root = Path(__file__).parent.parent.parent
sys.path.insert(0, str(_root / "src" / "negative_obstacle_common"))
sys.path.insert(0, str(_root / "src" / "negative_obstacle_perception"))
sys.path.insert(0, str(_root / "src" / "negative_obstacle_stvl"))

from negative_obstacle_common.camera_utils import CameraIntrinsics
from negative_obstacle_common.risk_labels import HARD_NEGATIVE_SCENE_TYPES
from negative_obstacle_common.metrics import compute_all_metrics
from negative_obstacle_perception.geometric_risk_analyzer import GeometricRiskAnalyzer
from negative_obstacle_stvl.temporal_persistence import TemporalPointAccumulator
from negative_obstacle_stvl.edge_to_pointcloud import edge_mask_to_3d_points

from ablation_config import ABLATION_GROUPS, AblationConfig


def _load_sample(stem: str, raw_dir: Path, labels_dir: Path) -> tuple | None:
    depth_p  = raw_dir    / f"{stem}_depth.npy"
    risk_p   = labels_dir / f"{stem}_risk_mask.png"
    edge_p   = labels_dir / f"{stem}_edge_mask.png"
    meta_p   = raw_dir    / f"{stem}_meta.json"
    if not all(p.exists() for p in [depth_p, risk_p, edge_p, meta_p]):
        return None
    depth    = np.load(str(depth_p))
    gt_risk  = cv2.imread(str(risk_p), cv2.IMREAD_GRAYSCALE)
    gt_edge  = cv2.imread(str(edge_p), cv2.IMREAD_GRAYSCALE)
    with open(meta_p) as f:
        meta = json.load(f)
    if gt_risk is None or gt_edge is None:
        return None
    return depth, gt_risk, gt_edge, meta


def _collect_stems(raw_dir: Path, max_samples: int) -> list[str]:
    return [
        f.name.replace("_depth.npy", "")
        for f in sorted(raw_dir.glob("*_depth.npy"))
    ][:max_samples]


def evaluate_config(
    cfg: AblationConfig,
    stems: list[str],
    raw_dir: Path,
    labels_dir: Path,
) -> dict:
    """Tek ablation konfigürasyonunu tüm dataset üzerinde değerlendir."""

    pos_recalls: list[float] = []
    pos_fsrates: list[float] = []
    pos_precisions: list[float] = []
    pos_edge_f1s: list[float] = []
    hn_fprs: list[float] = []
    latencies_ms: list[float] = []
    processed = 0

    # Temporal biriktiricisi (konfigürasyon bazlı)
    accum: TemporalPointAccumulator | None = None
    if cfg.use_temporal_persistence:
        accum = TemporalPointAccumulator(window_sec=cfg.temporal_window_sec, voxel_size=0.05)

    prev_intr: CameraIntrinsics | None = None
    prev_analyzer: GeometricRiskAnalyzer | None = None

    for stem in stems:
        sample = _load_sample(stem, raw_dir, labels_dir)
        if sample is None:
            continue
        depth, gt_risk, gt_edge, meta = sample

        h, w = depth.shape
        scene_type = meta.get("scene_type", "unknown")
        is_hn = scene_type in HARD_NEGATIVE_SCENE_TYPES

        # Intrinsics — boyut değişince yenile
        if prev_intr is None or prev_intr.width != w or prev_intr.height != h:
            prev_intr = CameraIntrinsics.from_fov(w, h, fov_h_deg=70.0)
            prev_analyzer = GeometricRiskAnalyzer(
                intrinsics=prev_intr,
                jump_threshold_m=cfg.jump_threshold_m,
                drop_threshold_m=cfg.drop_threshold_m,
                dilation_drop_px=cfg.dilation_drop_px,
                dilation_edge_px=cfg.dilation_edge_px,
                use_ground_estimator=cfg.use_ground_estimator,
            )

        # Analiz + gecikme ölçümü
        t0 = perf_counter()
        if cfg.use_ground_estimator:
            result = prev_analyzer.analyze(depth, rng_seed=0)
        else:
            result = prev_analyzer.analyze_edge_only(depth)
        t1 = perf_counter()
        latencies_ms.append((t1 - t0) * 1000)

        pred_risk = result["risk_mask"]
        pred_edge = result["edge_mask"]

        # Temporal birikim — risk mask'i temporal nokta yoğunluğuna göre güncelle
        if accum is not None:
            pts = edge_mask_to_3d_points(pred_edge, depth, prev_intr)
            accum.add(pts)
            # Biriktirilmiş nokta yoğunluğuna göre risk maskesi güçlendir
            if len(accum.get_accumulated()) > 0:
                # Eğer temporal birikim varsa drop bölgelerini genişlet
                import cv2 as _cv2
                k = _cv2.getStructuringElement(_cv2.MORPH_ELLIPSE, (5, 5))
                drop_region = (pred_risk == 3).astype(np.uint8)
                expanded = _cv2.dilate(drop_region, k)
                pred_risk = pred_risk.copy()
                pred_risk[expanded.astype(bool)] = 3

        metrics = compute_all_metrics(
            pred_mask=pred_risk,
            gt_mask=gt_risk,
            pred_edge=pred_edge,
            gt_edge=gt_edge,
            is_hard_negative=is_hn,
        )

        if is_hn:
            v = metrics.get("hard_negative_fpr", float("nan"))
            if v == v:
                hn_fprs.append(v)
        else:
            for lst, key in [
                (pos_recalls,    "drop_recall"),
                (pos_fsrates,    "false_safe_rate"),
                (pos_precisions, "drop_precision"),
                (pos_edge_f1s,   "edge_f1"),
            ]:
                v = metrics.get(key, float("nan"))
                if v == v:
                    lst.append(v)

        processed += 1

    def _m(lst: list[float]) -> float:
        return round(float(np.mean(lst)), 4) if lst else float("nan")

    return {
        "config_name":  cfg.name,
        "description":  cfg.description,
        "total":        processed,
        "positive": {
            "count":              len(pos_recalls),
            "drop_recall":        _m(pos_recalls),
            "false_safe_rate":    _m(pos_fsrates),
            "drop_precision":     _m(pos_precisions),
            "edge_f1":            _m(pos_edge_f1s),
        },
        "hard_negative": {
            "count": len(hn_fprs),
            "fpr":   _m(hn_fprs),
        },
        "latency_ms": {
            "mean": round(float(np.mean(latencies_ms)), 2) if latencies_ms else float("nan"),
            "p95":  round(float(np.percentile(latencies_ms, 95)), 2) if latencies_ms else float("nan"),
        },
    }


def _markdown_table(results: list[dict]) -> str:
    lines = [
        "# WP5 Ablation Karşılaştırma Sonuçları",
        "",
        "## Ana Metrikler",
        "",
        "| Grup | Drop Recall ↑ | False Safe Rate ↓ | Drop Precision ↑ | Edge F1 ↑ | HN-FPR ↓ | Gecikme (ms) |",
        "|------|:---:|:---:|:---:|:---:|:---:|:---:|",
    ]
    for r in results:
        pos = r["positive"]
        hn  = r["hard_negative"]
        lat = r["latency_ms"]
        lines.append(
            f"| **{r['config_name']}** "
            f"| {pos['drop_recall']:.4f} "
            f"| {pos['false_safe_rate']:.4f} "
            f"| {pos['drop_precision']:.4f} "
            f"| {pos['edge_f1']:.4f} "
            f"| {hn['fpr']:.4f} "
            f"| {lat['mean']:.1f} |"
        )

    lines += [
        "",
        "## Konfigürasyon Açıklamaları",
        "",
    ]
    for r in results:
        lines.append(f"- **{r['config_name']}**: {r['description']}")

    lines += [
        "",
        "## Yorumlar",
        "",
        "- **Drop Recall** ve **False Safe Rate** birbirinin tamamlayıcısıdır (FSR = 1 - Recall).",
        "- **False Safe Rate** makale için birincil güvenlik metriğidir.",
        "- **HN-FPR**: Hard-negative sahnelerde yanlış drop işaretleme oranı.",
        "- **Gecikme**: Ortalama tek-frame işleme süresi (ms).",
    ]
    return "\n".join(lines) + "\n"


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="WP5 Ablation karşılaştırma.")
    parser.add_argument("--data-dir",    type=Path, default=Path("data"))
    parser.add_argument("--output-dir",  type=Path, default=Path("experiments/results"))
    parser.add_argument("--max-samples", type=int,  default=500)
    return parser.parse_args()


def main() -> None:
    args = _parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)

    raw_dir    = args.data_dir / "raw"
    labels_dir = args.data_dir / "labels"
    stems      = _collect_stems(raw_dir, args.max_samples)

    print(f"Ablation karşılaştırması başlıyor — {len(stems)} örnek, {len(ABLATION_GROUPS)} grup")
    print("-" * 60)

    all_results = []
    for cfg in ABLATION_GROUPS:
        print(f"  {cfg.name} ...", end=" ", flush=True)
        t0 = perf_counter()
        res = evaluate_config(cfg, stems, raw_dir, labels_dir)
        t1 = perf_counter()
        all_results.append(res)
        pos = res["positive"]
        print(
            f"Recall={pos['drop_recall']:.3f}  "
            f"FSR={pos['false_safe_rate']:.3f}  "
            f"HN-FPR={res['hard_negative']['fpr']:.3f}  "
            f"[{t1-t0:.1f}s]"
        )

    # JSON kaydet
    json_path = args.output_dir / "ablation_comparison.json"
    with open(json_path, "w") as f:
        json.dump(all_results, f, indent=2, ensure_ascii=False)
    print(f"\nJSON: {json_path}")

    # Markdown kaydet
    md_path = args.output_dir / "ablation_comparison.md"
    md_path.write_text(_markdown_table(all_results), encoding="utf-8")
    print(f"Markdown: {md_path}")

    # En iyi konfigürasyonu bul (False Safe Rate bazlı)
    best = min(all_results, key=lambda r: r["positive"]["false_safe_rate"])
    print(f"\nEn düşük False Safe Rate: {best['config_name']} → {best['positive']['false_safe_rate']:.4f}")


if __name__ == "__main__":
    main()
