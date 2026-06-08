"""
Sentetik dataset kalite kontrol (QC) rapor üreticisi.

JSON ve Markdown formatında makale için istatistik raporu üretir.
"""

from __future__ import annotations

import argparse
import json
from collections import defaultdict
from pathlib import Path

import cv2
import numpy as np


RISK_LABEL_NAMES = {0: "SAFE", 1: "UNCERTAIN", 2: "UNSAFE_SOLID", 3: "UNSAFE_DROP"}

HARD_NEGATIVE_TYPES = {
    "ramp_hard_negative",
    "shadow_hard_negative",
    "dark_floor_hard_negative",
    "reflective_floor_hard_negative",
    "uneven_ground_hard_negative",
    "sloped_ground_hard_negative",
}


def _load_splits(splits_dir: Path) -> dict[str, set[str]]:
    result: dict[str, set[str]] = {}
    for split_name in ("train", "val", "test"):
        txt = splits_dir / f"{split_name}.txt"
        if txt.exists():
            stems = {s.strip() for s in txt.read_text().splitlines() if s.strip()}
        else:
            stems = set()
        result[split_name] = stems
    return result


def _check_scene_leakage(splits: dict[str, set[str]]) -> bool:
    """Split'ler arasında scene leakage var mı kontrol et. True = temiz."""
    train, val, test = splits.get("train", set()), splits.get("val", set()), splits.get("test", set())
    return not (train & val or train & test or val & test)


def compute_qc_report(
    data_dir: Path,
) -> dict:
    """Tüm QC istatistiklerini hesapla ve dict olarak döndür."""
    raw_dir = data_dir / "raw"
    labels_dir = data_dir / "labels"
    splits_dir = data_dir / "splits"

    metas: list[dict] = []
    for meta_path in sorted(raw_dir.glob("*_meta.json")):
        try:
            with open(meta_path) as f:
                metas.append(json.load(f))
        except (json.JSONDecodeError, OSError):
            continue

    total_samples = len(metas)
    scene_ids: set[str] = {m["scene_id"] for m in metas}
    total_scenes = len(scene_ids)

    scene_type_counts: dict[str, int] = defaultdict(int)
    has_neg_count = 0
    hard_neg_count = 0
    label_pixel_totals: dict[int, int] = defaultdict(int)
    total_pixels = 0
    edge_ratios: list[float] = []
    bad_hard_neg: list[str] = []

    for meta in metas:
        st = meta.get("scene_type", "unknown")
        scene_type_counts[st] += 1
        if meta.get("has_negative_obstacle", False):
            has_neg_count += 1
        if st in HARD_NEGATIVE_TYPES:
            hard_neg_count += 1

        stem = meta.get("scene_id", "")

        # Risk mask pixel sayımı
        risk_path = labels_dir / f"{stem}_risk_mask.png"
        if risk_path.exists():
            mask = cv2.imread(str(risk_path), cv2.IMREAD_GRAYSCALE)
            if mask is not None:
                for label in range(4):
                    cnt = int(np.sum(mask == label))
                    label_pixel_totals[label] += cnt
                total_pixels += mask.size

                # Hard-negative sahnelerde UNSAFE_DROP varsa uyarı
                if st in HARD_NEGATIVE_TYPES and np.any(mask == 3):
                    bad_hard_neg.append(stem)

        # Edge mask pixel oranı
        edge_path = labels_dir / f"{stem}_edge_mask.png"
        if edge_path.exists():
            edge = cv2.imread(str(edge_path), cv2.IMREAD_GRAYSCALE)
            if edge is not None and edge.size > 0:
                edge_ratios.append(float(np.sum(edge > 127)) / edge.size)

    # Split istatistikleri
    splits = _load_splits(splits_dir)
    split_counts = {k: len(v) for k, v in splits.items()}
    leakage_ok = _check_scene_leakage(splits)

    # Piksel oranları
    label_pixel_ratios: dict[str, float] = {}
    for label, name in RISK_LABEL_NAMES.items():
        if total_pixels > 0:
            label_pixel_ratios[name] = round(label_pixel_totals[label] / total_pixels, 4)
        else:
            label_pixel_ratios[name] = 0.0

    report = {
        "total_samples": total_samples,
        "total_scenes": total_scenes,
        "scene_type_distribution": dict(scene_type_counts),
        "has_negative_obstacle_count": has_neg_count,
        "hard_negative_count": hard_neg_count,
        "label_pixel_ratios": label_pixel_ratios,
        "average_edge_pixel_ratio": round(float(np.mean(edge_ratios)), 4) if edge_ratios else 0.0,
        "split_distribution": split_counts,
        "scene_leakage_clean": leakage_ok,
        "warnings": {
            "hard_negative_with_unsafe_drop": bad_hard_neg,
        },
    }
    return report


def _report_to_markdown(report: dict) -> str:
    lines = [
        "# Sentetik Dataset QC Raporu",
        "",
        "## Genel İstatistikler",
        "",
        f"| Alan | Değer |",
        f"|------|-------|",
        f"| Toplam örnek | {report['total_samples']} |",
        f"| Toplam scene | {report['total_scenes']} |",
        f"| Negatif engel içeren | {report['has_negative_obstacle_count']} |",
        f"| Hard-negative | {report['hard_negative_count']} |",
        f"| Ortalama edge piksel oranı | {report['average_edge_pixel_ratio']:.4f} |",
        f"| Scene leakage temiz | {'Evet' if report['scene_leakage_clean'] else 'HAYIR'} |",
        "",
        "## Scene Tipi Dağılımı",
        "",
        "| Scene Tipi | Sayı |",
        "|------------|------|",
    ]
    for st, cnt in sorted(report["scene_type_distribution"].items()):
        lines.append(f"| {st} | {cnt} |")

    lines += [
        "",
        "## Risk Label Piksel Oranları",
        "",
        "| Label | Oran |",
        "|-------|------|",
    ]
    for name, ratio in report["label_pixel_ratios"].items():
        lines.append(f"| {name} | {ratio:.4f} |")

    lines += [
        "",
        "## Train / Val / Test Dağılımı",
        "",
        "| Split | Örnek Sayısı |",
        "|-------|-------------|",
    ]
    for split, cnt in report["split_distribution"].items():
        lines.append(f"| {split} | {cnt} |")

    bad = report["warnings"]["hard_negative_with_unsafe_drop"]
    if bad:
        lines += [
            "",
            "## UYARI: Hard-Negative'de UNSAFE_DROP Etiketi",
            "",
            "Aşağıdaki örneklerde hard-negative sahnede UNSAFE_DROP etiketi tespit edildi:",
            "",
        ]
        for b in bad:
            lines.append(f"- `{b}`")

    return "\n".join(lines) + "\n"


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Sentetik dataset QC rapor üreticisi."
    )
    parser.add_argument("--data-dir", type=Path, default=Path("data"))
    parser.add_argument(
        "--output-json",
        type=Path,
        default=Path("experiments/results/synthetic_dataset_qc.json"),
    )
    parser.add_argument(
        "--output-md",
        type=Path,
        default=None,
        help="Opsiyonel Markdown rapor yolu. Verilmezse sadece JSON yazilir.",
    )
    return parser.parse_args()


def main() -> None:
    args = _parse_args()

    print("QC raporu hesaplaniyor...")
    report = compute_qc_report(args.data_dir)

    # JSON çıktısı
    args.output_json.parent.mkdir(parents=True, exist_ok=True)
    with open(args.output_json, "w") as f:
        json.dump(report, f, indent=2, ensure_ascii=False)
    print(f"JSON rapor yazildi: {args.output_json}")

    # Markdown çıktısı opsiyoneldir; proje dokumantasyonu tek ana MD dosyasinda tutulur.
    if args.output_md is not None:
        args.output_md.parent.mkdir(parents=True, exist_ok=True)
        md = _report_to_markdown(report)
        args.output_md.write_text(md, encoding="utf-8")
        print(f"Markdown rapor yazildi: {args.output_md}")

    # Özet konsola
    print(f"\n  Toplam ornek     : {report['total_samples']}")
    print(f"  Toplam scene     : {report['total_scenes']}")
    print(f"  Hard-negative    : {report['hard_negative_count']}")
    print(f"  Scene leakage    : {'Temiz' if report['scene_leakage_clean'] else 'HATA!'}")

    bad = report["warnings"]["hard_negative_with_unsafe_drop"]
    if bad:
        print(f"\n  UYARI: {len(bad)} hard-negative ornekte UNSAFE_DROP etiketi var!")
        for b in bad:
            print(f"    - {b}")


if __name__ == "__main__":
    main()
