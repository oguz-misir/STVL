"""
Figür 2: Ablation bar chart üretici.

Kaynak: experiments/results/ablation_comparison.json
Çıktı : experiments/figures/fig2_ablation_bars.pdf + .png
"""

from __future__ import annotations

import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import numpy as np

# ---------------------------------------------------------------------------
ROOT = Path(__file__).parent.parent.parent
DATA = ROOT / "experiments" / "results" / "ablation_comparison.json"
OUT_PDF = ROOT / "experiments" / "figures" / "fig2_ablation_bars.pdf"
OUT_PNG = ROOT / "experiments" / "figures" / "fig2_ablation_bars.png"


def load_data() -> list:
    with open(DATA) as f:
        return json.load(f)


def make_chart(data: list) -> None:
    # Kısa etiketler
    short_labels = ["A1\nEdge Only", "A2\nGeom", "A3\nGeom+Temp", "A4\nAggressive", "A5\nConservative"]

    fsr    = [g["positive"]["false_safe_rate"] for g in data]
    recall = [g["positive"]["drop_recall"]      for g in data]
    hnfpr  = [g["hard_negative"]["fpr"]         for g in data]

    x = np.arange(len(data))
    width = 0.26

    fig, ax = plt.subplots(figsize=(8, 4.5))

    bars_fsr    = ax.bar(x - width, fsr,    width, label="False Safe Rate ↓", color="#d62728", zorder=3)
    bars_recall = ax.bar(x,         recall, width, label="Drop Recall ↑",     color="#2ca02c", zorder=3)
    bars_hn     = ax.bar(x + width, hnfpr,  width, label="HN-FPR ↓",          color="#ff7f0e", zorder=3)

    # A3 sütununu vurgula
    a3_idx = 2
    for bar_group in [bars_fsr, bars_recall, bars_hn]:
        bar_group[a3_idx].set_edgecolor("black")
        bar_group[a3_idx].set_linewidth(2)

    ax.set_xticks(x)
    ax.set_xticklabels(short_labels, fontsize=9)
    ax.set_ylim(0, 1.05)
    ax.set_ylabel("Metric Value", fontsize=10)
    ax.set_title("Ablation Study: Negative Obstacle Detection Performance\n"
                 "(★ = selected operational configuration)", fontsize=10)
    ax.legend(fontsize=9, loc="upper right")
    ax.grid(axis="y", linestyle="--", alpha=0.5, zorder=0)

    # A3 seçim işareti
    ax.annotate("★", xy=(a3_idx, 1.0), ha="center", fontsize=14, color="black")

    # Değer etiketleri (yalnızca FSR)
    for i, (bar, val) in enumerate(zip(bars_fsr, fsr)):
        ax.text(bar.get_x() + bar.get_width() / 2, val + 0.02,
                f"{val:.2f}", ha="center", va="bottom", fontsize=7.5, color="#d62728")

    plt.tight_layout()
    fig.savefig(OUT_PDF, dpi=300, bbox_inches="tight")
    fig.savefig(OUT_PNG, dpi=150, bbox_inches="tight")
    print(f"Kaydedildi: {OUT_PDF}")
    print(f"Kaydedildi: {OUT_PNG}")


def main() -> None:
    data = load_data()
    make_chart(data)


if __name__ == "__main__":
    main()
