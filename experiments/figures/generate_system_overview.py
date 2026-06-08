"""
Figür 1: Sistem blok diyagramı (matplotlib programatik).

Üretim:
  python3 experiments/figures/generate_system_overview.py

Çıktı:
  experiments/figures/fig1_system_overview.pdf + .png
  manuscript/figures/fig1_system_overview.pdf  (otomatik kopyalanır)
"""

from __future__ import annotations

import shutil
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch

ROOT = Path(__file__).parent.parent.parent
OUT_PDF = ROOT / "experiments" / "figures" / "fig1_system_overview.pdf"
OUT_PNG = ROOT / "experiments" / "figures" / "fig1_system_overview.png"
MANUSCRIPT_PDF = ROOT / "manuscript" / "figures" / "fig1_system_overview.pdf"

# Renk paleti
COL_INPUT  = "#A8D8EA"   # açık mavi: girdi/sensör
COL_NODE   = "#FFD3B6"   # açık turuncu: ROS node
COL_TOPIC  = "#E8E8E8"   # gri: topic
COL_OUTPUT = "#FFAAA5"   # mercan: çıktı/STVL
COL_TEXT   = "#222222"


def add_box(ax, xy, w, h, text, color, fontsize=9, bold=False):
    x, y = xy
    box = FancyBboxPatch((x, y), w, h,
                         boxstyle="round,pad=0.02,rounding_size=0.05",
                         linewidth=1.0, edgecolor="black", facecolor=color, zorder=2)
    ax.add_patch(box)
    weight = "bold" if bold else "normal"
    ax.text(x + w/2, y + h/2, text, ha="center", va="center",
            fontsize=fontsize, fontweight=weight, color=COL_TEXT, zorder=3)
    return (x, y, w, h)


def arrow(ax, src, dst, label=None, dy=0):
    sx = src[0] + src[2]
    sy = src[1] + src[3]/2
    dx = dst[0]
    dy_ = dst[1] + dst[3]/2 + dy
    ar = FancyArrowPatch((sx, sy), (dx, dy_),
                         arrowstyle="-|>", mutation_scale=14,
                         linewidth=1.2, color="#333333", zorder=1)
    ax.add_patch(ar)
    if label:
        mx = (sx + dx) / 2
        my = (sy + dy_) / 2
        ax.text(mx, my + 0.06, label, ha="center", va="bottom",
                fontsize=7.5, color="#444444",
                bbox=dict(boxstyle="round,pad=0.1", fc="white", ec="none", alpha=0.8))


def main() -> None:
    fig, ax = plt.subplots(figsize=(10, 4))
    ax.set_xlim(0, 11)
    ax.set_ylim(0, 4)
    ax.axis("off")

    # === Üst sıra: ana boru hattı ===
    cam     = add_box(ax, (0.2, 1.7), 1.6, 0.8, "RGB-D\nCamera",        COL_INPUT, fontsize=9, bold=True)
    risk    = add_box(ax, (2.4, 1.7), 1.7, 0.8, "Geometric Risk\n(RANSAC + DDM)", COL_NODE, fontsize=8.5)
    edge    = add_box(ax, (4.7, 1.7), 1.5, 0.8, "Edge\nExtraction",     COL_NODE, fontsize=9)
    proj    = add_box(ax, (6.7, 1.7), 1.7, 0.8, "Edge → 3D\n(intrinsics + TF)", COL_NODE, fontsize=8.5)
    temp    = add_box(ax, (9.0, 1.7), 1.7, 0.8, "Temporal\nPersistence (1.5 s)", COL_NODE, fontsize=8.5)

    # === Alt sıra: STVL ve Nav2 ===
    stvl    = add_box(ax, (6.7, 0.3), 1.7, 0.8, "STVL\n(2nd source)", COL_OUTPUT, fontsize=9, bold=True)
    costmap = add_box(ax, (9.0, 0.3), 1.7, 0.8, "Local Costmap\n(lethal marks)", COL_OUTPUT, fontsize=9, bold=True)

    # === Oklar (üst sıra) ===
    arrow(ax, cam,  risk,  "depth\n/camera/depth")
    arrow(ax, risk, edge,  "/risk/\ngeometric_mask")
    arrow(ax, edge, proj,  "edge\nmask")
    arrow(ax, proj, temp,  "PointCloud2")

    # === Alt sıra ok: temp → stvl, stvl → costmap ===
    # temp'ten stvl'ye dikey ok
    sx = temp[0] + temp[2]/2
    sy = temp[1]
    dx = stvl[0] + stvl[2]/2
    dy_ = stvl[1] + stvl[3]
    ar = FancyArrowPatch((sx, sy), (dx, dy_),
                         arrowstyle="-|>", mutation_scale=14,
                         linewidth=1.2, color="#333333",
                         connectionstyle="arc3,rad=0.25", zorder=1)
    ax.add_patch(ar)
    ax.text(8.7, 1.45, "/semantic_drop_points", ha="center", va="center",
            fontsize=7.5, color="#444444",
            bbox=dict(boxstyle="round,pad=0.15", fc="white", ec="#888"))

    arrow(ax, stvl, costmap, "marking")

    # === Başlıklar ===
    ax.text(5.5, 3.55, "Negative Obstacle Detection Pipeline",
            ha="center", va="center", fontsize=12, fontweight="bold")
    ax.text(5.5, 3.20, "(STVL parameter-only integration; no C++ core changes)",
            ha="center", va="center", fontsize=9, color="#555")

    # === Legend ===
    legend_patches = [
        mpatches.Patch(color=COL_INPUT,  label="Sensor input"),
        mpatches.Patch(color=COL_NODE,   label="Processing node"),
        mpatches.Patch(color=COL_OUTPUT, label="Nav2 / STVL"),
    ]
    ax.legend(handles=legend_patches, loc="lower left",
              bbox_to_anchor=(0.0, -0.02), fontsize=8.5,
              ncol=3, frameon=False)

    plt.tight_layout()
    fig.savefig(OUT_PDF, dpi=300, bbox_inches="tight")
    fig.savefig(OUT_PNG, dpi=150, bbox_inches="tight")
    print(f"Kaydedildi: {OUT_PDF}")
    print(f"Kaydedildi: {OUT_PNG}")

    # manuscript/figures'a otomatik kopyala
    if MANUSCRIPT_PDF.parent.exists():
        shutil.copy(OUT_PDF, MANUSCRIPT_PDF)
        print(f"Kopyalandı:  {MANUSCRIPT_PDF}")


if __name__ == "__main__":
    main()
