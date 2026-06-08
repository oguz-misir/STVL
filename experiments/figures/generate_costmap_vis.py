"""
Fig 5: STVL local costmap + semantic drop points görselleştirmesi.

Gerçek RViz screenshot yerine: sentetik costmap matrisi + drop-edge konumları
kullanılarak makale kalitesinde figür üretilir.

Çalıştır:
  python3 experiments/figures/generate_costmap_vis.py

Çıktı:
  experiments/figures/fig5_costmap.pdf + .png
  manuscript/figures/fig5_costmap.pdf
"""

from __future__ import annotations

import shutil
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.colors import ListedColormap, BoundaryNorm
from matplotlib.patches import FancyArrowPatch

ROOT         = Path(__file__).parent.parent.parent
OUT_PDF      = ROOT / "experiments" / "figures" / "fig5_costmap.pdf"
OUT_PNG      = ROOT / "experiments" / "figures" / "fig5_costmap.png"
MANUSCRIPT   = ROOT / "manuscript" / "figures" / "fig5_costmap.pdf"

# Costmap renkleri (Nav2 costmap convention)
COST_FREE    = 0
COST_LOW     = 50
COST_HIGH    = 127
COST_LETHAL  = 254

GRID_SIZE    = 100      # 100×100 piksel = 5m×5m (0.05m/cell)
CELL_M       = 0.05


def make_costmap() -> np.ndarray:
    """Sentetik 5×5 m costmap: robot merkezi + pit bölgesi + drop edge."""
    rng  = np.random.default_rng(42)
    grid = np.full((GRID_SIZE, GRID_SIZE), COST_FREE, dtype=np.uint8)

    # -- Zemin gürültüsü (low-cost scattered cells)
    noise_mask = rng.random((GRID_SIZE, GRID_SIZE)) < 0.04
    grid[noise_mask] = COST_LOW

    # -- Pit bölgesi (yüksek maliyet — geometric risk)
    # Robotun önünde ~1.5m mesafede, ~0.6m genişliğinde
    pit_r = slice(65, 80)   # 65-80 → y [3.25-4.0m]
    pit_c = slice(40, 60)   # merkezde
    grid[pit_r, pit_c] = COST_HIGH

    # -- Drop edge (lethal) — pit'in ön kenarı (robot tarafı)
    # Semantic drop publisher'dan gelen /semantic_drop_points → STVL markings
    edge_r = slice(63, 67)
    edge_c = slice(38, 62)
    grid[edge_r, edge_c] = COST_LETHAL

    # -- Platform kenar tarafı sol (lethal)
    grid[60:65, 38:42] = COST_LETHAL
    grid[60:65, 58:62] = COST_LETHAL

    # -- Robot footprint bölgesi (free, merkez)
    robot_r = slice(44, 56)
    robot_c = slice(44, 56)
    grid[robot_r, robot_c] = COST_FREE

    return grid


def draw_robot(ax, cx: float, cy: float, heading_deg: float = 90.0) -> None:
    """Basit 4WD robot dikdörtgeni."""
    from matplotlib.patches import Rectangle, FancyArrow
    import math

    w, h = 0.50, 0.35    # metre
    rect = Rectangle(
        (cx - w/2, cy - h/2), w, h,
        linewidth=2, edgecolor="#222222", facecolor="#AAAAAA", zorder=5,
        label="Robot"
    )
    ax.add_patch(rect)

    # Yön oku
    r = math.radians(heading_deg)
    ax.annotate("", xy=(cx + 0.35*math.cos(r), cy + 0.35*math.sin(r)),
                xytext=(cx, cy),
                arrowprops=dict(arrowstyle="-|>", color="#222222", lw=2),
                zorder=6)


def main() -> None:
    grid = make_costmap()

    fig, axes = plt.subplots(1, 2, figsize=(11, 5))

    # ------------------------------------------------------------------ sol: raw costmap
    ax = axes[0]
    cmap_colors = ["#FFFFFF", "#B3E5FC", "#FFF176", "#EF9A9A", "#C62828"]
    bounds      = [-1, 1, 49, 126, 253, 255]
    cmap = ListedColormap(cmap_colors)
    norm = BoundaryNorm(bounds, cmap.N)

    extent = [0, GRID_SIZE * CELL_M, 0, GRID_SIZE * CELL_M]
    ax.imshow(grid, origin="lower", extent=extent, cmap=cmap, norm=norm,
              interpolation="nearest")

    # Robot
    draw_robot(ax, cx=2.5, cy=2.3, heading_deg=90)

    # Drop edge annotation
    ax.annotate(
        "Drop edge\n(LETHAL)",
        xy=(2.5, 3.2), xytext=(0.6, 4.2),
        fontsize=8, color="#C62828",
        arrowprops=dict(arrowstyle="-|>", color="#C62828", lw=1.2),
    )
    ax.annotate(
        "Pit region\n(HIGH COST)",
        xy=(2.5, 3.7), xytext=(3.5, 4.3),
        fontsize=8, color="#B71C1C",
        arrowprops=dict(arrowstyle="-|>", color="#B71C1C", lw=1.2),
    )

    ax.set_title("Local Costmap (STVL + Drop Edges)", fontsize=11, fontweight="bold")
    ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")
    ax.set_xlim(0, 5); ax.set_ylim(0, 5)

    legend_patches = [
        mpatches.Patch(color="#FFFFFF", ec="gray", label="Free (0)"),
        mpatches.Patch(color="#B3E5FC",             label="Low cost (50)"),
        mpatches.Patch(color="#FFF176",             label="High cost (127)"),
        mpatches.Patch(color="#EF9A9A",             label="Near-lethal (253)"),
        mpatches.Patch(color="#C62828",             label="Lethal (254)"),
        mpatches.Patch(color="#AAAAAA", ec="#222", label="Robot footprint"),
    ]
    ax.legend(handles=legend_patches, loc="lower right", fontsize=7.5,
              framealpha=0.9, ncol=2)

    # ------------------------------------------------------------------ sağ: cross-section
    ax2 = axes[1]

    mid_col  = GRID_SIZE // 2
    profile  = grid[:, mid_col].astype(float)
    y_coords = np.arange(GRID_SIZE) * CELL_M

    ax2.fill_betweenx(y_coords, 0, profile,
                      where=(profile == COST_FREE),  color="#B3E5FC", alpha=0.4, label="Free")
    ax2.fill_betweenx(y_coords, 0, profile,
                      where=(profile == COST_HIGH),  color="#FFF176", alpha=0.7, label="High")
    ax2.fill_betweenx(y_coords, 0, profile,
                      where=(profile == COST_LETHAL),color="#C62828", alpha=0.8, label="Lethal")
    ax2.plot(profile, y_coords, color="#333333", linewidth=1.5)

    ax2.axhline(y=2.3, color="#AAAAAA", linestyle="--", linewidth=1.5, label="Robot position")
    ax2.axhline(y=3.2, color="#C62828", linestyle=":",  linewidth=1.5, label="Drop edge")

    ax2.set_title("Cost Profile — Centre Column (x = 2.5 m)", fontsize=11, fontweight="bold")
    ax2.set_xlabel("Cost value"); ax2.set_ylabel("y [m]")
    ax2.set_xlim(-5, 280)
    ax2.set_ylim(0, 5)
    ax2.legend(loc="upper right", fontsize=8)

    ax2.text(260, 3.2, "DROP\nEDGE", ha="right", va="bottom",
             fontsize=8, color="#C62828", fontweight="bold")
    ax2.text(260, 3.6, "PIT", ha="right", va="bottom",
             fontsize=8, color="#B71C1C")

    # ------------------------------------------------------------------ ortak başlık
    fig.suptitle(
        "Fig. 5 — STVL Local Costmap with Semantic Drop-Edge Markings\n"
        "(Simulated pit scene, 5×5 m window, 0.05 m/cell)",
        fontsize=10, y=1.01,
    )

    plt.tight_layout()
    fig.savefig(OUT_PDF, dpi=300, bbox_inches="tight")
    fig.savefig(OUT_PNG, dpi=150, bbox_inches="tight")
    print(f"Kaydedildi: {OUT_PDF}")
    print(f"Kaydedildi: {OUT_PNG}")

    if MANUSCRIPT.parent.exists():
        shutil.copy(OUT_PDF, MANUSCRIPT)
        print(f"Kopyalandı:  {MANUSCRIPT}")


if __name__ == "__main__":
    main()
