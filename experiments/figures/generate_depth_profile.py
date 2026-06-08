"""
Figür 4: Pit sahnesinde orta satır derinlik profili.

Kaynak: experiments/results/ veya /tmp/gazebo_depth_test/pit_scene_0000_depth.npy
        experiments/results/ablation_comparison.json (risk label bilgisi için)
Çıktı : experiments/figures/fig4_depth_profile.pdf + .png
"""

from __future__ import annotations

from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import numpy as np

ROOT = Path(__file__).parent.parent.parent
OUT_PDF = ROOT / "experiments" / "figures" / "fig4_depth_profile.pdf"
OUT_PNG = ROOT / "experiments" / "figures" / "fig4_depth_profile.png"

# Gazebo'dan toplanan veri yolları (önce gerçek, sonra sentetik fallback)
DEPTH_CANDIDATES = [
    Path("/tmp/gazebo_depth_test/pit_scene_0000_depth.npy"),
    ROOT / "data" / "processed" / "pit_scene_0000_depth.npy",
]


def find_depth() -> np.ndarray | None:
    for p in DEPTH_CANDIDATES:
        if p.exists():
            return np.load(str(p))
    return None


def make_profile(depth: np.ndarray) -> None:
    h, w = depth.shape
    mid_row = h // 2

    row = depth[mid_row, :].copy()
    xs = np.arange(w)

    # inf → NaN (görsel için)
    row_plot = np.where(np.isfinite(row), row, np.nan)

    # Bölge renklendirme: basit eşik ile
    safe_mask = np.isfinite(row) & (row < 2.0)
    drop_mask = np.isfinite(row) & (row >= 2.0)
    unc_mask  = ~np.isfinite(row)

    fig, ax = plt.subplots(figsize=(7, 3.5))

    # Bölge arka planları
    for i in xs:
        if safe_mask[i]:
            ax.axvspan(i - 0.5, i + 0.5, color="#90ee90", alpha=0.3, linewidth=0)
        elif drop_mask[i]:
            ax.axvspan(i - 0.5, i + 0.5, color="#ff9999", alpha=0.3, linewidth=0)
        elif unc_mask[i]:
            ax.axvspan(i - 0.5, i + 0.5, color="#cccccc", alpha=0.3, linewidth=0)

    ax.plot(xs, row_plot, color="navy", linewidth=1.5, label="Depth (m)")
    ax.set_xlabel("Pixel column (u)", fontsize=10)
    ax.set_ylabel("Depth (m)", fontsize=10)
    ax.set_title(f"Depth Profile at Row y={mid_row} — Pit Scene\n"
                 "Green: SAFE, Red: UNSAFE_DROP, Grey: UNCERTAIN", fontsize=9)
    ax.set_xlim(0, w - 1)
    ax.set_ylim(0, 4.5)
    ax.grid(linestyle="--", alpha=0.4)

    patches = [
        mpatches.Patch(color="#90ee90", alpha=0.6, label="SAFE"),
        mpatches.Patch(color="#ff9999", alpha=0.6, label="UNSAFE_DROP"),
        mpatches.Patch(color="#cccccc", alpha=0.6, label="UNCERTAIN (inf)"),
    ]
    ax.legend(handles=patches, fontsize=8, loc="upper left")

    plt.tight_layout()
    fig.savefig(OUT_PDF, dpi=300, bbox_inches="tight")
    fig.savefig(OUT_PNG, dpi=150, bbox_inches="tight")
    print(f"Kaydedildi: {OUT_PDF}")
    print(f"Kaydedildi: {OUT_PNG}")


def main() -> None:
    depth = find_depth()
    if depth is None:
        print("HATA: Derinlik dosyası bulunamadı.")
        print("Önce Gazebo veri toplama pipeline'ını çalıştırın:")
        print("  DISPLAY=:0 gz sim src/negative_obstacle_bringup/worlds/pit_scene.sdf -r -s")
        print("  ros2 launch negative_obstacle_bringup gazebo_data_collection.launch.py ...")
        return
    make_profile(depth)


if __name__ == "__main__":
    main()
