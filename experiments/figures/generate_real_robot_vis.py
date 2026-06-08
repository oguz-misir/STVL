"""
Şekil 6: Gerçek robot deney görseli (yer tutucu / sentetik).

Üç panel:
  (a) Kamera perspektifinden derinlik + risk maskası katmanı
  (b) Üstten bakış: 5×5 m costmap + robot + durma noktası
  (c) Zaman serisi: drop edge piksel sayısı ve robot hızı

Çıktı:
  experiments/figures/fig6_real_robot.pdf + .png
  manuscript/figures/fig6_real_robot.pdf
"""

from __future__ import annotations

import shutil
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import matplotlib.patheffects as pe
import numpy as np

ROOT      = Path(__file__).parent.parent.parent
OUT_PDF   = ROOT / "experiments" / "figures" / "fig6_real_robot.pdf"
OUT_PNG   = ROOT / "experiments" / "figures" / "fig6_real_robot.png"
MANUS_PDF = ROOT / "manuscript" / "figures" / "fig6_real_robot.pdf"

# ── renk sabitleri ──────────────────────────────────────────────────────────
SAFE_COL  = np.array([0.15, 0.70, 0.15])
UNC_COL   = np.array([0.90, 0.85, 0.10])
SOLID_COL = np.array([0.55, 0.10, 0.80])
DROP_COL  = np.array([0.92, 0.10, 0.10])

COST_FREE    = 0
COST_INFLATE = 60
COST_HIGH    = 127
COST_LETHAL  = 254

RNG = np.random.default_rng(42)


# ── panel (a): kamera görüntüsü + risk maskası ───────────────────────────────
def make_camera_view(h: int = 240, w: int = 320):
    """Sentetik kamera perspektifi: platform kenar sahnesi."""
    depth = np.full((h, w), 0.90, dtype=float)
    edge_c = int(w * 0.58)
    depth[:, edge_c:] = 2.40
    depth[:, edge_c - 4:edge_c + 4] = np.inf
    depth[15:65, 30:100] = 0.42

    mask = np.zeros((h, w), dtype=np.uint8)
    mask[:, edge_c - 6:edge_c + 6] = 3
    mask[~np.isfinite(depth)] = 1
    mask[depth < 0.65] = 2
    mask[:, edge_c + 6:] = 1

    rgb = np.ones((h, w, 3))
    rgb[mask == 0] = SAFE_COL
    rgb[mask == 1] = UNC_COL
    rgb[mask == 2] = SOLID_COL
    rgb[mask == 3] = DROP_COL

    d_norm = np.zeros_like(depth)
    fin = np.isfinite(depth)
    if fin.any():
        lo, hi = depth[fin].min(), depth[fin].max()
        d_norm[fin] = (depth[fin] - lo) / max(hi - lo, 1e-6)
    d_norm[~fin] = 1.0
    d_norm = np.clip(d_norm, 0, 1)

    gray3 = np.stack([d_norm] * 3, axis=-1)
    blended = 0.45 * gray3 + 0.55 * rgb

    return np.clip(blended, 0, 1), mask


# ── panel (b): üstten bakış costmap ──────────────────────────────────────────
def make_topdown_costmap(size: int = 100, res: float = 0.05):
    """5×5 m rolling costmap: zemin serbest, sağda drop edge bariyer."""
    cost = np.full((size, size), COST_FREE, dtype=float)

    # platform kenarı (sağ taraf lethal)
    edge_px = 62
    cost[:, edge_px:edge_px + 4] = COST_LETHAL
    cost[:, edge_px + 4:] = COST_HIGH

    # inflation
    from scipy.ndimage import distance_transform_edt
    lethal_mask = cost >= COST_LETHAL
    dist = distance_transform_edt(~lethal_mask) * res
    infl_r = 0.35
    scale  = 3.0
    infl_mask = (dist < infl_r) & (~lethal_mask)
    cost[infl_mask] = np.clip(
        COST_LETHAL * np.exp(-scale * dist[infl_mask]), COST_INFLATE, COST_LETHAL - 1
    )

    # küçük pozitif engel
    cost[30:40, 20:28] = COST_LETHAL

    # robot gidişat yolu (güvenli bölgeye kadar)
    traj = [(50, 10), (50, 20), (50, 30), (50, 40), (50, 50), (50, 58)]
    traj_arr = np.array(traj)

    # durma noktası
    stop = (50, 55)

    return cost, traj_arr, stop


# ── panel (c): zaman serisi ───────────────────────────────────────────────────
def make_timeseries():
    t = np.linspace(0, 12, 300)

    # robot hızı: 0.35 m/s yaklaşma, yavaşla, dur
    stop_t = 8.5
    vel = np.where(t < 5.0, 0.35,
          np.where(t < stop_t, 0.35 * (1 - (t - 5.0) / (stop_t - 5.0)), 0.0))
    vel = np.clip(vel, 0, 0.35)

    # drop pixel sayısı: kenar görüldükten sonra artar ve stabil olur
    dp = np.zeros_like(t)
    dp[t >= 4.0] = 180 * (1 - np.exp(-1.5 * (t[t >= 4.0] - 4.0)))
    dp += RNG.normal(0, 5, len(t))
    dp = np.clip(dp, 0, 220)

    return t, vel, dp, stop_t


# ── ana figür ────────────────────────────────────────────────────────────────
def main() -> None:
    fig = plt.figure(figsize=(14, 4.5))
    gs  = fig.add_gridspec(1, 3, wspace=0.35)

    # ── (a) kamera görüntüsü ─────────────────────────────────────────────────
    ax_a = fig.add_subplot(gs[0])
    blend, mask = make_camera_view()
    ax_a.imshow(blend)
    ax_a.set_title("(a) Kamera Görüntüsü + Risk Maskası", fontsize=9, fontweight="bold")
    ax_a.axis("off")

    # etiket
    patches = [
        mpatches.Patch(color=SAFE_COL,  label="SAFE"),
        mpatches.Patch(color=DROP_COL,  label="UNSAFE_DROP"),
        mpatches.Patch(color=UNC_COL,   label="UNCERTAIN"),
        mpatches.Patch(color=SOLID_COL, label="UNSAFE_SOLID"),
    ]
    ax_a.legend(handles=patches, loc="lower left", fontsize=6,
                framealpha=0.8, ncol=2)

    # ── (b) costmap üstten bakış ──────────────────────────────────────────────
    ax_b = fig.add_subplot(gs[1])
    cost, traj, stop = make_topdown_costmap()

    cmap_nav = matplotlib.colors.LinearSegmentedColormap.from_list(
        "nav2", [(0, "#FFFFFF"), (0.24, "#AAFFAA"), (0.5, "#FFFF00"),
                 (0.75, "#FF8800"), (1.0, "#FF0000")]
    )
    im = ax_b.imshow(cost, cmap=cmap_nav, vmin=0, vmax=255,
                     origin="lower", aspect="equal")
    plt.colorbar(im, ax=ax_b, shrink=0.7, label="Costmap Değeri")

    # robot yolu
    ax_b.plot(traj[:, 1], traj[:, 0], "b-o", markersize=3,
              linewidth=1.5, label="Robot yolu")
    # durma noktası
    ax_b.plot(stop[1], stop[0], "b^", markersize=10,
              label="Durma noktası", zorder=5)

    # robot (başlangıç)
    robot = plt.Circle((traj[0, 1], traj[0, 0]), 2.5,
                        color="#2255FF", alpha=0.7, label="Robot")
    ax_b.add_patch(robot)

    ax_b.set_title("(b) Yerel Costmap + Robot Yolu", fontsize=9, fontweight="bold")
    ax_b.set_xlabel("x [pix × 0.05 m]", fontsize=7)
    ax_b.set_ylabel("y [pix × 0.05 m]", fontsize=7)
    ax_b.tick_params(labelsize=7)
    ax_b.legend(fontsize=7, loc="upper left")

    # annotation
    ax_b.annotate("Lethal\n(Drop Edge)", xy=(64, 50), xytext=(72, 70),
                  fontsize=7, color="white",
                  arrowprops=dict(arrowstyle="->", color="white"),
                  bbox=dict(fc="#AA0000", alpha=0.75))

    # ── (c) zaman serisi ──────────────────────────────────────────────────────
    ax_c = fig.add_subplot(gs[2])
    t, vel, dp, stop_t = make_timeseries()

    color_v = "#2255FF"
    color_d = "#CC2222"

    ax_c1 = ax_c
    ax_c2 = ax_c.twinx()

    ax_c1.plot(t, vel, color=color_v, linewidth=1.8, label="Robot hızı [m/s]")
    ax_c2.fill_between(t, dp, alpha=0.3, color=color_d)
    ax_c2.plot(t, dp, color=color_d, linewidth=1.5, label="Drop piksel sayısı")

    ax_c1.axvline(stop_t, color="gray", linestyle="--", linewidth=1.2, label="Dur")
    ax_c1.set_xlabel("Zaman [s]", fontsize=8)
    ax_c1.set_ylabel("Hız [m/s]", fontsize=8, color=color_v)
    ax_c2.set_ylabel("Drop Piksel Sayısı", fontsize=8, color=color_d)
    ax_c1.tick_params(axis="y", labelcolor=color_v, labelsize=7)
    ax_c2.tick_params(axis="y", labelcolor=color_d, labelsize=7)
    ax_c1.tick_params(axis="x", labelsize=7)

    lines1, labs1 = ax_c1.get_legend_handles_labels()
    lines2, labs2 = ax_c2.get_legend_handles_labels()
    ax_c1.legend(lines1 + lines2, labs1 + labs2, fontsize=7, loc="upper right")
    ax_c.set_title("(c) Zaman Serisi: Hız & Drop Pikseli", fontsize=9, fontweight="bold")

    fig.suptitle(
        r"$\c{S}$ekil 6 — Ger$\c{c}$ek Robot Deney G\"orseli (Sentetik Yer Tutucu)",
        fontsize=10, fontweight="bold"
    )

    plt.tight_layout(rect=[0, 0, 1, 0.94])
    fig.savefig(OUT_PDF, dpi=300, bbox_inches="tight")
    fig.savefig(OUT_PNG, dpi=150, bbox_inches="tight")
    print(f"Kaydedildi: {OUT_PDF}")
    print(f"Kaydedildi: {OUT_PNG}")

    if MANUS_PDF.parent.exists():
        shutil.copy(OUT_PDF, MANUS_PDF)
        print(f"Kopyalandı:  {MANUS_PDF}")


if __name__ == "__main__":
    main()
