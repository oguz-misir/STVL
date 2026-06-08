"""
Şekil 7: Sentetik ve gerçek dünya derinlik çerçevelerinin karşılaştırması.

Dört senaryo × iki sütun (Sentetik | Gerçek-Dünya Temsili):
  1. Çukur (Pit)
  2. Platform kenarı (Platform Edge)
  3. Merdiven inişi (Stair Descent)
  4. Gölge bölgesi / zor-negatif (Shadow / Hard-Negative)

Gerçek dünya görüntüleri mevcut değilse sentetik gürültüyle canlandırılır.
Her alt panel altında 4-sınıf risk maskası renk çubuğu gösterilir.

Çıktı:
  experiments/figures/fig7_dataset.pdf + .png
  manuscript/figures/fig7_dataset.pdf
"""

from __future__ import annotations

import shutil
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import numpy as np

ROOT      = Path(__file__).parent.parent.parent
OUT_PDF   = ROOT / "experiments" / "figures" / "fig7_dataset.pdf"
OUT_PNG   = ROOT / "experiments" / "figures" / "fig7_dataset.png"
MANUS_PDF = ROOT / "manuscript" / "figures" / "fig7_dataset.pdf"

H, W = 160, 213

RISK_COLOR = {
    0: np.array([0.15, 0.70, 0.15]),
    1: np.array([0.90, 0.85, 0.10]),
    2: np.array([0.55, 0.10, 0.80]),
    3: np.array([0.92, 0.10, 0.10]),
}
LABEL_NAMES = ["SAFE", "UNCERTAIN", "UNSAFE_SOLID", "UNSAFE_DROP"]


def norm_depth(d: np.ndarray) -> np.ndarray:
    fin = np.isfinite(d)
    out = np.ones_like(d)
    if fin.any():
        lo, hi = d[fin].min(), d[fin].max()
        out[fin] = (d[fin] - lo) / max(hi - lo, 1e-6)
    return np.clip(out, 0, 1)


def colorize(mask: np.ndarray) -> np.ndarray:
    rgb = np.ones((*mask.shape, 3))
    for lbl, col in RISK_COLOR.items():
        rgb[mask == lbl] = col
    return rgb


def add_noise(depth: np.ndarray, rng: np.random.Generator, sigma: float = 0.04) -> np.ndarray:
    """Gerçek derinlik gürültüsü simülasyonu: salt & pepper + Gaussian."""
    out = depth.copy()
    fin = np.isfinite(out)
    out[fin] += rng.normal(0, sigma, fin.sum())
    # salt: %1 piksel geçersiz
    sp = rng.random(out.shape) < 0.015
    out[sp] = np.inf
    return out


# ── Senaryo üreticileri ──────────────────────────────────────────────────────

def scene_pit(rng):
    d = np.full((H, W), 1.05)
    pr = slice(int(H * 0.50), int(H * 0.80))
    pc = slice(int(W * 0.30), int(W * 0.70))
    d[pr, pc] = 2.70
    sh = slice(int(H * 0.46), int(H * 0.52))
    sc = slice(int(W * 0.28), int(W * 0.72))
    d[sh, sc] = np.inf
    d[8:40, 8:55] = 0.52

    m = np.zeros((H, W), dtype=np.uint8)
    for r in range(1, H - 1):
        for c in range(1, W - 1):
            if not np.isfinite(d[r, c]):
                m[r, c] = 1
                continue
            nbrs = d[r - 1:r + 2, c - 1:c + 2].ravel()
            fin  = nbrs[np.isfinite(nbrs)]
            jmp  = (fin.max() - d[r, c]) if fin.size else 0
            if jmp > 0.12 or any(
                not np.isfinite(d[r + dr, c + dc])
                for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]
                if 0 <= r + dr < H and 0 <= c + dc < W
            ):
                m[r, c] = 3
    m[d < 0.65] = 2
    m[~np.isfinite(d)] = 1
    return d, m


def scene_platform(rng):
    d = np.full((H, W), 0.90)
    ec = int(W * 0.60)
    d[:, ec:] = 2.50
    d[:, ec - 3:ec + 3] = np.inf
    d[12:50, int(W * 0.15):int(W * 0.30)] = 0.42

    m = np.zeros((H, W), dtype=np.uint8)
    m[:, ec - 5:ec + 5] = 3
    m[~np.isfinite(d)] = 1
    m[d < 0.65] = 2
    m[:, ec + 5:] = 1
    return d, m


def scene_stair(rng):
    d = np.full((H, W), 1.00)
    step_h = [int(H * 0.40), int(H * 0.55), int(H * 0.70)]
    depths  = [1.35, 1.70, 2.05]
    for r_start, dep in zip(step_h, depths):
        d[r_start:, :] = dep
        d[r_start:r_start + 4, :] = np.inf

    m = np.zeros((H, W), dtype=np.uint8)
    for rs in step_h:
        m[rs:rs + 5, :] = 3
        m[rs:rs + 4, :] = 1
    m[~np.isfinite(d)] = 1
    m[d < 0.65] = 2
    return d, m


def scene_shadow(rng):
    """Zor-negatif: gölge → inf derinlik ama gerçek düşme yok."""
    d = np.full((H, W), 1.00)
    # gölge bandı (inf ama gerçek zemin var)
    sr = slice(int(H * 0.50), int(H * 0.62))
    sc = slice(int(W * 0.25), int(W * 0.65))
    d[sr, sc] = np.inf

    m = np.zeros((H, W), dtype=np.uint8)
    m[sr, sc] = 1          # doğru sınıf: UNCERTAIN
    m[d < 0.65] = 2
    return d, m


SCENES_SYNTH = [
    ("Çukur (Pit)", scene_pit),
    ("Platform Kenarı", scene_platform),
    ("Merdiven İnişi", scene_stair),
    ("Gölge / Zor-Negatif", scene_shadow),
]


def main() -> None:
    rng = np.random.default_rng(7)

    n_rows = len(SCENES_SYNTH)
    fig, axes = plt.subplots(n_rows, 4,
                             figsize=(14, 3.2 * n_rows),
                             gridspec_kw={"wspace": 0.08, "hspace": 0.45})

    col_titles = [
        "Sentetik Derinlik",
        "Sentetik Risk Maskası",
        "Gerçek Dünya Derinlik*",
        "Gerçek Dünya Risk Maskası*",
    ]

    for col, title in enumerate(col_titles):
        axes[0, col].set_title(title, fontsize=8, fontweight="bold", pad=6)

    for row, (name, fn) in enumerate(SCENES_SYNTH):
        d_syn, m_syn = fn(rng)
        d_real = add_noise(d_syn, rng, sigma=0.05)
        # Gerçek için de aynı mask (gerçek veri yoksa etiket senkronize)
        m_real = m_syn.copy()
        # biraz gürültü: %5 piksel etiket hatası (sensör hatası simülasyonu)
        noise_idx = rng.random(m_real.shape) < 0.05
        m_real[noise_idx] = rng.integers(0, 4, noise_idx.sum())

        datasets = [(d_syn, m_syn, False), (d_real, m_real, True)]

        for pair_idx, (d, m, is_real) in enumerate(datasets):
            ax_d = axes[row, pair_idx * 2]
            ax_m = axes[row, pair_idx * 2 + 1]

            # derinlik
            ax_d.imshow(norm_depth(d), cmap="gray", vmin=0, vmax=1)
            ax_d.axis("off")
            if pair_idx == 0:
                ax_d.set_ylabel(name, fontsize=8, rotation=0,
                                labelpad=60, va="center")

            # risk maskası
            ax_m.imshow(colorize(m))
            ax_m.axis("off")

            # gerçek veri yer tutucu bandı
            if is_real:
                for ax in (ax_d, ax_m):
                    ax.text(0.02, 0.97, "* Yer Tutucu",
                            transform=ax.transAxes, fontsize=6,
                            color="white", va="top",
                            bbox=dict(fc="#333", alpha=0.65, pad=2))

    # ortak legend
    patches = [
        mpatches.Patch(facecolor=RISK_COLOR[0], edgecolor="k", lw=0.5, label="SAFE"),
        mpatches.Patch(facecolor=RISK_COLOR[1], edgecolor="k", lw=0.5, label="UNCERTAIN"),
        mpatches.Patch(facecolor=RISK_COLOR[2], edgecolor="k", lw=0.5, label="UNSAFE SOLID"),
        mpatches.Patch(facecolor=RISK_COLOR[3], edgecolor="k", lw=0.5, label="UNSAFE DROP"),
    ]
    fig.legend(handles=patches, loc="lower center", ncol=4,
               fontsize=8, frameon=True, bbox_to_anchor=(0.5, 0.0))

    fig.suptitle(
        "Şekil 7 — Sentetik ve Gerçek Dünya Derinlik Veri Seti Karşılaştırması\n"
        "(*) Gerçek çerçeveler fiziksel dağıtım sonrası güncellenecektir.",
        fontsize=10, fontweight="bold"
    )

    plt.tight_layout(rect=[0, 0.05, 1, 0.96])
    fig.savefig(OUT_PDF, dpi=300, bbox_inches="tight")
    fig.savefig(OUT_PNG, dpi=150, bbox_inches="tight")
    print(f"Kaydedildi: {OUT_PDF}")
    print(f"Kaydedildi: {OUT_PNG}")

    if MANUS_PDF.parent.exists():
        shutil.copy(OUT_PDF, MANUS_PDF)
        print(f"Kopyalandı:  {MANUS_PDF}")


if __name__ == "__main__":
    main()
