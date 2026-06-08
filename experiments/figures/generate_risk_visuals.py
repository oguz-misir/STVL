"""
Figür 3: Risk mask görsel örnekleri (pit + platform_edge).

Gerçek Gazebo frame'i varsa onu kullanır, yoksa sentetik analitik fallback
üretir — her iki durumda da tam renkli figür çıkar.

Çıktı:
  experiments/figures/fig3_risk_masks.pdf + .png
  manuscript/figures/fig3_risk_masks.pdf
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
OUT_PDF   = ROOT / "experiments" / "figures" / "fig3_risk_masks.pdf"
OUT_PNG   = ROOT / "experiments" / "figures" / "fig3_risk_masks.png"
MANUS_PDF = ROOT / "manuscript" / "figures" / "fig3_risk_masks.pdf"

# Renk haritası — IEEE baskısında ayırt edilebilir canlı renkler
RISK_COLOR = {
    0: np.array([0.15, 0.70, 0.15]),   # SAFE        → koyu yeşil
    1: np.array([0.90, 0.85, 0.10]),   # UNCERTAIN   → sarı
    2: np.array([0.55, 0.10, 0.80]),   # UNSAFE_SOLID → mor
    3: np.array([0.92, 0.10, 0.10]),   # UNSAFE_DROP  → parlak kırmızı
}

LABEL_NAMES = {
    0: "SAFE (Güvenli)",
    1: "UNCERTAIN (Belirsiz)",
    2: "UNSAFE_SOLID (Kati Engel)",
    3: "UNSAFE_DROP (Dusme)",
}


# ---------------------------------------------------------------------------
# Renklendirici
# ---------------------------------------------------------------------------
def colorize(mask: np.ndarray) -> np.ndarray:
    rgb = np.ones((*mask.shape, 3), dtype=float)
    for label, col in RISK_COLOR.items():
        rgb[mask == label] = col
    return rgb


def norm_depth(d: np.ndarray) -> np.ndarray:
    valid = np.isfinite(d)
    out = np.zeros_like(d)
    if valid.any():
        lo, hi = d[valid].min(), d[valid].max()
        out[valid] = (d[valid] - lo) / max(hi - lo, 1e-6)
    out[~valid] = 1.0   # sonsuz derinlik → beyaz
    return np.clip(out, 0, 1)


# ---------------------------------------------------------------------------
# Sentetik fallback veri üreticisi
# ---------------------------------------------------------------------------
def make_synthetic_pit(h: int = 240, w: int = 320) -> tuple[np.ndarray, np.ndarray]:
    """Analitik çukur sahnesi: derinlik + risk maskası."""
    rng = np.random.default_rng(0)
    depth = np.full((h, w), 1.1, dtype=float)

    # Çukur bölgesi (görüntü alt-ortası)
    pit_r = slice(int(h * 0.50), int(h * 0.80))
    pit_c = slice(int(w * 0.30), int(w * 0.70))
    depth[pit_r, pit_c] = 2.75          # çukur tabanı

    # Çukur üstü — geçersiz derinlik (gölge / kör nokta)
    shadow_r = slice(int(h * 0.45), int(h * 0.52))
    shadow_c = slice(int(w * 0.28), int(w * 0.72))
    depth[shadow_r, shadow_c] = np.inf

    # Üst köşede kutu engel (pozitif)
    depth[10:60, 10:80] = 0.55

    # Risk maskası üret
    mask = np.zeros((h, w), dtype=np.uint8)   # SAFE default

    # UNSAFE_DROP: çukur sınır pikselleri (derinlik atlaması > 0.12 m)
    for r in range(1, h - 1):
        for c in range(1, w - 1):
            if not np.isfinite(depth[r, c]):
                mask[r, c] = 1   # UNCERTAIN
                continue
            nbrs = depth[r-1:r+2, c-1:c+2].ravel()
            jump = np.nanmax(nbrs[np.isfinite(nbrs)]) - depth[r, c] if np.isfinite(nbrs).any() else 0
            if jump > 0.12:
                mask[r, c] = 3   # UNSAFE_DROP
            if any(not np.isfinite(depth[r+dr, c+dc])
                   for dr, dc in [(-1,0),(1,0),(0,-1),(0,1)]
                   if 0 <= r+dr < h and 0 <= c+dc < w):
                mask[r, c] = 3

    mask[depth < 0.7] = 2          # UNSAFE_SOLID (kutu)
    mask[~np.isfinite(depth)] = 1  # UNCERTAIN (sonsuz)

    return depth, mask


def make_synthetic_platform(h: int = 240, w: int = 320) -> tuple[np.ndarray, np.ndarray]:
    """Analitik platform kenar sahnesi."""
    depth = np.full((h, w), 0.95, dtype=float)

    # Platform sonu — sağ taraf düşüyor
    edge_c = int(w * 0.60)
    depth[:, edge_c:] = 2.60

    # Kenar üstünde dar geçersiz bant
    depth[:, edge_c - 3:edge_c + 3] = np.inf

    # Küçük pozitif engel
    depth[20:70, int(w * 0.15):int(w * 0.30)] = 0.45

    mask = np.zeros((h, w), dtype=np.uint8)
    mask[:, edge_c - 5:edge_c + 5] = 3   # DROP kenar
    mask[~np.isfinite(depth)] = 1         # UNCERTAIN
    mask[depth < 0.7] = 2                  # SOLID
    mask[:, edge_c + 5:] = 1              # uzak bölge — UNCERTAIN

    return depth, mask


# ---------------------------------------------------------------------------
# Gerçek Gazebo verisi yükleyici
# ---------------------------------------------------------------------------
def load_gazebo(depth_path: Path, label_path: Path):
    try:
        from PIL import Image
        depth = np.load(str(depth_path))
        mask  = np.array(Image.open(str(label_path)))
        return depth, mask
    except Exception:
        return None, None


# ---------------------------------------------------------------------------
# Ana figür üretici
# ---------------------------------------------------------------------------
def main() -> None:
    gazebo_sources = [
        ("/tmp/gazebo_depth_test/pit_scene_0000_depth.npy",
         "/tmp/gazebo_labeled/labels/pit_scene_0000_risk_mask.png",
         "Pit Scene (Gazebo)"),
        ("/tmp/gazebo_platform_test/platform_edge_0000_depth.npy",
         "/tmp/gazebo_platform_labeled/labels/platform_edge_0000_risk_mask.png",
         "Platform Edge (Gazebo)"),
    ]

    scenes = []
    for dp, lp, name in gazebo_sources:
        d, m = load_gazebo(Path(dp), Path(lp))
        if d is not None:
            scenes.append((name, d, m))

    # Gerçek veri yoksa sentetik fallback
    if not scenes:
        print("Gazebo verisi bulunamadı — sentetik fallback kullanılıyor.")
        scenes = [
            ("Pit Scene (Sentetik)",      *make_synthetic_pit()),
            ("Platform Edge (Sentetik)",  *make_synthetic_platform()),
        ]

    n = len(scenes)
    fig, axes = plt.subplots(n, 3, figsize=(12, 4 * n))
    if n == 1:
        axes = [axes]

    for row, (name, depth, mask) in enumerate(scenes):
        ax_d, ax_r, ax_e = axes[row]

        # Derinlik (gri skala)
        ax_d.imshow(norm_depth(depth), cmap="gray", vmin=0, vmax=1)
        ax_d.set_title(f"{name}\nDerinlik Görüntüsü", fontsize=9, fontweight="bold")
        ax_d.axis("off")
        # renk çubuğu ipucu
        ax_d.text(0.02, 0.96, "Koyu=Yakın\nAçık=Uzak",
                  transform=ax_d.transAxes, fontsize=7,
                  color="white", va="top",
                  bbox=dict(fc="#333", alpha=0.6, pad=2))

        # Risk maskası — renkli
        ax_r.imshow(colorize(mask))
        ax_r.set_title(f"{name}\nRisk Maskası", fontsize=9, fontweight="bold")
        ax_r.axis("off")

        # Yalnızca DROP sınırı — kenar vurgusu
        drop_only = np.zeros_like(mask)
        drop_only[mask == 3] = 1
        ax_e.imshow(norm_depth(depth), cmap="gray", vmin=0, vmax=1, alpha=0.5)
        ax_e.imshow(drop_only,
                    cmap=matplotlib.colors.ListedColormap(["none", "#FF2222"]),
                    vmin=0, vmax=1, alpha=0.85)
        ax_e.set_title(f"{name}\nDüşme Kenarı (DROP)", fontsize=9, fontweight="bold")
        ax_e.axis("off")

    # Ortak legend
    patches = [
        mpatches.Patch(facecolor=RISK_COLOR[0], edgecolor="k", linewidth=0.5,
                       label="SAFE (Güvenli)"),
        mpatches.Patch(facecolor=RISK_COLOR[1], edgecolor="k", linewidth=0.5,
                       label="UNCERTAIN (Belirsiz)"),
        mpatches.Patch(facecolor=RISK_COLOR[2], edgecolor="k", linewidth=0.5,
                       label="UNSAFE_SOLID (Kati Engel)"),
        mpatches.Patch(facecolor=RISK_COLOR[3], edgecolor="k", linewidth=0.5,
                       label="UNSAFE_DROP (Dusme)"),
    ]
    fig.legend(handles=patches, loc="lower center", ncol=4,
               fontsize=9, frameon=True,
               bbox_to_anchor=(0.5, 0.0))

    fig.suptitle("Şekil 3 — Geometrik Risk Maskası Örnekleri",
                 fontsize=11, fontweight="bold")

    plt.tight_layout(rect=[0, 0.06, 1, 0.97])
    fig.savefig(OUT_PDF, dpi=300, bbox_inches="tight")
    fig.savefig(OUT_PNG, dpi=150, bbox_inches="tight")
    print(f"Kaydedildi: {OUT_PDF}")
    print(f"Kaydedildi: {OUT_PNG}")

    if MANUS_PDF.parent.exists():
        shutil.copy(OUT_PDF, MANUS_PDF)
        print(f"Kopyalandı:  {MANUS_PDF}")


if __name__ == "__main__":
    main()
