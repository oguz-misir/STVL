"""
Scene-based train/val/test split oluşturucu.

Aynı scene_id içindeki örneklerin farklı splitlere karışmasını
(scene leakage) önler.
"""

from __future__ import annotations

import argparse
import json
import random
from collections import defaultdict
from pathlib import Path


def _load_scene_sample_map(raw_dir: Path) -> dict[str, list[str]]:
    """meta.json dosyalarından scene_id -> [sample_stem] haritası üret."""
    scene_map: dict[str, list[str]] = defaultdict(list)

    for meta_path in sorted(raw_dir.glob("*_meta.json")):
        stem = meta_path.name.replace("_meta.json", "")
        try:
            with open(meta_path) as f:
                meta = json.load(f)
            scene_id = meta.get("scene_id", stem)
            scene_map[scene_id].append(stem)
        except (json.JSONDecodeError, OSError):
            continue

    return dict(scene_map)


def _split_scenes(
    scene_ids: list[str],
    train_ratio: float,
    val_ratio: float,
    seed: int,
) -> tuple[list[str], list[str], list[str]]:
    """Scene listesini train/val/test olarak böl (oranların toplamı 1 olmalı)."""
    rng = random.Random(seed)
    shuffled = scene_ids[:]
    rng.shuffle(shuffled)

    n = len(shuffled)
    n_train = int(n * train_ratio)
    n_val = int(n * val_ratio)

    train_scenes = shuffled[:n_train]
    val_scenes = shuffled[n_train: n_train + n_val]
    test_scenes = shuffled[n_train + n_val:]

    return train_scenes, val_scenes, test_scenes


def _check_scene_leakage(
    train: list[str],
    val: list[str],
    test: list[str],
) -> bool:
    """Scene leakage olup olmadığını kontrol et; True = temiz."""
    train_set = set(train)
    val_set = set(val)
    test_set = set(test)

    train_val = train_set & val_set
    train_test = train_set & test_set
    val_test = val_set & test_set

    if train_val or train_test or val_test:
        print("HATA: Scene leakage tespit edildi!")
        if train_val:
            print(f"  Train ∩ Val : {train_val}")
        if train_test:
            print(f"  Train ∩ Test: {train_test}")
        if val_test:
            print(f"  Val ∩ Test  : {val_test}")
        return False

    print("OK: Scene leakage yok. Tum sceneler tek bir splitte.")
    return True


def build_splits(
    data_dir: Path,
    train_ratio: float = 0.70,
    val_ratio: float = 0.15,
    seed: int = 42,
) -> tuple[list[str], list[str], list[str]]:
    """Split dosyalarını oluştur ve (train_stems, val_stems, test_stems) döndür."""
    raw_dir = data_dir / "raw"
    splits_dir = data_dir / "splits"
    splits_dir.mkdir(parents=True, exist_ok=True)

    scene_map = _load_scene_sample_map(raw_dir)
    if not scene_map:
        print(f"UYARI: {raw_dir} altinda hic meta.json bulunamadi.")
        return [], [], []

    scene_ids = sorted(scene_map.keys())
    train_scenes, val_scenes, test_scenes = _split_scenes(
        scene_ids, train_ratio, val_ratio, seed
    )

    # Scene leakage kontrolü
    _check_scene_leakage(train_scenes, val_scenes, test_scenes)

    # Her split için sample stem listesi oluştur
    def _stems_for(scenes: list[str]) -> list[str]:
        stems = []
        for sc in scenes:
            stems.extend(scene_map[sc])
        return sorted(stems)

    train_stems = _stems_for(train_scenes)
    val_stems = _stems_for(val_scenes)
    test_stems = _stems_for(test_scenes)

    # txt dosyalarına yaz
    for name, stems in [("train", train_stems), ("val", val_stems), ("test", test_stems)]:
        out_path = splits_dir / f"{name}.txt"
        out_path.write_text("\n".join(stems) + "\n" if stems else "")

    return train_stems, val_stems, test_stems


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Scene-based train/val/test split oluşturucu."
    )
    parser.add_argument(
        "--data-dir",
        type=Path,
        default=Path("data"),
        help="Veri kök klasörü.",
    )
    parser.add_argument("--train-ratio", type=float, default=0.70)
    parser.add_argument("--val-ratio",   type=float, default=0.15)
    parser.add_argument("--test-ratio",  type=float, default=0.15,
                        help="Bilgi amaçlı; 1 - train - val olarak hesaplanır.")
    parser.add_argument("--seed", type=int, default=42)
    return parser.parse_args()


def main() -> None:
    args = _parse_args()

    train, val, test = build_splits(
        data_dir=args.data_dir,
        train_ratio=args.train_ratio,
        val_ratio=args.val_ratio,
        seed=args.seed,
    )

    print(f"\nSplit sonuclari:")
    print(f"  Train : {len(train):4d} ornek")
    print(f"  Val   : {len(val):4d} ornek")
    print(f"  Test  : {len(test):4d} ornek")
    print(f"  Toplam: {len(train) + len(val) + len(test):4d} ornek")
    print(f"\nDosyalar: {args.data_dir / 'splits'}")


if __name__ == "__main__":
    main()
