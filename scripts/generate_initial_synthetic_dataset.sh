#!/usr/bin/env bash
# İlk sentetik veri setini üretir (~320 örnek, tüm sahne tipleri).
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(dirname "$SCRIPT_DIR")"
GENERATOR="$REPO_ROOT/src/negative_obstacle_data/negative_obstacle_data/synthetic_depth_generator.py"
VISUALIZER="$REPO_ROOT/src/negative_obstacle_data/negative_obstacle_data/synthetic_label_visualizer.py"
SPLIT_BUILDER="$REPO_ROOT/src/negative_obstacle_data/negative_obstacle_data/synthetic_split_builder.py"
QC_SCRIPT="$REPO_ROOT/src/negative_obstacle_data/negative_obstacle_data/synthetic_dataset_qc.py"
DATA_DIR="$REPO_ROOT/data"

# Klasörleri oluştur
mkdir -p "$DATA_DIR/raw" "$DATA_DIR/labels" "$DATA_DIR/processed" "$DATA_DIR/splits"
mkdir -p "$REPO_ROOT/experiments/results"

echo "============================================"
echo " Sentetik veri seti uretimi basliyor"
echo " Hedef: ~320 ornek"
echo "============================================"

python3 "$GENERATOR" --scene-type rectangular_pit      --count 50 --output-dir "$DATA_DIR" --seed 42  --width 320 --height 240
python3 "$GENERATOR" --scene-type circular_pit         --count 40 --output-dir "$DATA_DIR" --seed 100 --width 320 --height 240
python3 "$GENERATOR" --scene-type platform_edge        --count 40 --output-dir "$DATA_DIR" --seed 200 --width 320 --height 240
python3 "$GENERATOR" --scene-type trench               --count 40 --output-dir "$DATA_DIR" --seed 300 --width 320 --height 240
python3 "$GENERATOR" --scene-type stairs_descent       --count 30 --output-dir "$DATA_DIR" --seed 400 --width 320 --height 240
python3 "$GENERATOR" --scene-type curb_drop            --count 30 --output-dir "$DATA_DIR" --seed 500 --width 320 --height 240
python3 "$GENERATOR" --scene-type ramp_hard_negative   --count 30 --output-dir "$DATA_DIR" --seed 600 --width 320 --height 240
python3 "$GENERATOR" --scene-type shadow_hard_negative --count 30 --output-dir "$DATA_DIR" --seed 700 --width 320 --height 240
python3 "$GENERATOR" --scene-type uneven_ground_hard_negative --count 30 --output-dir "$DATA_DIR" --seed 800 --width 320 --height 240

echo ""
echo "============================================"
echo " QC gorselleştirme"
echo "============================================"
python3 "$VISUALIZER" --data-dir "$DATA_DIR" --max-samples 50

echo ""
echo "============================================"
echo " Scene-based split olusturuluyor"
echo "============================================"
python3 "$SPLIT_BUILDER" \
  --data-dir "$DATA_DIR" \
  --train-ratio 0.7 \
  --val-ratio 0.15 \
  --test-ratio 0.15 \
  --seed 42

echo ""
echo "============================================"
echo " Dataset QC raporu uretiliyor"
echo "============================================"
python3 "$QC_SCRIPT" \
  --data-dir "$DATA_DIR" \
  --output-json "$REPO_ROOT/experiments/results/synthetic_dataset_qc.json"

echo ""
echo "============================================"
echo " TAMAMLANDI"
echo " data/raw/       -> depth.npy, meta.json"
echo " data/labels/    -> risk_mask.png, edge_mask.png"
echo " data/processed/ -> overlay.png, QC gorselleri"
echo " data/splits/    -> train.txt, val.txt, test.txt"
echo " experiments/results/synthetic_dataset_qc.json"
echo "============================================"
