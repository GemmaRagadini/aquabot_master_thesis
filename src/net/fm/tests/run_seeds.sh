#!/usr/bin/env bash
# Allena GRU e LSTM (stessi iperparametri = default di train.py) con N seed,
# poi valuta tutto sul TEST set: metriche one-step (metrics.py), riassunto
# media +- std (summarize_seeds.py) e rollout per orizzonte (rollout_test.py).
#
# Uso (dalla root della repo):
#   ./src/net/fm/tests/run_seeds.sh                      # 5 seed
#   N_SEEDS=3 ./src/net/fm/tests/run_seeds.sh
#   SKIP_TRAIN=1 ./src/net/fm/tests/run_seeds.sh         # solo valutazione (checkpoint gia' allenati)
#   TRAIN_ARGS="--threads 4" ./src/net/fm/tests/run_seeds.sh   # argomenti in piu' per train.py
#
# Prima di lanciarlo: metti in train.py, come default, gli iperparametri scelti dal tuning.
set -euo pipefail

N_SEEDS=${N_SEEDS:-5}
DATASET=${DATASET:-src/net/dataset}          # 140 trial (train + validation)
TEST=${TEST:-src/net/testset}               # 20 trial del test (cartella da adattare)
SCALER=${SCALER:-src/net/scaler/scaler.pkl}
CKPT=${CKPT:-src/net/fm/checkpoints_fm}
OUT=${OUT:-src/net/fm/results_seeds}
export PYTHONPATH="$(pwd)/src:$(pwd)/src/net/fm:${PYTHONPATH:-}"

mkdir -p "$OUT"
SEEDS=$(seq 1 "$N_SEEDS")

if [ -z "${SKIP_TRAIN:-}" ]; then
  for cell in gru lstm; do
    for s in $SEEDS; do
      echo "=== train $cell seed $s ==="
      python3 -u src/net/fm/train.py --dataset_dir "$DATASET" --scaler_path "$SCALER" \
        --checkpoint_dir "$CKPT" --cell "$cell" --seed "$s" --tag "${cell}_s${s}" ${TRAIN_ARGS:-} \
        > "$OUT/train_${cell}_s${s}.log" 2>&1
      tail -n 3 "$OUT/train_${cell}_s${s}.log"
    done
  done
fi

for cell in gru lstm; do
  for s in $SEEDS; do
    echo "=== metrics (test) $cell seed $s ==="
    python3 src/net/fm/tests/metrics.py --checkpoint "$CKPT/best_${cell}_s${s}.pt" \
      --dataset_dir "$TEST" --train_dir "$DATASET" --scaler_path "$SCALER" \
      --all_trials --summary_only --overwrite --csv_out "$OUT/metrics_${cell}_s${s}.csv" \
      > "$OUT/metrics_${cell}_s${s}.log" 2>&1
  done
done

echo "=== one-step, test set: mean +- std over the seeds ==="
python3 src/net/fm/tests/summarize_seeds.py \
  --gru  $(for s in $SEEDS; do echo "$OUT/metrics_gru_s${s}.csv";  done) \
  --lstm $(for s in $SEEDS; do echo "$OUT/metrics_lstm_s${s}.csv"; done) \
  | tee "$OUT/summary_one_step.txt"

echo "=== rollout, test set ==="
python3 src/net/fm/tests/rollout_test.py \
  --checkpoint $(for c in gru lstm; do for s in $SEEDS; do echo "$CKPT/best_${c}_s${s}.pt"; done; done) \
  --dataset_dir "$TEST" --train_dir "$DATASET" --scaler_path "$SCALER" --all_trials \
  --out "$OUT/rollout" | tee "$OUT/summary_rollout.txt"

echo "Fatto. Risultati in $OUT/"