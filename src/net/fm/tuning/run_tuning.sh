#!/usr/bin/env bash
# Lancia N worker paralleli sullo stesso studio Optuna (forward model).
# Uso:  ./run_tuning_fm.sh <n_worker> <trial_totali> [argomenti per tune_fm.py...]
# Es.:  ./run_tuning_fm.sh 8 100                                   (CPU, default)
#       ./run_tuning_fm.sh 20 100 --threads 2                     (piu worker, meno thread)
#       ./run_tuning_fm.sh 4 200 --max_epochs 80 --patience 20
# Gli argomenti passati in fondo sovrascrivono i default (es. --device cpu --threads 4).
# I worker girano con priorita' bassa (nice 10) per non rubare CPU agli altri utenti.
set -euo pipefail

WORKERS=${1:?Uso: ./run_tuning_fm.sh <n_worker> <trial_totali> [args tune_fm.py]}
TOTAL=${2:?Uso: ./run_tuning_fm.sh <n_worker> <trial_totali> [args tune_fm.py]}
shift 2
PER_WORKER=$(( (TOTAL + WORKERS - 1) / WORKERS ))

export OMP_NUM_THREADS=4
export MKL_NUM_THREADS=4
export CUDA_VISIBLE_DEVICES=""   # niente GPU: i worker non la vedono nemmeno
export PYTHONPATH="$(pwd)/src:${PYTHONPATH:-}"

TUNE_PY="src/net/fm/tuning/tune.py"
OUT_DIR="src/net/fm/tuning/tuning_results"
LOG_DIR="${OUT_DIR}/logs_tuning_fm"
mkdir -p "${LOG_DIR}"

STAMP=$(date +%Y%m%d_%H%M)
echo "Tuning FM | $WORKERS worker x $PER_WORKER trial = ~$(( WORKERS * PER_WORKER )) | args: $*"

for i in $(seq 1 "$WORKERS"); do
EXTRA=()
# solo il primo worker accoda la config attuale di train_fm.py
if [ "$i" -ne 1 ]; then EXTRA=(--no_warm_start); fi
LOG="${LOG_DIR}/${STAMP}_w${i}.log"
nohup nice -n 10 python -u "$TUNE_PY" --n_trials "$PER_WORKER" --threads 4 --device cpu \
${EXTRA[@]+"${EXTRA[@]}"} "$@" > "$LOG" 2>&1 &
echo "  worker $i -> PID $!  (log: $LOG)"
sleep 3   # sfasa la creazione dello studio su sqlite
done

echo
echo "Monitoraggio:  tail -f ${LOG_DIR}/${STAMP}_w1.log"
echo "Stato worker:  pgrep -af tune.py"
echo "Stop di tutti: pkill -f tune.py"
echo "Dashboard:     optuna-dashboard sqlite:///${OUT_DIR}/optuna_fm.db"