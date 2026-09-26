#!/usr/bin/env bash
# Lancia N worker paralleli sullo stesso studio Optuna (fase unica IM+FM).
# Uso:  ./run_tuning.sh <n_worker> <trial_totali> [argomenti per tune_joint.py...]
# Es.:  ./run_tuning.sh 20 100 --train_mode combo --p 1 --rollout_steps 10 --lambda_roll 1.0 \
#           --roll_warmup 10 --device cpu --threads 4
#       ./run_tuning.sh 4 200 --train_mode supervised --p 10            (GPU, default)
# Gli argomenti passati in fondo sovrascrivono i default (es. --device cpu --threads 4).
# I worker girano con priorita' bassa (nice 10) per non rubare CPU agli altri utenti.
set -euo pipefail

WORKERS=${1:?Uso: ./run_tuning.sh <n_worker> <trial_totali> [args tune_joint.py]}
TOTAL=${2:?Uso: ./run_tuning.sh <n_worker> <trial_totali> [args tune_joint.py]}
shift 2
PER_WORKER=$(( (TOTAL + WORKERS - 1) / WORKERS ))

export OMP_NUM_THREADS=4
export MKL_NUM_THREADS=4
export PYTHONPATH="$(pwd)/src:${PYTHONPATH:-}"

TUNE_PY="src/net/Estimator/tuning/tune_joint.py"
OUT_DIR="src/net/Estimator/tuning/tuning_results"
LOG_DIR="${OUT_DIR}/logs_tuning"
mkdir -p "${LOG_DIR}"

STAMP=$(date +%Y%m%d_%H%M)
echo "Tuning IM+FM | $WORKERS worker x $PER_WORKER trial = ~$(( WORKERS * PER_WORKER )) | args: $*"

for i in $(seq 1 "$WORKERS"); do
EXTRA=()
# solo il primo worker accoda la config attuale di train.py
if [ "$i" -ne 1 ]; then EXTRA=(--no_warm_start); fi
LOG="${LOG_DIR}/${STAMP}_w${i}.log"
nohup nice -n 10 python -u "$TUNE_PY" --n_trials "$PER_WORKER" --threads 4 --device cuda \
${EXTRA[@]+"${EXTRA[@]}"} "$@" > "$LOG" 2>&1 &
echo "  worker $i -> PID $!  (log: $LOG)"
sleep 3   # sfasa la creazione dello studio su sqlite
done

echo
echo "Monitoraggio:  tail -f ${LOG_DIR}/${STAMP}_w1.log"
echo "Stato worker:  pgrep -af tune_joint.py"
echo "Stop di tutti: pkill -f tune_joint.py"
echo "Dashboard:     optuna-dashboard sqlite:///${OUT_DIR}/optuna_joint.db"