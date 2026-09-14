#!/usr/bin/env bash
# Lancia N worker paralleli sulla stessa fase del tuning FM, storage sqlite condiviso.
# Uso:  ./run_tuning_fm.sh <fase> [n_worker] [trial_totali]
set -euo pipefail

PHASE=${1:?Uso: ./run_tuning_fm.sh <fase 1|2|3> [n_worker] [trial_totali]}
WORKERS=${2:-4}

# trial totali di default per fase (fase 2: PER architettura)
declare -A DEFAULT_TOTAL=( [1]=16 [2]=30 [3]=50 )
TOTAL=${3:-${DEFAULT_TOTAL[$PHASE]}}
PER_WORKER=$(( (TOTAL + WORKERS - 1) / WORKERS ))

export OMP_NUM_THREADS=4
export MKL_NUM_THREADS=4

# rende importabile il package 'net' (che vive dentro src/)
export PYTHONPATH="$(pwd)/src:${PYTHONPATH:-}"

TUNE_PY="src/net/Estimator/tuning/tune_joint.py"
OUT_DIR="src/net/Estimator/tuning/tuning_results"

mkdir -p "${OUT_DIR}/tuning_results" "${OUT_DIR}/logs_tuning"

echo "Tuning FM | Fase $PHASE | $WORKERS worker x $PER_WORKER trial = ~$TOTAL trial totali"

for i in $(seq 1 "$WORKERS"); do
    nohup python -u "$TUNE_PY" \
        --phase    "$PHASE" \
        --n_trials "$PER_WORKER" \
        --threads  4 \
        --device   cuda \
        > "${OUT_DIR}/logs_tuning/phase${PHASE}_w${i}.log" 2>&1 &
    echo "  worker $i -> PID $!  (log: ${OUT_DIR}/logs_tuning/phase${PHASE}_w${i}.log)"
    sleep 2   # sfasa la creazione degli studi su sqlite
done

echo
echo "Monitoraggio:  tail -f ${OUT_DIR}/logs_tuning/phase${PHASE}_w1.log"
echo "GPU:           nvtop"
echo "Attendi fine:  wait"