#!/bin/bash
# collect_dataset.sh
# Lancia i trial di raccolta dati automaticamente.
# Prerequisito: i nodi ROS2 devono essere già in esecuzione.

DRY_RUN=false
if [[ "$1" == "--dry-run" ]]; then
    DRY_RUN=true
    echo "[DRY RUN] Nessun comando verrà eseguito."
fi

MARGIN=3          # secondi extra dopo trial_duration prima del prossimo trial
REPS=3            # ripetizioni per ogni combinazione

run() {
    if $DRY_RUN; then
        echo "  >> $*"
    else
        eval "$@"
    fi
}

wait_trial() {
    local duration=$1
    local total=$((duration + MARGIN))
    if $DRY_RUN; then
        echo "  >> sleep $total"
    else
        echo "    attendo ${total}s ..."
        sleep "$total"
    fi
}

set_param() {
    run "ros2 param set /master_node $1 $2"
}

start_trial() {
    run "ros2 service call /trial std_srvs/srv/SetBool '{data: true}'"
}

echo "================================================"
echo "  Raccolta dataset fish robot"
echo "  Ripetizioni per combinazione: $REPS"
echo "================================================"

# ── Gruppo 1: amp_sweep — freq fissa, ampiezza varia ──────────────────────────
echo ""
echo "=== GRUPPO 1: amp_sweep ==="
DURATION=30
DURATION_ROS=30.0

set_param mode amp_sweep
set_param amp_min_rad 0.3
set_param amp_max_rad 1.2
set_param trial_duration_sec $DURATION_ROS

for freq in 0.5 1.0 1.5 2.0; do
    set_param tail_freq_hz $freq
    for rep in $(seq 1 $REPS); do
        echo ""
        echo "  amp_sweep | freq=${freq}Hz | rep=${rep}/${REPS}"
        start_trial
        wait_trial $DURATION
    done
done

# ── Gruppo 2: freq_sweep — amp fissa, frequenza varia ─────────────────────────
echo ""
echo "=== GRUPPO 2: freq_sweep ==="
DURATION=30
DURATION_ROS=30.0

set_param mode freq_sweep
set_param freq_min_hz 0.5
set_param freq_max_hz 2.0
set_param trial_duration_sec $DURATION_ROS

for amp in 0.4 0.7 1.0; do
    set_param tail_amp_rad $amp
    for rep in $(seq 1 $REPS); do
        echo ""
        echo "  freq_sweep | amp=${amp}rad | rep=${rep}/${REPS}"
        start_trial
        wait_trial $DURATION
    done
done

echo ""
echo "================================================"
echo "  Dataset completo: $((( 4 + 3 ) * REPS)) trial in ./logs"
echo "================================================"