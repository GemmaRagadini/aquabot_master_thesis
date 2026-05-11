#!/bin/bash
# collect_dataset.sh
# nodi ROS2 devono essere già in esecuzione.

DRY_RUN=false
if [[ "$1" == "--dry-run" ]]; then
    DRY_RUN=true
    echo "[DRY RUN] Nessun comando verrà eseguito."
fi

MARGIN=3          # secondi extra dopo trial_duration prima del prossimo trial
REPS=3            # ripetizioni per ogni combinazione

# range condivisi
AMP_MIN=0.3
AMP_MAX=1.2
FREQ_MIN=0.5
FREQ_MAX=2.0

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

# conteggio totale trial 
TOTAL_TRIALS=0

echo "================================================"
echo "  Raccolta dataset fish robot"
echo "  Ripetizioni per combinazione: $REPS"
echo "================================================"

# ── Gruppo 1: amp_sweep
echo ""
echo "=== GRUPPO 1: amp_sweep ==="
DURATION=30
DURATION_ROS=30.0

set_param mode amp_sweep
set_param amp_min_rad $AMP_MIN
set_param amp_max_rad $AMP_MAX
set_param trial_duration_sec $DURATION_ROS

for freq in 0.5 1.0 1.5 2.0; do
    set_param tail_freq_hz $freq
    for rep in $(seq 1 $REPS); do
        echo ""
        echo "  amp_sweep | freq=${freq}Hz | rep=${rep}/${REPS}"
        start_trial
        wait_trial $DURATION
        TOTAL_TRIALS=$((TOTAL_TRIALS + 1))
    done
done

# ── Gruppo 2: freq_sweep 
echo ""
echo "=== GRUPPO 2: freq_sweep ==="
DURATION=30
DURATION_ROS=30.0

set_param mode freq_sweep
set_param freq_min_hz $FREQ_MIN
set_param freq_max_hz $FREQ_MAX
set_param trial_duration_sec $DURATION_ROS

for amp in 0.4 0.7 1.0; do
    set_param tail_amp_rad $amp
    for rep in $(seq 1 $REPS); do
        echo ""
        echo "  freq_sweep | amp=${amp}rad | rep=${rep}/${REPS}"
        start_trial
        wait_trial $DURATION
        TOTAL_TRIALS=$((TOTAL_TRIALS + 1))
    done
done

# ── Gruppo 3: combined_sweep 
echo ""
echo "=== GRUPPO 3: combined_sweep ==="
DURATION=60
DURATION_ROS=60.0

set_param mode combined_sweep
set_param amp_min_rad $AMP_MIN
set_param amp_max_rad $AMP_MAX
set_param freq_min_hz $FREQ_MIN
set_param freq_max_hz $FREQ_MAX
set_param trial_duration_sec $DURATION_ROS

for rep in $(seq 1 $REPS); do
    echo ""
    echo "  combined_sweep | rep=${rep}/${REPS}"
    start_trial
    wait_trial $DURATION
    TOTAL_TRIALS=$((TOTAL_TRIALS + 1))
done

# ── Gruppo 4: turning_combined

echo ""
echo "=== GRUPPO 4: turning_combined ==="
DURATION=60
DURATION_ROS=60.0

set_param mode turning_combined
set_param amp_min_rad $AMP_MIN
set_param amp_max_rad $AMP_MAX
set_param freq_min_hz $FREQ_MIN
set_param freq_max_hz $FREQ_MAX
set_param trial_duration_sec $DURATION_ROS

for turning_amp in 0.2 0.4; do
    set_param turning_bias_amp_rad $turning_amp
    for rep in $(seq 1 $REPS); do
        echo ""
        echo "  turning_combined | turning_amp=${turning_amp}rad | rep=${rep}/${REPS}"
        start_trial
        wait_trial $DURATION
        TOTAL_TRIALS=$((TOTAL_TRIALS + 1))
    done
done

echo ""
echo "================================================"
echo "  Dataset completo: ${TOTAL_TRIALS} trial in ./logs"
echo "================================================"