#!/bin/bash
# collect_dataset.sh
# nodi ROS2 devono essere già in esecuzione.

# Limiti fisici (NON MODIFICARE):
#   tail_min_rad = 0.385  (tick 2299)
#   tail_max_rad = 1.422  (tick 2975)
#   tail_bias    = 0.903
#   max_amp      = 0.519  = (1.422 - 0.385) / 2
#
# Verifica peggio caso per ogni gruppo:
#   amp_sweep:        bias(0.903) + amp_max(0.518) = 1.421  ✅
#   freq_sweep:       bias(0.903) + amp(0.519)     = 1.422  ✅ (limite esatto)
#   combined_sweep:   bias(0.903) + amp_max(0.518) = 1.421  ✅
#   turning_fixed:    bias(0.903) + t_amp(0.2) + amp(0.3)   = 1.403  ✅
#   turning_combined: bias(0.903) + t_amp(0.2) + amp_max(0.518) = 1.621 → clampato a 1.422 ✅

DRY_RUN=false
if [[ "$1" == "--dry-run" ]]; then
    DRY_RUN=true
    echo "[DRY RUN] Nessun comando verrà eseguito."
fi

MARGIN=3          # secondi extra dopo trial_duration prima del prossimo trial
REPS=3            # ripetizioni per ogni combinazione
DURATION=30       # durata di ogni trial in secondi
DURATION_ROS=30.0

# Limiti condivisi — NON CAMBIARE
AMP_MIN=0.3
AMP_MAX=0.518     # < 0.519 = semiampiezza massima fisica
FREQ_MIN=0.5
FREQ_MAX=1.0

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

TOTAL_TRIALS=0

echo "================================================"
echo "  Raccolta dataset fish robot"
echo "  Ripetizioni per combinazione: $REPS"
echo "================================================"

# ── Gruppo 1: amp_sweep ──────────────────────────────
echo ""
echo "=== GRUPPO 1: amp_sweep ==="

set_param mode amp_sweep
set_param amp_min_rad $AMP_MIN
set_param amp_max_rad $AMP_MAX
set_param trial_duration_sec $DURATION_ROS

for freq in 0.3 0.5 0.7 1.0; do
    set_param tail_freq_hz $freq
    for rep in $(seq 1 $REPS); do
        echo ""
        echo "  amp_sweep | freq=${freq}Hz | rep=${rep}/${REPS}"
        start_trial
        wait_trial $DURATION
        TOTAL_TRIALS=$((TOTAL_TRIALS + 1))
    done
done

# ── Gruppo 2: freq_sweep ─────────────────────────────
echo ""
echo "=== GRUPPO 2: freq_sweep ==="

set_param mode freq_sweep
set_param freq_min_hz $FREQ_MIN
set_param freq_max_hz $FREQ_MAX
set_param trial_duration_sec $DURATION_ROS

# amp=0.519 è il limite esatto: bias(0.903)+0.519=1.422 ✅
for amp in 0.2 0.3 0.4 0.519; do
    set_param tail_amp_rad $amp
    for rep in $(seq 1 $REPS); do
        echo ""
        echo "  freq_sweep | amp=${amp}rad | rep=${rep}/${REPS}"
        start_trial
        wait_trial $DURATION
        TOTAL_TRIALS=$((TOTAL_TRIALS + 1))
    done
done

# ── Gruppo 3: combined_sweep ─────────────────────────
echo ""
echo "=== GRUPPO 3: combined_sweep ==="

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

# ── Gruppo 4: turning_combined (amp e freq variabili) ─
echo ""
echo "=== GRUPPO 4: turning_combined ==="

set_param mode turning_combined
set_param amp_min_rad $AMP_MIN
set_param amp_max_rad $AMP_MAX
set_param freq_min_hz $FREQ_MIN
set_param freq_max_hz $FREQ_MAX
set_param trial_duration_sec $DURATION_ROS

# turning_amp=0.2: bias(0.903)+0.2+amp_max(0.518)=1.621 → clampato ✅
for turning_amp in 0.2; do
    set_param turning_bias_amp_rad $turning_amp
    for rep in $(seq 1 $REPS); do
        echo ""
        echo "  turning_combined | turning_amp=${turning_amp}rad | rep=${rep}/${REPS}"
        start_trial
        wait_trial $DURATION
        TOTAL_TRIALS=$((TOTAL_TRIALS + 1))
    done
done

# ── Gruppo 5: turning_fixed (amp e freq costanti) ─────
# Curva con parametri fissi: amp_min=amp_max e freq_min=freq_max
# bias(0.903) + turning_amp(0.2) + amp(0.3) = 1.403 ✅ ben dentro i limiti
echo ""
echo "=== GRUPPO 5: turning_fixed ==="

set_param mode turning_combined
set_param amp_min_rad 0.3
set_param amp_max_rad 0.3    # uguale a min → ampiezza fissa
set_param freq_min_hz 0.5
set_param freq_max_hz 0.5    # uguale a min → frequenza fissa
set_param trial_duration_sec $DURATION_ROS

for turning_amp in 0.2; do
    set_param turning_bias_amp_rad $turning_amp
    for rep in $(seq 1 $REPS); do
        echo ""
        echo "  turning_fixed | amp=0.3 freq=0.5Hz turning_amp=${turning_amp}rad | rep=${rep}/${REPS}"
        start_trial
        wait_trial $DURATION
        TOTAL_TRIALS=$((TOTAL_TRIALS + 1))
    done
done

echo ""
echo "================================================"
echo "  Dataset completo: ${TOTAL_TRIALS} trial in ./src/net/dataset"
echo "================================================"