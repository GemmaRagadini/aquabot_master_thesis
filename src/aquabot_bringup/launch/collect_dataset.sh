#!/bin/bash
# collect_dataset.sh
# nodi ROS2 devono essere già in esecuzione.
#
# Limiti fisici (NON MODIFICARE):
#   tail_min_rad = 0.385  (tick 2299)
#   tail_max_rad = 1.422  (tick 2975)
#   tail_bias    = 0.903
#   max_amp      = 0.519  = (1.422 - 0.385) / 2
#
# Vincolo generale: bias(0.903) +/- |turning_offset| +/- amp deve restare in [0.385, 1.422].
# Margine simmetrico disponibile su ogni lato: 0.519.
#
# Verifica peggior caso per ogni gruppo:
#   amp_sweep:        bias(0.903) + amp_max(0.518)                  = 1.421  OK
#   freq_sweep:       bias(0.903) + amp(0.519)                      = 1.422  OK (limite esatto)
#   combined_sweep:   bias(0.903) + amp_max(0.518)                  = 1.421  OK
#   turning_fixed:    amp + turning_amp <= 0.519 imposto per config = <=1.422 OK
#   turning_combined: bias(0.903) + t_amp(0.2) + amp_max(0.518)     = 1.621  -> clampato OK
#   random_walk:      bias(0.903) + amp_max(0.518)                  = 1.421  OK (no turning offset)
#   chaotic_stop:     bias(0.903) + amp_max(0.518)                  = 1.421  OK (no turning offset)

DRY_RUN=false
if [[ "$1" == "--dry-run" ]]; then
    DRY_RUN=true
    echo "[DRY RUN] Nessun comando verrà eseguito."
fi

MARGIN=3          # secondi extra dopo trial_duration prima del prossimo trial
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
echo "  1 trial per configurazione"
echo "================================================"

# ── Gruppo 1: amp_sweep ──────────────────────────────
# Sweep triangolare di ampiezza [amp_min, amp_max], freq fissa. Variamo la freq.
echo ""
echo "=== GRUPPO 1: amp_sweep ==="

set_param mode amp_sweep
set_param amp_min_rad $AMP_MIN
set_param amp_max_rad $AMP_MAX
set_param trial_duration_sec $DURATION_ROS

for freq in 0.3 0.4 0.5 0.6 0.7 0.85 1.0; do
    set_param tail_freq_hz $freq
    echo ""
    echo "  amp_sweep | freq=${freq}Hz"
    start_trial
    wait_trial $DURATION
    TOTAL_TRIALS=$((TOTAL_TRIALS + 1))
done

# ── Gruppo 2: freq_sweep ─────────────────────────────
# Sweep triangolare di frequenza [freq_min, freq_max], amp fissa. Variamo l'amp.
echo ""
echo "=== GRUPPO 2: freq_sweep ==="

set_param mode freq_sweep
set_param freq_min_hz $FREQ_MIN
set_param freq_max_hz $FREQ_MAX
set_param trial_duration_sec $DURATION_ROS

# amp=0.519 è il limite esatto: bias(0.903)+0.519=1.422 OK
for amp in 0.15 0.2 0.3 0.4 0.45 0.519; do
    set_param tail_amp_rad $amp
    echo ""
    echo "  freq_sweep | amp=${amp}rad"
    start_trial
    wait_trial $DURATION
    TOTAL_TRIALS=$((TOTAL_TRIALS + 1))
done

# ── Gruppo 3: combined_sweep ─────────────────────────
# Amp e freq entrambe in sweep (periodi sfasati via PHI nel nodo).
# Variamo le finestre di amp e freq per campionare regimi diversi.
echo ""
echo "=== GRUPPO 3: combined_sweep ==="

set_param mode combined_sweep
set_param trial_duration_sec $DURATION_ROS

# cfg: amp_min  amp_max  freq_min  freq_max   (amp_max <= 0.518 sempre)
for cfg in \
    "0.3 0.518 0.5 1.0" \
    "0.3 0.518 0.3 0.8" \
    "0.3 0.518 0.7 1.2" \
    "0.2 0.45  0.5 1.0" \
    "0.15 0.4  0.4 0.9" \
    "0.3 0.518 0.5 1.5"; do
    read amin amax fmin fmax <<< "$cfg"
    set_param amp_min_rad $amin
    set_param amp_max_rad $amax
    set_param freq_min_hz $fmin
    set_param freq_max_hz $fmax
    echo ""
    echo "  combined_sweep | amp=[${amin}, ${amax}] freq=[${fmin}, ${fmax}]Hz"
    start_trial
    wait_trial $DURATION
    TOTAL_TRIALS=$((TOTAL_TRIALS + 1))
done

# ── Gruppo 4: turning_combined ───────────────────────
# Centro di oscillazione che varia sinusoidalmente + amp/freq in sweep.
# turning_amp=0.2: bias+0.2+amp_max=1.621 -> clampato OK
echo ""
echo "=== GRUPPO 4: turning_combined ==="

set_param mode turning_combined
set_param amp_min_rad $AMP_MIN
set_param amp_max_rad $AMP_MAX
set_param freq_min_hz $FREQ_MIN
set_param freq_max_hz $FREQ_MAX
set_param trial_duration_sec $DURATION_ROS

# cfg: turning_amp  turning_freq
for turning_cfg in \
    "0.10 0.08" \
    "0.15 0.08" \
    "0.20 0.08" \
    "0.20 0.12" \
    "0.15 0.05" \
    "0.20 0.15"; do
    read t_amp t_freq <<< "$turning_cfg"
    set_param turning_bias_amp_rad $t_amp
    set_param turning_bias_freq_hz $t_freq
    echo ""
    echo "  turning_combined | turning_amp=${t_amp}rad turning_freq=${t_freq}Hz"
    start_trial
    wait_trial $DURATION
    TOTAL_TRIALS=$((TOTAL_TRIALS + 1))
done

# ── Gruppo 5: turning_fixed ──────────────────────────
# Come turning_combined ma amp e freq costanti (min=max).
# Vincolo: bias(0.903) + turning_amp + amp <= 1.422  -> turning_amp + amp <= 0.519
echo ""
echo "=== GRUPPO 5: turning_fixed ==="

set_param mode turning_combined
set_param turning_bias_freq_hz 0.08
set_param trial_duration_sec $DURATION_ROS

# cfg: amp(fissa)  turning_amp  freq(fissa)   (amp + turning_amp <= 0.519)
for cfg in \
    "0.30 0.15 0.5" \
    "0.30 0.20 0.5" \
    "0.25 0.25 0.5" \
    "0.30 0.15 0.8" \
    "0.20 0.25 0.6" \
    "0.35 0.15 0.5"; do
    read amp t_amp freq <<< "$cfg"
    set_param amp_min_rad $amp
    set_param amp_max_rad $amp      # min=max → ampiezza fissa
    set_param freq_min_hz $freq
    set_param freq_max_hz $freq     # min=max → frequenza fissa
    set_param turning_bias_amp_rad $t_amp
    echo ""
    echo "  turning_fixed | amp=${amp} freq=${freq}Hz turning_amp=${t_amp}rad"
    start_trial
    wait_trial $DURATION
    TOTAL_TRIALS=$((TOTAL_TRIALS + 1))
done

# ── Gruppo 6: random_walk ────────────────────────────
# Amp/freq che vagano con continuità (interpolate). seed=0 → ogni trial diverso.
# Nessun turning offset: bias(0.903)+amp_max(0.518)=1.421 OK.
# Variamo morbidezza (rand_smooth_alpha), cadenza di ricampionamento e finestre amp/freq.
echo ""
echo "=== GRUPPO 6: random_walk ==="

set_param mode random_walk
set_param trial_duration_sec $DURATION_ROS
set_param rand_seed 0        # sempre diverso

# cfg: smooth_alpha  update_min  update_max  amp_min  amp_max  freq_min  freq_max
for rw_cfg in \
    "0.10 0.5 1.5 0.3 0.518 0.5 1.0" \
    "0.20 0.4 1.2 0.3 0.518 0.5 1.0" \
    "0.30 0.3 0.9 0.3 0.518 0.5 1.0" \
    "0.15 0.5 1.5 0.2 0.45  0.4 0.9" \
    "0.25 0.3 1.0 0.3 0.518 0.6 1.2" \
    "0.10 0.6 1.8 0.15 0.4  0.5 1.0" \
    "0.35 0.2 0.7 0.3 0.518 0.5 1.3" \
    "0.20 0.4 1.2 0.25 0.5  0.4 1.0"; do
    read alpha umin umax amin amax fmin fmax <<< "$rw_cfg"
    set_param rand_smooth_alpha $alpha
    set_param rand_update_min_sec $umin
    set_param rand_update_max_sec $umax
    set_param amp_min_rad $amin
    set_param amp_max_rad $amax
    set_param freq_min_hz $fmin
    set_param freq_max_hz $fmax
    echo ""
    echo "  random_walk | alpha=${alpha} update=[${umin}, ${umax}]s amp=[${amin}, ${amax}] freq=[${fmin}, ${fmax}]"
    start_trial
    wait_trial $DURATION
    TOTAL_TRIALS=$((TOTAL_TRIALS + 1))
done

# ── Gruppo 7: chaotic_stop ───────────────────────────
# Cambi bruschi di amp/freq + pause casuali. seed=0 → ogni trial diverso.
# Nessun turning offset: bias(0.903)+amp_max(0.518)=1.421 OK.
# Variamo probabilità/durata degli stop, cadenza dei cambi e finestre amp/freq.
echo ""
echo "=== GRUPPO 7: chaotic_stop ==="

set_param mode chaotic_stop
set_param trial_duration_sec $DURATION_ROS
set_param rand_seed 0        # sempre diverso

# cfg: stop_prob  stop_min  stop_max  update_min  update_max  amp_min  amp_max  freq_min  freq_max
for cs_cfg in \
    "0.20 0.3 1.0 0.4 1.2 0.3 0.518 0.5 1.0" \
    "0.35 0.4 1.5 0.4 1.2 0.3 0.518 0.5 1.0" \
    "0.25 0.3 0.8 0.6 1.5 0.3 0.518 0.5 1.0" \
    "0.15 0.2 0.6 0.4 1.0 0.3 0.518 0.6 1.2" \
    "0.30 0.5 1.2 0.5 1.3 0.2 0.45  0.4 0.9" \
    "0.40 0.4 1.0 0.3 0.9 0.3 0.518 0.5 1.0" \
    "0.25 0.3 1.0 0.5 1.5 0.15 0.4  0.5 1.1" \
    "0.20 0.4 1.4 0.4 1.2 0.25 0.5  0.5 1.3"; do
    read sprob smin smax umin umax amin amax fmin fmax <<< "$cs_cfg"
    set_param stop_prob $sprob
    set_param stop_min_sec $smin
    set_param stop_max_sec $smax
    set_param rand_update_min_sec $umin
    set_param rand_update_max_sec $umax
    set_param amp_min_rad $amin
    set_param amp_max_rad $amax
    set_param freq_min_hz $fmin
    set_param freq_max_hz $fmax
    echo ""
    echo "  chaotic_stop | stop_prob=${sprob} stop=[${smin}, ${smax}]s update=[${umin}, ${umax}]s amp=[${amin}, ${amax}] freq=[${fmin}, ${fmax}]"
    start_trial
    wait_trial $DURATION
    TOTAL_TRIALS=$((TOTAL_TRIALS + 1))
done

echo ""
echo "================================================"
echo "  Dataset completo: ${TOTAL_TRIALS} trial in ./src/net/dataset"
echo "================================================"