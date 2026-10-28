#!/bin/bash
# collect_dataset_random.sh
# Genera il dataset con la modalita' 'random_continuous'.
# I nodi ROS2 devono essere gia' in esecuzione con system_launch.py.

# Genera un solo csv lungo

# Limiti meccanici (tick 1860 - 2320, centro 2090):
#   tail_min = -0.288 rad, tail_max = 0.417 rad, centro (bias) = 0.064 rad
#   semiampiezza massima MAX_AMP_RAD = 0.353 rad
# Il master garantisce sempre |offset centro| + amp <= MAX_AMP_RAD, quindi
# bias +/- |offset| +/- amp resta in [-0.288, 0.417].
#
# Uso:
#   ./collect_dataset_random.sh                 # run completo (default 600 s = 10 min)
#   ./collect_dataset_random.sh --secs 180      # TEST breve di 3 minuti
#   ./collect_dataset_random.sh --dry-run       # stampa i comandi, non esegue nulla
#   ./collect_dataset_random.sh --secs 120 --dry-run

DRY_RUN=false
DURATION=60     # secondi

while [[ $# -gt 0 ]]; do
    case "$1" in
        --dry-run|--dry_run) DRY_RUN=true; shift ;;
        --secs|--seconds)    DURATION="$2"; shift 2 ;;
        *) echo "argomento sconosciuto: $1"; exit 1 ;;
    esac
done
$DRY_RUN && echo "[DRY RUN] Nessun comando verra' eseguito."

# --- Range esplorati
AMP_MIN=0.10
AMP_MAX=0.34      # sotto il massimo fisico 0.353 (circa 8 tick di margine). NON superare 0.353.
FREQ_MIN=0.3
FREQ_MAX=2.0      # alzata da 1.5 per compensare l'ampiezza minore (velocita' di picco ~4.3 rad/s).
                  # Con log a 10 Hz sono 5 campioni per ciclo: NON salire oltre senza alzare log_rate_hz.
CENTER_MAX=0.25   # offset centro max; il master lo riduce comunque a MAX_AMP_RAD - amp corrente

# --- Caratteristiche della variazione casuale ---
HOLD=4.0          # ogni quanti secondi cambia un parametro
STEP_PROB=0.5     # 0.5 = meta' cambi netti, meta' continui

run() {
    if $DRY_RUN; then echo "  >> $*"; else eval "$@"; fi
}
set_param() { run "ros2 param set /master_node $1 $2"; }
start_trial() { run "ros2 service call /trial std_srvs/srv/SetBool '{data: true}'"; }
stop_trial()  { run "ros2 service call /trial std_srvs/srv/SetBool '{data: false}'"; }

echo "================================================"
echo "  Dataset random_continuous — 1 trial, 1 CSV"
echo "  durata=${DURATION}s hold=${HOLD}s step_prob=${STEP_PROB}"
echo "  amp[${AMP_MIN},${AMP_MAX}] freq[${FREQ_MIN},${FREQ_MAX}] center_max=${CENTER_MAX}"
echo "  limiti coda [-0.288, 0.417] rad, centro 0.064 rad"
echo "================================================"

set_param mode random_continuous
set_param amp_min_rad $AMP_MIN
set_param amp_max_rad $AMP_MAX
set_param freq_min_hz $FREQ_MIN
set_param freq_max_hz $FREQ_MAX
set_param random_center_max_rad $CENTER_MAX
set_param random_hold_sec  $HOLD
set_param random_step_prob $STEP_PROB

# UNA registrazione sola -> UN CSV solo
echo ""
echo "  registro per ${DURATION}s in un unico CSV ..."
start_trial
if $DRY_RUN; then echo "  >> sleep $DURATION"; else sleep "$DURATION"; fi
stop_trial

echo ""
echo "================================================"
echo "  Fatto. Un unico CSV in ./logs (o nel log_dir configurato)."
echo "  Verifica: python3 preflight_random.py --csv logs/trial_<timestamp>.csv"
echo "================================================"