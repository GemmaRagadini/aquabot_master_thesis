#!/bin/bash
# collect_random.sh
# Genera il dataset con la modalita' 'random_continuous'.
# I nodi ROS2 devono essere gia' in esecuzione (es. dal launch file con
# mode: random_continuous). collect_dataset.sh resta separato per le modalita'
# vecchie: questo script e' SOLO per la generazione casuale.
#
# UN SOLO trial lungo = UN SOLO CSV.
# La registrazione parte una volta (start) e si ferma una volta (stop): tutti i
# cambi casuali di amp/freq/center finiscono nello STESSO file trial_*.csv.
#
# Lo spazio esplorato (amp/freq/center) e' l'UNIONE di quello che coprivano i
# gruppi di collect_dataset.sh, ma percorso in modo casuale con moltissime piu'
# configurazioni. Il nodo garantisce sempre bias +/- |offset| +/- amp in
# [0.385, 1.422].
#
# Uso:
#   ./collect_dataset_random.sh                 # run completo (default 2700s = 45min)
#   ./collect_dataset_random.sh --secs 180      # TEST breve di 3 minuti
#   ./collect_dataset_random.sh --dry-run       # stampa i comandi, non esegue nulla
#   ./collect_dataset_random.sh --secs 120 --dry-run

DRY_RUN=false
DURATION=3600     # secondi => 1h

while [[ $# -gt 0 ]]; do
case "$1" in
--dry-run|--dry_run) DRY_RUN=true; shift ;;
--secs|--seconds)    DURATION="$2"; shift 2 ;;
*) echo "argomento sconosciuto: $1"; exit 1 ;;
esac
done
$DRY_RUN && echo "[DRY RUN] Nessun comando verra' eseguito."

# --- Range esplorati (UNIONE dei vecchi gruppi) ---
AMP_MIN=0.10      # abbassato ~3 gradi (era 0.15 rad = 8.6 deg; ora 0.10 rad = 5.7 deg)
AMP_MAX=0.518     # massimo fisico (< 0.519). NON superare.
FREQ_MIN=0.3      # min visto in amp_sweep/combined_sweep
FREQ_MAX=1.5      # max visto in combined_sweep
CENTER_MAX=0.30   # offset centro max (turning_fixed arrivava a 0.30)

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