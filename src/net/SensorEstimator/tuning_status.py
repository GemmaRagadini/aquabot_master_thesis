"""Stato del tuning: trial completati/in corso per ogni studio + best attuale.
Uso:  python tuning_status.py [--storage sqlite:///tuning_results/optuna_fish.db] [--watch]
"""
import argparse
import math
import os
import subprocess
import time

import optuna
from optuna.trial import TrialState

optuna.logging.set_verbosity(optuna.logging.WARNING)

# directory di questo script -> il default dello storage e' ancorato qui
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

TOTALS = {  # trial attesi per studio (per sapere quanto manca)
    "fish_forward_phase1":       16,
    "fish_forward_phase2_arch1": 30,
    "fish_forward_phase2_arch2": 30,
    "fish_forward_phase3":       50,
}


def show(storage_url):
    try:
        summaries = optuna.study.get_all_study_summaries(storage=storage_url)
    except Exception as e:
        print(f"Impossibile leggere lo storage: {e}")
        return

    if not summaries:
        print("Nessuno studio nel database.")
        return

    print(f"{'studio':<28} {'done':>5} {'run':>4} {'prun':>5} {'fail':>5} {'tot':>7}   best")
    print("-" * 78)
    for s in sorted(summaries, key=lambda x: x.study_name):
        study = optuna.load_study(study_name=s.study_name, storage=storage_url)
        trials = study.trials
        done   = sum(t.state == TrialState.COMPLETE for t in trials)
        run    = sum(t.state == TrialState.RUNNING  for t in trials)
        pruned = sum(t.state == TrialState.PRUNED   for t in trials)
        fail   = sum(t.state == TrialState.FAIL     for t in trials)
        total  = TOTALS.get(s.study_name)
        tot_s  = f"{done}/{total}" if total else str(done)

        finite = [t for t in trials
                  if t.value is not None and math.isfinite(t.value)]
        if finite:
            b = min(finite, key=lambda t: t.value)
            best = f"{b.value:.4f}  {b.params}"
        else:
            best = "-"
        print(f"{s.study_name:<28} {done:>5} {run:>4} {pruned:>5} {fail:>5} {tot_s:>7}   {best}")

    # worker attivi
    try:
        out = subprocess.run(["pgrep", "-af", "tune.py --phase"],
                             capture_output=True, text=True).stdout.strip()
        n = len(out.splitlines()) if out else 0
        print(f"\nWorker attivi: {n}")
    except Exception:
        pass


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--storage", default=f"sqlite:///{os.path.join(SCRIPT_DIR, 'tuning_results', 'optuna_fish.db')}")
    parser.add_argument("--watch", action="store_true",
                        help="aggiorna ogni 30 secondi (Ctrl-C per uscire)")
    args = parser.parse_args()

    if args.watch:
        while True:
            print("\033c", end="")   # clear screen
            print(time.strftime("%H:%M:%S"), "\n")
            show(args.storage)
            time.sleep(30)
    else:
        show(args.storage)