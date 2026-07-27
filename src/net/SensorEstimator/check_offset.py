"""
check_offset.py — diagnostica la calibrazione per-episodio del modo comune.

Ipotesi da verificare: la loss alta su sensor_mean e' dovuta al fatto che lo
zero di riposo (offset_mean = media dei primi 50 campioni di sensor_mean) varia
molto da trial a trial. Se cosi', lo stesso comando produce sensor_mean_cal
diversi in trial diversi, e la rete non puo' che tagliare gli estremi.

Confronta la DISPERSIONE degli offset tra trial con l'AMPIEZZA tipica del
segnale calibrato: se sono dello stesso ordine, la calibrazione e' il colpevole.

Uso (dalla cartella src/net/SensorEstimator):
    python check_offset.py
    python check_offset.py --dataset_dir ../dataset
"""
import argparse
import ast
import os
from pathlib import Path

import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


def parse_sensor_values(series):
    def parse_one(s):
        try:
            vals = ast.literal_eval(str(s))
            if len(vals) < 2:
                return [np.nan, np.nan]
            out = [float(v) for v in vals[:2]]
            if not all(np.isfinite(out)):
                return [np.nan, np.nan]
            return out
        except Exception:
            return [np.nan, np.nan]
    return np.array(series.apply(parse_one).tolist(), dtype=np.float32)


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    repo = os.path.abspath(os.path.join(here, "..", "..", ".."))

    ap = argparse.ArgumentParser()
    ap.add_argument("--dataset_dir", default=os.path.join(repo, "src", "net", "dataset"))
    ap.add_argument("--n_calib", type=int, default=50,
                    help="quanti campioni iniziali per l'offset a riposo (come nel dataset)")
    ap.add_argument("--save_path", default="offset_diagnosis.png")
    args = ap.parse_args()

    csv_files = sorted(Path(args.dataset_dir).glob("trial_*.csv"))
    if not csv_files:
        raise FileNotFoundError(f"Nessun trial_*.csv in {args.dataset_dir}")
    print(f"Trovati {len(csv_files)} trial.\n")

    names          = []
    offset_mean    = []   # zero a riposo del modo comune, per trial
    offset_diff    = []   # zero a riposo della differenza, per trial (confronto)
    span_mean_cal  = []   # ampiezza (p1-p99) di sensor_mean calibrato, per trial

    for p in csv_files:
        try:
            df = pd.read_csv(p)
            sensors = parse_sensor_values(df["sensor_values"])
            # interpolazione minima dei NaN, per non falsare le statistiche
            for k in range(sensors.shape[1]):
                col = sensors[:, k]; m = np.isnan(col)
                if m.any() and (~m).any():
                    col[m] = np.interp(np.flatnonzero(m), np.flatnonzero(~m), col[~m])
                sensors[:, k] = col

            s_diff = sensors[:, 0] - sensors[:, 1]
            s_mean = (sensors[:, 0] + sensors[:, 1]) / 2.0

            off_m = float(np.mean(s_mean[:args.n_calib]))
            off_d = float(np.mean(s_diff[:args.n_calib]))

            mean_cal = s_mean - off_m
            # ampiezza robusta: distanza tra 1° e 99° percentile del calibrato
            span = float(np.percentile(mean_cal, 99) - np.percentile(mean_cal, 1))

            names.append(p.name)
            offset_mean.append(off_m)
            offset_diff.append(off_d)
            span_mean_cal.append(span)
        except Exception as e:
            print(f"  saltato {p.name}: {e}")

    offset_mean   = np.array(offset_mean)
    offset_diff   = np.array(offset_diff)
    span_mean_cal = np.array(span_mean_cal)

    # --- il confronto che decide tutto ---
    std_off_mean = offset_mean.std()
    std_off_diff = offset_diff.std()
    span_tipico  = np.median(span_mean_cal)
    ratio        = std_off_mean / (span_tipico + 1e-9)

    print("=" * 60)
    print("DISPERSIONE DEGLI OFFSET TRA TRIAL")
    print("=" * 60)
    print(f"  sensor_mean  offset:  std tra trial = {std_off_mean:8.2f}   "
          f"range = [{offset_mean.min():.1f}, {offset_mean.max():.1f}]")
    print(f"  sensor_diff  offset:  std tra trial = {std_off_diff:8.2f}   "
          f"(canale di controllo, quello che funziona)")
    print()
    print(f"  ampiezza tipica di sensor_mean calibrato (p1-p99): {span_tipico:.2f}")
    print(f"  RAPPORTO  std(offset_mean) / ampiezza  =  {ratio:.2f}")
    print("=" * 60)
    if ratio > 0.3:
        print(">> La dispersione degli offset e' RILEVANTE rispetto al segnale.")
        print(">> La calibrazione per-episodio e' probabilmente una causa")
        print(">> importante della loss alta su sensor_mean.")
    elif ratio > 0.1:
        print(">> Dispersione degli offset NON trascurabile: contribuisce, ma")
        print(">> forse non e' l'unica causa. Guarda anche gli estremi rari.")
    else:
        print(">> Gli offset sono abbastanza consistenti tra trial: la")
        print(">> calibrazione NON sembra essere il problema principale.")
        print(">> Guarda piuttosto la rarita' dei picchi estremi (loss Huber).")
    print("=" * 60)

    # --- figura ---
    fig, axes = plt.subplots(1, 2, figsize=(14, 5))

    ax = axes[0]
    ax.bar(range(len(offset_mean)), offset_mean, alpha=0.8)
    ax.axhline(offset_mean.mean(), color="red", ls="--",
               label=f"media = {offset_mean.mean():.1f}")
    ax.set_xlabel("indice trial"); ax.set_ylabel("offset_mean (zero a riposo)")
    ax.set_title(f"Offset del modo comune per trial\nstd tra trial = {std_off_mean:.2f}")
    ax.legend(); ax.grid(True, alpha=0.3)

    ax = axes[1]
    ax.hist(offset_mean, bins=max(5, len(offset_mean) // 3), alpha=0.8)
    ax.axvline(offset_mean.mean(), color="red", ls="--")
    ax.set_xlabel("offset_mean"); ax.set_ylabel("n. trial")
    ax.set_title(f"Distribuzione degli offset\nampiezza segnale ~ {span_tipico:.1f}, "
                 f"rapporto = {ratio:.2f}")
    ax.grid(True, alpha=0.3)

    fig.tight_layout()
    fig.savefig(args.save_path, dpi=140, bbox_inches="tight")
    print(f"\nsalvato {args.save_path}")


if __name__ == "__main__":
    main()