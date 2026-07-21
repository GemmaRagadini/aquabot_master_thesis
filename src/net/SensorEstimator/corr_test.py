"""
Test diagnostico delle correlazioni per capire perche' sensor_mean
si apprende peggio di sensor_diff.
Calcola:
    corr(cmd_servo, sensor_diff)   -> atteso ALTO  (il comando pilota diff)
    corr(cmd_servo, sensor_mean)   -> atteso BASSO (il comando si cancella)
    corr(L, R)                     -> se ~ -1 : controfase pura, mean ~ rumore
                                      se mista: c'e' struttura comune recuperabile

Uso:
    python corr_test.py --dataset_dir ./src/net/dataset
"""
import argparse
import ast
from pathlib import Path

import numpy as np
import pandas as pd


def parse_sensor_values(series):
    """Identico a FishDataset._parse_sensor_values."""
    def parse_one(s):
        try:
            vals = ast.literal_eval(str(s))
            if len(vals) < 2:
                return [0.0, 0.0]
            out = [float(v) for v in vals[:2]]
            if not all(np.isfinite(out)):
                return [np.nan, np.nan]
            return out
        except Exception:
            return [np.nan, np.nan]

    parsed = series.apply(parse_one)
    return np.array(parsed.tolist(), dtype=np.float32)


def channels_from_df(df):
    """Ricostruisce L, R, sensor_diff_cal, sensor_mean_cal, cmd_servo
    come fa dataset.py._process_episode (senza normalizzazione: la
    correlazione e' invariante a scala/offset, non serve normalizzare)."""
    sensors = parse_sensor_values(df["sensor_values"])

    # interpolazione dei NaN, come nel dataset
    if np.isnan(sensors).any():
        for k in range(sensors.shape[1]):
            col = sensors[:, k]
            mask = np.isnan(col)
            if mask.all():
                return None
            col[mask] = np.interp(np.flatnonzero(mask),
                                  np.flatnonzero(~mask), col[~mask])
            sensors[:, k] = col

    L = sensors[:, 0]
    R = sensors[:, 1]

    sensor_diff = L - R
    sensor_mean = (L + R) / 2.0

    # calibrazione offset sui primi 50 campioni (identica al dataset)
    sensor_diff_cal = sensor_diff - sensor_diff[:50].mean()
    sensor_mean_cal = sensor_mean - sensor_mean[:50].mean()

    cmd_servo = pd.to_numeric(df["tail_target_rad"], errors="coerce").values.astype(np.float32)

    return L, R, sensor_diff_cal, sensor_mean_cal, cmd_servo


def safe_corr(a, b):
    """Correlazione di Pearson robusta a canali costanti (std=0)."""
    a = np.asarray(a, dtype=np.float64)
    b = np.asarray(b, dtype=np.float64)
    m = np.isfinite(a) & np.isfinite(b)
    a, b = a[m], b[m]
    if len(a) < 3 or a.std() < 1e-9 or b.std() < 1e-9:
        return np.nan
    return float(np.corrcoef(a, b)[0, 1])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dataset_dir", default="./src/net/dataset")
    args = ap.parse_args()

    csv_files = sorted(Path(args.dataset_dir).glob("trial_*.csv"))
    if not csv_files:
        raise FileNotFoundError(f"Nessun trial_*.csv in {args.dataset_dir}")

    print(f"Trovati {len(csv_files)} trial.\n")
    print(f"{'trial':<22}{'corr(cmd,diff)':>16}{'corr(cmd,mean)':>16}{'corr(L,R)':>12}")
    print("-" * 66)

    rows = []
    # accumulatori per la correlazione aggregata (concatenando i canali
    # gia' calibrati per-episodio, cosi' l'aggregato e' coerente col dataset)
    all_cmd, all_diff, all_mean, all_L, all_R = [], [], [], [], []

    for path in csv_files:
        try:
            df = pd.read_csv(path)
            res = channels_from_df(df)
            if res is None:
                print(f"{path.name:<22}{'--- skip (sensori vuoti) ---':>44}")
                continue
            L, R, diff, mean, cmd = res

            c_cd = safe_corr(cmd, diff)
            c_cm = safe_corr(cmd, mean)
            c_lr = safe_corr(L, R)
            rows.append((c_cd, c_cm, c_lr))

            all_cmd.append(cmd); all_diff.append(diff); all_mean.append(mean)
            all_L.append(L);     all_R.append(R)

            print(f"{path.name:<22}{c_cd:>16.3f}{c_cm:>16.3f}{c_lr:>12.3f}")
        except Exception as e:
            print(f"{path.name:<22}  errore: {e}")

    if not rows:
        print("\nNessun trial valido.")
        return

    rows = np.array(rows)

    print("-" * 66)
    # media delle correlazioni per-trial (in modulo, cosi' i segni opposti
    # tra trial non si cancellano tra loro)
    print(f"{'MEDIA |corr| per-trial':<22}"
          f"{np.nanmean(np.abs(rows[:, 0])):>16.3f}"
          f"{np.nanmean(np.abs(rows[:, 1])):>16.3f}"
          f"{np.nanmean(np.abs(rows[:, 2])):>12.3f}")

    # correlazione aggregata su tutti i campioni concatenati
    cmd_all  = np.concatenate(all_cmd)
    diff_all = np.concatenate(all_diff)
    mean_all = np.concatenate(all_mean)
    L_all    = np.concatenate(all_L)
    R_all    = np.concatenate(all_R)
    print(f"{'AGGREGATO (concat)':<22}"
          f"{safe_corr(cmd_all, diff_all):>16.3f}"
          f"{safe_corr(cmd_all, mean_all):>16.3f}"
          f"{safe_corr(L_all, R_all):>12.3f}")

    print("\nLettura:")
    print("  |corr(cmd,diff)| alto  e  |corr(cmd,mean)| basso")
    print("     -> conferma: il comando spiega diff ma non mean.")
    print("  corr(L,R) ~ -1  -> controfase pura, sensor_mean e' quasi solo")
    print("     rumore/drift: de-pesalo o rimuovilo dalla loss.")
    print("  corr(L,R) mista (-0.3..-0.7) -> c'e' struttura comune reale:")
    print("     recuperabile solo con input piu' ricco (corrente, cmd^2, storia sensori).")


if __name__ == "__main__":
    main()