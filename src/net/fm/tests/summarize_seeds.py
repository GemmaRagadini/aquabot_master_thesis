"""
Riassume i CSV di metrics.py ottenuti con seed diversi: media +- std sui seed
delle metriche one-step (riga POOLED), per GRU e LSTM, piu' le baseline
(persistenza e ARX, che non dipendono dal seed).

Stampa la tabella e le righe LaTeX (siunitx) per la tesi.

Uso:
  python3 src/net/fm/tests/summarize_seeds.py \
      --gru  metrics_gru_s1.csv  metrics_gru_s2.csv  ... \
      --lstm metrics_lstm_s1.csv metrics_lstm_s2.csv ...
"""
import argparse

import numpy as np
import pandas as pd

CHANNELS = ["sensor_diff", "current"]
METRICS  = ["RMSE", "nRMSE"]
DEC      = {"RMSE": 2, "nRMSE": 3}


def pooled_row(path):
    df = pd.read_csv(path)
    row = df[df["split"].astype(str).str.startswith("POOLED")]
    if row.empty:
        raise SystemExit(f"{path}: nessuna riga POOLED (lancia metrics.py con --agg pooled o both)")
    return row.iloc[0]


def collect(paths, suffix=""):
    """{(canale, metrica): array sui file} per le colonne <ch>_<METRICA><suffix>."""
    rows = [pooled_row(p) for p in paths]
    out = {}
    for ch in CHANNELS:
        for mt in METRICS:
            col = f"{ch}_{mt}{suffix}"
            if col in rows[0].index:
                out[(ch, mt)] = np.array([float(r[col]) for r in rows])
    return out


def fmt(x, mt, latex=False):
    d = DEC[mt]
    num = lambda v: "\\num{" + format(v, f".{d}f") + "}"
    if len(x) > 1:
        m, s = x.mean(), x.std(ddof=1)
        return ("$" + num(m) + " \\pm " + num(s) + "$" if latex
                else f"{m:.{d}f} ± {s:.{d}f}")
    return num(x[0]) if latex else f"{x[0]:.{d}f}"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--gru", nargs="+", required=True, help="CSV di metrics.py, uno per seed")
    ap.add_argument("--lstm", nargs="+", default=None, help="CSV di metrics.py, uno per seed")
    a = ap.parse_args()

    models = [("Persistence", collect(a.gru[:1], "_persist")),
              ("ARX",         collect(a.gru[:1], "_arx"))]
    if a.lstm:
        models.append((f"LSTM ({len(a.lstm)} seeds)", collect(a.lstm)))
    models.append((f"GRU ({len(a.gru)} seeds)", collect(a.gru)))

    cols = [(ch, mt) for ch in CHANNELS for mt in METRICS]
    head = ["Model"] + [f"{ch} {mt}" for ch, mt in cols]
    print("\n" + " | ".join(f"{h:>24s}" for h in head))
    for name, res in models:
        cells = [fmt(res[c], c[1]) if c in res else "-" for c in cols]
        print(" | ".join(f"{x:>24s}" for x in [name] + cells))

    print("\nRighe LaTeX (sensor_diff RMSE & nRMSE & current RMSE & nRMSE):")
    for name, res in models:
        cells = [fmt(res[c], c[1], latex=True) if c in res else "--" for c in cols]
        print(f"    {name.split(' (')[0]} & " + " & ".join(cells) + r" \\")
    print("\nRMSE: sensor_diff in sensor units, current in mA. nRMSE = RMSE / std del segnale.")


if __name__ == "__main__":
    main()