"""
Divide un CSV unico in trial da DURATA secondi: trial_1.csv, trial_2.csv, ...

Il taglio usa la colonna tempo (--time_col). Se il tempo non e' presente, si
taglia per numero di righe con --rate (Hz). Ogni trial riceve la colonna
t_rel_sec che riparte da 0, usata dagli script di plot per l'asse temporale.

L'ultimo pezzo viene scartato se e' piu' corto di --min_frac * durata.

Uso:
  python3 split_trials.py dataset_unico.csv --out_dir src/net/dataset --duration 30
  python3 split_trials.py dataset_unico.csv --out_dir src/net/dataset --duration 30 \
      --resample_on_sensor            # 20 Hz -> 10 Hz, una riga per lettura sensore
  python3 split_trials.py dataset_unico.csv --out_dir src/net/dataset --rate 10   # senza colonna tempo
"""
import argparse
from pathlib import Path

import numpy as np
import pandas as pd

TIME_CANDIDATES = ["t_rel_sec", "t_sec", "time", "timestamp", "t"]


def resample_on_sensor_updates(df, current_mode="last"):
    """Tiene solo le righe in cui sensor_values cambia rispetto alla riga prima.

    I sensori arrivano a ~10 Hz mentre il log e' a 20 Hz: le righe intermedie
    ripetono il valore vecchio. Tenendo solo le righe 'fresche' ogni campione
    ha un sensore nuovo e il passo temporale diventa ~0.1 s.
    Comando, contesto e tempo sono quelli della riga tenuta (valori a quell'istante).
    La corrente (rumorosa, 20 Hz vera) con current_mode='mean' e' la media delle
    righe dall'aggiornamento precedente escluso fino a quello attuale incluso:
    solo passato, nessuna informazione dal futuro."""
    sv = df["sensor_values"].astype(str)
    fresh = (sv != sv.shift(1)).to_numpy().copy()
    fresh[0] = True
    keep = np.flatnonzero(fresh)

    out = df.iloc[keep].copy()
    if current_mode == "mean":
        cur = pd.to_numeric(df["present_current_ma"], errors="coerce").to_numpy(dtype=float)
        starts = np.r_[0, keep[:-1] + 1]
        out["present_current_ma"] = [np.nanmean(cur[s:e + 1]) for s, e in zip(starts, keep)]

    if "t_rel_sec" in df.columns:
        dt = np.diff(pd.to_numeric(out["t_rel_sec"], errors="coerce").to_numpy(dtype=float))
        med = float(np.median(dt))
        irregular = int(np.sum(np.abs(dt - med) > 0.5 * med))
        print(f"Ricampionamento sugli aggiornamenti sensore: {len(df)} -> {len(out)} righe, "
              f"passo mediano {med:.3f}s ({1 / med:.1f} Hz), passi irregolari: {irregular} "
              f"({irregular / max(len(dt), 1):.1%})")
        if irregular / max(len(dt), 1) > 0.02:
            print("  [avviso] piu' del 2% di passi irregolari: sensori persi o valori "
                  "identici per due letture consecutive. Controlla prima di allenare.")
    else:
        print(f"Ricampionamento sugli aggiornamenti sensore: {len(df)} -> {len(out)} righe")
    return out.reset_index(drop=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv", help="CSV unico da dividere")
    ap.add_argument("--out_dir", required=True)
    ap.add_argument("--duration", type=float, default=30.0, help="durata di ogni trial [s]")
    ap.add_argument("--time_col", default=None,
                    help=f"colonna tempo in secondi. Default: la prima trovata tra {TIME_CANDIDATES}")
    ap.add_argument("--rate", type=float, default=None,
                    help="frequenza di campionamento [Hz]: se data, taglia per numero di righe")
    ap.add_argument("--min_frac", type=float, default=0.5,
                    help="scarta l'ultimo pezzo se piu' corto di min_frac*duration")
    ap.add_argument("--resample_on_sensor", action="store_true",
                    help="tieni solo le righe in cui sensor_values si aggiorna "
                         "(20 Hz -> ~10 Hz, niente campioni sensore duplicati)")
    ap.add_argument("--current_mode", default="last", choices=["mean", "last"],
                    help="con --resample_on_sensor: corrente = media delle righe "
                         "dall'ultimo aggiornamento (mean, riduce l'aliasing del "
                         "rumore) o valore della riga tenuta (last)")
    ap.add_argument("--overwrite", action="store_true",
                    help="consenti di sovrascrivere trial_*.csv gia' presenti in out_dir")
    args = ap.parse_args()

    df = pd.read_csv(args.csv)
    if args.resample_on_sensor:
        df = resample_on_sensor_updates(df, args.current_mode)
    out = Path(args.out_dir)
    out.mkdir(parents=True, exist_ok=True)

    existing = sorted(out.glob("trial_*.csv"))
    if existing and not args.overwrite:
        raise SystemExit(f"{out} contiene gia' {len(existing)} trial_*.csv: "
                         f"svuotala o usa --overwrite (evita di mescolare vecchi e nuovi).")
    for f in existing:
        f.unlink()

    # --- tempo in secondi per ogni riga ---
    if args.rate is not None:
        t = np.arange(len(df)) / args.rate
        src = f"indice riga / {args.rate} Hz"
    else:
        col = args.time_col or next((c for c in TIME_CANDIDATES if c in df.columns), None)
        if col is None:
            raise SystemExit(f"Nessuna colonna tempo tra {TIME_CANDIDATES}: "
                             f"passa --time_col o --rate.")
        t = pd.to_numeric(df[col], errors="coerce").to_numpy(dtype=float)
        if np.isnan(t).any():
            raise SystemExit(f"Colonna tempo '{col}' con valori non numerici/NaN.")
        dt = np.diff(t)
        if (dt < 0).any():
            raise SystemExit(f"Colonna tempo '{col}' non monotona ({int((dt < 0).sum())} "
                             f"salti indietro): il CSV contiene piu' registrazioni concatenate?")
        src = f"colonna '{col}'"
    t = t - t[0]

    # --- assegnazione al trial e scrittura ---
    idx = np.floor(t / args.duration).astype(int)
    n_written, n_dropped = 0, 0
    for k in np.unique(idx):
        chunk = df[idx == k].copy()
        tc = t[idx == k]
        span = tc[-1] - tc[0]
        if span < args.min_frac * args.duration:
            n_dropped += 1
            print(f"  scartato pezzo {k}: {span:.1f}s, {len(chunk)} righe (troppo corto)")
            continue
        chunk["t_rel_sec"] = tc - tc[0]
        n_written += 1
        chunk.to_csv(out / f"trial_{n_written}.csv", index=False)

    rows = len(df) / max(n_written + n_dropped, 1)
    print(f"Tempo da {src}. Scritti {n_written} trial da {args.duration:.0f}s in {out} "
          f"(~{rows:.0f} righe l'uno), scartati {n_dropped}.")


if __name__ == "__main__":
    main()