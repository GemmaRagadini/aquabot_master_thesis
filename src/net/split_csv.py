"""
Divide un CSV lungo (registrazione continua) in trial da 30 s, con lo stesso
formato dei trial di training/validation: trial_TS_1.csv, trial_TS_2.csv, ...

- Se nel CSV c'e' una colonna di tempo (default: t_rel_sec, oppure --time_col)
  il taglio e' fatto sul TEMPO, cosi' resta giusto anche se la frequenza di
  logging non e' esattamente 10 Hz. Altrimenti si taglia per numero di righe
  (--hz * --duration).
- In ogni trial la colonna di tempo viene riportata a partire da 0.
- L'ultimo pezzo viene tenuto solo se dura almeno --min_duration secondi.
- Avvisa se dentro un trial ci sono buchi nel tempo (log interrotto).

Uso:
  python3 split_test_csv.py datasetlungo.csv src/net/test_set
  python3 split_test_csv.py datasetlungo.csv src/net/test_set --duration 30 --min_duration 20

"""
import argparse
from pathlib import Path

import numpy as np
import pandas as pd


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("input", help="CSV lungo da dividere")
    ap.add_argument("out_dir", help="cartella di destinazione (separata dal dataset di training)")
    ap.add_argument("--duration", type=float, default=30.0, help="durata di ogni trial (s)")
    ap.add_argument("--min_duration", type=float, default=20.0,
                    help="l'ultimo pezzo si tiene solo se dura almeno tanto (s)")
    ap.add_argument("--time_col", default="t_rel_sec",
                    help="colonna di tempo in secondi (se manca si taglia per righe)")
    ap.add_argument("--hz", type=float, default=10.0,
                    help="frequenza di logging, usata solo se manca la colonna di tempo")
    ap.add_argument("--prefix", default="trial_TS_")
    ap.add_argument("--start", type=int, default=1, help="numero del primo trial")
    ap.add_argument("--overwrite", action="store_true", help="sovrascrive file gia' esistenti")
    args = ap.parse_args()

    df = pd.read_csv(args.input)
    print(f"Letto {args.input}: {len(df)} righe, colonne: {list(df.columns)}")
    out = Path(args.out_dir)
    out.mkdir(parents=True, exist_ok=True)

    # ---- confini dei trial ----
    use_time = args.time_col in df.columns
    if use_time:
        t = pd.to_numeric(df[args.time_col], errors="coerce").to_numpy(dtype=float)
        if np.isnan(t).any() or np.any(np.diff(t) < 0):
            raise SystemExit(f"La colonna {args.time_col} ha valori mancanti o non crescenti: "
                             f"controlla il CSV o usa --time_col.")
        t = t - t[0]
        dt = np.diff(t)
        print(f"Taglio sul tempo ({args.time_col}): durata {t[-1]:.1f} s, "
              f"passo mediano {np.median(dt):.3f} s (~{1 / np.median(dt):.1f} Hz)")
        chunk = np.floor(t / args.duration).astype(int)
        gap_lim = 5 * np.median(dt)
        hz_est = 1 / np.median(dt)
    else:
        n = int(round(args.hz * args.duration))
        print(f"Colonna '{args.time_col}' non trovata: taglio per righe ({n} righe = "
              f"{args.duration:.0f} s a {args.hz:.0f} Hz)")
        chunk = np.arange(len(df)) // n
        t = np.arange(len(df)) / args.hz
        hz_est = args.hz

    # ---- scrittura ----
    k = args.start
    written = []
    for c in np.unique(chunk):
        idx = np.flatnonzero(chunk == c)
        dur = len(idx) / hz_est                       # durata coperta dai campioni
        if dur < args.min_duration:
            print(f"  scarto l'ultimo pezzo: {dur:.1f} s < {args.min_duration:.0f} s")
            continue
        part = df.iloc[idx].copy()
        if use_time:
            part[args.time_col] = t[idx] - t[idx[0]]
            gaps = np.diff(t[idx])
            if (gaps > gap_lim).any():
                print(f"  [attenzione] {args.prefix}{k}: buco nel log di "
                      f"{gaps.max():.2f} s a t={t[idx][np.argmax(gaps)] - t[idx[0]]:.1f} s")
        path = out / f"{args.prefix}{k}.csv"
        if path.exists() and not args.overwrite:
            raise SystemExit(f"{path} esiste gia': usa --overwrite o un'altra cartella.")
        part.to_csv(path, index=False)
        written.append((path.name, len(part), dur))
        k += 1

    print(f"\nScritti {len(written)} trial in {out}/:")
    for name, rows, dur in written:
        print(f"  {name:20s} {rows:4d} righe  {dur:5.1f} s")


if __name__ == "__main__":
    main()