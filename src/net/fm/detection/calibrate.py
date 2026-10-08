"""
CALIBRAZIONE del detector su trial NORMALI: sigma, soglie, leave-one-trial-out.

Ogni trial viene fatto passare nello STESSO detector usato online, con sigma=1
e soglie infinite: cosi' il rollout viene riagganciato con la stessa cadenza
che avra' online (ogni min_reanchor_s) e i residui di calibrazione hanno la
stessa statistica di quelli online.

  1. residui a un passo e in rollout per ogni trial
  2. sigma[modo, canale] = RMS del residuo su tutti i trial
  3. per ogni trial: massimo nel tempo dello score
  4. soglia = margin * quantile_q dei massimi per trial (default q=1: il massimo)
  5. leave-one-trial-out: soglia ricalcolata senza il trial i, poi si guarda di
     quanto il trial i la supera (rapporto > 1 = falso allarme)

Di default usa SOLO i trial di validation: sui trial di train il FM ha residui
piu' piccoli del vero e le soglie verrebbero troppo strette.

Uso:
  python3 src/net/fm/detection/calibrate.py --checkpoint src/net/fm/checkpoints_fm/best_fm_tuned.pt
  python3 .../calibrate.py --checkpoint ... --margin 1.2 --W 0.5 --min_reanchor 20
Scrive <checkpoint>_detector.json (o --out).
"""
import argparse

import numpy as np
import pandas as pd
import torch

from det_common import (DEFAULT_DATASET, DEFAULT_SCALER, default_calib_path,
                        load_dataset, load_model, replay, trial_series)
from detector import (CHANNELS, MODES, MR, Calibration, DetectorConfig,
                      PerturbationDetector)


def named(a):
    """(2,2) [modo, canale] -> {'S1_sensor_diff': ..., 'SR_current': ...}"""
    return {f"S{m}_{c}": float(a[mi, ci])
            for mi, m in enumerate(MODES) for ci, c in enumerate(CHANNELS)}


def main():
    d = DetectorConfig()
    ap = argparse.ArgumentParser()
    ap.add_argument("--checkpoint", required=True)
    ap.add_argument("--dataset_dir", default=DEFAULT_DATASET)
    ap.add_argument("--scaler_path", default=DEFAULT_SCALER,
                    help="DEVE essere lo stesso scaler del training")
    ap.add_argument("--device", default="cpu")
    ap.add_argument("--out", default=None, help="JSON di calibrazione")
    ap.add_argument("--all_trials", action="store_true",
                    help="usa anche i trial di train (sconsigliato: soglie ottimistiche)")
    ap.add_argument("--quantile", type=float, default=1.0,
                    help="quantile dei massimi per trial usato come soglia (1.0 = massimo)")
    ap.add_argument("--margin", type=float, default=1.0,
                    help="fattore moltiplicativo sulla soglia (per rialzarla)")
    ap.add_argument("--hz", type=float, default=d.hz)
    ap.add_argument("--W", type=float, default=d.W_s, help="finestra score (s)")
    ap.add_argument("--T", type=float, default=d.T_s)
    ap.add_argument("--Tc", type=float, default=d.Tc_s)
    ap.add_argument("--Tb", type=float, default=d.Tb_s)
    ap.add_argument("--quiet", type=float, default=d.quiet_s)
    ap.add_argument("--min_reanchor", type=float, default=d.min_reanchor_s)
    args = ap.parse_args()

    cfg = DetectorConfig(hz=args.hz, W_s=args.W, T_s=args.T, Tc_s=args.Tc, Tb_s=args.Tb,
                         quiet_s=args.quiet, min_reanchor_s=args.min_reanchor)
    device = torch.device(args.device)
    FM, ckpt = load_model(args.checkpoint, device)
    dataset, val_trials = load_dataset(args.dataset_dir, args.scaler_path, ckpt)
    trials = list(range(len(dataset.trial_names))) if args.all_trials else val_trials

    det = PerturbationDetector(FM, Calibration.neutral(cfg), H=dataset.h, device=device)

    # ---- 1. residui e score grezzi (sigma=1) per trial ----
    names, sumsq, count, rawmax = [], [], [], []
    ages, res_R = [], []
    for ti in trials:
        cmd, sens = trial_series(dataset, ti)
        r = replay(det, cmd, sens)
        ok = np.isfinite(r["resid"][:, 0, 0])
        if not np.isfinite(r["score"]).any():
            print(f"  salto {dataset.trial_names[ti]}: troppo corto per uno score")
            continue
        names.append(dataset.trial_names[ti])
        sumsq.append((r["resid"][ok] ** 2).sum(axis=0))
        count.append(int(ok.sum()))
        rawmax.append(np.nanmax(r["score"], axis=0))       # RMS_W grezzo massimo
        ages.append(r["age"][ok])
        res_R.append(r["resid"][ok][:, MR, :])
    n = len(names)
    if n < 2:
        raise SystemExit("Servono almeno 2 trial per calibrare.")
    sumsq, count, rawmax = np.stack(sumsq), np.array(count), np.stack(rawmax)

    # ---- 2-4. sigma e soglie ----
    sigma = np.sqrt(sumsq.sum(axis=0) / count.sum())                     # (2,2)
    thr_raw = args.margin * np.quantile(rawmax, args.quantile, axis=0)   # (2,2)
    threshold = thr_raw / sigma

    # ---- 5. leave-one-trial-out ----
    # Lo score e' RMS_W(e)/sigma e la soglia e' un quantile degli stessi massimi
    # divisi per la stessa sigma: sigma si semplifica, il confronto si fa sui grezzi.
    ratio = np.zeros_like(rawmax)
    for i in range(n):
        others = np.delete(rawmax, i, axis=0)
        ratio[i] = rawmax[i] / (args.margin * np.quantile(others, args.quantile, axis=0))
    fa = ratio > 1.0
    fa_alarm = fa[:, MR, :].any(axis=1)        # un allarme = S_R di almeno un canale

    # ---- crescita del residuo in rollout col tempo dall'ultimo riaggancio ----
    ages, res_R = np.concatenate(ages), np.concatenate(res_R)
    blk = cfg.n(1.0)
    age_rows = []
    for b in range(int(np.ceil(ages.max() / blk))):
        sel = (ages > b * blk) & (ages <= (b + 1) * blk)
        if sel.sum() < 5:
            continue
        rel = np.sqrt((res_R[sel] ** 2).mean(axis=0)) / sigma[MR]
        age_rows.append({"eta_s": f"{b}-{b + 1}", "n": int(sel.sum()),
                         **{f"rmsR/sigma_{c}": float(rel[ci]) for ci, c in enumerate(CHANNELS)}})

    # ---- stampa ----
    pd.set_option("display.width", 220)
    print(f"\nCalibrazione su {n} trial ({'tutti' if args.all_trials else 'solo validation'}) | "
          f"W={cfg.W_s}s riaggancio ogni {cfg.min_reanchor_s}s | "
          f"quantile={args.quantile} margin={args.margin}")
    print("\nsigma (unita' normalizzate) e soglie (in multipli di sigma):")
    print(pd.DataFrame({"sigma": named(sigma), "soglia": named(threshold)}).round(4).to_string())

    df = pd.DataFrame([{"trial": names[i], **named(ratio[i])} for i in range(n)])
    print("\nLeave-one-trial-out: massimo score del trial escluso / soglia ricalcolata "
          "senza di lui (>1 = falso allarme)")
    print(df.round(3).to_string(index=False))
    print("\nFrazione di trial sopra soglia, per score:")
    print(pd.Series(named(fa.mean(axis=0))).round(3).to_string())
    print(f"Tasso di falso allarme per trial (S_R di almeno un canale): "
          f"{fa_alarm.mean():.3f}  ({int(fa_alarm.sum())}/{n})")
    if args.quantile >= 1.0:
        print("NB: con soglia = massimo, per ogni score il trial col massimo assoluto\n"
              "    supera SEMPRE la soglia ricalcolata senza di lui. Il numero utile e' di\n"
              "    QUANTO la supera: rapporti appena sopra 1 = soglia stabile; rapporti\n"
              "    molto sopra 1 = code pesanti, servono piu' trial o un --margin.")

    print("\nResiduo in rollout in funzione del tempo dall'ultimo riaggancio "
          "(1 = media; se cresce molto, sigma_R costante non basta):")
    print(pd.DataFrame(age_rows).round(3).to_string(index=False))

    out = args.out or default_calib_path(args.checkpoint)
    Calibration(sigma, threshold, cfg, meta={
        "checkpoint": args.checkpoint, "scaler_path": args.scaler_path,
        "H": int(dataset.h), "trials": names,
        "split": "all" if args.all_trials else "val",
        "quantile": args.quantile, "margin": args.margin,
        "per_trial_max_score": {names[i]: named(rawmax[i] / sigma) for i in range(n)},
        "loo_ratio": {names[i]: named(ratio[i]) for i in range(n)},
        "loo_false_alarm_rate_per_trial": float(fa_alarm.mean()),
        "rollout_rms_vs_age": age_rows,
    }).save(out)
    print(f"\nCalibrazione salvata in {out}")


if __name__ == "__main__":
    main()