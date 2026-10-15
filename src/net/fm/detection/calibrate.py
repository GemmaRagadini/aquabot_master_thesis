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
import os

import numpy as np
import pandas as pd
import torch

from det_common import (FishJointDataset, DEFAULT_DATASET, DEFAULT_SCALER, default_calib_path,
                        load_dataset, load_model, replay, trial_series)
from detector import (CHANNELS, MODES, MR, SLOW_MODES, Calibration, DetectorConfig,
                      PerturbationDetector, cusum_run)


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
    ap.add_argument("--cusum_mode", default=d.cusum_mode, choices=list(SLOW_MODES),
                    help="residuo del detector lento: 1i un passo isolato, 1 un passo, R rollout")
    ap.add_argument("--cusum_ref", type=float, default=d.cusum_ref_s,
                    help="secondi di riferimento di sessione del detector lento (0 = fisso)")
    ap.add_argument("--cusum_iso", default=d.cusum_iso,
                    help="canali isolati in modo 1i, separati da virgola (es. sensor_diff,current)")
    ap.add_argument("--cusum_kappa", type=float, default=d.cusum_kappa)
    ap.add_argument("--cusum_clip", type=float, default=d.cusum_clip)
    ap.add_argument("--cusum_margin", type=float, default=None,
                    help="margine sulla soglia h del detector lento (default: --margin)")
    ap.add_argument("--compare_dir", default=None,
                    help="cartella con trial da CONFRONTARE coi normali (es. senza pinna): "
                         "non entrano nella calibrazione, si usano sigma e soglie appena calcolate")
    args = ap.parse_args()

    cfg = DetectorConfig(hz=args.hz, W_s=args.W, T_s=args.T, Tc_s=args.Tc, Tb_s=args.Tb,
                         quiet_s=args.quiet, min_reanchor_s=args.min_reanchor,
                         cusum_mode=args.cusum_mode, cusum_ref_s=args.cusum_ref,
                         cusum_iso=args.cusum_iso,
                         cusum_kappa=args.cusum_kappa, cusum_clip=args.cusum_clip)
    cmargin = args.cusum_margin if args.cusum_margin is not None else args.margin
    iso = cfg.cusum_mode == "1i"
    mC = None if iso else MODES.index(cfg.cusum_mode)
    iso_mask = np.array([c in [x.strip() for x in cfg.cusum_iso.split(",")] for c in CHANNELS])

    def slow_res(r, ok):
        if iso:   # isolato sui canali di cusum_iso, un passo normale sugli altri
            return np.where(iso_mask, r["resid_iso"][ok], r["resid"][ok][:, 0, :])
        return r["resid"][ok][:, mC, :]
    device = torch.device(args.device)
    FM, ckpt = load_model(args.checkpoint, device)
    dataset, val_trials = load_dataset(args.dataset_dir, args.scaler_path, ckpt)
    trials = list(range(len(dataset.trial_names))) if args.all_trials else val_trials

    det = PerturbationDetector(FM, Calibration.neutral(cfg), H=dataset.h, device=device)

    # ---- 1. residui e score grezzi (sigma=1) per trial ----
    names, sumsq, count, rawmax = [], [], [], []
    ages, res_R, res_C = [], [], []
    for k, ti in enumerate(trials, start=1):
        print(f"\r  trial {k}/{len(trials)}: {dataset.trial_names[ti]}        ",
              end="", flush=True)
        cmd, sens = trial_series(dataset, ti)
        r = replay(det, cmd, sens)
        ok = np.isfinite(r["resid"][:, 0, 0])
        if not np.isfinite(r["score"]).any():
            print(f"\n  salto {dataset.trial_names[ti]}: troppo corto per uno score")
            continue
        names.append(dataset.trial_names[ti])
        sumsq.append((r["resid"][ok] ** 2).sum(axis=0))
        count.append(int(ok.sum()))
        rawmax.append(np.nanmax(r["score"], axis=0))       # RMS_W grezzo massimo
        ages.append(r["age"][ok])
        res_R.append(r["resid"][ok][:, MR, :])
        res_C.append(slow_res(r, ok))                 # residuo del detector lento
    print()
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

    # ---- z^2 medio per trial: errore medio del trial rispetto a quello normale ----
    # 1 = errore tipico; quanto e' sparso tra i trial normali e' il limite di una
    # detection "lenta" (CUSUM) con riferimento fisso.
    meanz2 = (sumsq / count[:, None, None]) / sigma ** 2                   # (n,2,2)

    # ---- trial da confrontare (es. senza pinna) ----
    cmp_rows = []
    if args.compare_dir:
        if not os.path.exists(args.scaler_path):
            raise SystemExit(f"--compare_dir richiede uno scaler gia' esistente "
                             f"({args.scaler_path}): non va rifittato sui trial da confrontare.")
        cds = FishJointDataset(args.compare_dir, h=dataset.h, p=1, scaler_path=args.scaler_path)
        cds.prepare(list(range(len(cds.trial_names))))
        for ti, cname in enumerate(cds.trial_names):
            cmd, sens = trial_series(cds, ti)
            r = replay(det, cmd, sens)
            ok = np.isfinite(r["resid"][:, 0, 0])
            if not np.isfinite(r["score"]).any():
                print(f"  salto {cname}: troppo corto")
                continue
            mz = (r["resid"][ok] ** 2).mean(axis=0) / sigma ** 2
            alarm = np.nanmax(r["score"], axis=0) / thr_raw                 # >1: allarme veloce
            cmp_rows.append((cname, mz, alarm, slow_res(r, ok)))

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

    print("\nz^2 medio per trial (1 = errore tipico del modello). La dispersione tra i "
          "trial normali e' il limite di una detection lenta con riferimento fisso:")
    dz = pd.DataFrame([{"trial": names[i], **named(meanz2[i])} for i in range(n)])
    print(dz.round(3).to_string(index=False))
    summ = pd.DataFrame({"min": named(meanz2.min(axis=0)),
                         "mediana": named(np.median(meanz2, axis=0)),
                         "max": named(meanz2.max(axis=0))})
    print("\nRiepilogo z^2 medio sui trial normali:")
    print(summ.round(3).to_string())

    for cname, mz, alarm, _ in cmp_rows:
        print(f"\nConfronto: {cname} (non usato per la calibrazione)")
        rows = []
        for mi, m in enumerate(MODES):
            for ci, c in enumerate(CHANNELS):
                col = meanz2[:, mi, ci]
                rows.append({"score": f"S{m}_{c}", "z2_medio": mz[mi, ci],
                             "normali_max": col.max(),
                             "normali_sotto": f"{int((col < mz[mi, ci]).sum())}/{n}",
                             "max_score/soglia": alarm[mi, ci]})
        print(pd.DataFrame(rows).round(3).to_string(index=False))
        print("  normali_sotto = quanti trial normali hanno z^2 medio piu' basso "
              f"({n}/{n} = sopra tutti i normali); max_score/soglia > 1 = il detector "
              "attuale andrebbe in allarme")

    print("\nResiduo in rollout in funzione del tempo dall'ultimo riaggancio "
          "(1 = media; se cresce molto, sigma_R costante non basta):")
    print(pd.DataFrame(age_rows).round(3).to_string(index=False))

    # ---- detector lento (CUSUM): soglia h dai trial normali ----
    # Due varianti: riferimento di SESSIONE (primi cusum_ref s del trial, quella
    # usata online se cusum_ref > 0) e riferimento FISSO (ref = 1).
    nmin = cfg.n(5.0)                              # almeno 5 s dopo il riferimento
    variants = {"fisso": 0}
    if cfg.cusum_ref_s > 0:
        variants["sessione"] = cfg.n(cfg.cusum_ref_s)
    # normalizzatore proprio del residuo lento (per "1"/"R" coincide con sigma del modo)
    cusum_sigma = np.sqrt((np.concatenate(res_C) ** 2).mean(axis=0))
    cz = [(rc / cusum_sigma) ** 2 for rc in res_C]
    cusum = {}
    for vname, ref_n in variants.items():
        gmax, used = [], []
        for i, z2 in enumerate(cz):
            if len(z2) < ref_n + nmin:
                continue
            g, _ = cusum_run(z2, ref_n, cfg.cusum_kappa, cfg.cusum_clip)
            gmax.append(np.nanmax(g, axis=0))
            used.append(names[i])
        if len(gmax) < 2:
            print(f"\n[CUSUM {vname}] trial troppo corti per il riferimento di "
                  f"{ref_n / cfg.hz:.0f} s: soglia non calcolata")
            continue
        gmax = np.stack(gmax)
        h = cmargin * gmax.max(axis=0)
        loo = np.array([gmax[i] / (cmargin * np.delete(gmax, i, axis=0).max(axis=0))
                        for i in range(len(gmax))])
        cusum[vname] = {"h": h, "ref_n": ref_n, "gmax": gmax, "loo": loo, "trials": used}

    mode_txt = {"1i": f"un passo, isolato su [{cfg.cusum_iso}]", "1": "un passo",
                "R": "rollout"}[cfg.cusum_mode]
    print(f"\nDetector lento (CUSUM) | residuo {mode_txt} | sigma "
          f"{np.round(cusum_sigma, 4).tolist()} (un passo: {np.round(sigma[0], 4).tolist()}) | "
          f"kappa={cfg.cusum_kappa} clip={cfg.cusum_clip} margine={cmargin}")
    for vname, cu in cusum.items():
        dur = (np.mean([len(cz[names.index(t)]) for t in cu["trials"]]) - cu["ref_n"]) / cfg.hz
        h_txt = ", ".join("%s %.1f" % (c, cu["h"][ci]) for ci, c in enumerate(CHANNELS))
        print(f"  riferimento {vname} ({cu['ref_n'] / cfg.hz:.0f} s): h = {h_txt} | "
              f"{len(cu['trials'])} trial, ~{dur:.0f} s monitorati per trial | "
              f"LOO max {cu['loo'].max(axis=0).round(2).tolist()} (>1 = falso allarme)")
    print("  NB: h cresce con la durata monitorata. Calibrata su trial brevi e' "
          "ottimistica per l'uso online: servono trial normali lunghi.")

    for cname, _, _, rc in cmp_rows:
        z2 = (rc / cusum_sigma) ** 2
        print(f"\nDetector lento su {cname}:")
        for vname, cu in cusum.items():
            g, ref = cusum_run(z2, cu["ref_n"], cfg.cusum_kappa, cfg.cusum_clip)
            parts = []
            for ci, c in enumerate(CHANNELS):
                hit = np.flatnonzero(g[:, ci] > cu["h"][ci])
                when = (f"allarme a {(hit[0] + dataset.h) / cfg.hz:.1f} s dall'inizio"
                        if len(hit) else "nessun allarme")
                parts.append(f"{c}: {when} (g max {np.nanmax(g[:, ci]):.0f}, h {cu['h'][ci]:.0f})")
            print(f"  riferimento {vname}: " + " | ".join(parts))
        if "sessione" in cusum:
            print("  (con riferimento di sessione la perturbazione presente dall'inizio "
                  "entra nel riferimento: non e' un test valido per quella variante)")

    use = "sessione" if cfg.cusum_ref_s > 0 else "fisso"
    cusum_h = cusum[use]["h"] if use in cusum else np.full(2, np.inf)
    if use not in cusum:
        print(f"\n[ATTENZIONE] soglia CUSUM ({use}) non calcolata: detector lento "
              f"DISATTIVATO nel JSON (h = inf).")

    out = args.out or default_calib_path(args.checkpoint)
    Calibration(sigma, threshold, cfg, cusum_h=cusum_h, cusum_sigma=cusum_sigma, meta={
        "checkpoint": args.checkpoint, "scaler_path": args.scaler_path,
        "H": int(dataset.h), "trials": names,
        "split": "all" if args.all_trials else "val",
        "quantile": args.quantile, "margin": args.margin,
        "per_trial_max_score": {names[i]: named(rawmax[i] / sigma) for i in range(n)},
        "loo_ratio": {names[i]: named(ratio[i]) for i in range(n)},
        "loo_false_alarm_rate_per_trial": float(fa_alarm.mean()),
        "mean_z2_per_trial": {names[i]: named(meanz2[i]) for i in range(n)},
        "rollout_rms_vs_age": age_rows,
        "cusum": {v: {"h": {c: float(cu["h"][ci]) for ci, c in enumerate(CHANNELS)},
                      "ref_s": cu["ref_n"] / cfg.hz, "margin": cmargin,
                      "loo_max": {c: float(cu["loo"][:, ci].max()) for ci, c in enumerate(CHANNELS)}}
                  for v, cu in cusum.items()},
    }).save(out)
    print(f"\nCalibrazione salvata in {out}")


if __name__ == "__main__":
    main()