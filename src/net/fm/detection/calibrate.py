"""
CALIBRATION of the detector on NORMAL trials: sigma, thresholds, leave-one-trial-out.

Every trial is run through the SAME detector used online, with sigma=1 and
infinite thresholds: this way the rollout is re-anchored with the same cadence
it will have online (every min_reanchor_s) and the calibration residuals have
the same statistics as the online ones.

  1. one-step and rollout residuals for every trial
  2. sigma[mode, channel] = RMS of the residual over all trials
  3. for every trial: maximum of the score over time
  4. threshold = margin * quantile_q of the per-trial maxima (default q=1: the maximum)
  5. leave-one-trial-out: threshold recomputed without trial i, then we check
     by how much trial i exceeds it (ratio > 1 = false alarm)

By default it uses ONLY the validation trials: on the training trials the FM
has smaller residuals than in reality and the thresholds would be too tight.

Usage:
  python3 src/net/fm/detection/calibrate.py --checkpoint src/net/fm/checkpoints_fm/best_fm_tuned.pt
  python3 .../calibrate.py --checkpoint ... --margin 1.2 --W 0.5 --min_reanchor 20
Writes <checkpoint>_detector.json (or --out).
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
    """(2,2) [mode, channel] -> {'S1_sensor_diff': ..., 'SR_current': ...}"""
    return {f"S{m}_{c}": float(a[mi, ci])
            for mi, m in enumerate(MODES) for ci, c in enumerate(CHANNELS)}


def main():
    d = DetectorConfig()
    ap = argparse.ArgumentParser()
    ap.add_argument("--checkpoint", required=True)
    ap.add_argument("--dataset_dir", default=DEFAULT_DATASET)
    ap.add_argument("--scaler_path", default=DEFAULT_SCALER,
                    help="MUST be the same scaler used for training")
    ap.add_argument("--device", default="cpu")
    ap.add_argument("--out", default=None, help="calibration JSON")
    ap.add_argument("--all_trials", action="store_true",
                    help="also use the training trials (not recommended: optimistic thresholds)")
    ap.add_argument("--quantile", type=float, default=1.0,
                    help="quantile of the per-trial maxima used as threshold (1.0 = maximum)")
    ap.add_argument("--margin", type=float, default=1.0,
                    help="multiplicative factor on the threshold (to raise it)")
    ap.add_argument("--hz", type=float, default=d.hz)
    ap.add_argument("--W", type=float, default=d.W_s, help="score window (s)")
    ap.add_argument("--T", type=float, default=d.T_s)
    ap.add_argument("--Tc", type=float, default=d.Tc_s)
    ap.add_argument("--Tb", type=float, default=d.Tb_s)
    ap.add_argument("--quiet", type=float, default=d.quiet_s)
    ap.add_argument("--min_reanchor", type=float, default=d.min_reanchor_s)
    ap.add_argument("--cusum_mode", default=d.cusum_mode, choices=list(SLOW_MODES),
                    help="slow detector residual: 1i isolated one-step, 1 one-step, R rollout")
    ap.add_argument("--cusum_ref", type=float, default=d.cusum_ref_s,
                    help="seconds of session reference for the slow detector (0 = fixed)")
    ap.add_argument("--cusum_iso", default=d.cusum_iso,
                    help="channels isolated in mode 1i, comma separated (e.g. sensor_diff,current)")
    ap.add_argument("--cusum_kappa", type=float, default=d.cusum_kappa)
    ap.add_argument("--cusum_clip", type=float, default=d.cusum_clip)
    ap.add_argument("--cusum_margin", type=float, default=None,
                    help="margin on the slow detector threshold h (default: --margin)")
    ap.add_argument("--compare_dir", default=None,
                    help="folder with trials to COMPARE with the normal ones (e.g. without fin): "
                         "they are not used for calibration, the sigma and thresholds just "
                         "computed are applied to them")
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
        if iso:   # isolated on the cusum_iso channels, normal one-step on the others
            return np.where(iso_mask, r["resid_iso"][ok], r["resid"][ok][:, 0, :])
        return r["resid"][ok][:, mC, :]
    device = torch.device(args.device)
    FM, ckpt = load_model(args.checkpoint, device)
    dataset, val_trials = load_dataset(args.dataset_dir, args.scaler_path, ckpt)
    trials = list(range(len(dataset.trial_names))) if args.all_trials else val_trials

    det = PerturbationDetector(FM, Calibration.neutral(cfg), H=dataset.h, device=device)

    # ---- 1. raw residuals and scores (sigma=1) per trial ----
    names, sumsq, count, rawmax = [], [], [], []
    ages, res_R, res_C = [], [], []
    for k, ti in enumerate(trials, start=1):
        print(f"\r  trial {k}/{len(trials)}: {dataset.trial_names[ti]}        ",
              end="", flush=True)
        cmd, sens = trial_series(dataset, ti)
        r = replay(det, cmd, sens)
        ok = np.isfinite(r["resid"][:, 0, 0])
        if not np.isfinite(r["score"]).any():
            print(f"\n  skipping {dataset.trial_names[ti]}: too short for a score")
            continue
        names.append(dataset.trial_names[ti])
        sumsq.append((r["resid"][ok] ** 2).sum(axis=0))
        count.append(int(ok.sum()))
        rawmax.append(np.nanmax(r["score"], axis=0))       # maximum raw RMS_W
        ages.append(r["age"][ok])
        res_R.append(r["resid"][ok][:, MR, :])
        res_C.append(slow_res(r, ok))                 # slow detector residual
    print()
    n = len(names)
    if n < 2:
        raise SystemExit("At least 2 trials are needed for calibration.")
    sumsq, count, rawmax = np.stack(sumsq), np.array(count), np.stack(rawmax)

    # ---- 2-4. sigma and thresholds ----
    sigma = np.sqrt(sumsq.sum(axis=0) / count.sum())                     # (2,2)
    thr_raw = args.margin * np.quantile(rawmax, args.quantile, axis=0)   # (2,2)
    threshold = thr_raw / sigma

    # ---- 5. leave-one-trial-out ----
    # The score is RMS_W(e)/sigma and the threshold is a quantile of the same
    # maxima divided by the same sigma: sigma cancels, the comparison is on raw values.
    ratio = np.zeros_like(rawmax)
    for i in range(n):
        others = np.delete(rawmax, i, axis=0)
        ratio[i] = rawmax[i] / (args.margin * np.quantile(others, args.quantile, axis=0))
    fa = ratio > 1.0
    fa_alarm = fa[:, MR, :].any(axis=1)        # an alarm = S_R of at least one channel

    # ---- mean z^2 per trial: mean error of the trial with respect to the normal one ----
    # 1 = typical error; how spread it is across normal trials is the limit of a
    # "slow" detection (CUSUM) with a fixed reference.
    meanz2 = (sumsq / count[:, None, None]) / sigma ** 2                   # (n,2,2)

    # ---- trials to compare (e.g. without fin) ----
    cmp_rows = []
    if args.compare_dir:
        if not os.path.exists(args.scaler_path):
            raise SystemExit(f"--compare_dir needs an existing scaler "
                             f"({args.scaler_path}): it must not be refitted on the compared trials.")
        cds = FishJointDataset(args.compare_dir, h=dataset.h, p=1, scaler_path=args.scaler_path)
        cds.prepare(list(range(len(cds.trial_names))))
        for ti, cname in enumerate(cds.trial_names):
            cmd, sens = trial_series(cds, ti)
            r = replay(det, cmd, sens)
            ok = np.isfinite(r["resid"][:, 0, 0])
            if not np.isfinite(r["score"]).any():
                print(f"  skipping {cname}: too short")
                continue
            mz = (r["resid"][ok] ** 2).mean(axis=0) / sigma ** 2
            alarm = np.nanmax(r["score"], axis=0) / thr_raw                 # >1: fast alarm
            cmp_rows.append((cname, mz, alarm, slow_res(r, ok)))

    # ---- growth of the rollout residual with the time since the last re-anchor ----
    ages, res_R = np.concatenate(ages), np.concatenate(res_R)
    blk = cfg.n(1.0)
    age_rows = []
    for b in range(int(np.ceil(ages.max() / blk))):
        sel = (ages > b * blk) & (ages <= (b + 1) * blk)
        if sel.sum() < 5:
            continue
        rel = np.sqrt((res_R[sel] ** 2).mean(axis=0)) / sigma[MR]
        age_rows.append({"age_s": f"{b}-{b + 1}", "n": int(sel.sum()),
                         **{f"rmsR/sigma_{c}": float(rel[ci]) for ci, c in enumerate(CHANNELS)}})

    # ---- print ----
    pd.set_option("display.width", 220)
    print(f"\nCalibration on {n} trials ({'all' if args.all_trials else 'validation only'}) | "
          f"W={cfg.W_s}s re-anchor every {cfg.min_reanchor_s}s | "
          f"quantile={args.quantile} margin={args.margin}")
    print("\nsigma (normalised units) and thresholds (in multiples of sigma):")
    print(pd.DataFrame({"sigma": named(sigma), "threshold": named(threshold)}).round(4).to_string())

    df = pd.DataFrame([{"trial": names[i], **named(ratio[i])} for i in range(n)])
    print("\nLeave-one-trial-out: maximum score of the excluded trial / threshold recomputed "
          "without it (>1 = false alarm)")
    print(df.round(3).to_string(index=False))
    print("\nFraction of trials above threshold, per score:")
    print(pd.Series(named(fa.mean(axis=0))).round(3).to_string())
    print(f"False alarm rate per trial (S_R of at least one channel): "
          f"{fa_alarm.mean():.3f}  ({int(fa_alarm.sum())}/{n})")
    if args.quantile >= 1.0:
        print("NB: with threshold = maximum, for every score the trial with the absolute\n"
              "    maximum ALWAYS exceeds the threshold recomputed without it. The useful\n"
              "    number is BY HOW MUCH: ratios just above 1 = stable threshold; ratios\n"
              "    well above 1 = heavy tails, more trials or a --margin are needed.")

    print("\nMean z^2 per trial (1 = typical model error). The spread across normal "
          "trials is the limit of a slow detection with a fixed reference:")
    dz = pd.DataFrame([{"trial": names[i], **named(meanz2[i])} for i in range(n)])
    print(dz.round(3).to_string(index=False))
    summ = pd.DataFrame({"min": named(meanz2.min(axis=0)),
                         "median": named(np.median(meanz2, axis=0)),
                         "max": named(meanz2.max(axis=0))})
    print("\nSummary of mean z^2 over the normal trials:")
    print(summ.round(3).to_string())

    for cname, mz, alarm, _ in cmp_rows:
        print(f"\nComparison: {cname} (not used for calibration)")
        rows = []
        for mi, m in enumerate(MODES):
            for ci, c in enumerate(CHANNELS):
                col = meanz2[:, mi, ci]
                rows.append({"score": f"S{m}_{c}", "mean_z2": mz[mi, ci],
                             "normal_max": col.max(),
                             "normal_below": f"{int((col < mz[mi, ci]).sum())}/{n}",
                             "max_score/threshold": alarm[mi, ci]})
        print(pd.DataFrame(rows).round(3).to_string(index=False))
        print("  normal_below = how many normal trials have a lower mean z^2 "
              f"({n}/{n} = above all normal ones); max_score/threshold > 1 = the current "
              "detector would alarm")

    print("\nRollout residual as a function of the time since the last re-anchor "
          "(1 = mean; if it grows a lot, a constant sigma_R is not enough):")
    print(pd.DataFrame(age_rows).round(3).to_string(index=False))

    # ---- slow detector (CUSUM): threshold h from the normal trials ----
    # Two variants: SESSION reference (first cusum_ref s of the trial, the one used
    # online if cusum_ref > 0) and FIXED reference (ref = 1).
    nmin = cfg.n(5.0)                              # at least 5 s after the reference
    variants = {"fixed": 0}
    if cfg.cusum_ref_s > 0:
        variants["session"] = cfg.n(cfg.cusum_ref_s)
    # own normaliser of the slow residual (for "1"/"R" it equals the sigma of the mode)
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
            print(f"\n[CUSUM {vname}] trials too short for a reference of "
                  f"{ref_n / cfg.hz:.0f} s: threshold not computed")
            continue
        gmax = np.stack(gmax)
        h = cmargin * gmax.max(axis=0)
        loo = np.array([gmax[i] / (cmargin * np.delete(gmax, i, axis=0).max(axis=0))
                        for i in range(len(gmax))])
        cusum[vname] = {"h": h, "ref_n": ref_n, "gmax": gmax, "loo": loo, "trials": used}

    mode_txt = {"1i": f"one-step, isolated on [{cfg.cusum_iso}]", "1": "one-step",
                "R": "rollout"}[cfg.cusum_mode]
    print(f"\nSlow detector (CUSUM) | residual {mode_txt} | sigma "
          f"{np.round(cusum_sigma, 4).tolist()} (one-step: {np.round(sigma[0], 4).tolist()}) | "
          f"kappa={cfg.cusum_kappa} clip={cfg.cusum_clip} margin={cmargin}")
    for vname, cu in cusum.items():
        dur = (np.mean([len(cz[names.index(t)]) for t in cu["trials"]]) - cu["ref_n"]) / cfg.hz
        h_txt = ", ".join("%s %.1f" % (c, cu["h"][ci]) for ci, c in enumerate(CHANNELS))
        print(f"  {vname} reference ({cu['ref_n'] / cfg.hz:.0f} s): h = {h_txt} | "
              f"{len(cu['trials'])} trials, ~{dur:.0f} s monitored per trial | "
              f"LOO max {cu['loo'].max(axis=0).round(2).tolist()} (>1 = false alarm)")
    print("  NB: h grows with the monitored duration. Calibrated on short trials it is "
          "optimistic for online use: long normal trials are needed.")

    for cname, _, _, rc in cmp_rows:
        z2 = (rc / cusum_sigma) ** 2
        print(f"\nSlow detector on {cname}:")
        for vname, cu in cusum.items():
            g, ref = cusum_run(z2, cu["ref_n"], cfg.cusum_kappa, cfg.cusum_clip)
            parts = []
            for ci, c in enumerate(CHANNELS):
                hit = np.flatnonzero(g[:, ci] > cu["h"][ci])
                when = (f"alarm at {(hit[0] + dataset.h) / cfg.hz:.1f} s from the start"
                        if len(hit) else "no alarm")
                parts.append(f"{c}: {when} (g max {np.nanmax(g[:, ci]):.0f}, h {cu['h'][ci]:.0f})")
            print(f"  {vname} reference: " + " | ".join(parts))
        if "session" in cusum:
            print("  (with a session reference, a perturbation present from the start "
                  "becomes part of the reference: not a valid test for that variant)")

    use = "session" if cfg.cusum_ref_s > 0 else "fixed"
    cusum_h = cusum[use]["h"] if use in cusum else np.full(2, np.inf)
    if use not in cusum:
        print(f"\n[WARNING] CUSUM threshold ({use}) not computed: slow detector "
              f"DISABLED in the JSON (h = inf).")

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
    print(f"\nCalibration saved to {out}")


if __name__ == "__main__":
    main()