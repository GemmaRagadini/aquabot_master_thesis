"""
SYSTEMATIC EVALUATION of the detector with injected synthetic perturbations.

For every trial (default: the validation trials), every channel, every scenario
and every amplitude it runs a full replay through the detector and measures:
  fast detector     detected?  delay (alarm - perturbation start)
                    correct class (transient / persistent)
                    correct onset (abrupt for steps, gradual for ramps)
                    correct channel (the perturbed channel is classified)
                    only the correct channel (the other one is "none")
  slow detector     detected on the perturbed channel?  delay
                    correct origin (the first slow alarm is on the perturbed channel)
  false alarms      alarms before the start of the perturbation and on the clean trials

Scenarios (amplitudes in standard deviations of the signal, normalised space):
  step_1s    1 s step                        expected: transient, abrupt
  step_3s    3 s step                        expected: transient, abrupt
  step_pers  step until the end of the trial expected: persistent, abrupt
  ramp_pers  5 s ramp, then constant         expected: persistent, gradual
  gain_pers  signal * (1 + a) until the end  expected: persistent (onset not evaluated)
The start time is random in [t0_min, t0_max] and the same for all the
amplitudes of the same (trial, scenario, channel, repetition): the curves
versus amplitude are therefore comparable.

WARNING
- The thresholds are calibrated on the same validation trials: false alarms
  on the clean trials are zero by construction. The honest estimate of false
  alarms is the leave-one-trial-out of calibrate.py, which is printed here.
- An additive perturbation on the sensor is a surrogate of real ones.

Usage (from the aquabot folder):
  python3 src/net/fm/detection/evaluate.py --checkpoint src/net/fm/checkpoints_fm/best_fm_tuned.pt
  ... --workers 4 --reps 2
  ... --amps_step 0.5,1,2 --amps_ramp 1,2 --amps_gain 0.25,0.5    (quick test)

Outputs in --out (default detection_out/evaluate):
  evaluate_runs.csv      one row per replay
  evaluate_summary.csv   summary per channel / scenario / amplitude
  evaluate_summary.png   detection rate and delay as a function of the amplitude
"""
import argparse
import multiprocessing as mp
import os
import time
import zlib

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import torch

from det_common import (DEFAULT_DATASET, DEFAULT_SCALER, SCRIPT_DIR, default_calib_path,
                        load_dataset, load_model, replay, trial_series)
from detector import CHANNELS, M1, MR, Calibration, PerturbationDetector
from run_detector import apply_injections

COL_CH = {"sensor_diff": "#2a78d6", "current": "#d9822b"}

# name: (type, duration in s (0 = until the end of the trial), expected class, expected onset)
SCENARIOS = {
    "step_1s":   ("step", 1.0, "transient", "abrupt"),
    "step_3s":   ("step", 3.0, "transient", "abrupt"),
    "step_pers": ("step", 0.0, "persistent", "abrupt"),
    "ramp_pers": ("ramp", 5.0, "persistent", "gradual"),
    "gain_pers": ("gain", 0.0, "persistent", None),
}
SCEN_TITLE = {"step_1s": "Step, 1 s", "step_3s": "Step, 3 s", "step_pers": "Persistent step",
              "ramp_pers": "Persistent ramp (5 s)", "gain_pers": "Persistent gain"}

# global variables of the worker processes (inherited with fork)
G = {}


def floats(s):
    return [float(x) for x in s.split(",") if x.strip()]


def job_t0(seed, trial, rep, scen, ch, lo, hi):
    """Reproducible start time, the same for all the amplitudes."""
    key = zlib.crc32(f"{seed}|{trial}|{rep}|{scen}|{ch}".encode())
    return float(np.round(np.random.default_rng(key).uniform(lo, hi), 1))


def run_job(job):
    ti, name, scen, ch, amp, t0, rep = job
    calib, hz = G["calib"], G["calib"].config.hz
    cmd, sens = G["series"][ti]
    n = len(cmd)
    det = PerturbationDetector(G["FM"], calib, H=G["H"], device="cpu",
                               norm_stats=G["norm_stats"])

    if scen == "clean":
        r = replay(det, cmd, sens)
        ratio = r["score"][:, MR, :] / calib.threshold[MR]
        gh = r["g"] / calib.cusum_h
        return {"trial": name, "scenario": scen, "channel": "-", "amp": 0.0, "rep": rep,
                "t0": np.nan,
                "n_alarms": int(r["alarm"].sum()), "n_slow_alarms": len(det.slow_events),
                "max_SR_sd": float(np.nanmax(ratio[:, 0])), "max_SR_cur": float(np.nanmax(ratio[:, 1])),
                "max_g_sd": float(np.nanmax(gh[:, 0])), "max_g_cur": float(np.nanmax(gh[:, 1]))}

    kind, dur, exp_cls, exp_onset = SCENARIOS[scen]
    sens_p, _ = apply_injections(sens, [(ch, kind, t0, dur, amp)], hz)
    r = replay(det, cmd, sens_p)

    ci = CHANNELS.index(ch)
    other = CHANNELS[1 - ci]
    i0 = int(round(t0 * hz))
    persistent = exp_cls == "persistent"
    pert_end = n if (persistent or dur <= 0) else int(round((t0 + dur) * hz))
    # window in which an alarm counts as a detection: for transients up to
    # 1 s + W after the end of the perturbation
    win_end = n if persistent else min(n, pert_end + det.nW + int(hz))

    alarms = np.flatnonzero(r["alarm"])
    hit = alarms[(alarms >= i0) & (alarms < win_end)]
    row = {"trial": name, "scenario": scen, "channel": ch, "amp": amp, "rep": rep, "t0": t0,
           "fa_before": bool((alarms < i0).any()),
           "fa_after": int((alarms >= win_end).sum()),
           "n_alarms_window": int(len(hit)),
           "detected": bool(len(hit)), "delay_s": np.nan,
           "classified": False, "class": None, "class_ok": np.nan,
           "onset": None, "onset_ok": np.nan, "trigger": None,
           "channel_ok": np.nan, "channel_only": np.nan, "persistent_by": None}
    if len(hit):
        a = int(hit[0])
        row["delay_s"] = (a - i0) / hz
        ev = next((e for e in det.events if e["t_alarm"] == a), None)
        if ev is not None:
            chans = ev["channels"]
            row.update(classified=True, **{"class": ev["class"]},
                       class_ok=float(ev["class"] == exp_cls),
                       trigger="+".join(ev["trigger"]),
                       onset=chans[ch]["onset"],
                       persistent_by=chans[ch].get("persistent_by"),
                       channel_ok=float(chans[ch]["class"] != "none"),
                       channel_only=float(chans[ch]["class"] != "none"
                                          and chans[other]["class"] == "none"))
            if exp_onset is not None and chans[ch]["onset"] is not None:
                row["onset_ok"] = float(chans[ch]["onset"] == exp_onset)

    # ---- slow detector ----
    slow = sorted(det.slow_events, key=lambda e: (e["t_alarm"], e["secondary"]))
    row["fa_slow_before"] = any(e["t_alarm"] < i0 for e in slow)
    after = [e for e in slow if e["t_alarm"] >= i0]
    mine = [e for e in after if e["channel"] == ch]
    row["slow_detected"] = bool(mine)
    row["slow_delay_s"] = (mine[0]["t_alarm"] - i0) / hz if mine else np.nan
    row["slow_origin_ok"] = (float(after[0]["channel"] == ch and not after[0]["secondary"])
                             if after else np.nan)
    row["slow_other_channel"] = any(e["channel"] == other and not e["secondary"] for e in after)
    return row


def pct(s):
    s = pd.to_numeric(s, errors="coerce").dropna()
    return 100.0 * s.mean() if len(s) else np.nan


def summarize(df):
    rows = []
    for (ch, scen, amp), d in df.groupby(["channel", "scenario", "amp"], sort=False):
        det = d[d.detected]
        rows.append({
            "channel": ch, "scenario": scen, "amp": amp, "n": len(d),
            "detected_%": pct(d.detected),
            "delay_med_s": det.delay_s.median(),
            "class_ok_%": pct(det.class_ok),
            "onset_ok_%": pct(det.onset_ok),
            "channel_ok_%": pct(det.channel_ok),
            "channel_only_%": pct(det.channel_only),
            "extra_alarms": (d.n_alarms_window - 1).clip(lower=0).mean() + d.fa_after.mean(),
            "slow_detected_%": pct(d.slow_detected),
            "slow_delay_med_s": d.slow_delay_s.median(),
            "slow_origin_ok_%": pct(d.slow_origin_ok),
            "fa_before": int(d.fa_before.sum() + d.fa_slow_before.sum()),
        })
    return pd.DataFrame(rows)


def min_amp(summary, col, target):
    """Minimum amplitude from which on the rate stays >= target (NaN if never)."""
    out = {}
    for (ch, scen), d in summary.groupby(["channel", "scenario"], sort=False):
        d = d.sort_values("amp")
        ok = (d[col] >= target).to_numpy()
        tail = np.flatnonzero(~ok)
        if ok.all():
            out[(ch, scen)] = d.amp.iloc[0]
        elif tail[-1] + 1 < len(d):
            out[(ch, scen)] = d.amp.iloc[tail[-1] + 1]
        else:
            out[(ch, scen)] = np.nan
    return out


def plot(summary, path):
    scen = list(dict.fromkeys(summary.scenario))
    fig, axes = plt.subplots(2, len(scen), figsize=(3.6 * len(scen), 6.2), squeeze=False)
    for j, s in enumerate(scen):
        d = summary[summary.scenario == s]
        for ch in CHANNELS:
            e = d[d.channel == ch].sort_values("amp")
            if e.empty:
                continue
            c = COL_CH[ch]
            axes[0, j].plot(e.amp, e["detected_%"], "o-", color=c, label=f"{ch} fast")
            axes[0, j].plot(e.amp, e["slow_detected_%"], "s--", color=c, alpha=0.7, label=f"{ch} slow")
            axes[1, j].plot(e.amp, e["delay_med_s"], "o-", color=c)
            axes[1, j].plot(e.amp, e["slow_delay_med_s"], "s--", color=c, alpha=0.7)
        axes[0, j].set_title(SCEN_TITLE.get(s, s), fontsize=10)
        axes[0, j].set_ylim(-5, 105)
        axes[1, j].set_xlabel("amplitude (signal std)" if s != "gain_pers" else "gain a  (signal × (1 + a))")
        for ax in axes[:, j]:
            ax.grid(True, alpha=0.3)
    axes[0, 0].set_ylabel("detected (%)")
    axes[1, 0].set_ylabel("median delay (s)")
    axes[0, 0].legend(fontsize=7)
    fig.suptitle("Evaluation with injected perturbations — solid: fast detector, dashed: slow detector",
                 fontsize=10)
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--checkpoint", required=True)
    ap.add_argument("--calib", default=None)
    ap.add_argument("--dataset_dir", default=DEFAULT_DATASET)
    ap.add_argument("--scaler_path", default=DEFAULT_SCALER)
    ap.add_argument("--trials", default="val", help="'val' (default), 'all' or comma-separated names")
    ap.add_argument("--channels", default="sensor_diff,current")
    ap.add_argument("--scenarios", default=",".join(SCENARIOS))
    ap.add_argument("--amps_step", default="0.25,0.5,1,1.5,2,3")
    ap.add_argument("--amps_ramp", default="0.25,0.5,1,2")
    ap.add_argument("--amps_gain", default="0.1,0.25,0.5,1")
    ap.add_argument("--reps", type=int, default=1, help="random start times per trial")
    ap.add_argument("--t0_min", type=float, default=5.0)
    ap.add_argument("--t0_max", type=float, default=15.0)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--workers", type=int, default=max(1, (os.cpu_count() or 2) - 1))
    ap.add_argument("--no_persist_cusum", action="store_true",
                    help="persistent classification from S_R only (old rule, for comparison)")
    ap.add_argument("--out", default=os.path.join(SCRIPT_DIR, "detection_out", "evaluate"))
    args = ap.parse_args()

    FM, ckpt = load_model(args.checkpoint, torch.device("cpu"))
    calib = Calibration.load(args.calib or default_calib_path(args.checkpoint))
    if args.no_persist_cusum:
        calib.config.persist_cusum = False
    print(f"persist_cusum = {calib.config.persist_cusum}")
    dataset, val_trials = load_dataset(args.dataset_dir, args.scaler_path, ckpt)
    names = dataset.trial_names
    if args.trials == "val":
        tids = val_trials
    elif args.trials == "all":
        tids = list(range(len(names)))
    else:
        tids = [names.index(x.strip()) for x in args.trials.split(",")]
    hz = calib.config.hz
    G.update(FM=FM, calib=calib, H=dataset.h, norm_stats=ckpt.get("norm_stats"),
             series={ti: trial_series(dataset, ti) for ti in tids})

    short = min(len(G["series"][ti][0]) for ti in tids) / hz
    if args.t0_max + calib.config.T_s + 3 > short:
        print(f"[warning] trials are {short:.1f} s long: with t0_max={args.t0_max} some "
              f"perturbations may not be classified before the end.")

    amps = {"step": floats(args.amps_step), "ramp": floats(args.amps_ramp), "gain": floats(args.amps_gain)}
    chans = [c.strip() for c in args.channels.split(",")]
    scens = [s.strip() for s in args.scenarios.split(",")]
    for s in scens:
        if s not in SCENARIOS:
            raise SystemExit(f"unknown scenario: {s} (valid: {list(SCENARIOS)})")

    jobs = [(ti, names[ti], "clean", "-", 0.0, np.nan, 0) for ti in tids]
    for ti in tids:
        for rep in range(args.reps):
            for s in scens:
                for ch in chans:
                    t0 = job_t0(args.seed, names[ti], rep, s, ch, args.t0_min, args.t0_max)
                    for a in amps[SCENARIOS[s][0]]:
                        jobs.append((ti, names[ti], s, ch, a, t0, rep))

    print(f"{len(tids)} trials, {len(jobs)} replays, {args.workers} processes")
    ns = ckpt.get("norm_stats")
    if ns:
        print(f"Amplitude 1 = {ns['sd_std']:.1f} sensor units on sensor_diff, "
              f"{ns['vf_std']:.1f} mA on current")
    print(f"Amplitude 1 = {1 / calib.sigma[M1, 0]:.1f} sigma of the one-step residual on sensor_diff, "
          f"{1 / calib.sigma[M1, 1]:.1f} on current")

    t_start = time.time()
    rows = []
    if args.workers > 1:
        ctx = mp.get_context("fork")
        with ctx.Pool(args.workers) as pool:
            for k, row in enumerate(pool.imap_unordered(run_job, jobs, chunksize=4), 1):
                rows.append(row)
                el = time.time() - t_start
                print(f"\r  {k}/{len(jobs)}  elapsed {el / 60:.1f} min, "
                      f"remaining ~{el / k * (len(jobs) - k) / 60:.1f} min   ", end="", flush=True)
    else:
        for k, job in enumerate(jobs, 1):
            rows.append(run_job(job))
            el = time.time() - t_start
            print(f"\r  {k}/{len(jobs)}  elapsed {el / 60:.1f} min, "
                  f"remaining ~{el / k * (len(jobs) - k) / 60:.1f} min   ", end="", flush=True)
    print()

    os.makedirs(args.out, exist_ok=True)
    df = pd.DataFrame(rows)
    clean = df[df.scenario == "clean"]
    pert = df[df.scenario != "clean"].copy()
    order = {s: i for i, s in enumerate(scens)}
    pert["_o"] = pert.scenario.map(order)
    pert = pert.sort_values(["channel", "_o", "amp", "trial", "rep"]).drop(columns="_o")
    df_out = pd.concat([pert, clean], ignore_index=True)
    df_out.to_csv(os.path.join(args.out, "evaluate_runs.csv"), index=False)

    # ---- clean trials ----
    print("\n=== Clean trials (no injection) ===")
    print(f"  fast alarms: {int(clean.n_alarms.sum())} | slow alarms: "
          f"{int(clean.n_slow_alarms.sum())} over {len(clean)} trials")
    print(f"  max S_R/threshold: sensor_diff {clean.max_SR_sd.max():.2f}, current {clean.max_SR_cur.max():.2f}"
          f" | max g/h: sensor_diff {clean.max_g_sd.max():.2f}, current {clean.max_g_cur.max():.2f}")
    meta = calib.meta
    if "loo_false_alarm_rate_per_trial" in meta:
        print(f"  leave-one-trial-out on the CALIBRATION trials (from calibrate.py): fast false alarms in "
              f"{100 * meta['loo_false_alarm_rate_per_trial']:.0f}% of the trials")
    for v, cu in meta.get("cusum", {}).items():
        if "loo_max" in cu:
            lm = cu["loo_max"]
            print(f"  leave-one-trial-out CUSUM ({v}): max g/h sensor_diff {lm['sensor_diff']:.2f}, "
                  f"current {lm['current']:.2f} (>1 = slow false alarm)")

    # ---- summary ----
    summary = summarize(pert)
    summary.to_csv(os.path.join(args.out, "evaluate_summary.csv"), index=False)
    pd.set_option("display.width", 250)
    pd.set_option("display.max_columns", None)
    print("\n=== Injected perturbations ===")
    print("detected_%       fast alarm during the perturbation (transients: up to 1 s + W after)")
    print("class/onset/channel_ok_%   among the detected ones | channel_only_% = the other channel is 'none'")
    print("extra_alarms     additional fast alarms per replay (fragmentation or after the end)")
    print("slow_*           slow detector on the perturbed channel | origin_ok = first slow alarm on the right channel")
    print("fa_before        alarms (fast + slow) before the start of the perturbation\n")
    for ch in chans:
        print(f"--- {ch} ---")
        print(summary[summary.channel == ch].drop(columns="channel").round(2).to_string(index=False))
        print()

    for col, lab in [("detected_%", "fast"), ("slow_detected_%", "slow")]:
        m = min_amp(summary, col, 90.0)
        print(f"Minimum amplitude with {lab} detection >= 90%:")
        for (ch, s), a in m.items():
            print(f"  {ch:12s} {s:10s} {'never' if np.isnan(a) else f'{a:g}'}")
    plot(summary, os.path.join(args.out, "evaluate_summary.png"))
    print(f"\nSaved in {args.out}/: evaluate_runs.csv, evaluate_summary.csv, evaluate_summary.png")


if __name__ == "__main__":
    main()