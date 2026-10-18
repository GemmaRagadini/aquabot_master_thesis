"""
REPLAY of a trial through the online detector, sample by sample, with figure
and event summary. Optionally injects SYNTHETIC perturbations on the sensors
to test the alarm -> classification chain.

--inject channel:type:t_start:duration:amplitude   (repeatable)
  channel    sensor_diff | current
  type       step  constant offset for `duration` seconds (duration 0 = until the end)
             ramp  grows linearly up to `amplitude` in `duration` s, then stays
             gain  multiplies the signal by (1 + amplitude) for `duration` seconds
  amplitude  in standard deviations of the signal (normalised space)
  e.g. --inject sensor_diff:step:10:1:2      1 s abrupt transient
       --inject current:gain:15:0:0.5        amplitude +50% from 15 s to the end
                                             (test of the slow detector)
       --inject current:ramp:8:6:1.5         gradual persistent

WARNING: an additive perturbation on the sensor is a surrogate. Real ones
change the dynamics; these serve to verify the logic, not the sensitivity.

Usage:
  python3 src/net/fm/detection/run_detector.py --checkpoint .../best_fm_tuned.pt
  python3 .../run_detector.py --checkpoint ... --trial trial_106.csv --inject sensor_diff:step:10:1:2
"""
import argparse
import json
import os
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import torch

from det_common import (CH_TO_KEY, CH_UNIT, DEFAULT_DATASET, DEFAULT_SCALER, SCRIPT_DIR,
                        default_calib_path, load_dataset, load_model, replay,
                        resolve_trial, trial_series)
from detector import (ALARM, CHANNELS, M1, MR, PERSISTENT, Calibration,
                      PerturbationDetector)

COL_SURFACE, COL_TEXT, COL_TEXT_SEC = "#fcfcfb", "#0b0b0b", "#52514e"
COL_MUTED, COL_GRID, COL_BASELINE   = "#898781", "#e1e0d9", "#c3c2b7"
COL_SIGNAL, COL_ONE, COL_ROLL       = "#0b0b0b", "#2a78d6", "#d9822b"
COL_CH        = {"sensor_diff": "#2a78d6", "current": "#d9822b"}
COL_STATE     = {ALARM: "#f2c14e", PERSISTENT: "#c1666b"}


def parse_inject(spec):
    try:
        ch, kind, t0, dur, amp = spec.split(":")
        t0, dur, amp = float(t0), float(dur), float(amp)
    except ValueError:
        raise SystemExit(f"--inject '{spec}': format channel:type:t_start:duration:amplitude")
    if ch not in CHANNELS or kind not in ("step", "ramp", "gain"):
        raise SystemExit(f"--inject '{spec}': invalid channel or type")
    if kind == "ramp" and dur <= 0:
        raise SystemExit(f"--inject '{spec}': ramp needs duration > 0")
    return ch, kind, t0, dur, amp


def apply_injections(sens, injections, hz):
    """Returns (perturbed sens, list of intervals (t0, t1) in seconds)."""
    sens = sens.copy()
    t = np.arange(len(sens)) / hz
    spans = []
    for ch, kind, t0, dur, amp in injections:
        ci = CHANNELS.index(ch)
        t1 = t[-1] + 1.0 / hz if dur <= 0 else t0 + dur
        inside = (t >= t0) & (t < t1)
        if kind == "step":
            sens[inside, ci] += amp
            spans.append((t0, t1))
        elif kind == "gain":
            sens[inside, ci] *= 1.0 + amp
            spans.append((t0, t1))
        else:  # ramp: rises in [t0, t1) and then stays
            sens[:, ci] += amp * np.clip((t - t0) / dur, 0.0, 1.0)
            spans.append((t0, float(t[-1])))
    return sens, spans


def style_axis(ax):
    ax.set_facecolor(COL_SURFACE)
    ax.grid(True, color=COL_GRID, linewidth=0.8, zorder=0)
    for sp in ("top", "right"):
        ax.spines[sp].set_visible(False)
    for sp in ("left", "bottom"):
        ax.spines[sp].set_color(COL_BASELINE)
    ax.tick_params(colors=COL_MUTED, labelsize=8)


def shade(ax, t, r, spans, hz):
    """Background: ALARM/PERSISTENT states, injected intervals, re-anchors."""
    for st, col in COL_STATE.items():
        on = np.concatenate([[False], r["state"] == st, [False]])
        edges = np.flatnonzero(np.diff(on.astype(int)))
        for a, b in zip(edges[::2], edges[1::2]):
            ax.axvspan(t[a], t[b - 1] + 1.0 / hz, color=col, alpha=0.22, linewidth=0, zorder=1)
    for t0, t1 in spans:
        ax.axvspan(t0, t1, facecolor="none", edgecolor=COL_MUTED, hatch="///",
                   linewidth=0, alpha=0.5, zorder=1)
    for tr in t[r["reanchored"]]:
        ax.axvline(tr, color=COL_MUTED, linewidth=0.9, linestyle=":", zorder=2)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--checkpoint", required=True)
    ap.add_argument("--calib", default=None, help="JSON from calibrate.py")
    ap.add_argument("--dataset_dir", default=DEFAULT_DATASET)
    ap.add_argument("--scaler_path", default=DEFAULT_SCALER)
    ap.add_argument("--device", default="cpu")
    ap.add_argument("--trial", default=None, help="name/substring/index. Default: first validation trial.")
    ap.add_argument("--inject", action="append", default=[],
                    help="channel:type:t_start:duration:amplitude (repeatable)")
    ap.add_argument("--out", default=os.path.join(SCRIPT_DIR, "detection_out"))
    args = ap.parse_args()

    device = torch.device(args.device)
    FM, ckpt = load_model(args.checkpoint, device)
    calib = Calibration.load(args.calib or default_calib_path(args.checkpoint))
    dataset, val_trials = load_dataset(args.dataset_dir, args.scaler_path, ckpt)
    hz = calib.config.hz

    ti = resolve_trial(dataset, args.trial, val_trials)
    name = dataset.trial_names[ti]
    split = "val" if ti in val_trials else "train"
    if name in calib.meta.get("trials", []):
        print(f"[note] {name} was used to calibrate the thresholds.")

    cmd, sens_clean = trial_series(dataset, ti)
    sens, spans = apply_injections(sens_clean, [parse_inject(s) for s in args.inject], hz)

    det = PerturbationDetector(FM, calib, H=dataset.h, device=device,
                               norm_stats=ckpt.get("norm_stats"))
    r = replay(det, cmd, sens)
    t = np.arange(len(cmd)) / hz

    # ---- summary ----
    print(f"\nTrial {name} [{split}] | {len(cmd)} samples ({t[-1]:.1f} s) | "
          f"{len(spans)} injected perturbations")
    print(f"Rollout re-anchors at t = {[round(float(x), 1) for x in t[r['reanchored']]]} s")
    if not det.events and det.state != ALARM:
        print("No alarm.")
    for ev in det.events:
        print(f"ALARM at {ev['t_alarm_s']:.1f} s (channels: {', '.join(ev['trigger'])}) "
              f"-> {ev['class'].upper()} decided at {ev['t_decision_s']:.1f} s")
        if "t_end_s" in ev:
            print(f"    ended at {ev['t_end_s']:.1f} s (duration {ev['duration_s']:.1f} s), "
                  f"automatic return to normal")
        elif ev["class"] == "persistent":
            print("    still in progress at the end of the trial")
        for c, v in ev["channels"].items():
            if v["class"] != "none":
                by = f", persistent by {v['persistent_by']}" if v.get("persistent_by") else ""
                if v["SR_rise"] is None:
                    print(f"    {c:12s} {v['class']:12s} onset {v['onset']} "
                          f"(alarm at startup: S_R history too short to measure the rise{by})")
                else:
                    print(f"    {c:12s} {v['class']:12s} onset {v['onset']} "
                          f"(S_R rise: {v['SR_rise']:.2f} thresholds{by})")
    if det.state == ALARM:
        print("Trial ended with an alarm not yet classified.")
    print(f"Final state: {det.state}")

    # ---- slow detector (CUSUM) ----
    if not np.isfinite(det.h).any():
        print("Slow detector: disabled (no CUSUM threshold in the JSON: recalibrate).")
    elif det.ref is None:
        print("Slow detector: session reference not completed (trial too short).")
    else:
        print(f"Slow detector: reference {np.round(det.ref, 3).tolist()} "
              f"({'fixed' if calib.config.cusum_ref_s <= 0 else 'first %.0f s' % calib.config.cusum_ref_s}), "
              f"thresholds h {np.round(det.h, 1).tolist()}")
        if not det.slow_events:
            print("  no slow alarm")
        for ev in det.slow_events:
            end = (f"ended at {ev['t_end_s']:.1f} s (duration {ev['duration_s']:.1f} s)"
                   if "t_end_s" in ev else "still active at the end of the trial")
            kind = (f"SECONDARY, probable effect of {ev['origin']}"
                    if ev.get("secondary") else "origin")
            print(f"  SLOW ALARM on {ev['channel']} at {ev['t_alarm_s']:.1f} s, {end} "
                  f"({kind})")

    # ---- figure ----
    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)
    fig, axes = plt.subplots(5, 1, figsize=(11, 12.5), sharex=True)
    fig.patch.set_facecolor(COL_SURFACE)

    for ci, ch in enumerate(CHANNELS):
        ax = axes[ci]
        style_axis(ax)
        shade(ax, t, r, spans, hz)
        sc = dataset.scalers[CH_TO_KEY[ch]]
        real = lambda a: a * float(sc.scale_[0]) + float(sc.mean_[0])
        ax.plot(t, real(r["pred"][:, MR, ci]), color=COL_ROLL, linewidth=1.3, zorder=3,
                label="rollout")
        ax.plot(t, real(r["pred"][:, M1, ci]), color=COL_ONE, linewidth=1.3, zorder=4,
                label="one step")
        ax.plot(t, real(sens[:, ci]), color=COL_SIGNAL, linewidth=1.3, zorder=5,
                label="measured")
        ax.set_ylabel(CH_UNIT[ch], color=COL_TEXT_SEC, fontsize=9)
        ax.set_title(ch, color=COL_TEXT, fontsize=11, fontweight="bold", loc="left")
        ax.legend(loc="upper right", ncol=3, frameon=False, fontsize=8,
                  labelcolor=COL_TEXT_SEC)

    for ax, mi, lab in ((axes[2], MR, "S_R"), (axes[3], M1, "S_1")):
        style_axis(ax)
        shade(ax, t, r, spans, hz)
        for ci, ch in enumerate(CHANNELS):
            ax.plot(t, r["score"][:, mi, ci] / calib.threshold[mi, ci],
                    color=COL_CH[ch], linewidth=1.5, zorder=3, label=ch)
        ax.axhline(1.0, color=COL_TEXT, linewidth=1.0, linestyle="--", zorder=2)
        ax.set_ylim(bottom=0)
        ax.set_ylabel(f"{lab} / threshold", color=COL_TEXT_SEC, fontsize=9)
        ax.set_title(f"{lab} — {'rollout' if mi == MR else 'one-step'} score "
                     f"(dashed line = threshold)",
                     color=COL_TEXT, fontsize=11, fontweight="bold", loc="left")
        ax.legend(loc="upper right", ncol=2, frameon=False, fontsize=8,
                  labelcolor=COL_TEXT_SEC)
    ax = axes[4]
    style_axis(ax)
    shade(ax, t, r, spans, hz)
    for ci, ch in enumerate(CHANNELS):
        on = np.concatenate([[False], r["slow"][:, ci], [False]])
        edges = np.flatnonzero(np.diff(on.astype(int)))
        for a, b in zip(edges[::2], edges[1::2]):
            ax.axvspan(t[a], t[b - 1] + 1.0 / hz, color=COL_CH[ch], alpha=0.12,
                       linewidth=0, zorder=1)
        hh = det.h[ci]
        y = r["g"][:, ci] / hh if np.isfinite(hh) else r["g"][:, ci]
        ax.plot(t, y, color=COL_CH[ch], linewidth=1.5, zorder=3, label=ch)
    if np.isfinite(det.h).any():
        ax.axhline(1.0, color=COL_TEXT, linewidth=1.0, linestyle="--", zorder=2)
        ylab = "g / h"
    else:
        ylab = "g (no threshold)"
    ax.set_ylim(bottom=0)
    ax.set_ylabel(ylab, color=COL_TEXT_SEC, fontsize=9)
    ref_txt = (f"{calib.config.cusum_ref_s:.0f} s session reference"
               if calib.config.cusum_ref_s > 0 else "fixed reference")
    ax.set_title(f"Slow detector (CUSUM, {calib.config.cusum_mode}-mode residual, "
                 f"{ref_txt}) — shaded: slow alarm active",
                 color=COL_TEXT, fontsize=11, fontweight="bold", loc="left")
    ax.legend(loc="upper right", ncol=2, frameon=False, fontsize=8, labelcolor=COL_TEXT_SEC)
    axes[4].set_xlabel("time (s)", color=COL_TEXT_SEC, fontsize=9)

    fig.suptitle(f"Perturbation detector — trial {name} [{split}]   "
                 f"yellow: alarm · red: persistent · hatched: injected · dotted: rollout re-anchor",
                 color=COL_TEXT, fontsize=10, fontweight="bold", x=0.01, ha="left", y=0.995)
    fig.tight_layout(rect=[0, 0, 1, 0.97])
    stem = Path(name).stem + ("_inj" if spans else "")
    fig_path = out_dir / f"detector_{stem}.png"
    fig.savefig(fig_path, dpi=160, facecolor=COL_SURFACE)
    plt.close(fig)
    print(f"Saved {fig_path}")

    ev_path = out_dir / f"detector_{stem}_events.json"
    with open(ev_path, "w") as f:
        json.dump({"trial": name, "split": split, "inject": args.inject,
                   "reanchor_s": [float(x) for x in t[r["reanchored"]]],
                   "slow_events": det.slow_events,
                   "final_state": det.state, "events": det.events}, f, indent=2)
    print(f"Saved {ev_path}")


if __name__ == "__main__":
    main()