"""
Data overview (Figure 4.3 of the thesis).

For one trial, plots the command c (with the centre beta of the oscillation),
the sensor difference s_sd and the motor current, one panel each.
Same style and time axis (t_rel_sec of the recording) as plot_prediction_fm.py.

Usage:
  python3 plot_data_overview.py --csv src/net/dataset/trial_15_DS.csv
  python3 plot_data_overview.py --csv src/net/dataset/trial_15_DS.csv --t_start 425 --t_end 440
"""
import argparse
import ast

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

# --- same palette as plot_prediction_fm.py ---
COL_SURFACE  = "#fcfcfb"
COL_TEXT     = "#0b0b0b"
COL_TEXT_SEC = "#52514e"
COL_MUTED    = "#898781"
COL_GRID     = "#e1e0d9"
COL_BASELINE = "#c3c2b7"
COL_SIGNAL   = "#0b0b0b"   # measured signals, as "real signal" in the prediction plots


def style(ax):
    ax.set_facecolor(COL_SURFACE)
    ax.grid(True, color=COL_GRID, linewidth=0.8, zorder=0)
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)
    for spine in ("left", "bottom"):
        ax.spines[spine].set_color(COL_BASELINE)
    ax.tick_params(colors=COL_MUTED, labelsize=8)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--csv", required=True)
    ap.add_argument("--t_start", type=float, default=None, help="start of the window (s, t_rel_sec)")
    ap.add_argument("--t_end", type=float, default=None, help="end of the window (s, t_rel_sec)")
    ap.add_argument("--out", default="data_overview.pdf")
    a = ap.parse_args()

    d = pd.read_csv(a.csv)
    t = d["t_rel_sec"].to_numpy()
    lo = a.t_start if a.t_start is not None else t[0]
    hi = a.t_end if a.t_end is not None else t[-1]
    m = (t >= lo) & (t <= hi)
    s = np.array([ast.literal_eval(str(v))[:2] for v in d["sensor_values"]], dtype=float)

    t = t[m]
    c = d["tail_target_rad"].to_numpy()[m]
    beta = d["center_rad"].to_numpy()[m]
    sd = (s[:, 0] - s[:, 1])[m]
    cur = d["present_current_ma"].to_numpy()[m]

    fig, ax = plt.subplots(3, 1, figsize=(11, 7.2), sharex=True)
    fig.patch.set_facecolor(COL_SURFACE)

    l_c, = ax[0].plot(t, c, color=COL_SIGNAL, linewidth=1.6, zorder=3)
    l_b, = ax[0].plot(t, beta, color=COL_MUTED, linewidth=1.4, linestyle="--", zorder=4)
    ax[0].legend([l_c, l_b], ["command", "centre of the oscillation"], loc="lower right",
                 bbox_to_anchor=(1.0, 0.98), ncol=2, frameon=False, fontsize=8,
                 labelcolor=COL_TEXT_SEC)
    ax[1].plot(t, sd, color=COL_SIGNAL, linewidth=1.6, zorder=3)
    ax[2].plot(t, cur, color=COL_SIGNAL, linewidth=1.6, zorder=3)

    titles = ["command", "sensor_diff", "current"]
    units = ["rad", "sensor units", "mA"]
    for k in range(3):
        style(ax[k])
        ax[k].set_title(titles[k], color=COL_TEXT, fontsize=12, fontweight="bold",
                        loc="left", pad=10)
        ax[k].set_ylabel(units[k], color=COL_TEXT_SEC, fontsize=9)
    ax[2].set_xlabel("time (s)", color=COL_TEXT_SEC, fontsize=9)
    fig.align_ylabels(ax)
    fig.tight_layout()
    fig.savefig(a.out, dpi=160, facecolor=COL_SURFACE, bbox_inches="tight")
    print(f"Saved {a.out}")


if __name__ == "__main__":
    main()