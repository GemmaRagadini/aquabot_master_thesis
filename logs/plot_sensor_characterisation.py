"""
Characterisation of the flex sensors (figure of Section 3.3).

Over all trials of the normal dataset:
  (a) raw readings s1 and s2 vs measured servo position
  (b) sensor difference s_sd vs measured servo position
Same style as plot_prediction_fm.py. Scatter points are rasterized, so the
PDF stays small even with the whole dataset.

Usage:
  python3 plot_sensor_characterisation.py --csv "src/net/dataset/trial_*_DS.csv" --out sensor_characterisation.pdf
"""
import argparse
import ast
import glob

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
COL_SIGNAL   = "#0b0b0b"
# two colours not used elsewhere for the two sensors
COL_S1 = "#1b9e77"
COL_S2 = "#7570b3"


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
    ap.add_argument("--csv", nargs="+", required=True)
    ap.add_argument("--out", default="sensor_characterisation.pdf")
    ap.add_argument("--max_points", type=int, default=60000,
                    help="random subsample for plotting (all points used for the printed ranges)")
    ap.add_argument("--alpha_scale", type=float, default=2.5,
                    help="opacity of the points (1 = faint, 2.5 = default, 4 = strong)")
    a = ap.parse_args()
    files = sorted(set(f for p in a.csv for f in glob.glob(p)))

    S, pos = [], []
    for f in files:
        d = pd.read_csv(f)
        S.append(np.array([ast.literal_eval(str(v))[:2] for v in d["sensor_values"]], dtype=float))
        pos.append(d["real_position_rad"].to_numpy())
    S, pos = np.vstack(S), np.concatenate(pos)
    sd = S[:, 0] - S[:, 1]
    print(f"{len(files)} files, {len(sd)} samples")
    for name, x in (("s1", S[:, 0]), ("s2", S[:, 1]), ("s_sd", sd)):
        print(f"  {name:5s} [{x.min():.1f}, {x.max():.1f}]")
    print(f"  position [{pos.min():.3f}, {pos.max():.3f}] rad")

    idx = np.arange(len(sd))
    if len(idx) > a.max_points:
        idx = np.random.default_rng(0).choice(idx, a.max_points, replace=False)

    fig, ax = plt.subplots(1, 2, figsize=(11, 4.0))
    fig.patch.set_facecolor(COL_SURFACE)
    # fainter points when there are many; raise --alpha_scale for stronger colours
    alpha = float(np.clip(a.alpha_scale * 4000 / len(idx), 0.05, 0.9))
    kw = dict(s=2.5, alpha=alpha, linewidths=0, rasterized=True, zorder=2)
    ax[0].scatter(pos[idx], S[idx, 0], color=COL_S1, **kw)
    ax[0].scatter(pos[idx], S[idx, 1], color=COL_S2, **kw)
    h1 = ax[0].scatter([], [], color=COL_S1, s=12)
    h2 = ax[0].scatter([], [], color=COL_S2, s=12)
    ax[0].legend([h1, h2], [r"$s_1$", r"$s_2$"], loc="lower center", ncol=2,
                 frameon=False, fontsize=9, labelcolor=COL_TEXT_SEC)
    ax[1].scatter(pos[idx], sd[idx], color=COL_SIGNAL, **kw)

    titles = ["(a) raw readings", "(b) sensor_diff"]
    for k in range(2):
        style(ax[k])
        ax[k].set_title(titles[k], color=COL_TEXT, fontsize=12, fontweight="bold",
                        loc="left", pad=10)
        ax[k].set_xlabel("measured position (rad)", color=COL_TEXT_SEC, fontsize=9)
        ax[k].set_ylabel("sensor units", color=COL_TEXT_SEC, fontsize=9)
    fig.tight_layout()
    fig.savefig(a.out, dpi=200, facecolor=COL_SURFACE, bbox_inches="tight")
    print(f"Saved {a.out}")


if __name__ == "__main__":
    main()