"""
Distributions of the excitation commands (Figure 4.1 of the thesis).

Reads all CSV logs of the dataset and produces:
  (a) histogram of the command c
  (b) joint distribution of amplitude A and centre offset beta - beta0,
      with the admissible region |beta - beta0| + A <= A_max
  (c) histogram of the frequency f
and prints the total duration of the dataset and the coverage of the
admissible (A, offset) region.

Usage:
  python3 plot_excitation_distributions.py --dataset_dir path/to/dataset_v3 --out excitation_distributions.pdf
"""
import argparse
import glob
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

# mechanical limits and generator settings (must match master_node / launch file)
C_MIN, C_MAX = -0.288, 0.417
BETA0 = 0.064
A_MAX = 0.353
A_RANGE = (0.10, 0.34)
F_RANGE = (0.3, 2.0)
CENTER_MAX = 0.25

COL_BAR = "#2a78d6"
COL_EDGE = "#0b0b0b"
COL_LIM = "#c1666b"


def load(dataset_dir):
    files = sorted(glob.glob(os.path.join(dataset_dir, "*.csv")))
    if not files:
        raise SystemExit(f"No CSV files in {dataset_dir}")
    frames, duration = [], 0.0
    for f in files:
        d = pd.read_csv(f, usecols=["t_rel_sec", "mode", "tail_target_rad",
                                    "tail_amp_rad", "tail_freq_hz", "center_rad"])
        d = d[d["mode"] == "random_continuous"]
        if len(d) < 2:
            continue
        duration += float(d["t_rel_sec"].iloc[-1] - d["t_rel_sec"].iloc[0])
        frames.append(d)
    return pd.concat(frames, ignore_index=True), len(frames), duration


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dataset_dir", required=True)
    ap.add_argument("--out", default="excitation_distributions.pdf")
    ap.add_argument("--bins", type=int, default=40)
    args = ap.parse_args()

    d, n_files, duration = load(args.dataset_dir)
    c = d["tail_target_rad"].to_numpy()
    A = d["tail_amp_rad"].to_numpy()
    f = d["tail_freq_hz"].to_numpy()
    off = d["center_rad"].to_numpy() - BETA0

    # ---- coverage of the admissible (A, offset) region ----
    a_edges = np.linspace(A_RANGE[0], A_RANGE[1], args.bins + 1)
    o_edges = np.linspace(-CENTER_MAX, CENTER_MAX, args.bins + 1)
    H, _, _ = np.histogram2d(A, off, bins=[a_edges, o_edges])
    ac = 0.5 * (a_edges[:-1] + a_edges[1:])
    oc = 0.5 * (o_edges[:-1] + o_edges[1:])
    AA, OO = np.meshgrid(ac, oc, indexing="ij")
    admissible = np.abs(OO) <= np.minimum(CENTER_MAX, A_MAX - AA)
    visited = (H > 0) & admissible
    on_boundary = np.isclose(np.abs(off), np.minimum(CENTER_MAX, A_MAX - A), atol=1e-3)

    print(f"Files: {n_files} | samples: {len(d)} | total duration: "
          f"{duration:.0f} s = {duration / 60:.1f} min = {duration / 3600:.2f} h")
    print(f"Command c: [{c.min():.3f}, {c.max():.3f}] rad "
          f"(mechanical range [{C_MIN}, {C_MAX}])")
    print(f"Amplitude A: [{A.min():.3f}, {A.max():.3f}] rad | mean {A.mean():.3f}")
    print(f"Frequency f: [{f.min():.3f}, {f.max():.3f}] Hz | mean {f.mean():.3f}")
    print(f"Offset beta-beta0: [{off.min():.3f}, {off.max():.3f}] rad")
    print(f"Coverage of the admissible (A, offset) region: "
          f"{visited.sum()}/{admissible.sum()} cells = {visited.sum() / admissible.sum():.1%}")
    print(f"Samples with the offset on the constraint boundary: {on_boundary.mean():.1%}")

    # ---- figure ----
    fig, ax = plt.subplots(1, 3, figsize=(12, 3.6))

    ax[0].hist(c, bins=args.bins, color=COL_BAR, edgecolor="white", linewidth=0.4)
    for x in (C_MIN, C_MAX):
        ax[0].axvline(x, color=COL_LIM, linestyle="--", linewidth=1)
    ax[0].set_xlabel(r"command $c$ (rad)")
    ax[0].set_ylabel("samples")
    ax[0].set_title("(a)", loc="left")

    im = ax[1].hist2d(A, off, bins=[a_edges, o_edges], cmap="Blues", cmin=1)[3]
    a_line = np.linspace(A_RANGE[0], A_RANGE[1], 200)
    lim = np.minimum(CENTER_MAX, A_MAX - a_line)
    ax[1].plot(a_line, lim, color=COL_LIM, linewidth=1.2)
    ax[1].plot(a_line, -lim, color=COL_LIM, linewidth=1.2)
    ax[1].set_xlabel(r"amplitude $A$ (rad)")
    ax[1].set_ylabel(r"offset $\beta-\beta_0$ (rad)")
    ax[1].set_title("(b)", loc="left")
    fig.colorbar(im, ax=ax[1], label="samples")

    ax[2].hist(f, bins=args.bins, color=COL_BAR, edgecolor="white", linewidth=0.4)
    for x in F_RANGE:
        ax[2].axvline(x, color=COL_LIM, linestyle="--", linewidth=1)
    ax[2].set_xlabel(r"frequency $f$ (Hz)")
    ax[2].set_ylabel("samples")
    ax[2].set_title("(c)", loc="left")

    for a in ax:
        a.spines["top"].set_visible(False)
        a.spines["right"].set_visible(False)
    fig.tight_layout()
    fig.savefig(args.out, bbox_inches="tight")
    print(f"Saved {args.out}")


if __name__ == "__main__":
    main()