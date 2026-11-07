"""
AUTOREGRESSIVE evaluation of the FORWARD MODEL over the WHOLE trial, without retraining,
compared with the baselines, as a function of the rollout horizon.

For every trial ONE single rollout is run from the start to the end:
  - it starts from the TRUE history of the first H samples (commands and sensors)
  - at every step the model predicts sens(t) from the TRUE command at t
  - the prediction goes back into the sensor history instead of the true value
After the first H samples the model never sees a real sensor again.

Models compared (same trials, same initial history):
  - GRU  : one or more checkpoints (seeds) -> mean +- std over the seeds
  - LSTM : one or more checkpoints (seeds), if given
  - ARX  : linear model with the same inputs, fitted on the TRAINING trials, run in rollout
  - persistence: the last true value is held for the whole trial

Produces (in the --out folder):
  rollout_horizon.pdf / .png — nRMSE vs time since the last true sensor value (1 s blocks)
  rollout_horizon.csv        — RMSE (physical units) and nRMSE per horizon, per model
  full_rollout_<channel>.png — example trial, first GRU checkpoint
  + on screen: RMSE, nRMSE and R2 over the whole rollout, per model

Usage (validation trials):
  python3 src/net/fm/tests/rollout_test.py \
      --checkpoint ckpt/best_gru_s1.pt ckpt/best_gru_s2.pt ... ckpt/best_lstm_s1.pt ...
Usage (test set; the ARX is fitted on the training data):
  python3 src/net/fm/tests/rollout_test.py --checkpoint ... \
      --dataset_dir src/net/test_set --all_trials --train_dir src/net/dataset
"""
import argparse
import os
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import torch

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT  = os.path.abspath(os.path.join(SCRIPT_DIR, "..", "..", "..", ".."))
sys.path.insert(0, os.path.join(REPO_ROOT, "src"))

from net.fm.model import load_forward_model
from net.dataset  import FishJointDataset

FM_CHANNELS = ["sensor_diff", "current"]
CH_TO_KEY = {"sensor_diff": "sd", "current": "vf"}
CH_UNIT   = {"sensor_diff": "sensor units", "current": "mA"}

SAMPLE_HZ = 10.0     # logging frequency: converts steps into seconds

COL_SURFACE  = "#fcfcfb"
COL_TEXT     = "#0b0b0b"
COL_TEXT_SEC = "#52514e"
COL_MUTED    = "#898781"
COL_GRID     = "#e1e0d9"
COL_BASELINE = "#c3c2b7"
COL_SIGNAL   = "#0b0b0b"
COL_MODEL    = "#2a78d6"   # GRU (as "model prediction" in the other plots)
COL_LSTM     = "#d95f02"
COL_ARX      = "#52514e"
COL_PERSIST  = "#898781"

VAL_FRAC   = 0.2
SPLIT_SEED = 42


# ------------------------------- rollouts -------------------------------
@torch.no_grad()
def rollout_predict(FM, seq_cmd, seq_sens, fut_cmd):
    """Autoregressive rollout over N = fut_cmd.shape[1] steps.
    seq_cmd (b,H,1), seq_sens (b,H,2): initial true history.
    fut_cmd (b,N,1): TRUE commands t .. t+N-1.
    Returns pred (b,N,2): predicted sensors t .. t+N-1.
    At every step the window is shifted and re-encoded from a zero hidden state."""
    buf_cmd, buf_sens = seq_cmd, seq_sens
    preds = []
    for k in range(fut_cmd.shape[1]):
        cmd_t = fut_cmd[:, k, :]                                  # (b,1) true
        s = FM(buf_cmd, buf_sens, cmd_t)                          # (b,2) predicted
        preds.append(s.unsqueeze(1))
        buf_cmd  = torch.cat([buf_cmd[:, 1:, :],  cmd_t.unsqueeze(1)], dim=1)
        buf_sens = torch.cat([buf_sens[:, 1:, :], s.unsqueeze(1)],     dim=1)
    return torch.cat(preds, dim=1)


def rollout_arx(W, lags, seq_cmd, seq_sens, fut_cmd):
    """Same rollout with the linear ARX (numpy, normalised space).
    Features as in metrics.arx_features: last `lags` commands, last `lags`
    sensors, command at t, constant."""
    cmd  = list(seq_cmd[:, 0])                 # history, (H,)
    sens = list(seq_sens)                      # history, (H,2)
    out = []
    for c_t in fut_cmd[:, 0]:
        x = np.concatenate([np.asarray(cmd[-lags:]).reshape(-1),
                            np.asarray(sens[-lags:]).reshape(-1),
                            [c_t, 1.0]])
        s = x @ W
        out.append(s)
        cmd.append(c_t)
        sens.append(s)
    return np.asarray(out)


def trial_arrays(dataset, trial_idx):
    """Normalised arrays of a trial: initial history, true commands and sensors."""
    idxs = np.sort(np.nonzero(dataset.window_trial == trial_idx)[0])
    seq_cmd  = dataset.seq_cmd[idxs[0]].cpu().numpy()            # (H,1)
    seq_sens = dataset.seq_sens[idxs[0]].cpu().numpy()           # (H,2)
    fut_cmd  = dataset.tgt_cmd[idxs][:, 0, :].cpu().numpy()      # (n,1)
    true     = dataset.tgt_sens[idxs][:, 0, :].cpu().numpy()     # (n,2)
    return seq_cmd, seq_sens, fut_cmd, true


def fit_arx(dataset, train_idx, lags, ridge):
    """Ridge least squares, same features as metrics.py (constant not penalised)."""
    sc = dataset.seq_cmd[train_idx][:, -lags:, :].reshape(len(train_idx), -1)
    ss = dataset.seq_sens[train_idx][:, -lags:, :].reshape(len(train_idx), -1)
    ct = dataset.tgt_cmd[train_idx][:, 0, :]
    X = torch.cat([sc, ss, ct], dim=1).cpu().numpy().astype(np.float64)
    X = np.hstack([X, np.ones((len(X), 1))])
    Y = dataset.tgt_sens[train_idx][:, 0, :].cpu().numpy().astype(np.float64)
    R = ridge * np.eye(X.shape[1])
    R[-1, -1] = 0.0
    return np.linalg.solve(X.T @ X + R, X.T @ Y)


# ------------------------------- helpers -------------------------------
def resolve_trial(dataset, trial_arg, trials):
    names = dataset.trial_names
    if trial_arg is None:
        return sorted(trials)[0]
    try:
        return int(trial_arg)
    except ValueError:
        pass
    if trial_arg in names:
        return names.index(trial_arg)
    matches = [i for i, n in enumerate(names) if trial_arg in n]
    if len(matches) == 1:
        return matches[0]
    raise ValueError(f"Trial '{trial_arg}' not found or ambiguous: {[names[i] for i in matches]}")


def style_axis(ax):
    ax.set_facecolor(COL_SURFACE)
    ax.grid(True, color=COL_GRID, linewidth=0.8, zorder=0)
    for sp in ("top", "right"):
        ax.spines[sp].set_visible(False)
    for sp in ("left", "bottom"):
        ax.spines[sp].set_color(COL_BASELINE)
    ax.tick_params(colors=COL_MUTED, labelsize=8)


def block_rmse(err, blk, n_blk):
    """err (T, n) -> RMSE per block of `blk` steps, pooled over the trials: (n_blk,)"""
    return np.array([np.sqrt((err[:, b * blk:(b + 1) * blk] ** 2).mean()) for b in range(n_blk)])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--checkpoint", nargs="+",
                    default=[os.path.join(SCRIPT_DIR, "..", "checkpoints_fm", "best.pt")],
                    help="one or more checkpoints (GRU and/or LSTM, one per seed)")
    ap.add_argument("--dataset_dir", default=os.path.join(REPO_ROOT, "src", "net", "dataset"))
    ap.add_argument("--train_dir", default=None,
                    help="training dataset for the ARX. Required with --all_trials (test set).")
    ap.add_argument("--scaler_path", default=os.path.join(REPO_ROOT, "src", "net", "scaler", "scaler.pkl"))
    ap.add_argument("--device", default="cuda" if torch.cuda.is_available() else "cpu")
    ap.add_argument("--trial", default=None,
                    help="trial for the example plot. Default: first evaluated trial.")
    ap.add_argument("--all_trials", action="store_true",
                    help="use all the trials of --dataset_dir (e.g. a test set), not only the validation split")
    ap.add_argument("--no_arx", action="store_true")
    ap.add_argument("--arx_ridge", type=float, default=1.0)
    ap.add_argument("--out", default=os.path.join(SCRIPT_DIR, "rollout_fm"), help="output FOLDER")
    args = ap.parse_args()

    if not os.path.exists(args.scaler_path):
        raise SystemExit(f"Scaler {args.scaler_path} not found: the data would be normalised "
                         f"with statistics different from the training ones.")
    device = torch.device(args.device)

    # ---- models, grouped by cell ----
    models = {}                       # "GRU"/"LSTM" -> list of (path, FM)
    H = None
    for path in args.checkpoint:
        FM, ckpt = load_forward_model(path, device)
        models.setdefault(FM.cell.upper(), []).append((path, FM))
        if H is not None and ckpt.get("H") != H:
            raise SystemExit("The checkpoints have different H.")
        H = ckpt.get("H")

    ds_kwargs = {"scaler_path": args.scaler_path, "p": 1}
    if H is not None:
        ds_kwargs["h"] = int(H)
    dataset = FishJointDataset(args.dataset_dir, **ds_kwargs)
    train_ds, val_ds = dataset.split_by_trial(val_frac=VAL_FRAC, seed=SPLIT_SEED)
    val_trials = set(int(i) for i in np.unique(dataset.window_trial[np.asarray(val_ds.indices)]))
    trials = list(range(len(dataset.trial_names))) if args.all_trials else sorted(val_trials)
    ex_idx = resolve_trial(dataset, args.trial, trials)

    # ---- ARX, always fitted on training trials ----
    W = None
    if not args.no_arx:
        if args.train_dir and os.path.abspath(args.train_dir) != os.path.abspath(args.dataset_dir):
            tr_dataset = FishJointDataset(args.train_dir, **ds_kwargs)
            tr_ds, _ = tr_dataset.split_by_trial(val_frac=VAL_FRAC, seed=SPLIT_SEED)
            W = fit_arx(tr_dataset, np.sort(np.asarray(tr_ds.indices)), dataset.h, args.arx_ridge)
        elif args.all_trials:
            raise SystemExit("With --all_trials (test set) give --train_dir for the ARX, "
                             "or use --no_arx.")
        else:
            W = fit_arx(dataset, np.sort(np.asarray(train_ds.indices)), dataset.h, args.arx_ridge)

    # ---- rollouts ----
    data = {ti: trial_arrays(dataset, ti) for ti in set(trials) | {ex_idx}}
    n_min = min(len(data[ti][3]) for ti in trials)
    true = np.stack([data[ti][3][:n_min] for ti in trials])           # (T, n_min, 2)

    preds = {}                       # name -> list (one per seed) of (T, n_min, 2)
    for name, lst in models.items():
        for path, FM in lst:
            p = []
            for ti in trials:
                sc, ss, fc, _ = data[ti]
                t = lambda a: torch.from_numpy(a).float().unsqueeze(0).to(device)
                p.append(rollout_predict(FM, t(sc), t(ss), t(fc))[0].cpu().numpy()[:n_min])
            preds.setdefault(name, []).append(np.stack(p))
    if W is not None:
        preds["ARX"] = [np.stack([rollout_arx(W, dataset.h, *data[ti][:3])[:n_min] for ti in trials])]
    preds["Persistence"] = [np.stack([np.repeat(data[ti][1][-1:], n_min, axis=0) for ti in trials])]

    order = [m for m in ("GRU", "LSTM", "ARX", "Persistence") if m in preds]
    colors = {"GRU": COL_MODEL, "LSTM": COL_LSTM, "ARX": COL_ARX, "Persistence": COL_PERSIST}
    styles = {"GRU": "-", "LSTM": "-", "ARX": "--", "Persistence": ":"}
    print(f"\nWhole-trial rollout | {len(trials)} {'trials (all)' if args.all_trials else 'validation trials'} | "
          f"common duration: {n_min} steps ({n_min / SAMPLE_HZ:.1f} s) | "
          + ", ".join(f"{m}: {len(preds[m])} run(s)" for m in order))

    # ---- error vs horizon (1 s blocks) ----
    blk = int(SAMPLE_HZ)
    n_blk = n_min // blk
    rows = {"from_s": np.arange(n_blk), "to_s": np.arange(1, n_blk + 1)}
    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)

    fig, axes = plt.subplots(1, len(FM_CHANNELS), figsize=(11, 3.8))
    fig.patch.set_facecolor(COL_SURFACE)
    x = np.arange(n_blk) + 0.5
    print()
    for ci, ch in enumerate(FM_CHANNELS):
        scale = float(dataset.scalers[CH_TO_KEY[ch]].scale_[0])   # norm -> real units
        std = float(true[:, :, ci].std()) * scale                  # std of the measured signal
        ax = axes[ci]
        style_axis(ax)
        ax.axhline(1.0, color=COL_BASELINE, linewidth=1.0, linestyle="--", zorder=1)
        for m in order:
            runs_blk, runs_all = [], []
            for p in preds[m]:
                err = (p[:, :, ci] - true[:, :, ci]) * scale
                runs_blk.append(block_rmse(err, blk, n_blk))
                runs_all.append(float(np.sqrt((err ** 2).mean())))
            rb, ra = np.array(runs_blk), np.array(runs_all)
            mean_b = rb.mean(0)
            std_b = rb.std(0, ddof=1) if len(rb) > 1 else np.zeros(n_blk)
            rows[f"{ch}_RMSE_{m}"] = mean_b
            rows[f"{ch}_nRMSE_{m}"] = mean_b / std
            if len(rb) > 1:
                rows[f"{ch}_nRMSE_{m}_std"] = std_b / std
            ax.plot(x, mean_b / std, color=colors[m], linestyle=styles[m], linewidth=2.0, zorder=3)
            if len(rb) > 1:
                ax.fill_between(x, (mean_b - std_b) / std, (mean_b + std_b) / std,
                                color=colors[m], alpha=0.2, linewidth=0, zorder=2)
            pm = f" ± {ra.std(ddof=1):.4g}" if len(ra) > 1 else ""
            print(f"{ch:12s} {m:12s} RMSE over the whole rollout: {ra.mean():.4g}{pm} {CH_UNIT[ch]} | "
                  f"nRMSE {ra.mean() / std:.3f} | R2 {1.0 - (ra.mean() / std) ** 2:.3f}")
        print(f"{ch:12s} signal std {std:.4g} {CH_UNIT[ch]}\n")
        ax.set_ylim(bottom=0)
        ax.set_xlabel("time since last real sensor value (s)", color=COL_TEXT_SEC, fontsize=9)
        ax.set_ylabel("nRMSE", color=COL_TEXT_SEC, fontsize=9)
        ax.set_title(ch, color=COL_TEXT, fontsize=12, fontweight="bold", loc="left", pad=8)

    labels = {"GRU": "GRU", "LSTM": "LSTM", "ARX": "ARX", "Persistence": "persistence"}
    handles = [plt.Line2D([], [], color=colors[m], linestyle=styles[m], linewidth=2.0) for m in order]
    handles.append(plt.Line2D([], [], color=COL_BASELINE, linewidth=1.0, linestyle="--"))
    fig.legend(handles, [labels[m] for m in order] + ["nRMSE = 1 (predicting the mean)"],
               loc="lower center", ncol=len(handles), frameon=False, fontsize=8,
               labelcolor=COL_TEXT_SEC)
    fig.tight_layout(rect=[0, 0.07, 1, 1])
    for ext in ("pdf", "png"):
        fig.savefig(out_dir / f"rollout_horizon.{ext}", dpi=160, facecolor=COL_SURFACE)
    plt.close(fig)

    table = pd.DataFrame(rows).round(4)
    table.to_csv(out_dir / "rollout_horizon.csv", index=False)
    pd.set_option("display.width", 260)
    pd.set_option("display.max_columns", None)
    print(table[["from_s", "to_s"] + [c for c in table.columns if "nRMSE" in c and not c.endswith("_std")]]
          .to_string(index=False))
    print(f"\nSaved {out_dir / 'rollout_horizon.pdf'} and {out_dir / 'rollout_horizon.csv'}")

    # ---- example trial: first GRU checkpoint (or the first model given) ----
    first = (models.get("GRU") or next(iter(models.values())))[0][1]
    sc, ss, fc, t_ex = data[ex_idx]
    tt = lambda a: torch.from_numpy(a).float().unsqueeze(0).to(device)
    p_ex = rollout_predict(first, tt(sc), tt(ss), tt(fc))[0].cpu().numpy()
    name = dataset.trial_names[ex_idx]
    t = (dataset.h + np.arange(len(p_ex))) / SAMPLE_HZ
    raw = pd.read_csv(Path(args.dataset_dir) / name)
    if "t_rel_sec" in raw.columns and len(raw) >= dataset.h + len(p_ex):
        t = raw["t_rel_sec"].values[dataset.h: dataset.h + len(p_ex)].astype(np.float32)
    for ci, ch in enumerate(FM_CHANNELS):
        sc_ = dataset.scalers[CH_TO_KEY[ch]]
        inv = lambda a: sc_.inverse_transform(a[:, ci:ci + 1]).ravel()
        p_real, t_real = inv(p_ex), inv(t_ex)
        rmse_ex = float(np.sqrt(np.mean((p_real - t_real) ** 2)))

        fig, ax = plt.subplots(1, 1, figsize=(11, 3.8))
        fig.patch.set_facecolor(COL_SURFACE)
        style_axis(ax)
        ax.plot(t, p_real, color=COL_MODEL,  linewidth=1.6, zorder=3)
        ax.plot(t, t_real, color=COL_SIGNAL, linewidth=1.6, zorder=4)
        ax.set_title(f"{ch} — RMSE {rmse_ex:.3g} {CH_UNIT[ch]} over the whole trial",
                     color=COL_TEXT, fontsize=12, fontweight="bold", loc="left", pad=10)
        ax.set_xlabel("time (s)", color=COL_TEXT_SEC, fontsize=9)
        ax.set_ylabel(CH_UNIT[ch], color=COL_TEXT_SEC, fontsize=9)
        ax.legend([plt.Line2D([], [], color=COL_SIGNAL, linewidth=1.6),
                   plt.Line2D([], [], color=COL_MODEL, linewidth=1.6)],
                  ["real signal", "model, free-running (real commands only)"],
                  loc="upper center", bbox_to_anchor=(0.5, -0.2), ncol=2,
                  frameon=False, fontsize=8, labelcolor=COL_TEXT_SEC)
        fig.tight_layout()
        path = out_dir / f"full_rollout_{ch}.png"
        fig.savefig(path, dpi=160, facecolor=COL_SURFACE, bbox_inches="tight")
        plt.close(fig)
        print(f"Saved {path}  (example trial {name})")


if __name__ == "__main__":
    main()