"""
Valutazione AUTOREGRESSIVA del FORWARD MODEL su TUTTO il trial, senza riallenare.

Per ogni trial si fa UN solo rollout dall'inizio alla fine:
  - si parte dalla storia VERA dei primi H campioni (comandi e sensori)
  - a ogni passo il modello predice sens(t) dal comando VERO a t
  - la predizione rientra nella storia dei sensori al posto del valore vero
Dopo i primi H campioni il modello non rivede piu' nessun sensore reale: usa
solo le proprie predizioni e i comandi veri, fino alla fine del trial.

Produce (nella cartella --out):
  full_rollout_<canale>.png  — trial d'esempio: segnale vero vs modello
  full_rmse_vs_time.png      — RMSE in funzione del tempo dall'ultimo sensore
                               vero (blocchi di 1 s), sui trial di validation
  + a schermo: RMSE e R2 sull'intero rollout, e la tabella secondo per secondo

Uso:
  python3 src/net/fm/tests/rollout_test.py --checkpoint src/net/fm/checkpoints_fm/best_fm_tuned.pt
  python3 src/net/fm/tests/rollout_test.py --checkpoint ... --trial trial_106.csv
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

# flat import (come train_fm). In repo: from net.Estimator.model_fm import ...
from net.fm.model import build_model
from net.dataset  import FishJointDataset

FM_CHANNELS = ["sensor_diff", "current"]
CH_TO_KEY = {"sensor_diff": "sd", "current": "vf"}
CH_UNIT   = {"sensor_diff": "sensor units", "current": "mA"}

SAMPLE_HZ = 10.0     # frequenza di logging: converte i passi in secondi

COL_SURFACE  = "#fcfcfb"
COL_TEXT     = "#0b0b0b"
COL_TEXT_SEC = "#52514e"
COL_MUTED    = "#898781"
COL_GRID     = "#e1e0d9"
COL_BASELINE = "#c3c2b7"
COL_SIGNAL   = "#0b0b0b"
COL_MODEL    = "#2a78d6"

VAL_FRAC   = 0.2
SPLIT_SEED = 42


def load_model(checkpoint, device):
    """Ricostruisce il forward model con gli iperparametri salvati nel checkpoint."""
    ckpt = torch.load(checkpoint, map_location=device, weights_only=False)
    if ckpt.get("residual", True):
        raise ValueError(
            f"{checkpoint} e' stato allenato con uscita residua, non piu' "
            f"supportata: riallena con train_fm.py.")
    FM = build_model(gru_hidden=ckpt["gru_hidden"], mlp_hidden=ckpt["mlp_hidden"],
                     num_layers=ckpt.get("num_layers", 1))
    FM.load_state_dict(ckpt["fm_state"])
    FM.to(device).eval()
    print(f"Modello dal checkpoint: GRU {FM.gru_hidden}x{FM.num_layers} | "
          f"MLP {FM.mlp_hidden} | H={ckpt.get('H')}")
    return FM, ckpt


@torch.no_grad()
def rollout_predict(FM, seq_cmd, seq_sens, fut_cmd):
    """Rollout autoregressivo su N = fut_cmd.shape[1] passi.
    seq_cmd (b,H,1), seq_sens (b,H,2): storia vera iniziale.
    fut_cmd (b,N,1): comandi VERI t .. t+N-1.
    Ritorna pred (b,N,2): sensori predetti t .. t+N-1."""
    buf_cmd, buf_sens = seq_cmd, seq_sens
    preds = []
    for k in range(fut_cmd.shape[1]):
        cmd_t = fut_cmd[:, k, :]                                  # (b,1) vero
        s = FM(buf_cmd, buf_sens, cmd_t)                          # (b,2) predetto
        preds.append(s.unsqueeze(1))
        buf_cmd  = torch.cat([buf_cmd[:, 1:, :],  cmd_t.unsqueeze(1)], dim=1)
        buf_sens = torch.cat([buf_sens[:, 1:, :], s.unsqueeze(1)],     dim=1)
    return torch.cat(preds, dim=1)


def rollout_trial(dataset, FM, device, trial_idx):
    """Un solo rollout su tutto il trial. Ritorna (pred, true) normalizzati, (n,2)."""
    idxs = np.sort(np.nonzero(dataset.window_trial == trial_idx)[0])
    seq_cmd  = dataset.seq_cmd[idxs[:1]]                       # (1,H,1) storia iniziale
    seq_sens = dataset.seq_sens[idxs[:1]]                      # (1,H,2)
    fut_cmd  = dataset.tgt_cmd[idxs][:, 0, :].unsqueeze(0)     # (1,n,1) comandi veri
    true     = dataset.tgt_sens[idxs][:, 0, :].cpu().numpy()   # (n,2)
    pred = rollout_predict(FM, seq_cmd.to(device), seq_sens.to(device),
                           fut_cmd.to(device))[0].cpu().numpy()
    return pred, true


def resolve_trial(dataset, trial_arg, val_trials):
    names = dataset.trial_names
    if trial_arg is None:
        return sorted(val_trials)[0] if val_trials else 0
    try:
        return int(trial_arg)
    except ValueError:
        pass
    if trial_arg in names:
        return names.index(trial_arg)
    matches = [i for i, n in enumerate(names) if trial_arg in n]
    if len(matches) == 1:
        return matches[0]
    raise ValueError(f"Trial '{trial_arg}' non trovato o ambiguo: {[names[i] for i in matches]}")


def style_axis(ax):
    ax.set_facecolor(COL_SURFACE)
    ax.grid(True, color=COL_GRID, linewidth=0.8, zorder=0)
    for sp in ("top", "right"):
        ax.spines[sp].set_visible(False)
    for sp in ("left", "bottom"):
        ax.spines[sp].set_color(COL_BASELINE)
    ax.tick_params(colors=COL_MUTED, labelsize=8)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--checkpoint", default=os.path.join(SCRIPT_DIR, "checkpoints_fm", "best.pt"))
    ap.add_argument("--dataset_dir", default=os.path.join(REPO_ROOT, "src", "net", "dataset"))
    ap.add_argument("--scaler_path", default=os.path.join(REPO_ROOT, "src", "net", "scaler", "scalers_joint.pkl"))
    ap.add_argument("--device", default="cuda" if torch.cuda.is_available() else "cpu")
    ap.add_argument("--trial", default=None,
                    help="trial per il grafico d'esempio. Default: primo del validation split.")
    ap.add_argument("--out", default=os.path.join(SCRIPT_DIR, "rollout_fm"),
                    help="CARTELLA di output")
    args = ap.parse_args()

    device = torch.device(args.device)
    FM, ckpt = load_model(args.checkpoint, device)

    ds_kwargs = {"scaler_path": args.scaler_path, "p": 1}
    if ckpt.get("H") is not None:
        ds_kwargs["h"] = int(ckpt["H"])
    dataset = FishJointDataset(args.dataset_dir, **ds_kwargs)
    _, val_ds = dataset.split_by_trial(val_frac=VAL_FRAC, seed=SPLIT_SEED)
    val_trials = set(int(i) for i in np.unique(
        dataset.window_trial[np.asarray(val_ds.indices)]))
    trials = sorted(val_trials)
    ex_idx = resolve_trial(dataset, args.trial, val_trials)

    outs = {ti: rollout_trial(dataset, FM, device, ti) for ti in set(trials) | {ex_idx}}
    n_min = min(len(outs[ti][0]) for ti in trials)
    print(f"Rollout su tutto il trial | {len(trials)} trial di validation | "
          f"durata comune: {n_min} passi ({n_min / SAMPLE_HZ:.1f} s) | "
          f"esempio: {dataset.trial_names[ex_idx]}")

    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)
    pred = np.stack([outs[ti][0][:n_min] for ti in trials])    # (T, n_min, 2)
    true = np.stack([outs[ti][1][:n_min] for ti in trials])

    # ---- errore in funzione del tempo dall'inizio del rollout (blocchi di 1 s) ----
    blk = int(SAMPLE_HZ)
    n_blk = n_min // blk
    rows = {"from_s": np.arange(n_blk), "to_s": np.arange(1, n_blk + 1)}
    fig, axes = plt.subplots(1, len(FM_CHANNELS), figsize=(11, 3.8))
    fig.patch.set_facecolor(COL_SURFACE)
    print()
    for ci, ch in enumerate(FM_CHANNELS):
        scale = float(dataset.scalers[CH_TO_KEY[ch]].scale_[0])   # norm -> unita' reali
        err = (pred[:, :, ci] - true[:, :, ci]) * scale
        rmse_blk = np.array([np.sqrt((err[:, b * blk:(b + 1) * blk] ** 2).mean())
                             for b in range(n_blk)])
        std = float(true[:, :, ci].std()) * scale
        rmse_all = float(np.sqrt((err ** 2).mean()))
        r2 = 1.0 - rmse_all ** 2 / std ** 2
        rows[f"{ch}_RMSE"] = rmse_blk
        print(f"{ch:12s} RMSE su tutto il rollout: {rmse_all:.4g} {CH_UNIT[ch]} | "
              f"std segnale {std:.4g} | R2 {r2:.3f}")

        ax = axes[ci]
        style_axis(ax)
        ax.axhline(std, color=COL_BASELINE, linewidth=1.0, linestyle="--", zorder=1)
        ax.plot(np.arange(n_blk) + 0.5, rmse_blk, color=COL_MODEL, linewidth=2.0, zorder=3)
        ax.set_ylim(bottom=0)
        ax.set_xlabel("time since last real sensor value (s)", color=COL_TEXT_SEC, fontsize=9)
        ax.set_ylabel(f"RMSE [{CH_UNIT[ch]}]", color=COL_TEXT_SEC, fontsize=9)
        ax.set_title(ch, color=COL_TEXT, fontsize=12, fontweight="bold", loc="left", pad=8)
    fig.legend([plt.Line2D([], [], color=COL_MODEL, linewidth=2.0),
                plt.Line2D([], [], color=COL_BASELINE, linewidth=1.0, linestyle="--")],
               ["model (autoregressive, whole trial)", "signal std (predicting the mean)"],
               loc="lower center", ncol=2, frameon=False, fontsize=8,
               labelcolor=COL_TEXT_SEC)
    fig.suptitle(f"Forward model — error along a full-trial rollout ({len(trials)} trials)",
                 color=COL_TEXT, fontsize=12, fontweight="bold", x=0.01, ha="left", y=0.99)
    fig.tight_layout(rect=[0, 0.07, 1, 0.94])
    path = out_dir / "full_rmse_vs_time.png"
    fig.savefig(path, dpi=160, facecolor=COL_SURFACE)
    plt.close(fig)
    print(f"Salvato {path}")

    pd.set_option("display.width", 220)
    print()
    print(pd.DataFrame(rows).round(4).to_string(index=False))

    # ---- trial d'esempio: un solo rollout dall'inizio alla fine ----
    p_ex, t_ex = outs[ex_idx]
    name = dataset.trial_names[ex_idx]
    t = (dataset.h + np.arange(len(p_ex))) / SAMPLE_HZ
    raw = pd.read_csv(Path(args.dataset_dir) / name)
    if "t_rel_sec" in raw.columns and len(raw) >= dataset.h + len(p_ex):
        t = raw["t_rel_sec"].values[dataset.h: dataset.h + len(p_ex)].astype(np.float32)
    split_txt = "val" if ex_idx in val_trials else "train"
    print()
    for ci, ch in enumerate(FM_CHANNELS):
        sc = dataset.scalers[CH_TO_KEY[ch]]
        inv = lambda a: sc.inverse_transform(a[:, ci:ci + 1]).ravel()
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
        fig.suptitle(f"Forward model, full-trial autoregressive rollout — trial {name} "
                     f"[{split_txt}]", color=COL_TEXT, fontsize=12,
                     fontweight="bold", x=0.01, ha="left", y=0.99)
        fig.tight_layout(rect=[0, 0, 1, 0.93])
        path = out_dir / f"full_rollout_{ch}.png"
        fig.savefig(path, dpi=160, facecolor=COL_SURFACE, bbox_inches="tight")
        plt.close(fig)
        print(f"Salvato {path}")


if __name__ == "__main__":
    main()