"""
overlay.py — overlay predetto vs reale per FishSensorEstimator,
con colorazione dello sfondo per trial di provenienza.

Canali (ordine dataset):  [sensor_diff, sensor_mean, current].
Con lambda_future = 0 e' addestrata solo la testa 'history' -> default --head history.

Sfondo a bande, una per trial: mostra se le zone mal ricostruite coincidono con
trial specifici (calibrazione) o sono sparse ovunque ci siano estremi (rarita').

Richiede il dataset con window_trial/trial_names e un checkpoint che salva input_size.

Uso (da src/net/SensorEstimator):
    python overlay.py
    python overlay.py --n_steps 2400 --start 0
"""
import argparse
import os
import numpy as np
import torch
from torch.utils.data import DataLoader
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from model import FishSensorEstimator
from dataset import FishDataset

CHANNELS = ["sensor_diff", "sensor_mean", "current"]


@torch.no_grad()
def overlay(model, ds, device, head="history", start=0, n_steps=300,
            save_path="overlay.png"):
    model.eval()
    loader = DataLoader(ds, batch_size=256, shuffle=False)

    preds, trues = [], []
    for seq, t_hist, t_fut, _ in loader:
        seq = seq.to(device)
        pred_history, pred_future, _ = model(seq)
        if head == "history":
            p = pred_history[:, -1, :].cpu().numpy()
            t = t_hist[:, -1, :].cpu().numpy()
        else:
            p = pred_future.cpu().numpy()
            t = t_fut.cpu().numpy()
        preds.append(p); trues.append(t)

    preds = np.concatenate(preds)
    trues = np.concatenate(trues)
    wtrial = ds.window_trial

    end = min(start + n_steps, len(preds))
    sl = slice(start, end)
    preds_v, trues_v, wt_v = preds[sl], trues[sl], wtrial[sl]
    x = np.arange(start, end)

    change = np.flatnonzero(np.diff(wt_v)) + 1
    bounds = [0, *change.tolist(), len(wt_v)]
    band_colors = ["#f5f5f5", "#e6eef7"]

    fig, axes = plt.subplots(3, 1, figsize=(14, 9), sharex=True)
    for c in range(3):
        ax = axes[c]
        for b in range(len(bounds) - 1):
            i0, i1 = bounds[b], bounds[b + 1]
            ax.axvspan(x[i0], x[i1 - 1] + 1, color=band_colors[b % 2], alpha=0.7, zorder=0)

        ax.plot(x, trues_v[:, c], label="reale",    lw=1.6, zorder=3)
        ax.plot(x, preds_v[:, c], label="predetto", lw=1.2, alpha=0.9, zorder=3)
        ax.axhline(trues_v[:, c].mean(), color="gray", ls=":", lw=1, alpha=0.8, zorder=2)

        mse_c = np.mean((preds_v[:, c] - trues_v[:, c]) ** 2)
        ss_res = np.sum((trues_v[:, c] - preds_v[:, c]) ** 2)
        ss_tot = np.sum((trues_v[:, c] - trues_v[:, c].mean()) ** 2) + 1e-12
        r2 = 1 - ss_res / ss_tot
        ax.set_title(f"{CHANNELS[c]}   MSE = {mse_c:.4f}   R2 = {r2:.3f}")
        ax.legend(loc="upper right"); ax.grid(True, alpha=0.25, zorder=1)

    for b in range(len(bounds) - 1):
        i0, i1 = bounds[b], bounds[b + 1]
        tid = int(wt_v[i0])
        xc = (x[i0] + x[i1 - 1]) / 2
        axes[0].text(xc, axes[0].get_ylim()[1], f"trial {tid}",
                     ha="center", va="bottom", fontsize=8, color="#33628f")

    axes[-1].set_xlabel("indice campione (bande = trial di provenienza)")
    fig.suptitle(f"Overlay predetto vs reale — testa '{head}' — sfondo per trial",
                 fontweight="bold")
    fig.tight_layout()
    fig.savefig(save_path, dpi=140, bbox_inches="tight")
    print(f"salvato {save_path}")
    print(f"finestra: campioni {start}..{end}  |  trial mostrati: "
          f"{sorted(set(int(v) for v in wt_v))}")
    return fig


if __name__ == "__main__":
    here = os.path.dirname(os.path.abspath(__file__))
    repo = os.path.abspath(os.path.join(here, "..", "..", ".."))

    ap = argparse.ArgumentParser()
    ap.add_argument("--dataset_dir", default=os.path.join(repo, "src", "net", "dataset"))
    ap.add_argument("--scaler_path", default=os.path.join(repo, "src", "net", "scaler", "scalers.pkl"))
    ap.add_argument("--checkpoint",  default=os.path.join(here, "checkpoints", "best.pt"))
    ap.add_argument("--gru_hidden",  type=int, default=512)
    ap.add_argument("--mlp_hidden",  type=int, default=32)
    ap.add_argument("--head",        default="history", choices=["history", "future"])
    ap.add_argument("--start",       type=int, default=0)
    ap.add_argument("--n_steps",     type=int, default=300)
    args = ap.parse_args()

    device = torch.device("cpu")

    ds = FishDataset(args.dataset_dir, scaler_path=args.scaler_path)

    ckpt = torch.load(args.checkpoint, map_location=device)
    # input_size dal checkpoint (fallback alle feature del dataset se assente)
    input_size = ckpt.get("input_size", ds.sequences.shape[-1])
    print(f"input_size del modello: {input_size}")

    model = FishSensorEstimator(input_size=input_size,
                                gru_hidden=args.gru_hidden,
                                mlp_hidden=args.mlp_hidden).to(device)
    model.load_state_dict(ckpt["model_state"])

    overlay(model, ds, device, head=args.head, start=args.start, n_steps=args.n_steps)