"""
Disegna, per un trial reale, le VERE predizioni della rete confrontate con il
segnale vero.

Per ogni finestra i della history (i = h .. n-2) il modello predice il valore
al tempo i+1 (pred_future). Facendo scorrere i lungo tutto il trial si ottiene
una serie temporale continua di predizioni "un passo avanti" (teacher forced:
la history in input e' sempre quella vera, non autoregressiva), direttamente
confrontabile punto per punto col segnale reale e con la MSE futuro che gia'
misuri con channel_loss.py.

Uso:
  python3 src/net/SensorEstimator/checkpoints/plot_prediction.py --checkpoint src/net/SensorEstimator/checkpoints/best.pt
  python3 src/net/SensorEstimator/checkpoints/plot_prediction.py --list_trials //elenca i trial ==> sceglierne uno che è nel val
  python3 src/net/SensorEstimator/checkpoints/plot_prediction.py --checkpoint src/net/SensorEstimator/checkpoints/best.pt --trial trial_20260519_152944.csv ==> per specificare quale trial
  (Se non specificato prende il primo trial nel validation)
"""
import argparse
import os
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import torch

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT  = os.path.abspath(os.path.join(SCRIPT_DIR, "..", "..", "..", ".."))
sys.path.insert(0, os.path.join(REPO_ROOT, "src"))

from net.SensorEstimator.model import FishSensorEstimator
from net.SensorEstimator.dataset import FishDataset

CHANNELS_ALL = ["sensor_diff", "sensor_mean", "current"]
CHANNELS_2   = ["sensor_diff", "current"]
CHANNEL_TO_SCALER_KEY = {"sensor_diff": "sd", "sensor_mean": "sm", "current": "vf"}
CHANNEL_UNIT = {"sensor_diff": "unita' sensore", "sensor_mean": "unita' sensore", "current": "mA"}

# --- palette (dataviz skill, ordine categorico fisso) ---
COL_SURFACE  = "#fcfcfb"
COL_TEXT     = "#0b0b0b"
COL_TEXT_SEC = "#52514e"
COL_MUTED    = "#898781"
COL_GRID     = "#e1e0d9"
COL_BASELINE = "#c3c2b7"
COL_SIGNAL   = "#0b0b0b"
COL_MODEL    = "#2a78d6"   # categorical slot 1

# split usato per etichettare/scegliere i trial di validation.
# DEVE combaciare con quello di train.py/tune.py per essere onesto.
VAL_FRAC = 0.2
SPLIT_SEED = 42


def prepare_dataset(dataset):
    """Costruisce finestre e scaler. Nella nuova versione di FishDataset le
    finestre non esistono finche' non si chiama split_by_trial() (lo scaler
    viene fittato sul solo train, o caricato dal pkl se presente). Qui invochiamo
    lo split una volta cosi' window_trial/sequences/scalers sono disponibili, e
    restituiamo gli indici dei trial di validation per etichettarli."""
    _, val_ds = dataset.split_by_trial(val_frac=VAL_FRAC, seed=SPLIT_SEED)
    val_trial_idxs = np.unique(dataset.window_trial[np.asarray(val_ds.indices)])
    return set(int(i) for i in val_trial_idxs)


def dims_from_state_dict(sd):
    gru_hidden = sd["gru.weight_hh_l0"].shape[1]
    mlp_hidden = sd["mlp.0.weight"].shape[0]
    return int(gru_hidden), int(mlp_hidden)


def pick_default_trial(val_trial_idxs):
    """Sceglie un trial dal validation split (out-of-sample), per un grafico
    onesto. val_trial_idxs e' l'insieme calcolato in prepare_dataset()."""
    if val_trial_idxs:
        return int(sorted(val_trial_idxs)[0]), True
    return 0, False


def resolve_trial(dataset, trial_arg, val_trial_idxs):
    names = dataset.trial_names
    if trial_arg is None:
        idx, is_val = pick_default_trial(val_trial_idxs)
        print(f"Nessun --trial specificato: uso '{names[idx]}' "
              f"({'dal validation split' if is_val else 'fallback trial 0'}).")
        return idx
    # indice numerico?
    try:
        idx = int(trial_arg)
        return idx
    except ValueError:
        pass
    # match esatto, poi per sottostringa
    if trial_arg in names:
        return names.index(trial_arg)
    matches = [i for i, n in enumerate(names) if trial_arg in n]
    if len(matches) == 1:
        return matches[0]
    if not matches:
        raise ValueError(f"Nessun trial trovato per '{trial_arg}'. Usa --list_trials.")
    raise ValueError(f"'{trial_arg}' ambiguo, trovati {[names[i] for i in matches]}.")


def run_model_on_trial(dataset, model, device, trial_idx, batch_size=256):
    mask = dataset.window_trial == trial_idx
    idxs = np.nonzero(mask)[0]
    idxs.sort()  # le finestre di un trial sono gia' contigue e in ordine temporale

    seq   = dataset.sequences[idxs]
    t_fut = dataset.targets_future[idxs]

    preds = []
    with torch.no_grad():
        for start in range(0, len(idxs), batch_size):
            chunk = seq[start:start + batch_size].to(device)
            _, pred_future, _ = model(chunk)
            preds.append(pred_future.cpu())
    pred_fut = torch.cat(preds, dim=0)

    return idxs, t_fut, pred_fut


def real_time_axis(dataset_dir, trial_name, h, n_windows):
    df = pd.read_csv(Path(dataset_dir) / trial_name)
    t_full = df["t_rel_sec"].values.astype(np.float32)
    t_future = t_full[h + 1: h + 1 + n_windows]
    if len(t_future) != n_windows:
        # fallback robusto se le lunghezze non tornano esattamente
        t_future = np.arange(n_windows, dtype=np.float32) / 20.0
    return t_future


def channel_panel(ax, t, true_real, pred_real, unit, title):
    ax.set_facecolor(COL_SURFACE)
    ax.grid(True, color=COL_GRID, linewidth=0.8, zorder=0)
    for spine in ("top", "right"):
        ax.spines[spine].set_visible(False)
    for spine in ("left", "bottom"):
        ax.spines[spine].set_color(COL_BASELINE)

    ax.plot(t, pred_real, color=COL_MODEL, linewidth=1.6, zorder=3, label="predizione modello")
    ax.plot(t, true_real, color=COL_SIGNAL, linewidth=1.6, zorder=4, label="segnale reale")

    ax.set_title(title, color=COL_TEXT, fontsize=12, fontweight="bold", loc="left", pad=10)
    ax.set_ylabel(unit, color=COL_TEXT_SEC, fontsize=9)
    ax.tick_params(colors=COL_MUTED, labelsize=8)
    ax.legend(loc="upper right", frameon=False, fontsize=8, labelcolor=COL_TEXT_SEC)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--checkpoint", default=os.path.join(SCRIPT_DIR, "best.pt"))
    parser.add_argument("--dataset_dir", default=os.path.join(REPO_ROOT, "src", "net", "dataset"))
    parser.add_argument("--scaler_path", default=os.path.join(REPO_ROOT, "src", "net", "scaler", "scalers.pkl"))
    parser.add_argument("--device", default="cuda" if torch.cuda.is_available() else "cpu")
    parser.add_argument("--trial", default=None,
                         help="nome (o sottostringa) o indice del trial da plottare. "
                              "Default: primo trial del validation split.")
    parser.add_argument("--list_trials", action="store_true", help="stampa i trial disponibili ed esce")
    parser.add_argument("--out", default=os.path.join(SCRIPT_DIR, "predictions.png"))
    args = parser.parse_args()

    device = torch.device(args.device)
    dataset = FishDataset(args.dataset_dir, scaler_path=args.scaler_path)

    # Nella nuova versione di FishDataset le finestre e gli scaler non esistono
    # finche' non si chiama split_by_trial(): lo facciamo qui una volta sola.
    # Restituisce anche gli indici dei trial di validation per etichettarli.
    try:
        val_trial_idxs = prepare_dataset(dataset)
    except Exception as e:
        print(f"[avviso] non riesco a costruire il validation split ({e}); "
              f"proseguo senza etichette train/val.", file=sys.stderr)
        val_trial_idxs = set()

    if args.list_trials:
        for i, n in enumerate(dataset.trial_names):
            tag = ("val" if i in val_trial_idxs else "train") if val_trial_idxs else "?"
            print(f"{i:3d}  [{tag:5s}]  {n}")
        return

    trial_idx = resolve_trial(dataset, args.trial, val_trial_idxs)
    trial_name = dataset.trial_names[trial_idx]

    ckpt = torch.load(args.checkpoint, map_location=device)
    state = ckpt["model_state"]
    input_size = ckpt.get("input_size", dataset.sequences.shape[-1])
    gru_hidden, mlp_hidden = dims_from_state_dict(state)
    model = FishSensorEstimator(input_size=input_size, gru_hidden=gru_hidden,
                                 mlp_hidden=mlp_hidden, h=dataset.h).to(device)
    model.load_state_dict(state)
    model.eval()

    nc = dataset.targets_future.shape[-1]
    CHANNELS = CHANNELS_2 if nc == 2 else CHANNELS_ALL

    idxs, t_fut_norm, pred_fut_norm = run_model_on_trial(dataset, model, device, trial_idx)
    t_axis = real_time_axis(args.dataset_dir, trial_name, dataset.h, len(idxs))

    to_plot = [c for c in ("sensor_diff", "current") if c in CHANNELS]
    fig, axes = plt.subplots(len(to_plot), 1, figsize=(11, 3.6 * len(to_plot)), sharex=True)
    if len(to_plot) == 1:
        axes = [axes]
    fig.patch.set_facecolor(COL_SURFACE)

    for ax, ch in zip(axes, to_plot):
        ci = CHANNELS.index(ch)
        scaler = dataset.scalers[CHANNEL_TO_SCALER_KEY[ch]]
        true_real = scaler.inverse_transform(t_fut_norm[:, ci:ci + 1].numpy()).ravel()
        pred_real = scaler.inverse_transform(pred_fut_norm[:, ci:ci + 1].numpy()).ravel()

        rmse_model = float(np.sqrt(np.mean((pred_real - true_real) ** 2)))
        unit = CHANNEL_UNIT[ch]
        title = f"{ch} — RMSE modello {rmse_model:.1f} {unit}"
        channel_panel(ax, t_axis, true_real, pred_real, unit, title)

    axes[-1].set_xlabel("tempo (s)", color=COL_TEXT_SEC, fontsize=9)
    fig.suptitle(f"Predizioni reali del modello — trial {trial_name}",
                 color=COL_TEXT, fontsize=13, fontweight="bold", x=0.01, ha="left", y=0.995)
    fig.tight_layout(rect=[0, 0, 1, 0.96])
    fig.savefig(args.out, dpi=160, facecolor=COL_SURFACE)
    print(f"Salvato {args.out} (trial: {trial_name}, {len(idxs)} punti)")


if __name__ == "__main__":
    main()