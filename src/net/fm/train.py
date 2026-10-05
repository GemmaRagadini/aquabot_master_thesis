import argparse
import json
import os
import sys
import random
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import torch
import torch.nn as nn
from torch.utils.data import DataLoader

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT  = os.path.abspath(os.path.join(SCRIPT_DIR, "..", "..", ".."))
sys.path.insert(0, os.path.join(REPO_ROOT, "src"))

# in repo: from net.Estimator.model_fm import ... / from net.Estimator.dataset import ...
from model import build_model, H as MODEL_H
from net.dataset  import FishJointDataset, H as DATA_H

random.seed(42)
np.random.seed(42)
torch.manual_seed(42)

# ---------------------------------------------------------------------------
"""
FORWARD MODEL a 1 passo:  sens(t) = f(storia cmd+sens fino a t-1, cmd(t))

Il dataset viene costruito con p=1, quindi per ogni finestra:
  seq_cmd  (H,1)  cmd  t-H .. t-1
  seq_sens (H,2)  sens t-H .. t-1
  tgt_cmd  (1,1)  cmd(t)   -> INGRESSO del modello
  tgt_sens (1,2)  sens(t)  -> TARGET
  (il ctx [amp, freq, center] restituito dal dataset NON viene usato)

NOTE:
  - Baseline di PERSISTENZA: sens(t) = sens(t-1). A 10 Hz e' una baseline forte:
    se la val loss non scende chiaramente sotto, il modello non sta imparando
    nulla di utile. Viene calcolata una volta sul val e stampata/plottata.
  - Loss per canale (sd, current) nel log: servono a capire quale sensore limita.
  - Loss normalizzate per numero di CAMPIONI, non di batch.
  - Lo scaler condiviso (--scaler_path) va tenuto UGUALE tra i run per confrontarli.
  - --seed cambia solo inizializzazione/shuffle; lo split train/val resta fisso
    (seed 42 in split_by_trial).
  - on_epoch_end: hook per Optuna (report/pruning/early stop).

USO:
  python3 src/net/Estimator/train_fm.py --tag fm_base

Output in --checkpoint_dir: best_<tag>.pt (miglior val), fm_<tag>.pt (finale),
loss_history_<tag>.json, loss_curve_<tag>.png.
"""
# ---------------------------------------------------------------------------


def unpack_batch(batch):
    """Dal batch del dataset (p=1) agli ingressi/target del forward model."""
    seq_cmd, seq_sens, _ctx, tgt_cmd, tgt_sens, _ = batch
    cmd_t  = tgt_cmd[:, 0, :]     # (B, 1)  comando a t
    y      = tgt_sens[:, 0, :]    # (B, 2)  sensori a t
    return seq_cmd, seq_sens, cmd_t, y


@torch.no_grad()
def persistence_baseline(loader, n):
    """MSE di sens(t) = sens(t-1), totale e per canale (sd, current)."""
    tot = np.zeros(2)
    for batch in loader:
        _, seq_sens, _, y = unpack_batch(batch)
        err = ((seq_sens[:, -1, :] - y) ** 2).mean(dim=0)      # (2,)
        tot += err.cpu().numpy() * y.shape[0]
    per_ch = tot / n
    return float(per_ch.mean()), per_ch


def train(FM, dataset, epochs, lr, batch_size, checkpoint_dir,
          weight_decay=0.0, clip_norm=1.0, best_name="best.pt", meta=None,
          save_checkpoints=True, on_epoch_end=None):
    """on_epoch_end(epoch, va_loss, best_val, best_epoch) -> bool: hook opzionale
    (tuning Optuna). Se ritorna True il training si ferma. save_checkpoints=False
    non scrive nulla su disco (tuning)."""
    train_ds, val_ds = dataset.split_by_trial(val_frac=0.2, seed=42)
    print(f"Split per-trial: {len(train_ds)} finestre train | {len(val_ds)} finestre val")

    train_loader = DataLoader(train_ds, batch_size=batch_size, shuffle=True)
    val_loader   = DataLoader(val_ds,   batch_size=batch_size)
    n_train, n_val = len(train_ds), len(val_ds)

    base_val, base_ch = persistence_baseline(val_loader, n_val)
    print(f"Baseline persistenza (val): {base_val:.4f} "
          f"(sd {base_ch[0]:.4f} | current {base_ch[1]:.4f})")

    optimizer = torch.optim.AdamW(FM.parameters(), lr=lr, weight_decay=weight_decay)
    scheduler = torch.optim.lr_scheduler.ReduceLROnPlateau(optimizer, patience=10, factor=0.5)
    mse = nn.MSELoss()

    best_val_loss = float('inf')
    best_epoch = -1
    hist = {k: [] for k in ("train", "val", "val_sd", "val_current")}

    for epoch in range(epochs):
        FM.train()
        tr_loss = 0.0
        for batch in train_loader:
            seq_cmd, seq_sens, cmd_t, y = unpack_batch(batch)
            pred = FM(seq_cmd, seq_sens, cmd_t)
            loss = mse(pred, y)

            if not torch.isfinite(loss):
                raise RuntimeError(f"Loss non finita a epoch {epoch}: riduci lr "
                                   f"o controlla i dati (check_nan.py)")

            optimizer.zero_grad()
            loss.backward()
            nn.utils.clip_grad_norm_(FM.parameters(), max_norm=clip_norm)
            optimizer.step()
            tr_loss += loss.item() * y.shape[0]

        FM.eval()
        va_ch = np.zeros(2)
        with torch.no_grad():
            for batch in val_loader:
                seq_cmd, seq_sens, cmd_t, y = unpack_batch(batch)
                pred = FM(seq_cmd, seq_sens, cmd_t)
                err = ((pred - y) ** 2).mean(dim=0)           # (2,) per canale
                va_ch += err.cpu().numpy() * y.shape[0]

        # normalizzazione per numero di CAMPIONI
        tr_loss /= n_train
        va_ch   /= n_val
        va_loss  = float(va_ch.mean())       # == nn.MSELoss su (B,2)

        scheduler.step(va_loss)
        hist["train"].append(tr_loss); hist["val"].append(va_loss)
        hist["val_sd"].append(float(va_ch[0])); hist["val_current"].append(float(va_ch[1]))

        ratio = va_loss / base_val if base_val > 0 else float('nan')
        print(f"Epoch {epoch:3d} | train {tr_loss:.4f} | val {va_loss:.4f} "
              f"(sd {va_ch[0]:.4f} current {va_ch[1]:.4f}) | val/persist {ratio:.3f} "
              f"| lr {optimizer.param_groups[0]['lr']:.2e}")

        if va_loss < best_val_loss:
            best_val_loss = va_loss
            best_epoch = epoch
            if save_checkpoints:
                save_checkpoint(FM, dataset.norm_stats, checkpoint_dir, name=best_name,
                                meta={**(meta or {}), "best_epoch": epoch})

        if on_epoch_end is not None and on_epoch_end(epoch, va_loss, best_val_loss, best_epoch):
            break

    print(f"\nTraining completato. Best val: {best_val_loss:.4f} a epoch {best_epoch} "
          f"(persistenza {base_val:.4f})")
    hist["best_epoch"] = best_epoch
    hist["best_val"]   = best_val_loss
    hist["persistence_val"] = base_val
    hist["persistence_sd"]  = float(base_ch[0])
    hist["persistence_current"] = float(base_ch[1])
    return FM, hist


def _ckpt_dict(FM, norm_stats, meta=None):
    d = {
        "fm_state":   {k: v.cpu() for k, v in FM.state_dict().items()},
        "norm_stats": norm_stats,
        "gru_hidden": FM.gru_hidden,
        "mlp_hidden": FM.mlp_hidden,
        "num_layers": FM.num_layers,
        "residual":   False,      # uscita diretta sens(t): i vecchi checkpoint
                                  # con residual=True NON sono compatibili
        "H":          MODEL_H,
    }
    if meta:
        d["train_meta"] = dict(meta)
    return d


def save_checkpoint(FM, norm_stats, checkpoint_dir, name="checkpoint.pt", meta=None):
    os.makedirs(checkpoint_dir, exist_ok=True)
    torch.save(_ckpt_dict(FM, norm_stats, meta), os.path.join(checkpoint_dir, name))


def checkpoint_names(tag=None):
    """tag=None -> ('best.pt', 'fm.pt') ; tag='x' -> ('best_x.pt', 'fm_x.pt')"""
    if not tag:
        return "best.pt", "fm.pt"
    return f"best_{tag}.pt", f"fm_{tag}.pt"


if __name__ == '__main__':
    assert MODEL_H == DATA_H, f"H disallineato: model={MODEL_H} dataset={DATA_H}"

    parser = argparse.ArgumentParser()
    parser.add_argument('--dataset_dir',    default=os.path.join(REPO_ROOT, 'src', 'net', 'dataset'))
    parser.add_argument('--checkpoint_dir', default=os.path.join(SCRIPT_DIR, 'checkpoints_fm'))
    parser.add_argument('--epochs',         type=int,   default=80)
    parser.add_argument('--lr',             type=float, default=0.0005621588402773221)
    parser.add_argument('--batch_size',     type=int,   default=64)
    parser.add_argument('--gru_hidden',     type=int,   default=256)
    parser.add_argument('--mlp_hidden',     type=int,   default=512)
    parser.add_argument('--num_layers',     type=int,   default=1)
    parser.add_argument('--dropout',        type=float, default=0.06668923974698159)
    parser.add_argument('--weight_decay',   type=float, default=1.582283830185358e-05)
    parser.add_argument('--clip_norm',      type=float, default=0.5)
    parser.add_argument('--device',         default='cuda' if torch.cuda.is_available() else 'cpu')
    parser.add_argument('--threads',        type=int,   default=8)
    parser.add_argument('--scaler_path',    default=os.path.join(REPO_ROOT, 'src', 'net', 'scaler', 'scalers_joint.pkl'))
    parser.add_argument('--tag',            default=None,
                        help="tag per distinguere la run (best_<tag>.pt, fm_<tag>.pt, "
                             "loss_curve_<tag>.png)")
    parser.add_argument('--seed',           type=int,   default=42,
                        help="seed di inizializzazione pesi e shuffle. Lo split "
                             "train/val NON dipende da questo (resta seed 42).")
    args = parser.parse_args()

    random.seed(args.seed)
    np.random.seed(args.seed)
    torch.manual_seed(args.seed)

    tag = args.tag
    best_name, final_name = checkpoint_names(tag)
    if tag:
        print(f"Tag run: '{tag}' -> checkpoint: {best_name}, {final_name}")

    torch.set_num_threads(args.threads)
    DEVICE = torch.device(args.device)
    if DEVICE.type == "cuda":
        torch.backends.cudnn.benchmark = True
    print(f"Device: {DEVICE} | threads: {args.threads}")

    print("Caricamento dataset (p=1: target = sensori a t)...")
    os.makedirs(os.path.dirname(args.scaler_path) or ".", exist_ok=True)
    dataset = FishJointDataset(args.dataset_dir, h=MODEL_H, p=1,
                               scaler_path=args.scaler_path).to(DEVICE)

    FM = build_model(gru_hidden=args.gru_hidden, mlp_hidden=args.mlp_hidden,
                     num_layers=args.num_layers, dropout=args.dropout).to(DEVICE)
    n_par = sum(p.numel() for p in FM.parameters())
    print(f"Parametri FM: {n_par} | GRU {FM.gru_hidden}x{FM.num_layers} | "
          f"MLP {FM.mlp_hidden}")

    meta = {"tag": tag, "seed": args.seed, "optimizer": "AdamW", "lr": args.lr,
            "weight_decay": args.weight_decay, "dropout": args.dropout,
            "batch_size": args.batch_size, "clip_norm": args.clip_norm}

    print("\nInizio training forward model...")
    FM, hist = train(FM, dataset, epochs=args.epochs, lr=args.lr,
                     batch_size=args.batch_size, checkpoint_dir=args.checkpoint_dir,
                     weight_decay=args.weight_decay, clip_norm=args.clip_norm,
                     best_name=best_name, meta=meta)

    os.makedirs(args.checkpoint_dir, exist_ok=True)
    final_path = os.path.join(args.checkpoint_dir, final_name)
    torch.save(_ckpt_dict(FM, dataset.norm_stats,
                          {**meta, "best_epoch": hist["best_epoch"]}), final_path)
    print(f"Checkpoint finale salvato in {final_path}")

    hist_name = f"loss_history_{tag}.json" if tag else "loss_history.json"
    with open(os.path.join(args.checkpoint_dir, hist_name), "w") as f:
        json.dump({k: (float(v) if isinstance(v, (int, float)) else [float(x) for x in v])
                   for k, v in hist.items()}, f)
    print(f"Storia loss salvata in {os.path.join(args.checkpoint_dir, hist_name)}")

    # --- plot ---
    epochs_x   = range(1, len(hist["train"]) + 1)
    best_epoch = hist["best_epoch"] + 1

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 6))

    ax1.plot(epochs_x, hist["train"], color='steelblue', linewidth=1.5, label='Train')
    ax1.plot(epochs_x, hist["val"],   color='tomato',    linewidth=1.5, label='Val')
    ax1.axhline(hist["persistence_val"], color='black', linewidth=1.0, linestyle=':',
                label='Persistence baseline (val)')
    if hist["best_epoch"] >= 0:
        ax1.axvline(best_epoch, color='gray', linewidth=1.0, linestyle='--',
                    label=f'Best val (epoch {best_epoch})')
    ax1.set_xlabel("Epoch", fontsize=13); ax1.set_ylabel("Loss (MSE) — per sample", fontsize=13)
    ax1.set_yscale('log')
    ax1.set_title("Forward model — sens(t)", fontsize=14, fontweight='bold')
    ax1.legend(fontsize=9, loc='upper center', bbox_to_anchor=(0.5, -0.16), ncol=4, frameon=False)
    ax1.grid(True, which='both')

    ax2.plot(epochs_x, hist["val_sd"], color='seagreen',   linewidth=1.5, label='Val sensor_diff')
    ax2.plot(epochs_x, hist["val_current"], color='darkorange', linewidth=1.5, label='Val current')
    ax2.axhline(hist["persistence_sd"], color='seagreen',   linewidth=1.0, linestyle=':',
                label='Persistence sensor_diff')
    ax2.axhline(hist["persistence_current"], color='darkorange', linewidth=1.0, linestyle=':',
                label='Persistence current')
    ax2.set_xlabel("Epoch", fontsize=13); ax2.set_ylabel("Loss (MSE) — per sample", fontsize=13)
    ax2.set_yscale('log')
    ax2.set_title("Val per canale", fontsize=14, fontweight='bold')
    ax2.legend(fontsize=9, loc='upper center', bbox_to_anchor=(0.5, -0.16), ncol=2, frameon=False)
    ax2.grid(True, which='both')

    suptitle = "Forward model (GRU + MLP)"
    if tag:
        suptitle += f"  [{tag}]"
    fig.suptitle(suptitle, fontsize=16, fontweight='bold')
    plt.tight_layout()
    loss_curve_name = f"loss_curve_{tag}.png" if tag else "loss_curve.png"
    plot_path = os.path.join(args.checkpoint_dir, loss_curve_name)
    plt.savefig(plot_path, dpi=150, bbox_inches='tight')
    print(f"Loss curve salvata in {plot_path}")