import argparse
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

from net.SensorEstimator.model import FishSensorEstimator
from net.SensorEstimator.dataset import FishDataset

random.seed(42)
np.random.seed(42)
torch.manual_seed(42)

DEVICE = torch.device("cpu")


def train(model, dataset, epochs, lr, batch_size, checkpoint_dir, lambda_future):
    # split a livello di trial (niente leak da finestre sovrapposte)
    train_ds, val_ds = dataset.split_by_trial(val_frac=0.2, seed=42)
    print(f"Split per-trial: {len(train_ds)} finestre train | {len(val_ds)} finestre val")

    train_loader = DataLoader(train_ds, batch_size=batch_size, shuffle=True)
    val_loader   = DataLoader(val_ds,   batch_size=batch_size)

    optimizer = torch.optim.Adam(model.parameters(), lr=lr)
    scheduler = torch.optim.lr_scheduler.ReduceLROnPlateau(optimizer, patience=10, factor=0.5)
    mse = nn.MSELoss()

    best_val_loss = float('inf')
    train_losses, val_losses = [], []

    for epoch in range(epochs):
        # --- training ---
        model.train()
        train_loss = 0.0
        for seq, t_hist, t_fut, _ in train_loader:
            pred_history, pred_future, _ = model(seq)
            loss_history = mse(pred_history, t_hist)
            loss_future  = mse(pred_future,  t_fut)
            loss = loss_history + lambda_future * loss_future

            if not torch.isfinite(loss):
                raise RuntimeError(
                    f"Loss non finita a epoch {epoch}: training divergente "
                    f"(riduci lr) o dati sporchi (esegui check_nan.py)"
                )

            optimizer.zero_grad()
            loss.backward()
            nn.utils.clip_grad_norm_(model.parameters(), max_norm=1.0)
            optimizer.step()
            train_loss += loss.item()

        # --- validation ---
        model.eval()
        val_loss = 0.0
        with torch.no_grad():
            for seq, t_hist, t_fut, _ in val_loader:
                pred_history, pred_future, _ = model(seq)
                loss_history = mse(pred_history, t_hist)
                loss_future  = mse(pred_future,  t_fut)
                val_loss += (loss_history + lambda_future * loss_future).item()

        train_loss /= len(train_loader)
        val_loss   /= len(val_loader)

        scheduler.step(val_loss)
        train_losses.append(train_loss)
        val_losses.append(val_loss)

        print(f"Epoch {epoch:3d} | train {train_loss:.4f} | val {val_loss:.4f} "
              f"| lr {optimizer.param_groups[0]['lr']:.2e}")

        if val_loss < best_val_loss:
            best_val_loss = val_loss
            save_checkpoint(model, dataset.norm_stats, checkpoint_dir, name="best.pt")

    print(f"\nTraining completato. Best val loss: {best_val_loss:.4f}")
    return model, train_losses, val_losses


def save_checkpoint(model, norm_stats, checkpoint_dir, name="checkpoint.pt"):
    os.makedirs(checkpoint_dir, exist_ok=True)
    path = os.path.join(checkpoint_dir, name)
    state_cpu = {k: v.cpu() for k, v in model.state_dict().items()}
    # salvo anche input_size: serve a ricostruire il modello in inference/overlay
    torch.save({
        "model_state": state_cpu,
        "norm_stats":  norm_stats,
        "input_size":  model.gru.input_size,
    }, path)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--dataset_dir',        default=os.path.join(REPO_ROOT, 'src', 'net', 'dataset'))
    parser.add_argument('--checkpoint_dir', default=os.path.join(SCRIPT_DIR, 'checkpoints'))
    parser.add_argument('--epochs',         type=int,   default=100)
    parser.add_argument('--lr',             type=float, default=0.0017056448762352463)
    parser.add_argument('--batch_size',     type=int,   default=32)
    parser.add_argument('--gru_hidden',     type=int,   default=256)
    parser.add_argument('--mlp_hidden',     type=int,   default=32)
    parser.add_argument('--lambda_future', type=float, default=0.0010095021820975363,
                        help='peso della loss sulla testa "future" nella loss combinata.')
    parser.add_argument('--device',         default='cuda' if torch.cuda.is_available() else 'cpu')
    parser.add_argument('--threads',        type=int,   default=8)
    parser.add_argument('--scaler_path',    default=os.path.join(REPO_ROOT, 'src', 'net', 'scaler', 'scalers.pkl'),
                        help='normalizzatore fisso: se esiste lo carica e lo riusa, '
                             'altrimenti lo fitta sui CSV e lo salva qui. '
                             'ATTENZIONE: con amp/freq in input il vecchio pkl non e\' '
                             'compatibile, cancellalo o usa un nuovo path.')
    args = parser.parse_args()

    torch.set_num_threads(args.threads)
    DEVICE = torch.device(args.device)
    if DEVICE.type == "cuda":
        torch.backends.cudnn.benchmark = True
    print(f"Device: {DEVICE} | threads: {args.threads}")

    print("Caricamento dataset...")
    os.makedirs(os.path.dirname(args.scaler_path) or ".", exist_ok=True)
    dataset = FishDataset(args.dataset_dir, scaler_path=args.scaler_path).to(DEVICE)

    # numero di feature in input letto DIRETTAMENTE dal dataset:
    # cosi' se cambi le feature non devi aggiornare nulla qui a mano
    input_size = dataset.sequences.shape[-1]
    print(f"Feature in input per timestep: {input_size}  (storia di [cmd, amp, freq])")

    model = FishSensorEstimator(
        input_size=input_size,
        gru_hidden=args.gru_hidden,
        mlp_hidden=args.mlp_hidden,
    ).to(DEVICE)
    n_params = sum(p.numel() for p in model.parameters())
    print(f"Parametri della rete: {n_params}")

    print("\nInizio training...")
    model, train_losses, val_losses = train(
        model, dataset,
        epochs=args.epochs,
        lr=args.lr,
        batch_size=args.batch_size,
        checkpoint_dir=args.checkpoint_dir,
        lambda_future=args.lambda_future,
    )

    os.makedirs(args.checkpoint_dir, exist_ok=True)
    final_path = os.path.join(args.checkpoint_dir, "fish_sensor_estimator.pt")
    torch.save({
        "model_state": {k: v.cpu() for k, v in model.state_dict().items()},
        "norm_stats":  dataset.norm_stats,
        "input_size":  model.gru.input_size,
        "theta_star":  {n: p.detach().cpu().clone() for n, p in model.named_parameters()},
    }, final_path)
    print(f"Checkpoint finale salvato in {final_path}")

    best_epoch = val_losses.index(min(val_losses)) + 1

    fig, ax = plt.subplots(figsize=(10, 5))
    ax.plot(range(1, len(train_losses) + 1), train_losses, color='steelblue', linewidth=1.5, label='Train loss')
    ax.plot(range(1, len(val_losses)   + 1), val_losses,   color='tomato',    linewidth=1.5, label='Val loss')
    ax.axvline(best_epoch, color='gray', linewidth=1.0, linestyle='--',
               label=f'Best val (epoch {best_epoch})')
    ax.set_xlabel("Epoch", fontsize=13)
    ax.set_ylabel("Loss (MSE)", fontsize=13)
    ax.set_title("SensorEstimator — Training & Validation Loss", fontsize=15, fontweight='bold')
    ax.legend(fontsize=12)
    ax.grid(True)
    plt.tight_layout()

    plot_path = os.path.join(args.checkpoint_dir, "loss_curve.png")
    plt.savefig(plot_path, dpi=150)
    print(f"Loss curve salvata in {plot_path}")