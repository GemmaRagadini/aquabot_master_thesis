import argparse
import os
import random
import matplotlib
matplotlib.use("Agg")  
import matplotlib.pyplot as plt
import numpy as np
import torch
import torch.nn as nn
from torch.utils.data import DataLoader, random_split

from model import FishSensorEstimator
from dataset import FishDataset

random.seed(42)
np.random.seed(42)
torch.manual_seed(42)

DEVICE = torch.device("cpu")


def train(model, dataset, epochs, lr, batch_size, checkpoint_dir, lambda_future):
    n_val   = int(0.2 * len(dataset))
    n_train = len(dataset) - n_val
    train_ds, val_ds = random_split(
        dataset, [n_train, n_val],
        generator=torch.Generator().manual_seed(42)
    )

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
    # state_dict portato su CPU: il checkpoint si ricarica ovunque
    # (anche sul robot, senza GPU)
    state_cpu = {k: v.cpu() for k, v in model.state_dict().items()}
    torch.save({"model_state": state_cpu, "norm_stats": norm_stats}, path)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--log_dir',        default='../../logs/ds')
    parser.add_argument('--checkpoint_dir', default='checkpoints/')
    parser.add_argument('--epochs',         type=int,   default=150)
    parser.add_argument('--lr',             type=float, default=0.0038139114562008637)
    parser.add_argument('--batch_size',     type=int,   default=32)
    parser.add_argument('--gru_hidden',     type=int,   default=512)
    parser.add_argument('--mlp_hidden',     type=int,   default=128)
    parser.add_argument('--lambda_future', type=float, default=0.053076977868201876,
                        help='peso della loss sulla testa "future" nella loss combinata')
    parser.add_argument('--device',         default='cuda' if torch.cuda.is_available() else 'cpu')
    parser.add_argument('--threads',        type=int,   default=8)
    args = parser.parse_args()

    torch.set_num_threads(args.threads)
    DEVICE = torch.device(args.device)
    if DEVICE.type == "cuda":
        torch.backends.cudnn.benchmark = True
    print(f"Device: {DEVICE} | threads: {args.threads}")

    print("Caricamento dataset...")
    dataset = FishDataset(args.log_dir).to(DEVICE)   # tutto in VRAM una volta sola

    model = FishSensorEstimator(
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

    # salvataggio finale (su CPU, ricaricabile ovunque)
    os.makedirs(args.checkpoint_dir, exist_ok=True)
    final_path = os.path.join(args.checkpoint_dir, "fish_sensor_estimator.pt")
    torch.save({
        "model_state": {k: v.cpu() for k, v in model.state_dict().items()},
        "norm_stats":  dataset.norm_stats,
        "theta_star":  {n: p.detach().cpu().clone() for n, p in model.named_parameters()},
    }, final_path)
    print(f"Checkpoint finale salvato in {final_path}")

    # plot loss curves
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