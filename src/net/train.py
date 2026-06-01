import argparse
import os
import torch
import torch.nn as nn
import matplotlib.pyplot as plt
import random
import numpy as np
from torch.utils.data import DataLoader, random_split
from model   import FishSensorEstimator
from dataset import FishDataset

random.seed(42)
np.random.seed(42)
torch.manual_seed(42)

# peso della testa futuro nella loss combinata
LAMBDA_FUTURE = 0.10206261356081058


def train(model, dataset, epochs, lr, checkpoint_dir):
    n_val   = int(0.2 * len(dataset))
    n_train = len(dataset) - n_val
    train_ds, val_ds = random_split(
        dataset, [n_train, n_val],
        generator=torch.Generator().manual_seed(42)
    )
    train_loader = DataLoader(train_ds, batch_size=32, shuffle=True)
    val_loader   = DataLoader(val_ds,   batch_size=32)

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

            # loss storia:  (batch, h, 3) vs (batch, h, 3)
            loss_history = mse(pred_history, t_hist)
            # loss futuro:  (batch, 3)        vs (batch, 3)
            loss_future  = mse(pred_future,  t_fut)
            loss = loss_history + LAMBDA_FUTURE * loss_future

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
                val_loss += (loss_history + LAMBDA_FUTURE * loss_future).item()

        train_loss /= len(train_loader)
        val_loss   /= len(val_loader)

        scheduler.step(val_loss)
        train_losses.append(train_loss)
        val_losses.append(val_loss)

        print(f"Epoch {epoch:3d} | train {train_loss:.4f} | val {val_loss:.4f}")

        if val_loss < best_val_loss:
            best_val_loss = val_loss
            save_checkpoint(model, dataset.norm_stats, checkpoint_dir, name="best.pt")

    print(f"\nTraining completato. Best val loss: {best_val_loss:.4f}")
    return model, train_losses, val_losses


def save_checkpoint(model, norm_stats, checkpoint_dir, name="checkpoint.pt"):
    os.makedirs(checkpoint_dir, exist_ok=True)
    path = os.path.join(checkpoint_dir, name)
    torch.save({"model_state": model.state_dict(), "norm_stats": norm_stats}, path)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--log_dir',        default='../../logs/ds')
    parser.add_argument('--checkpoint_dir', default='checkpoints/')
    parser.add_argument('--epochs',         type=int,   default=100)
    parser.add_argument('--lr',             type=float, default=0.0035796261186095872)
    args = parser.parse_args()

    print("Caricamento dataset...")
    dataset = FishDataset(args.log_dir)

    model = FishSensorEstimator()
    n_params = sum(p.numel() for p in model.parameters())
    print(f"Parametri della rete: {n_params}")

    print("\nInizio training...")
    model, train_losses, val_losses = train(
        model, dataset,
        epochs=args.epochs,
        lr=args.lr,
        checkpoint_dir=args.checkpoint_dir,
    )

    # salvataggio finale
    os.makedirs(args.checkpoint_dir, exist_ok=True)
    final_path = os.path.join(args.checkpoint_dir, "fish_sensor_estimator.pt")
    torch.save({
        "model_state": model.state_dict(),
        "norm_stats":  dataset.norm_stats,
        "theta_star":  {n: p.clone() for n, p in model.named_parameters()},
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
    plt.show()