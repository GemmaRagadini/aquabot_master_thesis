import argparse
import os
import torch
import torch.nn as nn
import matplotlib.pyplot as plt
from torch.utils.data import DataLoader, random_split

from model   import FishStaticNet
from dataset import FishDataset

# produce un file checkpoints/fish_static_net.pt che contiene i pesi addestrati => verra' caricato dal master per fare inferenza
def train(model, dataset, epochs=100, lr=1e-3, lambda_flow=0.3, checkpoint_dir="checkpoints/"):
    n_val   = int(0.2 * len(dataset))
    n_train = len(dataset) - n_val
    train_ds, val_ds = random_split(dataset, [n_train, n_val])

    train_loader = DataLoader(train_ds, batch_size=64, shuffle=True)
    val_loader   = DataLoader(val_ds,   batch_size=64)

    optimizer = torch.optim.Adam(model.parameters(), lr=lr)
    scheduler = torch.optim.lr_scheduler.ReduceLROnPlateau(optimizer, patience=10, factor=0.5)
    mse = nn.MSELoss()

    best_val_loss = float('inf')
    train_losses, val_losses = [], []

    for epoch in range(epochs):
        # training
        model.train()
        train_loss = 0.0
        for seq, target, label in train_loader:
            cmd_pred, vflow_pred, _ = model(seq, target)
            l_cmd   = mse(cmd_pred,   label[:, 0])
            l_vflow = mse(vflow_pred, label[:, 1])
            loss    = l_cmd + lambda_flow * l_vflow

            optimizer.zero_grad()
            loss.backward()
            nn.utils.clip_grad_norm_(model.parameters(), max_norm=1.0)
            optimizer.step()
            train_loss += loss.item()

        # validation
        model.eval()
        val_loss = 0.0
        with torch.no_grad():
            for seq, target, label in val_loader:
                cmd_pred, vflow_pred, _ = model(seq, target)
                l_cmd   = mse(cmd_pred,   label[:, 0])
                l_vflow = mse(vflow_pred, label[:, 1])
                val_loss += (l_cmd + lambda_flow * l_vflow).item()

        train_loss /= len(train_loader)
        val_loss   /= len(val_loader)

        scheduler.step(val_loss)

        train_losses.append(train_loss)
        val_losses.append(val_loss)

        if epoch % 10 == 0:
            print(f"Epoch {epoch:3d} | train {train_loss:.4f} | val {val_loss:.4f}")

        if val_loss < best_val_loss:
            best_val_loss = val_loss
            save_checkpoint(model, dataset.norm_stats, checkpoint_dir, name="best.pt")

    print(f"\nTraining completato. Best val loss: {best_val_loss:.4f}")
    return model, train_losses, val_losses



# def compute_fisher(model, dataset, n_samples=500):
#     # calcola la matrice di Fisher sui pesi del modello
#     # serve per EWC nella fase di adattamento strutturale (dinamica)
#     loader = DataLoader(dataset, batch_size=1, shuffle=True) 
#     fisher = {n:torch.zeros_like(p) for n,p in model.named_parameters()} 
#     mse = nn.MSELoss() 

#     model.eval() 
#     for i, (seq,target, label) in enumerate(loader): 
#         if i >=  n_samples: 
#             break 

#         cmd_pred, _, _ = model(seq, target) 
#         loss = mse(cmd_pred, label[:,0]) 

#         model.zero_grad() 
#         loss.backward() 

#         for n,p in model.named_parameters(): 
#             if p.grad is not None: 
#                 fisher[n] += p.grad.pow(2) 
    
#     for n in fisher: 
#         fisher[n] /= n_samples
    
#     return fisher


def save_checkpoint(model, norm_stats, checkpoint_dir, name = "checkpoint.pt"): 
    os.makedirs(checkpoint_dir, exist_ok= True)
    path = os.path.join(checkpoint_dir, name) 
    # theta_star e fisher vengono aggiunti dopo compute_fisher()
    torch.save({"model_state": model.state_dict(), "norm_stats": norm_stats}, path)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--log_dir',        default='../../logs')
    parser.add_argument('--checkpoint_dir', default='checkpoints/')
    parser.add_argument('--epochs',         type=int,   default=100)
    parser.add_argument('--lr',             type=float, default=1e-3)
    parser.add_argument('--lambda_flow',    type=float, default=0.3)
    args = parser.parse_args()

    # dataset 
    print("Caricamento dataset...") 
    dataset = FishDataset(args.log_dir) 

    #modello  
    model = FishStaticNet() 
    # n_params = sum(p.numel() for p in model.parameters()) 
    # print(f"Parametri della rete in : {n_params}")

    # training 
    print("\nInizio training...")
    model, train_losses, val_losses = train(
        model, dataset, 
        epochs=args.epochs, 
        lr = args.lr, 
        lambda_flow=args.lambda_flow, 
        checkpoint_dir=args.checkpoint_dir,
    )

    # # Fisher  
    # print("\nCalcolo matrice di Fisher...") 
    # fisher = compute_fisher(model, dataset) 

    # salvataggio 
    os.makedirs(args.checkpoint_dir, exist_ok=True)
    final_path=os.path.join(args.checkpoint_dir, "fish_static_net.pt") 
    torch.save({
        "model_state": model.state_dict(), 
        "norm_stats": dataset.norm_stats,
        "theta_star": {n:p.clone() for n,p in model.named_parameters()},
        # "fisher": fisher, 
    }, final_path)
    print(f"Checkpoint finale salvato in {final_path})")

    # plot loss curves
    best_epoch = val_losses.index(min(val_losses)) + 1

    fig, ax = plt.subplots(figsize=(10, 5))
    ax.plot(range(1, len(train_losses) + 1), train_losses, color='steelblue', linewidth=1.5, label='Train loss')
    ax.plot(range(1, len(val_losses)   + 1), val_losses,   color='tomato',    linewidth=1.5, label='Val loss')
    ax.axvline(best_epoch, color='gray', linewidth=1.0, linestyle='--',
               label=f'Best val (epoch {best_epoch})')
    ax.set_xlabel("Epoch", fontsize=13)
    ax.set_ylabel("Loss (MSE)", fontsize=13)
    ax.set_title("Training & Validation Loss", fontsize=15, fontweight='bold')
    ax.legend(fontsize=12)
    ax.grid(True)
    plt.tight_layout()

    plot_path = os.path.join(args.checkpoint_dir, "loss_curve.png")
    plt.savefig(plot_path, dpi=150)
    print(f"Loss curve salvata in {plot_path}")
    plt.show()