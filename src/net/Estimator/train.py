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

# in repo: from net.Estimator.model import ... / from net.Estimator.dataset import ...
from model   import build_models, P as MODEL_P
from dataset import FishJointDataset, CTX_DIM, P as DATA_P

random.seed(42)
np.random.seed(42)
torch.manual_seed(42)

DEVICE = torch.device("cpu")


def cycle_loss(IM, FM, seq_cmd, seq_sens, ctx, pred_cmd, pred_sens,
               tgt_cmd, tgt_sens, mse):
    """FASE B — termine di consistenza closed-loop.

    In Fase A NON viene chiamato (lambda_cyc=0). Lo isolo qui cosi' in Fase B
    modifichi solo questa funzione (in particolare la ricomposizione della
    finestra scorsa quando P>1) senza toccare il resto del training.

    Idea: scorri la finestra in avanti di P mettendo le predizioni al posto dei
    valori futuri veri, rivaluta le reti e chiedi coerenza. Placeholder one-step.
    """
    raise NotImplementedError("Ciclo attivato in Fase B.")


def train(IM, FM, dataset, epochs, lr, batch_size, checkpoint_dir, lambda_cyc):
    train_ds, val_ds = dataset.split_by_trial(val_frac=0.2, seed=42)
    print(f"Split per-trial: {len(train_ds)} finestre train | {len(val_ds)} finestre val")

    train_loader = DataLoader(train_ds, batch_size=batch_size, shuffle=True)
    val_loader   = DataLoader(val_ds,   batch_size=batch_size)

    params = list(IM.parameters()) + list(FM.parameters())
    optimizer = torch.optim.Adam(params, lr=lr)
    scheduler = torch.optim.lr_scheduler.ReduceLROnPlateau(optimizer, patience=10, factor=0.5)
    mse = nn.MSELoss()

    best_val_loss = float('inf')
    train_losses, val_losses = [], []
    train_im_losses, train_fm_losses = [], []
    val_im_losses,   val_fm_losses   = [], []

    for epoch in range(epochs):
        IM.train(); FM.train()
        train_loss = 0.0
        train_im = 0.0
        train_fm = 0.0
        for seq_cmd, seq_sens, ctx, tgt_cmd, tgt_sens, _ in train_loader:
            # ingresso condiviso [C_T, S_T] -> (batch, H, 3)
            seq = torch.cat([seq_cmd, seq_sens], dim=-1)

            pred_cmd,  _ = IM(seq, ctx)     # (batch, P, 1)
            pred_sens, _ = FM(seq, ctx)     # (batch, P, 2)

            loss_im = mse(pred_cmd,  tgt_cmd)
            loss_fm = mse(pred_sens, tgt_sens)
            loss = loss_im + loss_fm

            # --- FASE B: aggancio del ciclo ---
            if lambda_cyc > 0:
                loss = loss + lambda_cyc * cycle_loss(
                    IM, FM, seq_cmd, seq_sens, ctx,
                    pred_cmd, pred_sens, tgt_cmd, tgt_sens, mse)

            if not torch.isfinite(loss):
                raise RuntimeError(
                    f"Loss non finita a epoch {epoch}: training divergente "
                    f"(riduci lr / lambda_cyc) o dati sporchi (check_nan.py)")

            optimizer.zero_grad()
            loss.backward()
            nn.utils.clip_grad_norm_(params, max_norm=1.0)
            optimizer.step()
            train_loss += loss.item()
            train_im   += loss_im.item()
            train_fm   += loss_fm.item()

        IM.eval(); FM.eval()
        val_loss = 0.0
        val_im = 0.0
        val_fm = 0.0
        with torch.no_grad():
            for seq_cmd, seq_sens, ctx, tgt_cmd, tgt_sens, _ in val_loader:
                seq = torch.cat([seq_cmd, seq_sens], dim=-1)
                pred_cmd,  _ = IM(seq, ctx)
                pred_sens, _ = FM(seq, ctx)
                l_im = mse(pred_cmd,  tgt_cmd)
                l_fm = mse(pred_sens, tgt_sens)
                val_im   += l_im.item()
                val_fm   += l_fm.item()
                val_loss += (l_im + l_fm).item()

        nbt = len(train_loader)
        nbv = len(val_loader)
        train_loss /= nbt; train_im /= nbt; train_fm /= nbt
        val_loss   /= nbv; val_im   /= nbv; val_fm   /= nbv
        scheduler.step(val_loss)
        train_losses.append(train_loss); val_losses.append(val_loss)
        train_im_losses.append(train_im); train_fm_losses.append(train_fm)
        val_im_losses.append(val_im);     val_fm_losses.append(val_fm)

        print(f"Epoch {epoch:3d} | train {train_loss:.4f} (IM {train_im:.4f} FM {train_fm:.4f}) "
              f"| val {val_loss:.4f} (IM {val_im:.4f} FM {val_fm:.4f}) "
              f"| lr {optimizer.param_groups[0]['lr']:.2e}")

        if val_loss < best_val_loss:
            best_val_loss = val_loss
            save_checkpoint(IM, FM, dataset.norm_stats, checkpoint_dir, name="best.pt")

    print(f"\nTraining completato. Best val loss: {best_val_loss:.4f}")
    return IM, FM, {
        "train": train_losses, "val": val_losses,
        "train_im": train_im_losses, "train_fm": train_fm_losses,
        "val_im": val_im_losses, "val_fm": val_fm_losses,
    }


def save_checkpoint(IM, FM, norm_stats, checkpoint_dir, name="checkpoint.pt"):
    os.makedirs(checkpoint_dir, exist_ok=True)
    path = os.path.join(checkpoint_dir, name)
    torch.save({
        "im_state":   {k: v.cpu() for k, v in IM.state_dict().items()},
        "fm_state":   {k: v.cpu() for k, v in FM.state_dict().items()},
        "norm_stats": norm_stats,
        "im_input_size": IM.gru.input_size,
        "fm_input_size": FM.gru.input_size,
        "ctx_dim":    IM.ctx_dim,
        "P":          IM.p,
    }, path)


if __name__ == '__main__':
    assert MODEL_P == DATA_P, f"P disallineato: model={MODEL_P} dataset={DATA_P}"

    parser = argparse.ArgumentParser()
    parser.add_argument('--dataset_dir',    default=os.path.join(REPO_ROOT, 'src', 'net', 'dataset'))
    parser.add_argument('--checkpoint_dir', default=os.path.join(SCRIPT_DIR, 'checkpoints_joint'))
    parser.add_argument('--epochs',         type=int,   default=50)
    parser.add_argument('--lr',             type=float, default=0.002)
    parser.add_argument('--batch_size',     type=int,   default=128)
    parser.add_argument('--gru_hidden_im',  type=int,   default=128)
    parser.add_argument('--mlp_hidden_im',  type=int,   default=64)
    parser.add_argument('--gru_hidden_fm',  type=int,   default=256)
    parser.add_argument('--mlp_hidden_fm',  type=int,   default=256)
    parser.add_argument('--lambda_cyc',     type=float, default=0.0,
                        help='FASE A: 0 (reti allenate sui dati reali, nessun ciclo). '
                             'FASE B: accendi con warm-up.')
    parser.add_argument('--device',         default='cuda' if torch.cuda.is_available() else 'cpu')
    parser.add_argument('--threads',        type=int,   default=8)
    parser.add_argument('--scaler_path',    default=os.path.join(REPO_ROOT, 'src', 'net', 'scaler', 'scalers_joint.pkl'))
    args = parser.parse_args()

    torch.set_num_threads(args.threads)
    DEVICE = torch.device(args.device)
    if DEVICE.type == "cuda":
        torch.backends.cudnn.benchmark = True
    print(f"Device: {DEVICE} | threads: {args.threads} | lambda_cyc: {args.lambda_cyc}")

    print("Caricamento dataset...")
    os.makedirs(os.path.dirname(args.scaler_path) or ".", exist_ok=True)
    dataset = FishJointDataset(args.dataset_dir, scaler_path=args.scaler_path).to(DEVICE)

    IM, FM = build_models(
        gru_hidden_im=args.gru_hidden_im, mlp_hidden_im=args.mlp_hidden_im,
        gru_hidden_fm=args.gru_hidden_fm, mlp_hidden_fm=args.mlp_hidden_fm,
    )
    IM = IM.to(DEVICE); FM = FM.to(DEVICE)
    n_im = sum(p.numel() for p in IM.parameters())
    n_fm = sum(p.numel() for p in FM.parameters())
    print(f"Parametri: IM={n_im} | FM={n_fm} | tot={n_im + n_fm}")

    print("\nInizio training congiunto...")
    IM, FM, hist = train(
        IM, FM, dataset,
        epochs=args.epochs, lr=args.lr, batch_size=args.batch_size,
        checkpoint_dir=args.checkpoint_dir, lambda_cyc=args.lambda_cyc,
    )

    os.makedirs(args.checkpoint_dir, exist_ok=True)
    final_path = os.path.join(args.checkpoint_dir, "fish_joint.pt")
    torch.save({
        "im_state":   {k: v.cpu() for k, v in IM.state_dict().items()},
        "fm_state":   {k: v.cpu() for k, v in FM.state_dict().items()},
        "norm_stats": dataset.norm_stats,
        "im_input_size": IM.gru.input_size,
        "fm_input_size": FM.gru.input_size,
        "ctx_dim":    IM.ctx_dim,
        "P":          IM.p,
    }, final_path)
    print(f"Checkpoint finale salvato in {final_path}")

    epochs_x = range(1, len(hist["train"]) + 1)
    best_epoch = hist["val"].index(min(hist["val"])) + 1

    # due pannelli: sinistra loss totale, destra IM vs FM separate
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 5))

    ax1.plot(epochs_x, hist["train"], color='steelblue', linewidth=1.5, label='Train loss')
    ax1.plot(epochs_x, hist["val"],   color='tomato',    linewidth=1.5, label='Val loss')
    ax1.axvline(best_epoch, color='gray', linewidth=1.0, linestyle='--', label=f'Best val (epoch {best_epoch})')
    ax1.set_xlabel("Epoch", fontsize=13); ax1.set_ylabel("Loss (MSE)", fontsize=13)
    ax1.set_title("Totale (IM + FM)", fontsize=14, fontweight='bold')
    ax1.legend(fontsize=11); ax1.grid(True)

    ax2.plot(epochs_x, hist["train_im"], color='steelblue', linewidth=1.5, label='IM train')
    ax2.plot(epochs_x, hist["val_im"],   color='steelblue', linewidth=1.5, linestyle='--', label='IM val')
    ax2.plot(epochs_x, hist["train_fm"], color='seagreen',  linewidth=1.5, label='FM train')
    ax2.plot(epochs_x, hist["val_fm"],   color='seagreen',  linewidth=1.5, linestyle='--', label='FM val')
    ax2.set_xlabel("Epoch", fontsize=13); ax2.set_ylabel("Loss (MSE)", fontsize=13)
    ax2.set_title("IM (comando) vs FM (sensori)", fontsize=14, fontweight='bold')
    ax2.legend(fontsize=11); ax2.grid(True)

    fig.suptitle("Joint IM+FM — Training & Validation Loss", fontsize=16, fontweight='bold')
    plt.tight_layout()
    plot_path = os.path.join(args.checkpoint_dir, "loss_curve.png")
    plt.savefig(plot_path, dpi=150)
    print(f"Loss curve salvata in {plot_path}")