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

# in repo: from net.Joint.model import ... / from net.Joint.dataset import ...
from model   import build_models, P as MODEL_P, H as MODEL_H
from dataset import FishJointDataset, CTX_DIM, P as DATA_P, H as DATA_H

random.seed(42)
np.random.seed(42)
torch.manual_seed(42)

DEVICE = torch.device("cpu")

# ---------------------------------------------------------------------------
#MODI DI TRAINING (--train_mode):
"""
  supervised : 1 passo teacher-forced. loss = loss_im + loss_fm

  rollout    : closed-loop differenziabile su K passi (le predizioni rientrano nei
               buffer, BPTT sul rollout). Insegna stabilita' in autoregressione.

  combo      : supervised + lambda(t)*rollout, con warm-up lineare di lambda.

  FLAG ortogonale:
  --detach_cross : stacca l'hidden incrociato -> ogni rete aggiornata solo dalla
                   propria loss (gradienti separati). Vale per tutti i modi.

NOTE:
  - Il rollout supervisiona i K passi sui target t+1..t+K dell'item -> serve K<=P.
    Con P=1 degenera a 1 passo (== supervised): per un vero rollout allena con P>1.
  - Nel log e nel plot, IM/FM sono SEMPRE la loss a 1 passo (comparabile tra i modi),
    anche quando la loss ottimizzata e' il rollout.
  - Lo scaler condiviso (--scaler_path) va tenuto UGUALE tra i run per confrontarli:
    il primo run lo fitta sul train e lo salva, i successivi lo riusano.
  - --tag distingue i file salvati (best_<tag>.pt, fish_joint_<tag>.pt,
    loss_curve_<tag>.png); senza tag, il modo diventa il tag (tranne 'supervised').
  - I checkpoint salvano P, H, dimensioni GRU/MLP e ctx_static: gli script di
    valutazione li rileggono da li'.

USO:
  # supervised, orizzonte P=10 (1 s a 10 Hz)
  python3 src/net/Estimator/train.py --train_mode supervised --p 10 --tag sup_p10 \
      -

  # ablation gradienti separati
  python3 src/net/Estimator/train.py --train_mode supervised --p 10 --detach_cross \
      --tag sup_p10_detach

  # rollout
  python3 src/net/Estimator/train.py --train_mode rollout --p 10 --rollout_steps 5 \

  # combo
    python3 src/net/Estimator/train.py --train_mode combo --p 10 --lambda_roll 1.0 \
      --roll_warmup 10

Output in --checkpoint_dir: best_<tag>.pt (miglior val), fish_joint_<tag>.pt (finale),
loss_curve_<tag>.png.
"""
# ---------------------------------------------------------------------------


def forward_pair(IM, FM, seq_cmd, seq_sens, ctx, detach_cross=False):
    """Forward accoppiato a 1 passo con hidden incrociato.

    detach_cross=True stacca l'hidden dell'ALTRA rete (h_fm verso IM, h_im
    verso FM): ogni rete resta aggiornata solo dalla propria loss."""
    h_im = IM.encode(seq_cmd)    # (B, gru_hidden_im)
    h_fm = FM.encode(seq_sens)   # (B, gru_hidden_fm)
    cross_to_im = h_fm.detach() if detach_cross else h_fm
    cross_to_fm = h_im.detach() if detach_cross else h_im
    pred_cmd  = IM.decode(h_im, cross_to_im, ctx)   # (B, P, 1)
    pred_sens = FM.decode(h_fm, cross_to_fm, ctx)   # (B, P, 2)
    return pred_cmd, pred_sens


def rollout_loss(IM, FM, seq_cmd, seq_sens, ctx, tgt_cmd, tgt_sens, mse,
                 K, detach_cross=False):
    """Closed-loop rollout differenziabile (loss di ciclo, self+cross accoppiati).

    A ogni passo entrambe le reti predicono t+1 (indice 0 dell'orizzonte); le
    predizioni rientrano nei rispettivi buffer (comandi/sensori) e, via hidden
    incrociato, raggiungono anche l'altra rete al passo successivo. Il gradiente
    passa attraverso tutto il rollout (BPTT): niente .detach() sulle predizioni
    rimesse in input.

    Il contesto statico [amp, freq, center] e' tenuto costante lungo il rollout
    (regime lentamente variabile): teacher forcing solo sul contesto.

    Supervisione: il passo k contro il target vero t+1+k (tgt_*[:, k]). Richiede
    K <= P (i target disponibili nell'item)."""
    P = tgt_cmd.shape[1]
    K = min(K, P)

    buf_cmd  = seq_cmd.clone()    # (B, H, 1)
    buf_sens = seq_sens.clone()   # (B, H, 2)

    loss = seq_cmd.new_zeros(())
    for k in range(K):
        h_im = IM.encode(buf_cmd)
        h_fm = FM.encode(buf_sens)
        cross_to_im = h_fm.detach() if detach_cross else h_fm
        cross_to_fm = h_im.detach() if detach_cross else h_im
        pc = IM.decode(h_im, cross_to_im, ctx)   # (B, P, 1)
        ps = FM.decode(h_fm, cross_to_fm, ctx)   # (B, P, 2)
        c1 = pc[:, :1, :]                         # (B, 1, 1)  passo t+1
        s1 = ps[:, :1, :]                         # (B, 1, 2)

        loss = loss + mse(c1[:, 0], tgt_cmd[:, k]) + mse(s1[:, 0], tgt_sens[:, k])

        # avanza i buffer di uno: butta il piu' vecchio, appende la predizione
        buf_cmd  = torch.cat([buf_cmd[:, 1:, :],  c1], dim=1)
        buf_sens = torch.cat([buf_sens[:, 1:, :], s1], dim=1)

    return loss / K


def compute_losses(IM, FM, batch, mse, mode, detach_cross, K, lam_roll):
    """Calcola la loss per un batch secondo il modo scelto.
    Ritorna (loss_totale, loss_im, loss_fm, loss_roll) — gli ultimi tre come
    scalari per il logging (loss_im/loss_fm sempre a 1 passo, per confronto tra
    modi; loss_roll = 0 se non usato)."""
    seq_cmd, seq_sens, ctx, tgt_cmd, tgt_sens, _ = batch

    # termine a 1 passo (sempre calcolato: comparabile tra i modi)
    pred_cmd, pred_sens = forward_pair(IM, FM, seq_cmd, seq_sens, ctx, detach_cross)
    loss_im = mse(pred_cmd,  tgt_cmd)
    loss_fm = mse(pred_sens, tgt_sens)
    loss_sup = loss_im + loss_fm

    loss_roll = seq_cmd.new_zeros(())
    if mode in ("rollout", "combo"):
        loss_roll = rollout_loss(IM, FM, seq_cmd, seq_sens, ctx,
                                 tgt_cmd, tgt_sens, mse, K, detach_cross)

    if mode == "supervised":
        loss = loss_sup
    elif mode == "rollout":
        loss = loss_roll
    elif mode == "combo":
        loss = loss_sup + lam_roll * loss_roll
    else:
        raise ValueError(f"train_mode sconosciuto: {mode}")

    return loss, loss_im, loss_fm, loss_roll


def train(IM, FM, dataset, epochs, lr, batch_size, checkpoint_dir,
          mode, detach_cross, rollout_steps, lambda_roll, roll_warmup,
          weight_decay=0.0, best_name="best.pt"):
    train_ds, val_ds = dataset.split_by_trial(val_frac=0.2, seed=42)
    print(f"Split per-trial: {len(train_ds)} finestre train | {len(val_ds)} finestre val")

    train_loader = DataLoader(train_ds, batch_size=batch_size, shuffle=True)
    val_loader   = DataLoader(val_ds,   batch_size=batch_size)

    # numero TOTALE di finestre (campioni), non di batch: serve a normalizzare le
    # loss per campione (medie confrontabili tra train e val, indipendenti da come
    # sono spezzati i batch e dall'ultimo batch piu' piccolo).
    n_train = len(train_ds)
    n_val   = len(val_ds)

    params = list(IM.parameters()) + list(FM.parameters())
    optimizer = torch.optim.Adam(params, lr=lr, weight_decay=weight_decay)
    scheduler = torch.optim.lr_scheduler.ReduceLROnPlateau(optimizer, patience=10, factor=0.5)
    mse = nn.MSELoss()

    best_val_loss = float('inf')
    hist = {k: [] for k in ("train", "val", "train_im", "train_fm",
                            "val_im", "val_fm", "train_roll", "val_roll")}

    for epoch in range(epochs):
        # lambda del rollout: warm-up lineare 0 -> lambda_roll su roll_warmup epoche
        if mode == "combo":
            lam = lambda_roll * min(1.0, epoch / max(1, roll_warmup))
        elif mode == "rollout":
            lam = lambda_roll
        else:
            lam = 0.0

        IM.train(); FM.train()
        tr_loss = tr_im = tr_fm = tr_roll = 0.0
        for batch in train_loader:
            loss, l_im, l_fm, l_roll = compute_losses(
                IM, FM, batch, mse, mode, detach_cross, rollout_steps, lam)

            if not torch.isfinite(loss):
                raise RuntimeError(
                    f"Loss non finita a epoch {epoch} (mode={mode}): training "
                    f"divergente (riduci lr / lambda_roll / rollout_steps) o dati "
                    f"sporchi (check_nan.py)")

            optimizer.zero_grad()
            loss.backward()
            nn.utils.clip_grad_norm_(params, max_norm=1.0)
            optimizer.step()
            # accumulo pesato per la dimensione REALE del batch: cosi' la somma,
            # divisa per il numero di campioni, e' una vera media per campione
            # (l'ultimo batch piu' piccolo pesa meno, com'e' giusto).
            bs = batch[0].shape[0]
            tr_loss += loss.item()   * bs; tr_im += l_im.item()   * bs
            tr_fm   += l_fm.item()   * bs; tr_roll += l_roll.item() * bs

        IM.eval(); FM.eval()
        va_loss = va_im = va_fm = va_roll = 0.0
        with torch.no_grad():
            for batch in val_loader:
                loss, l_im, l_fm, l_roll = compute_losses(
                    IM, FM, batch, mse, mode, detach_cross, rollout_steps, lam)
                bs = batch[0].shape[0]
                va_loss += loss.item()   * bs; va_im += l_im.item()   * bs
                va_fm   += l_fm.item()   * bs; va_roll += l_roll.item() * bs

        # normalizzazione per numero di CAMPIONI (finestre), non di batch
        tr_loss/=n_train; tr_im/=n_train; tr_fm/=n_train; tr_roll/=n_train
        va_loss/=n_val;   va_im/=n_val;   va_fm/=n_val;   va_roll/=n_val
        scheduler.step(va_loss)
        hist["train"].append(tr_loss); hist["val"].append(va_loss)
        hist["train_im"].append(tr_im); hist["train_fm"].append(tr_fm)
        hist["val_im"].append(va_im);   hist["val_fm"].append(va_fm)
        hist["train_roll"].append(tr_roll); hist["val_roll"].append(va_roll)

        roll_txt = f" roll {tr_roll:.4f}/{va_roll:.4f} (λ={lam:.2f})" if mode != "supervised" else ""
        print(f"Epoch {epoch:3d} | train {tr_loss:.4f} (IM {tr_im:.4f} FM {tr_fm:.4f}) "
              f"| val {va_loss:.4f} (IM {va_im:.4f} FM {va_fm:.4f}){roll_txt} "
              f"| lr {optimizer.param_groups[0]['lr']:.2e}")

        if va_loss < best_val_loss:
            best_val_loss = va_loss
            save_checkpoint(IM, FM, dataset.norm_stats, checkpoint_dir, name=best_name)

    print(f"\nTraining completato ({mode}). Best val loss: {best_val_loss:.4f}")
    return IM, FM, hist


def _ckpt_dict(IM, FM, norm_stats):
    return {
        "im_state":   {k: v.cpu() for k, v in IM.state_dict().items()},
        "fm_state":   {k: v.cpu() for k, v in FM.state_dict().items()},
        "norm_stats": norm_stats,
        "im_input_size": IM.gru.input_size,   # 1 (comandi)
        "fm_input_size": FM.gru.input_size,   # 2 (sensori)
        "im_gru_hidden": IM.gru_hidden,
        "fm_gru_hidden": FM.gru_hidden,
        "im_cross_hidden": IM.cross_hidden,   # = fm_gru_hidden
        "fm_cross_hidden": FM.cross_hidden,   # = im_gru_hidden
        "ctx_static":    CTX_DIM,             # 3 (amp, freq, center)
        "H":             MODEL_H,
        "P":             IM.p,
    }


def save_checkpoint(IM, FM, norm_stats, checkpoint_dir, name="checkpoint.pt"):
    os.makedirs(checkpoint_dir, exist_ok=True)
    torch.save(_ckpt_dict(IM, FM, norm_stats), os.path.join(checkpoint_dir, name))


def checkpoint_names(tag=None):
    """Nomi dei due checkpoint a partire da un tag opzionale.
    tag=None  -> ('best.pt', 'fish_joint.pt')
    tag='rollout' -> ('best_rollout.pt', 'fish_joint_rollout.pt')
    Il tag serve a NON sovrascrivere i checkpoint tra run diverse (es. i vari
    --train_mode). Non cambia il modello."""
    if not tag:
        return "best.pt", "fish_joint.pt"
    return f"best_{tag}.pt", f"fish_joint_{tag}.pt"


if __name__ == '__main__':
    assert MODEL_P == DATA_P, f"P disallineato: model={MODEL_P} dataset={DATA_P}"
    assert MODEL_H == DATA_H, f"H disallineato: model={MODEL_H} dataset={DATA_H}"

    parser = argparse.ArgumentParser()
    parser.add_argument('--dataset_dir',    default=os.path.join(REPO_ROOT, 'src', 'net', 'dataset'))
    parser.add_argument('--checkpoint_dir', default=os.path.join(SCRIPT_DIR, 'checkpoints_joint'))
    parser.add_argument('--epochs',         type=int,   default=80)
    parser.add_argument('--p',              type=int,   default=MODEL_P,
                        help=f'orizzonte di predizione P (default: {MODEL_P}). Per il '
                             f'rollout serve P>1 (i K passi si supervisionano sui '
                             f'target t+1..t+K dell item).')
    parser.add_argument('--lr',             type=float, default=0.0003585794155087849)
    parser.add_argument('--batch_size',     type=int,   default=32)
    parser.add_argument('--gru_hidden_im',  type=int,   default=128)
    parser.add_argument('--mlp_hidden_im',  type=int,   default=64)
    parser.add_argument('--gru_hidden_fm',  type=int,   default=256)
    parser.add_argument('--mlp_hidden_fm',  type=int,   default=128)
    parser.add_argument('--dropout_im',     type=float, default=0.0)
    parser.add_argument('--dropout_fm',     type=float, default=0.10842905375567242)
    parser.add_argument('--weight_decay',   type=float, default=2.5314946929205504e-05)
    parser.add_argument('--device',         default='cuda' if torch.cuda.is_available() else 'cpu')
    parser.add_argument('--threads',        type=int,   default=8)
    parser.add_argument('--scaler_path',    default=os.path.join(REPO_ROOT, 'src', 'net', 'scaler', 'scalers_joint.pkl'))
    parser.add_argument('--tag',            default=None,
                        help="tag per distinguere la run (checkpoint best_<tag>.pt / "
                             "fish_joint_<tag>.pt e curva loss_curve_<tag>.png). Se "
                             "assente, usa il train_mode come tag (tranne 'supervised').")

    # --- selezione dello schema di training ---
    parser.add_argument('--train_mode', default='supervised',
                        choices=['supervised', 'rollout', 'combo'],
                        help="supervised: 1 passo teacher-forced (Fase A). "
                             "rollout: closed-loop differenziabile (K passi). "
                             "combo: supervised + lambda*rollout con warm-up.")
    parser.add_argument('--detach_cross', action='store_true',
                        help="stacca l'hidden incrociato: ogni rete aggiornata solo "
                             "dalla propria loss (gradienti separati). Vale per tutti i modi.")
    parser.add_argument('--rollout_steps', type=int, default=None,
                        help="K passi del rollout (modi rollout/combo). Default: P. "
                             "Viene comunque limitato a P.")
    parser.add_argument('--lambda_roll', type=float, default=1.0,
                        help="peso del termine di rollout (modi rollout/combo).")
    parser.add_argument('--roll_warmup', type=int, default=10,
                        help="epoche di warm-up lineare di lambda_roll (solo combo).")
    args = parser.parse_args()

    if args.p != MODEL_P:
        print(f"[avviso] --p={args.p} diverso dalla costante dei file "
              f"(model={MODEL_P}, dataset={DATA_P}). Uso --p={args.p}.")
    P = args.p

    K = args.rollout_steps if args.rollout_steps is not None else P
    if args.train_mode in ("rollout", "combo"):
        if P == 1:
            print(f"[avviso] train_mode={args.train_mode} con P=1: il rollout degenera "
                  f"a 1 passo (== supervised). Allena con --p>1 per un vero rollout.",
                  file=sys.stderr)
        if K > P:
            print(f"[avviso] rollout_steps={K} > P={P}: limito K a {P}.", file=sys.stderr)
            K = P

    # tag di default = train_mode (cosi' le run non si sovrascrivono), tranne supervised
    tag = args.tag if args.tag is not None else (
        None if args.train_mode == "supervised" else args.train_mode)
    best_name, final_name = checkpoint_names(tag)
    print(f"Train mode: {args.train_mode} | detach_cross={args.detach_cross} | "
          f"K={K} | lambda_roll={args.lambda_roll} | warmup={args.roll_warmup}")
    if tag:
        print(f"Tag run: '{tag}' -> checkpoint: {best_name}, {final_name}")
    print(f"Orizzonte di predizione P = {P}")

    torch.set_num_threads(args.threads)
    DEVICE = torch.device(args.device)
    if DEVICE.type == "cuda":
        torch.backends.cudnn.benchmark = True
    print(f"Device: {DEVICE} | threads: {args.threads}")

    print("Caricamento dataset...")
    os.makedirs(os.path.dirname(args.scaler_path) or ".", exist_ok=True)
    dataset = FishJointDataset(args.dataset_dir, p=P,
                               scaler_path=args.scaler_path).to(DEVICE)

    IM, FM = build_models(
        gru_hidden_im=args.gru_hidden_im, mlp_hidden_im=args.mlp_hidden_im,
        gru_hidden_fm=args.gru_hidden_fm, mlp_hidden_fm=args.mlp_hidden_fm,
        dropout_im=args.dropout_im, dropout_fm=args.dropout_fm,
        p=P,
    )
    IM = IM.to(DEVICE); FM = FM.to(DEVICE)
    n_im = sum(p.numel() for p in IM.parameters())
    n_fm = sum(p.numel() for p in FM.parameters())
    print(f"Parametri: IM={n_im} | FM={n_fm} | tot={n_im + n_fm}")
    print(f"Hidden incrociato: IM h={IM.gru_hidden}<-cross {IM.cross_hidden} | "
          f"FM h={FM.gru_hidden}<-cross {FM.cross_hidden} | ctx statico={CTX_DIM}")

    print(f"\nInizio training congiunto (mode={args.train_mode})...")
    IM, FM, hist = train(
        IM, FM, dataset,
        epochs=args.epochs, lr=args.lr, batch_size=args.batch_size,
        checkpoint_dir=args.checkpoint_dir,
        mode=args.train_mode, detach_cross=args.detach_cross,
        rollout_steps=K, lambda_roll=args.lambda_roll, roll_warmup=args.roll_warmup,
        weight_decay=args.weight_decay, best_name=best_name,
    )

    os.makedirs(args.checkpoint_dir, exist_ok=True)
    final_path = os.path.join(args.checkpoint_dir, final_name)
    torch.save(_ckpt_dict(IM, FM, dataset.norm_stats), final_path)
    print(f"Checkpoint finale salvato in {final_path}")

    epochs_x = range(1, len(hist["train"]) + 1)
    best_epoch = hist["val"].index(min(hist["val"])) + 1

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 5))

    ax1.plot(epochs_x, hist["train"], color='steelblue', linewidth=1.5, label='Train loss')
    ax1.plot(epochs_x, hist["val"],   color='tomato',    linewidth=1.5, label='Val loss')
    if args.train_mode != "supervised":
        ax1.plot(epochs_x, hist["train_roll"], color='seagreen', linewidth=1.2,
                 alpha=0.8, label='Train rollout')
        ax1.plot(epochs_x, hist["val_roll"], color='seagreen', linewidth=1.2,
                 alpha=0.8, linestyle='--', label='Val rollout')
    ax1.axvline(best_epoch, color='gray', linewidth=1.0, linestyle='--', label=f'Best val (epoch {best_epoch})')
    ax1.set_xlabel("Epoch", fontsize=13); ax1.set_ylabel("Loss (MSE) — per sample", fontsize=13)
    ax1.set_yscale('log')   # scala log: la coda della curva (convergenza/overfitting) resta leggibile
    ax1.set_title(f"Total — mode={args.train_mode}", fontsize=14, fontweight='bold')
    ax1.legend(fontsize=10); ax1.grid(True, which='both')

    ax2.plot(epochs_x, hist["train_im"], color='steelblue', linewidth=1.5, label='IM train (1-step)')
    ax2.plot(epochs_x, hist["val_im"],   color='steelblue', linewidth=1.5, linestyle='--', label='IM val (1-step)')
    ax2.plot(epochs_x, hist["train_fm"], color='seagreen',  linewidth=1.5, label='FM train (1-step)')
    ax2.plot(epochs_x, hist["val_fm"],   color='seagreen',  linewidth=1.5, linestyle='--', label='FM val (1-step)')
    ax2.set_xlabel("Epoch", fontsize=13); ax2.set_ylabel("Loss (MSE) — per sample", fontsize=13)
    ax2.set_yscale('log')   # scala log anche qui
    ax2.set_title("IM (command) vs FM (sensors) — 1-step", fontsize=14, fontweight='bold')
    ax2.legend(fontsize=10); ax2.grid(True, which='both')

    suptitle = f"Joint IM+FM — {args.train_mode}"
    if tag:
        suptitle += f"  [{tag}]"
    fig.suptitle(suptitle, fontsize=16, fontweight='bold')
    plt.tight_layout()
    loss_curve_name = f"loss_curve_{tag}.png" if tag else "loss_curve.png"
    plot_path = os.path.join(args.checkpoint_dir, loss_curve_name)
    plt.savefig(plot_path, dpi=150)
    print(f"Loss curve salvata in {plot_path}")