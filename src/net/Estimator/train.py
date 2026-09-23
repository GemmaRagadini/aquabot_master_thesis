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
  - K (passi di rollout) e P (uscite della testa) sono INDIPENDENTI: nei modi
    rollout/combo il dataset costruisce target lunghi T = K + P - 1. La loss
    diretta usa i primi P; il rollout al passo k confronta tutte le P uscite con
    tgt[k:k+P] e reimmette nei buffer solo t+1.
      P=1 -> rollout classico (predittore a 1 passo allenato in autoregressione).
      P>1 -> tutto l'orizzonte e' allenato anche in closed-loop.
  - Selezione del best: in 'combo' si usa sup + lambda_roll*roll a peso FISSO e si
    salva solo dopo il warm-up (prima il best poteva cadere a lambda~0 = supervised).
    Lo scheduler usa la stessa metrica.
  - Con T>P il numero di finestre per trial cala di T-P (coda del trial): tra run
    con T diversi il set di validazione differisce di poche finestre per trial.
    Lo scaler NON dipende da P/T (fit sui segnali interi, o caricato da file).
  - --seed cambia solo l'inizializzazione/shuffle; lo split train/val resta fisso
    (seed 42 in split_by_trial), quindi run con seed diversi sono confrontabili.
  - DUE LOSS DISTINTE, non confonderle:
      * sup  (loss_im + loss_fm) = MSE sull'INTERO orizzonte P della testa diretta
        -> e' cio' che OTTIMIZZA la predizione diretta t+1..t+P.
      * roll = closed-loop su K passi (BPTT) -> OTTIMIZZA la stabilita' in
        autoregressione; supervisiona tutte le P uscite, reimmette solo t+1.
    In 'combo' la loss totale e' sup + lambda*roll: alleni ENTRAMBI gli usi.
  - IM/FM nel log e nel plot sono la diagnostica a t+1 (slice 0), NON entrano nella
    loss ottimizzata: servono solo a leggere il t+1 in modo comparabile tra i modi.
    (Prima erano la media su tutto P: con slice non allenate davano ~0.5 fuorvianti.)
  - Lo scaler condiviso (--scaler_path) va tenuto UGUALE tra i run per confrontarli:
    il primo run lo fitta sul train e lo salva, i successivi lo riusano.
  - --tag distingue i file salvati (best_<tag>.pt, fish_joint_<tag>.pt,
    loss_curve_<tag>.png); senza tag, il modo diventa il tag (tranne 'supervised').
  - I checkpoint salvano P, H, dimensioni GRU/MLP e ctx_static: gli script di
    valutazione li rileggono da li'.

USO:
  # supervised, orizzonte P=10 (1 s a 10 Hz)
  python3 src/net/Estimator/train.py --train_mode supervised --p 10 --tag sup_p10 \
      

  # ablation gradienti separati
  python3 src/net/Estimator/train.py --train_mode supervised --p 10 --detach_cross \
      --tag sup_p10_detach

  # rollout (P=1, K=10: predittore a 1 passo allenato in closed-loop)
  python3 src/net/Estimator/train.py --train_mode rollout --p 1 --rollout_steps 10 \
      --tag roll_p1_k10

  # combo (P=1: supervised a 1 passo + closed-loop su K passi)
  python3 src/net/Estimator/train.py --train_mode combo --p 1 --rollout_steps 10 \
      --lambda_roll 1.0 --roll_warmup 10 --tag combo_p1_k10

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

    Supervisione su TUTTO l'orizzonte della testa: al passo k l'uscita (P passi)
    e' confrontata con i target veri t+1+k .. t+k+P (tgt_*[:, k:k+P]). Nei buffer
    rientra comunque solo t+1 (si avanza di un passo per volta).
      - P=1  -> supervisione del solo passo t+1 (rollout classico a 1 passo).
      - P>1  -> tutte le P uscite sono allenate in closed-loop.
    Richiede target lunghi T >= K + P - 1 (train.py costruisce il dataset cosi')."""
    T = tgt_cmd.shape[1]
    P_head = IM.p
    K = min(K, T - P_head + 1)
    if K < 1:
        raise ValueError(f"target troppo corti per il rollout: T={T}, P={P_head}")

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

        # tutta l'uscita (P passi) contro i target da t+1+k in poi
        loss = loss + mse(pc, tgt_cmd[:, k:k + P_head]) \
                    + mse(ps, tgt_sens[:, k:k + P_head])

        # avanza i buffer di uno: butta il piu' vecchio, appende la predizione
        buf_cmd  = torch.cat([buf_cmd[:, 1:, :],  c1], dim=1)
        buf_sens = torch.cat([buf_sens[:, 1:, :], s1], dim=1)

    return loss / K


def compute_losses(IM, FM, batch, mse, mode, detach_cross, K, lam_roll):
    """Calcola la loss per un batch secondo il modo scelto.

    Ritorna (loss, loss_im1, loss_fm1, loss_roll, loss_sup):
      - loss      : la loss OTTIMIZZATA (dipende dal modo).
      - loss_im1  : diagnostica IM a t+1 (slice 0), staccata dal grafo, solo log.
      - loss_fm1  : diagnostica FM a t+1 (slice 0), staccata dal grafo, solo log.
      - loss_roll : termine closed-loop (0 se non usato), scalare per il log.
      - loss_sup  : termine diretto full-horizon = loss_im + loss_fm (media su P),
                    scalare per il log; e' cio' che ottimizza la predizione diretta.

    NB: loss_im1/loss_fm1 NON entrano nella loss ottimizzata; l'ottimizzazione
    della testa diretta usa loss_sup (tutto l'orizzonte)."""
    seq_cmd, seq_sens, ctx, tgt_cmd, tgt_sens, _ = batch

    # forward diretto a P passi (usa TUTTO l'orizzonte): OTTIMIZZA t+1..t+P
    pred_cmd, pred_sens = forward_pair(IM, FM, seq_cmd, seq_sens, ctx, detach_cross)
    # I target possono essere piu' lunghi di P (T = max(P, K), servono al
    # rollout): la loss diretta usa solo i primi P passi, quelli della testa.
    P_head = pred_cmd.shape[1]
    loss_im = mse(pred_cmd,  tgt_cmd[:, :P_head])     # media su tutto l'orizzonte P
    loss_fm = mse(pred_sens, tgt_sens[:, :P_head])
    loss_sup = loss_im + loss_fm

    # diagnostica SOLO a t+1 (slice 0): comparabile tra i modi, fuori dal grafo.
    # e' cio' che le curve 'IM/FM @t+1' mostrano nel plot.
    loss_im1 = mse(pred_cmd[:, :1],  tgt_cmd[:, :1]).detach()
    loss_fm1 = mse(pred_sens[:, :1], tgt_sens[:, :1]).detach()

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

    return loss, loss_im1, loss_fm1, loss_roll, loss_sup


def train(IM, FM, dataset, epochs, lr, batch_size, checkpoint_dir,
          mode, detach_cross, rollout_steps, lambda_roll, roll_warmup,
          weight_decay_im=0.0, weight_decay_fm=0.0, clip_norm=1.0,
          best_name="best.pt", meta=None,
          save_checkpoints=True, on_epoch_end=None):
    """on_epoch_end(epoch, va_sel, best_val, best_epoch) -> bool: hook opzionale
    chiamato a fine epoca (lo usa il tuning Optuna per report/pruning/early stop).
    Se ritorna True il training si ferma. save_checkpoints=False non scrive nulla
    su disco (tuning)."""
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
    # AdamW: weight decay disaccoppiato dal gradiente (vero decadimento dei pesi,
    # uguale per tutti i parametri), con un valore separato per ciascuna rete.
    optimizer = torch.optim.AdamW(
        [{"params": IM.parameters(), "weight_decay": weight_decay_im},
         {"params": FM.parameters(), "weight_decay": weight_decay_fm}],
        lr=lr)
    scheduler = torch.optim.lr_scheduler.ReduceLROnPlateau(optimizer, patience=10, factor=0.5)
    mse = nn.MSELoss()

    best_val_loss = float('inf')
    best_epoch = -1
    hist = {k: [] for k in ("train", "val", "val_sel", "train_im", "train_fm",
                            "val_im", "val_fm", "train_roll", "val_roll",
                            "train_sup", "val_sup")}

    # In combo la loss ottimizzata (sup + lambda(t)*roll) cambia scala durante il
    # warm-up: con lambda~0 e' piu' bassa per costruzione. Per scheduler e scelta
    # del best si usa quindi una metrica a peso FISSO (lambda finale), e il best
    # si salva solo a warm-up concluso, cosi' best_combo.pt e' davvero un combo.
    min_save_epoch = roll_warmup if mode == "combo" else 0

    for epoch in range(epochs):
        # lambda del rollout: warm-up lineare 0 -> lambda_roll su roll_warmup epoche
        if mode == "combo":
            lam = lambda_roll * min(1.0, epoch / max(1, roll_warmup))
        elif mode == "rollout":
            lam = lambda_roll
        else:
            lam = 0.0

        IM.train(); FM.train()
        tr_loss = tr_im = tr_fm = tr_roll = tr_sup = 0.0
        for batch in train_loader:
            loss, l_im, l_fm, l_roll, l_sup = compute_losses(
                IM, FM, batch, mse, mode, detach_cross, rollout_steps, lam)

            if not torch.isfinite(loss):
                raise RuntimeError(
                    f"Loss non finita a epoch {epoch} (mode={mode}): training "
                    f"divergente (riduci lr / lambda_roll / rollout_steps) o dati "
                    f"sporchi (check_nan.py)")

            optimizer.zero_grad()
            loss.backward()
            nn.utils.clip_grad_norm_(params, max_norm=clip_norm)
            optimizer.step()
            # accumulo pesato per la dimensione REALE del batch: cosi' la somma,
            # divisa per il numero di campioni, e' una vera media per campione
            # (l'ultimo batch piu' piccolo pesa meno, com'e' giusto).
            bs = batch[0].shape[0]
            tr_loss += loss.item()   * bs; tr_im += l_im.item()   * bs
            tr_fm   += l_fm.item()   * bs; tr_roll += l_roll.item() * bs
            tr_sup  += l_sup.item()  * bs

        IM.eval(); FM.eval()
        va_loss = va_im = va_fm = va_roll = va_sup = 0.0
        with torch.no_grad():
            for batch in val_loader:
                loss, l_im, l_fm, l_roll, l_sup = compute_losses(
                    IM, FM, batch, mse, mode, detach_cross, rollout_steps, lam)
                bs = batch[0].shape[0]
                va_loss += loss.item()   * bs; va_im += l_im.item()   * bs
                va_fm   += l_fm.item()   * bs; va_roll += l_roll.item() * bs
                va_sup  += l_sup.item()  * bs

        # normalizzazione per numero di CAMPIONI (finestre), non di batch
        tr_loss/=n_train; tr_im/=n_train; tr_fm/=n_train; tr_roll/=n_train; tr_sup/=n_train
        va_loss/=n_val;   va_im/=n_val;   va_fm/=n_val;   va_roll/=n_val;   va_sup/=n_val

        # metrica di selezione (scala costante lungo il training)
        if mode == "combo":
            va_sel = va_sup + lambda_roll * va_roll
        else:
            va_sel = va_loss
        scheduler.step(va_sel)
        hist["val_sel"].append(va_sel)
        hist["train"].append(tr_loss); hist["val"].append(va_loss)
        hist["train_im"].append(tr_im); hist["train_fm"].append(tr_fm)
        hist["val_im"].append(va_im);   hist["val_fm"].append(va_fm)
        hist["train_roll"].append(tr_roll); hist["val_roll"].append(va_roll)
        hist["train_sup"].append(tr_sup);   hist["val_sup"].append(va_sup)

        # sup: diretto full-horizon; roll: closed-loop. Mostrati fuori da 'supervised'
        # (dove sup==total e roll==0).
        sup_txt  = f" sup {tr_sup:.4f}/{va_sup:.4f}"  if mode != "supervised" else ""
        roll_txt = f" roll {tr_roll:.4f}/{va_roll:.4f} (λ={lam:.2f})" if mode != "supervised" else ""
        print(f"Epoch {epoch:3d} | train {tr_loss:.4f} (IM@1 {tr_im:.4f} FM@1 {tr_fm:.4f}) "
              f"| val {va_loss:.4f} (IM@1 {va_im:.4f} FM@1 {va_fm:.4f}){sup_txt}{roll_txt} "
              f"| lr {optimizer.param_groups[0]['lr']:.2e}")

        if epoch >= min_save_epoch and va_sel < best_val_loss:
            best_val_loss = va_sel
            best_epoch = epoch
            if save_checkpoints:
                save_checkpoint(IM, FM, dataset.norm_stats, checkpoint_dir, name=best_name,
                                meta={**(meta or {}), "best_epoch": epoch})

        if on_epoch_end is not None and on_epoch_end(epoch, va_sel, best_val_loss, best_epoch):
            break

    sel_txt = f" (sup + {lambda_roll}*roll, da epoch {min_save_epoch})" if mode == "combo" else ""
    print(f"\nTraining completato ({mode}). Best val{sel_txt}: {best_val_loss:.4f} "
          f"a epoch {best_epoch}")
    hist["best_epoch"] = best_epoch
    hist["best_val"] = best_val_loss
    return IM, FM, hist


def _ckpt_dict(IM, FM, norm_stats, meta=None):
    d = {
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
    # metadati della run (train_mode, K, detach_cross, lambda, warmup, best_epoch):
    # solo informativi, gli script di valutazione non ne dipendono.
    if meta:
        d["train_meta"] = dict(meta)
    return d


def save_checkpoint(IM, FM, norm_stats, checkpoint_dir, name="checkpoint.pt", meta=None):
    os.makedirs(checkpoint_dir, exist_ok=True)
    torch.save(_ckpt_dict(IM, FM, norm_stats, meta), os.path.join(checkpoint_dir, name))


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
    # AdamW: weight decay separato per rete. NB: i valori trovati col vecchio Adam
    # (L2 nel gradiente) non sono equivalenti -> rifare il tuning.
    parser.add_argument('--weight_decay_im', type=float, default=0.0)
    parser.add_argument('--weight_decay_fm', type=float, default=2.5314946929205504e-05)
    parser.add_argument('--clip_norm',      type=float, default=1.0)
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
                        help="K passi del rollout (modi rollout/combo). Default: "
                             "max(P, 10). Indipendente da P: il dataset costruisce "
                             "target lunghi T=max(P,K), quindi funziona anche con P=1.")
    parser.add_argument('--lambda_roll', type=float, default=1.0,
                        help="peso del termine di rollout (modi rollout/combo).")
    parser.add_argument('--roll_warmup', type=int, default=10,
                        help="epoche di warm-up lineare di lambda_roll (solo combo).")
    parser.add_argument('--seed', type=int, default=42,
                        help="seed di inizializzazione pesi e shuffle. Lo split "
                             "train/val NON dipende da questo (resta seed 42).")
    args = parser.parse_args()

    random.seed(args.seed)
    np.random.seed(args.seed)
    torch.manual_seed(args.seed)

    if args.p != MODEL_P:
        print(f"[avviso] --p={args.p} diverso dalla costante dei file "
              f"(model={MODEL_P}, dataset={DATA_P}). Uso --p={args.p}.")
    P = args.p

    # K (passi di rollout) e' ora indipendente da P (uscite della testa).
    # Il dataset costruisce target lunghi T = max(P, K): la loss diretta usa i
    # primi P, il rollout i primi K. Cosi' il rollout ha senso anche con P=1.
    K = args.rollout_steps if args.rollout_steps is not None else max(P, 10)
    if K < 1:
        parser.error("--rollout_steps deve essere >= 1")
    uses_roll = args.train_mode in ("rollout", "combo")
    if uses_roll and K == 1:
        print(f"[avviso] train_mode={args.train_mode} con K=1: il rollout degenera "
              f"a 1 passo (== supervised a t+1).", file=sys.stderr)
    # al passo k del rollout servono i target t+1+k .. t+k+P  ->  T = K + P - 1
    T = K + P - 1 if uses_roll else P
    if uses_roll and P > 1:
        print(f"[nota] P={P}>1 in {args.train_mode}: nel rollout si reimmette solo t+1, "
              f"ma tutte le {P} uscite sono supervisionate a ogni passo (T={T}).",
              file=sys.stderr)

    # tag di default = train_mode (cosi' le run non si sovrascrivono), tranne supervised
    tag = args.tag if args.tag is not None else (
        None if args.train_mode == "supervised" else args.train_mode)
    best_name, final_name = checkpoint_names(tag)
    print(f"Train mode: {args.train_mode} | detach_cross={args.detach_cross} | "
          f"K={K} | lambda_roll={args.lambda_roll} | warmup={args.roll_warmup}")
    if tag:
        print(f"Tag run: '{tag}' -> checkpoint: {best_name}, {final_name}")
    print(f"Orizzonte di predizione P = {P} | orizzonte target dataset T = {T}")

    torch.set_num_threads(args.threads)
    DEVICE = torch.device(args.device)
    if DEVICE.type == "cuda":
        torch.backends.cudnn.benchmark = True
    print(f"Device: {DEVICE} | threads: {args.threads}")

    print("Caricamento dataset...")
    os.makedirs(os.path.dirname(args.scaler_path) or ".", exist_ok=True)
    dataset = FishJointDataset(args.dataset_dir, p=T,
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

    meta = {"train_mode": args.train_mode, "detach_cross": args.detach_cross,
            "K": K if uses_roll else None, "T": T,
            "lambda_roll": args.lambda_roll if uses_roll else None,
            "roll_warmup": args.roll_warmup if args.train_mode == "combo" else None,
            "tag": tag, "seed": args.seed,
            "optimizer": "AdamW", "weight_decay_im": args.weight_decay_im,
            "weight_decay_fm": args.weight_decay_fm, "clip_norm": args.clip_norm}

    print(f"\nInizio training congiunto (mode={args.train_mode})...")
    IM, FM, hist = train(
        IM, FM, dataset,
        epochs=args.epochs, lr=args.lr, batch_size=args.batch_size,
        checkpoint_dir=args.checkpoint_dir,
        mode=args.train_mode, detach_cross=args.detach_cross,
        rollout_steps=K, lambda_roll=args.lambda_roll, roll_warmup=args.roll_warmup,
        weight_decay_im=args.weight_decay_im, weight_decay_fm=args.weight_decay_fm,
        clip_norm=args.clip_norm, best_name=best_name, meta=meta,
    )

    os.makedirs(args.checkpoint_dir, exist_ok=True)
    final_path = os.path.join(args.checkpoint_dir, final_name)
    torch.save(_ckpt_dict(IM, FM, dataset.norm_stats,
                          {**meta, "best_epoch": hist["best_epoch"]}), final_path)
    print(f"Checkpoint finale salvato in {final_path}")

    epochs_x = range(1, len(hist["train"]) + 1)
    # epoca del checkpoint best EFFETTIVAMENTE salvato (1-based per il plot)
    best_epoch = hist["best_epoch"] + 1

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 5))

    # --- pannello sinistro: loss totale + (in combo) le due componenti ---
    ax1.plot(epochs_x, hist["train"], color='steelblue', linewidth=1.5, label='Train total')
    ax1.plot(epochs_x, hist["val"],   color='tomato',    linewidth=1.5, label='Val total')
    # In 'rollout' total == roll (curve identiche): NON le ridisegno, era la
    # duplicazione che rendeva la legenda illeggibile. In 'combo' sup e roll sono
    # davvero diversi -> li mostro entrambi.
    if args.train_mode == "combo":
        ax1.plot(epochs_x, hist["train_sup"], color='darkorange', linewidth=1.2,
                 alpha=0.9, label='Train sup (diretto P)')
        ax1.plot(epochs_x, hist["val_sup"], color='darkorange', linewidth=1.2,
                 alpha=0.9, linestyle='--', label='Val sup (diretto P)')
        ax1.plot(epochs_x, hist["train_roll"], color='seagreen', linewidth=1.2,
                 alpha=0.8, label='Train rollout (closed-loop)')
        ax1.plot(epochs_x, hist["val_roll"], color='seagreen', linewidth=1.2,
                 alpha=0.8, linestyle='--', label='Val rollout (closed-loop)')
        ax1.plot(epochs_x, hist["val_sel"], color='black', linewidth=1.2,
                 linestyle=':', label=f'Val selezione (sup + {args.lambda_roll}·roll)')
        ax1.axvspan(0.5, args.roll_warmup + 0.5, color='gray', alpha=0.08,
                    label='warm-up (best non salvato)')
    ax1.axvline(best_epoch, color='gray', linewidth=1.0, linestyle='--', label=f'Best val (epoch {best_epoch})')
    ax1.set_xlabel("Epoch", fontsize=13); ax1.set_ylabel("Loss (MSE) — per sample", fontsize=13)
    ax1.set_yscale('log')   # scala log: la coda della curva (convergenza/overfitting) resta leggibile
    ax1.set_title(f"Total — mode={args.train_mode}", fontsize=14, fontweight='bold')
    ax1.legend(fontsize=10); ax1.grid(True, which='both')

    # --- pannello destro: diagnostica a t+1 (slice 0), comparabile tra i modi ---
    ax2.plot(epochs_x, hist["train_im"], color='steelblue', linewidth=1.5, label='IM train @t+1')
    ax2.plot(epochs_x, hist["val_im"],   color='steelblue', linewidth=1.5, linestyle='--', label='IM val @t+1')
    ax2.plot(epochs_x, hist["train_fm"], color='seagreen',  linewidth=1.5, label='FM train @t+1')
    ax2.plot(epochs_x, hist["val_fm"],   color='seagreen',  linewidth=1.5, linestyle='--', label='FM val @t+1')
    ax2.set_xlabel("Epoch", fontsize=13); ax2.set_ylabel("Loss (MSE) — per sample", fontsize=13)
    ax2.set_yscale('log')   # scala log anche qui
    ax2.set_title("IM (command) vs FM (sensors) — @t+1", fontsize=14, fontweight='bold')
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