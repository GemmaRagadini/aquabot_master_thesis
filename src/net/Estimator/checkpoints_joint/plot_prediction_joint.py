"""
Disegna, per un trial reale, le VERE predizioni delle reti congiunte (IM + FM)
confrontate col segnale vero.

Per ogni finestra i (i = h .. n-1) le reti predicono il valore al tempo i
(one-step, P=1). Facendo scorrere i lungo tutto il trial si ottiene una serie
temporale continua di predizioni "un passo avanti" (teacher forced: la history
in input e' sempre quella vera, non autoregressiva).

  FM (diretta): [C_1:H, S_1:H, ctx] -> sensori futuri [sensor_diff, current]
  IM (inversa): [C_1:H, S_1:H, ctx] -> comando futuro  [tail_target]

Con P>1 viene plottato il PRIMO passo predetto (indice 0 della finestra P).

Uso:
  python3 src/net/Estimator/checkpoints_joint/plot_prediction_joint.py --checkpoint src/net/Estimator/checkpoints_joint/best.pt
  python3 src/net/Estimator/checkpoints_joint/plot_prediction_joint.py --list_trials      # scegline uno che e' 'val'
  python3 src/net/Estimator/checkpoints_joint/plot_prediction_joint.py --checkpoint src/net/Estimator/checkpoints_joint/best.pt --trial trial_XX.csv
--step k # sceglie quale passo tra i P predetti plottare 
--t_start 5 --t_end 15 # zoom su una parte 
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

# flat import (come train_joint). In repo: from net.Joint.model import ...
from net.Estimator.model   import build_models
from net.Estimator.dataset import FishJointDataset

# canali di uscita FM (sensori) e IM (comando)
FM_CHANNELS = ["sensor_diff", "current"]
IM_CHANNELS = ["tail_target"]
CHANNEL_TO_SCALER_KEY = {"sensor_diff": "sd", "current": "vf", "tail_target": "cmd"}
CHANNEL_UNIT = {"sensor_diff": "unita' sensore", "current": "mA", "tail_target": "rad"}

# --- palette (dataviz skill, ordine categorico fisso) ---
COL_SURFACE  = "#fcfcfb"
COL_TEXT     = "#0b0b0b"
COL_TEXT_SEC = "#52514e"
COL_MUTED    = "#898781"
COL_GRID     = "#e1e0d9"
COL_BASELINE = "#c3c2b7"
COL_SIGNAL   = "#0b0b0b"
COL_MODEL    = "#2a78d6"

VAL_FRAC = 0.2
SPLIT_SEED = 42


def prepare_dataset(dataset):
    """Costruisce finestre e scaler tramite lo split (scaler fittato sul solo
    train). Restituisce l'insieme degli indici dei trial di validation."""
    _, val_ds = dataset.split_by_trial(val_frac=VAL_FRAC, seed=SPLIT_SEED)
    val_trial_idxs = np.unique(dataset.window_trial[np.asarray(val_ds.indices)])
    return set(int(i) for i in val_trial_idxs)


def pick_default_trial(val_trial_idxs):
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
    try:
        return int(trial_arg)
    except ValueError:
        pass
    if trial_arg in names:
        return names.index(trial_arg)
    matches = [i for i, n in enumerate(names) if trial_arg in n]
    if len(matches) == 1:
        return matches[0]
    if not matches:
        raise ValueError(f"Nessun trial trovato per '{trial_arg}'. Usa --list_trials.")
    raise ValueError(f"'{trial_arg}' ambiguo, trovati {[names[i] for i in matches]}.")


def run_models_on_trial(dataset, IM, FM, device, trial_idx, step=0, batch_size=256):
    """Fa girare entrambe le reti sulle finestre (contigue e ordinate) del trial.
    Ritorna target e predizioni per FM (sensori) e IM (comando), al passo
    'step' dell'orizzonte P (step=0 -> t+1, ..., step=P-1 -> t+P)."""
    mask = dataset.window_trial == trial_idx
    idxs = np.nonzero(mask)[0]
    idxs.sort()

    seq_cmd  = dataset.seq_cmd[idxs]
    seq_sens = dataset.seq_sens[idxs]
    ctx      = dataset.context[idxs]
    tgt_sens = dataset.tgt_sens[idxs]     # (n, P, 2)
    tgt_cmd  = dataset.tgt_cmd[idxs]      # (n, P, 1)

    P = tgt_cmd.shape[1]
    if not (0 <= step < P):
        raise ValueError(f"--step {step} fuori range: il modello ha P={P} "
                         f"(passi validi: 0..{P - 1}).")

    fm_pred, im_pred = [], []
    with torch.no_grad():
        for start in range(0, len(idxs), batch_size):
            sc = seq_cmd[start:start + batch_size].to(device)
            ss = seq_sens[start:start + batch_size].to(device)
            cx = ctx[start:start + batch_size].to(device)
            seq = torch.cat([sc, ss], dim=-1)          # [C, S] -> (b, H, 3)
            p_cmd,  _ = IM(seq, cx)                     # (b, P, 1)
            p_sens, _ = FM(seq, cx)                     # (b, P, 2)
            im_pred.append(p_cmd.cpu())
            fm_pred.append(p_sens.cpu())

    fm_pred = torch.cat(fm_pred, dim=0)
    im_pred = torch.cat(im_pred, dim=0)
    # passo scelto dell'orizzonte
    return (idxs, P,
            tgt_sens[:, step, :], fm_pred[:, step, :],
            tgt_cmd[:, step, :],  im_pred[:, step, :])


def real_time_axis(dataset_dir, trial_name, h, n_windows, step=0):
    df = pd.read_csv(Path(dataset_dir) / trial_name)
    t_full = df["t_rel_sec"].values.astype(np.float32)
    # il target della finestra i (input 0:H a t=i-1) e' a t=i per il primo passo;
    # il passo 'step' predice t=i+step -> l'asse slitta di 'step'.
    t_future = t_full[h + step: h + step + n_windows]
    if len(t_future) != n_windows:
        t_future = np.arange(n_windows, dtype=np.float32) / 20.0
    return t_future


def channel_panel(ax, t, true_real, pred_real, unit, title):
    ax.set_facecolor(COL_SURFACE)
    ax.grid(True, color=COL_GRID, linewidth=0.8, zorder=0)
    for spine in ("top", "right"):
        ax.spines[spine].set_visible(False)
    for spine in ("left", "bottom"):
        ax.spines[spine].set_color(COL_BASELINE)

    ax.plot(t, pred_real, color=COL_MODEL,  linewidth=1.6, zorder=3, label="predizione modello")
    ax.plot(t, true_real, color=COL_SIGNAL, linewidth=1.6, zorder=4, label="segnale reale")

    ax.set_title(title, color=COL_TEXT, fontsize=12, fontweight="bold", loc="left", pad=10)
    ax.set_ylabel(unit, color=COL_TEXT_SEC, fontsize=9)
    ax.tick_params(colors=COL_MUTED, labelsize=8)
    ax.legend(loc="upper right", frameon=False, fontsize=8, labelcolor=COL_TEXT_SEC)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--checkpoint", default=os.path.join(SCRIPT_DIR, "checkpoints_joint", "best.pt"))
    parser.add_argument("--dataset_dir", default=os.path.join(REPO_ROOT, "src", "net", "dataset"))
    parser.add_argument("--scaler_path", default=os.path.join(REPO_ROOT, "src", "net", "scaler", "scalers_joint.pkl"))
    parser.add_argument("--device", default="cuda" if torch.cuda.is_available() else "cpu")
    parser.add_argument("--trial", default=None,
                        help="nome/sottostringa/indice del trial. Default: primo del validation split.")
    parser.add_argument("--list_trials", action="store_true", help="stampa i trial disponibili ed esce")
    parser.add_argument("--t_start", type=float, default=None,
                        help="istante iniziale (s) della finestra da plottare. Default: inizio.")
    parser.add_argument("--t_end", type=float, default=None,
                        help="istante finale (s) della finestra da plottare. Default: fine.")
    parser.add_argument("--step", type=int, default=0,
                        help="quale dei P passi predetti plottare: 0=t+1 (primo, "
                             "default) ... P-1=t+P (ultimo). Con P=1 solo 0 e' valido.")
    parser.add_argument("--out", default=os.path.join(SCRIPT_DIR, "predictions_joint.png"))
    args = parser.parse_args()

    device = torch.device(args.device)
    dataset = FishJointDataset(args.dataset_dir, scaler_path=args.scaler_path)

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

    ckpt = torch.load(args.checkpoint, map_location=device, weights_only=False)
    P = ckpt.get("P", 1)
    ctx_dim = ckpt.get("ctx_dim", int(dataset.context.shape[-1]))

    # Le dimensioni NON sono salvate nel checkpoint (solo i pesi). Le deduco
    # dalle shape dello state_dict, cosi' ricostruisco IM e FM con le stesse
    # dimensioni con cui sono state allenate (che dopo il tuning NON sono i
    # default di build_models).
    def dims_from_state(sd):
        gru_hidden = sd["gru.weight_hh_l0"].shape[1]   # (3*gru_hidden, gru_hidden)
        mlp_hidden = sd["mlp.0.weight"].shape[0]        # (mlp_hidden, gru_hidden+ctx)
        return int(gru_hidden), int(mlp_hidden)

    gh_im, mh_im = dims_from_state(ckpt["im_state"])
    gh_fm, mh_fm = dims_from_state(ckpt["fm_state"])
    print(f"Dimensioni dal checkpoint: IM gru={gh_im} mlp={mh_im} | FM gru={gh_fm} mlp={mh_fm}")

    IM, FM = build_models(
        gru_hidden_im=gh_im, mlp_hidden_im=mh_im,
        gru_hidden_fm=gh_fm, mlp_hidden_fm=mh_fm,
        p=P, ctx_dim=ctx_dim)
    IM.load_state_dict(ckpt["im_state"]); IM.to(device).eval()
    FM.load_state_dict(ckpt["fm_state"]); FM.to(device).eval()

    if not (0 <= args.step < P):
        print(f"[errore] --step {args.step} fuori range: il checkpoint ha P={P} "
              f"(passi validi: 0..{P - 1}).", file=sys.stderr)
        sys.exit(1)

    idxs, P_run, tgt_sens, fm_pred, tgt_cmd, im_pred = run_models_on_trial(
        dataset, IM, FM, device, trial_idx, step=args.step)
    t_axis = real_time_axis(args.dataset_dir, trial_name, dataset.h,
                            len(idxs), step=args.step)
    print(f"P={P_run} | passo plottato: {args.step} (t+{args.step + 1})")

    # --- finestra temporale opzionale (in secondi) ---
    t_lo = args.t_start if args.t_start is not None else float(t_axis[0])
    t_hi = args.t_end   if args.t_end   is not None else float(t_axis[-1])
    if t_lo > t_hi:
        t_lo, t_hi = t_hi, t_lo
    win = (t_axis >= t_lo) & (t_axis <= t_hi)
    if not win.any():
        print(f"[avviso] finestra [{t_lo:.1f}, {t_hi:.1f}]s vuota "
              f"(trial va da {t_axis[0]:.1f} a {t_axis[-1]:.1f}s): plotto tutto.",
              file=sys.stderr)
        win = np.ones_like(t_axis, dtype=bool)
    win_t    = torch.from_numpy(win)
    t_axis   = t_axis[win]
    tgt_sens = tgt_sens[win_t]
    fm_pred  = fm_pred[win_t]
    tgt_cmd  = tgt_cmd[win_t]
    im_pred  = im_pred[win_t]

    # 3 pannelli: sensor_diff, current (FM) + tail_target (IM)
    panels = [("sensor_diff", tgt_sens, fm_pred, 0),
              ("current",     tgt_sens, fm_pred, 1),
              ("tail_target", tgt_cmd,  im_pred, 0)]

    fig, axes = plt.subplots(len(panels), 1, figsize=(11, 3.4 * len(panels)), sharex=True)
    fig.patch.set_facecolor(COL_SURFACE)

    for ax, (ch, tgt, pred, ci) in zip(axes, panels):
        scaler = dataset.scalers[CHANNEL_TO_SCALER_KEY[ch]]
        true_real = scaler.inverse_transform(tgt[:, ci:ci + 1].numpy()).ravel()
        pred_real = scaler.inverse_transform(pred[:, ci:ci + 1].numpy()).ravel()
        rmse = float(np.sqrt(np.mean((pred_real - true_real) ** 2)))
        unit = CHANNEL_UNIT[ch]
        net = "IM" if ch == "tail_target" else "FM"
        channel_panel(ax, t_axis, true_real, pred_real, unit,
                      f"{ch} [{net}] — RMSE {rmse:.2f} {unit}")

    axes[-1].set_xlabel("tempo (s)", color=COL_TEXT_SEC, fontsize=9)
    win_txt = f"  [{t_axis[0]:.1f}–{t_axis[-1]:.1f}s]" if (args.t_start is not None or args.t_end is not None) else ""
    # mostra sempre P; se P>1, specifica quale passo dell'orizzonte e' plottato
    if P_run > 1:
        step_txt = f"  ·  P={P_run}, passo t+{args.step + 1} ({args.step + 1}/{P_run})"
    else:
        step_txt = f"  ·  P={P_run}"
    fig.suptitle(f"Predizioni reali IM + FM — trial {trial_name}{win_txt}{step_txt}",
                 color=COL_TEXT, fontsize=13, fontweight="bold", x=0.01, ha="left", y=0.995)
    fig.tight_layout(rect=[0, 0, 1, 0.96])
    fig.savefig(args.out, dpi=160, facecolor=COL_SURFACE)
    print(f"Salvato {args.out} (trial: {trial_name}, {len(idxs)} punti)")


if __name__ == "__main__":
    main()