"""
Disegna, per un trial reale, le VERE predizioni delle reti congiunte (IM + FM)
confrontate col segnale vero, con una BANDA min-max per ogni istante.

Idea: la finestra scorre di 1, quindi ogni istante bersaglio T viene predetto
piu' volte, una per ciascun passo dell'orizzonte P che "cade" su T:

    finestra w, passo s  ->  istante  T = h + w + s      (s = 0..P-1)

Fissato T, le predizioni che lo riguardano vengono da P finestre diverse:
    (w=T-h, s=0)=t+1,  (w=T-h-1, s=1)=t+2,  ...,  (w=T-h-(P-1), s=P-1)=t+P
Ai bordi del trial ne cadono meno di P, ed e' corretto.

Per ogni istante si plotta:
  - il segnale reale
  - la banda tra il MIN e il MAX di tutte le predizioni di quell'istante
  - (linea sottile) la predizione a 1 passo (s=0) come riferimento

Con P=1 la banda collassa sulla linea (nessuna variazione): comportamento atteso.

Architettura a HIDDEN INCROCIATO:
  IM (inversa): GRU_IM(comandi) -> h_im ; MLP_IM([h_im, h_fm, ctx]) -> comando
  FM (diretta): GRU_FM(sensori) -> h_fm ; MLP_FM([h_fm, h_im, ctx]) -> sensori
ctx = [amp, freq, center].

Uso:
  python3 src/net/Estimator/checkpoints_joint/plot_prediction_band.py --checkpoint src/net/Estimator/checkpoints_joint/best.pt
  python3 src/net/Estimator/checkpoints_joint/plot_prediction_band.py --list_trials      # scegline uno che e' 'val'
  python3 src/net/Estimator/checkpoints_joint/plot_prediction_band.py --checkpoint src/net/Estimator/checkpoints_joint/best.pt --trial trial_XX.csv
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


def run_models_on_trial(dataset, IM, FM, device, trial_idx, batch_size=256):
    """Fa girare entrambe le reti sulle finestre (contigue e ordinate) del trial.
    Ritorna i target e le predizioni COMPLETE su tutto l'orizzonte P:
        tgt_sens (n,P,2), fm_pred (n,P,2), tgt_cmd (n,P,1), im_pred (n,P,1)
    L'aggregazione min-max per istante viene fatta a valle.

    Forward a hidden incrociato: ogni GRU codifica il proprio segnale, poi ogni
    MLP decodifica con l'hidden proprio + l'hidden dell'ALTRA rete + ctx."""
    mask = dataset.window_trial == trial_idx
    idxs = np.nonzero(mask)[0]
    idxs.sort()

    seq_cmd  = dataset.seq_cmd[idxs]
    seq_sens = dataset.seq_sens[idxs]
    ctx      = dataset.context[idxs]
    tgt_sens = dataset.tgt_sens[idxs]     # (n, P, 2)
    tgt_cmd  = dataset.tgt_cmd[idxs]      # (n, P, 1)

    P = tgt_cmd.shape[1]

    fm_pred, im_pred = [], []
    with torch.no_grad():
        for start in range(0, len(idxs), batch_size):
            sc = seq_cmd[start:start + batch_size].to(device)   # (b, H, 1) comandi
            ss = seq_sens[start:start + batch_size].to(device)  # (b, H, 2) sensori
            cx = ctx[start:start + batch_size].to(device)       # (b, 3)
            h_im = IM.encode(sc)                # GRU_IM sui comandi
            h_fm = FM.encode(ss)               # GRU_FM sui sensori
            p_cmd  = IM.decode(h_im, h_fm, cx)  # (b, P, 1)
            p_sens = FM.decode(h_fm, h_im, cx)  # (b, P, 2)
            im_pred.append(p_cmd.cpu())
            fm_pred.append(p_sens.cpu())

    fm_pred = torch.cat(fm_pred, dim=0)                 # (n, P, 2)
    im_pred = torch.cat(im_pred, dim=0)                 # (n, P, 1)
    return idxs, P, tgt_sens, fm_pred, tgt_cmd, im_pred


def _to_np(a):
    return a.numpy() if hasattr(a, "numpy") else np.asarray(a)


def aggregate_per_instant(pred_full, tgt_full):
    """Aggrega le predizioni PER ISTANTE BERSAGLIO (ancora in spazio scalato).

    pred_full, tgt_full: (n, P, C).
    La finestra w al passo s predice l'istante locale j = w + s.
    Costruisce array di lunghezza M = n + P - 1:
        true[j]   valore reale all'istante j
        center[j] predizione a 1 passo (s=0), definita per j = 0..n-1
        lo[j]/hi[j]  min/max di TUTTE le predizioni che riguardano j
    Gli istanti senza predizioni restano NaN (non capita, ma per sicurezza)."""
    pf = _to_np(pred_full).astype(np.float64)
    tf = _to_np(tgt_full).astype(np.float64)
    n, P, C = pf.shape
    M = n + P - 1

    lo   = np.full((M, C),  np.inf, dtype=np.float64)
    hi   = np.full((M, C), -np.inf, dtype=np.float64)
    true = np.full((M, C),  np.nan, dtype=np.float64)

    # scatter: il passo s manda le finestre 0..n-1 sugli istanti s..n-1+s
    for s in range(P):
        j = np.arange(n) + s
        np.minimum.at(lo, j, pf[:, s, :])
        np.maximum.at(hi, j, pf[:, s, :])
        true[j] = tf[:, s, :]                  # idempotente: stesso valore reale

    center = np.full((M, C), np.nan, dtype=np.float64)
    center[:n] = pf[:, 0, :]                    # predizione t+1

    empty = ~np.isfinite(lo).all(axis=1)        # istanti senza alcuna predizione
    lo[empty] = np.nan
    hi[empty] = np.nan
    return true, center, lo, hi


def channel_panel(ax, t, true_real, center_real, lo_real, hi_real, unit, title):
    ax.set_facecolor(COL_SURFACE)
    ax.grid(True, color=COL_GRID, linewidth=0.8, zorder=0)
    for spine in ("top", "right"):
        ax.spines[spine].set_visible(False)
    for spine in ("left", "bottom"):
        ax.spines[spine].set_color(COL_BASELINE)

    # banda min-max delle predizioni per ciascun istante
    ax.fill_between(t, lo_real, hi_real, color=COL_MODEL, alpha=0.20, linewidth=0,
                    zorder=2, label="banda min–max predizioni")
    # bordi della banda, sottili
    ax.plot(t, lo_real, color=COL_MODEL, linewidth=0.7, alpha=0.55, zorder=3)
    ax.plot(t, hi_real, color=COL_MODEL, linewidth=0.7, alpha=0.55, zorder=3)
    # predizione a 1 passo come riferimento (sottile)
    ax.plot(t, center_real, color=COL_MODEL, linewidth=1.1, alpha=0.9, zorder=4,
            label="predizione a 1 passo (t+1)")
    # segnale reale sopra a tutto
    ax.plot(t, true_real, color=COL_SIGNAL, linewidth=1.6, zorder=5, label="segnale reale")

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
    parser.add_argument("--p", type=int, default=None,
                        help="orizzonte P per costruire i target del dataset. Default: "
                             "il P salvato nel checkpoint. Passalo solo per forzare.")
    parser.add_argument("--out", default=os.path.join(SCRIPT_DIR, "predictions_joint_band.png"))
    args = parser.parse_args()

    device = torch.device(args.device)

    # leggo il P dal checkpoint PRIMA di costruire il dataset, cosi' i target
    # hanno lo stesso orizzonte del modello.
    P_ckpt = None
    try:
        _ck = torch.load(args.checkpoint, map_location="cpu", weights_only=False)
        P_ckpt = _ck.get("P", None)
    except Exception:
        pass
    P_ds = args.p if args.p is not None else (P_ckpt if P_ckpt is not None else None)
    if args.p is not None and P_ckpt is not None and args.p != P_ckpt:
        print(f"[avviso] --p={args.p} diverso dal P del checkpoint ({P_ckpt}): "
              f"uso --p={args.p}.", file=sys.stderr)

    ds_kwargs = {"scaler_path": args.scaler_path}
    if P_ds is not None:
        ds_kwargs["p"] = P_ds
    dataset = FishJointDataset(args.dataset_dir, **ds_kwargs)

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
    ctx_static = ckpt.get("ctx_static", int(dataset.context.shape[-1]))

    def dims_from_state(sd):
        gru_hidden = sd["gru.weight_hh_l0"].shape[1]   # (3*gru_hidden, gru_hidden)
        mlp_hidden = sd["mlp.0.weight"].shape[0]        # (mlp_hidden, gru+cross+ctx)
        return int(gru_hidden), int(mlp_hidden)

    gh_im, mh_im = dims_from_state(ckpt["im_state"])
    gh_fm, mh_fm = dims_from_state(ckpt["fm_state"])
    print(f"Dimensioni dal checkpoint: IM gru={gh_im} mlp={mh_im} | FM gru={gh_fm} mlp={mh_fm} "
          f"| ctx_static={ctx_static}")

    IM, FM = build_models(
        gru_hidden_im=gh_im, mlp_hidden_im=mh_im,
        gru_hidden_fm=gh_fm, mlp_hidden_fm=mh_fm,
        p=P, ctx_static=ctx_static)
    IM.load_state_dict(ckpt["im_state"]); IM.to(device).eval()
    FM.load_state_dict(ckpt["fm_state"]); FM.to(device).eval()

    idxs, P_run, tgt_sens, fm_pred, tgt_cmd, im_pred = run_models_on_trial(
        dataset, IM, FM, device, trial_idx)
    print(f"P={P_run} | banda min–max su tutti i passi dell'orizzonte, per istante")

    # aggregazione per istante (spazio scalato)
    true_s, cen_s, lo_s, hi_s = aggregate_per_instant(fm_pred, tgt_sens)   # (M,2)
    true_c, cen_c, lo_c, hi_c = aggregate_per_instant(im_pred, tgt_cmd)    # (M,1)
    M = true_s.shape[0]

    # asse tempi reale: gli istanti locali 0..M-1 stanno a t_full[h : h+M].
    # t_rel_sec non e' piu' tra le colonne del dataset ma resta nel CSV grezzo;
    # se assente, fallback a 20 Hz.
    df = pd.read_csv(Path(args.dataset_dir) / trial_name)
    if "t_rel_sec" in df.columns:
        t_full = df["t_rel_sec"].values.astype(np.float32)
        if len(t_full) >= dataset.h + M:
            t_axis = t_full[dataset.h: dataset.h + M].astype(np.float32)
        else:
            t_axis = np.arange(M, dtype=np.float32) / 20.0
    else:
        t_axis = np.arange(M, dtype=np.float32) / 20.0

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

    t_axis = t_axis[win]
    true_s, cen_s, lo_s, hi_s = true_s[win], cen_s[win], lo_s[win], hi_s[win]
    true_c, cen_c, lo_c, hi_c = true_c[win], cen_c[win], lo_c[win], hi_c[win]

    # 3 pannelli: sensor_diff, current (FM) + tail_target (IM)
    panels = [("sensor_diff", true_s, cen_s, lo_s, hi_s, 0),
              ("current",     true_s, cen_s, lo_s, hi_s, 1),
              ("tail_target", true_c, cen_c, lo_c, hi_c, 0)]

    fig, axes = plt.subplots(len(panels), 1, figsize=(11, 3.4 * len(panels)), sharex=True)
    fig.patch.set_facecolor(COL_SURFACE)

    for ax, (ch, true, cen, lo, hi, ci) in zip(axes, panels):
        scaler = dataset.scalers[CHANNEL_TO_SCALER_KEY[ch]]

        def inv(a):
            return scaler.inverse_transform(a[:, ci:ci + 1]).ravel()

        true_real   = inv(true)
        center_real = inv(cen)
        lo_real     = inv(lo)
        hi_real     = inv(hi)

        # RMSE a 1 passo dove la predizione centrale e' definita
        m = np.isfinite(center_real) & np.isfinite(true_real)
        rmse = float(np.sqrt(np.mean((center_real[m] - true_real[m]) ** 2))) if m.any() else float("nan")

        unit = CHANNEL_UNIT[ch]
        net = "IM" if ch == "tail_target" else "FM"
        channel_panel(ax, t_axis, true_real, center_real, lo_real, hi_real, unit,
                      f"{ch} [{net}] — RMSE(t+1) {rmse:.2f} {unit}")

    axes[-1].set_xlabel("tempo (s)", color=COL_TEXT_SEC, fontsize=9)
    win_txt = f"  [{t_axis[0]:.1f}–{t_axis[-1]:.1f}s]" if (args.t_start is not None or args.t_end is not None) else ""
    fig.suptitle(f"Predizioni reali IM + FM — banda min–max su P={P_run} passi — trial {trial_name}{win_txt}",
                 color=COL_TEXT, fontsize=13, fontweight="bold", x=0.01, ha="left", y=0.995)
    fig.tight_layout(rect=[0, 0, 1, 0.96])
    fig.savefig(args.out, dpi=160, facecolor=COL_SURFACE)
    print(f"Salvato {args.out} (trial: {trial_name}, {M} istanti, {len(idxs)} finestre)")


if __name__ == "__main__":
    main()