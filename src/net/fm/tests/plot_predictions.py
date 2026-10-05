"""
Disegna, per un trial reale, le predizioni del FORWARD MODEL confrontate col
segnale vero.

Per ogni finestra i (i = H .. n-1) il modello predice i sensori al tempo t=i a
partire da:
  - storia vera di comandi e sensori  t-H .. t-1
  - comando vero a t
Facendo scorrere i lungo tutto il trial si ottiene una serie temporale continua
di predizioni a 1 passo (teacher forced: la storia in input e' sempre quella
vera, non autoregressiva).

Nel titolo di ogni figura c'e' anche l'RMSE della baseline di PERSISTENZA
(sens(t) = sens(t-1)): il modello e' utile solo se fa chiaramente meglio.

Uso:
  python3 src/net/Estimator/plot_prediction_fm.py --checkpoint src/net/Estimator/checkpoints_fm/best_fm_base.pt
  python3 src/net/Estimator/plot_prediction_fm.py --list_trials      # scegline uno che e' 'val'
  python3 src/net/Estimator/plot_prediction_fm.py --checkpoint ... --trial trial_XX.csv --t_start 10 --t_end 30
"""
import argparse
import os
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import torch

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT  = os.path.abspath(os.path.join(SCRIPT_DIR, "..", "..", ".."))
sys.path.insert(0, os.path.join(REPO_ROOT, "src"))

# flat import (come train_fm). In repo: from net.Estimator.model_fm import ...
from model_fm import build_model
from dataset  import FishJointDataset

# canali di uscita del forward model (sensori)
FM_CHANNELS = ["sensor_diff", "current"]
CHANNEL_TO_SCALER_KEY = {"sensor_diff": "sd", "current": "vf"}
CHANNEL_UNIT = {"sensor_diff": "sensor units", "current": "mA"}

# frequenza di fallback per l'asse temporale se il CSV non ha t_rel_sec
FALLBACK_HZ = 10.0

# --- palette (ordine categorico fisso) ---
COL_SURFACE  = "#fcfcfb"
COL_TEXT     = "#0b0b0b"
COL_TEXT_SEC = "#52514e"
COL_MUTED    = "#898781"
COL_GRID     = "#e1e0d9"
COL_BASELINE = "#c3c2b7"
COL_SIGNAL   = "#0b0b0b"
COL_MODEL    = "#2a78d6"
COL_ERROR    = "#c1666b"   # residuo (predetto - reale), asse destro

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


def load_model(checkpoint, device):
    """Ricostruisce il forward model con gli iperparametri salvati nel checkpoint."""
    ckpt = torch.load(checkpoint, map_location=device, weights_only=False)
    # I pesi di un modello allenato con uscita residua si caricherebbero senza
    # errori ma darebbero predizioni sbagliate (manca la somma con sens(t-1)).
    if ckpt.get("residual", True):
        raise ValueError(
            f"{checkpoint} e' stato allenato con uscita residua, non piu' "
            f"supportata: riallena con train_fm.py.")
    FM = build_model(gru_hidden=ckpt["gru_hidden"], mlp_hidden=ckpt["mlp_hidden"],
                     num_layers=ckpt.get("num_layers", 1))
    FM.load_state_dict(ckpt["fm_state"])
    FM.to(device).eval()
    print(f"Modello dal checkpoint: GRU {FM.gru_hidden}x{FM.num_layers} | "
          f"MLP {FM.mlp_hidden} | H={ckpt.get('H')}")
    return FM, ckpt


def run_model_on_trial(dataset, FM, device, trial_idx, batch_size=256):
    """Fa girare il forward model sulle finestre (contigue e ordinate) del trial.
    Ritorna: idxs, target sens(t), predizione sens(t), persistenza sens(t-1)."""
    idxs = np.sort(np.nonzero(dataset.window_trial == trial_idx)[0])

    seq_cmd  = dataset.seq_cmd[idxs]
    seq_sens = dataset.seq_sens[idxs]
    cmd_t    = dataset.tgt_cmd[idxs][:, 0, :]     # (n, 1) comando a t
    tgt      = dataset.tgt_sens[idxs][:, 0, :]    # (n, 2) sensori a t

    preds = []
    with torch.no_grad():
        for start in range(0, len(idxs), batch_size):
            sl = slice(start, start + batch_size)
            p = FM(seq_cmd[sl].to(device), seq_sens[sl].to(device),
                   cmd_t[sl].to(device))
            preds.append(p.cpu())
    pred = torch.cat(preds, dim=0)
    persist = seq_sens[:, -1, :].cpu()            # baseline sens(t-1)
    return idxs, tgt.cpu(), pred, persist


def real_time_axis(dataset_dir, trial_name, h, n_windows):
    """Istante t di ogni target: la finestra k-esima (i = h + k) predice t = i."""
    df = pd.read_csv(Path(dataset_dir) / trial_name)
    if "t_rel_sec" in df.columns:
        t_full = df["t_rel_sec"].values.astype(np.float32)
        t = t_full[h: h + n_windows]
        if len(t) == n_windows:
            return t
    return (h + np.arange(n_windows, dtype=np.float32)) / FALLBACK_HZ


def channel_panel(ax, t, true_real, pred_real, unit, title):
    ax.set_facecolor(COL_SURFACE)
    ax.grid(True, color=COL_GRID, linewidth=0.8, zorder=0)
    ax.spines["top"].set_visible(False)
    for spine in ("left", "bottom"):
        ax.spines[spine].set_color(COL_BASELINE)

    # --- asse destro: errore (predetto - reale) ---
    axr = ax.twinx()
    axr.set_facecolor(COL_SURFACE)
    diff = np.asarray(pred_real) - np.asarray(true_real)
    axr.axhline(0.0, color=COL_ERROR, linewidth=0.8, alpha=0.5, zorder=1)
    axr.fill_between(t, 0.0, diff, color=COL_ERROR, alpha=0.15, linewidth=0, zorder=1)
    l_err, = axr.plot(t, diff, color=COL_ERROR, linewidth=1.0, alpha=0.9, zorder=2)
    amax = float(np.nanmax(np.abs(diff))) if diff.size else 1.0
    amax = amax if amax > 0 else 1.0
    axr.set_ylim(-amax * 1.05, amax * 1.05)
    axr.spines["top"].set_visible(False)
    axr.spines["left"].set_visible(False)
    axr.spines["right"].set_color(COL_ERROR)
    axr.tick_params(axis="y", colors=COL_ERROR, labelsize=8)
    axr.set_ylabel(f"error [{unit}]", color=COL_ERROR, fontsize=9)

    # --- asse sinistro: segnale e predizione (sopra all'errore) ---
    l_pred, = ax.plot(t, pred_real, color=COL_MODEL,  linewidth=1.6, zorder=3)
    l_true, = ax.plot(t, true_real, color=COL_SIGNAL, linewidth=1.6, zorder=4)
    ax.set_zorder(axr.get_zorder() + 1)
    ax.patch.set_visible(False)

    ax.set_title(title, color=COL_TEXT, fontsize=12, fontweight="bold", loc="left", pad=10)
    ax.set_ylabel(unit, color=COL_TEXT_SEC, fontsize=9)
    ax.tick_params(colors=COL_MUTED, labelsize=8)
    ax.legend([l_true, l_pred, l_err],
              ["real signal", "model prediction", "error (pred − real)"],
              loc="upper right", frameon=False, fontsize=8, labelcolor=COL_TEXT_SEC)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--checkpoint", default=os.path.join(SCRIPT_DIR, "checkpoints_fm", "best.pt"))
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
    parser.add_argument("--out", default=os.path.join(SCRIPT_DIR, "predictions_fm"),
                        help="CARTELLA di output: vi salva sensor_diff.png e current.png")
    args = parser.parse_args()

    device = torch.device(args.device)

    # H dal checkpoint (se leggibile), cosi' le finestre combaciano col modello
    ds_kwargs = {"scaler_path": args.scaler_path, "p": 1}
    try:
        _ck = torch.load(args.checkpoint, map_location="cpu", weights_only=False)
        if _ck.get("H") is not None:
            ds_kwargs["h"] = int(_ck["H"])
    except Exception:
        pass
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

    trial_idx  = resolve_trial(dataset, args.trial, val_trial_idxs)
    trial_name = dataset.trial_names[trial_idx]
    split_txt  = ("val" if trial_idx in val_trial_idxs else "train") if val_trial_idxs else "?"

    FM, _ = load_model(args.checkpoint, device)

    idxs, tgt, pred, persist = run_model_on_trial(dataset, FM, device, trial_idx)
    t_axis = real_time_axis(args.dataset_dir, trial_name, dataset.h, len(idxs))

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
    win_t   = torch.from_numpy(win)
    t_axis  = t_axis[win]
    tgt     = tgt[win_t]
    pred    = pred[win_t]
    persist = persist[win_t]

    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)
    win_txt = (f"  [{t_axis[0]:.1f}–{t_axis[-1]:.1f}s]"
               if (args.t_start is not None or args.t_end is not None) else "")

    for ci, ch in enumerate(FM_CHANNELS):
        scaler = dataset.scalers[CHANNEL_TO_SCALER_KEY[ch]]
        inv = lambda x: scaler.inverse_transform(x[:, ci:ci + 1].numpy()).ravel()
        true_real, pred_real, pers_real = inv(tgt), inv(pred), inv(persist)
        rmse      = float(np.sqrt(np.mean((pred_real - true_real) ** 2)))
        rmse_pers = float(np.sqrt(np.mean((pers_real - true_real) ** 2)))
        unit = CHANNEL_UNIT[ch]

        fig, ax = plt.subplots(1, 1, figsize=(11, 3.8))
        fig.patch.set_facecolor(COL_SURFACE)
        channel_panel(ax, t_axis, true_real, pred_real, unit,
                      f"{ch} — RMSE {rmse:.3g} {unit}  (persistence {rmse_pers:.3g})")
        ax.set_xlabel("time (s)", color=COL_TEXT_SEC, fontsize=9)
        fig.suptitle(f"Forward model, 1-step predictions — trial {trial_name} [{split_txt}]{win_txt}",
                     color=COL_TEXT, fontsize=12, fontweight="bold", x=0.01, ha="left", y=0.99)
        fig.tight_layout(rect=[0, 0, 1, 0.93])
        path = out_dir / f"{ch}.png"
        fig.savefig(path, dpi=160, facecolor=COL_SURFACE)
        plt.close(fig)
        print(f"Salvato {path}  (RMSE {rmse:.4g} | persistenza {rmse_pers:.4g} {unit})")

    print(f"Fatto: {len(FM_CHANNELS)} figure in {out_dir}/ (trial: {trial_name}, {len(t_axis)} punti)")


if __name__ == "__main__":
    main()