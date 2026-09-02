"""
Test in loop chiuso (closed-loop rollout) fra rete DIRETTA (SensorEstimator) e
rete INVERSA (InverseEstimator).

Idea
----
Le due reti vengono accoppiate in un anello autoregressivo:

    ... -> [DIRETTA] -> sensori(t+1) -> [INVERSA] -> comando(t+1) -> [DIRETTA] -> ...

Ad ogni passo si usa SOLO l'uscita "future" (head_future, il valore a t+1) di
ciascuna rete; nessun teacher forcing dopo il warmup. Si mantengono due buffer
scorrevoli di lunghezza H:

  * buf_cmd : storia comandi normalizzati con lo scaler DIRETTO  -> input diretta
  * buf_sen : storia [sensor_diff, current] norm. scaler INVERSO -> input inversa

Passaggio di dominio (il punto chiave del test)
-----------------------------------------------
L'uscita di una rete e' normalizzata con il PROPRIO scaler; per darla in pasto
all'altra rete va riportata in unita' reali e poi rinormalizzata con lo scaler
dell'altra rete:

  diretta -> [sd,vf] norm(diretto) --denorm(diretto)--> reale --norm(inverso)--> input inversa
  inversa -> [cmd]  norm(inverso)  --denorm(inverso)--> reale --norm(diretto)--> input diretta

Warmup
------
I primi H campioni VERI del trial (scelto dal validation split) inizializzano
entrambi i buffer. Da li' in poi il rollout e' completamente autoregressivo.

Plot
----
Tre pannelli in unita' fisiche, ciascuno rollout vs segnale vero dello stesso
trial: sensor_diff, current, comando (tail_target_rad).

Uso
---
  python3 closed_loop_test.py \
      --fwd_checkpoint     src/net/SensorEstimator/checkpoints/best.pt \
      --inv_checkpoint     src/net/InverseEstimator/checkpoints_inverse/best.pt \
      --fwd_scaler         src/net/scaler/scalers.pkl \
      --inv_scaler         src/net/scaler/scalers_inverse.pkl \
      --dataset_dir        src/net/dataset
  # opzionali: --trial <nome|indice>  --steps <N>  --list_trials  --out <png>
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
REPO_ROOT  = os.path.abspath(os.path.join(SCRIPT_DIR, "..", "..", "..", ".."))
sys.path.insert(0, os.path.join(REPO_ROOT, "src"))

from SensorEstimator.model    import FishSensorEstimator
from SensorEstimator.dataset  import FishDataset
from InverseEstimator.model_inverse   import FishInverseEstimator
from InverseEstimator.dataset_inverse import FishInverseDataset

# split: DEVE combaciare con train.py / train_inverse.py per un test onesto
VAL_FRAC   = 0.2
SPLIT_SEED = 42

# --- palette (coerente con plot_prediction.py) ---
COL_SURFACE  = "#fcfcfb"
COL_TEXT     = "#0b0b0b"
COL_TEXT_SEC = "#52514e"
COL_MUTED    = "#898781"
COL_GRID     = "#e1e0d9"
COL_BASELINE = "#c3c2b7"
COL_SIGNAL   = "#0b0b0b"   # segnale reale
COL_MODEL    = "#2a78d6"   # rollout closed-loop

CHANNEL_UNIT = {
    "sensor_diff": "unita' sensore",
    "current":     "mA",
    "cmd":         "rad",
}


# ----------------------------------------------------------------------------
# utilita' scaler: normalizza / denormalizza un singolo canale con StandardScaler
# ----------------------------------------------------------------------------
def denorm(scaler, x):
    """x_norm -> x_reale, scalare o array 1D."""
    x = np.asarray(x, dtype=np.float64).reshape(-1, 1)
    return scaler.inverse_transform(x).ravel()

def norm(scaler, x):
    """x_reale -> x_norm, scalare o array 1D."""
    x = np.asarray(x, dtype=np.float64).reshape(-1, 1)
    return scaler.transform(x).ravel()


def dims_from_state_dict(sd):
    gru_hidden = sd["gru.weight_hh_l0"].shape[1]
    mlp_hidden = sd["mlp.0.weight"].shape[0]
    return int(gru_hidden), int(mlp_hidden)


def load_model(model_cls, checkpoint_path, input_size_fallback, h, device):
    ckpt  = torch.load(checkpoint_path, map_location=device)
    state = ckpt["model_state"]
    input_size = ckpt.get("input_size", input_size_fallback)
    gru_hidden, mlp_hidden = dims_from_state_dict(state)
    model = model_cls(input_size=input_size, gru_hidden=gru_hidden,
                      mlp_hidden=mlp_hidden, h=h).to(device)
    model.load_state_dict(state)
    model.eval()
    return model, input_size


def val_trial_indices(dataset):
    """Indici (in dataset._episodes) dei trial finiti nel validation split."""
    _, val_ds = dataset.split_by_trial(val_frac=VAL_FRAC, seed=SPLIT_SEED)
    idxs = np.unique(dataset.window_trial[np.asarray(val_ds.indices)])
    return set(int(i) for i in idxs)


def resolve_trial(dataset, trial_arg, val_idxs):
    names = dataset.trial_names
    if trial_arg is None:
        if val_idxs:
            idx = int(sorted(val_idxs)[0])
            print(f"Nessun --trial: uso '{names[idx]}' (primo del validation split).")
        else:
            idx = 0
            print(f"Nessun --trial e nessun validation split: uso '{names[0]}' (fallback).")
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
        raise ValueError(f"Nessun trial per '{trial_arg}'. Usa --list_trials.")
    raise ValueError(f"'{trial_arg}' ambiguo: {[names[i] for i in matches]}.")


def real_signals_from_csv(dataset_dir, trial_name):
    """Rilegge i segnali VERI in unita' fisiche dal CSV del trial, con la stessa
    calibrazione di offset dei dataset (media primi 50 campioni su sensor_diff).
    Ritorna dict con array reali: sensor_diff, current, cmd, t_rel_sec."""
    df = pd.read_csv(Path(dataset_dir) / trial_name)

    # ricostruzione sensor_diff calibrato identica a dataset(_inverse).py
    # NB: nei dataset sensor_diff = sensors[:,0]-sensors[:,1], poi calibrato con
    # offset = media primi 50 campioni. Qui non ho accesso al parser interno
    # sensors[], quindi mi appoggio ai segnali gia' estratti dal dataset (vedi
    # build_real_from_dataset) per garantire coerenza. Questa funzione resta
    # come fallback se servisse il CSV grezzo.
    out = {"t_rel_sec": df["t_rel_sec"].values.astype(np.float32)
                        if "t_rel_sec" in df.columns else None}
    return out, df


def build_real_from_dataset(fwd_ds, trial_idx):
    """Estrae i segnali reali (fisici) del trial direttamente dagli episodi gia'
    parsati dal FishDataset diretto: garantisce identica calibrazione/offset a
    quella vista dalle reti. Ritorna array reali sensor_diff, current, cmd."""
    ep = fwd_ds._episodes[trial_idx]
    return {
        "sensor_diff": np.asarray(ep["sensor_diff_cal"], dtype=np.float64),
        "current":     np.asarray(ep["current"],         dtype=np.float64),
        "cmd":         np.asarray(ep["cmd_servo"],        dtype=np.float64),
    }


# ----------------------------------------------------------------------------
# ROLLOUT closed-loop
# ----------------------------------------------------------------------------
def closed_loop_rollout(fwd_model, inv_model, fwd_scalers, inv_scalers,
                        real, h, n_steps, device):
    """
    Rollout closed-loop: UN tick = UN avanzamento temporale di uno.

    real: dict con array reali 'sensor_diff','current','cmd' (interi del trial).

    Semantica fisica:
      Entrambe le reti sono addestrate un-passo-avanti con la STESSA convenzione:
      finestra che termina a t -> uscita a t+1.
        - DIRETTA: comandi fino a t            -> sensori [sd,vf] a t+1
        - INVERSA: sensori  fino a t           -> comando a t+1
      I valori appena predetti (istante t+1) entrano nei buffer solo A FINE tick,
      per il tick successivo. Cosi' un tick = un solo campione fisico.

    Nota sul bug corretto:
      La versione precedente costruiva la finestra dell'inversa terminandola a
      t+1 (buf_sd[1:] + [sd_next]). Ma l'inversa, addestrata come "sensori fino a
      t -> comando a t+1", con una finestra che termina a t+1 produce il comando
      a t+2: uno shift di un campione che, chiuso nell'anello, raddoppiava la
      frequenza del segnale ricostruito. Qui l'inversa riceve buf_sd/buf_vf
      (finestra che termina a t), esattamente come in open-loop.

    Allineamento:
      buf termina all'indice h-1 (t = h-1), quindi il primo istante predetto
      e' t+1 = h. start = h; la curva vera si allinea con slice(start, start+n).

    Ritorna dict di array reali predetti + 'start' e 'n_steps'.
    """
    sd_r  = real["sensor_diff"]
    vf_r  = real["current"]
    cmd_r = real["cmd"]

    sc_cmd_fwd = fwd_scalers["cmd"]
    sc_sd_fwd  = fwd_scalers["sd"]
    sc_vf_fwd  = fwd_scalers["vf"]
    sc_sd_inv  = inv_scalers["sd"]
    sc_vf_inv  = inv_scalers["vf"]
    sc_cmd_inv = inv_scalers["cmd"]

    if len(cmd_r) < h + 1:
        raise ValueError(f"Trial troppo corto per h={h}: {len(cmd_r)} campioni "
                         f"(servono almeno {h+1}).")

    # --- warmup: buffer allineati (nessuno shift artificiale) ---
    # buffer comandi e sensori terminano entrambi all'indice h-1 (istante t=h-1).
    buf_cmd = list(cmd_r[0:h].astype(np.float64))     # per la diretta
    buf_sd  = list(sd_r[0:h].astype(np.float64))      # per l'inversa (canale 0)
    buf_vf  = list(vf_r[0:h].astype(np.float64))      # per l'inversa (canale 1)

    pred_sd, pred_vf, pred_cmd = [], [], []

    # primo istante predetto = t+1 con t=h-1  ->  indice h
    start = h
    max_steps = len(cmd_r) - start
    n_steps = min(n_steps, max_steps)

    with torch.no_grad():
        for _ in range(n_steps):
            # ---- DIRETTA: comandi fino a t -> sensori a t+1 ----
            cmd_norm = norm(sc_cmd_fwd, buf_cmd)                       # (h,)
            seq_fwd  = torch.tensor(cmd_norm, dtype=torch.float32,
                                    device=device).reshape(1, h, 1)
            _, fut_fwd, _ = fwd_model(seq_fwd)                        # (1,2) norm diretto
            fut_fwd = fut_fwd.cpu().numpy().ravel()
            sd_next = float(denorm(sc_sd_fwd, fut_fwd[0])[0])         # -> reale, istante t+1
            vf_next = float(denorm(sc_vf_fwd, fut_fwd[1])[0])

            pred_sd.append(sd_next)
            pred_vf.append(vf_next)

            # ---- INVERSA: sensori fino a t -> comando a t+1 ----
            # stessa convenzione della diretta: finestra che TERMINA a t.
            # i sensori appena predatti (t+1) NON entrano qui: entrano nei buffer
            # solo a fine tick, per il tick successivo.
            sd_in  = norm(sc_sd_inv, buf_sd)
            vf_in  = norm(sc_vf_inv, buf_vf)
            seq_inv = torch.tensor(np.stack([sd_in, vf_in], axis=1),
                                   dtype=torch.float32,
                                   device=device).reshape(1, h, 2)
            _, fut_inv, _ = inv_model(seq_inv)                       # (1,1) norm inverso
            cmd_next = float(denorm(sc_cmd_inv,
                                    float(fut_inv.cpu().numpy().ravel()[0]))[0])  # reale, t+1

            pred_cmd.append(cmd_next)

            # ---- avanzamento di UN SOLO passo: tutti i buffer scorrono di 1 ----
            # i valori appena predetti (istante t+1) diventano l'ultimo campione
            # dei rispettivi buffer per il tick successivo.
            buf_cmd.append(cmd_next); buf_cmd.pop(0)
            buf_sd.append(sd_next);   buf_sd.pop(0)
            buf_vf.append(vf_next);   buf_vf.pop(0)

    return {
        "sensor_diff": np.asarray(pred_sd),
        "current":     np.asarray(pred_vf),
        "cmd":         np.asarray(pred_cmd),
        "start":       start,
        "n_steps":     n_steps,
    }


# ----------------------------------------------------------------------------
def panel(ax, t, true_real, pred_real, unit, title):
    ax.set_facecolor(COL_SURFACE)
    ax.grid(True, color=COL_GRID, linewidth=0.8, zorder=0)
    for spine in ("top", "right"):
        ax.spines[spine].set_visible(False)
    for spine in ("left", "bottom"):
        ax.spines[spine].set_color(COL_BASELINE)

    ax.plot(t, true_real, color=COL_SIGNAL, linewidth=1.6, zorder=4, label="segnale reale")
    ax.plot(t, pred_real, color=COL_MODEL,  linewidth=1.6, zorder=3,
            label="rollout closed-loop")

    ax.set_title(title, color=COL_TEXT, fontsize=12, fontweight="bold", loc="left", pad=10)
    ax.set_ylabel(unit, color=COL_TEXT_SEC, fontsize=9)
    ax.tick_params(colors=COL_MUTED, labelsize=8)
    ax.legend(loc="upper right", frameon=False, fontsize=8, labelcolor=COL_TEXT_SEC)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--fwd_checkpoint", default=os.path.join(
        REPO_ROOT, "src", "net", "SensorEstimator", "checkpoints", "best.pt"))
    ap.add_argument("--inv_checkpoint", default=os.path.join(
        REPO_ROOT, "src", "net", "InverseEstimator", "checkpoints_inverse", "best.pt"))
    ap.add_argument("--fwd_scaler", default=os.path.join(
        REPO_ROOT, "src", "net", "scaler", "scalers.pkl"))
    ap.add_argument("--inv_scaler", default=os.path.join(
        REPO_ROOT, "src", "net", "scaler", "scalers_inverse.pkl"))
    ap.add_argument("--dataset_dir", default=os.path.join(
        REPO_ROOT, "src", "net", "dataset"))
    ap.add_argument("--device", default="cuda" if torch.cuda.is_available() else "cpu")
    ap.add_argument("--trial", default=None,
                    help="nome/sottostringa/indice del trial. Default: primo del val split.")
    ap.add_argument("--steps", type=int, default=200,
                    help="numero di passi del rollout. Default: tutto il trial.")
    ap.add_argument("--list_trials", action="store_true")
    ap.add_argument("--out", default=os.path.join(SCRIPT_DIR, "closed_loop.png"))
    args = ap.parse_args()

    device = torch.device(args.device)

    # --- dataset diretto: da qui leggo i segnali reali del trial e il val split ---
    fwd_ds = FishDataset(args.dataset_dir, scaler_path=args.fwd_scaler)
    try:
        val_idxs = val_trial_indices(fwd_ds)   # costruisce anche finestre/scaler
    except Exception as e:
        print(f"[avviso] impossibile costruire il val split ({e}); "
              f"proseguo senza etichette train/val.", file=sys.stderr)
        val_idxs = set()

    if args.list_trials:
        for i, n in enumerate(fwd_ds.trial_names):
            tag = ("val" if i in val_idxs else "train") if val_idxs else "?"
            print(f"{i:3d}  [{tag:5s}]  {n}")
        return

    trial_idx  = resolve_trial(fwd_ds, args.trial, val_idxs)
    trial_name = fwd_ds.trial_names[trial_idx]
    if val_idxs and trial_idx not in val_idxs:
        print(f"[avviso] '{trial_name}' NON e' nel validation split: "
              f"il test non e' out-of-sample.", file=sys.stderr)

    h = fwd_ds.h

    # --- scaler: due file DISTINTI (diretto e inverso) ---
    fwd_scalers = FishDataset.load_scalers(args.fwd_scaler)
    inv_scalers = FishInverseDataset.load_scalers(args.inv_scaler)
    for k in ("sd", "vf", "cmd"):
        if k not in fwd_scalers:
            raise ValueError(f"scaler diretto: manca la chiave '{k}' in {args.fwd_scaler}")
        if k not in inv_scalers:
            raise ValueError(f"scaler inverso: manca la chiave '{k}' in {args.inv_scaler}")

    # --- modelli ---
    fwd_model, _ = load_model(FishSensorEstimator, args.fwd_checkpoint,
                              input_size_fallback=1, h=h, device=device)
    inv_model, _ = load_model(FishInverseEstimator, args.inv_checkpoint,
                              input_size_fallback=2, h=h, device=device)

    # --- segnali reali del trial (calibrazione identica a quella vista dalle reti) ---
    real = build_real_from_dataset(fwd_ds, trial_idx)

    # start del rollout = h (vedi closed_loop_rollout); passi disponibili fino
    # a fine trial = len - h.
    n_avail = len(real["cmd"]) - h
    n_steps = n_avail if args.steps is None else min(args.steps, n_avail)
    if n_steps <= 0:
        raise ValueError(f"Trial troppo corto per h={h}: {len(real['cmd'])} campioni "
                         f"(servono almeno {h+1}).")

    print(f"Trial: {trial_name} | campioni={len(real['cmd'])} | h={h} | "
          f"passi rollout={n_steps}  (~{n_steps/20.0:.1f}s @20Hz)")

    # --- rollout ---
    pred = closed_loop_rollout(fwd_model, inv_model, fwd_scalers, inv_scalers,
                               real, h, n_steps, device)
    start, n = pred["start"], pred["n_steps"]

    # --- serie reali allineate: la predizione al passo k stima il campione start+k ---
    # (start = h+1: primo indice predetto dalla diretta con finestra comandi [0:h])
    sl = slice(start, start + n)
    true = {
        "sensor_diff": real["sensor_diff"][sl],
        "current":     real["current"][sl],
        "cmd":         real["cmd"][sl],
    }

    # asse temporale reale se disponibile
    _, df = real_signals_from_csv(args.dataset_dir, trial_name)
    if "t_rel_sec" in df.columns:
        t_full = df["t_rel_sec"].values.astype(np.float64)
        t_axis = t_full[sl] if len(t_full) >= start + n else np.arange(n) / 20.0
        if len(t_axis) != n:
            t_axis = np.arange(n) / 20.0
    else:
        t_axis = np.arange(n) / 20.0

    # --- RMSE closed-loop per canale, in unita' fisiche ---
    def rmse(a, b): return float(np.sqrt(np.mean((np.asarray(a) - np.asarray(b)) ** 2)))
    rmse_sd  = rmse(pred["sensor_diff"], true["sensor_diff"])
    rmse_vf  = rmse(pred["current"],     true["current"])
    rmse_cmd = rmse(pred["cmd"],         true["cmd"])
    print(f"RMSE closed-loop  | sensor_diff {rmse_sd:.3f} {CHANNEL_UNIT['sensor_diff']} "
          f"| current {rmse_vf:.3f} {CHANNEL_UNIT['current']} "
          f"| cmd {rmse_cmd:.4f} {CHANNEL_UNIT['cmd']}")

    # --- plot: 3 pannelli ---
    fig, axes = plt.subplots(3, 1, figsize=(11, 3.4 * 3), sharex=True)
    fig.patch.set_facecolor(COL_SURFACE)

    panel(axes[0], t_axis, true["sensor_diff"], pred["sensor_diff"],
          CHANNEL_UNIT["sensor_diff"],
          f"sensor_diff — RMSE closed-loop {rmse_sd:.2f} {CHANNEL_UNIT['sensor_diff']}")
    panel(axes[1], t_axis, true["current"], pred["current"],
          CHANNEL_UNIT["current"],
          f"current — RMSE closed-loop {rmse_vf:.2f} {CHANNEL_UNIT['current']}")
    panel(axes[2], t_axis, true["cmd"], pred["cmd"],
          CHANNEL_UNIT["cmd"],
          f"comando (tail_target_rad) — RMSE closed-loop {rmse_cmd:.4f} {CHANNEL_UNIT['cmd']}")

    axes[-1].set_xlabel("tempo (s)", color=COL_TEXT_SEC, fontsize=9)
    fig.suptitle(f"Test closed-loop diretta↔inversa — trial {trial_name} "
                 f"({n} passi, ~{n/20.0:.1f}s)",
                 color=COL_TEXT, fontsize=13, fontweight="bold", x=0.01, ha="left", y=0.997)
    fig.tight_layout(rect=[0, 0, 1, 0.96])
    fig.savefig(args.out, dpi=160, facecolor=COL_SURFACE)
    print(f"Salvato {args.out}")


if __name__ == "__main__":
    main()